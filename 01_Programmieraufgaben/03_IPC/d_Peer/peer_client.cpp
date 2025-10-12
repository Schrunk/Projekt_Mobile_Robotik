#include "peer_shared_memory.h"
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cstring>
#include <ctime>
#include <signal.h>

class PeerClient {
private:
    int peer_id;
    std::string peer_name;
    int shm_id;
    int sem_id;
    PeerSharedData* shared_data;
    volatile bool running;

public:
    PeerClient(int id, const std::string& name) 
        : peer_id(id), peer_name(name), shm_id(-1), sem_id(-1), shared_data(nullptr), running(true) {
    }

    ~PeerClient() {
        disconnect();
    }

    bool connect() {
        std::cout << "=== Peer List Client [" << peer_name << "] ===" << std::endl;
        
        // Try to attach to existing shared memory first
        shm_id = attach_peer_shared_memory();
        
        // If no shared memory exists, create it (first peer)
        if (shm_id == -1) {
            std::cout << "🔧 No shared memory found, creating new one..." << std::endl;
            shm_id = create_peer_shared_memory();
            if (shm_id == -1) {
                std::cerr << "❌ Failed to create shared memory" << std::endl;
                return false;
            }
        }

        // Attach to shared memory
        shared_data = static_cast<PeerSharedData*>(shmat(shm_id, nullptr, 0));
        if (shared_data == reinterpret_cast<void*>(-1)) {
            perror("shmat failed");
            return false;
        }

        // Create or attach to semaphore
        sem_id = semget(PEER_SEM_KEY, 1, 0666);
        if (sem_id == -1) {
            sem_id = create_peer_semaphore();
            if (sem_id == -1) {
                shmdt(shared_data);
                return false;
            }
        }

        // Register this peer
        registerPeer();

        std::cout << "✅ Connected to peer shared memory system" << std::endl;
        std::cout << "🆔 Peer ID: " << peer_id << ", Name: " << peer_name << std::endl;
        
        return true;
    }

    void disconnect() {
        if (shared_data != nullptr) {
            unregisterPeer();
            shmdt(shared_data);
            shared_data = nullptr;
        }
        std::cout << "🔌 Disconnected from peer system" << std::endl;
    }

    void registerPeer() {
        if (lock_peer_semaphore(sem_id) == 0) {
            // Initialize shared data if this is the first peer
            bool is_first_peer = (shared_data->active_peers == 0);
            if (is_first_peer) {
                memset(shared_data, 0, sizeof(PeerSharedData));
                shared_data->next_entry_id = 1;
                std::cout << "🌟 First peer - initialized shared memory" << std::endl;
            }

            // Find or create peer slot
            int peer_slot = -1;
            for (int i = 0; i < MAX_PEER_CLIENTS; i++) {
                if (!shared_data->peers[i].is_active) {
                    peer_slot = i;
                    break;
                }
            }

            if (peer_slot != -1) {
                PeerInfo& peer = shared_data->peers[peer_slot];
                peer.peer_id = peer_id;
                strncpy(peer.name, peer_name.c_str(), sizeof(peer.name) - 1);
                peer.name[sizeof(peer.name) - 1] = '\0';
                peer.last_seen = time(nullptr);
                peer.is_active = true;
                peer.entries_created = 0;
                
                shared_data->active_peers++;
                shared_data->last_activity = time(nullptr);
                
                snprintf(shared_data->last_action, sizeof(shared_data->last_action), 
                        "Peer %s joined the system", peer_name.c_str());
            }

            unlock_peer_semaphore(sem_id);
        }
    }

    void unregisterPeer() {
        if (lock_peer_semaphore(sem_id) == 0) {
            for (int i = 0; i < MAX_PEER_CLIENTS; i++) {
                if (shared_data->peers[i].is_active && 
                    shared_data->peers[i].peer_id == peer_id) {
                    shared_data->peers[i].is_active = false;
                    shared_data->active_peers--;
                    
                    snprintf(shared_data->last_action, sizeof(shared_data->last_action), 
                            "Peer %s left the system", peer_name.c_str());
                    break;
                }
            }
            unlock_peer_semaphore(sem_id);
        }
    }

    void updateLastSeen() {
        if (lock_peer_semaphore(sem_id) == 0) {
            for (int i = 0; i < MAX_PEER_CLIENTS; i++) {
                if (shared_data->peers[i].is_active && 
                    shared_data->peers[i].peer_id == peer_id) {
                    shared_data->peers[i].last_seen = time(nullptr);
                    break;
                }
            }
            unlock_peer_semaphore(sem_id);
        }
    }

    std::string executeCommand(PeerCommand cmd, const std::string& argument = "", int entry_id = -1) {
        if (lock_peer_semaphore(sem_id) != 0) {
            return "❌ Failed to acquire lock";
        }

        std::string result;
        
        switch (cmd) {
            case PEER_CMD_ADD:
                result = cmdAdd(argument);
                break;
            case PEER_CMD_LIST:
                result = cmdList();
                break;
            case PEER_CMD_GET:
                result = cmdGet(entry_id);
                break;
            case PEER_CMD_REMOVE:
                result = cmdRemove(entry_id);
                break;
            case PEER_CMD_CLEAR:
                result = cmdClear();
                break;
            case PEER_CMD_CLEAR_ALL:
                result = cmdClearAll();
                break;
            case PEER_CMD_SIZE:
                result = cmdSize();
                break;
            case PEER_CMD_FIND:
                result = cmdFind(argument);
                break;
            case PEER_CMD_PEERS:
                result = cmdPeers();
                break;
            case PEER_CMD_HELP:
                result = cmdHelp();
                break;
            default:
                result = "❌ Unknown command. Type 'help' for available commands.";
                break;
        }

        shared_data->last_activity = time(nullptr);
        updateLastSeen();
        unlock_peer_semaphore(sem_id);
        
        return result;
    }

    std::string cmdAdd(const std::string& content) {
        if (content.empty()) {
            return "❌ Usage: add <text>";
        }

        if (shared_data->total_entries >= MAX_PEER_LIST_ENTRIES) {
            return "❌ List is full. Cannot add more entries.";
        }

        // Find next available slot
        int slot = -1;
        for (int i = 0; i < MAX_PEER_LIST_ENTRIES; i++) {
            if (!shared_data->entries[i].is_active) {
                slot = i;
                break;
            }
        }

        if (slot == -1) {
            return "❌ No available slots in the list.";
        }

        PeerListEntry& entry = shared_data->entries[slot];
        entry.entry_id = shared_data->next_entry_id++;
        entry.peer_id = peer_id;
        strncpy(entry.author, peer_name.c_str(), sizeof(entry.author) - 1);
        entry.author[sizeof(entry.author) - 1] = '\0';
        strncpy(entry.content, content.c_str(), sizeof(entry.content) - 1);
        entry.content[sizeof(entry.content) - 1] = '\0';
        entry.timestamp = time(nullptr);
        entry.is_active = true;

        shared_data->total_entries++;
        shared_data->active_entries++;

        // Update peer statistics
        for (int i = 0; i < MAX_PEER_CLIENTS; i++) {
            if (shared_data->peers[i].is_active && 
                shared_data->peers[i].peer_id == peer_id) {
                shared_data->peers[i].entries_created++;
                break;
            }
        }

        snprintf(shared_data->last_action, sizeof(shared_data->last_action), 
                "Peer %s added entry #%d", peer_name.c_str(), entry.entry_id);

        return "✅ Added entry #" + std::to_string(entry.entry_id) + ": \"" + content + "\"";
    }

    std::string cmdList() {
        if (shared_data->active_entries == 0) {
            return "📝 List is empty.";
        }

        std::stringstream ss;
        ss << "📝 List entries (" << shared_data->active_entries << " active, " 
           << shared_data->total_entries << " total):\n";

        int count = 0;
        for (int i = 0; i < MAX_PEER_LIST_ENTRIES; i++) {
            if (shared_data->entries[i].is_active) {
                const PeerListEntry& entry = shared_data->entries[i];
                ss << "[" << entry.entry_id << "] \"" << entry.content 
                   << "\" (by " << entry.author << ")\n";
                count++;
            }
        }

        return ss.str();
    }

    std::string cmdGet(int entry_id) {
        if (entry_id <= 0) {
            return "❌ Usage: get <entry_id>";
        }

        for (int i = 0; i < MAX_PEER_LIST_ENTRIES; i++) {
            if (shared_data->entries[i].is_active && 
                shared_data->entries[i].entry_id == entry_id) {
                
                const PeerListEntry& entry = shared_data->entries[i];
                std::stringstream ss;
                ss << "📄 Entry #" << entry.entry_id << ":\n";
                ss << "   Content: \"" << entry.content << "\"\n";
                ss << "   Author: " << entry.author << " (Peer " << entry.peer_id << ")\n";
                ss << "   Created: " << std::ctime(&entry.timestamp);
                
                return ss.str();
            }
        }

        return "❌ Entry #" + std::to_string(entry_id) + " not found.";
    }

    std::string cmdRemove(int entry_id) {
        if (entry_id <= 0) {
            return "❌ Usage: remove <entry_id>";
        }

        for (int i = 0; i < MAX_PEER_LIST_ENTRIES; i++) {
            if (shared_data->entries[i].is_active && 
                shared_data->entries[i].entry_id == entry_id) {
                
                PeerListEntry& entry = shared_data->entries[i];
                
                // Check if this peer owns the entry
                if (entry.peer_id != peer_id) {
                    return "❌ You can only remove your own entries. Entry #" + 
                           std::to_string(entry_id) + " belongs to " + entry.author;
                }

                std::string content = entry.content;
                entry.is_active = false;
                shared_data->active_entries--;

                snprintf(shared_data->last_action, sizeof(shared_data->last_action), 
                        "Peer %s removed entry #%d", peer_name.c_str(), entry_id);

                return "✅ Removed entry #" + std::to_string(entry_id) + ": \"" + content + "\"";
            }
        }

        return "❌ Entry #" + std::to_string(entry_id) + " not found.";
    }

    std::string cmdClear() {
        int removed_count = 0;
        
        for (int i = 0; i < MAX_PEER_LIST_ENTRIES; i++) {
            if (shared_data->entries[i].is_active && 
                shared_data->entries[i].peer_id == peer_id) {
                shared_data->entries[i].is_active = false;
                shared_data->active_entries--;
                removed_count++;
            }
        }

        if (removed_count > 0) {
            snprintf(shared_data->last_action, sizeof(shared_data->last_action), 
                    "Peer %s cleared %d own entries", peer_name.c_str(), removed_count);
        }

        return "✅ Cleared " + std::to_string(removed_count) + " of your entries.";
    }

    std::string cmdClearAll() {
        int removed_count = 0;
        
        for (int i = 0; i < MAX_PEER_LIST_ENTRIES; i++) {
            if (shared_data->entries[i].is_active) {
                shared_data->entries[i].is_active = false;
                removed_count++;
            }
        }

        shared_data->active_entries = 0;

        if (removed_count > 0) {
            snprintf(shared_data->last_action, sizeof(shared_data->last_action), 
                    "Peer %s cleared ALL %d entries", peer_name.c_str(), removed_count);
        }

        return "🧹 Cleared ALL " + std::to_string(removed_count) + " entries from the list.";
    }

    std::string cmdSize() {
        return "📊 List contains " + std::to_string(shared_data->active_entries) + 
               " active entries (max: " + std::to_string(MAX_PEER_LIST_ENTRIES) + ").";
    }

    std::string cmdFind(const std::string& search_text) {
        if (search_text.empty()) {
            return "❌ Usage: find <text>";
        }

        std::stringstream ss;
        int found_count = 0;

        for (int i = 0; i < MAX_PEER_LIST_ENTRIES; i++) {
            if (shared_data->entries[i].is_active && 
                strstr(shared_data->entries[i].content, search_text.c_str()) != nullptr) {
                
                if (found_count == 0) {
                    ss << "🔍 Found entries containing \"" << search_text << "\":\n";
                }
                
                const PeerListEntry& entry = shared_data->entries[i];
                ss << "[" << entry.entry_id << "] \"" << entry.content 
                   << "\" (by " << entry.author << ")\n";
                found_count++;
            }
        }

        if (found_count == 0) {
            return "🔍 No entries found containing \"" + search_text + "\".";
        }

        return ss.str();
    }

    std::string cmdPeers() {
        std::stringstream ss;
        ss << "👥 Connected peers (" << shared_data->active_peers << " active):\n";

        bool found_any = false;
        for (int i = 0; i < MAX_PEER_CLIENTS; i++) {
            if (shared_data->peers[i].is_active) {
                const PeerInfo& peer = shared_data->peers[i];
                time_t now = time(nullptr);
                int seconds_ago = static_cast<int>(now - peer.last_seen);
                
                ss << "  [" << peer.peer_id << "] " << peer.name 
                   << " (" << peer.entries_created << " entries, last seen " 
                   << seconds_ago << "s ago)\n";
                found_any = true;
            }
        }

        if (!found_any) {
            ss << "  No active peers found.\n";
        }

        ss << "\nLast action: " << shared_data->last_action;
        return ss.str();
    }

    std::string cmdHelp() {
        return "📖 Available commands:\n"
               "  add <text>     - Add text to the shared list\n"
               "  list           - Show all entries\n"
               "  get <id>       - Get entry by ID\n"
               "  remove <id>    - Remove your own entry by ID\n"
               "  clear          - Remove all your own entries\n"
               "  clear_all      - Remove ALL entries (use carefully!)\n"
               "  size           - Show list size\n"
               "  find <text>    - Find entries containing text\n"
               "  peers          - Show connected peers\n"
               "  help           - Show this help\n"
               "  quit           - Exit client";
    }

    void parseAndExecuteCommand(const std::string& input) {
        std::istringstream iss(input);
        std::string command;
        iss >> command;

        PeerCommand cmd = parse_peer_command(command.c_str());
        std::string result;

        if (cmd == PEER_CMD_ADD || cmd == PEER_CMD_FIND) {
            std::string text;
            std::getline(iss, text);
            if (!text.empty() && text[0] == ' ') {
                text = text.substr(1); // Remove leading space
            }
            result = executeCommand(cmd, text);
        }
        else if (cmd == PEER_CMD_GET || cmd == PEER_CMD_REMOVE) {
            int entry_id;
            if (!(iss >> entry_id)) {
                result = "❌ Usage: " + command + " <entry_id>";
            } else {
                result = executeCommand(cmd, "", entry_id);
            }
        }
        else {
            result = executeCommand(cmd);
        }

        std::cout << result << std::endl << std::endl;
    }

    void interactiveMode() {
        std::cout << "🚀 Peer List Client Interactive Mode" << std::endl;
        std::cout << "💡 Type 'help' for available commands, 'quit' to exit" << std::endl;
        std::cout << "🔄 All operations are performed on shared memory visible to all peers" << std::endl << std::endl;

        std::string input;
        while (running) {
            std::cout << peer_name << "> ";
            std::getline(std::cin, input);

            if (input == "quit" || input == "exit") {
                break;
            }

            if (input.empty()) {
                continue;
            }

            parseAndExecuteCommand(input);
        }
    }

    void stop() {
        running = false;
    }
};

// Global client for signal handling
PeerClient* global_peer_client = nullptr;

void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << ". Shutting down..." << std::endl;
    if (global_peer_client) {
        global_peer_client->stop();
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3) {
        std::cout << "Usage: " << argv[0] << " <peer_id> [peer_name]" << std::endl;
        std::cout << "Example: " << argv[0] << " 1 Alice" << std::endl;
        std::cout << "Example: " << argv[0] << " 2" << std::endl;
        return 1;
    }

    int peer_id = std::stoi(argv[1]);
    std::string peer_name = (argc == 3) ? argv[2] : ("Peer" + std::to_string(peer_id));

    if (peer_id < 1 || peer_id > MAX_PEER_CLIENTS) {
        std::cerr << "❌ Peer ID must be between 1 and " << MAX_PEER_CLIENTS << std::endl;
        return 1;
    }

    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        PeerClient client(peer_id, peer_name);
        global_peer_client = &client;

        if (!client.connect()) {
            return 1;
        }

        client.interactiveMode();

    } catch (const std::exception& e) {
        std::cerr << "❌ Peer client error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}