#include "shared_memory_list.h"
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <string>
#include <signal.h>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <ctime>

class ListServer {
private:
    int shm_id;
    int sem_id;
    ListSharedData* shared_data;
    volatile bool running;
    std::chrono::steady_clock::time_point last_status_time;

public:
    ListServer() : shm_id(-1), sem_id(-1), shared_data(nullptr), running(true) {
        last_status_time = std::chrono::steady_clock::now();
    }

    ~ListServer() {
        cleanup();
    }

    bool initialize() {
        std::cout << "=== IPC List Server (C++) ===" << std::endl;
        std::cout << "Initializing list server..." << std::endl;

        // Create shared memory
        shm_id = create_list_shared_memory();
        if (shm_id == -1) {
            std::cerr << "Failed to create shared memory" << std::endl;
            return false;
        }

        // Attach to shared memory
        shared_data = static_cast<ListSharedData*>(shmat(shm_id, nullptr, 0));
        if (shared_data == reinterpret_cast<void*>(-1)) {
            perror("shmat failed");
            return false;
        }

        // Create semaphore
        sem_id = create_list_semaphore();
        if (sem_id == -1) {
            shmdt(shared_data);
            return false;
        }

        // Initialize shared data
        if (lock_semaphore(sem_id) == 0) {
            memset(shared_data, 0, sizeof(ListSharedData));
            shared_data->entry_count = 0;
            shared_data->active_clients = 0;
            shared_data->last_activity = time(nullptr);
            unlock_semaphore(sem_id);
        }

        std::cout << "✅ List server initialized successfully" << std::endl;
        std::cout << "📊 Shared memory ID: " << shm_id << std::endl;
        std::cout << "🔒 Semaphore ID: " << sem_id << std::endl;
        std::cout << "📝 Max list entries: " << MAX_LIST_ENTRIES << std::endl;
        std::cout << "⏳ Waiting for client commands..." << std::endl << std::endl;

        return true;
    }

    void processClientRequests() {
        if (lock_semaphore(sem_id) != 0) {
            return;
        }

        for (int i = 0; i < MAX_CLIENTS; i++) {
            ClientRequest& request = shared_data->client_requests[i];
            ServerResponse& response = shared_data->server_responses[i];

            if (request.is_new && !request.processed) {
                std::cout << "📨 Processing command from Client " << request.client_id 
                         << ": " << command_to_string(request.command);
                
                if (strlen(request.argument) > 0) {
                    std::cout << " \"" << request.argument << "\"";
                }
                std::cout << std::endl;

                // Process the command
                processCommand(request, response);
                
                // Mark as processed
                request.processed = true;
                request.is_new = false;
                
                // Update activity
                shared_data->last_activity = time(nullptr);
                
                std::cout << "📤 Response sent to Client " << request.client_id << std::endl << std::endl;
            }
        }

        unlock_semaphore(sem_id);
    }

    void processCommand(const ClientRequest& request, ServerResponse& response) {
        response.client_id = request.client_id;
        response.timestamp = time(nullptr);
        response.success = true;
        response.result_count = 0;
        response.is_new = true;

        switch (request.command) {
            case CMD_ADD:
                handleAdd(request, response);
                break;
            case CMD_LIST:
                handleList(request, response);
                break;
            case CMD_GET:
                handleGet(request, response);
                break;
            case CMD_REMOVE:
                handleRemove(request, response);
                break;
            case CMD_CLEAR:
                handleClear(request, response);
                break;
            case CMD_SIZE:
                handleSize(request, response);
                break;
            case CMD_FIND:
                handleFind(request, response);
                break;
            case CMD_HELP:
                handleHelp(request, response);
                break;
            default:
                response.success = false;
                strncpy(response.content, "Unknown command. Type 'help' for available commands.", 
                       sizeof(response.content) - 1);
                break;
        }
    }

    void handleAdd(const ClientRequest& request, ServerResponse& response) {
        if (shared_data->entry_count >= MAX_LIST_ENTRIES) {
            response.success = false;
            strncpy(response.content, "List is full. Cannot add more entries.", 
                   sizeof(response.content) - 1);
            return;
        }

        if (strlen(request.argument) == 0) {
            response.success = false;
            strncpy(response.content, "Usage: add <text>", sizeof(response.content) - 1);
            return;
        }

        ListEntry& entry = shared_data->entries[shared_data->entry_count];
        entry.client_id = request.client_id;
        strncpy(entry.content, request.argument, sizeof(entry.content) - 1);
        entry.content[sizeof(entry.content) - 1] = '\0';
        entry.timestamp = time(nullptr);
        entry.is_active = true;

        shared_data->entry_count++;

        snprintf(response.content, sizeof(response.content), 
                "Added entry #%d: \"%s\"", shared_data->entry_count - 1, entry.content);
    }

    void handleList(const ClientRequest&, ServerResponse& response) {
        if (shared_data->entry_count == 0) {
            strncpy(response.content, "List is empty.", sizeof(response.content) - 1);
            return;
        }

        std::stringstream ss;
        ss << "List entries (" << shared_data->entry_count << " total):\n";
        
        int active_count = 0;
        for (int i = 0; i < shared_data->entry_count; i++) {
            if (shared_data->entries[i].is_active) {
                ss << "[" << i << "] \"" << shared_data->entries[i].content 
                   << "\" (Client " << shared_data->entries[i].client_id << ")\n";
                active_count++;
            }
        }
        
        response.result_count = active_count;
        strncpy(response.content, ss.str().c_str(), sizeof(response.content) - 1);
        response.content[sizeof(response.content) - 1] = '\0';
    }

    void handleGet(const ClientRequest& request, ServerResponse& response) {
        int index = request.index;
        
        if (index < 0 || index >= shared_data->entry_count || 
            !shared_data->entries[index].is_active) {
            response.success = false;
            snprintf(response.content, sizeof(response.content), 
                    "Invalid index %d. Use 'list' to see available entries.", index);
            return;
        }

        const ListEntry& entry = shared_data->entries[index];
        snprintf(response.content, sizeof(response.content), 
                "Entry #%d: \"%s\" (Client %d, %s)", 
                index, entry.content, entry.client_id, ctime(&entry.timestamp));
        
        // Remove newline from ctime
        char* newline = strchr(response.content, '\n');
        if (newline) *newline = ')';
    }

    void handleRemove(const ClientRequest& request, ServerResponse& response) {
        int index = request.index;
        
        if (index < 0 || index >= shared_data->entry_count || 
            !shared_data->entries[index].is_active) {
            response.success = false;
            snprintf(response.content, sizeof(response.content), 
                    "Invalid index %d. Use 'list' to see available entries.", index);
            return;
        }

        shared_data->entries[index].is_active = false;
        snprintf(response.content, sizeof(response.content), 
                "Removed entry #%d: \"%s\"", index, shared_data->entries[index].content);
    }

    void handleClear(const ClientRequest&, ServerResponse& response) {
        int cleared_count = 0;
        for (int i = 0; i < shared_data->entry_count; i++) {
            if (shared_data->entries[i].is_active) {
                shared_data->entries[i].is_active = false;
                cleared_count++;
            }
        }
        
        snprintf(response.content, sizeof(response.content), 
                "Cleared %d entries from the list.", cleared_count);
    }

    void handleSize(const ClientRequest&, ServerResponse& response) {
        int active_count = 0;
        for (int i = 0; i < shared_data->entry_count; i++) {
            if (shared_data->entries[i].is_active) {
                active_count++;
            }
        }
        
        response.result_count = active_count;
        snprintf(response.content, sizeof(response.content), 
                "List contains %d active entries (max: %d).", active_count, MAX_LIST_ENTRIES);
    }

    void handleFind(const ClientRequest& request, ServerResponse& response) {
        if (strlen(request.argument) == 0) {
            response.success = false;
            strncpy(response.content, "Usage: find <text>", sizeof(response.content) - 1);
            return;
        }

        std::stringstream ss;
        int found_count = 0;
        
        for (int i = 0; i < shared_data->entry_count; i++) {
            if (shared_data->entries[i].is_active && 
                strstr(shared_data->entries[i].content, request.argument) != nullptr) {
                
                if (found_count == 0) {
                    ss << "Found entries containing \"" << request.argument << "\":\n";
                }
                ss << "[" << i << "] \"" << shared_data->entries[i].content << "\"\n";
                found_count++;
            }
        }
        
        if (found_count == 0) {
            ss << "No entries found containing \"" << request.argument << "\".";
        }
        
        response.result_count = found_count;
        strncpy(response.content, ss.str().c_str(), sizeof(response.content) - 1);
        response.content[sizeof(response.content) - 1] = '\0';
    }

    void handleHelp(const ClientRequest&, ServerResponse& response) {
        const char* help_text = 
            "Available commands:\n"
            "  add <text>     - Add text to the list\n"
            "  list           - Show all entries\n"
            "  get <index>    - Get entry at index\n"
            "  remove <index> - Remove entry at index\n"
            "  clear          - Remove all entries\n"
            "  size           - Show list size\n"
            "  find <text>    - Find entries containing text\n"
            "  help           - Show this help\n"
            "  quit           - Exit client";
        
        strncpy(response.content, help_text, sizeof(response.content) - 1);
        response.content[sizeof(response.content) - 1] = '\0';
    }

    void displayStatus() {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_status_time);

        if (duration.count() >= 10) {
            if (lock_semaphore(sem_id) == 0) {
                int active_entries = 0;
                for (int i = 0; i < shared_data->entry_count; i++) {
                    if (shared_data->entries[i].is_active) active_entries++;
                }

                std::cout << "🔄 Server Status - Active entries: " << active_entries 
                         << "/" << shared_data->entry_count 
                         << ", Active clients: " << shared_data->active_clients << std::endl;
                unlock_semaphore(sem_id);
            }
            last_status_time = now;
        }
    }

    void run() {
        std::cout << "🚀 List server started. Press Ctrl+C to stop." << std::endl;

        while (running) {
            processClientRequests();
            displayStatus();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void stop() {
        running = false;
        std::cout << "\n🛑 Stopping list server..." << std::endl;
    }

    void cleanup() {
        if (shared_data != nullptr) {
            shmdt(shared_data);
            shared_data = nullptr;
        }

        if (shm_id != -1) {
            if (shmctl(shm_id, IPC_RMID, NULL) == -1) {
                perror("shmctl IPC_RMID failed");
            } else {
                std::cout << "📝 List shared memory cleaned up" << std::endl;
            }
        }
        
        if (sem_id != -1) {
            if (semctl(sem_id, 0, IPC_RMID) == -1) {
                perror("semctl IPC_RMID failed");
            } else {
                std::cout << "🔒 List semaphore cleaned up" << std::endl;
            }
        }
        
        std::cout << "✅ List server shutdown complete." << std::endl;
    }
};

// Global server instance for signal handling
std::unique_ptr<ListServer> global_list_server;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << ". Shutting down list server..." << std::endl;
    if (global_list_server) {
        global_list_server->stop();
    }
}

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        global_list_server = std::make_unique<ListServer>();

        if (!global_list_server->initialize()) {
            std::cerr << "❌ Failed to initialize list server" << std::endl;
            return 1;
        }

        global_list_server->run();

    } catch (const std::exception& e) {
        std::cerr << "❌ List server error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}