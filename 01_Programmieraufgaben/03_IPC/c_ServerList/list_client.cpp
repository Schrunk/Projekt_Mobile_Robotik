#include "shared_memory_list.h"
#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <cstring>
#include <ctime>

class ListClient {
private:
    int client_id;
    int shm_id;
    int sem_id;
    ListSharedData* shared_data;
    int request_index;

public:
    ListClient(int id) : client_id(id), shm_id(-1), sem_id(-1), shared_data(nullptr) {
        request_index = client_id - 1;
    }

    ~ListClient() {
        disconnect();
    }

    bool connect() {
        if (client_id < 1 || client_id > MAX_CLIENTS) {
            std::cerr << "Client ID must be between 1 and " << MAX_CLIENTS << std::endl;
            return false;
        }

        // Attach to existing shared memory
        shm_id = attach_list_shared_memory();
        if (shm_id == -1) {
            std::cerr << "Cannot attach to shared memory. Is the list server running?" << std::endl;
            return false;
        }

        shared_data = static_cast<ListSharedData*>(shmat(shm_id, nullptr, 0));
        if (shared_data == reinterpret_cast<void*>(-1)) {
            perror("shmat failed");
            return false;
        }

        // Attach to semaphore
        sem_id = semget(LIST_SEM_KEY, 1, 0666);
        if (sem_id == -1) {
            perror("semget failed");
            shmdt(shared_data);
            return false;
        }

        std::cout << "=== List Client " << client_id << " (C++) ===" << std::endl;
        std::cout << "✅ Connected to list server" << std::endl;
        return true;
    }

    void disconnect() {
        if (shared_data != nullptr) {
            shmdt(shared_data);
            shared_data = nullptr;
        }
        std::cout << "🔌 Disconnected from list server" << std::endl;
    }

    bool sendCommand(ListCommand cmd, const std::string& argument = "", int index = -1) {
        if (lock_semaphore(sem_id) != 0) {
            return false;
        }

        ClientRequest& request = shared_data->client_requests[request_index];
        
        // Clear previous request
        memset(&request, 0, sizeof(ClientRequest));
        
        request.client_id = client_id;
        request.command = cmd;
        request.index = index;
        request.timestamp = time(nullptr);
        request.is_new = true;
        request.processed = false;

        if (!argument.empty()) {
            strncpy(request.argument, argument.c_str(), sizeof(request.argument) - 1);
            request.argument[sizeof(request.argument) - 1] = '\0';
        }

        shared_data->active_clients = client_id;

        unlock_semaphore(sem_id);

        std::cout << "📤 Command sent: " << command_to_string(cmd);
        if (!argument.empty()) {
            std::cout << " \"" << argument << "\"";
        }
        if (index >= 0) {
            std::cout << " (index: " << index << ")";
        }
        std::cout << std::endl;

        return true;
    }

    std::string waitForResponse() {
        const int max_wait_time = 5000; // 5 seconds
        const int check_interval = 100; // 100ms
        int waited_time = 0;

        while (waited_time < max_wait_time) {
            if (lock_semaphore(sem_id) == 0) {
                ServerResponse& response = shared_data->server_responses[request_index];
                
                if (response.is_new && response.client_id == client_id) {
                    std::string result = response.content;
                    response.is_new = false; // Mark as read
                    unlock_semaphore(sem_id);
                    return result;
                }
                unlock_semaphore(sem_id);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(check_interval));
            waited_time += check_interval;
        }

        return "⏰ Timeout waiting for server response";
    }

    void parseAndExecuteCommand(const std::string& input) {
        std::istringstream iss(input);
        std::string command;
        iss >> command;

        if (command == "add") {
            std::string text;
            std::getline(iss, text);
            if (!text.empty() && text[0] == ' ') {
                text = text.substr(1); // Remove leading space
            }
            
            if (text.empty()) {
                std::cout << "❌ Usage: add <text>" << std::endl;
                return;
            }
            
            sendCommand(CMD_ADD, text);
        }
        else if (command == "list") {
            sendCommand(CMD_LIST);
        }
        else if (command == "get") {
            int index;
            if (!(iss >> index)) {
                std::cout << "❌ Usage: get <index>" << std::endl;
                return;
            }
            sendCommand(CMD_GET, "", index);
        }
        else if (command == "remove") {
            int index;
            if (!(iss >> index)) {
                std::cout << "❌ Usage: remove <index>" << std::endl;
                return;
            }
            sendCommand(CMD_REMOVE, "", index);
        }
        else if (command == "clear") {
            sendCommand(CMD_CLEAR);
        }
        else if (command == "size") {
            sendCommand(CMD_SIZE);
        }
        else if (command == "find") {
            std::string text;
            std::getline(iss, text);
            if (!text.empty() && text[0] == ' ') {
                text = text.substr(1); // Remove leading space
            }
            
            if (text.empty()) {
                std::cout << "❌ Usage: find <text>" << std::endl;
                return;
            }
            
            sendCommand(CMD_FIND, text);
        }
        else if (command == "help") {
            sendCommand(CMD_HELP);
        }
        else {
            std::cout << "❌ Unknown command: " << command << std::endl;
            std::cout << "💡 Type 'help' for available commands" << std::endl;
            return;
        }

        // Wait for and display response
        std::string response = waitForResponse();
        std::cout << "📨 Server response:" << std::endl;
        std::cout << response << std::endl << std::endl;
    }

    void interactiveMode() {
        std::cout << "📝 List Client Interactive Mode" << std::endl;
        std::cout << "💡 Type 'help' for available commands, 'quit' to exit" << std::endl << std::endl;

        std::string input;
        while (true) {
            std::cout << "ListClient " << client_id << "> ";
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
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <client_id>" << std::endl;
        std::cout << "Example: " << argv[0] << " 1" << std::endl;
        return 1;
    }

    int client_id = std::stoi(argv[1]);

    try {
        ListClient client(client_id);

        if (!client.connect()) {
            return 1;
        }

        client.interactiveMode();

    } catch (const std::exception& e) {
        std::cerr << "❌ List client error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}