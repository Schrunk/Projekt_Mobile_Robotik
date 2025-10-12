#include "shared_memory_common.h"
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <cstring>
#include <ctime>

class IPCClient {
private:
    int client_id;
    int shm_id;
    int sem_id;
    SharedData* shared_data;
    int message_index;

public:
    IPCClient(int id) : client_id(id), shm_id(-1), sem_id(-1), shared_data(nullptr) {
        message_index = client_id - 1; // Convert to 0-based index
    }

    ~IPCClient() {
        disconnect();
    }

    bool connect() {
        if (client_id < 1 || client_id > MAX_CLIENTS) {
            std::cerr << "Client ID must be between 1 and " << MAX_CLIENTS << std::endl;
            return false;
        }

        // Attach to existing shared memory
        shm_id = attach_shared_memory();
        if (shm_id == -1) {
            std::cerr << "Cannot attach to shared memory. Is the server running?" << std::endl;
            return false;
        }

        shared_data = static_cast<SharedData*>(shmat(shm_id, nullptr, 0));
        if (shared_data == reinterpret_cast<void*>(-1)) {
            perror("shmat failed");
            return false;
        } else {
            shared_data->active_clients++;
        }

        // Attach to semaphore
        sem_id = semget(SEM_KEY, 1, 0666);
        if (sem_id == -1) {
            perror("semget failed");
            shmdt(shared_data);
            return false;
        }

        std::cout << "=== IPC Client " << client_id << " (C++) ===" << std::endl;
        std::cout << "✅ Connected to shared memory" << std::endl;
        return true;
    }

    void disconnect() {
        if (shared_data != nullptr) {
            shared_data->active_clients--;
            shmdt(shared_data);
            shared_data = nullptr;
        }
        std::cout << "🔌 Disconnected from server" << std::endl;
    }

    bool sendMessage(const std::string& message) {
        if (message.empty() || message.length() >= MAX_MESSAGE_SIZE) {
            std::cerr << "Message too long or empty" << std::endl;
            return false;
        }

        if (lock_semaphore(sem_id) != 0) {
            return false;
        }

        Message& client_msg = shared_data->client_messages[message_index];

        client_msg.client_id = client_id;
        strncpy(client_msg.content, message.c_str(), MAX_MESSAGE_SIZE - 1);
        client_msg.content[MAX_MESSAGE_SIZE - 1] = '\0';
        client_msg.timestamp = time(nullptr);
        client_msg.is_new = 1;

        unlock_semaphore(sem_id);

        std::cout << "📤 Message sent to server" << std::endl;
        return true;
    }

    std::string checkForResponse() {
        if (lock_semaphore(sem_id) != 0) {
            return "";
        }

        Message& server_response = shared_data->server_responses[message_index];
        std::string response;

        if (server_response.is_new == 1) {
            response = server_response.content;
            server_response.is_new = 0; // Mark as read
        }

        unlock_semaphore(sem_id);
        return response;
    }

    void interactiveMode() {
        std::cout << "💬 Enter messages (type 'quit' to exit):" << std::endl;

        std::string input;
        while (true) {
            std::cout << "Client " << client_id << "> ";
            std::getline(std::cin, input);

            if (input == "quit" || input == "exit") {
                break;
            }

            if (input.empty()) {
                continue;
            }

            if (sendMessage(input)) {
                // Wait a moment and check for response
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                std::string response = checkForResponse();
                if (!response.empty()) {
                    std::cout << "📨 Server response: \"" << response << "\"" << std::endl;
                } else {
                    std::cout << "⏳ No response yet..." << std::endl;
                }
            }
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
        IPCClient client(client_id);

        if (!client.connect()) {
            return 1;
        }

        client.interactiveMode();

    } catch (const std::exception& e) {
        std::cerr << "❌ Client error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}