#include "shared_memory_common.h"
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

class IPCServer {
private:
    int shm_id;
    int sem_id;
    SharedData* shared_data;
    volatile bool running;
    std::chrono::steady_clock::time_point last_status_time;

public:
    IPCServer() : shm_id(-1), sem_id(-1), shared_data(nullptr), running(true) {
        last_status_time = std::chrono::steady_clock::now();
    }

    ~IPCServer() {
        cleanup();
    }

    // Initialize server resources
    bool initialize() {
        std::cout << "=== IPC Shared Memory Server (C++) ===" << std::endl;
        std::cout << "Initializing server..." << std::endl;

        // Create shared memory
        shm_id = create_shared_memory();
        if (shm_id == -1) {
            std::cerr << "Failed to create shared memory" << std::endl;
            return false;
        }

        // Attach to shared memory
        shared_data = static_cast<SharedData*>(shmat(shm_id, nullptr, 0));
        if (shared_data == reinterpret_cast<void*>(-1)) {
            perror("shmat failed");
            return false;
        }

        // Create semaphore
        sem_id = create_semaphore();
        if (sem_id == -1) {
            shmdt(shared_data);
            return false;
        }

        // Initialize shared data
        if (lock_semaphore(sem_id) == 0) {
            memset(shared_data, 0, sizeof(SharedData));
            shared_data->active_clients = 0;
            shared_data->last_activity = time(nullptr);
            unlock_semaphore(sem_id);
        }

        std::cout << "✅ Server initialized successfully" << std::endl;
        std::cout << "📊 Shared memory ID: " << shm_id << std::endl;
        std::cout << "🔒 Semaphore ID: " << sem_id << std::endl;
        std::cout << "⏳ Waiting for client messages..." << std::endl << std::endl;

        return true;
    }

    // Process incoming messages and send echo responses
    void processMessages() {
        if (lock_semaphore(sem_id) != 0) {
            return;
        }

        for (int i = 0; i < MAX_CLIENTS; i++) {
            Message& client_msg = shared_data->client_messages[i];
            Message& server_response = shared_data->server_responses[i];

            // Check if there's a new message from client
            if (client_msg.is_new == 1) {
                
                std::cout << "📨 New message from Client " << client_msg.client_id 
                         << ": \"" << client_msg.content << "\"" << std::endl;

                // Create echo response with timestamp
                server_response.client_id = 0; // Server ID
                
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << "ECHO [" << std::put_time(std::localtime(&time_t), "%H:%M:%S") 
                   << "]: " << client_msg.content;
                
                strncpy(server_response.content, ss.str().c_str(), MAX_MESSAGE_SIZE - 1);
                server_response.content[MAX_MESSAGE_SIZE - 1] = '\0';
                server_response.timestamp = time(nullptr);
                server_response.is_new = 1;

                // Mark client message as processed
                client_msg.is_new = 0;

                // Update activity timestamp
                shared_data->last_activity = time(nullptr);

                std::cout << "📤 Echo response sent: \"" << server_response.content 
                         << "\"" << std::endl << std::endl;
            }
        }

        unlock_semaphore(sem_id);
    }

    // Display server status periodically
    void displayStatus() {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_status_time);

        // Display status every 10 seconds
        if (duration.count() >= 10) {
            if (lock_semaphore(sem_id) == 0) {
                auto time_str = std::ctime(&shared_data->last_activity);
                // Remove newline from ctime output
                std::string time_clean(time_str);
                if (!time_clean.empty() && time_clean.back() == '\n') {
                    time_clean.pop_back();
                }

                std::cout << "🔄 Server Status - Active clients: " 
                         << shared_data->active_clients 
                         << ", Last activity: " << time_clean << std::endl;
                unlock_semaphore(sem_id);
            }
            last_status_time = now;
        }
    }

    // Main server loop
    void run() {
        std::cout << "🚀 Server started. Press Ctrl+C to stop." << std::endl;

        while (running) {
            processMessages();
            displayStatus();

            // Sleep for 100ms to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // Stop the server
    void stop() {
        running = false;
        std::cout << "\n🛑 Stopping server..." << std::endl;
    }

    // Cleanup resources
    void cleanup() {
        if (shared_data != nullptr) {
            shmdt(shared_data);
            shared_data = nullptr;
        }

        cleanup_resources(shm_id, sem_id);
        std::cout << "✅ Server shutdown complete." << std::endl;
    }

    // Check if server is running
    bool isRunning() const {
        return running;
    }
};

// Global server instance for signal handling
std::unique_ptr<IPCServer> global_server;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << ". Shutting down server..." << std::endl;
    if (global_server) {
        global_server->stop();
    }
}

int main() {
    // Set up signal handlers for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        // Create server instance
        global_server = std::make_unique<IPCServer>();

        // Initialize server
        if (!global_server->initialize()) {
            std::cerr << "❌ Failed to initialize server" << std::endl;
            return 1;
        }

        // Run server loop
        global_server->run();

    } catch (const std::exception& e) {
        std::cerr << "❌ Server error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}