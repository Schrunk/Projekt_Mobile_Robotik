#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <mutex>
#include "list.hpp"

#define PORT 8080
#define BUFFER_SIZE 1024

/**
 * List Server - Provides List operations over the network
 * 
 * Supported commands:
 * - PUSH <value>     : Adds a value to the list
 * - POP              : Removes the top element
 * - TOP              : Returns the top element
 * - LENGTH           : Returns the number of elements
 * - ISEMPTY          : Checks if the list is empty
 * - PRINT            : Outputs all elements
 * - CLEAR            : Empties the entire list
 * - HELP             : Shows all available commands
 * - QUIT             : Ends the client connection
 */
class ListServer {
private:
    List<std::string> serverList;  // The server list (as string list)
    std::mutex listMutex;          // Mutex for thread safety
    int serverSocket;
    bool running;
    
public:
    ListServer() : running(false), serverSocket(-1) {}
    
    ~ListServer() {
        stop();
    }
    
    bool start() {
        // Create socket
        serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket < 0) {
            std::cerr << "Error creating socket" << std::endl;
            return false;
        }
        
        // Set socket options (reuse address)
        int opt = 1;
        if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            std::cerr << "Error in setsockopt" << std::endl;
            close(serverSocket);
            return false;
        }
        
        // Configure server address
        sockaddr_in serverAddr{};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        serverAddr.sin_port = htons(PORT);
        
        // Bind socket
        if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            std::cerr << "Error binding socket" << std::endl;
            close(serverSocket);
            return false;
        }
        
        // Listen for connections
        if (listen(serverSocket, 5) < 0) {
            std::cerr << "Error listening" << std::endl;
            close(serverSocket);
            return false;
        }
        
        running = true;
        std::cout << "List Server started on port " << PORT << std::endl;
        std::cout << "Available commands for clients:" << std::endl;
        std::cout << "  PUSH <value>, POP, TOP, LENGTH, ISEMPTY, PRINT, CLEAR, HELP, QUIT" << std::endl;
        std::cout << "Waiting for client connections..." << std::endl;
        
        return true;
    }
    
    void run() {
        while (running) {
            sockaddr_in clientAddr{};
            socklen_t clientLen = sizeof(clientAddr);
            
            int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);
            if (clientSocket < 0) {
                if (running) {
                    std::cerr << "Error accepting connection" << std::endl;
                }
                continue;
            }
            
            // Start new thread for client
            std::thread clientThread(&ListServer::handleClient, this, clientSocket, clientAddr);
            clientThread.detach();
        }
    }
    
    void stop() {
        running = false;
        if (serverSocket >= 0) {
            close(serverSocket);
            serverSocket = -1;
        }
    }
    
private:
    void handleClient(int clientSocket, sockaddr_in clientAddr) {
        char clientIP[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &clientAddr.sin_addr, clientIP, INET_ADDRSTRLEN);
        
        std::cout << "New client connected: " << clientIP << ":" << ntohs(clientAddr.sin_port) << std::endl;
        
        // Send welcome message
        std::string welcome = "Welcome to List Server!\nAvailable commands: PUSH <value>, POP, TOP, LENGTH, ISEMPTY, PRINT, CLEAR, HELP, QUIT\n";
        send(clientSocket, welcome.c_str(), welcome.length(), 0);
        
        char buffer[BUFFER_SIZE];
        while (true) {
            memset(buffer, 0, BUFFER_SIZE);
            ssize_t bytesReceived = recv(clientSocket, buffer, BUFFER_SIZE - 1, 0);
            
            if (bytesReceived <= 0) {
                break; // Client closed connection
            }
            
            std::string command(buffer);
            // Remove newline characters
            if (!command.empty() && command.back() == '\n') {
                command.pop_back();
            }
            if (!command.empty() && command.back() == '\r') {
                command.pop_back();
            }
            
            std::cout << "Client " << clientIP << " -> " << command << std::endl;
            
            std::string response = processCommand(command);
            send(clientSocket, response.c_str(), response.length(), 0);
            
            // If QUIT command, end connection
            if (command == "QUIT") {
                break;
            }
        }
        
        close(clientSocket);
        std::cout << "Client " << clientIP << " disconnected" << std::endl;
    }
    
    std::string processCommand(const std::string& command) {
        std::lock_guard<std::mutex> lock(listMutex);
        
        std::istringstream iss(command);
        std::string cmd;
        iss >> cmd;
        
        // Convert command to uppercase
        for (char& c : cmd) {
            c = std::toupper(c);
        }
        
        try {
            if (cmd == "PUSH") {
                std::string value;
                if (std::getline(iss, value) && !value.empty()) {
                    // Remove leading whitespace
                    value = value.substr(value.find_first_not_of(" \t"));
                    serverList.push(value);
                    return "OK: '" + value + "' added to list\n";
                } else {
                    return "ERROR: PUSH requires a value (PUSH <value>)\n";
                }
            }
            else if (cmd == "POP") {
                if (serverList.isEmpty()) {
                    return "ERROR: List is empty\n";
                } else {
                    std::string topValue = serverList.top();
                    serverList.pop();
                    return "OK: '" + topValue + "' removed\n";
                }
            }
            else if (cmd == "TOP") {
                if (serverList.isEmpty()) {
                    return "ERROR: List is empty\n";
                } else {
                    return "OK: " + serverList.top() + "\n";
                }
            }
            else if (cmd == "LENGTH") {
                return "OK: " + std::to_string(serverList.length()) + "\n";
            }
            else if (cmd == "ISEMPTY") {
                return serverList.isEmpty() ? "OK: true\n" : "OK: false\n";
            }
            else if (cmd == "PRINT") {
                if (serverList.isEmpty()) {
                    return "OK: List is empty\n";
                } else {
                    // Collect all elements
                    List<std::string> tempList = serverList; // Create copy
                    std::vector<std::string> elements;
                    
                    while (!tempList.isEmpty()) {
                        elements.push_back(tempList.top());
                        tempList.pop();
                    }
                    
                    std::string result = "OK: [";
                    for (size_t i = 0; i < elements.size(); ++i) {
                        if (i > 0) result += ", ";
                        result += "'" + elements[i] + "'";
                    }
                    result += "]\n";
                    return result;
                }
            }
            else if (cmd == "CLEAR") {
                size_t count = serverList.length();
                serverList.clear();
                return "OK: " + std::to_string(count) + " elements removed\n";
            }
            else if (cmd == "HELP") {
                return "Available commands:\n"
                       "  PUSH <value> - Adds a value to the list\n"
                       "  POP          - Removes the top element\n"
                       "  TOP          - Returns the top element\n"
                       "  LENGTH       - Returns the number of elements\n"
                       "  ISEMPTY      - Checks if the list is empty\n"
                       "  PRINT        - Outputs all elements\n"
                       "  CLEAR        - Empties the entire list\n"
                       "  HELP         - Shows this help\n"
                       "  QUIT         - Ends the connection\n";
            }
            else if (cmd == "QUIT") {
                return "OK: Goodbye!\n";
            }
            else {
                return "ERROR: Unknown command '" + cmd + "'. Use HELP for help.\n";
            }
        }
        catch (const std::exception& e) {
            return "ERROR: " + std::string(e.what()) + "\n";
        }
    }
};

int main() {
    ListServer server;
    
    if (!server.start()) {
        std::cerr << "Error starting server" << std::endl;
        return 1;
    }
    
    // Run server in separate thread
    std::thread serverThread(&ListServer::run, &server);
    
    std::cout << "\nServer running. Press Enter to stop..." << std::endl;
    std::cin.get();
    
    server.stop();
    serverThread.join();
    
    std::cout << "Server stopped." << std::endl;
    return 0;
}