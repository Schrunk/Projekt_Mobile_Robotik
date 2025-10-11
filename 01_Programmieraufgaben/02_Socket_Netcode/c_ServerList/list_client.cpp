#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 8080
#define BUFFER_SIZE 1024

/**
 * List Client - Connects to the List Server
 * and enables interactive use of List operations
 */
class ListClient {
private:
    int clientSocket;
    std::string serverIP;
    bool connected;
    
public:
    ListClient(const std::string& ip = "127.0.0.1") : serverIP(ip), connected(false), clientSocket(-1) {}
    
    ~ListClient() {
        disconnect();
    }
    
    bool connect() {
        // create socket
        clientSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (clientSocket < 0) {
            std::cerr << "Error creating socket" << std::endl;
            return false;
        }
        
        // configure server address
        sockaddr_in serverAddr{};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(PORT);
        
        if (inet_pton(AF_INET, serverIP.c_str(), &serverAddr.sin_addr) <= 0) {
            std::cerr << "Invalid server address" << std::endl;
            close(clientSocket);
            return false;
        }
        
        // establish connection to server
        if (::connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            std::cerr << "Connection to server failed" << std::endl;
            close(clientSocket);
            return false;
        }
        
        connected = true;
        std::cout << "Successfully connected to List Server (" << serverIP << ":" << PORT << ")" << std::endl;
        
        // receive welcome message
        receiveResponse();
        
        return true;
    }
    
    void disconnect() {
        if (connected && clientSocket >= 0) {
            close(clientSocket);
            connected = false;
            clientSocket = -1;
            std::cout << "Connection to server closed." << std::endl;
        }
    }
    
    void run() {
        if (!connected) {
            std::cerr << "Not connected to server!" << std::endl;
            return;
        }
        
        std::cout << "\n=== List Client started ===" << std::endl;
        std::cout << "Enter commands (HELP for help, QUIT to exit):" << std::endl;
        
        std::string input;
        while (connected) {
            std::cout << "\nList> ";
            if (!std::getline(std::cin, input)) {
                break;
            }
            
            if (input.empty()) {
                continue;
            }
            
            // send command to server
            if (!sendCommand(input)) {
                break;
            }
            
            // receive and display response
            if (!receiveResponse()) {
                break;
            }
            
            // exit loop on QUIT command
            if (input == "QUIT" || input == "quit") {
                break;
            }
        }
    }
    
private:
    bool sendCommand(const std::string& command) {
        ssize_t bytesSent = send(clientSocket, command.c_str(), command.length(), 0);
        if (bytesSent < 0) {
            std::cerr << "Error sending command" << std::endl;
            return false;
        }
        return true;
    }
    
    bool receiveResponse() {
        char buffer[BUFFER_SIZE];
        memset(buffer, 0, BUFFER_SIZE);
        
        ssize_t bytesReceived = recv(clientSocket, buffer, BUFFER_SIZE - 1, 0);
        if (bytesReceived <= 0) {
            std::cerr << "Connection to server lost" << std::endl;
            connected = false;
            return false;
        }
        
        std::cout << buffer;
        return true;
    }
};

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [server_ip]" << std::endl;
    std::cout << "  server_ip: IP address of the List Server (default: 127.0.0.1)" << std::endl;
    std::cout << "\nExamples:" << std::endl;
    std::cout << "  " << programName << "                    # Connect to localhost" << std::endl;
    std::cout << "  " << programName << " 192.168.1.100      # Connect to 192.168.1.100" << std::endl;
}

int main(int argc, char* argv[]) {
    std::string serverIP = "127.0.0.1"; // default: localhost
    
    if (argc > 2) {
        printUsage(argv[0]);
        return 1;
    }
    
    if (argc == 2) {
        serverIP = argv[1];
    }
    
    std::cout << "=== List Client ===" << std::endl;
    std::cout << "Attempting connection to " << serverIP << ":" << PORT << "..." << std::endl;
    
    ListClient client(serverIP);
    
    if (!client.connect()) {
        std::cerr << "Error: Could not connect to List Server!" << std::endl;
        std::cerr << "Make sure the server is running." << std::endl;
        return 1;
    }
    
    client.run();
    
    return 0;
}