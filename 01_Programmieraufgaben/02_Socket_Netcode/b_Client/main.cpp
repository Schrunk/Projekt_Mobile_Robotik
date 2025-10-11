#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <thread>
#include <atomic>

/*
 * TCP Client für Kommunikation mit einem Server
 * Kann Daten senden und empfangen
 * Usage: <name> <IP> <PORT>
 */

#define IP "127.0.0.1"
#define PORT 8080

class TCPClient {
private:
    int sock;
    std::string server_ip;
    int server_port;
    std::atomic<bool> running{true};
    
public:
    TCPClient(const std::string& ip, int port) : server_ip(ip), server_port(port), sock(-1) {}
    
    ~TCPClient() {
        disconnect();
    }
    
    bool connect_to_server() {
        // Socket erstellen
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << "Socket creation error\n";
            return false;
        }
        
        // Server-Adresse konfigurieren
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port);
        
        // IP-Adresse konvertieren
        if (inet_pton(AF_INET, server_ip.c_str(), &server_addr.sin_addr) <= 0) {
            std::cerr << "Invalid address/ Address not supported\n";
            close(sock);
            return false;
        }
        
        // Verbindung zum Server herstellen
        if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Connection Failed\n";
            close(sock);
            return false;
        }
        
        std::cout << "Successfully connected to " << server_ip << ":" << server_port << "\n";
        return true;
    }
    
    void disconnect() {
        running = false;
        if (sock >= 0) {
            close(sock);
            sock = -1;
        }
    }
    
    bool send_data(const std::string& data) {
        if (sock < 0) return false;
        
        ssize_t sent = send(sock, data.c_str(), data.length(), 0);
        if (sent < 0) {
            std::cerr << "Send failed\n";
            return false;
        }
        return true;
    }
    
    std::string receive_data() {
        if (sock < 0) {
            return "";
        }

        char buffer[1024] = {0};
        ssize_t received = recv(sock, buffer, sizeof(buffer) - 1, 0);
        
        if (received < 0) {
            std::cerr << "Receive failed\n";
            return "";
        } else if (received == 0) {
            std::cout << "Server disconnected\n";
            running = false;
            return "";
        }
        
        return std::string(buffer, received);
    }
    
    void start_receive_thread() {
        std::thread receive_thread([this]() {
            while (running) {
                std::string received = receive_data();
                if (!received.empty()) {
                    std::cout << "\n[Server]: " << received << std::endl;
                    std::cout << "> " << std::flush;
                }

                if (!running) {
                    break;
                }
            }
        });
        receive_thread.detach();
    }
    
    void run_interactive() {
        std::cout << "\n=== TCP Client gestartet ===\n";
        std::cout << "Geben Sie Nachrichten ein. 'exit' zum Beenden.\n";
        std::cout << "Verbunden mit: " << server_ip << ":" << server_port << "\n\n";
        
        // Thread für das Empfangen von Nachrichten starten
        start_receive_thread();
        
        std::string input;
        while (running) {
            std::cout << "> ";
            if (!std::getline(std::cin, input)) {
                break;
            }
            
            if (input == "exit" || input == "quit") {
                break;
            }
            
            if (!input.empty()) {
                if (!send_data(input)) {
                    std::cerr << "Failed to send message\n";
                    break;
                }
            }
        }
        
        disconnect();
        std::cout << "\nConnection closed.\n";
    }
};

int main(int argc, char* argv[]) {
    // Argumente prüfen
    // if (argc != 3) {
    //     std::cerr << "Usage: " << argv[0] << " <IP> <PORT>\n";
    //     std::cerr << "Example: " << argv[0] << " 127.0.0.1 8080\n";
    //     return 1;
    // }
    
    // std::string server_ip = argv[1];
    // int server_port;
    
    // try {
    //     server_port = std::stoi(argv[2]);
    //     if (server_port <= 0 || server_port > 65535) {
    //         std::cerr << "Error: Port must be between 1 and 65535\n";
    //         return 1;
    //     }
    // } catch (const std::exception& e) {
    //     std::cerr << "Error: Invalid port number\n";
    //     return 1;
    // }
    
    // // Client erstellen und verbinden
    // TCPClient client(server_ip, server_port);

    TCPClient client(IP, PORT);
    
    if (!client.connect_to_server()) {
        std::cerr << "Failed to connect to server\n";
        return 1;
    }
    
    // Interaktive Kommunikation starten
    client.run_interactive();
    
    return 0;
}