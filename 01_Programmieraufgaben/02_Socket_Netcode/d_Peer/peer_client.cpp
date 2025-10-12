#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>
#include <ifaddrs.h>


#define LOCAL true
// Outputs
#define DEBUG false

#define BASE_PORT 8080
#define MAX_CLIENTS 10
#define BUFFER_SIZE 1024
#define DISCOVERY_INTERVAL 60  // seconds
#define LOCAL_IP "127.0.0.1"

/**
 * Peer-to-Peer Client System
 * Each client finds a free port starting from 8080
 * Clients discover each other via ping mechanism
 * Messages can be broadcast to all peers with response capability
 */

struct PeerInfo {
    std::string ip;
    int port;
    std::string clientId;
    std::chrono::steady_clock::time_point lastSeen;
    
    PeerInfo() = default;
    PeerInfo(const std::string& ip, int port, const std::string& id) 
        : ip(ip), port(port), clientId(id), lastSeen(std::chrono::steady_clock::now()) {}
};

struct IncomingMessage {
    std::string senderId;
    std::string content;
    std::chrono::steady_clock::time_point timestamp;
    
    IncomingMessage(const std::string& id, const std::string& msg) 
        : senderId(id), content(msg), timestamp(std::chrono::steady_clock::now()) {}
};

class PeerClient {
private:
    std::string clientId;
    std::string localIP;
    int serverPort;
    int serverSocket;
    std::atomic<bool> running{false};
    
    std::map<std::string, PeerInfo> peers;
    std::mutex peersMutex;
    std::mutex debugMutex; // For thread-safe debug output
    
    std::queue<IncomingMessage> messageQueue;
    std::mutex messageQueueMutex;
    
    std::thread serverThread;
    std::thread discoveryThread;
    
public:
    PeerClient(const std::string& id = "") : serverSocket(-1) {
        clientId = id.empty() ? generateClientId() : id;
        #if LOCAL
            localIP = LOCAL_IP;
        #else   
            localIP = getLocalIP();
        #endif
        serverPort = findFreePort();
    }
    
    ~PeerClient() {
        stop();
    }
    
    bool start() {
        if (!startServer()) {
            return false;
        }
        
        running = true;
        
        // Start server thread to handle incoming connections
        serverThread = std::thread(&PeerClient::serverLoop, this);
        
        // Start discovery thread to find other peers
        discoveryThread = std::thread(&PeerClient::discoveryLoop, this);
        
        std::cout << "Peer Client '" << clientId << "' started on " << localIP << ":" << serverPort << std::endl;
        std::cout << "Commands:" << std::endl;
        std::cout << "  list          - Show all known peers" << std::endl;
        std::cout << "  send <msg>    - Send message to all peers" << std::endl;
        std::cout << "  ping          - Manual discovery ping" << std::endl;
        std::cout << "  test          - Test connection to specific port" << std::endl;
        std::cout << "  messages      - Show pending messages" << std::endl;
        std::cout << "  reply <id>    - Reply to message from specific peer" << std::endl;
        std::cout << "  quit          - Exit application" << std::endl;
        
        return true;
    }
    
    void stop() {
        running = false;
        
        if (serverSocket >= 0) {
            close(serverSocket);
        }
        
        if (serverThread.joinable()) {
            serverThread.join();
        }
        
        if (discoveryThread.joinable()) {
            discoveryThread.join();
        }
    }
    
    void run() {
        std::string input;
        while (running) {
            std::cout << clientId << "> ";
            if (!std::getline(std::cin, input)) {
                break;
            }
            
            if (input.empty()) continue;
            
            if (input == "quit" || input == "exit") {
                break;
            } else if (input == "list") {
                listPeers();
            } else if (input == "ping") {
                discoverPeers();
            } else if (input == "test") {
                std::cout << "Enter port to test: ";
                int testPort;
                std::cin >> testPort;
                std::cin.ignore(); // Clear newline
                pingPeer(localIP, testPort);
            } else if (input == "messages") {
                showPendingMessages();
            } else if (input.substr(0, 6) == "reply ") {
                std::string senderId = input.substr(6);
                replyToMessage(senderId);
            } else if (input.substr(0, 5) == "send ") {
                std::string message = input.substr(5);
                sendMessageToAll(message);
            } else {
                std::cout << "Unknown command. Use: list, send <msg>, messages, reply <id>, ping, quit" << std::endl;
            }
        }
    }
    
private:
    std::string generateClientId() {
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        return "Client_" + std::to_string(timestamp % 10000);
    }
    
    std::string getLocalIP() {
        struct ifaddrs *ifaddrs_ptr;
        std::string ip = "127.0.0.1";
        
        if (getifaddrs(&ifaddrs_ptr) == 0) {
            for (struct ifaddrs *ifa = ifaddrs_ptr; ifa != nullptr; ifa = ifa->ifa_next) {
                if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) {
                    struct sockaddr_in* sockaddr_ipv4 = (struct sockaddr_in*)ifa->ifa_addr;
                    char ip_str[INET_ADDRSTRLEN];
                    
                    if (inet_ntop(AF_INET, &sockaddr_ipv4->sin_addr, ip_str, INET_ADDRSTRLEN)) {
                        std::string candidate(ip_str);
                        if (candidate != "127.0.0.1") {
                            ip = candidate;
                            break;
                        }
                    }
                }
            }
            freeifaddrs(ifaddrs_ptr);
        }
        
        return ip;
    }
    
    int findFreePort() {
        for (int port = BASE_PORT; port < BASE_PORT + MAX_CLIENTS; ++port) {
            int testSocket = socket(AF_INET, SOCK_STREAM, 0);
            if (testSocket < 0) continue;
            
            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = INADDR_ANY;
            addr.sin_port = htons(port);
            
            if (bind(testSocket, (struct sockaddr*)&addr, sizeof(addr)) == 0) {
                close(testSocket);
                return port;
            }
            close(testSocket);
        }
        return -1; // No free port found
    }
    
    bool startServer() {
        serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket < 0) {
            std::cerr << "Error creating server socket" << std::endl;
            return false;
        }
        
        int opt = 1;
        setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        sockaddr_in serverAddr{};
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        serverAddr.sin_port = htons(serverPort);
        
        if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            std::cerr << "Error binding server socket" << std::endl;
            close(serverSocket);
            return false;
        }
        
        if (listen(serverSocket, 5) < 0) {
            std::cerr << "Error listening on server socket" << std::endl;
            close(serverSocket);
            return false;
        }
        
        return true;
    }
    
    void serverLoop() {
        #if DEBUG
            std::cout << "[DEBUG] Server loop started" << std::endl;
        #endif

        while (running) {
            fd_set readfds;
            struct timeval timeout;
            
            FD_ZERO(&readfds);
            FD_SET(serverSocket, &readfds);
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;
            
            int activity = select(serverSocket + 1, &readfds, nullptr, nullptr, &timeout);
            
            if (activity > 0 && FD_ISSET(serverSocket, &readfds)) {
                sockaddr_in clientAddr{};
                socklen_t clientLen = sizeof(clientAddr);
                
                int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);
                if (clientSocket >= 0) {
                    #if DEBUG
                        std::lock_guard<std::mutex> lock(debugMutex);
                        std::cout << "[DEBUG] Accepted new connection on socket " << clientSocket << std::endl;
                    #endif

                    std::thread(&PeerClient::handleConnection, this, clientSocket, clientAddr).detach();
                    
                } else {
                    #if DEBUG
                        std::lock_guard<std::mutex> lock(debugMutex);
                        std::cout << "[DEBUG] Failed to accept connection" << std::endl;
                    #endif
                }
            }
        }
    }
    
    void handleConnection(int clientSocket, sockaddr_in clientAddr) {
        char buffer[BUFFER_SIZE];
       
        ssize_t bytesReceived = recv(clientSocket, buffer, BUFFER_SIZE - 1, 0);
        
        if (bytesReceived > 0) {
            buffer[bytesReceived] = '\0';
            std::string message(buffer);
            
            char clientIP[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &clientAddr.sin_addr, clientIP, INET_ADDRSTRLEN);
            
            {
                std::lock_guard<std::mutex> lock(debugMutex);
                std::cout << "[SERVER] Received message: '" << message << "' from " << clientIP << std::endl;
            }
            
            handleMessage(message, std::string(clientIP), clientSocket);
        } else {
            #if DEBUG
            std::lock_guard<std::mutex> lock(debugMutex);
            std::cout << "[DEBUG] recv() failed with errno: " << errno << " - " << strerror(errno) << std::endl;
            #endif
        }
        
        close(clientSocket);
    }
    
    void handleMessage(const std::string& message, const std::string& senderIP, int responseSocket) {
        std::istringstream iss(message);
        std::string command;
        iss >> command;

        
        #if DEBUG
        {
            std::lock_guard<std::mutex> lock(debugMutex);
            std::cout << "[DEBUG] Handling command: " << command << std::endl;
        }
        #endif

        if (command == "PING") {
            std::string senderPort, senderId;
            iss >> senderPort >> senderId;
            
            int port = std::stoi(senderPort);
            
            // Add peer to list
            {
                std::lock_guard<std::mutex> lock(peersMutex);
                peers[senderId] = PeerInfo(senderIP, port, senderId);
            }
            
            {
                std::lock_guard<std::mutex> lock(debugMutex);
                std::cout << "[DISCOVERY] New peer discovered: " << senderId << " at " << senderIP << ":" << port << std::endl;
            }
            
            // Send PONG response
            std::string response = "PONG " + std::to_string(serverPort) + " " + clientId;
            
            #if DEBUG
            {
                std::lock_guard<std::mutex> lock(debugMutex);
                std::cout << "[SERVER] Sending PONG: '" << response << "'" << std::endl;
            }
            #endif
            send(responseSocket, response.c_str(), response.length(), 0);
            
        } else if (command == "PONG") {
            std::string senderPort, senderId;
            iss >> senderPort >> senderId;
            
            int port = std::stoi(senderPort);
            
            // Add peer to list
            {
                std::lock_guard<std::mutex> lock(peersMutex);
                peers[senderId] = PeerInfo(senderIP, port, senderId);
            }
            
            std::cout << "\n[DISCOVERY] Peer responded: " << senderId << " at " << senderIP << ":" << port << std::endl;
            std::cout << clientId << "> " << std::flush;
            
        } else if (command == "MESSAGE") {
            std::string senderId;
            iss >> senderId;
            
            std::string msgContent;
            std::getline(iss, msgContent);
            if (!msgContent.empty() && msgContent[0] == ' ') {
                msgContent = msgContent.substr(1);
            }
            
            // Add message to queue instead of handling immediately
            {
                std::lock_guard<std::mutex> lock(messageQueueMutex);
                messageQueue.emplace(senderId, msgContent);
            }
            
            std::cout << "\n[MESSAGE] " << senderId << ": " << msgContent << std::endl;
            std::cout << clientId << "> " << std::flush;
            
        } else if (command == "REPLY") {
            std::string senderId;
            iss >> senderId;
            
            std::string replyContent;
            std::getline(iss, replyContent);
            if (!replyContent.empty() && replyContent[0] == ' ') {
                replyContent = replyContent.substr(1);
            }
            
            std::cout << "\n[REPLY] " << senderId << ": " << replyContent << std::endl;
            std::cout << clientId << "> " << std::flush;
        }
    }
    
    void discoveryLoop() {
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(DISCOVERY_INTERVAL));
            if (running) {
                discoverPeers();
                cleanupOldPeers();
            }
        }
    }
    
    void discoverPeers() {
        // Ping all possible ports in range
        #if DEBUG
        {
            std::lock_guard<std::mutex> lock(debugMutex);
            std::cout << "[DEBUG] Starting discovery on IP " << localIP << ", own port: " << serverPort << std::endl;
        }
        #endif
        
        
        for (int port = BASE_PORT; port < BASE_PORT + MAX_CLIENTS; ++port) {
            if (port == serverPort) continue; // Skip own port
            
                #if DEBUG
                {
                    std::lock_guard<std::mutex> lock(debugMutex);
                    std::cout << "[DEBUG] Trying to ping port " << port << std::endl;
                }
                #endif
                
            std::thread(&PeerClient::pingPeer, this, localIP, port).detach();
        }
    }
    
    void pingPeer(const std::string& ip, int port) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {

            #if DEBUG
                std::lock_guard<std::mutex> lock(debugMutex);
                std::cout << "[DEBUG] Failed to create socket for " << ip << ":" << port << std::endl;
            #endif

            return;
        }
        
        sockaddr_in peerAddr{};
        peerAddr.sin_family = AF_INET;
        peerAddr.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &peerAddr.sin_addr);
        
        #if DEBUG
        {
            std::lock_guard<std::mutex> lock(debugMutex);
            std::cout << "[DEBUG] Attempting to connect to " << ip << ":" << port << std::endl;
        }
        #endif

        
        if (connect(sock, (struct sockaddr*)&peerAddr, sizeof(peerAddr)) == 0) {
            #if DEBUG
            {
                std::lock_guard<std::mutex> lock(debugMutex);
                std::cout << "[DEBUG] Connected to " << ip << ":" << port << ", sending PING" << std::endl;
            }
            #endif

            std::string pingMessage = "PING " + std::to_string(serverPort) + " " + clientId;
 
            send(sock, pingMessage.c_str(), pingMessage.length(), 0);
                          
            // Wait for response
            char buffer[BUFFER_SIZE];
            ssize_t received = recv(sock, buffer, BUFFER_SIZE - 1, 0);
            
            if (received > 0) {
                buffer[received] = '\0';                    

                #if DEBUG
                {
                    std::lock_guard<std::mutex> lock(debugMutex);
                    std::cout << "[DEBUG] Received response: " << buffer << std::endl;
                }
                #endif
            
            
            // Parse PONG response directly here
            std::istringstream iss(buffer);
            std::string command, senderPort, senderId;
            iss >> command >> senderPort >> senderId;
            
            if (command == "PONG") {
                int peerPort = std::stoi(senderPort);
                
                // Add peer to list
                {
                    std::lock_guard<std::mutex> lock(peersMutex);
                    peers[senderId] = PeerInfo(ip, peerPort, senderId);
                }
                
                {
                    std::lock_guard<std::mutex> lock(debugMutex);
                    std::cout << "[DISCOVERY] Peer responded: " << senderId << " at " << ip << ":" << peerPort << std::endl;
                }
            } else {
                #if DEBUG
                {
                    std::lock_guard<std::mutex> lock(debugMutex);
                    std::cout << "[DEBUG] No response received from " << ip << ":" << port << std::endl;
                }
                #endif
            }
        } else {
            #if DEBUG
            {
                std::lock_guard<std::mutex> lock(debugMutex);
                std::cout << "[DEBUG] Failed to connect to " << ip << ":" << port << " (probably no peer there)" << std::endl;
            }
            #endif
        }
        
        close(sock);
    }
    }
    
    void sendMessageToAll(const std::string& message) {
        // std::lock_guard<std::mutex> lock(peersMutex);
        
        if (peers.empty()) {
            std::cout << "No peers connected." << std::endl;
            return;
        }
        
        std::cout << "Broadcasting message to " << peers.size() << " peers..." << std::endl;
        
        for (const auto& peer : peers) {
            sendMessageToPeer(peer.first, "MESSAGE " + clientId + " " + message);
        }
    }
    
    void sendMessageToPeer(const std::string& peerId, const std::string& message) {
        std::lock_guard<std::mutex> lock(peersMutex);
        
        auto it = peers.find(peerId);
        if (it == peers.end()) {
            return;
        }
        
        const PeerInfo& peer = it->second;

        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) return;

        sockaddr_in peerAddr{};
        peerAddr.sin_family = AF_INET;
        peerAddr.sin_port = htons(peer.port);
        inet_pton(AF_INET, peer.ip.c_str(), &peerAddr.sin_addr);
        
        if (connect(sock, (struct sockaddr*)&peerAddr, sizeof(peerAddr)) == 0) {
            send(sock, message.c_str(), message.length(), 0);
            #if DEBUG
            {
                std::lock_guard<std::mutex> lock(debugMutex);
                std::cout << "[DEBUG] Sent message to " << peer.clientId << ": '" << message << "'" << std::endl;
            }
            #endif
        }
        
        close(sock);
    }
    
    void listPeers() {
        std::lock_guard<std::mutex> lock(peersMutex);
        
        std::cout << "\n=== Connected Peers ===" << std::endl;
        std::cout << "Own ID: " << clientId << " at " << localIP << ":" << serverPort << std::endl;
        
        if (peers.empty()) {
            std::cout << "No other peers connected." << std::endl;
        } else {
            for (const auto& peer : peers) {
                auto timeSince = std::chrono::steady_clock::now() - peer.second.lastSeen;
                auto seconds = std::chrono::duration_cast<std::chrono::seconds>(timeSince).count();
                
                std::cout << "  " << peer.second.clientId 
                         << " at " << peer.second.ip << ":" << peer.second.port
                         << " (last seen " << seconds << "s ago)" << std::endl;
            }
        }
        std::cout << "===================" << std::endl;
    }
    
    void cleanupOldPeers() {
        std::lock_guard<std::mutex> lock(peersMutex);
        
        auto now = std::chrono::steady_clock::now();
        auto it = peers.begin();
        
        while (it != peers.end()) {
            auto timeSince = now - it->second.lastSeen;
            if (std::chrono::duration_cast<std::chrono::seconds>(timeSince).count() > 30) {
                std::cout << "\n[CLEANUP] Removing inactive peer: " << it->second.clientId << std::endl;
                std::cout << clientId << "> " << std::flush;
                it = peers.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    void showPendingMessages() {
        std::lock_guard<std::mutex> lock(messageQueueMutex);
        
        if (messageQueue.empty()) {
            std::cout << "No pending messages." << std::endl;
            return;
        }
        
        std::cout << "\n=== Pending Messages ===" << std::endl;
        auto queueCopy = messageQueue; // Copy to avoid modifying while iterating
        int index = 1;
        
        while (!queueCopy.empty()) {
            const auto& msg = queueCopy.front();
            auto timeSince = std::chrono::steady_clock::now() - msg.timestamp;
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(timeSince).count();
            
            std::cout << index++ << ". From " << msg.senderId << " (" << seconds << "s ago): " << msg.content << std::endl;
            queueCopy.pop();
        }
        std::cout << "========================" << std::endl;
    }
    
    void replyToMessage(const std::string& senderId) {
        IncomingMessage targetMessage("", "");
        bool found = false;
        
        // Find and remove message from queue
        {
            std::lock_guard<std::mutex> lock(messageQueueMutex);
            std::queue<IncomingMessage> tempQueue;
            
            while (!messageQueue.empty()) {
                auto msg = messageQueue.front();
                messageQueue.pop();
                
                if (!found && msg.senderId == senderId) {
                    targetMessage = msg;
                    found = true;
                } else {
                    tempQueue.push(msg);
                }
            }
            
            messageQueue = tempQueue; // Restore queue without the replied message
        }
        
        if (!found) {
            std::cout << "No message found from " << senderId << std::endl;
            return;
        }
        
        std::cout << "Original message: " << targetMessage.content << std::endl;
        std::cout << "Your reply: ";
        
        std::string reply;
        if (std::getline(std::cin, reply)) {
            sendMessageToPeer(senderId, "REPLY " + clientId + " " + reply);
            std::cout << "Reply sent to " << senderId << std::endl;
        }
    }
};

int main(int argc, char* argv[]) {
    std::string clientId;
    if (argc > 1) {
        clientId = argv[1];
    }
    
    PeerClient client(clientId);
    
    if (!client.start()) {
        std::cerr << "Failed to start peer client!" << std::endl;
        return 1;
    }
    
    client.run();
    
    std::cout << "Shutting down..." << std::endl;
    return 0;
}
