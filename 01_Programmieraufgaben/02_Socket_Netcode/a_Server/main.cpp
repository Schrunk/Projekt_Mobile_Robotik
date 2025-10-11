#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#define PORT 8080

bool handle_client(int client_socket) {
    char buffer[1024] = {0};
    ssize_t bytes_received = recv(client_socket, buffer, sizeof(buffer), 0);
    if (bytes_received > 0) {
        std::cout << "Empfangen: " << buffer << std::endl;
        send(client_socket, buffer, sizeof(buffer), 0);
    }
    close(client_socket);
}

int main(int argc, char* argv[]) {
    int server_fd, client_socket;
    struct sockaddr_in address;
    socklen_t addr_len = sizeof(address);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        std::cerr << "Socket konnte nicht erstellt werden\n";
        return -1;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "Bind fehlgeschlagen\n";
        return -1;
    }

    if (listen(server_fd, 3) < 0) {
        std::cerr << "Listen fehlgeschlagen\n";
        return -1;
    }

    bool running = true;
    while (running) {
        std::cout << " Warte auf Verbindung...\n";

        client_socket = accept(server_fd, (struct sockaddr*)&address, &addr_len);
        if (client_socket >= 0) {
            running = handle_client(client_socket);
        } else {
            std::cerr << "Verbindungsaufnahme fehlgeschlagen\n";
        }
    }

    close(server_fd);
    return 0;
}
