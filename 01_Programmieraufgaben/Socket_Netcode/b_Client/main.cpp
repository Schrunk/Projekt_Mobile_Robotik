#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

/*
    * Simple TCP client that connects to a server and sends user input.
    * Usage: <name> <IP> <PORT>
*/
int main(int argc, char* argv[]) {
    // Check for correct number of arguments -> name address port
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <IP> <PORT>\n";
        return 1;
    } else {
        server_ip

        u_short server_port = static_cast<u_short>(std::stoi(argv[2]));

    }

    // if (argv[1] == "localhost") {
    //     argv[1] = "12
    // }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);

    if (inet_pton(AF_INET, server_ip, &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address/ Address not supported\n";
        close(sock);
        return 1;
    }

    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Connection Failed\n";
        close(sock);
        return 1;
    }

    std::cout << "Connected to " << server_ip << ":" << server_port << "\n";
    std::string input;
    while (true) {
        std::cout << "> ";
        if (!std::getline(std::cin, input)) break;
        if (input == "exit") break;
        ssize_t sent = send(sock, input.c_str(), input.size(), 0);
        if (sent < 0) {
            std::cerr << "Send failed\n";
            break;
        }
    }

    close(sock);
    std::cout << "Connection closed.\n";
    return 0;
}