#ifndef TERMINALPUB_CPP
#define TERMINALPUB_CPP

#include <atomic>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/logging.hpp"
#include "terminalPub.hpp"

TerminalPublisher::TerminalPublisher() : Node("terminalPublisher") {
    // publisher for user input
    _inputPub = this->create_publisher<std_msgs::msg::String>("/userInput", 10);
    _inputThread = std::thread([this]() {this->publishInput(); });
}

TerminalPublisher::~TerminalPublisher() {
    // stop input thread
    _running = false;
    if (_inputThread.joinable()) {
        _inputThread.join();
    }
}

void TerminalPublisher::publishInput() {
    // continuously read user input and publish
    std::string input;
    while (rclcpp::ok() && _running) {
        std::cout << "> ";
        std::getline(std::cin, input);

        auto message = std_msgs::msg::String();
        message.data = input;
        _inputPub->publish(message);

        RCLCPP_INFO(this->get_logger(), "Published user input: %s", input.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _inputPub;
    std::thread _inputThread;
    std::atomic<bool> _running{true};
}

#endif // TERMINALPUB_CPP