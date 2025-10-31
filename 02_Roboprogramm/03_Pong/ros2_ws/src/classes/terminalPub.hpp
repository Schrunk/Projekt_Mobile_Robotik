#ifndef TERMINALPUB_HPP
#define TERMINALPUB_HPP

#include "rclcpp/rclcpp.hpp"

class TerminalPublisher : public rclcpp::Node {
public:
    TerminalPublisher();

    ~TerminalPublisher();

private:
    void publishInput();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _inputPub;
    std::thread _inputThread;
    std::atomic<bool> _running{true};
};

#endif // TERMINALPUB_HPP