#include <functional>
#include "initState.hpp"

// constructor
InitState::InitState(StateMachine *stateMachine)
    : State(stateMachine), rclcpp::Node("init_state") {
}

// called when entering the init state
void InitState::onEnter() {
    RCLCPP_INFO(this->get_logger(), "Entering Init State");
    _inputSubscription = this->create_subscription<std_msgs::msg::String>(
        "/userInput", 10,
        std::bind(&InitState::receiveUserInput, this, std::placeholders::_1)
    );
}

// main execution loop for init state (non-blocking)
void InitState::run() {
    // No blocking work here; spinning is managed by the executor
}

// called when exiting the init state
void InitState::onExit() {
    // Cleanup if needed
}

// get state name for debugging
const char* InitState::getName() const {
    return "InitState";
}

// receive user input during init state
void InitState::receiveUserInput(const std_msgs::msg::String::SharedPtr msg) {
    const std::string &input = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received user input: %s", input.c_str());
    if (input == "IDLE") {
        _stateMachine->transitionTo(StateType::IDLE);
        RCLCPP_INFO(this->get_logger(), "Transitioning to Idle State");
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown command in Init State: %s", input.c_str());
    }
}