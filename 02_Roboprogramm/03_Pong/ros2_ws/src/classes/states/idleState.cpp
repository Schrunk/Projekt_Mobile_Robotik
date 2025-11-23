#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "idleState.hpp"
#include "../statemachine.hpp"

// constructor
IdleState::IdleState(StateMachine* stateMachine)
    : State(stateMachine), rclcpp::Node("idleState"){
}


void IdleState::onEnter() {
    RCLCPP_DEBUG(this->get_logger(), "Entering Idle State");

    // button subscriber
    _buttonSubscription = this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
      "/interface_buttons", 10,
      [this](const irobot_create_msgs::msg::InterfaceButtons::SharedPtr msg) {
          // Check if button 2 is pressed
          if (msg->button_2.is_pressed) {
              RCLCPP_DEBUG(this->get_logger(), "Button 2 pressed - transitioning to Drive State");
              _stateMachine->transitionTo(StateType::DRIVE);
          }
    });

    _terminalSubscription = this->create_subscription<std_msgs::msg::String>(
        "/app/userInput", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            receiveUserInput(msg);
    });

    _terminalInput.clear();
}

void IdleState::run() {
    // set LED ring to blue to indicate missing setup
    RCLCPP_INFO_ONCE(this->get_logger(), "Ready to start a game. Press Button 1 or enter \"start\" to begin.");

    // check for terminal input to start the game
    if (_terminalInput == "start") {
        RCLCPP_INFO(this->get_logger(), "Let's start the game!");
        _stateMachine->transitionTo(StateType::DRIVE);
        _terminalInput.clear();
    } else if (_terminalInput == "reset") {
        RCLCPP_INFO(this->get_logger(), "Reset command received");
        _stateMachine->transitionTo(StateType::BACK_TO_START);
        _terminalInput.clear();
    }
}

void IdleState::onExit() {    
    // TODO: Disable idle monitoring
    // TODO: Prepare for active state
    RCLCPP_DEBUG(this->get_logger(), "Exiting Idle State");
    _buttonSubscription.reset();
    _terminalSubscription.reset();
}

const char* IdleState::getName() const {
    return "IdleState";
}

void IdleState::receiveUserInput(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received user input: %s", msg->data.c_str());
    _terminalInput = msg->data;
}