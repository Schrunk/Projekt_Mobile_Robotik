#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "idleState.hpp"
#include "../statemachine.hpp"
#include "../turtlebot4.cpp"

// constructor
IdleState::IdleState(StateMachine* stateMachine)
    : State(stateMachine), rclcpp::Node("idleState"){
}


void IdleState::onEnter() {
    RCLCPP_INFO(this->get_logger(), "Entering Idle State");

    // create LED ring publisher
    _lightringPublisher = this->create_publisher<irobot_create_msgs::msg::LightringLeds>(
      "/cmd_lightring", rclcpp::SensorDataQoS());

    _lightringPublisher->publish(createLightringMessage(LightringColor::BLUE, this->get_clock()->now()));
    _currentColor = LightringColor::BLUE;

    // button subscriber
    _buttonSubscription = this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
      "/sensor_interface_buttons", rclcpp::SensorDataQoS(),
      [this](const irobot_create_msgs::msg::InterfaceButtons::SharedPtr msg) {
          // Check if button 1 is pressed
          if (msg->button_1.is_pressed) {
              RCLCPP_INFO(this->get_logger(), "Button 1 pressed - transitioning to Drive State");
              _stateMachine->transitionTo(StateType::DRIVE);
          }
      });

    _start = std::chrono::steady_clock::now();
}

void IdleState::run() {
    // set LED ring to blue to indicate missing setup
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Setting lightring to BLUE in Idle State");

    auto now = std::chrono::steady_clock::now();

    if (_currentColor == LightringColor::BLUE && ((now - _start) >= std::chrono::milliseconds(500))) {
        _lightringPublisher->publish(createLightringMessage(LightringColor::OFF, this->get_clock()->now()));
        _start = std::chrono::steady_clock::now();

    } else {
        _lightringPublisher->publish(createLightringMessage(LightringColor::BLUE, this->get_clock()->now()));
        _start = std::chrono::steady_clock::now();
    }
}

void IdleState::onExit() {    
    // TODO: Disable idle monitoring
    // TODO: Prepare for active state
    RCLCPP_INFO(this->get_logger(), "Exiting Idle State");
    _lightringPublisher.reset();
    _buttonSubscription.reset();
}

const char* IdleState::getName() const {
    return "IdleState";
}