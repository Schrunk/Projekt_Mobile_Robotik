#ifndef IDLESTATE_HPP
#define IDLESTATE_HPP

#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"
#include "irobot_create_msgs/msg/interface_buttons.hpp"

#include "state.hpp"
#include "../turtlebot4.cpp"
#include "../statemachine.hpp"

// Idle State - Robot is stationary, waiting for commands
class IdleState : public State, public rclcpp::Node {
public:
    // Constructor
    explicit IdleState(StateMachine *stateMachine);

    // Destructor
    ~IdleState() override = default;

    // called when entering the idle state
    // setup idle behavior and monitoring
    void onEnter() override;

    // main execution loop for idle state
    void run() override;

    // called when exiting the idle state
    // cleanup idle resources
    void onExit() override;

    // get state name for debugging
    const char* getName() const override;

private:
    rclcpp::Publisher<irobot_create_msgs::msg::LightringLeds>::SharedPtr _lightringPublisher;
    rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr _buttonSubscription;

    LightringColor _currentColor;

    std::chrono::steady_clock::time_point _start;
};

#endif // IDLESTATE_HPP