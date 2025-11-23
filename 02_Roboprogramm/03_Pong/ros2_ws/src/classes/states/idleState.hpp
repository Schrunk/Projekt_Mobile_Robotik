#ifndef IDLESTATE_HPP
#define IDLESTATE_HPP

#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"
#include "irobot_create_msgs/msg/interface_buttons.hpp"

#include "state.hpp"
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
    // terminal input subscription
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _terminalSubscription;
    std::string _terminalInput;

    void receiveUserInput(const std_msgs::msg::String::SharedPtr msg);

    // button subscription    
    rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr _buttonSubscription;
    bool _button{false};

    std::chrono::steady_clock::time_point _start;

    
};

#endif // IDLESTATE_HPP