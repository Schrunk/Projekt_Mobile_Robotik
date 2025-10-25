#ifndef INITSTATE_HPP
#define INITSTATE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "state.hpp"
#include "../statemachine.hpp"

// Initialization State Class
class InitState : public State, public rclcpp::Node {
public:
    // constructor
    explicit InitState(StateMachine *stateMachine);

    // destructor
    ~InitState() override = default;

    // called when entering the init state
    void onEnter() override;

    // main execution method for the init state
    void run() override;

    // called when exiting the init state
    void onExit() override;

    // get the name of this state
    const char* getName() const override;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _inputSubscription;

    void receiveUserInput(const std_msgs::msg::String::SharedPtr msg);
};

#endif // INITSTATE_HPP