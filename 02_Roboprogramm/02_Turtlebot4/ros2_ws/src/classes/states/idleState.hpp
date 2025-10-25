#ifndef IDLESTATE_HPP
#define IDLESTATE_HPP

#include "rclcpp/rclcpp.hpp"
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

};

#endif // IDLESTATE_HPP