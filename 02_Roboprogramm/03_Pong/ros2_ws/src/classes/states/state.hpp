#ifndef STATE_HPP
#define STATE_HPP

#include "rclcpp/rclcpp.hpp"

// forward declaration to avoid circular dependency
class StateMachine;

// abstract base state class following the State Pattern
class State {
public:
    virtual ~State() = default;

    // abstract function called when entering this state
    virtual void onEnter() = 0;

    // main execution method called repeatedly while in this state
    virtual void run() = 0;

    // called when exiting this state
    virtual void onExit() = 0;

    // get the name of this state for debugging purposes
    virtual const char* getName() const = 0;

protected:
    // reference to the state machine for state transitions
    StateMachine* _stateMachine;

    // constructor - only accessible to derived classes
    explicit State(StateMachine* stateMachine) : _stateMachine(stateMachine) {};
};

#endif // STATE_HPP