#include "idleState.hpp"
#include "../statemachine.hpp"
#include "../turtlebot4.hpp"


IdleState::IdleState(StateMachine* stateMachine)
    : State(stateMachine), rclcpp::Node("idle_state"){
}

void IdleState::onEnter() {
    // set LED ring
    

}

void IdleState::run() {
    // set LED ring to blue to indicate missing setup
    
}

void IdleState::onExit() {    
    // TODO: Disable idle monitoring
    // TODO: Prepare for active state
}

const char* IdleState::getName() const {
    return "IdleState";
}