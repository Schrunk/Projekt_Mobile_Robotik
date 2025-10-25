#ifndef BOUNCESTATE_HPP
#define BOUNCESTATE_HPP

#include "bounceState.hpp"
#include "../statemachine.hpp"


BounceState::BounceState(StateMachine *stateMachine) 
    : State(stateMachine) {
}

void BounceState::onEnter() {

    
    // TODO: Stop current movement
    // TODO: Activate obstacle avoidance sensors
}

void BounceState::run() {

}

void BounceState::onExit() {
    // TODO: Stop any ongoing bounce movements
    // TODO: Reset obstacle avoidance sensors to normal mode
    // TODO: Clear obstacle flags
}

const char* BounceState::getName() const {
    return "BounceState";
}

#endif // BOUNCESTATE_HPP