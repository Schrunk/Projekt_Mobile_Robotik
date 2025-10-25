#ifndef BACKTOSTARTSTATE_CPP
#define BACKTOSTARTSTATE_CPP

#include "backToStartState.hpp"
#include "../statemachine.hpp"


BackToStartState::BackToStartState(StateMachine* stateMachine) 
    : State(stateMachine) {
}

void BackToStartState::onEnter() {

    
    // TODO: Get current position
    // TODO: Calculate route to home position
    // TODO: Initialize return navigation parameters
}

void BackToStartState::run() {

}

void BackToStartState::onExit() {

}

const char* BackToStartState::getName() const {
    return "BackToStartState";
}

#endif // BACKTOSTARTSTATE_CPP