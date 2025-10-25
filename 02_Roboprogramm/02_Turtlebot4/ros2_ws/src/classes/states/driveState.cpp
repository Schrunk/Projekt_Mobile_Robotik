#ifndef DRIVESTATE_CPP
#define DRIVESTATE_CPP

#include "driveState.hpp"
#include "../statemachine.hpp"


DriveState::DriveState(StateMachine *stateMachine) 
    : State(stateMachine) {
}

void DriveState::onEnter() {

}

void DriveState::run() {

}

void DriveState::onExit() {

}

const char* DriveState::getName() const {
    return "DriveState";
}

#endif // DRIVESTATE_CPP