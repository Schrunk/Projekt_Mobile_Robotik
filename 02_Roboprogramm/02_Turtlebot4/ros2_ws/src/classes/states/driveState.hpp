#ifndef DRIVE_STATE_HPP
#define DRIVE_STATE_HPP

#include "state.hpp"
#include "../statemachine.hpp"
#include "driveState.cpp"


class DriveState : public State {
public:

    explicit DriveState(StateMachine* stateMachine);
    
    ~DriveState() override = default;

 
    void onEnter() override;

 
    void run() override;

 
    void onExit() override;

 
    const char* getName() const override;

private:
};

#endif // DRIVE_STATE_HPP