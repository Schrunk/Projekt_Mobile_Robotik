#ifndef BOUNCE_STATE_HPP
#define BOUNCE_STATE_HPP

#include "state.hpp"
#include "../statemachine.hpp"
#include "bounceState.cpp"


class BounceState : public State {
public:

    BounceState(StateMachine *stateMachine);
    
    ~BounceState() override = default;

    void onEnter() override;

    void run() override;

    void onExit() override;

    const char* getName() const override;

private:
    StateMachine *_stateMachine;
};

#endif // BOUNCE_STATE_HPP