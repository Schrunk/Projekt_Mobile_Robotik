#ifndef BACK_TO_START_STATE_HPP
#define BACK_TO_START_STATE_HPP

#include "state.hpp"
#include "../statemachine.hpp"
#include "backToStartState.cpp"


class BackToStartState : public State {
public:

    BackToStartState(StateMachine* stateMachine);
    

    ~BackToStartState() override = default;

 
    void onEnter() override;


    void run() override;

 
    void onExit() override;


    const char* getName() const override;

private:
};

#endif // BACK_TO_START_STATE_HPP