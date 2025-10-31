#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <memory>
#include <unordered_map>
#include <functional>
#include "states/state.hpp"

// enumeration of all available states
enum class StateType {
    INIT_STATE,
    IDLE,
    BOUNCE,
    DRIVE,
    BACK_TO_START,
    ERROR,
    SETUP
};

struct Line {
    float a;
    float b;
    float c;
};

// State Machine Class
class StateMachine {
public:
    // constructor - initializes the state machine
    StateMachine();

    // destructor - ensures proper cleanup
    ~StateMachine();

    // initialize the state machine with all states
    void initialize();

    // run one iteration of the state machine
    void update();

    // transition to a new state
    void transitionTo(StateType newState);

    // get the current state type
    StateType getCurrentStateType() const;

    // get a raw pointer to the current state (may be a Node-derived instance)
    State* getCurrentState() const { return _currentState; }

    // get line references
    void getLineReferences(float &a1, float &b1, float &c1,
                           float &a2, float &b2, float &c2) const;

    // check if the state machine is running
    bool isRunning() const;

    // set postion references
    void setLineReference(float a1, float b1, float c1,
                               float a2, float b2, float c2);

    // stop the state machine
    void stop();

    // register a callback invoked whenever a transition completes switching current state
    void setOnStateChanged(std::function<void(State* oldState, State* newState)> cb) { _onStateChanged = std::move(cb); }

    // set scores
    void increaseScore(int player);

private:
    // member variables
    std::unordered_map<StateType, std::unique_ptr<State>> _states;
    State *_currentState;
    StateType _currentStateType;
    bool _isRunning;
    bool _isInitialized;
    std::function<void(State* oldState, State* newState)> _onStateChanged;

    // line references
    // f(x,y) = a*x + b*y + c = 0
    // if F(x,y) changes sign, robot crossed the line
    float _a1;
    float _b1;
    float _c1;

    float _a2;
    float _b2;
    float _c2;

    // score counter
    int _score1{0};
    int _score2{0};

    // register a state with the state machine
    void registerState(StateType stateType, std::unique_ptr<State> state);

};

#endif // STATEMACHINE_HPP