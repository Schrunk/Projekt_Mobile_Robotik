#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <memory>
#include <unordered_map>
#include <functional>
#include "states/state.hpp"
#include "turtlebot4.hpp"

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

    // check if the state machine is running
    bool isRunning() const;

    // stop the state machine
    void stop();

    TurtleBot4 *_puk;

    // register a callback invoked whenever a transition completes switching current state
    void setOnStateChanged(std::function<void(State* oldState, State* newState)> cb) { _onStateChanged = std::move(cb); }
    
private:
    // member variables
    std::unordered_map<StateType, std::unique_ptr<State>> _states;
    State *_currentState;
    StateType _currentStateType;
    bool _isRunning;
    bool _isInitialized;
    std::function<void(State* oldState, State* newState)> _onStateChanged;

    // register a state with the state machine
    void registerState(StateType stateType, std::unique_ptr<State> state);
};

#endif // STATEMACHINE_HPP