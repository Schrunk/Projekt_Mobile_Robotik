#include "statemachine.hpp"
#include "states/initState.hpp"
#include "states/idleState.hpp"
// #include "states/bounceState.hpp"
// #include "states/driveState.hpp"
// #include "states/backToStartState.hpp"
// #include "states/errorState.hpp"
// #include "states/setupState.hpp"


// State Machine Constructor
StateMachine::StateMachine() 
    : _currentState(nullptr), _currentStateType(StateType::INIT_STATE), 
    _isRunning(false), _isInitialized(false) {
}


// State Machine Destructor
StateMachine::~StateMachine() {
    if (_currentState) {
        _currentState->onExit();
    }
    _states.clear();
}


// initialize the state machine with all states
void StateMachine::initialize() {
    // register all states
    registerState(StateType::INIT_STATE, std::make_unique<InitState>(this));
    registerState(StateType::IDLE, std::make_unique<IdleState>(this));
    // registerState(StateType::SETUP, std::make_unique<SetupState>(this));
    // registerState(StateType::BOUNCE, std::make_unique<BounceState>(this));
    // registerState(StateType::DRIVE, std::make_unique<DriveState>(this));
    // registerState(StateType::BACK_TO_START, std::make_unique<BackToStartState>(this));
    // registerState(StateType::ERROR, std::make_unique<ErrorState>(this));
    
    // set initial state
    _currentState = _states[StateType::INIT_STATE].get();
    _currentStateType = StateType::INIT_STATE;

    // enter initial state
    if (_currentState) {
        _currentState->onEnter();
    }
    
    _isInitialized = true;
    _isRunning = true;
}


// run one iteration of the state machine
void StateMachine::update() {
    if (!_isInitialized || !_isRunning || !_currentState) {
        return;
    }
    
    // execute current state
    _currentState->run();
}


// transition to a new state
void StateMachine::transitionTo(StateType newState) {
    // check not initialized or edge case: unnecessary transition
    if (!_isInitialized || newState == _currentStateType) {
        return;
    }
    
    // find the new state    
    auto it = _states.find(newState);

    // state not found
    if (it == _states.end()) {
        return;
    }
    
    // exit current state
    State* oldState = _currentState;
    if (oldState) {
        oldState->onExit();
    }
    
    // switch to new state
    _currentState = it->second.get();
    _currentStateType = newState;
    
    // notify listeners so they can add/remove nodes from executors, etc.
    if (_onStateChanged) {
        _onStateChanged(oldState, _currentState);
    }
    
    // enter new state
    _currentState->onEnter();
}


// get the current state type
StateType StateMachine::getCurrentStateType() const {
    return _currentStateType;
}


// get the current state name for debugging
// const char* StateMachine::getCurrentStateName() const
// {
//     return _currentState ? _currentState->getName() : "UNKNOWN";
// }


// check if the state machine is running
bool StateMachine::isRunning() const {
    return _isRunning;
}


// stop the state machine
void StateMachine::stop() {
    _isRunning = false;

    if (_currentState) {
        _currentState->onExit();
    }
}


// register a state with the state machine
void StateMachine::registerState(StateType stateType, std::unique_ptr<State> state) {
    _states[stateType] = std::move(state);
}
