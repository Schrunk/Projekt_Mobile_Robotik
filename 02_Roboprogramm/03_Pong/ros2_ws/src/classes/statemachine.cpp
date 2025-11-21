#include "statemachine.hpp"
#include "states/initState.hpp"
#include "states/idleState.hpp"
#include "states/bounceState.hpp"
#include "states/driveState.hpp"
#include "states/backToStartState.hpp"
// #include "states/errorState.hpp"


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
    registerState(StateType::INIT_STATE, std::make_shared<InitState>(this));
    registerState(StateType::IDLE, std::make_shared<IdleState>(this));
    registerState(StateType::BOUNCE, std::make_shared<BounceState>(this));
    registerState(StateType::DRIVE, std::make_shared<DriveState>(this));
    registerState(StateType::BACK_TO_START, std::make_shared<BackToStartState>(this));
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

// get line references
void StateMachine::getLineReferences(float& a1, float& b1, float& c1,
                                     float& a2, float& b2, float& c2) const {
    a1 = _a1;
    b1 = _b1;
    c1 = _c1;
    a2 = _a2;
    b2 = _b2;
    c2 = _c2;

    return;
}

// check if the state machine is running
bool StateMachine::isRunning() const {
    return _isRunning;
}

// set line references
void StateMachine::setLineReference(float a1, float b1, float c1,
                                       float a2, float b2, float c2) {
    _a1 = a1;
    _b1 = b1;
    _c1 = c1;

    _a2 = a2;
    _b2 = b2;
    _c2 = c2;

    return;
}

// stop the state machine
void StateMachine::stop() {
    _isRunning = false;

    if (_currentState) {
        _currentState->onExit();

    }
}

// set scores
void StateMachine::increaseScore(int player) {
    if (player == 1) {
        _score1++;
    } else if (player == 2) {
        _score2++;
    }
}

// register a state with the state machine
void StateMachine::registerState(StateType stateType, std::shared_ptr<State> state) {
    _states[stateType] = std::move(state);
}

// set start pose
void StateMachine::setStartPose(double x, double y, double yaw) {
    _startX = x;
    _startY = y;
    _startYaw = yaw;
    _startPoseSet = true;
}

// get start pose
bool StateMachine::getStartPose(double &x, double &y, double &yaw) const {
    if (!_startPoseSet) {
        return false;
    }
    x = _startX;
    y = _startY;
    yaw = _startYaw;
    return true;
}