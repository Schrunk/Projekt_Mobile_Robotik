#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "std_msgs/msg/string.hpp"

#include "initState.hpp"
#include "../statemachine.hpp"


// constructor
InitState::InitState(StateMachine *stateMachine)
    : State(stateMachine), rclcpp::Node("init_state") {
}

// called when entering the init state
void InitState::onEnter() {
    RCLCPP_DEBUG(this->get_logger(), "Entering Init State");

    // button subscription on button 2
    _buttonSubscription = this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
        "/interface_buttons", 10,
        [this](const irobot_create_msgs::msg::InterfaceButtons::SharedPtr msg) {
            buttonPressed(msg);
    });

    // position subscriber (read current pose from /pose as PoseWithCovarianceStamped)
    _positionSubscription = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/pose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            _yPos = msg->pose.pose.position.y;
            _xPos = msg->pose.pose.position.x;
            _yawPos = static_cast<float>(quaternionToYaw(*msg));
            RCLCPP_DEBUG(this->get_logger(), "Current Position - x: %.2f, y: %.2f",
                        _xPos, _yPos);
    });
}

// main execution loop for init state (non-blocking)
void InitState::run() {
    // switch state init steps
    switch (_initSteps) {
        case 0: {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Set Roboter to initial position. Press Button 2 to continue.");

            if (_button) {
                RCLCPP_DEBUG(this->get_logger(), "Button 2 detected in Init State, proceeding to next step.");
                // Capture current pose as start pose in the state machine (dynamic start)
                _stateMachine->setStartPose(static_cast<double>(_xPos), static_cast<double>(_yPos), static_cast<double>(_yawPos));
                RCLCPP_INFO(this->get_logger(), "Start pose stored: x=%.2f y=%.2f yaw=%.2f deg", _xPos, _yPos, _yawPos * 180.0 / M_PI);

                _initSteps++;
                _button = false; // reset button flag
            }
            break;
        }

        case 1: {
            RCLCPP_INFO_ONCE(this->get_logger(),
                                 "Waiting for robot to reach corner position position 1.1. Press Button 2 to confirm.");

            if (_button) {
                RCLCPP_DEBUG(this->get_logger(), "Button 2 detected in Init State, setting corner position 1.1.");

                // Set corner position 1.1
                setPositionReference(_xPosLine11, _yPosLine11);

                _initSteps++;
                _button = false; // reset button flag
            }
            break;
        }
        case 2: {
            RCLCPP_INFO_ONCE(this->get_logger(),
                                 "Waiting for robot to reach corner position 1.2. Press Button 2 to confirm.");
            if (_button) {
                RCLCPP_DEBUG(this->get_logger(), "Button 2 detected in Init State, setting corner position 1.2.");

                // Set corner position 1.2
                setPositionReference(_xPosLine12, _yPosLine12);

                _initSteps++;
                _button = false; // reset button flag
            }
            break;
        }

        case 3: {
            RCLCPP_INFO_ONCE(this->get_logger(),
                                 "Waiting for robot to reach corner position 2.1. Press Button 2 to confirm.");
            if (_button) {
                RCLCPP_DEBUG(this->get_logger(), "Button 2 detected in Init State, setting corner position 2.1.");

                // Set corner position 2.1
                setPositionReference(_xPosLine21, _yPosLine21);

                _initSteps++;
                _button = false; // reset button flag
            }
            break;
        }

        case 4: {
            RCLCPP_INFO_ONCE(this->get_logger(),
                "Waiting for robot to reach corner position 2.2. Press Button 2 to confirm.");
            if (_button) {
                RCLCPP_DEBUG(this->get_logger(), "Button 2 detected in Init State, setting corner position 2.2.");

                // Set corner position 2.2
                setPositionReference(_xPosLine22, _yPosLine22);

                _initSteps++;
                _button = false; // reset button flag
            }
            break;
        }

        case 5: {
            RCLCPP_DEBUG(this->get_logger(), "Initialization complete. Calculate goal lines.");

            // set position references in state machine
            // a = y2 - y1 ; b = -(x2 - x1) ; c = x2*y1 - y2*x1
            float a1 = _yPosLine12 - _yPosLine11;
            float b1 = -(_xPosLine12 - _xPosLine11);
            float c1 = _xPosLine12 * _yPosLine11 - _yPosLine12 * _xPosLine11;

            float a2 = _yPosLine22 - _yPosLine21;
            float b2 = -(_xPosLine22 - _xPosLine21);
            float c2 = _xPosLine22 * _yPosLine21 - _yPosLine22 * _xPosLine21;

            _stateMachine->setLineReference(a1, b1, c1, a2, b2, c2);
            _initSteps++;
            break;
        }

        case 6: {
            RCLCPP_INFO(this->get_logger(), "Initialization finished.");
            _stateMachine->transitionTo(StateType::BACK_TO_START);
            break;
        }

        default:
            RCLCPP_WARN(this->get_logger(), "Init State in unknown step!");
            break;
    }
}

// called when exiting the init state
void InitState::onExit() {
    // Cleanup if needed
    RCLCPP_DEBUG(this->get_logger(), "Exiting Init State");
    _buttonSubscription.reset();
    _positionSubscription.reset();
    
}

// get state name for debugging
const char* InitState::getName() const {
    return "InitState";
}

// handle button press during init state
void InitState::buttonPressed(const irobot_create_msgs::msg::InterfaceButtons::SharedPtr msg) {
    // Check if button 2 is pressed
    if (msg->button_2.is_pressed) {
        RCLCPP_DEBUG(this->get_logger(), "Button 2 pressed");
        _button = true;
    }
}

// set position references
void InitState::setPositionReference(float &xRef, float &yRef) {
    xRef = _xPos;
    yRef = _yPos;

    RCLCPP_DEBUG(this->get_logger(), "Position reference set to x: %.2f, y: %.2f", xRef, yRef);
}

// overload for PoseWithCovarianceStamped
double InitState::quaternionToYaw(const geometry_msgs::msg::PoseWithCovarianceStamped &msg) {
    const auto & q = msg.pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}