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
    RCLCPP_INFO(this->get_logger(), "Entering Init State");
    _inputSubscription = this->create_subscription<std_msgs::msg::String>(
        "/app/userInput", 10,
        std::bind(&InitState::receiveUserInput, this, std::placeholders::_1)
    );

    // button subscription on button 2
    _buttonSubscription = this->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
        "/sensor_interface_buttons", 10,
        [this](const irobot_create_msgs::msg::InterfaceButtons::SharedPtr msg) {
            buttonPressed(msg);
        });

    // position subscriber
    _positionSubscription = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            _yPos = msg->pose.pose.position.y;
            _xPos = msg->pose.pose.position.x;
            RCLCPP_INFO(this->get_logger(), "Current Position - x: %.2f, y: %.2f",
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
                RCLCPP_INFO(this->get_logger(), "Button 2 detected in Init State, proceeding to next step.");

                // Set the robot to the initial position
                auto pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
                geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
                init_pose.header.frame_id = "map";
                init_pose.pose.pose.position.x = 0.0;
                init_pose.pose.pose.position.y = 0.0;
                init_pose.pose.pose.orientation.w = 1.0; // yaw=0
                pub->publish(init_pose);

                _initSteps++;
                _button = false; // reset button flag
            }
            break;
        }

        case 1: {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for robot to reach corner position position. Press Button 2 to confirm.");

            if (_button) {
                RCLCPP_INFO(this->get_logger(), "Button 2 detected in Init State, setting corner position 1.1.");

                // Set corner position 1.1
                setPositionReference(&_xPosLine11, &_yPosLine11);

                _initSteps++;
                _button = false; // reset button flag
            }
            break;
        }
        case 2: {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for robot to reach corner position 1.2. Press Button 2 to confirm.");
            if (_button) {
                RCLCPP_INFO(this->get_logger(), "Button 2 detected in Init State, setting corner position 1.2.");

                // Set corner position 1.2
                setPositionReference(&_xPosLine12, &_yPosLine12);

                _initSteps++;
                _button = false; // reset button flag
            }
            break;
        }

        case 3: {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for robot to reach corner position 2.1. Press Button 2 to confirm.");
            if (_button) {
                RCLCPP_INFO(this->get_logger(), "Button 2 detected in Init State, setting corner position 2.1.");

                // Set corner position 2.1
                setPositionReference(&_xPosLine21, &_yPosLine21);

                _initSteps++;
                _button = false; // reset button flag
            }
            break;
        }

        case 4: {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for robot to reach corner position 2.2. Press Button 2 to confirm.");
            if (_button) {
                RCLCPP_INFO(this->get_logger(), "Button 2 detected in Init State, setting corner position 2.2.");

                // Set corner position 2.2
                setPositionReference(&_xPosLine22, &_yPosLine22);

                _initSteps++;
                _button = false; // reset button flag
            }
            break;
        }

        case 5: {
            RCLCPP_INFO(this->get_logger(), "Initialization complete. Calculate goal lines.");

            // set position references in state machine
            // a = y2 - y1 ; b = -(x2 - x1) ; c = x2*y1 - y2*x1
            float a1 = _yPosLine12 - _yPosLine11;
            float b1 = -(_xPosLine12 - _xPosLine11);
            float c1 = _xPosLine12 * _yPosLine11 - _yPosLine12 * _xPosLine11;

            float a2 = _yPosLine22 - _yPosLine21;
            float b2 = -(_xPosLine22 - _xPosLine21);
            float c2 = _xPosLine22 * _yPosLine21 - _yPosLine22 * _xPosLine21;

            _stateMachine->setLineReference(a1, b1, c1, a2, b2, c2);
            break;
        }

        case 6: {
            RCLCPP_INFO(this->get_logger(), "Transitioning to Idle State. Set Turtlebot to starting position.");
            _stateMachine->transitionTo(StateType::IDLE);
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
    RCLCPP_INFO(this->get_logger(), "Exiting Init State");
    _inputSubscription.reset();
    _buttonSubscription.reset();
    _positionSubscription.reset();
    
}

// get state name for debugging
const char* InitState::getName() const {
    return "InitState";
}


// receive user input during init state
void InitState::receiveUserInput(const std_msgs::msg::String::SharedPtr msg) {
    _input = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received user input: %s", _input.c_str());
}


// handle button press during init state
void InitState::buttonPressed(const irobot_create_msgs::msg::InterfaceButtons::SharedPtr msg) {
    // Check if button 2 is pressed
    if (msg->button_2.is_pressed) {
        RCLCPP_INFO(this->get_logger(), "Button 2 pressed");
        _button = true;
    }
}

// set position references
void InitState::setPositionReference(float *xRef, float *yRef) {
    *xRef = _xPos;
    *yRef = _yPos;

    RCLCPP_INFO(this->get_logger(), "Position reference set to x: %.2f, y: %.2f", *xRef, *yRef);
}