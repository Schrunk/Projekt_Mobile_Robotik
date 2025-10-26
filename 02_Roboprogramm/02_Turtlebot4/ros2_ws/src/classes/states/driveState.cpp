#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


#include "driveState.hpp"
#include "../statemachine.hpp"
#include "../turtlebot4.cpp"


DriveState::DriveState(StateMachine *stateMachine) 
    : State(stateMachine), rclcpp::Node("driveState") {
}

void DriveState::onEnter() {
    RCLCPP_INFO(this->get_logger(), "Entering Drive State");

    // create LED ring publisher
    _lightringPublisher = this->create_publisher<irobot_create_msgs::msg::LightringLeds>(
      "/cmd_lightring", rclcpp::SensorDataQoS());
    _lightringPublisher->publish(createLightringMessage(LightringColor::GREEN, this->get_clock()->now()));

    // creater drive command publisher
    _drivePublisher = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10));

    // create input subscription
    _inputSubscription = this->create_subscription<std_msgs::msg::String>(
      "/input", 10,
      std::bind(&DriveState::inputCallback, this, std::placeholders::_1));

    // create hazard detection subscriber
    _hazardSubscription = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
      "/hazard_detection", 10,
      std::bind(&DriveState::bumperCallback, this, std::placeholders::_1));

    // position subscriber
    _positionSubscription = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            _yCurrentPos = msg->pose.pose.position.y;
            _xCurrentPos = msg->pose.pose.position.x;
            RCLCPP_INFO(this->get_logger(), "Current Position - x: %.2f, y: %.2f",
                        _xCurrentPos, _yCurrentPos);
    });

    // calculate line references
    float a1, b1, c1, a2, b2, c2;
    _stateMachine->getLineReferences(a1, b1, c1, a2, b2, c2);
    _fCurrentPosLine1 = a1 * _xCurrentPos + b1 * _yCurrentPos + c1;
    _fCurrentPosLine2 = a2 * _xCurrentPos + b2 * _yCurrentPos + c2;

    // create timer for drive commands
    _timerStart = std::chrono::steady_clock::now();
}

void DriveState::run() {
    // publish drive command
    if ((std::chrono::steady_clock::now() - _timerStart) > std::chrono::milliseconds(100)) {
        geometry_msgs::msg::Twist driveMsg;
        driveMsg.linear.x = 0.3;  // forward speed
        driveMsg.angular.z = 0.0; // no rotation
        _drivePublisher->publish(driveMsg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "Driving forward at 0.3 m/s");
        
        _timerStart = std::chrono::steady_clock::now();
    }
}

void DriveState::onExit() {
    RCLCPP_INFO(this->get_logger(), "Exiting Drive State");
    // stop the robot
    geometry_msgs::msg::Twist stopMsg;
    stopMsg.linear.x = 0.0;
    stopMsg.angular.z = 0.0;
    _drivePublisher->publish(stopMsg);

    // clean up publishers and subscriptions
    _lightringPublisher.reset();
    _drivePublisher.reset();
    _inputSubscription.reset();
    _hazardSubscription.reset();
}

const char* DriveState::getName() const {
    return "DriveState";
}

// bumper callback to handle obstacle detection
void DriveState::bumperCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg) {
    for (const auto& hazard : msg->detections) {
        if (hazard.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
            RCLCPP_WARN(this->get_logger(), "Bumper pressed! Transitioning to Bounce State.");
            _stateMachine->transitionTo(StateType::BOUNCE);
            return;
        }
    }
}

// input callback to handle external commands
void DriveState::inputCallback(const std_msgs::msg::String::SharedPtr msg) {
    _input = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received input: %s", _input.c_str());

    if (_input == "stop") {
        RCLCPP_INFO(this->get_logger(), "Stop command received! Transitioning to Idle State.");
        _stateMachine->transitionTo(StateType::IDLE);
    }
}

// check wether a line has been crossed
void DriveState::checkLineCrosses() {
    float a1, b1, c1, a2, b2, c2;
    _stateMachine->getLineReferences(a1, b1, c1, a2, b2, c2);

    float fNewPosLine1 = a1 * _xCurrentPos + b1 * _yCurrentPos + c1;
    if (fNewPosLine1 * _fCurrentPosLine1 <= 0) {
        RCLCPP_INFO(this->get_logger(), "Line 1 crossed!");
        _stateMachine->transitionTo(StateType::BACK_TO_START);
    }

    float fNewPosLine2 = a2 * _xCurrentPos + b2 * _yCurrentPos + c2;
    if (fNewPosLine2 * _fCurrentPosLine2 <= 0) {
        RCLCPP_INFO(this->get_logger(), "Line 2 crossed!");
        _stateMachine->transitionTo(StateType::BACK_TO_START);
    }
}