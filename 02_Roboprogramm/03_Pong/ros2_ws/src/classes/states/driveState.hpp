#ifndef DRIVE_STATE_HPP
#define DRIVE_STATE_HPP

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "irobot_create_msgs/msg/hazard_detection.hpp"
// Inputs and pose subscription types
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "state.hpp"
#include "../statemachine.hpp"

class DriveState : public State, public rclcpp::Node {
public:
    // constructor
    explicit DriveState(StateMachine* stateMachine);

    // destructor
    ~DriveState() override = default;

    // called when entering the drive state
    void onEnter() override;

    // main execution loop for drive state
    void run() override;

    // called when exiting the drive state
    void onExit() override;

    // get the name of the state
    const char* getName() const override;

private:
    // publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _drivePublisher;
    // subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _inputSubscription;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr _hazardSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _positionSubscription;

    // member variables
    std::chrono::steady_clock::time_point _timerStart;
    std::string _input;

    // line cross members
    float _xCurrentPos;
    float _yCurrentPos;

    float _fCurrentPosLine1;
    float _fCurrentPosLine2;
    
    void bumperCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr msg);
    void inputCallback(const std_msgs::msg::String::SharedPtr msg);
    void checkLineCrosses();
};

#endif // DRIVE_STATE_HPP