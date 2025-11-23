#include "backToStartState.hpp"
#include "../statemachine.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;

BackToStartState::BackToStartState(StateMachine* stateMachine) 
    : State(stateMachine), rclcpp::Node("backToStartState") {
}

void BackToStartState::onEnter() {
    RCLCPP_DEBUG(this->get_logger(), "Driving to init Position");
    // Create action client
    _navClient = rclcpp_action::create_client<NavigateToPose>(this->shared_from_this(), "navigate_to_pose");
    RCLCPP_DEBUG(this->get_logger(), "Nav2 Node aktiviert");

    // Try to send goal immediately; if server not ready, run() will retry
    sendBackToStart();
}

void BackToStartState::run() {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Driving to start position...");

    if (!_goalSent) {
        // Retry sending goal if it wasn't sent yet (e.g., server not available)
        static rclcpp::Time last = this->now();
        if ((this->now() - last).seconds() > 15.0) {
            RCLCPP_WARN(this->get_logger(), "Retrying to send NavToGoal to start pose...");
            sendBackToStart();
            last = this->now();
        }
    }
}

void BackToStartState::onExit() {
    RCLCPP_INFO(this->get_logger(), "Middle position reached");
    _goalSent = false;
    _navClient.reset();
}

const char* BackToStartState::getName() const {
    return "BackToStartState";
}

void BackToStartState::sendBackToStart() {
    if (!_navClient) {
        RCLCPP_ERROR(this->get_logger(), "Navigate action client not initialized");
        _goalSent = false;
        return;
    }
    
    if (!_navClient->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "navigate_to_pose action server not available yet");
        _goalSent = false;
        return;
    }

    // Obtain stored start pose from state machine
    double sx, sy, syaw;
    if (!_stateMachine->getStartPose(sx, sy, syaw)) {
        RCLCPP_ERROR(this->get_logger(), "Start pose not set - restart programm!");
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();
    goal.pose.pose.position.x = sx;
    goal.pose.pose.position.y = sy;
    // quaternion from yaw
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = std::sin(syaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(syaw / 2.0);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_DEBUG(this->get_logger(), "Reached start pose. Switching to DRIVE.");
                _stateMachine->transitionTo(StateType::IDLE);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Navigation aborted.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Navigation canceled.");
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown nav result code");
                break;
        }
    };

    (void)_navClient->async_send_goal(goal, send_goal_options);
    _goalSent = true;
}