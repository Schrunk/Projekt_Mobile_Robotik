#ifndef BACK_TO_START_STATE_HPP
#define BACK_TO_START_STATE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "state.hpp"
#include "../statemachine.hpp"


class BackToStartState : public State, public rclcpp::Node {
public:

    BackToStartState(StateMachine* stateMachine);
    

    ~BackToStartState() override = default;

 
    void onEnter() override;


    void run() override;

 
    void onExit() override;


    const char* getName() const override;

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    rclcpp_action::Client<NavigateToPose>::SharedPtr _navClient;
    bool _goalSent{false};

    void sendGoalToOrigin();
};

#endif // BACK_TO_START_STATE_HPP