#ifndef BOUNCESTATE_HPP
#define BOUNCESTATE_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "state.hpp"
#include "../statemachine.hpp"

class BounceState : public State, public rclcpp::Node {
public:

    explicit BounceState(StateMachine *stateMachine);
    
    ~BounceState() override = default;

    void onEnter() override;

    void run() override;

    void onExit() override;

    const char* getName() const override;

private:
    // Phases for bounce behavior
    enum class Phase { BackUp, Rotate, Done };

    Phase _phase = Phase::BackUp;

    // publishers and clients
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmdVelPub;
    rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SharedPtr _rotateClient;

    // timing and state
    std::chrono::steady_clock::time_point _backStart;
    std::chrono::milliseconds _backDuration{300}; // back up for a bit longer to clear wall
    bool _rotateGoalSent = false;

    // LaserScan data
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scanSub;
    sensor_msgs::msg::LaserScan _lastScan;
    bool _haveScan{false};
    double _chosenRotateAngle{0.0};

    void sendRotateGoal(double radians);
    double randomAngleRad();
    void analyzeWallAndChooseAngle();
    static double normalizeAngle(double a);
};

#endif // BOUNCE_STATE_HPP