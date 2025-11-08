#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "irobot_create_msgs/action/rotate_angle.hpp"

#include "bounceState.hpp"
#include "../statemachine.hpp"


BounceState::BounceState(StateMachine *stateMachine) 
    : State(stateMachine), rclcpp::Node("bounceState") {
}

void BounceState::onEnter() {
    RCLCPP_INFO(this->get_logger(), "Entering Bounce State");

    // Publisher for cmd_vel
    _cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));

    // Action client for rotate_angle
    _rotateClient = rclcpp_action::create_client<irobot_create_msgs::action::RotateAngle>(this->shared_from_this(), "/rotate_angle");

    // Start backing up
    _phase = Phase::BackUp;
    _backStart = std::chrono::steady_clock::now();

    // Immediately stop any current movement by publishing zero twist
    geometry_msgs::msg::Twist stop;
    _cmdVelPub->publish(stop);
}

void BounceState::run() {
    switch (_phase) {
        case Phase::BackUp: {
            // Drive backwards for the configured duration
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = -0.2; // m/s backwards
            cmd.angular.z = 0.0;
            _cmdVelPub->publish(cmd);

            auto now = std::chrono::steady_clock::now();
            if (now - _backStart >= _backDuration) {
                // Stop and move to rotation phase
                geometry_msgs::msg::Twist stop;
                _cmdVelPub->publish(stop);
                _phase = Phase::Rotate;
            }
            break;
        }

        case Phase::Rotate: {
            if (!_rotateGoalSent) {
                // Send a +90 degree rotation goal (in radians)
                sendRotateGoal(M_PI / 2.0);
                _rotateGoalSent = true;
            }
            // Waiting for result via callback
            break;
        }

        case Phase::Done: {
            // Transition back to drive state
            _stateMachine->transitionTo(StateType::DRIVE);
            break;
        }
    }
}

void BounceState::onExit() {
    // Stop movement
    if (_cmdVelPub) {
        geometry_msgs::msg::Twist stop;
        _cmdVelPub->publish(stop);
    }
    RCLCPP_INFO(this->get_logger(), "Exiting Bounce State");
    _cmdVelPub.reset();
    _rotateClient.reset();
}

const char* BounceState::getName() const {
    return "BounceState";
}

// send rotate goal to the action server
void BounceState::sendRotateGoal(double radians) {
    if (!_rotateClient->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "rotate_angle action server not available yet");
        // Retry later
        _rotateGoalSent = false;
        return;
    }

    irobot_create_msgs::action::RotateAngle::Goal goal;
    goal.angle = radians;
    // Optionally set max rotation speed if the field exists
    // goal.max_rotation_speed = 1.0f; // rad/s

    auto send_goal_options = rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SendGoalOptions();
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Rotation completed successfully");
                _phase = Phase::Done;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Rotation aborted");
                _phase = Phase::Done;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Rotation canceled");
                _phase = Phase::Done;
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown rotate action result code");
                _phase = Phase::Done;
                break;
        }
    };

    (void)_rotateClient->async_send_goal(goal, send_goal_options);
}

// // erste ausgabe
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <cmath>
// #include <vector>

// float calculate_wall_angle(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
//     std::vector<float> angles;
//     std::vector<float> distances;

//     for (size_t i = 0; i < scan->ranges.size(); ++i) {
//         float range = scan->ranges[i];
//         if (range < 0.5) { // Nur nahe Punkte betrachten
//             float angle = scan->angle_min + i * scan->angle_increment;
//             angles.push_back(angle);
//             distances.push_back(range);
//         }
//     }

//     // Beispiel: Mittelwert der Winkel als grobe Wandrichtung
//     float angle_sum = 0.0;
//     for (float a : angles) {
//         angle_sum += a;
//     }

//     return angles.empty() ? 0.0 : angle_sum / angles.size();
// }
// float bounce_angle = M_PI - wall_angle; // Spiegelung
// set_robot_heading(bounce_angle);

// // zweite Ausgabe Copilot

// public:
//     LaserScanSubscriber() : Node("laser_scan_subscriber") {
//         subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10,
//             std::bind(&LaserScanSubscriber::scan_callback, this, std::placeholders::_1)
//         );
//     }

// private:
//     void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//         RCLCPP_INFO(this->get_logger(), "Received LaserScan with %zu ranges", msg->ranges.size());

//         // Beispiel: Nahe Punkte auswerten
//         for (size_t i = 0; i < msg->ranges.size(); ++i) {
//             float range = msg->ranges[i];
//             if (range < 0.5) {
//                 float angle = msg->angle_min + i * msg->angle_increment;
//                 RCLCPP_INFO(this->get_logger(), "Nahe Punkt bei Winkel: %.2f°, Entfernung: %.2f m", angle * 180.0 / M_PI, range);
//             }
//         }
//     }

//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
// }