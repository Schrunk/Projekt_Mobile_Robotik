#include <cmath>
#include <vector>
#include <random>

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
    RCLCPP_DEBUG(this->get_logger(), "Entering Bounce State");

    // Publisher for cmd_vel
    _cmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));
    // Immediately stop any current movement by publishing zero twist
    geometry_msgs::msg::Twist stop;
    _cmdVelPub->publish(stop);

    // Action client for rotate_angle
    _rotateClient = rclcpp_action::create_client<irobot_create_msgs::action::RotateAngle>(this->shared_from_this(), "/rotate_angle");

    // LaserScan subscription (to detect wall orientation / proximity distribution)
    _scanSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){
            _lastScan = *msg;
            _haveScan = true;
        }
    );

    // Start backing up
    _phase = Phase::BackUp;
    _backStart = std::chrono::steady_clock::now();
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
                stop.linear.x = 0.0;
                stop.angular.z = 0.0;
                _cmdVelPub->publish(stop);
                // Before rotating, analyze scan and choose angle away from wall
                analyzeWallAndChooseAngle();
                _phase = Phase::Rotate;
            }
            break;
        }

        case Phase::Rotate: {
            if (!_rotateGoalSent) {
                // use chosen angle (fallback to random if not computed)
                double angle = (_chosenRotateAngle == 0.0) ? randomAngleRad() : _chosenRotateAngle;
                sendRotateGoal(angle);
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
    RCLCPP_DEBUG(this->get_logger(), "Exiting Bounce State");
    _cmdVelPub.reset();
    _rotateClient.reset();
    _scanSub.reset();
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

    auto send_goal_options = rclcpp_action::Client<irobot_create_msgs::action::RotateAngle>::SendGoalOptions();
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<irobot_create_msgs::action::RotateAngle>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_DEBUG(this->get_logger(), "Rotation completed successfully");
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

double BounceState::randomAngleRad() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist_deg(45, 90); // ganze Grad
    return dist_deg(gen) * M_PI / 180.0;
}

void BounceState::analyzeWallAndChooseAngle() {
    if (!_haveScan) {
        RCLCPP_WARN(this->get_logger(), "No scan available; using random angle");
        _chosenRotateAngle = randomAngleRad();
        return;
    }

    const auto & scan = _lastScan;
    // Determine indices around front-left and front-right sectors.
    // Assume angle_min .. angle_max spans typically -pi .. +pi.
    const double frontCenter = 0.0; // forward direction
    const double sectorWidth = M_PI / 4.0; // 45° sectors
    double leftMin = std::numeric_limits<double>::infinity();
    double rightMin = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double angle = scan.angle_min + i * scan.angle_increment;
        double r = scan.ranges[i];
        // discard invalid readings
        if (r <= 0.0 || std::isnan(r) || std::isinf(r)) continue;
        // right sector: angles between 0 and +sectorWidth
        if (angle >= frontCenter && angle <= frontCenter + sectorWidth) {
            rightMin = std::min(rightMin, r);
        }
        // left sector: angles between -sectorWidth and 0
        if (angle <= frontCenter && angle >= frontCenter - sectorWidth) {
            leftMin = std::min(leftMin, r);
        }
    }

    // Decide rotation direction away from closest side
    double baseAngle = randomAngleRad();
    if (leftMin < rightMin) {
        // obstacle closer on left -> rotate right (negative angle)
        _chosenRotateAngle = -baseAngle;
        RCLCPP_DEBUG(this->get_logger(), "Wall left (%.2f vs %.2f). Rotate right %.2f deg", leftMin, rightMin, std::abs(_chosenRotateAngle)*180.0/M_PI);
    } else if (rightMin < leftMin) {
        // obstacle closer on right -> rotate left (positive angle)
        _chosenRotateAngle = baseAngle;
        RCLCPP_DEBUG(this->get_logger(), "Wall right (%.2f vs %.2f). Rotate left %.2f deg", rightMin, leftMin, std::abs(_chosenRotateAngle)*180.0/M_PI);
    } else {
        // equal or no data -> fallback 
        _chosenRotateAngle = M_PI; // 180 degrees
        RCLCPP_DEBUG(this->get_logger(), "Wall ambiguous (%.2f vs %.2f). Fallback rotate %.2f deg", leftMin, rightMin, std::abs(_chosenRotateAngle)*180.0/M_PI);
    }
}

double BounceState::normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
