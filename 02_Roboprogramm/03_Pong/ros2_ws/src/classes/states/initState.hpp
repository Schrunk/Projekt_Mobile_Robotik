#ifndef INITSTATE_HPP
#define INITSTATE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"
#include <string>
#include "state.hpp"
#include "../statemachine.hpp"

// Initialization State Class
class InitState : public State, public rclcpp::Node {
public:
    // constructor
    explicit InitState(StateMachine *stateMachine);

    // destructor
    ~InitState() override = default;

    // called when entering the init state
    void onEnter() override;

    // main execution method for the init state
    void run() override;

    // called when exiting the init state
    void onExit() override;

    // get the name of this state
    const char* getName() const override;

private:
    // terminal input subscription
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _terminalSubscription;
    std::string _terminalInput;

    void receiveUserInput(const std_msgs::msg::String::SharedPtr msg);

    // button subscription
    rclcpp::Subscription<irobot_create_msgs::msg::InterfaceButtons>::SharedPtr _buttonSubscription;

    // position subscription
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _positionSubscription;

    bool _button{false};
    int _initSteps{0};

    float _xPos;
    float _yPos;
    float _yawPos{0.0f};

    // position references
    float _xPosLine11;
    float _yPosLine11;
    float _xPosLine12;
    float _yPosLine12;

    float _xPosLine21;
    float _yPosLine21;
    float _xPosLine22;
    float _yPosLine22;


    void buttonPressed(const irobot_create_msgs::msg::InterfaceButtons::SharedPtr msg);

    void setPositionReference(float &xRef, float &yRef);

    // helper to compute yaw from quaternion
    static double quaternionToYaw(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);
};

#endif // INITSTATE_HPP