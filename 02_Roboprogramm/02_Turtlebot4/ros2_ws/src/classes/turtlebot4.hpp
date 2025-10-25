#ifndef TURTLEBOT4_HPP
#define TURTLEBOT4_HPP

#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/interface_buttons.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"

using namespace std::chrono_literals;

enum class LightringColor {
    OFF,
    RED,
    GREEN,
    BLUE,
};

enum class Button {
    BUTTON_1,
    BUTTON_2,
};

class TurtleBot4 : public rclcpp::Node {
public:
    // constructor
    TurtleBot4();

    // destructor
    ~TurtleBot4();

    // set lightring color
    void setLightring(LightringColor color);

    // get button state 
    bool getButtonState(Button button);

    // get distance sensor reading
    void getDistanceSensorReading(float &angle, float &distance);

    // get bumper state
    bool getBumperState();

    // get odometry data
    float getOdometryData();

    // get approximity detection
    bool getApproximityDetection();

    // move linear
    bool moveLinear(float speed, float distance);

    // move angular
    bool moveAngular(float angular_speed, float angle);

    // navigate to position
    bool navigateTo(float x, float y, float theta);

private:
    // lightring publisher
    rclcpp::Publisher<irobot_create_msgs::msg::LightringLeds>::SharedPtr _lightringPublisher;
    
    // functions and members to be added here
    
};


#endif // TURTLEBOT4_HPP