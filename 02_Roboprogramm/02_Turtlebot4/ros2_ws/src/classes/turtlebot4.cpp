#ifndef TURTLEBOT4_CPP
#define TURTLEBOT4_CPP

#include "turtlebot4.hpp"
#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"

using namespace std::chrono_literals;


// constructor
TurtleBot4::TurtleBot4() : rclcpp::Node("turtlebot4") {
    // publisher
    _lightringPublisher = this->create_publisher<irobot_create_msgs::msg::LightringLeds>(
      "/cmd_lightring", rclcpp::SensorDataQoS());

    // subscriber
}


// destructor
TurtleBot4::~TurtleBot4() {
    // TODO cleanup if necessary
}


// set lightring color
void TurtleBot4::setLightring(LightringColor color) {
    // create lightring message
    auto lightring_msg = irobot_create_msgs::msg::LightringLeds();

    // stamp the message with the current time
    lightring_msg.header.stamp = this->get_clock()->now();

    // override system lights
    lightring_msg.override_system = true;

    // set all LEDs to the specified color
    for (int i = 0; i < 6; i++) {
        // first turn off all colors
        lightring_msg.leds[i].red = 0;
        lightring_msg.leds[i].green = 0;
        lightring_msg.leds[i].blue = 0;

        // then set the desired color
        switch (color) {
            case LightringColor::OFF:
                // all LEDs off
                break;
            case LightringColor::RED:
                lightring_msg.leds[i].red = 255;
                break;
            case LightringColor::GREEN:
                lightring_msg.leds[i].green = 255;
                break;
            case LightringColor::BLUE:
                lightring_msg.leds[i].blue = 255;
                break;
            default:
                break;
        }

    // publish the lightring message
    _lightringPublisher->publish(lightring_msg);
    }
}

#endif // TURTLEBOT4_CPP