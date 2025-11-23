#ifndef TURTLEBOT4_CPP
#define TURTLEBOT4_CPP

#include "rclcpp/rclcpp.hpp"
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

// create lightring message (internal linkage to allow safe inclusion in multiple TUs)
[[maybe_unused]] static irobot_create_msgs::msg::LightringLeds createLightringMessage(LightringColor color, const rclcpp::Time& now) {
    // create lightring message
    auto lightringMsg = irobot_create_msgs::msg::LightringLeds();

    // stamp the message with the current time
    lightringMsg.header.stamp = now;

    // override system lights
    lightringMsg.override_system = true;

    // set all LEDs to the specified color
    for (int i = 0; i < 6; i++) {
        // first turn off all colors
        lightringMsg.leds[i].red = 0;
        lightringMsg.leds[i].green = 0;
        lightringMsg.leds[i].blue = 0;

        // then set the desired color
        switch (color) {
            case LightringColor::OFF:
                // all LEDs off
                break;

            case LightringColor::RED:
                lightringMsg.leds[i].red = 255;
                break;

            case LightringColor::GREEN:
                lightringMsg.leds[i].green = 255;
                break;

            case LightringColor::BLUE:
                lightringMsg.leds[i].blue = 255;
                break;

            default:
                break;
        }
    }

    return lightringMsg;
}   

#endif // TURTLEBOT4_CPP