#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class Robot : public rclcpp::Node
{
public:
    Robot() : Node("robot")
    {
        // Publisher
        beep_pub_ = this->create_publisher<std_msgs::msg::Bool>("/beep", 10);
        display_pub_ = this->create_publisher<std_msgs::msg::String>("/display_text", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber
        bumper_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/bumper_state",
            10,
            std::bind(&Robot::bumper_callback, this, std::placeholders::_1)
        );

        distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/distance", 10,
            std::bind(&Robot::distance_callback, this, std::placeholders::_1));

        buttons_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/buttons", 10,
            std::bind(&Robot::buttons_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Node gestartet: wartet auf /buttons...");
    }

private:
    // --- Callback: Distance ---
    void distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        distance_ = msg->data;
        if (!bumper_hit_) {
            std_msgs::msg::String display_msg;
            display_msg.data = "Distanz: " + std::to_string(distance_) + " cm";
            display_pub_->publish(display_msg);
        }
    }

    // --- Callback: Buttons ---
    void buttons_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;

        if (msg->data == "up" && bumper_hit_ == false) {
            // Prüfe Sicherheitsabstand
            if (distance_ < 10.0) {
                RCLCPP_WARN(this->get_logger(), "Abstand zu gering (%.2f cm) -> Fahrt gestoppt", distance_);

                // Beeper einschalten
                std_msgs::msg::Bool beep_msg;
                beep_msg.data = true;
                beep_pub_->publish(beep_msg);

                // Display-Warnung
                std_msgs::msg::String display_msg;
                display_msg.data = "No safe distance!";
                display_pub_->publish(display_msg);

                // Keine Bewegung publizieren
                return;
            } else {
                cmd.linear.x = 0.5;   // Vorwärts
                cmd.angular.z = 0.0;
            }
        } else if (msg->data == "down") {
            cmd.linear.x = -0.5;  // Rückwärts
            cmd.angular.z = 0.0;
        } else if (msg->data == "left" && bumper_hit_ == false) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 1.0;  // Drehung links
        } else if (msg->data == "right" && bumper_hit_ == false) {
            cmd.linear.x = 0.0;
            cmd.angular.z = -1.0; // Drehung rechts
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }

        // Wenn kein Sicherheitsproblem, dann Fahrbefehl senden
        cmd_vel_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Button: %s -> cmd_vel published", msg->data.c_str());
    }

    // --- Callback: Bumper ---
    void bumper_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bumper_hit_ = msg->data;

        if (msg->data)
        {
            RCLCPP_WARN(this->get_logger(), "Bumper hit! -> Send Beep TRUE");
            std_msgs::msg::Bool beep_msg;
            beep_msg.data = true;
            beep_pub_->publish(beep_msg);

            std_msgs::msg::String display_msg;
            display_msg.data = "Bumper hit!";
            display_pub_->publish(display_msg);
        }
    }

    // ---- ROS Entities ----
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr beep_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr display_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr buttons_sub_;

    // ---- Zustände ----
    bool bumper_hit_ = false;
    double distance_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
}

