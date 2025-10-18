#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class Robot : public rclcpp::Node
{
public:
    Robot() : Node("robot"), step_(0)
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
        
        // Subscriber: Distance
        distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/distance", 10,
            std::bind(&Robot::distance_callback, this, std::placeholders::_1));

        // Timer: Bewegung wechselt alle 3,3s
        movement_step_timer_ = this->create_wall_timer(3300ms, std::bind(&Robot::update_movement, this));

        // Timer: sendet kontinuierlich aktuelle Bewegung (alle 100ms)
        cmd_publish_timer_ = this->create_wall_timer(100ms, std::bind(&Robot::publish_current_cmd, this));

        RCLCPP_INFO(this->get_logger(), "Node gestartet: wartet auf /bumper_state...");
    }

private:
    // --- Callback: Distance ---
    void distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        distance_ = msg->data;  // jetzt distance_ in cm, nicht multiplizieren
        if (!bumper_hit_) {
	    std_msgs::msg::String display_msg;
	    display_msg.data = "Distanz: " + std::to_string(distance_) + " cm";
	    display_pub_->publish(display_msg);
        }
    }


    // Wird alle 1s aufgerufen – wechselt zwischen Vorwärts und Drehen
    void update_movement()
    {
        if (step_ >= 8)  // 4x (vorwärts + drehen)
        {
            current_cmd_.linear.x = 0.0;
            current_cmd_.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Bewegungssequenz beendet.");
            movement_step_timer_->cancel();
            return;
        }

        if (step_ % 2 == 0)
        {
            current_cmd_.linear.x = 1.0;
            current_cmd_.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "Schritt %d: Vorwärts", step_ / 2 + 1);
        }
        else
        {
            current_cmd_.linear.x = 0.0;
            current_cmd_.angular.z = 1.0;
            RCLCPP_INFO(this->get_logger(), "Schritt %d: Drehen", step_ / 2 + 1);
        }

        step_++;
    }

    // Wird alle 100ms aufgerufen – sendet aktuellen Bewegungsbefehl kontinuierlich
    void publish_current_cmd()
    {
        cmd_vel_pub_->publish(current_cmd_);
    }

    // Reaktion auf Bumper
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
            display_msg.data = " Bumper hit!";
            display_pub_->publish(display_msg);
        }
        else
        {
            // Zeige aktuelle Distanz nach Freigabe

        }
    }

    // ---- ROS Entities ----
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr beep_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr display_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bumper_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
    rclcpp::TimerBase::SharedPtr movement_step_timer_;
    rclcpp::TimerBase::SharedPtr cmd_publish_timer_;

    // ---- Zustände ----
    int step_;
    bool bumper_hit_ = false;
    double distance_ = 0.0;
    geometry_msgs::msg::Twist current_cmd_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot>());
    rclcpp::shutdown();
    return 0;
}

