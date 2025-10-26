#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>

#include "classes/terminalPub.hpp"
#include "classes/statemachine.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create a multithreaded executor so multiple nodes can spin together
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Terminal input publisher node
    auto terminalNode = std::make_shared<TerminalPublisher>();
    executor->add_node(terminalNode);

    // Initialize the state machine (registers and enters InitState)
    auto stateMachine = std::make_unique<StateMachine>();

    // Register transition callback to keep executor nodes in sync with FSM state
    stateMachine->setOnStateChanged([executor](State* oldState, State* newState) {
      if (auto oldNode = dynamic_cast<rclcpp::Node*>(oldState)) {
        executor->remove_node(oldNode->get_node_base_interface());
      }
      if (auto newNode = dynamic_cast<rclcpp::Node*>(newState)) {
        executor->add_node(newNode->get_node_base_interface());
      }
    });

    stateMachine->initialize();

    // If the current state is an rclcpp::Node, add it to the executor
    if (auto statePtr = stateMachine->getCurrentState()) {
        if (auto nodePtr = dynamic_cast<rclcpp::Node*>(statePtr)) {
            executor->add_node(nodePtr->get_node_base_interface());
        }
    }

    // Spin and update FSM in the same loop
    while (rclcpp::ok() && stateMachine->isRunning()) {
      // Process available ROS callbacks without blocking indefinitely
      executor->spin_some();

      // Advance the state machine once (states should be non-blocking)
      stateMachine->update();

      // Small sleep to avoid busy-waiting; tune as needed
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::shutdown();
    return 0;
}

// enum class States {
//     INIT,
//     IDLE,
//     DRIVE,
//     BACK_TO_START,
//     BOUNCE
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);

//     auto terminalNode = std::make_shared<TerminalPublisher>();
//     auto turtlebot4Node = std::make_shared<TurtleBot4>();

//     int currentState = States::INIT;
//     bool setupComplete = false;

//     while (rclcpp::ok()) {

//         rclcpp::spin_some(terminalNode);
//         rclcpp::spin_some(turtlebot4Node);

//         switch (currentState) {
//             case States::INIT:
                
//                 break;

//             case States::IDLE:
//                 // Handle IDLE state
//                 break;
//             case States::DRIVE:
//                 // Handle DRIVE state
//                 break;
//             case States::BACK_TO_START:
//                 // Handle BACK_TO_START state
//                 break;
//             case States::BOUNCE:
//                 // Handle BOUNCE state
//                 break;

//             default:
//                 RCLCPP_ERROR(rclcpp::get_logger("main"), "Unknown state!");
//                 break;
//         }


//     }

//     rclcpp::shutdown();
//     return 0;
// }