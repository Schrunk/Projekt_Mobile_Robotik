#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/string.hpp>

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

  // Spin all nodes
  executor->spin();

  rclcpp::shutdown();
  return 0;
}