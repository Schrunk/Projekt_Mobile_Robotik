cmake_minimum_required(VERSION 3.8)
project(pong_game)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)

add_executable(pong_game 
  src/main.cpp
  src/classes/terminalPub.cpp
  src/classes/turtlebot4.cpp
  src/classes/states/initState.cpp
  src/classes/states/idleState.cpp
  src/classes/states/bounceState.cpp
  src/classes/states/driveState.cpp
  src/classes/states/backToStartState.cpp
  src/classes/states/errorState.cpp
  src/classes/states/setupState.cpp
  src/classes/statemachine.cpp
)

ament_target_dependencies(pong_game irobot_create_msgs)

target_compile_features(pong_game PRIVATE cxx_std_17)
ament_target_dependencies(pong_game rclcpp std_msgs)

install(TARGETS pong_game
  DESTINATION lib/${PROJECT_NAME})

ament_package()
