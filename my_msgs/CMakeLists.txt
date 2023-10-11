cmake_minimum_required(VERSION 3.8)
project(my_msgs)
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# cmake function related to ros2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyRobot.msg"
  "srv/AddNumbers.srv"
  "action/MoveRobot.action"
  DEPENDENCIES std_msgs rosidl_default_generators
)

ament_package()
