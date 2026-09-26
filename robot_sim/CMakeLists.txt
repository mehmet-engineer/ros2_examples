cmake_minimum_required(VERSION 3.8)
project(robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(example1 src/example1.cpp)
ament_target_dependencies(example1
  rclcpp
  moveit_ros_planning_interface
)

add_executable(example2 src/example2.cpp)
ament_target_dependencies(example2
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS
  example1 example2
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/example3.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
