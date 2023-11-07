cmake_minimum_required(VERSION 3.8)
project(my_general_pkg)
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# cpp executables
add_executable(executors_example_1 src/executors_example_1.cpp)
ament_target_dependencies(executors_example_1 rclcpp std_msgs)

add_executable(executors_example_2 src/executors_example_2.cpp)
ament_target_dependencies(executors_example_2 rclcpp std_msgs)

add_executable(executors_example_3 src/executors_example_3.cpp)
ament_target_dependencies(executors_example_3 rclcpp std_msgs)

# cpp files
install(
  TARGETS executors_example_1 executors_example_2 executors_example_3
  DESTINATION lib/${PROJECT_NAME}
)

# python files
install(
  PROGRAMS scripts/executors_py_example_1.py scripts/executors_py_example_2.py scripts/executors_py_example_3.py
  DESTINATION lib/${PROJECT_NAME}
)

# launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ament operations
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
