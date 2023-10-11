cmake_minimum_required(VERSION 3.8)
project(my_cpp_package)
set(CMAKE_CXX_STANDARD 14)

# if compilers are gnu or clang, then enable more compile information
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(my_msgs REQUIRED)

# executable targets
add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs)

add_executable(subscriber_node src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node rclcpp std_msgs sensor_msgs)

add_executable(timer_node src/timer_node.cpp)
ament_target_dependencies(timer_node rclcpp std_msgs)

add_executable(service_server_node src/service_server_node.cpp)
ament_target_dependencies(service_server_node rclcpp std_srvs)

add_executable(service_client_node src/service_client_node.cpp)
ament_target_dependencies(service_client_node rclcpp std_srvs)

add_executable(using_parameters_node src/using_parameters_node.cpp)
ament_target_dependencies(using_parameters_node rclcpp std_msgs)

add_executable(action_server_node src/action_server_node.cpp)
ament_target_dependencies(action_server_node rclcpp rclcpp_action rclcpp_components my_msgs)

add_executable(action_client_node src/action_client_node.cpp)
ament_target_dependencies(action_client_node rclcpp rclcpp_action rclcpp_components my_msgs)

# install cpp executables and share directories to destination
install(
  TARGETS publisher_node subscriber_node timer_node service_server_node service_client_node using_parameters_node action_server_node action_client_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ament operations
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
