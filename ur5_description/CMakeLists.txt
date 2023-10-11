cmake_minimum_required(VERSION 3.8)
project(ur5_description)
set(CMAKE_CXX_STANDARD 14)

# if compilers are gnu or clang, then enable more compile information
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

# install required files to share destination
install(
    DIRECTORY config launch urdf meshes rviz
    DESTINATION share/${PROJECT_NAME}
)

# ament operations
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
