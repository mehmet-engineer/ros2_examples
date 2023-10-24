import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # current package path
    this_package_name = "my_cpp_package"
    this_pkg_share_path = get_package_share_directory(this_package_name)

    # NODES -----------------------------------------------------------------

    publisher_node = Node(
        name = "publisher_node",
        package = this_package_name,
        executable = "publisher_node",
    )

    subscriber_node = Node(
        name = "subscriber_node",
        package = this_package_name,
        executable = "subscriber_node",
    )

    timer_node = Node(
        name = "timer_node",
        package = this_package_name,
        executable = "timer_node",
    )

    # INCLUDE LAUNCH -----------------------------------------------------------------
    description_package_name = "orion5_description"
    description_pkg_share_path = get_package_share_directory(description_package_name)
    launch_path = os.path.join(description_pkg_share_path, "launch/display_robot.launch.py")
    display_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            'js_gui': 'true',
        }.items(),
    )

    return LaunchDescription(
        [   
            publisher_node,
            subscriber_node,
            timer_node,
            display_robot_launch
        ]
    )
