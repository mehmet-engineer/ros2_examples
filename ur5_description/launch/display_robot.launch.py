from launch import LaunchDescription
from launch_ros.descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ARGUMENTS ---------------------------------------------------------------
    js_gui_arg = DeclareLaunchArgument("js_gui", default_value="true",
                                       description="Flag to enable joint_state_publisher_gui")

    # current package path
    pkg_share_path = get_package_share_directory("ur5_description")

    # Rviz config path
    rviz_config_path = PathJoinSubstitution(
        [pkg_share_path, "rviz", "view_robot.rviz"]
    )

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share_path, "urdf", "ur5.urdf.xacro"]
            ),
            " ",
            "name:=ur5"
        ]
    )

    # NODES -----------------------------------------------------------------
    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": ParameterValue(robot_description, value_type=str)}]
    )
    joint_state_publisher_gui_node = Node(
        name="joint_state_publisher_gui",
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("js_gui")),
    )
    joint_state_publisher_node = Node(
        name="joint_state_publisher",
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("js_gui")),
    )
    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [   
            js_gui_arg,
            joint_state_publisher_gui_node,
            joint_state_publisher_node,           
            robot_state_publisher_node,
            rviz_node,
            
        ]
    )
