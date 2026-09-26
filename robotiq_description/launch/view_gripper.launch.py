from launch import LaunchDescription
from launch_ros.descriptions import ParameterValue
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # current package path
    pkg_share_path = get_package_share_directory("robotiq_description")

    # Rviz config path
    rviz_config_path = PathJoinSubstitution(
        [pkg_share_path, "rviz", "view_urdf.rviz"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_share_path, "urdf", "robotiq_gripper.xacro"]
            ),
            " ",
            "name:=ur5"
        ]
    )

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=None)}

    # NODES -----------------------------------------------------------------

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )
    joint_state_publisher_gui_node = Node(
        name="joint_state_publisher_gui",
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
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
            joint_state_publisher_gui_node,        
            robot_state_publisher_node,
            rviz_node,
        ]
    )
