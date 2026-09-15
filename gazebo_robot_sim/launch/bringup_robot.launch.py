import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # -------------------------------------------------------------------------
    # PACKAGE PATHS
    # -------------------------------------------------------------------------

    robot_description_pkg = get_package_share_directory(
        "ur5_description"
    )

    gazebo_robot_sim_pkg = get_package_share_directory(
        "gazebo_robot_sim"
    )

    # -------------------------------------------------------------------------
    # ROBOT DESCRIPTION
    # -------------------------------------------------------------------------

    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            robot_description_pkg,
            "urdf",
            "ur5_urdf.xacro"
        ]),
        " ",
        "name:=ur5"
    ])

    # -------------------------------------------------------------------------
    # JOINT STATE BROADCASTER
    # -------------------------------------------------------------------------

    joint_state_broadcaster = Node(
        name="joint_state_broadcaster",
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"       
        ]
    )

    # -------------------------------------------------------------------------
    # ROBOT STATE PUBLISHER
    # -------------------------------------------------------------------------

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    robot_description,
                    value_type=str
                )
            }
        ]
    )

    # -------------------------------------------------------------------------
    # GAZEBO FORTRESS
    # -------------------------------------------------------------------------

    parent_dir = os.path.dirname(robot_description_pkg)

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + parent_dir
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = parent_dir

    gazebo_fortress = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": "-r " + os.path.join(
                gazebo_robot_sim_pkg,
                "worlds",
                "empty.world"
            )
        }.items()
    )

    # -------------------------------------------------------------------------
    # SPAWN ROBOT
    # -------------------------------------------------------------------------

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "ur5",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.4"
        ],
        output="screen"
    )

    # -------------------------------------------------------------------------
    # ROS2 CONTROL & JOINT TRAJECTORY CONTROLLER
    # -------------------------------------------------------------------------

    joint_trajectory_controller = Node(
        name="joint_trajectory_controller",
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager"       
        ]
    )

    load_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[joint_trajectory_controller],
        )
    )

    # -------------------------------------------------------------------------
    # LAUNCH
    # -------------------------------------------------------------------------

    return LaunchDescription([
        joint_state_broadcaster,
        robot_state_publisher,

        gazebo_fortress,
        spawn_robot,

        load_joint_trajectory_controller
    ])