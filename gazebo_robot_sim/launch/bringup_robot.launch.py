import os
from launch import LaunchDescription
from launch_ros.descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():

    # ARGUMENTS ---------------------------------------------------------------

    rviz_arg = DeclareLaunchArgument("rviz_gui", default_value="false",
                                       description="Flag to enable rviz")

    # current package path
    curr_pkg_share_path = get_package_share_directory("gazebo_robot_sim")
    robot_description_pkg_share_path = get_package_share_directory("ur5_description")

    # Rviz config path
    rviz_config_path = PathJoinSubstitution(
        [robot_description_pkg_share_path, "rviz", "view_robot.rviz"]
    )

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [robot_description_pkg_share_path, "urdf", "ur5.urdf.xacro"]
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
    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz_gui"))
    )

    # GAZEBO -----------------------------------------------------------------

    description_share = os.path.join(get_package_prefix("ur5_description"), "share")
    gazebo_env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", description_share)

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            'physics': '',   # default ode
            'verbose': 'true',
            'world': os.path.join(curr_pkg_share_path, "worlds", "empty.world"),
            'init': 'true',
            'factory': 'true',
            'force_system': 'true',
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
        launch_arguments={
            'verbose': 'true'
        }.items()
    )

    spawn_robot = Node(
        name="spawn_robot",
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-entity", "ur5", 
                   "-topic", "robot_description",
                   "-x", "0.0",
                   "-y", "0.0",
                   "-z", "0.5"]
    )

    # ROS 2 CONTROL ----------------------------------------------------------

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

    # Return Launch Function -------------------------------------------------

    return LaunchDescription(
        [   
            rviz_arg,
            rviz_node,
            robot_state_publisher_node,
            
            gazebo_env_var,
            gazebo_server,
            gazebo_client,
            spawn_robot,

            joint_state_broadcaster,
            joint_trajectory_controller,
        ]
    )
