from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # run joint trajectory control python node
    traj_control_node = Node(
        package='robot_sim',
        executable='example3.py',
        output='screen',
        parameters=[
            {"use_sim_time": True}
        ]
    )

    return LaunchDescription([
        traj_control_node
    ])