from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PythonExpression, LaunchConfiguration


def generate_launch_description():

    # current package path
    this_package_name = "my_general_pkg"
    this_pkg_share_path = get_package_share_directory(this_package_name)

    # LAUNCH ARGUMENTS ----------------------------------------------------
    node_start_arg = DeclareLaunchArgument(
        'node_start_arg',
        default_value='True'
    )

    # TERMINAL COMMANDS ----------------------------------------------------
    terminal_command = ExecuteProcess(
        cmd = [
            ['ros2 topic ', 'list']
        ],
        shell=True
    )

    # IF CONDITIONS --------------------------------------------------------
    publisher_node = Node(
        condition = IfCondition(LaunchConfiguration("node_start_arg")),
        name = "publisher_timer_node",
        package = "my_py_package",
        executable = "publisher_timer_node",
    )

    subscriber_node = Node(
        condition = UnlessCondition(LaunchConfiguration("node_start_arg")),
        name = "subscriber_node",
        package = "my_py_package",
        executable = "subscriber_node",
    )

    service_server_node = Node(
        condition = IfCondition( PythonExpression([
                '(',
                LaunchConfiguration("node_start_arg"),
                ' == True)',
                ' and ',
                '(1 == 1)'
            ])),
        name = "service_server_node",
        package = "my_py_package",
        executable = "service_server_node",
    )

    # TIME SLEEP - DELAY ---------------------------------------------------
    delay_time_sec = 3.0
    delayed_node = TimerAction(
        period = delay_time_sec,
        actions = [
            Node(
                name = "service_server_node",
                package = "my_py_package",
                executable = "service_server_node",
            )
        ]
    )

    # RETURN LAUNCH -------------------------------------------------------
    return LaunchDescription(
        [   
            node_start_arg,
            terminal_command,
            publisher_node,
            subscriber_node,
            service_server_node,
            delayed_node
        ]
    )
