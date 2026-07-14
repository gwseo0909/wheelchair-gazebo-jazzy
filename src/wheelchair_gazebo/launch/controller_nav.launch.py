import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("wheelchair_gazebo")

    default_params_file = os.path.join(
        package_share,
        "config",
        "nav2_params.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=remappings + [
            ("cmd_vel", "cmd_vel_nav"),
        ],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=remappings,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=remappings,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=remappings,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "bt_navigator",
                    "behavior_server",
                ],
            }
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            "RCUTILS_LOGGING_BUFFERED_STREAM",
            "1",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation clock",
        ),

        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Full path to the Nav2 parameter file",
        ),

        controller_server,
        planner_server,
        bt_navigator,
        behavior_server,
        lifecycle_manager,
    ])