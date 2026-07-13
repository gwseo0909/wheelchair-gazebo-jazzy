import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wheelchair_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(
        pkg_share,
        'worlds',
        'wheelchair_world.sdf'
    )

    models_path = os.path.join(
        pkg_share,
        'models'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='wheelchair_bridge',
        output='screen',
        arguments=[
            # ROS 2 -> Gazebo
            '/model/wheelchair/cmd_vel'
            '@geometry_msgs/msg/Twist'
            '@gz.msgs.Twist',

            # Gazebo -> ROS 2 odometry
            '/model/wheelchair/odometry'
            '@nav_msgs/msg/Odometry'
            '[gz.msgs.Odometry',

            # Gazebo -> ROS 2 TF
            '/model/wheelchair/tf'
            '@tf2_msgs/msg/TFMessage'
            '[gz.msgs.Pose_V',
        ],
        remappings=[
            (
                '/model/wheelchair/odometry',
                '/odom'
            ),
            (
                '/model/wheelchair/tf',
                '/tf'
            ),
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_path
        ),

        gazebo_launch,
        bridge,
    ])