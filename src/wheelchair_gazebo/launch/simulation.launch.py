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

            # Gazebo -> ROS 2 LiDAR
            '/model/wheelchair/scan'
            '@sensor_msgs/msg/LaserScan'
            '[gz.msgs.LaserScan',

            # Gazebo -> ROS 2 simulation clock
            '/clock'
            '@rosgraph_msgs/msg/Clock'
            '[gz.msgs.Clock',
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
            (
                '/model/wheelchair/scan',
                '/scan'
            ),
        ]
    )

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
        arguments=[
            '--x', '0.30',
            '--y', '0.0',
            '--z', '0.75',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'wheelchair/lidar_link/lidar_sensor',
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=models_path
        ),

        gazebo_launch,
        bridge,
        lidar_static_tf,
    ])