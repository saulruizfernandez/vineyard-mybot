# https://docs.ros.org/en/kilted/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
# Open Gazebo: $ gz sim
# Spawn robot in Gazebo: $ ros2 run ros_gz_sim create -topic robot_description -entity mybot

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.actions import AppendEnvironmentVariable


def generate_launch_description():

    package_name = 'vineyard-mybot'
    world_file = 'empty.sdf'
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory(package_name),'launch','rsp.launch.py'
            )]), launch_arguments={'use_sim_time': 'true'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ['-r -s -v4 ', world_file], 'on_exit_shutdown': 'true'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-g -v4 '}.items()
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-entity', 'mybot'],
            output='screen'
        )
    ])