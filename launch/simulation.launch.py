# https://docs.ros.org/en/kilted/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
# Open Gazebo: $ gz sim
# Spawn robot in Gazebo: $ ros2 run ros_gz_sim create -topic robot_description -entity mybot

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter


def generate_launch_description():

    package_name = 'vineyard-mybot'
    world_file = 'empty.sdf'
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    robot_controllers = PathJoinSubstitution(
    [
        FindPackageShare("vineyard-mybot"),
        "config",
        "my_controllers.yaml",
    ]
    )
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
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
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diffbot_base_controller",
                "--param-file",
                robot_controllers,
                "--controller-ros-args",
                "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}',
            ],
        )
    ])