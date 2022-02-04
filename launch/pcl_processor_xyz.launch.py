#
#  SPDX-License-Identifier: Apache-2.0
#
#  Author: Shrijit Singh <shrijitsingh99@gmail.com>
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    current_pkg = FindPackageShare('pcl_processor')

    # Create temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    configured_params = [
        RewrittenYaml(
            source_file=LaunchConfiguration('params_file'),
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
                'use_sim_time', default_value='false',
                description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
                'params_file',
                default_value=PathJoinSubstitution([current_pkg, 'params', 'example.yaml']),
                description='Full path to the ROS2 parameters file to use'),

        Node(
            package='pcl_processor',
            executable='processor_xyz',
            name='processor_xyz',
            output='screen',
            parameters=configured_params),
    ])
