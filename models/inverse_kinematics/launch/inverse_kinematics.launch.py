#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inverse_kinematics',
            executable='inverse_kinematics',
            name='inverse_kinematics_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('inverse_kinematics'), 
                'config', 'config.yaml')])
    ])