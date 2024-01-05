#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='forward_kinematics',
            executable='forward_kinematics',
            name='forward_kinematics_node',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('forward_kinematics'), 
                'config', 'config.yaml')])
    ])