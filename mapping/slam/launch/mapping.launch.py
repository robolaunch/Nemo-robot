# #!/usr/bin/env python3
# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     # SLAM
#     slam_toolbox_package_direction = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')

#     # SLAM Launch
#     slam_toolbox_command = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(slam_toolbox_package_direction, 'online_async_launch.py')
#         )
#     )

    

#     ld = LaunchDescription()

#     # Adding commands to launch description
#     ld.add_action(slam_toolbox_command)

#     return ld

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("bringup_robolaunch"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)

    return ld