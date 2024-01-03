# #!/usr/bin/env python3
# import os

# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration



# # https://answers.ros.org/question/401976/how-to-launch-nav2-without-prior-map/
# def generate_launch_description():
#     bringup_robolaunch_directory = get_package_share_directory('bringup_robolaunch')


#     map_yaml_file = LaunchConfiguration('map')
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     params_file = LaunchConfiguration('params_file')    

#     declare_params_file_cmd = DeclareLaunchArgument(
#         'params_file',
#         default_value=os.path.join(bringup_robolaunch_directory, 'config', 'nav2_params.yaml'),
#         description='Full path to the ROS2 parameters file to use for all launched nodes',
#     )

#     # Nav2 Bringup Package Directory
#     nav2_bringup_file_direction = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

#     # Nav2 Bringup Launch
#     nav2_bringup_command = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(nav2_bringup_file_direction, ('bringup_launch.py')),
#         ),
#         launch_arguments={'map': '/home/furkan/robolaunch_colcon_ws/src/bringup_robolaunch/maps/uberfactory/map.yaml',
#                           'use_sim_time': 'True',
#                           'params_file': params_file}.items()
#     )

#     ld = LaunchDescription()

#     ld.add_action(declare_params_file_cmd)
#     ld.add_action(nav2_bringup_command)

#     return ld


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('bringup_robolaunch'),
            'maps/office-v7',
            'map.yaml'
            ))

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('bringup_robolaunch'),
            'config',
            'nav2_params.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': 'false',
                'params_file': param_dir}.items(),
        ),

        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    arguments=['-d', rviz_config_dir],
        #    parameters=[{'use_sim_time': use_sim_time}],
        #    output='screen'),
    ])