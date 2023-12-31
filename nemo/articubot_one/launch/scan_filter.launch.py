
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

filter_config_path = PathJoinSubstitution(
        [FindPackageShare("articubot_one"), "config", "laser_filter.yaml"]
    )

def generate_launch_description():

    return LaunchDescription([

        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            parameters=[filter_config_path],
            # arguments=["--ros-args --remap scan:=scan1"]
            # remappings=[("scan_filtered",("scan_filtered_front"))]
        )
    ])


