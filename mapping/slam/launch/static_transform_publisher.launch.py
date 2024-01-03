from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    base_footprint_to_base_link_node = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"])
                       

    base_link_to_base_scan_node = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0.27", "0", "0", "0", "base_link", "base_scan"])


    base_link_to_right_wheel_node = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "-0.2646", "0.085", "1.570796327", "0", "0", "base_link", "wheel_right_link"])

    base_link_to_left_wheel_node = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0.2646", "0.085", "-1.570796327", "0", "0", "base_link", "wheel_left_link"])


    ld.add_action(base_footprint_to_base_link_node)
    ld.add_action(base_link_to_base_scan_node)
    ld.add_action(base_link_to_right_wheel_node)
    ld.add_action(base_link_to_left_wheel_node)

    return ld