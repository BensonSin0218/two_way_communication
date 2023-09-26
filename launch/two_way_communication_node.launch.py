from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os

package_name = "two_way_communication"

def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    param_path = os.path.join(
        get_package_share_directory(package_name),
        "config", "params.yaml")
    node = Node(package=package_name,
                executable="main",
                parameters=[param_path],
                output="screen")

    launch_description.add_action(node)

    return launch_description