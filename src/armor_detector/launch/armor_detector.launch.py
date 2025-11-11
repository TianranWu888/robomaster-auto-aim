from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='armor_detector',
            executable='armor_detector_node',
            name='armor_detector',
            output='screen'
        ),
    ])