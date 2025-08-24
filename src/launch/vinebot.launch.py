from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vinebot',
            executable='navigation_node',
            name='navigation_node'
        ),
        Node(
            package='vinebot',
            executable='camera_node',
            name='camera_node'
        )
    ])
