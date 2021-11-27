from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sphero_node',
            namespace='sphero',
            executable='sphero_node',
            name='sphero'
        ),
    ])