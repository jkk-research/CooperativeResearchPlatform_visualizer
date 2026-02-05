from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    visualizer = Node(
        package="visualizer",
        executable="visualizer",
        name="visualizer",
        output="screen"
    )

    return LaunchDescription([
        visualizer
    ])
