from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_mavros',
            executable='camera',
            name='camera_node',
            output='screen'
        ),
    ])


