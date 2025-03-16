from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14571@localhost:14571',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
                'fcu_port': '/dev/ttyUSB0',
                'fcu_baud': 57600,
                'system_id': 1,
                'component_id': 1,
                'log_output': 'screen',
                'log_level': 'info',
            }],
        ),
    ])