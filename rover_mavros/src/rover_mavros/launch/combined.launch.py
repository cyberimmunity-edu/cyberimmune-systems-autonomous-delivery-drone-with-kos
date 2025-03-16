import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_mavros')

    bridges_launch_path = os.path.join(pkg_share, 'launch', 'bridges.launch.py')
    mavros_launch_path = os.path.join(pkg_share, 'launch', 'mavros.launch.py')
    gazebo_launch_path = os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
    camera_launch_path = os.path.join(pkg_share, 'launch', 'camera.launch.py')
    lidar_launch_path = os.path.join(pkg_share, 'launch', 'lidar.launch.py')
    
    declare_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='false',
        description='Launch camera node?'
    )
    declare_lidar_arg = DeclareLaunchArgument(
        'launch_lidar',
        default_value='false',
        description='Launch lidar node?'
    )
    
    include_bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridges_launch_path)
    )
    include_mavros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mavros_launch_path)
    )
    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )
    include_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path),
        condition=IfCondition(LaunchConfiguration('launch_camera'))
    )
    include_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        condition=IfCondition(LaunchConfiguration('launch_lidar'))
    )

    return LaunchDescription([
        declare_camera_arg,
        declare_lidar_arg,
        include_bridges,
        include_mavros,
        include_gazebo,
        include_camera,
        include_lidar
    ])
