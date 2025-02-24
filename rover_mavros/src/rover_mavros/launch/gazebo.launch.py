import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_share = get_package_share_directory('rover_mavros')
    local_models_path = os.path.join(package_share, 'models')
    local_worlds_path = os.path.join(package_share, 'worlds')
    print(local_models_path)
    print(local_worlds_path)

    resource_path = f'{local_models_path}:{local_worlds_path}'

    env_setup = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    gz_sim_launch_path = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')

    include_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={'gz_args': '-r iris_runway.sdf --render-engine ogre'}.items()
    )

    return LaunchDescription([
        env_setup,
        include_gz_sim,
    ])