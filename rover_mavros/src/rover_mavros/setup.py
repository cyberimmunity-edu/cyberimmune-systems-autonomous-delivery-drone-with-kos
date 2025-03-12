import os
from setuptools import find_packages, setup

package_name = 'rover_mavros'

def generate_data_files(source, target_subdir):
    data = []
    for dirpath, _, filenames in os.walk(source):
        if filenames:
            relative_dir = os.path.relpath(dirpath, source)
            dst_dir = os.path.join('share', package_name, target_subdir, relative_dir)
            file_list = [os.path.join(dirpath, f) for f in filenames]
            data.append((dst_dir, file_list))
    return data

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gazebo.launch.py']),
        ('share/' + package_name + '/launch', ['launch/bridges.launch.py']),
        ('share/' + package_name + '/launch', ['launch/mavros.launch.py']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
        ('share/' + package_name + '/launch', ['launch/lidar.launch.py']),
        ('share/' + package_name + '/launch', ['launch/combined.launch.py']),
    ] +
    generate_data_files('models', 'models') + 
    generate_data_files('worlds', 'worlds'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sensei',
    maintainer_email='sensei@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_controller = rover_mavros.rover_mavros:main',
            'camera = rover_mavros.camera:main',
            'lidar = rover_mavros.lidar:main'
        ],
    },
)
