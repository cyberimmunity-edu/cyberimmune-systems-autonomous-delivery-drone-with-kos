import os
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess

def spawn_entity(model_file, entity_name, x, y, z):
    world = 'iris_runway'
    cmds = []
    for idx in range(len(model_file)):
        spawn_cmd = ExecuteProcess(
            cmd=[
            'ros2', 'launch', 'ros_gz_sim', 'gz_spawn_model.launch.py',
            'world:=' + world,
            'file:=' + model_file[idx],
            'entity_name:=' + entity_name[idx],
            'x:=' + x[idx],
            'y:=' + y[idx],
            'z:=' + z[idx]
            ],
            output='screen'
        )
        cmds.append(spawn_cmd)
    ld = LaunchDescription(cmds)
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

current_directory = os.getcwd() 
models = [
    f'{current_directory}/models/red_cube_long_1/model.sdf',
    f'{current_directory}/models/red_cube_long_2/model.sdf',
    f'{current_directory}/models/red_cube_long_3/model.sdf',
    f'{current_directory}/models/red_cube_big/model.sdf'
]

names = [
    'red_cube_long_1',
    'red_cube_long_2',
    'red_cube_long_3',
    'red_cube_big'
]

x = [
    '2.0',
    '10.0',
    '6.0',
    '6.0'
]

y = [
    '4.0',
    '4.0',
    '2.0',
    '6.0'
]

z = [
    '0.5',
    '0.5',
    '0.5',
    '0.5'
]

spawn_entity(models, names, x, y, z)