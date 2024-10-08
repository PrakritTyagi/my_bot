import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node



def generate_launch_description():

    package_name='my_bot'

    pkg_path = os.path.join(get_package_share_directory(package_name))
    gazebo_params_file = os.path.join(pkg_path, 'config', 'gazebo_params.yaml')
    twist_mux_params = os.path.join(pkg_path,'config','twist_mux.yaml')
    rvizconfig = os.path.join(pkg_path, 'config','rviz2_config.rviz')


    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')]), 
                launch_arguments={'use_sim_time': 'true'}.items())
    
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )
    
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'world': os.path.join(pkg_path, 'worlds/duckytown.world'),
                                    'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items(),)

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', 
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
                        output='screen')

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_diff_cont',
        output='screen',
        arguments=['diff_cont']
    )


    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name='spawner_joint_broad',
        output='screen',
        arguments=['joint_broad']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        parameters=[twist_mux_params, {'use_sim_time': True}]
    )

        # Register event handler to wait for spawn_entity to exit before launching the controllers
    on_spawn_entity_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                diff_drive_spawner,
                joint_broad_spawner
            ]
        )
    )
    # Launch them all!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        on_spawn_entity_exit,
        # diff_drive_spawner,
        # joint_broad_spawner,
        rviz_node
    ])