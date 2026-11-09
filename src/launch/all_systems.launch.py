import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_turtlebot4_gz = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_turtlebot4_viz = get_package_share_directory('turtlebot4_viz')
    pkg_rob_patroller = get_package_share_directory('rob_patroller')

    map_file_path = os.path.join(pkg_rob_patroller, 'maps', 'maps.yaml')


    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_gz, 'launch', 'turtlebot4_gz.launch.py')
        ),
        launch_arguments={
            'model': 'lite',
            'world': 'depot',
            'use_sim_time': 'true'
        }.items()
    )

    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_viz, 'launch', 'view_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    spawn_objects_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rob_patroller, 'launch', 'spawn_objects.launch.py')
        )
    )

    delayed_viz = TimerAction(
        period=5.0,
        actions=[viz_launch]
    )

    delayed_spawn = TimerAction(
        period=15.0, 
        actions=[spawn_objects_launch]
    )

    return LaunchDescription([
        sim_launch,
        delayed_viz,
        delayed_spawn
    ])
