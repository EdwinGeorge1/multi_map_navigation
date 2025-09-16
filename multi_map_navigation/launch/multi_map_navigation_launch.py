from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tb3_description_dir = get_package_share_directory('turtlebot3_description')

    # Get TURTLEBOT3_MODEL from environment, default to 'burger'
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    urdf_file_name = f"turtlebot3_{turtlebot3_model}.urdf"
    urdf_path = os.path.join(tb3_description_dir, 'urdf', urdf_file_name)

    # Path to your map
    map_file = os.path.expanduser('~/ros2_ws/src/multi_map_navigation/maps/room1.yaml')

    return LaunchDescription([
        # Launch Gazebo with empty world (can replace with custom .world file)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_dir, 'launch', 'empty_world.launch.py')
            ),
            launch_arguments={
                'world': os.path.expanduser('~/ros2_ws/src/multi_map_navigation/worlds/room1.world')
            }.items()
        ),

        # Spawn TurtleBot3
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'turtlebot3',
                 '-file', urdf_path],
            output='screen'
        ),

        # Launch Nav2 with your map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
                'autostart': 'true'
            }.items()
        ),
    ])

