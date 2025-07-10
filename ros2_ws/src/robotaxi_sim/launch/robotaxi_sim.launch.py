from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    desc_dir = get_package_share_directory('robotaxi_description')
    gazebo_launch_dir = get_package_share_directory('gazebo_ros')
    robotaxi_launch_dir = get_package_share_directory('robotaxi_launch')

    return LaunchDescription([
        # Gazebo server + client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_launch_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': os.path.join(desc_dir, 'worlds', 'test_track.world')}.items()
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robotaxi',
                       '-file', os.path.join(desc_dir, 'urdf', 'robotaxi.urdf'),
                       '-x', '0', '-y', '0', '-z', '0.05'],
            output='screen'
        ),

        # Bring up real robot stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robotaxi_launch_dir, 'robotaxi_launch.py')
            )
        ),
    ]) 