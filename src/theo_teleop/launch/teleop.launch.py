from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    launch_dir = PathJoinSubstitution(
        [
            FindPackageShare('theo_teleop'), 
            'launch'
        ]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    launch_dir, 
                    'joymapping.launch.py'
                ]
            )
        )
    ])
