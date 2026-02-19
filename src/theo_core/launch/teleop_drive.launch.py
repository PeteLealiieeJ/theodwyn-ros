from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    launch_dir = PathJoinSubstitution(
        [
            FindPackageShare('theo_core'), 
            'launch'
        ]
    )
    teleop_launch_dir = PathJoinSubstitution(
        [
            FindPackageShare('theo_teleop'), 
            'launch'
        ]
    )

    #  1. Launch File Deploying Mixing Interfaces
    #  2. Launch File Deploying Sabertooth Interfaces
    #  3. Launch File Deploying Teleoperation Interfaces
    #  4. Launch File Deploying Joystick Control Node

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    launch_dir, 
                    'mixer.launch.py'
                ]
            )
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    launch_dir, 
                    'sabertooth.launch.py'
                ]
            )
        ), 
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    teleop_launch_dir, 
                    'teleop.launch.py'
                ]
            )
        ),
        Node(
            package='joy',
            executable='joy_node',
            remappings=[
                ('/joy','/eowyn/joy')
            ]
        )
    ])
