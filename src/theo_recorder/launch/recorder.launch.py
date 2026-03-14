from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    launch_dir = PathJoinSubstitution(
        [
            FindPackageShare('theo_recorder'), 
            'launch'
        ]
    )

    #  1. Launch ROS bag
    #  2. Launch Recorder Node and underlying Interfaces

    return LaunchDescription([
        ExecuteProcess(
            cmd     =   [
                "ros2", 
                "bag", 
                "record",
                "--start-paused",
                "-a"
            ],
            cwd     =   "/mnt/usb/",
            output  =   "screen",
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [
                    launch_dir, 
                    'recorder_node.launch.py'
                ]
            )
        )
    ])