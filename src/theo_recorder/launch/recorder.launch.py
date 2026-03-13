from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    ros2bager = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "--start-paused",
            "--all"        
        ],
        cwd="/mnt/usb/bags",
        output="screen"
    )
    
    reorder_node=Node(
        namespace   = 'eowyn',
        package     = 'theo_recorder',
        executable  = 'recorder_node',
    )

    ld.add_action(ros2bager)
    ld.add_action(reorder_node)

    return ld
