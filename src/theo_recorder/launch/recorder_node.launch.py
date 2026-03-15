from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    reorder_node=Node(
        namespace   = 'eowyn',
        package     = 'theo_recorder',
        executable  = 'recorder_node',
    )

    ld.add_action(reorder_node)

    return ld
