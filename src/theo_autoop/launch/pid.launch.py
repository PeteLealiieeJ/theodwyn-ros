import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pid_controller_config = os.path.join(
        get_package_share_directory('theo_autoop'),
        'config',
        'pid_controller_node.yaml'
    )
        
    node=Node(
        namespace   = 'eowyn',
        package     = 'theo_autoop',
        executable  = 'pid_controller_node',
        parameters  = [pid_controller_config]
    )

    ld.add_action(node)
    
    return ld