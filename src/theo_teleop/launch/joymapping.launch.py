import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    jmap_config = os.path.join(
        get_package_share_directory('theo_teleop'),
        'config',
        'joymapping_node.yaml'
    )
        
    node=Node(
        namespace   = 'eowyn',
        package     = 'theo_teleop',
        executable  = 'joymapping_node',
        parameters  = [jmap_config]
    )

    ld.add_action(node)
    
    return ld