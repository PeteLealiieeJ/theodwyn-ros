import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    handler_config = os.path.join(
        get_package_share_directory('theo_core'),
        'config',
        'servo_handler_node.yaml'
    )

    responder_config = os.path.join(
        get_package_share_directory('theo_core'),
        'config',
        'servo_responder_node.yaml'
    )
        
    handler_node=Node(
        namespace   = 'eowyn',
        package     = 'theo_core',
        executable  = 'servo_handler_node.py',
        parameters  = [handler_config]
    )

    responder_node=Node(
        namespace   = 'eowyn',
        package     = 'theo_core',
        executable  = 'servo_responder_node',
        parameters  = [responder_config]
    )

    ld.add_action(handler_node)
    ld.add_action(responder_node)

    return ld
