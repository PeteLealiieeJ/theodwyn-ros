import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    file_path_arg = DeclareLaunchArgument(
        'transmit_file_path',
        default_value=TextSubstitution(text=""), # Optional: set a default value or leave empty
        description='Full path to the transmission csv'
    )
    file_path = LaunchConfiguration('transmit_file_path')

    transmitter_config = os.path.join(
        get_package_share_directory('theo_comm'),
        'config',
        'transmitter_node.yaml'
    )
        
    node=Node(
        namespace   = '/eowyn/external',
        package     = 'theo_comm',
        executable  = 'transmitter_node',
        parameters  = [transmitter_config,{'filename':file_path}]
    )

    ld.add_action(file_path_arg)
    ld.add_action(node)

    
    return ld