import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    params = os.path.join(
        get_package_share_directory('esp32_uwb'),
        'config',
        'example_params.yaml'
        )
        
    node=Node(
        package = 'esp32_uwb',
        # name = 'esp32_uwb_publisher',
        executable = 'esp32_uwb_publisher',
        parameters = [params],
        remappings = [
                # ('distpozyx', ''),
                # ('odomPozyx', '')
        ],
        output='screen',
        emulate_tty=True,
    )
    
    ld.add_action(node)
    
    return ld