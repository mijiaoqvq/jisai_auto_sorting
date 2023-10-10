import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='body',
            executable='body_node',
            emulate_tty=True,
        ),
        Node(
            package='eye',
            executable='camera_node',
            emulate_tty=True,
        ),
        Node(
            package='detect',
            executable='detect_node',
            emulate_tty=True,
            parameters=[
                {'model_path': os.path.join(get_package_share_directory('detect'),'best.pt')}
            ]
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([os.path.join(
                get_package_share_directory('rosbridge_server'), 'launch'),
                '/rosbridge_websocket_launch.xml']),
        ),
    ])
    
