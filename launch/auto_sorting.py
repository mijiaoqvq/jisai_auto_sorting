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
            executable='qr_code_node',
            emulate_tty=True,
        ),
        Node(
            package='body',
            executable='arm_node',
            emulate_tty=True,
        ),
        Node(
            package='body',
            executable='chassis_node',
            emulate_tty=True,
        ),
        Node(
            package='body',
            executable='find_device_node',
            emulate_tty=True,
        ),
        Node(
            package='eye',
            executable='camera_node',
            emulate_tty=True,
        ),
        Node(
            package='brain',
            executable='control_node',
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
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #         get_package_share_directory('realsense2_camera'), 'launch'),
        #         '/rs_t265_launch.py']),
        # ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([os.path.join(
                get_package_share_directory('rosbridge_server'), 'launch'),
                '/rosbridge_websocket_launch.xml']),
        ),
    ])
    
