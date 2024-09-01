import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dadbot'),
        'config',
        'v4l2_camera_params.yaml'
        )

    return LaunchDescription([
        # Node(
        #    package='dadbot',
        #    name='yahboom_g1_teleop',
        #    executable='yahboom_g1_teleop',
        # ),
        Node(
            package='dadbot',
            name='servo_controller',
            executable='servo_controller',
            remappings=[
                ('/cmd_vel', '/servo/cmd_vel'),
            ]
        ),
        #Node(
        #    package='dadbot',
        #    name='tank_control',
        #    executable='tank_control',
        #),
        Node(
            package='dadbot',
            executable='webcam_pub',
            name='pan_tilt_camera',
            parameters=[config]
        ),
        Node(
            package='dadbot',
            executable='webcam_pub',
            name='fisheye_camera',
            parameters=[config],
            remappings=[
                ('/image_raw', '/fisheye_image_raw'),
                ('/image_raw/compressed', '/fisheye_image_raw/compressed'),
            ]
        ),
        Node(
            package='dadbot',
            name='object_detection',
            executable='object_detection',
        ),
        # Node(
        #    package='dadbot',
        #    name='fisheye_object_detection',
        #    executable='fisheye_object_detection',
        # ),
        Node(
            package='rplidar_ros',
            name='rplidar_node',
            executable='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Express',
            }],
            output='screen'
        )
    ])
