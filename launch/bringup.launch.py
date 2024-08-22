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
            name='tank_control',
            executable='tank_control',
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='pan_tilt_camera',
            parameters=[config],
        ),
        Node(
            package='v4l2_camera',
            name='fisheye_camera',
            executable='v4l2_camera_node',
            parameters=[config],
            remappings=[
                ('/image_raw', '/fisheye_image_raw'),
                ('/image_raw/compressed', '/fisheye_image_raw/compressed'),
            ]
        ),
        Node(
            package='foxglove_bridge',
            name='foxglove_bridge',
            executable='foxglove_bridge',
        ),
    ])
