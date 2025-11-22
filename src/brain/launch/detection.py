from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_opencv',
            executable='aruco_tracker_autostart',
            name='aruco_tracker',
            parameters=[{
                'marker_dict': 'ARUCO_ORIGINAL',
                'camera_info_topic': '/camera/camera_info',
            }],
            remappings=[
                ('/camera/image_raw', '/camera/image'),
            ],
            output='screen'
        )
    ])