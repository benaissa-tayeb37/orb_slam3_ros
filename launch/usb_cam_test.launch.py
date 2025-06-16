from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},  # Changed from mjpeg to yuyv
                {'framerate': 30.0},
                {'camera_name': 'test_camera2'},
                {'camera_info_url': 'file:///home/pfe/ros2_test/src/orb_slam3_ros/config/usb_cam_info.yaml'},
                {'autoexposure': True},  # Added
                {'autofocus': False}  # Added
            ],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info')
            ],
            output='screen'
        ),
        Node(
            package='image_view',
            executable='image_view',
            name='image_view',
            parameters=[{'autosize': True}],
            remappings=[
                ('/image', '/camera/image_raw')
            ]
        )
    ])