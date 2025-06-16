from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get the USB camera launch file
    usb_cam_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('usb_cam'), 'launch', 'camera.launch.py'])]
        )
    )
    
    # Define the ORB-SLAM3 C++ node
    mono_node_cpp = Node(
        package='orb_slam3_ros',
        executable='mono_node_cpp',
        name='mono_slam_cpp',
        output='screen',
        parameters=[{
            'node_name_arg': 'mono_slam_cpp',
            # Automatically find paths for voc and settings files
            'voc_file_arg': os.path.expanduser('~/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin'),
            'settings_file_path_arg': os.path.expanduser('~/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Monocular/')
        }]
    )
    
    # Define the Python driver node
    real_time_mono_node = Node(
        package='ros2_orb_slam3',
        executable='RealTimeMonoNode.py',
        name='real_time_mono_node',
        output='screen',
        parameters=[{
            'camera_topic': '/usb_cam/image_raw',
            'camera_info_topic': '/usb_cam/camera_info',
            'settings_name': 'EuRoC',
            'workspace_path': os.path.expanduser('~/ros2_test/src/ros2_orb_slam3')
        }]
    )
    
    # Return the launch description
    return LaunchDescription([
        usb_cam_launch_include,
        mono_node_cpp,
        real_time_mono_node
    ])