#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set environment variables
    env_vars = [
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ]
    
    # Declare all launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Calculate paths to ORB-SLAM3 resource files
    orb_slam3_ros_dir = get_package_share_directory('orb_slam3_ros')
    voc_file = os.path.join(orb_slam3_ros_dir, '/home/xion/ros2_test/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.bin')
    settings_file = os.path.join(orb_slam3_ros_dir, '/home/xion/ros2_test/src/orb_slam3_ros/config/RGB-D/RealSense_D435i.yaml')
    
    # Create ORB-SLAM3 node with Pangolin enabled
    orb_slam3_node = Node(
        package='orb_slam3_ros',
        executable='ros_rgbd',
        name='orb_slam3_vis',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'voc_file': voc_file,
            'settings_file': settings_file,
            'world_frame_id': 'map',
            'cam_frame_id': 'camera',
            'enable_pangolin': True,  # Always enable Pangolin in this launch file
            'queue_size': 1,
            'processing_threads': 4,
            'disable_loop_closing': True
        }],
        remappings=[
            ('/camera/rgb/image_raw', '/camera/realsense2_camera_node/color/image_raw'),
            ('/camera/depth_registered/image_raw', '/camera/realsense2_camera_node/depth/image_rect_raw')
        ]
    )
    
    # Return the launch description
    return LaunchDescription(
        env_vars +
        [
            use_sim_time,
            orb_slam3_node
        ]
    )