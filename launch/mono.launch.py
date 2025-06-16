#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    orb_slam3_ros_dir = get_package_share_directory('orb_slam3_ros')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    voc_file = LaunchConfiguration('voc_file')
    settings_file = LaunchConfiguration('settings_file')
    world_frame_id = LaunchConfiguration('world_frame_id')
    cam_frame_id = LaunchConfiguration('cam_frame_id')
    enable_pangolin = LaunchConfiguration('enable_pangolin')
    camera_topic = LaunchConfiguration('camera_topic')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    declare_voc_file_cmd = DeclareLaunchArgument(
        'voc_file',
        default_value=os.path.expanduser('~/ros2_test/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.bin'),
        description='Path to vocabulary file')
        
    declare_settings_file_cmd = DeclareLaunchArgument(
        'settings_file',
        default_value=os.path.expanduser('~/ros2_test/src/orb_slam3_ros/orb_slam3/config/Monocular/EuRoC.yaml'),
        description='Path to settings file')
        
    declare_world_frame_cmd = DeclareLaunchArgument(
        'world_frame_id',
        default_value='map',
        description='Frame ID for world coordinate system')
        
    declare_cam_frame_cmd = DeclareLaunchArgument(
        'cam_frame_id',
        default_value='camera',
        description='Frame ID for camera')
        
    declare_enable_pangolin_cmd = DeclareLaunchArgument(
        'enable_pangolin',
        default_value='true',  # Changed from 'true' to 'false'
        description='Enable Pangolin visualization')


    declare_camera_topic_cmd = DeclareLaunchArgument(
        'camera_topic',
        default_value='/usb_cam/image_raw',  # Changed to match the actual topic
        description='Camera topic for ORB-SLAM3')
    
    # Get the USB camera launch file
    usb_cam_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('usb_cam'), 'launch', 'camera.launch.py'])]
        )
    )
    
    # ORB-SLAM3 Mono Node
    orb_slam3_mono_node = Node(
        package='orb_slam3_ros',
        executable='ros_mono',
        name='orb_slam3_mono',  # Changed the node name to avoid duplication
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'voc_file': voc_file,
            'settings_file': settings_file,
            'world_frame_id': world_frame_id,
            'cam_frame_id': cam_frame_id,
            'enable_pangolin': enable_pangolin,
        }],
        remappings=[
            # Changed from '/camera/image_raw' to match what the camera node is actually publishing
            ('/camera/image_raw', camera_topic),
        ]
    )
    
    # Add a static transform publisher between camera optical frame and base frame
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera'],
        output='screen'
    )
    



    # RViz node
    rviz_config_file = os.path.join(orb_slam3_ros_dir, 'config', 'orb_slam3_mono.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Declare launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_voc_file_cmd)
    ld.add_action(declare_settings_file_cmd)
    ld.add_action(declare_world_frame_cmd)
    ld.add_action(declare_cam_frame_cmd)
    ld.add_action(declare_enable_pangolin_cmd)
    ld.add_action(declare_camera_topic_cmd)
    # Add this to the launch description
    ld.add_action(static_transform_publisher)   
    # Add nodes to launch description
    ld.add_action(usb_cam_launch_include)
    ld.add_action(orb_slam3_mono_node)
    ld.add_action(rviz_node)
    
    return ld