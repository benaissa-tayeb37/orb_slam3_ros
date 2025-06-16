#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
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
    queue_size = LaunchConfiguration('queue_size')
    buffer_size = LaunchConfiguration('buffer_size')
    process_rate = LaunchConfiguration('process_rate')
    
    # Declare the launch arguments for ORB-SLAM3
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    declare_voc_file_cmd = DeclareLaunchArgument(
        'voc_file',
        default_value='/home/pfe/ros2_test/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.bin',
        description='Path to vocabulary file')
        
    declare_settings_file_cmd = DeclareLaunchArgument(
        'settings_file',
        default_value='/home/pfe/ros2_test/src/orb_slam3_ros/config/Stereo/RealSense_D435i_Optimized.yaml',
        description='Path to settings file')
        
    declare_world_frame_id_cmd = DeclareLaunchArgument(
        'world_frame_id',
        default_value='world',
        description='Frame ID for world coordinate system')
        
    declare_cam_frame_id_cmd = DeclareLaunchArgument(
        'cam_frame_id',
        default_value='camera',
        description='Frame ID for camera')
        
    declare_enable_pangolin_cmd = DeclareLaunchArgument(
        'enable_pangolin',
        default_value='false',
        description='Enable Pangolin visualization')
        
    declare_queue_size_cmd = DeclareLaunchArgument(
        'queue_size',
        default_value='1',
        description='Queue size for ROS topics')
        
    declare_buffer_size_cmd = DeclareLaunchArgument(
        'buffer_size',
        default_value='1',
        description='Buffer size')
        
    declare_process_rate_cmd = DeclareLaunchArgument(
        'process_rate',
        default_value='30',
        description='Processing rate')
    
    # Include the RealSense launch file with optimized parameters
    realsense_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_color': 'false',
            'enable_depth': 'true',
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'enable_pointcloud': 'false',
            'enable_sync': 'true',
            'align_depth': 'false',
            'depth_width': '848',
            'depth_height': '480',
            'depth_fps': '15',
            'infra_width': '848',
            'infra_height': '480',
            'infra_fps': '15',
            'stereo_module.exposure.1': '3000',
            'stereo_module.gain.1': '16',
            'stereo_module.exposure.2': '3000',
            'stereo_module.gain.2': '16',
            'tf_publish_rate': '0.0',  # Changed from integer to double
            'json_file_path': '/home/pfe/ros2_test/src/orb_slam3_ros/config/d435_low_latency.json'
        }.items()
    )
    
    # Create a group action for the topic remappings
    remappings = [
        ('/camera/left/image_raw', '/camera/camera/infra1/image_rect_raw'),
        ('/camera/right/image_raw', '/camera/camera/infra2/image_rect_raw')
    ]
    
    # ORB-SLAM3 Stereo Node
    orb_slam3_stereo_node = Node(
        package='orb_slam3_ros',
        executable='ros_stereo',
        name='orb_slam3',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'voc_file': voc_file,
            'settings_file': settings_file,
            'world_frame_id': world_frame_id,
            'cam_frame_id': cam_frame_id,
            'enable_pangolin': enable_pangolin,
            'queue_size': queue_size,
            'buffer_size': buffer_size,
            'process_rate': process_rate
        }],
        remappings=remappings
    )
    
    # RViz configuration
    rviz_config_file = os.path.join(orb_slam3_ros_dir, '/home/pfe/ros2_test/src/orb_slam3_ros/config/orb_slam3_stereo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Add a static transform publisher between world frame and camera frame
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', world_frame_id, cam_frame_id],
        output='screen'
    )
    
    # Create and return launch description
    ld = LaunchDescription()
    
    # Declare launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_voc_file_cmd)
    ld.add_action(declare_settings_file_cmd)
    ld.add_action(declare_world_frame_id_cmd)
    ld.add_action(declare_cam_frame_id_cmd)
    ld.add_action(declare_enable_pangolin_cmd)
    ld.add_action(declare_queue_size_cmd)
    ld.add_action(declare_buffer_size_cmd)
    ld.add_action(declare_process_rate_cmd)
    
    # Add static transform publisher
    ld.add_action(static_transform_publisher)
    
    # Add nodes and includes to launch description
    ld.add_action(realsense_launch_include)
    ld.add_action(orb_slam3_stereo_node)
    ld.add_action(rviz_node)
    
    return ld