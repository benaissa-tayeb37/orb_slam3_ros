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
    realsense2_camera_dir = get_package_share_directory('realsense2_camera')
    
    # Launch configuration variables for RealSense
    serial_no = LaunchConfiguration('serial_no')
    usb_port_id = LaunchConfiguration('usb_port_id')
    device_type = LaunchConfiguration('device_type')
    json_file_path = LaunchConfiguration('json_file_path')
    camera = LaunchConfiguration('camera')
    tf_prefix = LaunchConfiguration('tf_prefix')
    external_manager = LaunchConfiguration('external_manager')
    manager = LaunchConfiguration('manager')
    output = LaunchConfiguration('output')
    respawn = LaunchConfiguration('respawn')
    
    # Depth stream parameters
    depth_width = LaunchConfiguration('depth_width')
    depth_height = LaunchConfiguration('depth_height')
    enable_depth = LaunchConfiguration('enable_depth')
    depth_fps = LaunchConfiguration('depth_fps')
    
    # Infrared stream parameters
    infra_width = LaunchConfiguration('infra_width')
    infra_height = LaunchConfiguration('infra_height')
    enable_infra1 = LaunchConfiguration('enable_infra1')
    enable_infra2 = LaunchConfiguration('enable_infra2')
    infra_fps = LaunchConfiguration('infra_fps')
    
    # Color stream parameters
    color_width = LaunchConfiguration('color_width')
    color_height = LaunchConfiguration('color_height')
    enable_color = LaunchConfiguration('enable_color')
    color_fps = LaunchConfiguration('color_fps')
    
    # PointCloud parameters
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')
    pointcloud_texture_stream = LaunchConfiguration('pointcloud_texture_stream')
    pointcloud_texture_index = LaunchConfiguration('pointcloud_texture_index')
    allow_no_texture_points = LaunchConfiguration('allow_no_texture_points')
    ordered_pc = LaunchConfiguration('ordered_pc')
    
    # Synchronization and alignment
    enable_sync = LaunchConfiguration('enable_sync')
    align_depth = LaunchConfiguration('align_depth')
    
    # TF and odometry
    publish_tf = LaunchConfiguration('publish_tf')
    tf_publish_rate = LaunchConfiguration('tf_publish_rate')
    topic_odom_in = LaunchConfiguration('topic_odom_in')
    calib_odom_file = LaunchConfiguration('calib_odom_file')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    
    # Filters
    filters = LaunchConfiguration('filters')
    clip_distance = LaunchConfiguration('clip_distance')
    linear_accel_cov = LaunchConfiguration('linear_accel_cov')
    
    # Device timeout
    initial_reset = LaunchConfiguration('initial_reset')
    reconnect_timeout = LaunchConfiguration('reconnect_timeout')
    wait_for_device_timeout = LaunchConfiguration('wait_for_device_timeout')
    
    # Camera settings
    stereo_module_exposure_1 = LaunchConfiguration('stereo_module/exposure/1')
    stereo_module_gain_1 = LaunchConfiguration('stereo_module/gain/1')
    stereo_module_exposure_2 = LaunchConfiguration('stereo_module/exposure/2')
    stereo_module_gain_2 = LaunchConfiguration('stereo_module/gain/2')
    
    # Declare all launch arguments
    declare_serial_no_cmd = DeclareLaunchArgument(
        'serial_no',
        default_value='',
        description='Serial number of the camera')
        
    declare_usb_port_id_cmd = DeclareLaunchArgument(
        'usb_port_id',
        default_value='',
        description='USB port ID of the camera')
        
    declare_device_type_cmd = DeclareLaunchArgument(
        'device_type',
        default_value='',
        description='Type of the RealSense device')
        
    declare_json_file_path_cmd = DeclareLaunchArgument(
        'json_file_path',
        default_value='',
        description='Path to the JSON configuration file')
        
    declare_camera_cmd = DeclareLaunchArgument(
        'camera',
        default_value='camera',
        description='Camera name')
        
    declare_tf_prefix_cmd = DeclareLaunchArgument(
        'tf_prefix',
        default_value='camera',
        description='TF prefix for camera frames')
        
    declare_external_manager_cmd = DeclareLaunchArgument(
        'external_manager',
        default_value='false',
        description='Use external nodelet manager')
        
    declare_manager_cmd = DeclareLaunchArgument(
        'manager',
        default_value='realsense2_camera_manager',
        description='Name of the nodelet manager')
        
    declare_output_cmd = DeclareLaunchArgument(
        'output',
        default_value='screen',
        description='Output type')
        
    declare_respawn_cmd = DeclareLaunchArgument(
        'respawn',
        default_value='false',
        description='Respawn if the node crashes')
        
    # Depth stream parameters
    declare_depth_width_cmd = DeclareLaunchArgument(
        'depth_width',
        default_value='640',
        description='Depth image width')
        
    declare_depth_height_cmd = DeclareLaunchArgument(
        'depth_height',
        default_value='480',
        description='Depth image height')
        
    declare_enable_depth_cmd = DeclareLaunchArgument(
        'enable_depth',
        default_value='true',
        description='Enable depth stream')
        
    declare_depth_fps_cmd = DeclareLaunchArgument(
        'depth_fps',
        default_value='30',
        description='Depth stream FPS')
        
    # Infrared stream parameters
    declare_infra_width_cmd = DeclareLaunchArgument(
        'infra_width',
        default_value='640',
        description='Infrared image width')
        
    declare_infra_height_cmd = DeclareLaunchArgument(
        'infra_height',
        default_value='480',
        description='Infrared image height')
        
    declare_enable_infra1_cmd = DeclareLaunchArgument(
        'enable_infra1',
        default_value='true',
        description='Enable infrared stream 1')
        
    declare_enable_infra2_cmd = DeclareLaunchArgument(
        'enable_infra2',
        default_value='true',
        description='Enable infrared stream 2')
        
    declare_infra_fps_cmd = DeclareLaunchArgument(
        'infra_fps',
        default_value='30',
        description='Infrared stream FPS')
        
    # Color stream parameters
    declare_color_width_cmd = DeclareLaunchArgument(
        'color_width',
        default_value='640',
        description='Color image width')
        
    declare_color_height_cmd = DeclareLaunchArgument(
        'color_height',
        default_value='480',
        description='Color image height')
        
    declare_enable_color_cmd = DeclareLaunchArgument(
        'enable_color',
        default_value='true',
        description='Enable color stream')
        
    declare_color_fps_cmd = DeclareLaunchArgument(
        'color_fps',
        default_value='30',
        description='Color stream FPS')
        
    # PointCloud parameters
    declare_enable_pointcloud_cmd = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='true',
        description='Enable pointcloud')
        
    declare_pointcloud_texture_stream_cmd = DeclareLaunchArgument(
        'pointcloud_texture_stream',
        default_value='RS2_STREAM_COLOR',
        description='Texture stream for pointcloud')
        
    declare_pointcloud_texture_index_cmd = DeclareLaunchArgument(
        'pointcloud_texture_index',
        default_value='0',
        description='Texture index for pointcloud')
        
    declare_allow_no_texture_points_cmd = DeclareLaunchArgument(
        'allow_no_texture_points',
        default_value='false',
        description='Allow points with no texture')
        
    declare_ordered_pc_cmd = DeclareLaunchArgument(
        'ordered_pc',
        default_value='false',
        description='Ordered pointcloud')
        
    # Synchronization and alignment
    declare_enable_sync_cmd = DeclareLaunchArgument(
        'enable_sync',
        default_value='true',
        description='Enable synchronization')
        
    declare_align_depth_cmd = DeclareLaunchArgument(
        'align_depth',
        default_value='true',
        description='Align depth to color')
        
    # TF and odometry
    declare_publish_tf_cmd = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF')
        
    declare_tf_publish_rate_cmd = DeclareLaunchArgument(
        'tf_publish_rate',
        default_value='0',
        description='TF publish rate')
        
    declare_topic_odom_in_cmd = DeclareLaunchArgument(
        'topic_odom_in',
        default_value='odom_in',
        description='Topic for input odometry')
        
    declare_calib_odom_file_cmd = DeclareLaunchArgument(
        'calib_odom_file',
        default_value='',
        description='Odometry calibration file')
        
    declare_publish_odom_tf_cmd = DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='true',
        description='Publish odometry TF')
        
    # Filters
    declare_filters_cmd = DeclareLaunchArgument(
        'filters',
        default_value='',
        description='Filters to apply')
        
    declare_clip_distance_cmd = DeclareLaunchArgument(
        'clip_distance',
        default_value='2.0',
        description='Clip distance for the depth image')
        
    declare_linear_accel_cov_cmd = DeclareLaunchArgument(
        'linear_accel_cov',
        default_value='0.01',
        description='Linear acceleration covariance')
        
    # Device timeout
    declare_initial_reset_cmd = DeclareLaunchArgument(
        'initial_reset',
        default_value='false',
        description='Initial reset')
        
    declare_reconnect_timeout_cmd = DeclareLaunchArgument(
        'reconnect_timeout',
        default_value='6.0',
        description='Reconnect timeout')
        
    declare_wait_for_device_timeout_cmd = DeclareLaunchArgument(
        'wait_for_device_timeout',
        default_value='10.0',
        description='Wait for device timeout')
        
    # Camera settings
    declare_stereo_module_exposure_1_cmd = DeclareLaunchArgument(
        'stereo_module/exposure/1',
        default_value='7500',
        description='Exposure for stereo module 1')
        
    declare_stereo_module_gain_1_cmd = DeclareLaunchArgument(
        'stereo_module/gain/1',
        default_value='16',
        description='Gain for stereo module 1')
        
    declare_stereo_module_exposure_2_cmd = DeclareLaunchArgument(
        'stereo_module/exposure/2',
        default_value='1',
        description='Exposure for stereo module 2')
        
    declare_stereo_module_gain_2_cmd = DeclareLaunchArgument(
        'stereo_module/gain/2',
        default_value='16',
        description='Gain for stereo module 2')

    # RealSense node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=LaunchConfiguration('camera'),
        namespace=LaunchConfiguration('camera'),
        output=LaunchConfiguration('output'),
        parameters=[{
            'serial_no': serial_no,
            'usb_port_id': usb_port_id,
            'device_type': device_type,
            'json_file_path': json_file_path,
            'tf_prefix': tf_prefix,
            'enable_pointcloud': enable_pointcloud,
            'pointcloud_texture_stream': pointcloud_texture_stream,
            'pointcloud_texture_index': pointcloud_texture_index,
            'enable_sync': enable_sync,
            'align_depth': align_depth,
            'depth_width': depth_width,
            'depth_height': depth_height,
            'enable_depth': enable_depth,
            'depth_fps': depth_fps,
            'infra_width': infra_width,
            'infra_height': infra_height,
            'enable_infra1': enable_infra1,
            'enable_infra2': enable_infra2,
            'infra_fps': infra_fps,
            'color_width': color_width,
            'color_height': color_height,
            'enable_color': enable_color,
            'color_fps': color_fps,
            'publish_tf': publish_tf,
            'tf_publish_rate': tf_publish_rate,
            'filters': filters,
            'clip_distance': clip_distance,
            'linear_accel_cov': linear_accel_cov,
            'initial_reset': initial_reset,
            'reconnect_timeout': reconnect_timeout,
            'wait_for_device_timeout': wait_for_device_timeout,
            'topic_odom_in': topic_odom_in,
            'calib_odom_file': calib_odom_file,
            'publish_odom_tf': publish_odom_tf,
            'stereo_module.exposure.1': stereo_module_exposure_1,
            'stereo_module.gain.1': stereo_module_gain_1,
            'stereo_module.exposure.2': stereo_module_exposure_2,
            'stereo_module.gain.2': stereo_module_gain_2,
            'allow_no_texture_points': allow_no_texture_points,
            'ordered_pc': ordered_pc
        }]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add all launch arguments
    ld.add_action(declare_serial_no_cmd)
    ld.add_action(declare_usb_port_id_cmd)
    ld.add_action(declare_device_type_cmd)
    ld.add_action(declare_json_file_path_cmd)
    ld.add_action(declare_camera_cmd)
    ld.add_action(declare_tf_prefix_cmd)
    ld.add_action(declare_external_manager_cmd)
    ld.add_action(declare_manager_cmd)
    ld.add_action(declare_output_cmd)
    ld.add_action(declare_respawn_cmd)
    
    # Add depth parameters
    ld.add_action(declare_depth_width_cmd)
    ld.add_action(declare_depth_height_cmd)
    ld.add_action(declare_enable_depth_cmd)
    ld.add_action(declare_depth_fps_cmd)
    
    # Add infrared parameters
    ld.add_action(declare_infra_width_cmd)
    ld.add_action(declare_infra_height_cmd)
    ld.add_action(declare_enable_infra1_cmd)
    ld.add_action(declare_enable_infra2_cmd)
    ld.add_action(declare_infra_fps_cmd)
    
    # Add color parameters
    ld.add_action(declare_color_width_cmd)
    ld.add_action(declare_color_height_cmd)
    ld.add_action(declare_enable_color_cmd)
    ld.add_action(declare_color_fps_cmd)
    
    # Add pointcloud parameters
    ld.add_action(declare_enable_pointcloud_cmd)
    ld.add_action(declare_pointcloud_texture_stream_cmd)
    ld.add_action(declare_pointcloud_texture_index_cmd)
    ld.add_action(declare_allow_no_texture_points_cmd)
    ld.add_action(declare_ordered_pc_cmd)
    
    # Add sync and alignment parameters
    ld.add_action(declare_enable_sync_cmd)
    ld.add_action(declare_align_depth_cmd)
    
    # Add TF and odometry parameters
    ld.add_action(declare_publish_tf_cmd)
    ld.add_action(declare_tf_publish_rate_cmd)
    ld.add_action(declare_topic_odom_in_cmd)
    ld.add_action(declare_calib_odom_file_cmd)
    ld.add_action(declare_publish_odom_tf_cmd)
    
    # Add filter parameters
    ld.add_action(declare_filters_cmd)
    ld.add_action(declare_clip_distance_cmd)
    ld.add_action(declare_linear_accel_cov_cmd)
    
    # Add timeout parameters
    ld.add_action(declare_initial_reset_cmd)
    ld.add_action(declare_reconnect_timeout_cmd)
    ld.add_action(declare_wait_for_device_timeout_cmd)
    
    # Add camera settings
    ld.add_action(declare_stereo_module_exposure_1_cmd)
    ld.add_action(declare_stereo_module_gain_1_cmd)
    ld.add_action(declare_stereo_module_exposure_2_cmd)
    ld.add_action(declare_stereo_module_gain_2_cmd)
    
    # Add the node
    ld.add_action(realsense_node)
    
    return ld