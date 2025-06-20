#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare all launch arguments
    declared_arguments = [
        # Basic camera settings
        DeclareLaunchArgument('serial_no', default_value=''),
        DeclareLaunchArgument('usb_port_id', default_value=''),
        DeclareLaunchArgument('device_type', default_value=''),
        DeclareLaunchArgument('json_file_path', default_value=''),
        DeclareLaunchArgument('camera', default_value='camera'),
        DeclareLaunchArgument('tf_prefix', default_value=LaunchConfiguration('camera')),
        DeclareLaunchArgument('external_manager', default_value='false'),
        DeclareLaunchArgument('output', default_value='screen'),
        DeclareLaunchArgument('respawn', default_value='false'),
        
        # Stream settings
        DeclareLaunchArgument('fisheye_width', default_value='-1'),
        DeclareLaunchArgument('fisheye_height', default_value='-1'),
        DeclareLaunchArgument('enable_fisheye', default_value='false'),
        
        DeclareLaunchArgument('depth_width', default_value='640'),
        DeclareLaunchArgument('depth_height', default_value='480'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        
        DeclareLaunchArgument('confidence_width', default_value='-1'),
        DeclareLaunchArgument('confidence_height', default_value='-1'),
        DeclareLaunchArgument('enable_confidence', default_value='true'),
        DeclareLaunchArgument('confidence_fps', default_value='-1'),
        
        DeclareLaunchArgument('infra_width', default_value='640'),
        DeclareLaunchArgument('infra_height', default_value='480'),
        DeclareLaunchArgument('enable_infra', default_value='false'),
        DeclareLaunchArgument('enable_infra1', default_value='false'),
        DeclareLaunchArgument('enable_infra2', default_value='false'),
        DeclareLaunchArgument('infra_rgb', default_value='false'),
        
        DeclareLaunchArgument('color_width', default_value='640'),
        DeclareLaunchArgument('color_height', default_value='480'),
        DeclareLaunchArgument('enable_color', default_value='true'),
        
        # FPS settings
        DeclareLaunchArgument('fisheye_fps', default_value='-1'),
        DeclareLaunchArgument('depth_fps', default_value='30'),
        DeclareLaunchArgument('infra_fps', default_value='30'),
        DeclareLaunchArgument('color_fps', default_value='30'),
        DeclareLaunchArgument('gyro_fps', default_value='-1'),
        DeclareLaunchArgument('accel_fps', default_value='-1'),
        DeclareLaunchArgument('enable_gyro', default_value='false'),
        DeclareLaunchArgument('enable_accel', default_value='false'),
        
        # Pointcloud settings
        DeclareLaunchArgument('enable_pointcloud', default_value='false'),
        DeclareLaunchArgument('pointcloud_texture_stream', default_value='RS2_STREAM_COLOR'),
        DeclareLaunchArgument('pointcloud_texture_index', default_value='0'),
        DeclareLaunchArgument('allow_no_texture_points', default_value='false'),
        DeclareLaunchArgument('ordered_pc', default_value='false'),
        
        # Processing settings
        DeclareLaunchArgument('enable_sync', default_value='true'),
        DeclareLaunchArgument('align_depth', default_value='true'),
        
        # TF settings
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('tf_publish_rate', default_value='0'),
        
        # Advanced settings
        DeclareLaunchArgument('filters', default_value=''),
        DeclareLaunchArgument('clip_distance', default_value='-2'),
        DeclareLaunchArgument('linear_accel_cov', default_value='0.01'),
        DeclareLaunchArgument('initial_reset', default_value='false'),
        DeclareLaunchArgument('reconnect_timeout', default_value='6.0'),
        DeclareLaunchArgument('wait_for_device_timeout', default_value='-1.0'),
        DeclareLaunchArgument('topic_odom_in', default_value='odom_in'),
        DeclareLaunchArgument('calib_odom_file', default_value=''),
        DeclareLaunchArgument('publish_odom_tf', default_value='true'),
        DeclareLaunchArgument('hold_back_imu_for_frames', default_value='false'),
        
        # Stereo module settings
        DeclareLaunchArgument('stereo_module/exposure/1', default_value='7500'),
        DeclareLaunchArgument('stereo_module/gain/1', default_value='16'),
        DeclareLaunchArgument('stereo_module/exposure/2', default_value='1'),
        DeclareLaunchArgument('stereo_module/gain/2', default_value='16'),
    ]

    # Create the realsense node with all parameters
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=LaunchConfiguration('camera'),
        name='realsense2_camera_node',
        output=LaunchConfiguration('output'),
        respawn=LaunchConfiguration('respawn'),
        parameters=[
            {
                'tf_prefix': LaunchConfiguration('tf_prefix'),
                'serial_no': LaunchConfiguration('serial_no'),
                'usb_port_id': LaunchConfiguration('usb_port_id'),
                'device_type': LaunchConfiguration('device_type'),
                'json_file_path': LaunchConfiguration('json_file_path'),

                'enable_pointcloud': LaunchConfiguration('enable_pointcloud'),
                'pointcloud_texture_stream': LaunchConfiguration('pointcloud_texture_stream'),
                'pointcloud_texture_index': LaunchConfiguration('pointcloud_texture_index'),
                'enable_sync': LaunchConfiguration('enable_sync'),
                'align_depth': LaunchConfiguration('align_depth'),

                'fisheye_width': LaunchConfiguration('fisheye_width'),
                'fisheye_height': LaunchConfiguration('fisheye_height'),
                'enable_fisheye': LaunchConfiguration('enable_fisheye'),

                'depth_width': LaunchConfiguration('depth_width'),
                'depth_height': LaunchConfiguration('depth_height'),
                'enable_depth': LaunchConfiguration('enable_depth'),

                'confidence_width': LaunchConfiguration('confidence_width'),
                'confidence_height': LaunchConfiguration('confidence_height'),
                'enable_confidence': LaunchConfiguration('enable_confidence'),
                'confidence_fps': LaunchConfiguration('confidence_fps'),

                'color_width': LaunchConfiguration('color_width'),
                'color_height': LaunchConfiguration('color_height'),
                'enable_color': LaunchConfiguration('enable_color'),

                'infra_width': LaunchConfiguration('infra_width'),
                'infra_height': LaunchConfiguration('infra_height'),
                'enable_infra': LaunchConfiguration('enable_infra'),
                'enable_infra1': LaunchConfiguration('enable_infra1'),
                'enable_infra2': LaunchConfiguration('enable_infra2'),
                'infra_rgb': LaunchConfiguration('infra_rgb'),

                'fisheye_fps': LaunchConfiguration('fisheye_fps'),
                'depth_fps': LaunchConfiguration('depth_fps'),
                'infra_fps': LaunchConfiguration('infra_fps'),
                'color_fps': LaunchConfiguration('color_fps'),
                'gyro_fps': LaunchConfiguration('gyro_fps'),
                'accel_fps': LaunchConfiguration('accel_fps'),
                'enable_gyro': LaunchConfiguration('enable_gyro'),
                'enable_accel': LaunchConfiguration('enable_accel'),

                'publish_tf': LaunchConfiguration('publish_tf'),
                'tf_publish_rate': LaunchConfiguration('tf_publish_rate'),

                'filters': LaunchConfiguration('filters'),
                'clip_distance': LaunchConfiguration('clip_distance'),
                'linear_accel_cov': LaunchConfiguration('linear_accel_cov'),
                'initial_reset': LaunchConfiguration('initial_reset'),
                'reconnect_timeout': LaunchConfiguration('reconnect_timeout'),
                'wait_for_device_timeout': LaunchConfiguration('wait_for_device_timeout'),
                'topic_odom_in': LaunchConfiguration('topic_odom_in'),
                'calib_odom_file': LaunchConfiguration('calib_odom_file'),
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
                'stereo_module.exposure.1': LaunchConfiguration('stereo_module/exposure/1'),
                'stereo_module.gain.1': LaunchConfiguration('stereo_module/gain/1'),
                'stereo_module.exposure.2': LaunchConfiguration('stereo_module/exposure/2'),
                'stereo_module.gain.2': LaunchConfiguration('stereo_module/gain/2'),

                'allow_no_texture_points': LaunchConfiguration('allow_no_texture_points'),
                'ordered_pc': LaunchConfiguration('ordered_pc'),
            }
        ]
    )

    # Create launch description with all arguments and nodes
    return LaunchDescription(
        declared_arguments + 
        [
            GroupAction(
                actions=[realsense_node],
                scoped=True
            )
        ]
    )