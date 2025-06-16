#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
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
    
    use_trajectory_server = DeclareLaunchArgument(
        'use_trajectory_server',
        default_value='false',
        description='Launch hector trajectory server if available'
    )
    
    
    # Calculate paths to ORB-SLAM3 resource files
    orb_slam3_ros_dir = get_package_share_directory('orb_slam3_ros')
    voc_file = os.path.join(orb_slam3_ros_dir, '/home/pfe/ros2_test/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.bin')
    settings_file = os.path.join(orb_slam3_ros_dir, '/home/pfe/ros2_test/src/orb_slam3_ros/config/RGB-D/RealSense_D435i.yaml')
    
    
    # Create ORB-SLAM3 node
    orb_slam3_node = Node(
        package='orb_slam3_ros',
        executable='ros_rgbd',
        name='orb_slam3',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'voc_file': voc_file,
            'settings_file': settings_file,
            'world_frame_id': 'world',
            'cam_frame_id': 'camera',
            'enable_pangolin': True,
            'queue_size': 1,
            'processing_threads': 4,
            'disable_loop_closing': True
        }],
        remappings=[
            ('/camera/rgb/image_raw', '/camera/realsense2_camera_node/color/image_raw'),
            ('/camera/depth_registered/image_raw', '/camera/realsense2_camera_node/depth/image_rect_raw')
        ],
        # For ROS 2, nice is set differently using a prefix process
        prefix=['nice -n -5']
    )
    
    # Trajectory server node (optional)
    trajectory_server_node = Node(
        package='hector_trajectory_server',
        executable='hector_trajectory_server',
        name='trajectory_server_orb_slam3',
        output='screen',
        namespace='orb_slam3',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_frame_name': 'world',
            'source_frame_name': 'camera',
            'trajectory_update_rate': 10.0,
            'trajectory_publish_rate': 10.0
        }],
        condition=IfCondition(LaunchConfiguration('use_trajectory_server'))
    )
    
    # Return the launch description
    return LaunchDescription(
        env_vars +
        [
            use_sim_time,
            use_trajectory_server,
            orb_slam3_node,
            trajectory_server_node
        ]
    )