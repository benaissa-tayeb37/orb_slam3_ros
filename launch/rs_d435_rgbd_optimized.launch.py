#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Set environment variables for performance
    env_vars = [
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
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
    
    # ORB-SLAM3 parameters - reduced processing settings
    enable_pangolin = DeclareLaunchArgument(
        'enable_pangolin',
        default_value='false',  # Default to disabled for better performance
        description='Enable Pangolin visualization (increases CPU usage)'
    )
    
    processing_threads = DeclareLaunchArgument(
        'processing_threads',
        default_value='4',  # Adjust based on your CPU cores
        description='Number of processing threads for ORB-SLAM3'
    )
    
    disable_loop_closing = DeclareLaunchArgument(
        'disable_loop_closing',
        default_value='true',  # Disabling reduces CPU usage at expense of drift
        description='Disable loop closing (reduces CPU usage)'
    )
    
    # RealSense camera launch arguments - optimized for performance
    camera_name = DeclareLaunchArgument(
        'camera',
        default_value='camera',
        description='Camera name'
    )
    
    # Reduced resolution for better performance
    depth_width = DeclareLaunchArgument(
        'depth_width',
        default_value='424',  # D435 native resolution
        description='Depth image width'
    )
    
    depth_height = DeclareLaunchArgument(
        'depth_height',
        default_value='240',  # Reduced height for better performance
        description='Depth image height'
    )
    
    color_width = DeclareLaunchArgument(
        'color_width',
        default_value='424',  # Reduced width for better performance
        description='Color image width'
    )
    
    color_height = DeclareLaunchArgument(
        'color_height',
        default_value='240',  # Reduced height for better performance
        description='Color image height'
    )
    
    # Reduced FPS for more stable performance
    depth_fps = DeclareLaunchArgument(
        'depth_fps',
        default_value='15',  # Optimal balance between quality and performance
        description='Depth frames per second'
    )
    
    color_fps = DeclareLaunchArgument(
        'color_fps',
        default_value='15',  # Match with depth_fps
        description='Color frames per second'
    )
    
    # Minimize filters for performance
    filters = DeclareLaunchArgument(
        'filters',
        default_value='decimation',  # Only use essential filters
        description='RealSense filters to apply'
    )
    
    # Calculate paths to ORB-SLAM3 resource files
    orb_slam3_ros_dir = get_package_share_directory('orb_slam3_ros')
    voc_file = os.path.join(orb_slam3_ros_dir, '/home/xion/ros2_test/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt.bin')
    settings_file = os.path.join(orb_slam3_ros_dir, '/home/xion/ros2_test/src/orb_slam3_ros/config/RGB-D/RealSense_D435i.yaml')
    
    # Get the path to the realsense launch file
    realsense_dir = get_package_share_directory('orb_slam3_ros')
    realsense_launch_file = os.path.join(realsense_dir, '/home/xion/ros2_test/src/orb_slam3_ros/launch/realsense_d435.launch.py')
    
    # Include the RealSense camera launch file with optimized settings
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        launch_arguments={
            'camera': LaunchConfiguration('camera'),
            'depth_width': LaunchConfiguration('depth_width'),
            'depth_height': LaunchConfiguration('depth_height'),
            'color_width': LaunchConfiguration('color_width'),
            'color_height': LaunchConfiguration('color_height'),
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_pointcloud': 'false',  # Disable to save CPU
            'enable_sync': 'true',         # Keep sync for accurate SLAM
            'align_depth': 'true',         # Keep aligned depth for SLAM
            'depth_fps': LaunchConfiguration('depth_fps'),
            'color_fps': LaunchConfiguration('color_fps'),
            'filters': LaunchConfiguration('filters'),
            # Additional optimizations
            'enable_infra1': 'false',      # Disable unused streams
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'enable_confidence': 'false',  # Don't publish confidence if not needed
            'initial_reset': 'true',       # Reset camera at startup for clean state
            'reconnect_timeout': '3.0',    # Faster timeout for quicker recovery
            'wait_for_device_timeout': '3.0',
            'qos_depth': '1',             # Optimize QoS for low latency
            'qos_history_depth': '1'      # Minimize buffer size
        }.items()
    )
    
    # Create ORB-SLAM3 node with optimized parameters
    orb_slam3_node = Node(
        package='orb_slam3_ros',
        executable='ros_rgbd',
        name='orb_slam3',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'voc_file': voc_file,
            'settings_file': settings_file,
            'world_frame_id': 'map',
            'cam_frame_id': 'camera',
            'enable_pangolin': LaunchConfiguration('enable_pangolin'),
            'queue_size': 1,  # Small queue size to minimize latency
            'processing_threads': LaunchConfiguration('processing_threads'),
            'disable_loop_closing': LaunchConfiguration('disable_loop_closing'),
            'tf_publish_rate': 15.0,  # Match camera FPS
            # Additional performance optimizations
            'publish_tf': 'true',
            'publish_odom_tf': 'true',
            'publish_map_points': 'false',  # Disable if not needed for visualization
            'scale_factor': 1.2,  # Slightly faster scale factor for feature detection
            'nlevels': 6,        # Optimal pyramid levels
            'ini_th_FAST': 20,   # Higher threshold to detect fewer but more robust features
            'min_th_FAST': 7     # Higher minimum threshold
        }],
        remappings=[
            ('/camera/rgb/image_raw', '/camera/realsense2_camera_node/color/image_raw'),
            ('/camera/depth_registered/image_raw', '/camera/realsense2_camera_node/depth/image_rect_raw')
        ],
        # Using a regular priority value that doesn't require root permissions
        prefix=['nice -n 10']  # Changed from -15 to 10 (lower priority but doesn't need root)
        # Alternatively, remove the prefix line entirely to use default priority
    )
    
    # Trajectory server node (optional, only enabled when requested)
    trajectory_server_node = Node(
        package='hector_trajectory_server',
        executable='hector_trajectory_server',
        name='trajectory_server_orb_slam3',
        output='screen',
        namespace='orb_slam3',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_frame_name': 'map',
            'source_frame_name': 'camera',
            'trajectory_update_rate': 2.0,  # Reduced update rate
            'trajectory_publish_rate': 2.0  # Reduced publish rate
        }],
        condition=IfCondition(LaunchConfiguration('use_trajectory_server'))
    )
    
    # Return the launch description
    return LaunchDescription(
        env_vars +
        [
            use_sim_time,
            use_trajectory_server,
            enable_pangolin,
            processing_threads,
            disable_loop_closing,
            camera_name,
            depth_width,
            depth_height,
            color_width,
            color_height,
            depth_fps,
            color_fps,
            filters,
            realsense_launch,
            orb_slam3_node,
            trajectory_server_node
        ]
    )