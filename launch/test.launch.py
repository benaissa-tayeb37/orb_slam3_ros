import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # USB Camera node
    # Get the USB camera launch file
    usb_cam_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('usb_cam'), 'launch', 'camera.launch.py'])]
        )
    )

    # Get package directories
    orb_slam3_ros_dir = get_package_share_directory('orb_slam3_ros')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_pangolin = LaunchConfiguration('enable_pangolin')
    world_frame_id = LaunchConfiguration('world_frame_id')
    cam_frame_id = LaunchConfiguration('cam_frame_id')


    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_enable_pangolin_cmd = DeclareLaunchArgument(
        'enable_pangolin',
        default_value='true',
        description='Enable Pangolin visualization')


    declare_world_frame_id_cmd = DeclareLaunchArgument(
        'world_frame_id', 
        default_value='world',
        description='World frame ID')

    declare_cam_frame_id_cmd = DeclareLaunchArgument(
        'cam_frame_id',
        default_value='camera',
        description='Camera frame ID')
    
    # ORB-SLAM3 node
    orb_slam3_node = Node(
        package='orb_slam3_ros',
        executable='ros_mono',
        name='orb_slam3',
        output='screen',
        remappings=[
            ('/camera/image_raw', '/usb_cam/image_raw'),
        ],
        parameters=[{
            'voc_file':  os.path.expanduser('~/ros2_test/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin'),
            'settings_file': os.path.expanduser('~/ros2_test/src/ros2_orb_slam3/orb_slam3/config/Monocular/'),
            'world_frame_id': world_frame_id,
            'cam_frame_id': cam_frame_id,
            'enable_pangolin': enable_pangolin
        }]
    )




    # Static TF publisher
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_cam_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'camera']
    )

    # RViz
    rviz_config_file = os.path.join(orb_slam3_ros_dir, 'config', 'orb_slam3_no_imu.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Trajectory Server (equivalent to hector_trajectory_server)
    # Note: You need to find or create a ROS 2 equivalent package for hector_trajectory_server
    # This is an example assuming the package exists with similar parameters
    # If it doesn't exist, you'll need to find an alternative or port it
    # trajectory_server_node = Node(
    #     package='hector_trajectory_server',
    #     executable='hector_trajectory_server',
    #     name='trajectory_server_orb_slam3',
    #     output='screen',
    #     parameters=[{
    #         'target_frame_name': 'world',
    #         'source_frame_name': 'camera',
    #         'trajectory_update_rate': 20.0,
    #         'trajectory_publish_rate': 20.0
    #     }]
    # )

    # Create the launch description
    ld = LaunchDescription()

    # Add declared arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_enable_pangolin_cmd)
    ld.add_action(declare_world_frame_id_cmd)
    ld.add_action(declare_cam_frame_id_cmd)

    # Add the nodes to the launch description
    ld.add_action(usb_cam_launch_include)
    ld.add_action(orb_slam3_node)
    ld.add_action(static_tf_node)
    ld.add_action(rviz_node)
    # Comment out if hector_trajectory_server doesn't have a ROS 2 port yet
    # ld.add_action(trajectory_server_node)

    return ld