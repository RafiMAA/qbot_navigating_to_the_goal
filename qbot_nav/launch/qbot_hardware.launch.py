import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    qbot_nav_share = get_package_share_directory('qbot_nav')
    slam_config_path = os.path.join(qbot_nav_share, 'config', 'slam_toolbox_params.yaml')

    # 1. Kobuki Base Driver
    kobuki_driver = Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        name='kobuki_ros_node',
        output='screen',
        parameters=[{'device_port': '/dev/kobuki'}] # Update to match your udev rule if different
    )

    # 2. LD19 LiDAR Driver
    lidar_driver = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ldlidar_stl_ros2_node',
        output='screen',
        parameters=[{
            'product_name': 'LD19',
            'port_name': '/dev/ldlidar', # Update to match your udev rule if different
            'topic_name': 'scan',
            'frame_id': 'laser_link'
        }]
    )

    # 3. Static Transform (Base to LiDAR)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.14', '0.0', '0.0', '0.0', 'base_link', 'laser_link']
    )

    # 4. SLAM and Navigation Stack
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_path,
            {'use_sim_time': False, 'mode': 'mapping'}
        ],
    )

    slam_pose_node = Node(
        package='qbot_nav',
        executable='slam_pose_publisher',
        name='slam_pose_publisher',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    navigation_server = Node(
        package='qbot_nav',
        executable='navigation_server',
        name='navigation_server',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    qbot_controller = Node(
        package='qbot_nav',
        executable='qbot_controller',
        name='qbot_controller',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    return LaunchDescription([
        kobuki_driver,
        lidar_driver,
        static_tf,
        slam_node,
        slam_pose_node,
        navigation_server,
        qbot_controller
    ])