import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_robot_controller'

    pkg_path      = get_package_share_directory(package_name)
    nav2_pkg_path = get_package_share_directory('nav2_bringup')

    map_file_path    = os.path.join(pkg_path, 'maps', 'my_map.yaml')
    params_file_path = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    # 1. Simülasyon — hemen başlar (Gazebo + EKF + Bridge + URDF)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sim_launch.py')
        )
    )

    # 2. Nav2 — 5 saniye bekle, Gazebo ve EKF hazır olsun
    nav2_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_pkg_path, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'map': map_file_path,
                    'params_file': params_file_path,
                    'use_sim_time': 'true',
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    # 3. Statik engel algılama — 7 saniye bekle, Nav2 hazır olsun
    static_obstacle_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='my_robot_controller',
                executable='static_obstacle_node',
                name='static_obstacle_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'lidar_topic': '/scan',
                    'base_frame': 'base_link',
                    'roi_x_min': -1.0,
                    'roi_x_max': 15.0,
                    'roi_y_min': -4.0,
                    'roi_y_max': 4.0,
                    'ground_z_threshold': -0.1,
                    'obstacle_z_min': -0.05,
                    'obstacle_z_max': 2.5,
                    'voxel_size': 0.1,
                    'cluster_tolerance': 0.35,
                    'min_cluster_size': 4,
                    'max_cluster_size': 8000,
                    'min_obstacle_width': 0.08,
                    'max_obstacle_width': 10.0,
                    'laser_range_max': 15.0,
                }]
            )
        ]
    )

    # 4. Algı nodeları — 5 saniye bekle, simülasyon hazır olsun
    perception_nodes = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='my_robot_controller',
                executable='traffic_sign_detector',
                name='traffic_sign_detector',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
            Node(
                package='my_robot_controller',
                executable='traffic_light_detector',
                name='traffic_light_detector',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
            Node(
                package='my_robot_controller',
                executable='lane_detector_cv',
                name='lane_detector_cv',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
        ]
    )

    # 5. Araç kontrol zinciri — 5 saniye bekle
    vehicle_nodes = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='my_robot_controller',
                executable='ackermann_bridge',
                name='ackermann_bridge',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
            Node(
                package='my_robot_controller',
                executable='vehicle_emulator',
                name='vehicle_emulator',
                output='screen'
            ),
        ]
    )

    # 6. Statik engel algılama — 7 saniye bekle, Nav2 hazır olsun
    static_obstacle_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='my_robot_controller',
                executable='static_obstacle_node',
                name='static_obstacle_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'lidar_topic': '/scan',
                    'base_frame': 'base_link',
                    'roi_x_min': -1.0,
                    'roi_x_max': 15.0,
                    'roi_y_min': -4.0,
                    'roi_y_max': 4.0,
                    'ground_z_threshold': -0.1,
                    'obstacle_z_min': -0.05,
                    'obstacle_z_max': 2.5,
                    'voxel_size': 0.1,
                    'cluster_tolerance': 0.35,
                    'min_cluster_size': 4,
                    'max_cluster_size': 8000,
                    'min_obstacle_width': 0.08,
                    'max_obstacle_width': 10.0,
                    'laser_range_max': 15.0,
                }]
            )
        ]
    )

    # 7. BT karar düğümü — 8 saniye bekle, Nav2 + algı hazır olsun
    bt_decision = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='my_robot_controller',
                executable='bt_decision_node',
                name='bt_decision_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # 8. Görev yöneticisi — 10 saniye bekle, her şey hazır olsun
    mission = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='my_robot_controller',
                executable='mission_manager',
                name='mission_manager',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        sim_launch,
        nav2_launch,
        perception_nodes,
        vehicle_nodes,
        static_obstacle_node,
        bt_decision,
        mission,
    ])