import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name = 'my_robot_controller'

    pkg_path     = get_package_share_directory(package_name)
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
    # 5 saniye yetmezse 8'e çıkarabilirsin
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

    return LaunchDescription([
        sim_launch,
        nav2_launch,
    ])