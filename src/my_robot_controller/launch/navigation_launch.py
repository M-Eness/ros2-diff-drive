import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. AYARLAR
    package_name = 'my_robot_controller'

    # Dosya yollarını bul
    pkg_path = get_package_share_directory(package_name)
    nav2_pkg_path = get_package_share_directory('nav2_bringup')

    # Harita ve Parametre dosyalarının yerleri
    map_file_path = os.path.join(pkg_path, 'maps', 'my_map.yaml')
    params_file_path = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    # 2. SİMÜLASYON BAŞLATMA
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sim_launch.py')
        )
    )

    # 3. NAV2 BAŞLATMA (BRINGUP)
    # Nav2'nin kendi launch dosyası
    # Bu dosya; AMCL, Map Server, Controller, Planner hepsini açar.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_path, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file_path,
            'params_file': params_file_path,
            'use_sim_time': 'true',
            'autostart': 'true'  # Düğümler açılınca otomatik başlasın
        }.items()
    )
    return LaunchDescription([
        sim_launch,
        nav2_launch
    ])