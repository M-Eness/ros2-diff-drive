import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_controller'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Simülasyonu Başlat
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sim_launch.py')
        )
    )

    # 2. SLAM Toolbox (Haritalama Algoritması)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,          # Simülasyon zamanı (Kritik!)
            'base_frame': 'base_link',     # Robot gövdesi
            'odom_frame': 'odom',          # Odometri referansı
            'map_frame': 'map',            # Harita referansı
            'scan_topic': '/scan',         # Lidar verisi
            'mode': 'mapping'              # Haritalama modu
        }]
    )

    return LaunchDescription([
        sim_launch,
        slam_toolbox
    ])