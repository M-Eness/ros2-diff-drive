import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_robot_controller'
    pkg_path = get_package_share_directory(package_name)

    # 1. Simülasyon — hemen başlar (Gazebo + Bridge + URDF + lane + ışık)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'sim_launch.py')
        )
    )

    # 2. Algı nodeları — 10 saniye bekle, simülasyon ve kamera hazır olsun
    #    Trafik işareti + trafik ışığı YOLO tespiti
    perception_nodes = TimerAction(
        period=10.0,
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
        ]
    )

    # 3. Slalom Kontrolcüsü — 10 saniye bekle
    slalom_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='my_robot_controller',
                executable='slalom_controller',
                name='slalom_controller',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
        ]
    )

    # 4. BT Karar düğümü — 12 saniye bekle, algı hazır olsun
    #    durak_only_mode=True → GPS/durak mantığı devre dışı,
    #    sadece ışık/dur/slalom/şerit kararları aktif
    bt_decision = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='my_robot_controller',
                executable='bt_decision_node',
                name='bt_decision_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'durak_only_mode': False,  # tüm levha/ışık mantığı aktif
                    'tur_modu': 1,             # Tur 1: lane takibi + levhalar
                }]
            )
        ]
    )

    return LaunchDescription([
        sim_launch,
        perception_nodes,
        slalom_node,
        bt_decision,
    ])