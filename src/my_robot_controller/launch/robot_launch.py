import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_controller'

    # Bu yöntem "Hard Coded Path" (sabit dosya yolu) kullanmaktan çok daha profesyoneldir.
    # Bilgisayar değişse de çalışır.
    urdf_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'my_robot.urdf'
    )

    # Robot State Publisher Node'u
    # Görevi: URDF'i okur ve robotun eklemlerinin uzaydaki yerini (/tf) yayınlar.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file]
    )

    # Ortak eklem durumlarını yayınlayan node (Joint State Publisher)
    # Bu olmazsa tekerlekler havada asılı kalabilir, tf ağacı kopuk olur.
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])