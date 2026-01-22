import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_controller')
    sdf_file = os.path.join(pkg_path, 'models', 'box.sdf')

    # 2. Spawn Eden Düğüm
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'robotaxi_track_complex',
            '-name', 'my_dynamic_obstacle',
            '-file', sdf_file,  # Model dosyası
            '-x', '0.0',  # X Koordinatı
            '-y', '0.0',  # Y Koordinatı
            '-z', '1.0'  # Yükseklik
        ],
        output='screen'
    )

    return LaunchDescription([spawn_node])