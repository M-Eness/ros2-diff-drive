import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'my_robot_controller'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. URDF Dosyasının Yolu
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')

    # 2. Bridge Ayar Dosyasının Yolu
    bridge_params = os.path.join(pkg_share, 'config', 'bridge_params.yaml')

    # ========================================================================
    # NODE 1: ROBOT STATE PUBLISHER (MİMAR)
    # Robotun iskeletini ROS'a tanıtır.
    # ========================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file],
        parameters=[{'use_sim_time': True}]  # ÖNEMLİ: Simülasyon saati kullan
    )

    # ========================================================================
    # NODE: JOINT STATE PUBLISHER (KUKLACI)
    # Bu olmazsa Foxglove tekerleklerin döndüğünü anlamaz!
    # ========================================================================
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]  # Simülasyon saati ile senkronize ol
    )

    # ========================================================================
    # NODE 2: GAZEBO SIMULATOR (DÜNYA)
    # Gazebo'yu başlatır. "-r" (run) simülasyonu başlatır.
    # "empty.sdf" boş bir dünya açar.
    # ========================================================================
    world_file = os.path.join(pkg_share, 'worlds', 'robotaksi.world')

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', '-v', '4', world_file],
        output='screen'
    )

    # ========================================================================
    # NODE 3: SPAWNER (DOĞUM UZMANI)
    # Gazebo açıldıktan sonra robotu içine "doğurur".
    # ========================================================================
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',  # URDF'i bu topic'ten okur
                   '-entity', 'my_robot',  # Robotun Gazebo'daki adı
                   '-z', '2.0',
                   '-x', '2.0',
                   '-y', '3.0'],  # Yere gömülmesin diye 10cm yukarıda doğar
        output='screen'
    )

    # ========================================================================
    # NODE 4: BRIDGE (KÖPRÜ)
    # ROS ve Gazebo arasındaki iletişimi kurar (YAML dosyasını kullanarak)
    # ========================================================================
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}'
        ],
        output='screen'
    )

    # ========================================================================
    # NODE 5: EKF - Odometry'den odom->base_footprint TF'i üret
    # ========================================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
    )

    # ========================================================================
    # NODE 6: LIDAR FRAME DÜZELTİCİ
    # Gazebo'nun "bumperbot/base_footprint/lidar" frame'ini "laser_link"e bağla
    # ========================================================================
    lidar_frame_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_fix',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_link', 'bumperbot/base_footprint/lidar'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value='/home/rana/Documents/GitHub/ros2-diff-drive/src/my_robot_controller/models'
        ),
        robot_state_publisher,
        joint_state_publisher,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        ekf_node,
        lidar_frame_fix
    ])