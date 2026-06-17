import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'my_robot_controller'
    pkg_share = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    bridge_params = os.path.join(pkg_share, 'config', 'bridge_params.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file],
        parameters=[{'use_sim_time': True}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    world_file = os.path.join(pkg_share, 'worlds', 'robotaksi.world')
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', world_file],
        output='screen'
    )

    # 🔥 DÜZELTME BURADA: -entity yerine -name kullanıldı, yükseklik 0.15'e düşürüldü.
    spawn_entity = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'my_robot',      # -entity yerine -name
                    '-z', '0.15',             # Yükseklik 0.5'ten 0.15'e!
                    '-x', '2.0',
                    '-y', '3.0'
                ],
                output='screen'
            )
        ]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
    )

    lidar_frame_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_fix',
        arguments=['0', '0', '0', '0', '0', '0',
                   'base_footprint', 'bumperbot/base_footprint/lidar'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.path.join(pkg_share, 'models')
        ),
        robot_state_publisher,
        joint_state_publisher,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        ekf_node,
        lidar_frame_fix
    ])