import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name  = 'my_robot_controller'
    pkg_share = get_package_share_directory(pkg_name)

    urdf_file    = os.path.join(pkg_share, 'urdf',   'my_robot.urdf')
    bridge_params = os.path.join(pkg_share, 'config', 'bridge_params.yaml')
    world_file   = os.path.join(pkg_share, 'worlds', 'robotaksi.world')

    # ── Robot State Publisher (URDF → TF) ─────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file],
        parameters=[{'use_sim_time': True}]
    )

    # ── Gazebo ────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '4', world_file],
        output='screen'
    )

    # ── ROS <-> Gazebo Bridge ─────────────────────────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # ── GPS Simulator (dashboard için) ────────────────────────────────
    gps_sim = Node(
        package='my_robot_controller',
        executable='gps_simulator',
        name='gps_simulator',
        output='screen',
        parameters=[{'ref_lat': 41.015137, 'ref_lon': 28.979530}]
    )

    # ── Trafik Lambası (3 lamba, senkronize faz) ─────────────────────
    traffic_light = Node(
        package='my_robot_controller',
        executable='traffic_light_node',
        name='traffic_light_node',
        output='screen',
    )

    # ── UMS Köprüsü (Kill / Go butonları) ────────────────────────────
    ums_bridge = Node(
        package='my_robot_controller',
        executable='ums_bridge',
        name='ums_bridge',
        output='screen',
    )

    # ── Durak Navigatör (tek node — GPS/IMU zinciri yok) ─────────────
    durak_nav = Node(
        package='my_robot_controller',
        executable='durak_nav',
        name='durak_nav',
        output='screen',
        parameters=[{
            'geojson_path':  '',      # boş → hardcoded ROUTE kullanılır
            'require_start': True,    # Kamera hazır olunca otomatik başla
        }]
    )

    # ── Şerit Algılama (kamera → offset/angle/conf) ───────────────────
    lane_detector = Node(
        package='my_robot_controller',
        executable='lane_detector_cv',
        name='lane_detector_cv',
        output='screen',
    )

    # ── Şerit Kontrolcüsü (offset → /cmd_vel_lane) ────────────────────
    lane_controller = Node(
        package='my_robot_controller',
        executable='lane_controller_node',
        name='lane_controller_node',
        output='screen',
        parameters=[{
            # Hız
            'target_speed':   3.0,
            'recovery_speed': 1.5,
            'min_speed':      0.4,
            # PID — daha az agresif, salınım önleme
            'kp':  0.08,
            'kd':  0.04,
            'kff': 0.05,
            # Açı kompansasyonu — küçük tutulmazsa düz yolda sapma yapar
            'k_angle':       0.06,
            'k_offset_pred': 0.06,
            # Direksiyon yumuşatma
            'max_steer_straight': 0.28,
            'max_steer_turn':     0.45,
            'steer_rate_limit':   0.03,
            'steer_filter_alpha': 0.12,
            # Tek şerit
            'single_lane_yaw':  0.15,
            'single_lane_gain': 0.20,
        }]
    )

    # ── Web Dashboard ─────────────────────────────────────────────────
    dashboard = Node(
        package='my_robot_controller',
        executable='dashboard_node',
        name='dashboard_node',
        output='screen',
    )

    run_durak_nav_arg = DeclareLaunchArgument(
        'run_durak_nav',
        default_value='true',
        description='Whether to run the durak_nav node'
    )

    return LaunchDescription([
        run_durak_nav_arg,
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.path.join(pkg_share, 'models')
        ),
        robot_state_publisher,
        gazebo,
        ros_gz_bridge,
        TimerAction(period=5.0,  actions=[ums_bridge]),
        TimerAction(period=8.0,  actions=[traffic_light]),
        TimerAction(
            period=8.0,
            actions=[durak_nav],
            condition=IfCondition(LaunchConfiguration('run_durak_nav'))
        ),
        TimerAction(period=10.0, actions=[gps_sim]),
        # Lane node'ları erken başlasın; kamera hazır olunca hemen devreye girer
        TimerAction(period=10.0, actions=[lane_detector]),
        TimerAction(period=12.0, actions=[lane_controller]),

        dashboard,
    ])
