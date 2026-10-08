from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import AndSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


LOCALIZATION_LIFECYCLE_NODES = [
    'map_server',
    'amcl',
]

NAVIGATION_LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
    'collision_monitor',
]


def validate_map_argument(context, *args, **kwargs):
    map_value = LaunchConfiguration('map').perform(context).strip()
    if not map_value:
        raise RuntimeError('map:=/path/to/map.yaml is required for saved-map navigation')
    if not Path(map_value).expanduser().is_file():
        raise RuntimeError(
            f"Map file does not exist: {map_value}. "
            "Save a map to ~/maps/lekiwi/map.yaml or pass map:=/path/to/map.yaml."
        )
    context.launch_configurations['map'] = str(Path(map_value).expanduser().resolve())
    return []


def make_configured_params(params_file, use_sim_time, map_file):
    return ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    localization_autostart = LaunchConfiguration('localization_autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_arm_motors = LaunchConfiguration('auto_arm_motors')
    motor_guard_duration = LaunchConfiguration('motor_guard_duration')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    use_sim_time_override = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
    }
    configured_params = make_configured_params(params_file, use_sim_time, map_file)

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Navigation2에서 시뮬레이션 시간을 사용할지 여부.',
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=str(Path.home() / 'maps' / 'lekiwi' / 'map.yaml'),
        description='저장된 지도 경로. 기본값은 지도 저장 절차의 ~/maps/lekiwi/map.yaml입니다.',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('lekiwi_navigation2'),
            'config',
            'nav2_params.yaml',
        ]),
        description='LeKiwi base mode Nav2 파라미터 파일.',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='launch 후 Nav2 lifecycle node를 자동 활성화할지 여부.',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='LeKiwi Nav2 RViz 설정으로 RViz를 함께 실행할지 여부.',
    )
    auto_arm_motors_arg = DeclareLaunchArgument(
        'auto_arm_motors',
        default_value='true',
        description='RViz와 함께 실물 모터 감독 도구를 시작할지 여부. 실물 점검 전에는 false로 지정.',
    )
    motor_guard_duration_arg = DeclareLaunchArgument(
        'motor_guard_duration',
        default_value='3600',
        description='자동 모터 감독 도구의 최대 실행 시간(초).',
    )
    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Nav2 node가 종료되면 자동 재시작할지 여부.',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nav2 node log level.',
    )

    preflight_log = LogInfo(
        msg=[
            'LeKiwi Nav2 사전 확인: base bringup은 /odom과 TF ',
            'odom->base_footprint->base_link->soarm_base_link->lidar_link를 publish해야 하고, 라이다는 /scan을 publish해야 합니다. ',
            '기본 지도는 ~/maps/lekiwi/map.yaml이며, 다른 지도는 map:=/path/to/map.yaml로 지정합니다. ',
            'SLAM은 lekiwi_cartographer cartographer.launch.py로 별도 실행합니다. ',
            '기준 frame: map, odom, base_footprint, base_link, lidar_link. 실행 전 base bringup에서 /scan 수신을 확인하세요.',
        ],
    )

    localization_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[
                configured_params,
                use_sim_time_override,
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, use_sim_time_override],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                configured_params,
                use_sim_time_override,
                {'autostart': ParameterValue(localization_autostart, value_type=bool)},
                {'node_names': LOCALIZATION_LIFECYCLE_NODES},
            ],
        ),
    ]

    navigation_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, use_sim_time_override],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, use_sim_time_override],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, use_sim_time_override],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, use_sim_time_override],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, use_sim_time_override],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, use_sim_time_override],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, use_sim_time_override],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
            ],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[configured_params, use_sim_time_override],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                configured_params,
                use_sim_time_override,
                {'autostart': ParameterValue(autostart, value_type=bool)},
                {'node_names': NAVIGATION_LIFECYCLE_NODES},
            ],
        ),
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('lekiwi_navigation2'),
                'rviz',
                'nav2.rviz',
            ]),
        ],
        condition=IfCondition(use_rviz),
    )
    motor_guard_node = Node(
        package='lekiwi_teleop',
        executable='lekiwi_nav2_motor_guard',
        name='lekiwi_nav2_motor_guard',
        output='screen',
        arguments=[
            '--yes-i-confirm-motion',
            '--startup-timeout', '60',
            '--max-session-duration', motor_guard_duration,
        ],
        sigterm_timeout='10',
        condition=IfCondition(AndSubstitution(use_rviz, auto_arm_motors)),
    )
    stop_with_rviz = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_node,
            on_exit=[EmitEvent(event=Shutdown(reason='Nav2 RViz closed'))],
        ),
        condition=IfCondition(AndSubstitution(use_rviz, auto_arm_motors)),
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        use_sim_time_arg,
        map_arg,
        params_file_arg,
        autostart_arg,
        DeclareLaunchArgument('localization_autostart', default_value='true'),
        DeclareLaunchArgument('use_scan_filter', default_value='true'),
        use_rviz_arg,
        auto_arm_motors_arg,
        motor_guard_duration_arg,
        use_respawn_arg,
        log_level_arg,
        preflight_log,
        OpaqueFunction(function=validate_map_argument),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('lekiwi_sensors'), 'launch', 'navigation_scan.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(LaunchConfiguration('use_scan_filter')),
        ),
        GroupAction(actions=[
            SetParameter(
                name='use_sim_time',
                value=ParameterValue(use_sim_time, value_type=bool),
            ),
            *localization_nodes,
            *navigation_nodes,
        ]),
        stop_with_rviz,
        rviz_node,
        motor_guard_node,
    ])
