from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, Command, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # URDF/xacro file to be loaded by the Robot State Publisher node
    default_xacro_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'urdf',
        'zed_descr.urdf.xacro'
    )

    # Define launch arguments
    vicon_args = [
        # Adding mocap vicon specific parameters
        DeclareLaunchArgument('mocap', default_value='false'),
        DeclareLaunchArgument('mocap_server', default_value='192.168.8.2'),
        DeclareLaunchArgument('mocap_frame_rate', default_value='100'),
        DeclareLaunchArgument('mocap_max_accel', default_value='10.0'),
    ]    

    # ZED camera arguments
    zed_args = [
        DeclareLaunchArgument('zed_enable', default_value='true'),
        DeclareLaunchArgument('camera_name', default_value='zed'),
        DeclareLaunchArgument('camera_model', default_value='zedm'),
        DeclareLaunchArgument('publish_urdf', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('publish_map_tf', default_value='true'),
        DeclareLaunchArgument('publish_imu_tf', default_value='true'),
        DeclareLaunchArgument('xacro_path', default_value=TextSubstitution(text=default_xacro_path)),
        DeclareLaunchArgument('custom_baseline', default_value='0.0'),
        DeclareLaunchArgument('enable_gnss', default_value='false'),
        DeclareLaunchArgument('publish_svo_clock', default_value='false'),
    ]

    # KR interface arguments
    betaflight_config_file = os.path.join(
        get_package_share_directory('kr_betaflight_interface'),
        'config',
        'neurofly.yaml'
    )
    trackers_manager_config_file = os.path.join(
        get_package_share_directory('neurofly_interface'),
        'config',
        'neurofly_trackers.yaml'
    )
    so3_config_file = os.path.join(
        get_package_share_directory('neurofly_interface'),
        'config',
        'neurofly_gains.yaml'
    )
    kr_args = [
        DeclareLaunchArgument('robot', default_value='neurofly1'), # set robot namespace        
        DeclareLaunchArgument('mass', default_value='.680'), # set mass AUW
        DeclareLaunchArgument(
            'odom',
            default_value=PythonExpression([
            '"control_odom"' if LaunchConfiguration('zed_enable') == 'true' else '"odom"'
            ])
        ), # set odom topic (vio/ukf/vicon)
        DeclareLaunchArgument('so3_cmd', default_value='so3_cmd'),
    ]

    # Add all arguments to launch description
    for arg in vicon_args:
        ld.add_action(arg)
    for arg in zed_args:
        ld.add_action(arg)
    for arg in kr_args:
        ld.add_action(arg)

    plan_manage_pkg_share = get_package_share_directory('plan_manage')
    advanced_yaml = os.path.join(plan_manage_pkg_share, 'config', 'global_planning.yaml')

    # Declare all launch arguments
    ld.add_action(DeclareLaunchArgument('mav_name', default_value=LaunchConfiguration('robot')))
    ld.add_action(DeclareLaunchArgument('mav_id', default_value='1'))
    ld.add_action(DeclareLaunchArgument('mav_type', default_value='neurofly'))
    ld.add_action(DeclareLaunchArgument('world_frame_id', default_value='map'))
    ld.add_action(DeclareLaunchArgument('sim', default_value='1'))
    ld.add_action(DeclareLaunchArgument('vicon', default_value='1'))
    ld.add_action(DeclareLaunchArgument('vicon_fps', default_value='100'))
    ld.add_action(DeclareLaunchArgument('slow_baud_rate', default_value='true'))
    ld.add_action(DeclareLaunchArgument('odom_topic', default_value='/neurofly1/control_odom'))
    ld.add_action(DeclareLaunchArgument('waypoints_topic', default_value='/neurofly1/waypoints'))
    ld.add_action(DeclareLaunchArgument('tracker_topic', default_value='/neurofly1/tracker_cmd'))
    
    ld.add_action(DeclareLaunchArgument('map_size_x', default_value='30.0'))
    ld.add_action(DeclareLaunchArgument('map_size_y', default_value='30.0'))
    ld.add_action(DeclareLaunchArgument('map_size_z', default_value='5.0'))
    
    ld.add_action(DeclareLaunchArgument('map_origin_x', default_value='-15.0'))
    ld.add_action(DeclareLaunchArgument('map_origin_y', default_value='-15.0'))
    ld.add_action(DeclareLaunchArgument('map_origin_z', default_value='-1.0'))
    
    ld.add_action(DeclareLaunchArgument('initial_offset/x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('initial_offset/y', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('poly_srv_name', default_value='/neurofly1/mav_services/poly_tracker'))
    ld.add_action(DeclareLaunchArgument('planning_horizon', default_value='5.0'))
    ld.add_action(DeclareLaunchArgument('flight_type', default_value='1'))  # 1: hover, 2: waypoint
    ld.add_action(DeclareLaunchArgument('point_num', default_value='3'))

    # add identity tf between 'world' and 'map'
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen'
    ))

    # container for the planner component
    container = ComposableNodeContainer(
        name='opt_planner_container',
        namespace=LaunchConfiguration('mav_name'),
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='plan_manage',
                plugin='ReplanFSM',
                name='opt_planner_node',
                namespace=LaunchConfiguration('mav_name'),
                parameters=[
                    advanced_yaml,
                    {
                        'VehicleMass': LaunchConfiguration('mass'),
                        'plan_frame_id': LaunchConfiguration('world_frame_id'),
                        'initial_offset/x': LaunchConfiguration('initial_offset/x'),
                        'initial_offset/y': LaunchConfiguration('initial_offset/y'),
                        'poly_srv_name': LaunchConfiguration('poly_srv_name'),
                        'search/horizon': LaunchConfiguration('planning_horizon'),
                        'fsm/flight_type': LaunchConfiguration('flight_type'),
                        'fsm/waypoint_num': LaunchConfiguration('point_num'),
                        'map/x_size': LaunchConfiguration('map_size_x'),
                        'map/y_size': LaunchConfiguration('map_size_y'),
                        'map/z_size': LaunchConfiguration('map_size_z'),
                        'map/x_origin': LaunchConfiguration('map_origin_x'),
                        'map/y_origin': LaunchConfiguration('map_origin_y'),
                        'map/z_origin': LaunchConfiguration('map_origin_z'),
                    }
                ],
                remappings=[
                    ('odom_world', LaunchConfiguration('odom_topic')),
                    ('grid_map_depth', '/neurofly1/zed_node/depth/depth_registered'),
                    ('grid_map_depth/camera_info', '/neurofly1/zed_node/depth/camera_info'),
                    ('waypoints', LaunchConfiguration('waypoints_topic')),
                    ('tracker_cmd', LaunchConfiguration('tracker_topic'))
                ]
            ),
            ComposableNode(
                package="kr_betaflight_interface",
                plugin="SO3CmdToBetaflight",
                name="so3cmd_to_betaflight",
                namespace=LaunchConfiguration('robot'),
                parameters=[betaflight_config_file],
                remappings=[
                    ('so3_cmd', LaunchConfiguration('so3_cmd')),
                    ('odom', LaunchConfiguration('odom')),
                    ('imu', 'zed/zed_node/imu/data'),
                ]
            ),
            ComposableNode(
                package="kr_trackers_manager",
                plugin="TrackersManager",
                name="trackers_manager",
                namespace=LaunchConfiguration('robot'),
                parameters=[trackers_manager_config_file],
            ),
            ComposableNode(
                package="kr_trackers_manager",
                plugin="TrackersManagerLifecycleManager",
                name="lifecycle_manager",
                namespace=LaunchConfiguration('robot'),
                parameters=[
                    {'node_name': "trackers_manager"}
                ]
            ),
            ComposableNode(
                package="kr_mav_controllers",
                plugin="SO3ControlComponent",
                name="so3_controller",
                namespace=LaunchConfiguration('robot'),
                parameters=[
                    [so3_config_file],
                ],
            ),
            ComposableNode(
                condition=IfCondition(LaunchConfiguration('zed_enable')),
                package="zed_components",
                plugin="stereolabs::ZedCamera",
                name="zed_node",
                namespace=LaunchConfiguration('robot'),
                parameters=[
                    [FindPackageShare('zed_wrapper').find('zed_wrapper'), '/config/', LaunchConfiguration('camera_model'), '.yaml'],
                    [FindPackageShare('zed_wrapper').find('zed_wrapper'), '/config/common_stereo.yaml'],
                    # Finally apply launch-specific overrides
                    {
                        'general.camera_name': LaunchConfiguration('camera_name'),
                        'general.camera_model': LaunchConfiguration('camera_model'),
                        'pos_tracking.publish_tf': LaunchConfiguration('publish_tf'),
                        'pos_tracking.publish_map_tf': LaunchConfiguration('publish_map_tf'), 
                        'sensors.publish_imu_tf': LaunchConfiguration('publish_imu_tf', default='true'),
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                condition=IfCondition(LaunchConfiguration('zed_enable')),
                package="quadrotor_ukf_ros2",
                plugin="QuadrotorUKFNode",
                name="quadrotor_ukf_ros2",
                namespace=LaunchConfiguration('robot'),
                parameters=[{
                    'odom': LaunchConfiguration('odom'),
                    'imu_frame_id': 'zed_imu_link',
                    'imu_rotated_frame_id': 'zed_camera_link',                    
                    'base_link': 'zed_camera_link'
                }],
                remappings=[
                    ('odom', 'zed_node/odom'),
                    ('imu', 'zed_node/imu/data'),
                ],
            ),
            ComposableNode(
                condition=IfCondition(LaunchConfiguration('mocap')),
                package='mocap_vicon',
                plugin='mocap::ViconDriverComponent',
                name='vicon',
                namespace='vicon',
                parameters=[
                    {'server_address': LaunchConfiguration('mocap_server')},
                    {'frame_rate': LaunchConfiguration('mocap_frame_rate')},
                    {'max_accel': LaunchConfiguration('mocap_max_accel')},
                    {'publish_tf': False},
                    {'publish_pts': False},
                    {'fixed_frame_id': 'mocap'},
                    {'model_list': ['']},
                ],
                remappings=[
                    ('/vicon/neurofly1/odom', '/neurofly1/control_odom'),
                ]
            ),
        ],
        output='screen',
    )

    # robot state publisher to publish URDF and static transforms for zed
    ld.add_action(
        Node(
            condition=IfCondition(LaunchConfiguration('zed_enable')),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=[LaunchConfiguration('camera_name'), '_state_publisher'],
            namespace=LaunchConfiguration('robot'),
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('publish_svo_clock'),
                'robot_description': Command([
                    'xacro', ' ', LaunchConfiguration('xacro_path'),
                    ' camera_name:=', LaunchConfiguration('camera_name'),
                    ' camera_model:=', LaunchConfiguration('camera_model'),
                    ' custom_baseline:=', LaunchConfiguration('custom_baseline')
                ])
            }]
        )
    )

    ld.add_action(
        Node(
            package="kr_mav_manager",
            executable="mav_services",
            namespace=LaunchConfiguration('robot'),
            name="mav_services",
            output='screen',
            parameters = [
                {'mass': LaunchConfiguration('mass')},
            ],
        )
    )

    ld.add_action(container)

    return ld
