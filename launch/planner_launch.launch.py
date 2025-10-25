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

    kr_args = [
        DeclareLaunchArgument('robot', default_value='neurofly1'), # set robot namespace        
        DeclareLaunchArgument('mass', default_value='.680'), # set mass AUW
        DeclareLaunchArgument('odom', default_value='control_odom'), # set odom topic (vio/ukf/vicon)
        DeclareLaunchArgument('so3_cmd', default_value='so3_cmd'),
    ]
    
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
    
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
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
            )
        ],
        output='screen',
    )

    ld.add_action(container)

    return ld
