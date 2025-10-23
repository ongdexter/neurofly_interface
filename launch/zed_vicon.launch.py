from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mocap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mocap_vicon'),
            'launch',
            'vicon_launch.py'
        ))
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('zed_wrapper'),
            'launch',
            'zed_camera.launch.py'
        ))
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='init_world_odom_transform_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )

    return LaunchDescription([
        mocap_launch,
        zed_launch,
        static_tf,
    ])
