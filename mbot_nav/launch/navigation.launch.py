from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        description='Map file name (without .yaml extension)'
    )

    map_file_path = PathJoinSubstitution([
        get_package_share_directory('mbot_nav'),
        'maps',
        [LaunchConfiguration('map_name'), '.yaml']
    ])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path, 'use_sim_time': False}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': ['map_server']}]
    )

    localization_node = Node(
        package='mbot_slam',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[{'publish_tf': True}]
    )

    navigation_node = Node(
        package='mbot_nav',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{'pose_source': 'tf'}]
    )

    controller_node = Node(
        package='mbot_nav',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{'use_localization': True}]
    )

    return LaunchDescription([
        map_name_arg,
        map_server,
        lifecycle_manager,
        localization_node,
        navigation_node,
        controller_node,
    ])
