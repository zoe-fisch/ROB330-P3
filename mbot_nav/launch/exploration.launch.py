from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    slam_node = Node(
        package='mbot_slam',
        executable='slam_node',
        name='slam_node',
        output='screen'
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

    exploration_node = Node(
        package='mbot_nav',
        executable='exploration_node',
        name='exploration_node',
        output='screen'
    )

    return LaunchDescription([
        slam_node,
        navigation_node,
        controller_node,
        exploration_node,
    ])
