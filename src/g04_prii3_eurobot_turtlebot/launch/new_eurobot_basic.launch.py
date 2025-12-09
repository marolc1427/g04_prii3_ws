from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g04_prii3_eurobot_turtlebot',
            executable='new_eurobot_basic',
            name='new_eurobot_basic_node',
            output='screen',
        ),
    ])
