from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g04_prii3_eurobot_turtlebot',
            executable='eurobot_basic',
            name='eurobot_basic_node',
            output='screen'
        ),
    ])
