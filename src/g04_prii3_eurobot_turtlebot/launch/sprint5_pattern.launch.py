from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    
        
        Node(
            package='g04_prii3_eurobot_turtlebot',
            executable='detector_almacenes',
            name='nodo_pattern_matching',
            output='screen'
        ),
    ])
