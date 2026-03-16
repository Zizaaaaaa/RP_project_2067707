from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo Localizzatore
        Node(
            package='dmap_navigation',
            executable='localizer_node',
            name='localizer_node',
            output='screen'
        ),
        
        # Nodo Planner
        Node(
            package='dmap_navigation',
            executable='planner_node',
            name='planner_node',
            output='screen'
        ),
        
        # RViz2 per visualizzare
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])