from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Simulatore (Percorso relativo come nel terminale)
        Node(
            package='rp_simulator',
            executable='simulator',
            name='simulator',
            parameters=[{'config_file': 'src/dmap_navigation/maps/diag_single_robot.yaml'}],
            output='screen'
        ),
        
        # 2. Map Server (Percorso relativo! Questo evita il crash "Out of memory" su WSL)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': 'src/dmap_navigation/maps/map_ros2.yaml'}],
            output='screen'
        ),
        
        # 3. Lifecycle Manager (Per svegliare il map server)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[{'node_names': ['map_server'], 'autostart': True}],
            output='screen'
        ),
        
        # 4. RViz2 (Punta al file di configurazione salvato)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', 'progetto.rviz'],
            output='screen'
        )
    ])