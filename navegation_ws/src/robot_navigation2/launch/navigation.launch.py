import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_btnav = get_package_share_directory('nav2_bt_navigator')
    pkg_this = get_package_share_directory('robot_navigation2')

    # Rutas de config/mapa propias de tu paquete
    params_file = os.path.join(pkg_this, 'param', 'humble', 'agro2.yaml')
    map_file    = os.path.join(pkg_this, 'map', 'map_cuarto1.yaml')

    # BT por defecto (instalado con Nav2) para evitar rutas absolutas locales
    default_bt_xml = os.path.join(
        pkg_btnav, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'
    )

    # RViz config (vista por defecto de Nav2)
    rviz_file = os.path.join(pkg_nav2, 'rviz', 'tb3_navigation2.rviz')

    # Nodos manejados por cada lifecycle manager
    lifecycle_nodes_localization = [
        'map_server',
        'amcl',
    ]
    lifecycle_nodes_navigation = [
        'planner_server',
        'controller_server',
        'bt_navigator',
        'behavior_server',
        'velocity_smoother',
        # 'collision_monitor',  # Descomenta si lo usas y tienes sus params en el YAML
    ]

    # Remappings típicos para controladores diff-drive
    remappings = [
        ('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped'),
    ]

    return LaunchDescription([
        # Logs sin búfer
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # ---- LOCALIZATION STACK ----
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes_localization
            }],
        ),

        # ---- NAVIGATION STACK ----
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=remappings
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {
                'default_bt_xml_filename': default_bt_xml
            }],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
            remappings=remappings
        ),
        # (Opcional) Collision monitor si lo configuras en el YAML
        # Node(
        #     package='nav2_collision_monitor',
        #     executable='collision_monitor',
        #     name='collision_monitor',
        #     output='screen',
        #     parameters=[params_file],
        # ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes_navigation
            }],
        ),

        # ---- RVIZ ----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': False}],
        ),
    ])
