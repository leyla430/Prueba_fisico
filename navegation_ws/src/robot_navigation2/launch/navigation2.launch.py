import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---------------------- Nav2 , AMCL <-------------------------------

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_dir = LaunchConfiguration('map')

    default_params = os.path.join(
        get_package_share_directory('robot_navigation2'),
        'param',
        'humble',
        'agro.yaml' #--------------- configuracion .yaml
    )

    param_dir = LaunchConfiguration(
        'params_file',
        default=default_params
    )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(
        get_package_share_directory('robot_navigation2'),
        'rviz',
        'tb3_navigation2.rviz'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'map',
            default_value='/home/leyla/Desktop/mapas/map_cuarto1.yaml',
            description='Direcccion completa a  yaml DEL MAPA.'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to Nav2 params.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir
            }.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
