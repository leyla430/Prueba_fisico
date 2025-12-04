
# ------------------------------------------------
# SI DA - TERMIAND DE PRPBARLO MANEJANDO EL ROBOT 
# ------------------------------------------------

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz     = LaunchConfiguration('use_rviz',     default='true')

    tb3_cartographer_dir = get_package_share_directory('robot_cartographer')

    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(tb3_cartographer_dir, 'config')
    )

    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='turtlebot3_lds_2d.lua'
    )

    resolution         = LaunchConfiguration('resolution',          default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec',  default='1.0')

    rviz_config_dir = os.path.join(tb3_cartographer_dir, 'rviz', 'tb3_cartographer.rviz')

    return LaunchDescription([

        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir,
                              description='Full path to config dir with the .lua'),
        DeclareLaunchArgument('configuration_basename',  default_value=configuration_basename,
                              description='Lua filename'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use Gazebo/Sim time if true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('resolution', default_value=resolution),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec),

        # CARTOGRAPHER NODE (aquí van los remappings)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename',  configuration_basename
            ],
            remappings=[
                ('scan', '/scan'),                      # LIDAR
                #('odom', '/odometry/filtered'),        # EKF
            ],
        ),

        # OCCUPANCY GRID
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }.items(),
        ),

        # RVIZ (opcional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])
