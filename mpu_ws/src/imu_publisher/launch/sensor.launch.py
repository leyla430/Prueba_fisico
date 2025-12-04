from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            
            package='imu_publisher',
            executable='imu',  # 
            output='screen',
        ),

        Node(
            package='imu_publisher',
            executable='odom', # odometria, encoder
            output='screen',
        ),

        Node(
            package='imu_publisher',
            executable='motor',  # sub cmd_vel
            output='screen',
        ),

    ])