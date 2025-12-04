#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():


    channel_type = LaunchConfiguration('channel_type', default='serial')  
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0') 
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000') 
    frame_id = LaunchConfiguration('frame_id', default='laser')  # frame 'laser'
    inverted = LaunchConfiguration('inverted', default='false')  
    angle_compensate = LaunchConfiguration('angle_compensate', default='true') 
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')  


    return LaunchDescription([
 
 
        DeclareLaunchArgument('channel_type', default_value=channel_type, 
            description='Tipo de canal del LIDAR'),
        DeclareLaunchArgument('serial_port', default_value=serial_port, 
            description='Puerto serial para conectar el LIDAR'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate, 
            description='Velocidad de baudios del puerto serial'),
        DeclareLaunchArgument('frame_id', default_value=frame_id, 
            description='El frame_id del LIDAR'),
        DeclareLaunchArgument('inverted', default_value=inverted, 
            description='Si se invierten los datos de escaneo'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate, 
            description='Si se habilita la compensación de ángulo'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode, 
            description='Modo de escaneo del LIDAR'),

        # nodo del LIDAR
        Node(
            package='sllidar_ros2',  
            executable='sllidar_node',  
            name='sllidar_node',  
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,  #  frame_id 'laser'
                'inverted': inverted,
                'angle_compensate': angle_compensate,
            }],
            output='screen'  
        ),

    ])
