#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')

   
    default_model_path = os.path.join(pkg_share, 'urdf', 'JPAC2.xacro')
    
    world_path         = os.path.join(pkg_share, 'world', 'creado.sdf')
  

   
    # gui, false 
    gui_arg = DeclareLaunchArgument(
        name='gui', default_value='false', choices=['true', 'false'],
        description='Enable joint_state_publisher_gui'
    )
    model_arg = DeclareLaunchArgument(
        name='model', default_value=default_model_path,
        description='Absolute path to robot xacro/urdf file'
    )
    
    use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='true'
    )
    

    # robot_description 
    meshes_abs = os.path.join(pkg_share, 'meshes', 'visual')
    robot_description_content = Command([
        'xacro ',
        LaunchConfiguration('model'),
        ' ',
        'meshes_path:=', meshes_abs
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # spawn 
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen'
    )
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'JPAC2', '-topic', 'robot_description'],
        output='screen'
    )
    spawn_after_delay = TimerAction(period=3.0, actions=[spawn])


    return LaunchDescription([

        gui_arg, model_arg, 
        use_sim_time, 
        robot_state_publisher_node,

        gazebo,
        spawn_after_delay,

    ])



