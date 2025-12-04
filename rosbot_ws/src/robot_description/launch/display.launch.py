from ament_index_python.packages import get_package_share_path
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
import os
import launch_ros


# Si 

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_description').find('robot_description')
    pkg_path  = get_package_share_path('robot_description')

    default_model_path = pkg_path / 'urdf/proto_L.urdf'
    default_rviz_path  = pkg_path / 'rviz/urdf.rviz'

    gui_arg         = DeclareLaunchArgument('gui', default_value='false', choices=['true','false'])
    model_arg       = DeclareLaunchArgument('model', default_value=str(default_model_path))
    rviz_arg        = DeclareLaunchArgument('rvizconfig', default_value=str(default_rviz_path))
    use_sim_time_arg= DeclareLaunchArgument('use_sim_time', default_value='false')

    # Si tu URDF es xacro, deja el 'xacro ' delante
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),

                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
     
                    {'transform_tolerance': 0.1}, # tolerancia de 100ms
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        
        #remappings=[('odometry/filtered', '/demo/odom_filtered')]
        
    )

    return LaunchDescription([
        gui_arg, 
        model_arg, 
        rviz_arg, 
        use_sim_time_arg,
        jsp, 
        jsp_gui, 
        rsp, 
        ekf, 
        rviz
    ])
