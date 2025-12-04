from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    return LaunchDescription([

        Node(

            package='robot_description',
            executable='robot_height_scan.py',   # <- si usas setup.py con entry_point


            name='robot_height_scan',
            output='screen',
            parameters=[{
                'input_cloud': '/points2',    # tu nube 3D
                'scan_topic':  '/scan',       # salida LaserScan
                'scan_frame':  'base_scan',   # usa un frame que *exista* en TF
                'tilt_deg': 0.0, #90.0,             # sensor está ROTADO
                'pitch_min_deg':  2.0,
                'pitch_max_deg': 17.0,
                'yaw_min_deg':  -180.0,
                'yaw_max_deg':   180.0,
                'bin_width_deg': 1.0,
                'range_min': 0.10,
                'range_max': 20.0,
                'percentile': 0.15,
                'stride': 1,
                'front_sector_deg': 180.0,
                'use_horizontal_range': True
            }]

        )


    ])
