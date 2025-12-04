
# mpu_ws :  sensor mpu + motor
colcon build

source install/setup.bash

ros2 launch imu_publisher sensor.launch.py

# lidar2_ws : Lidar
colcon build

source install/setup.bash

ros2 launch sllidar_ros2 sllidar_a2m12_launch.py

# rosbot_ws  :  EKF + TF + URDF
colcon build

source install/setup.bash

ros2 launch robot_description display.launch.py

# navegation_ws - Mapeo 2D
colcon build

source install/setup.bash

ros2 launch robot_cartographer cartographer.launch.py
