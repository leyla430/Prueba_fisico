from setuptools import find_packages, setup

package_name = 'imu_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', ['launch/sensor.launch.py']),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leyla',
    maintainer_email='leyla@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'imu = imu_publisher.IMU:main',
    
            'motor= imu_publisher.motor:main',

            'cpr= imu_publisher.cpr:main',

            'odom= imu_publisher.odom:main',

            'pid= imu_publisher.pid:main',
            


        ],
    },
)
