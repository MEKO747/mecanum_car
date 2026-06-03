from setuptools import setup
import os
from glob import glob

package_name = 'mecanum_car'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zguss',
    maintainer_email='zguss@example.com',
    description='ROS2 Mecanum wheel car controller with PCA9685 motor driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_controller = mecanum_car.car_controller:main',
            'ir_avoidance = mecanum_car.ir_avoidance:main',
            'ir_sensor = mecanum_car.ir_sensor:main',
            'mecanum_keyboard = mecanum_car.mecanum_keyboard:main',
            'camera_avoidance = mecanum_car.camera_avoidance:main',
        ],
    },
)
