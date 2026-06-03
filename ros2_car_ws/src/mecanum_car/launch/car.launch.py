"""
启动文件：启动麦克纳姆轮小车控制节点
用法: ros2 launch mecanum_car car.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pca_address = LaunchConfiguration('pca_address', default='0x40')
    max_linear = LaunchConfiguration('max_linear_speed', default='1.0')
    max_angular = LaunchConfiguration('max_angular_speed', default='2.0')
    wheel_x = LaunchConfiguration('wheel_base_x', default='0.15')
    wheel_y = LaunchConfiguration('wheel_base_y', default='0.12')
    timeout = LaunchConfiguration('timeout', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument('pca_address', default_value='0x40',
                              description='PCA9685 I2C address (hex)'),
        DeclareLaunchArgument('max_linear_speed', default_value='1.0',
                              description='Maximum linear speed (m/s)'),
        DeclareLaunchArgument('max_angular_speed', default_value='2.0',
                              description='Maximum angular speed (rad/s)'),
        DeclareLaunchArgument('wheel_base_x', default_value='0.15',
                              description='Half track width (m)'),
        DeclareLaunchArgument('wheel_base_y', default_value='0.12',
                              description='Half wheelbase length (m)'),
        DeclareLaunchArgument('timeout', default_value='1.0',
                              description='Auto-stop timeout (seconds)'),

        Node(
            package='mecanum_car',
            executable='car_controller',
            name='car_controller',
            output='screen',
            parameters=[{
                'pca9685_address': pca_address,
                'max_linear_speed': max_linear,
                'max_angular_speed': max_angular,
                'wheel_base_x': wheel_x,
                'wheel_base_y': wheel_y,
                'timeout': timeout,
            }],
        ),
    ])
