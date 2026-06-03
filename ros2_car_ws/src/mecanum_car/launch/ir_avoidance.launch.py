"""
启动红外避障系统：ir_sensor + ir_avoidance + car_controller
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_car',
            executable='ir_sensor',
            name='ir_sensor',
            output='screen',
        ),
        Node(
            package='mecanum_car',
            executable='ir_avoidance',
            name='ir_avoidance',
            output='screen',
            parameters=[{
                'forward_speed': 0.3,
                'turn_speed': 0.4,
                'backup_speed': 0.3,
                'backup_duration': 0.4,
                'turn_duration': 0.5,
            }],
        ),
        Node(
            package='mecanum_car',
            executable='car_controller',
            name='car_controller',
            output='screen',
            parameters=[{
                'pca9685_address': 0x40,
                'max_linear_speed': 1.0,
                'max_angular_speed': 2.0,
            }],
        ),
    ])
