"""
启动文件：麦克纳姆轮键盘控制节点
用法: ros2 launch mecanum_car keyboard.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_rate = LaunchConfiguration('publish_rate', default='20.0')
    default_speed = LaunchConfiguration('default_speed', default='0.3')
    max_speed = LaunchConfiguration('max_speed', default='1.0')
    key_timeout = LaunchConfiguration('key_timeout', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument('publish_rate', default_value='20.0',
                              description='Twist 发布频率 (Hz)'),
        DeclareLaunchArgument('default_speed', default_value='0.3',
                              description='初始线速度 (m/s)'),
        DeclareLaunchArgument('max_speed', default_value='1.0',
                              description='最大线速度 (m/s)'),
        DeclareLaunchArgument('key_timeout', default_value='1.0',
                              description='松开按键后自动停止超时 (s)'),

        Node(
            package='mecanum_car',
            executable='mecanum_keyboard',
            name='mecanum_keyboard',
            output='screen',
            parameters=[{
                'publish_rate': publish_rate,
                'default_speed': default_speed,
                'max_speed': max_speed,
                'key_timeout': key_timeout,
            }],
        ),
    ])
