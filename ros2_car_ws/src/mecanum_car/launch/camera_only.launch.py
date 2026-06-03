"""
仅启动相机节点 (运行在树莓派)

用法:
  ros2 launch mecanum_car camera_only.launch.py

话题:
  /camera/image_raw           — 原始图像
  /camera/image_raw/compressed — JPEG 压缩 (自动，需 image_transport)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0',
                              description='USB camera device ID'),
        DeclareLaunchArgument('width', default_value='640',
                              description='Image width'),
        DeclareLaunchArgument('height', default_value='480',
                              description='Image height'),
        DeclareLaunchArgument('fps', default_value='30',
                              description='Capture frame rate'),

        Node(
            package='mecanum_car',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
            }],
        ),
    ])
