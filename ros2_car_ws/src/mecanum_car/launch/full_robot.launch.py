"""
启动树莓派端完整机器人系统

启动节点:
  - camera_node      — USB 相机发布
  - car_controller   — 电机驱动 (订阅 /cmd_vel)
  - ir_sensor        — 红外传感器 (安全层)

配合上位机 (笔记本) 运行的:
  ros2 launch embodied_agent agent_system.launch.py

用法:
  ros2 launch mecanum_car full_robot.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # 相机参数
        DeclareLaunchArgument('camera_device', default_value='0',
                              description='USB camera device ID'),

        # ── 相机节点 ──
        Node(
            package='mecanum_car',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('camera_device'),
                'width': 640,
                'height': 480,
                'fps': 30,
            }],
        ),

        # ── 电机控制 (订阅 /cmd_vel, 由笔记本 cmd_vel_mux 发布) ──
        Node(
            package='mecanum_car',
            executable='car_controller',
            name='car_controller',
            output='screen',
            parameters=[{
                'pca9685_address': 0x40,
                'max_linear_speed': 1.0,
                'max_angular_speed': 2.0,
                'wheel_base_x': 0.15,
                'wheel_base_y': 0.12,
                'timeout': 1.0,
            }],
        ),

        # ── 红外传感器 (安全层, 发布 /ir/status) ──
        Node(
            package='mecanum_car',
            executable='ir_sensor',
            name='ir_sensor',
            output='screen',
            parameters=[{
                'debounce_samples': 3,
            }],
        ),
    ])
