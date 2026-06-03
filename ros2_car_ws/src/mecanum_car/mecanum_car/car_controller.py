"""
ROS2 麦克纳姆轮运动控制节点
订阅 /cmd_vel (geometry_msgs/Twist) → 麦克纳姆轮运动学解算 → PCA9685 电机驱动
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .motor_driver import MotorDriver


class CarController(Node):
    """麦克纳姆轮小车控制节点"""

    def __init__(self):
        super().__init__('car_controller')

        # 声明参数
        self.declare_parameter('pca9685_address', 0x40)
        self.declare_parameter('max_linear_speed', 1.0)   # m/s
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s
        self.declare_parameter('wheel_base_x', 0.15)      # 半轮距（左右），单位 m
        self.declare_parameter('wheel_base_y', 0.12)      # 半轴距（前后），单位 m
        self.declare_parameter('timeout', 1.0)            # 无指令自动停止时间，秒
        self.declare_parameter('deadband', 0.01)          # 小于该值视为停止

        # 获取参数
        pca_addr = self.get_parameter('pca9685_address').value
        # 支持 0x40 字符串或整数 64
        if isinstance(pca_addr, str):
            pca_addr = int(pca_addr, 0)
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.wheel_x = self.get_parameter('wheel_base_x').value
        self.wheel_y = self.get_parameter('wheel_base_y').value
        self.timeout = self.get_parameter('timeout').value
        self.deadband = self.get_parameter('deadband').value
        self._stopped = True

        # 初始化电机驱动
        self.get_logger().info(f'Initializing PCA9685 at 0x{pca_addr:02X}...')
        self.driver = MotorDriver(address=pca_addr)
        self.get_logger().info('Motor driver initialized OK')

        # 订阅 /cmd_vel
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )

        # 超时定时器：如果 timeout 秒内没有收到新指令，自动停止
        self.last_cmd_time = self.get_clock().now()
        self.watchdog = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info('Car controller node started, waiting for /cmd_vel...')

    def cmd_callback(self, msg: Twist):
        """接收 /cmd_vel 指令，解算麦克纳姆轮运动学"""
        self.last_cmd_time = self.get_clock().now()

        # 提取速度分量
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.angular.z

        if abs(vx) < self.deadband:
            vx = 0.0
        if abs(vy) < self.deadband:
            vy = 0.0
        if abs(vz) < self.deadband:
            vz = 0.0

        # 限制最大速度
        vx = max(-self.max_linear, min(self.max_linear, vx))
        vy = max(-self.max_linear, min(self.max_linear, vy))
        vz = max(-self.max_angular, min(self.max_angular, vz))

        if vx == 0.0 and vy == 0.0 and vz == 0.0:
            self.driver.stop_all()
            self._stopped = True
            return

        # 各自按上限归一化到 [-1, 1]
        vx_norm = vx / self.max_linear
        vy_norm = vy / self.max_linear
        vz_norm = vz / self.max_angular

        # 麦克纳姆轮运动学解算（vy 符号已对齐原版 LOBOROBOT.py moveLeft/moveRight）
        fl = vx_norm + vy_norm - vz_norm
        fr = vx_norm - vy_norm + vz_norm
        rl = vx_norm - vy_norm - vz_norm
        rr = vx_norm + vy_norm + vz_norm

        # 饱和限制：仅在超出物理上限时等比缩放
        max_speed = max(abs(fl), abs(fr), abs(rl), abs(rr), 1.0)
        if max_speed > 1.0:
            fl /= max_speed
            fr /= max_speed
            rl /= max_speed
            rr /= max_speed

        # 驱动电机
        self.driver.set_speeds([fl, fr, rl, rr])
        self._stopped = False

        self.get_logger().debug(
            f'cmd_vel: vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} -> '
            f'motors: fl={fl:.2f} fr={fr:.2f} rl={rl:.2f} rr={rr:.2f}'
        )

    def watchdog_callback(self):
        """超时自动停止"""
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.timeout and not self._stopped:
            self.driver.stop_all()
            self._stopped = True

    def destroy_node(self):
        self.get_logger().info('Shutting down, stopping motors...')
        self.driver.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
