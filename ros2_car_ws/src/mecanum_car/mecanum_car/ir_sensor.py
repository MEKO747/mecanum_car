"""
红外传感器节点
GPIO 16 = 左红外传感器
GPIO 12 = 右红外传感器
0=障碍物 (active-low), 1=无障碍
发布 /ir/status (Int32): bit0=左被挡, bit1=右被挡
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import lgpio


class IRSensor(Node):

    def __init__(self):
        super().__init__('ir_sensor')

        self.declare_parameter('debounce_samples', 3)
        self._debounce_samples = max(1, self.get_parameter('debounce_samples').value)
        self._candidate_bits = None
        self._candidate_count = 0
        self._stable_bits = 0

        self._h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_input(self._h, 16, lgpio.SET_PULL_UP)
        lgpio.gpio_claim_input(self._h, 12, lgpio.SET_PULL_UP)

        self._pub = self.create_publisher(Int32, '/ir/status', 10)
        self.create_timer(0.05, self._timer_cb)
        self.get_logger().info('IR sensor node started (GPIO 16, 12)')

    def _timer_cb(self):
        left_raw = lgpio.gpio_read(self._h, 16)   # 0=障碍物
        right_raw = lgpio.gpio_read(self._h, 12)  # 0=障碍物
        raw_bits = 0
        if left_raw == 0:
            raw_bits |= 0b01
        if right_raw == 0:
            raw_bits |= 0b10

        if raw_bits == self._candidate_bits:
            self._candidate_count += 1
        else:
            self._candidate_bits = raw_bits
            self._candidate_count = 1

        if self._candidate_count >= self._debounce_samples:
            self._stable_bits = raw_bits

        msg = Int32()
        msg.data = self._stable_bits
        self._pub.publish(msg)

    def destroy_node(self):
        lgpio.gpiochip_close(self._h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IRSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
