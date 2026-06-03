"""
红外避障节点
订阅 /ir/status (Int32) → 非阻塞状态机 → 发布 /cmd_vel
bit0=左传感器被挡, bit1=右传感器被挡
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

STATE_STOPPED    = 'STOPPED'
STATE_FORWARD    = 'FORWARD'
STATE_BACK_UP    = 'BACK_UP'
STATE_TURN_AWAY  = 'TURN_AWAY'
STATE_TURN_LEFT  = 'TURN_LEFT'
STATE_TURN_RIGHT = 'TURN_RIGHT'


class IRAvoidance(Node):

    def __init__(self):
        super().__init__('ir_avoidance')

        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('turn_speed', 0.4)
        self.declare_parameter('backup_speed', 0.25)
        self.declare_parameter('backup_duration', 0.5)
        self.declare_parameter('turn_duration', 0.6)
        self.declare_parameter('clear_hold_duration', 0.5)
        self.declare_parameter('deadband', 0.01)

        self._fwd_speed = self.get_parameter('forward_speed').value
        self._turn_speed = self.get_parameter('turn_speed').value
        self._back_speed = self.get_parameter('backup_speed').value
        self._backup_sec = self.get_parameter('backup_duration').value
        self._turn_sec = self.get_parameter('turn_duration').value
        self._clear_hold_sec = self.get_parameter('clear_hold_duration').value
        self._deadband = self.get_parameter('deadband').value

        self._state = STATE_STOPPED
        self._state_enter = self.get_clock().now()
        self._last_msg_time = self.get_clock().now()
        self._last_bits = -1  # 哨兵值，首条消息到达前不动作
        self._clear_since = None

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Int32, '/ir/status', self._sensor_cb, 10)
        self.create_timer(0.05, self._timer_cb)

        self.get_logger().info('IR avoidance node started')

    def _sensor_cb(self, msg):
        self._last_msg_time = self.get_clock().now()
        bits = msg.data

        if bits != self._last_bits:
            names = []
            if bits & 0b01:
                names.append('LEFT')
            if bits & 0b10:
                names.append('RIGHT')
            desc = '+'.join(names) if names else 'CLEAR'
            self.get_logger().info(f'IR: {desc}')
            self._last_bits = bits

        if bits == 0:
            if self._clear_since is None:
                self._clear_since = self.get_clock().now()
        else:
            self._clear_since = None

    def _elapsed(self):
        return (self.get_clock().now() - self._state_enter).nanoseconds / 1e9

    def _enter(self, new_state):
        if self._state == new_state:
            return
        self._state = new_state
        self._state_enter = self.get_clock().now()
        self.get_logger().info(f'State -> {new_state}')

    def _clear_stable(self):
        if self._clear_since is None:
            return False
        elapsed = (self.get_clock().now() - self._clear_since).nanoseconds / 1e9
        return elapsed >= self._clear_hold_sec

    def _avoid_state_for_bits(self, left_b, right_b):
        if left_b and right_b:
            return STATE_BACK_UP
        if left_b:
            return STATE_TURN_RIGHT
        if right_b:
            return STATE_TURN_LEFT
        return STATE_FORWARD

    def _timer_cb(self):
        now = self.get_clock().now()

        # 看门狗：1 秒无传感器数据 → 停车
        dt_msg = (now - self._last_msg_time).nanoseconds / 1e9
        if dt_msg > 1.0:
            self._enter(STATE_STOPPED)
            self._publish(Twist())
            return

        # 首条传感器消息到达前不动作
        if self._last_bits < 0:
            return

        bits = self._last_bits
        left_b = bool(bits & 0b01)
        right_b = bool(bits & 0b10)
        obstacle = left_b or right_b
        elapsed = self._elapsed()

        if self._state == STATE_STOPPED:
            self._enter(self._avoid_state_for_bits(left_b, right_b))

        # ---- 状态机 ----
        if self._state == STATE_FORWARD:
            # 进入 FORWARD 后 0.3s 宽限期，避免反复触发
            if elapsed < 0.3:
                pass
            elif left_b and right_b:
                self._enter(STATE_BACK_UP)
            elif left_b:
                self._enter(STATE_TURN_RIGHT)
            elif right_b:
                self._enter(STATE_TURN_LEFT)

        elif self._state == STATE_BACK_UP:
            if elapsed >= self._backup_sec:
                # 后退结束，根据当前传感器状态决定转向方向
                if left_b and not right_b:
                    self._enter(STATE_TURN_RIGHT)
                elif right_b and not left_b:
                    self._enter(STATE_TURN_LEFT)
                else:
                    self._enter(STATE_TURN_AWAY)  # 两侧都挡或都通

        elif self._state == STATE_TURN_AWAY:
            # 持续转向直到传感器清除或超时
            if self._clear_stable():
                self._enter(STATE_FORWARD)
            elif elapsed >= self._turn_sec:
                self._enter(self._avoid_state_for_bits(left_b, right_b))

        elif self._state in (STATE_TURN_LEFT, STATE_TURN_RIGHT):
            # 单侧避障：只有稳定清除后恢复前进，否则按当前传感器继续避让。
            if self._clear_stable():
                self._enter(STATE_FORWARD)
            elif elapsed >= self._turn_sec:
                self._enter(self._avoid_state_for_bits(left_b, right_b))

        # ---- 发布 /cmd_vel 指令 ----
        twist = Twist()
        if self._state == STATE_FORWARD:
            twist.linear.x = self._fwd_speed
        elif self._state == STATE_BACK_UP:
            twist.linear.x = -self._back_speed
        elif self._state == STATE_TURN_LEFT:
            twist.angular.z = self._turn_speed
        elif self._state == STATE_TURN_RIGHT:
            twist.angular.z = -self._turn_speed
        elif self._state == STATE_TURN_AWAY:
            # 根据传感器选转向方向
            if left_b and not right_b:
                twist.angular.z = -self._turn_speed  # 仅右被挡 → 右转绕开
            else:
                twist.angular.z = self._turn_speed   # 其他情况 → 左转
        # STATE_STOPPED 保持全零

        self._publish(twist)

    def _publish(self, twist: Twist):
        self._pub.publish(twist)

    def destroy_node(self):
        self._publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IRAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
