"""
ROS2 麦克纳姆轮键盘控制节点
读取终端键盘输入 → 发布 geometry_msgs/Twist 到 /cmd_vel
"""

import atexit
import os
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


KEY_DIRECTIONS: dict[str, tuple[float, float, float]] = {
    'i': ( 1.0,  0.0,  0.0),   # 前进
    ',': (-1.0,  0.0,  0.0),   # 后退
    'j': ( 0.0,  1.0,  0.0),   # 左平移
    'l': ( 0.0, -1.0,  0.0),   # 右平移
    'u': ( 1.0,  1.0,  0.0),   # 左前对角
    'p': ( 1.0, -1.0,  0.0),   # 右前对角
    'm': (-1.0,  1.0,  0.0),   # 左后对角
    '.': (-1.0, -1.0,  0.0),   # 右后对角
    'k': ( 0.0,  0.0,  0.0),   # 停止
    'y': ( 0.0,  0.0,  1.0),   # 左自旋
    'o': ( 0.0,  0.0, -1.0),   # 右自旋
}

DIR_NAMES: dict[str, str] = {
    'i': '前进',
    ',': '后退',
    'j': '左移',
    'l': '右移',
    'u': '左前对角',
    'p': '右前对角',
    'm': '左后对角',
    '.': '右后对角',
    'k': '停止',
    'y': '左自旋',
    'o': '右自旋',
}

HELP_TEXT = """
麦克纳姆轮键盘控制
───────────────────────────────
    u    i    p      前进/后退: i / ,
    |    |    |      左移/右移: j / l
  j ── + ── l       对角移动: u p m .
    |    |    |      左自旋: y  右自旋: o
    m    ,    .      停止:     k
    y         o      加速: q  减速: e

当前速度: {speed:.1f} m/s
按 s 显示此帮助  按 Ctrl+C 退出
───────────────────────────────"""


class MecanumKeyboard(Node):
    """麦克纳姆轮键盘控制节点"""

    def __init__(self):
        super().__init__('mecanum_keyboard')

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('default_speed', 0.3)
        self.declare_parameter('min_speed', 0.1)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('speed_step', 0.1)
        self.declare_parameter('key_timeout', 1.0)
        self.declare_parameter('rotation_speed_ratio', 2.0)

        publish_rate = self.get_parameter('publish_rate').value
        self._speed = self.get_parameter('default_speed').value
        self._min_speed = self.get_parameter('min_speed').value
        self._max_speed = self.get_parameter('max_speed').value
        self._speed_step = self.get_parameter('speed_step').value
        self._key_timeout = self.get_parameter('key_timeout').value
        self._rotation_ratio = self.get_parameter('rotation_speed_ratio').value

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._timer = self.create_timer(1.0 / publish_rate, self._timer_callback)

        self._lock = threading.Lock()
        self._active_key: str | None = None
        self._last_activity_time = self.get_clock().now()

        if not sys.stdin.isatty():
            self.get_logger().warn('stdin is not a TTY, keyboard input disabled')

        print(HELP_TEXT.format(speed=self._speed))

        if sys.stdin.isatty():
            self._old_termios = termios.tcgetattr(sys.stdin.fileno())
            atexit.register(self._restore_terminal)
            self._thread = threading.Thread(target=self._keyboard_listener, daemon=True)
            self._thread.start()

    def _restore_terminal(self):
        """恢复终端设置"""
        try:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old_termios)
        except (termios.error, OSError):
            pass

    def _keyboard_listener(self):
        fd = sys.stdin.fileno()
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                try:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if rlist:
                        ch_bytes = os.read(fd, 1)
                        ch = ch_bytes.decode('utf-8')
                        self._handle_key(ch)
                except (select.error, InterruptedError, ValueError, OSError):
                    continue
        finally:
            self._restore_terminal()

    def _handle_key(self, ch: str):
        with self._lock:
            if ch in KEY_DIRECTIONS:
                self._active_key = ch
                self._last_activity_time = self.get_clock().now()
                name = DIR_NAMES.get(ch, ch)
                vx, vy, vz = KEY_DIRECTIONS[ch]
                print(f'  [{name}] vx={vx*self._speed:.2f} vy={vy*self._speed:.2f} vz={vz*self._speed*self._rotation_ratio:.2f}')
            elif ch == 'q':
                self._speed = min(self._speed + self._speed_step, self._max_speed)
                print(f'  速度: {self._speed:.1f} m/s')
            elif ch == 'e':
                self._speed = max(self._speed - self._speed_step, self._min_speed)
                print(f'  速度: {self._speed:.1f} m/s')
            elif ch == 's':
                print(HELP_TEXT.format(speed=self._speed))

    def _timer_callback(self):
        twist = Twist()

        with self._lock:
            now = self.get_clock().now()
            dt = (now - self._last_activity_time).nanoseconds / 1e9

            if self._active_key is not None and dt <= self._key_timeout:
                vx_norm, vy_norm, vz_norm = KEY_DIRECTIONS[self._active_key]
                twist.linear.x = vx_norm * self._speed
                twist.linear.y = vy_norm * self._speed
                twist.angular.z = vz_norm * self._speed * self._rotation_ratio

        self._pub.publish(twist)

    def destroy_node(self):
        self.get_logger().info('Shutting down keyboard control...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MecanumKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
