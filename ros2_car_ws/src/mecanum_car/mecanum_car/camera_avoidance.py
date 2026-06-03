"""
摄像头辅助红外避障节点
订阅 /ir/status → 红外触发后拍照分析左右空间 → 发布 /cmd_vel 横向平移绕行
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

import threading
import time

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import smbus2
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


STATE_STOPPED       = 'STOPPED'
STATE_FORWARD       = 'FORWARD'
STATE_BACK_UP       = 'BACK_UP'
STATE_CAPTURE       = 'CAPTURE'
STATE_AVOID_LEFT    = 'AVOID_LEFT'
STATE_AVOID_RIGHT   = 'AVOID_RIGHT'
STATE_AVOID_DEFAULT = 'AVOID_DEFAULT'
STATE_ROTATE        = 'ROTATE'


class CameraAvoidance(Node):

    def __init__(self):
        super().__init__('camera_avoidance')

        # 参数声明
        self.declare_parameter('forward_speed', 0.25)
        self.declare_parameter('strafe_speed', 0.3)
        self.declare_parameter('backup_speed', 0.2)
        self.declare_parameter('backup_duration', 0.3)
        self.declare_parameter('avoid_duration', 0.6)
        self.declare_parameter('clear_hold_duration', 0.3)
        self.declare_parameter('decision_threshold', 0.01)
        self.declare_parameter('camera_width', 320)
        self.declare_parameter('camera_height', 240)
        self.declare_parameter('crop_top', 0.25)
        self.declare_parameter('crop_bottom', 0.10)
        self.declare_parameter('center_exclude', 0.10)
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('default_direction', 'right')
        self.declare_parameter('deadband', 0.01)
        # 双舵机 — 匹配 CLBROBOT 硬件：通道 10=底座/pan(左右), 通道 9=倾斜/tilt(上下)
        self.declare_parameter('servo_pan_channel', 10)
        self.declare_parameter('servo_tilt_channel', 9)
        self.declare_parameter('servo_pan_center', 380)   # pan 正前方
        self.declare_parameter('servo_pan_left', 280)     # pan 左转45°
        self.declare_parameter('servo_pan_right', 480)    # pan 右转45°
        self.declare_parameter('servo_tilt_center', 130)  # tilt 中心 PWM（平视前方）
        self.declare_parameter('servo_settle_time', 0.3)  # 单步稳定时间
        self.declare_parameter('servo_smooth_steps', 8)   # 渐进移动步数，越大越平滑
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('rotate_duration', 0.8)

        self._fwd_speed    = self.get_parameter('forward_speed').value
        self._strafe_speed = self.get_parameter('strafe_speed').value
        self._back_speed   = self.get_parameter('backup_speed').value
        self._backup_sec   = self.get_parameter('backup_duration').value
        self._avoid_sec    = self.get_parameter('avoid_duration').value
        self._clear_hold   = self.get_parameter('clear_hold_duration').value
        self._dec_thresh   = self.get_parameter('decision_threshold').value
        self._cam_w        = self.get_parameter('camera_width').value
        self._cam_h        = self.get_parameter('camera_height').value
        self._crop_top     = self.get_parameter('crop_top').value
        self._crop_bottom  = self.get_parameter('crop_bottom').value
        self._center_excl  = self.get_parameter('center_exclude').value
        self._canny_low    = self.get_parameter('canny_low').value
        self._canny_high   = self.get_parameter('canny_high').value
        self._default_dir  = self.get_parameter('default_direction').value
        self._deadband     = self.get_parameter('deadband').value
        self._pan_ch       = self.get_parameter('servo_pan_channel').value
        self._tilt_ch      = self.get_parameter('servo_tilt_channel').value
        self._pan_center   = self.get_parameter('servo_pan_center').value
        self._pan_left     = self.get_parameter('servo_pan_left').value
        self._pan_right    = self.get_parameter('servo_pan_right').value
        self._tilt_center  = self.get_parameter('servo_tilt_center').value
        self._servo_settle = self.get_parameter('servo_settle_time').value
        self._smooth_steps = self.get_parameter('servo_smooth_steps').value
        self._max_retries  = self.get_parameter('max_retries').value
        self._turn_speed   = self.get_parameter('turn_speed').value
        self._rotate_sec   = self.get_parameter('rotate_duration').value

        # 状态机
        self._state        = STATE_STOPPED
        self._state_enter  = self.get_clock().now()
        self._last_bits    = -1
        self._clear_since  = None
        self._last_ir_time = self.get_clock().now()
        self._retry_count  = 0

        # 异步拍照
        self._capture_thread: threading.Thread | None = None
        self._capture_result: str | None = None

        # 手动覆盖（时间戳过滤自发布消息）
        self._manual_override   = False
        self._last_external_cmd = self.get_clock().now()
        self._last_pub_time     = self.get_clock().now()
        self._self_filter_ms    = 0.15  # 发布后 150ms 内收到的 Twist 视为自己的

        # 初始化 USB 摄像头（cv2.VideoCapture）
        self._camera_ok = False
        self._cap = None
        if HAS_CV2 and HAS_NUMPY:
            try:
                self._cap = cv2.VideoCapture(0)
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._cam_w)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._cam_h)
                if self._cap.isOpened():
                    self._camera_ok = True
                    actual_w = self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                    actual_h = self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                    self.get_logger().info(f'Camera ready: {actual_w:.0f}x{actual_h:.0f}')
                else:
                    self.get_logger().error('Camera open failed')
                    self._cap.release()
                    self._cap = None
            except Exception as e:
                self.get_logger().error(f'Camera init failed: {e}')
                self._cap = None
        else:
            self.get_logger().warn('cv2 or numpy not available, camera disabled')

        if not self._camera_ok:
            self.get_logger().warn('Camera unavailable, falling back to blind IR avoidance')

        # 舵机全部禁用 — pan 舵机物理损坏，tilt 舵机 PWM=130 堵转
        self._servo_ok = False
        self.get_logger().info('All servos disabled (pan damaged, tilt stalled)')

        # 发布和订阅
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Int32, '/ir/status', self._ir_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_timer(0.05, self._timer_cb)

        self.get_logger().info('Camera avoidance node started')

    # ── 订阅回调 ─────────────────────────────────────────

    def _ir_cb(self, msg: Int32):
        self._last_ir_time = self.get_clock().now()
        bits = msg.data
        if bits != self._last_bits:
            names = []
            if bits & 0b01: names.append('LEFT')
            if bits & 0b10: names.append('RIGHT')
            desc = '+'.join(names) if names else 'CLEAR'
            self.get_logger().info(f'IR: {desc}')
            self._last_bits = bits

        if bits == 0:
            if self._clear_since is None:
                self._clear_since = self.get_clock().now()
        else:
            self._clear_since = None

    def _cmd_cb(self, msg: Twist):
        # 时间戳过滤：发布后短时间内收到的 Twist 视为自己的消息
        now = self.get_clock().now()
        if (now - self._last_pub_time).nanoseconds / 1e9 < self._self_filter_ms:
            return

        moving = (
            abs(msg.linear.x)  >= self._deadband or
            abs(msg.linear.y)  >= self._deadband or
            abs(msg.angular.z) >= self._deadband
        )
        if moving:
            if not self._manual_override:
                self.get_logger().info('External /cmd_vel detected, manual override')
            self._manual_override = True
            self._last_external_cmd = now
        else:
            self._last_external_cmd = now

    # ── 辅助方法 ─────────────────────────────────────────

    def _set_servo(self, channel: int, count: int):
        """设置指定通道的舵机 PWM 脉冲宽度（计数值，0-4095）"""
        reg = 0x06 + 4 * channel
        self._i2c.write_byte_data(self._pca_addr, reg, 0)
        self._i2c.write_byte_data(self._pca_addr, reg + 1, 0)
        self._i2c.write_byte_data(self._pca_addr, reg + 2, count & 0xFF)
        self._i2c.write_byte_data(self._pca_addr, reg + 3, count >> 8)

    def _smooth_servo_move(self, channel: int, from_pwm: int, to_pwm: int, steps: int = None, delay: float = None):
        """渐进移动舵机，分步到达目标位置，避免跳变过冲"""
        if steps is None:
            steps = self._smooth_steps
        if delay is None:
            delay = self._servo_settle / steps
        for i in range(1, steps + 1):
            t = i / steps
            # 使用 ease-in-out 曲线：t' = 3*t² - 2*t³
            eased = 3 * t * t - 2 * t * t * t
            val = int(from_pwm + (to_pwm - from_pwm) * eased)
            self._set_servo(channel, val)
            time.sleep(delay)

    def _elapsed(self):
        return (self.get_clock().now() - self._state_enter).nanoseconds / 1e9

    def _enter(self, new_state: str):
        if self._state == new_state:
            return
        self.get_logger().info(f'State: {self._state} -> {new_state}')
        self._state = new_state
        self._state_enter = self.get_clock().now()

        # CAPTURE 状态：后台线程异步拍照，timer 回调检测完成后过渡
        if new_state == STATE_CAPTURE:
            self._last_ir_time = self.get_clock().now()
            self._clear_since = None
            self._capture_result = None
            self._capture_thread = threading.Thread(target=self._do_capture, daemon=True)
            self._capture_thread.start()

    def _do_capture(self):
        """后台执行拍照分析，结果存入 _capture_result"""
        direction = self._capture_and_decide()
        if direction == 'unknown':
            direction = self._blind_decision()
        self._capture_result = direction
        self.get_logger().info(f'Camera capture done: {direction}')

    def _clear_stable(self) -> bool:
        if self._clear_since is None:
            return False
        dt = (self.get_clock().now() - self._clear_since).nanoseconds / 1e9
        return dt >= self._clear_hold

    def _publish(self, twist: Twist):
        self._last_pub_time = self.get_clock().now()
        self._pub.publish(twist)

    # ── 摄像头分析 ───────────────────────────────────────

    def _capture_and_decide(self) -> str:
        """单帧拍照，对比画面左半/右半边缘密度。返回 'left' / 'right' / 'unknown'。"""
        if not self._camera_ok:
            return self._blind_decision()

        try:
            # 丢弃前几帧让曝光稳定
            for _ in range(2):
                self._cap.read()
            time.sleep(0.05)

            ret, frame = self._cap.read()
            if not ret or frame is None:
                return self._blind_decision()

            h, w = frame.shape[:2]
            mid = w // 2
            left_half = frame[:, :mid]
            right_half = frame[:, mid:]

            left_density = self._edge_density(left_half)
            right_density = self._edge_density(right_half)

            self.get_logger().info(
                f'Single-shot: L={left_density:.4f} R={right_density:.4f} '
                f'(diff={abs(left_density - right_density):.4f} thresh={self._dec_thresh})'
            )

            if right_density > left_density + self._dec_thresh:
                return 'left'   # 右边边缘多 → 右边有障碍 → 选左
            elif left_density > right_density + self._dec_thresh:
                return 'right'  # 左边边缘多 → 左边有障碍 → 选右
            else:
                return 'unknown'

        except Exception as e:
            self.get_logger().error(f'Capture analysis failed: {e}')
            return self._blind_decision()

    def _edge_density(self, frame: np.ndarray) -> float:
        """计算全帧边缘密度（摄像头已朝向目标方向，不需分左右）。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if np.mean(gray) < 10.0:
            return 0.0
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, self._canny_low, self._canny_high)
        h, _ = edges.shape
        top_crop = int(h * self._crop_top)
        bottom_crop = int(h * self._crop_bottom)
        roi = edges[top_crop:h - bottom_crop, :]
        return float(np.count_nonzero(roi)) / roi.size if roi.size > 0 else 0.0

    def _blind_decision(self) -> str:
        """摄像头不可用时的盲决策：根据 IR 传感器状态选择方向。"""
        bits = self._last_bits if self._last_bits >= 0 else 0
        left_b  = bool(bits & 0b01)
        right_b = bool(bits & 0b10)

        if left_b and not right_b:
            self.get_logger().info('Blind: left blocked -> avoid right')
            return 'right'
        elif right_b and not left_b:
            self.get_logger().info('Blind: right blocked -> avoid left')
            return 'left'
        else:
            self.get_logger().info(f'Blind: using default direction ({self._default_dir})')
            return self._default_dir

    # ── 状态机主循环 ─────────────────────────────────────

    def _timer_cb(self):
        now = self.get_clock().now()

        # 看门狗：1 秒无 IR 数据 → 停车
        if (now - self._last_ir_time).nanoseconds / 1e9 > 1.0:
            if self._state != STATE_STOPPED:
                self._enter(STATE_STOPPED)
            self._publish(Twist())
            return

        # 手动覆盖：外部 /cmd_vel 非零 → 让出控制权
        if self._manual_override:
            dt_manual = (now - self._last_external_cmd).nanoseconds / 1e9
            if dt_manual > 2.0:
                self._manual_override = False
                self.get_logger().info('Manual override released, resuming auto')
                self._enter(STATE_STOPPED)
            else:
                return  # 不发布任何指令

        # 等待首条 IR 消息
        if self._last_bits < 0:
            return

        bits = self._last_bits
        left_b  = bool(bits & 0b01)
        right_b = bool(bits & 0b10)
        obstacle = left_b or right_b
        elapsed = self._elapsed()

        # ── 状态转换 ──

        if self._state == STATE_STOPPED:
            if obstacle:
                self._enter(STATE_BACK_UP)
            else:
                self._enter(STATE_FORWARD)

        elif self._state == STATE_FORWARD:
            self._retry_count = 0  # 回到前进，重置重试计数
            if elapsed < 0.3:
                pass  # 宽限期，避免反复触发
            elif obstacle:
                self._enter(STATE_BACK_UP)

        elif self._state == STATE_BACK_UP:
            if elapsed >= self._backup_sec:
                self._enter(STATE_CAPTURE)  # 异步拍照 → AVOID_*

        elif self._state == STATE_CAPTURE:
            # 等待后台拍照线程完成，期间不发电机指令
            if self._capture_thread and self._capture_thread.is_alive():
                return
            # 拍照完成 → 过渡到对应 AVOID 状态
            direction = self._capture_result or self._blind_decision()
            target = STATE_AVOID_DEFAULT
            if direction == 'left':
                target = STATE_AVOID_LEFT
            elif direction == 'right':
                target = STATE_AVOID_RIGHT
            self._enter(target)
            # 不 return，让后续电机指令段为新状态发布 Twist

        elif self._state in (STATE_AVOID_LEFT, STATE_AVOID_RIGHT, STATE_AVOID_DEFAULT):
            if self._clear_stable():
                self._enter(STATE_FORWARD)
            elif elapsed >= self._avoid_sec:
                self._retry_count += 1
                if self._retry_count <= self._max_retries:
                    self.get_logger().info(f'Avoid timeout, retry {self._retry_count}/{self._max_retries}')
                    self._enter(STATE_CAPTURE)
                else:
                    self.get_logger().info(f'Avoid exhausted, using rotation escape')
                    self._enter(STATE_ROTATE)

        elif self._state == STATE_ROTATE:
            if elapsed >= self._rotate_sec:
                self._enter(STATE_FORWARD)

        # ── 执行运动 ──

        twist = Twist()

        if self._state == STATE_STOPPED:
            pass  # 全零

        elif self._state == STATE_FORWARD:
            twist.linear.x = self._fwd_speed

        elif self._state == STATE_BACK_UP:
            twist.linear.x = -self._back_speed

        elif self._state == STATE_ROTATE:
            twist.angular.z = self._turn_speed

        elif self._state == STATE_AVOID_LEFT:
            twist.linear.y = -self._strafe_speed

        elif self._state == STATE_AVOID_RIGHT:
            twist.linear.y = self._strafe_speed

        elif self._state == STATE_AVOID_DEFAULT:
            if self._default_dir == 'left':
                twist.linear.y = -self._strafe_speed
            else:
                twist.linear.y = self._strafe_speed

        self._publish(twist)

    def destroy_node(self):
        self.get_logger().info('Shutting down camera avoidance...')
        try:
            self._publish(Twist())
        except Exception:
            pass
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
