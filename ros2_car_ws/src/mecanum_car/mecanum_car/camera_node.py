"""
树莓派 USB 相机发布节点

捕获 USB 摄像头帧 → 发布 sensor_msgs/Image 到 /camera/image_raw
配合 image_transport 自动提供 /camera/image_raw/compressed

运行在: Raspberry Pi (下位机)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


class CameraNode(Node):
    """USB 相机 ROS2 发布节点"""

    def __init__(self):
        super().__init__('camera_node')

        # 参数
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_link')

        self._device_id = self.get_parameter('device_id').value
        self._width = self.get_parameter('width').value
        self._height = self.get_parameter('height').value
        self._fps = self.get_parameter('fps').value
        self._frame_id = self.get_parameter('frame_id').value

        if not HAS_CV2:
            self.get_logger().fatal('OpenCV (cv2) not available — cannot run camera_node')
            raise RuntimeError('cv2 not installed')

        # 打开摄像头
        self._cap = cv2.VideoCapture(self._device_id)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS, self._fps)

        if not self._cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {self._device_id}')
            self._camera_ok = False
        else:
            self._camera_ok = True
            actual_w = self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_h = self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(
                f'Camera ready: device={self._device_id}, '
                f'{actual_w:.0f}x{actual_h:.0f} @ {actual_fps:.0f}fps'
            )

        # CvBridge + 发布
        self._br = CvBridge()
        self._pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self._timer = self.create_timer(1.0 / self._fps, self._timer_cb)

        # 重连计数器
        self._reconnect_attempts = 0
        self._max_reconnect_attempts = 10

        self.get_logger().info(
            f'camera_node started, publishing {self._width}x{self._height} @ {self._fps}Hz'
        )

    def _timer_cb(self):
        if not self._camera_ok:
            # 尝试重连
            self._reconnect_attempts += 1
            if self._reconnect_attempts <= self._max_reconnect_attempts:
                self.get_logger().warn(
                    f'Camera not available, retry {self._reconnect_attempts}/{self._max_reconnect_attempts}'
                )
                self._cap.open(self._device_id)
                if self._cap.isOpened():
                    self._camera_ok = True
                    self._reconnect_attempts = 0
                    self.get_logger().info('Camera reconnected')
            return

        ret, frame = self._cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Frame capture failed', throttle_duration_sec=5.0)
            return

        try:
            msg = self._br.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            self._pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'cv2_to_imgmsg failed: {e}', throttle_duration_sec=5.0)

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        self.get_logger().info('Camera released')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
