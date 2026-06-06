import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import json, numpy as np, cv2

from PyQt5.QtCore import QObject, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap


class GCSRosNode(Node):
    def __init__(self, worker):
        super().__init__('gcs_ros_node')
        self._w = worker
        self.cmd_pub  = self.create_publisher(Twist,  '/cmd_vel',    10)
        self.mode_pub = self.create_publisher(String, '/drive_mode', 10)
        self.create_subscription(String, '/telemetry',      self._on_tel, 10)
        self.create_subscription(Image,  '/processed_frame', self._on_img, 10)
        self.get_logger().info("GCS ROS 2 Düğümü Başlatıldı.")

    def _on_tel(self, msg):
        try:
            self._w.sig_telemetry.emit(json.loads(msg.data))
        except Exception:
            pass

    def _on_img(self, msg):
        try:
            if msg.encoding == 'bgr8':
                arr = np.frombuffer(msg.data, np.uint8).reshape((msg.height, msg.width, 3))
                rgb = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
            elif msg.encoding == 'rgb8':
                rgb = np.frombuffer(msg.data, np.uint8).reshape((msg.height, msg.width, 3))
            else:
                return
            h, w, ch = rgb.shape
            px = QPixmap.fromImage(QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888))
            self._w.sig_image.emit(px)
        except Exception:
            pass

    def send_twist(self, lx: float, az: float):
        msg = Twist()
        msg.linear.x  = float(lx)
        msg.angular.z = float(az)
        self.cmd_pub.publish(msg)

    def send_drive_mode(self, mode: str):
        """brain_node'a MANUAL / AUTONOMOUS mod mesajı gönder."""
        msg = String()
        msg.data = mode.upper()
        self.mode_pub.publish(msg)


class GCSROSWorker(QObject):
    """
    ROS 2 node'unu ana thread'de QTimer ile çalıştırır.
    QThread yerine QObject kullanılır — C++ terminate crash olmaz.
    """
    sig_telemetry = pyqtSignal(dict)
    sig_image     = pyqtSignal(QPixmap)
    sig_status    = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.node     = None
        self.executor = None
        self._timer   = QTimer(self)
        self._timer.setInterval(50)        # 20 Hz spin
        self._timer.timeout.connect(self._spin_once)

    # ── QThread compat shims ─────────────────────────
    def isRunning(self) -> bool:
        return self._timer.isActive()

    def start(self):
        if self._timer.isActive():
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node     = GCSRosNode(self)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self._timer.start()
            self.sig_status.emit("CONNECTED")
        except Exception as e:
            self.sig_status.emit(f"ERROR: {e}")

    def stop(self):
        self._timer.stop()
        self._shutdown()

    def wait(self, *args, **kwargs):
        pass    # QObject versiyonunda beklemeye gerek yok

    # ── Internal ─────────────────────────────────────
    def _spin_once(self):
        try:
            if self.executor and rclpy.ok():
                self.executor.spin_once(timeout_sec=0)
        except Exception:
            pass

    def _shutdown(self):
        try:
            if self.executor:
                self.executor.shutdown(timeout_sec=0)
                self.executor = None
            if self.node:
                self.node.destroy_node()
                self.node = None
        except Exception:
            pass
        self.sig_status.emit("DISCONNECTED")

    # ── Komutlar ─────────────────────────────────────
    def send_move(self, throttle: float, steering: float):
        if self.node:
            self.node.send_twist(throttle, steering)

    def send_estop(self):
        if self.node:
            self.node.send_twist(0.0, 0.0)

    def send_drive_mode(self, mode: str):
        """GCS'den brain_node'a mod değişikliği gönder."""
        if self.node:
            self.node.send_drive_mode(mode)

    def cleanup(self):
        self.stop()
