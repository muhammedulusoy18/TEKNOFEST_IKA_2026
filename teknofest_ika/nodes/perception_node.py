import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import teknofest_ika.modules.perception as p_unit
import teknofest_ika.utils.camera_handler as c_handler

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.get_logger().info('👁️ Perception Node: Gözler ve Analiz Aktif.')

        self.camera = c_handler.CameraHandler()
        self.perceptor = p_unit.PerceptionUnit()
        self.bridge = CvBridge()
        self.state = "DRIVING"

        # Yayıncılar
        self.cmd_pub = self.create_publisher(Twist, 'autonomy_cmd', 10)
        self.img_pub = self.create_publisher(Image, 'processed_frame', 10)

        self.timer = self.create_timer(0.04, self.process_frame) # ~25 FPS

    def process_frame(self):
        frame = self.camera.get_frame()
        if frame is None: return

        # OpenCV Analizi
        decision, out_frame, next_state = self.perceptor.process_frame(frame, self.state)
        self.state = next_state

        # 1. Kararı Twist Mesajına Dönüştür ve Beyne Gönder
        cmd = Twist()
        cmd.linear.x = float(decision.get('throttle', 0.0))
        cmd.angular.z = float(decision.get('steer', 0.0))
        self.cmd_pub.publish(cmd)

        # 2. Görüntüyü ROS formatına çevir ve GUI'ye gönder
        try:
            ros_img = self.bridge.cv2_to_imgmsg(out_frame, "bgr8")
            self.img_pub.publish(ros_img)
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.camera.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()