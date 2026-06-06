import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        self.get_logger().info('🧠 Beyin Node: Karar Mekanizması Tam Kapasite Aktif.')

        # Mod: MANUAL (varsayılan) veya AUTONOMOUS
        self.mode = "MANUAL"
        self.failsafe_active = False

        # 1. Yayıncılar: Motorlara nihai komutu basar
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 2. Abonelikler
        self.create_subscription(Twist,            'autonomy_cmd',  self.autonomy_callback,  10)
        self.create_subscription(Float32MultiArray, 'imu_data',      self.safety_callback,    10)
        self.create_subscription(String,            'system_status', self.emergency_callback, 10)
        self.create_subscription(String,            'drive_mode',    self.mode_callback,      10)

        self.get_logger().info(f'🕹️ Başlangıç modu: {self.mode}')

    def mode_callback(self, msg: String):
        """GCS'den gelen mod değişikliği (MANUAL / AUTONOMOUS)."""
        new_mode = msg.data.strip().upper()
        if new_mode in ("MANUAL", "AUTONOMOUS") and new_mode != self.mode:
            self.mode = new_mode
            self.get_logger().info(f'🔄 Mod değişti: {self.mode}')

    def autonomy_callback(self, msg: Twist):
        """Otonom mod aktifse perception komutunu motorlara iletir."""
        if self.mode == "AUTONOMOUS" and not self.failsafe_active:
            self.cmd_pub.publish(msg)

    def safety_callback(self, msg: Float32MultiArray):
        """Eğim verilerini kontrol eder, 35 dereceden sonra uyarır."""
        roll, pitch = msg.data
        if abs(roll) > 35 or abs(pitch) > 35:
            self.get_logger().warn(f'⚠️ Yüksek Eğim: R:{roll:.1f} P:{pitch:.1f}')

    def emergency_callback(self, msg: String):
        """Kritik hata gelirse tüm motorları durdurur."""
        if "CRITICAL" in msg.data:
            self.failsafe_active = True
            self.cmd_pub.publish(Twist())  # sıfır komut
            self.get_logger().error('🛑 ACİL DURUM: Tüm motorlar kilitlendi!')

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Beyin kapatılıyor...')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()