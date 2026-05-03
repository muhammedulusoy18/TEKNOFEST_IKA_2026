import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        self.get_logger().info('🧠 Beyin Node: Karar Mekanizması Tam Kapasite Aktif.')

        # 1. Yayıncılar: Motorlara nihai komutu basar
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 2. Abonelikler: Gözlerden ve Sensörlerden veri alır
        self.create_subscription(Twist, 'autonomy_cmd', self.autonomy_callback, 10)
        self.create_subscription(Float32MultiArray, 'imu_data', self.safety_callback, 10)
        self.create_subscription(String, 'system_status', self.emergency_callback, 10)

        self.failsafe_active = False

    def autonomy_callback(self, msg):
        """Gözlerden gelen otonom sürüş komutunu motorlara iletir."""
        if not self.failsafe_active:
            # Gözlerden gelen veriyi olduğu gibi (veya süzerek) motor düğümüne paslar
            self.cmd_pub.publish(msg)

    def safety_callback(self, msg):
        """Eğim verilerini kontrol eder, 35 dereceden sonra hızı keser."""
        roll, pitch = msg.data
        if abs(roll) > 35 or abs(pitch) > 35:
            self.get_logger().warn(f"⚠️ Yüksek Eğim: R:{roll:.1f} P:{pitch:.1f} - Hız kısıtlanıyor!")
            # Burada hızı limitleyen bir mantık eklenebilir

    def emergency_callback(self, msg):
        """Kritik bir hata (E-STOP veya Takla) gelirse her şeyi durdurur."""
        if "CRITICAL" in msg.data:
            self.failsafe_active = True
            stop_msg = Twist() # Tüm değerler 0.0
            self.cmd_pub.publish(stop_msg)
            self.get_logger().error("🛑 ACİL DURUM: Tüm motorlar kilitlendi!")

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Beyin kapatılıyor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()