import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

# motor ve güvenlik katmanını çekiyoruz
import teknofest_ika.modules.pid_thread as pid_ctrl


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.get_logger().info('⚙️ Motor Node: Donanım ve PID Kalkanları Aktif Ediliyor...')

        # 1. Arka planda PID ve Güvenlik (Failsafe) döngülerini başlat
        self.pid_thread = threading.Thread(target=pid_ctrl.main, daemon=True)
        self.safety_thread = threading.Thread(target=pid_ctrl.safety_thread, daemon=True)

        self.pid_thread.start()
        self.safety_thread.start()

        # 2. Beyinden gelen hız ve yön komutlarına abone ol
        # /cmd_vel üzerinden gelen Twist mesajlarını dinler
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.get_logger().info('✅ /cmd_vel dinleniyor. Robot hareket emrine hazır.')

    def cmd_vel_callback(self, msg):
        throttle = msg.linear.x
        steer = msg.angular.z

        # 10kW motorların sağlığı için kısıtlamalar ve matematiksel dağılım
        # Sol ve sağ motor setpoint'lerini (SP) güncelle
        hedef_sol = throttle + steer
        hedef_sag = throttle - steer

        # PID katmanındaki değişkenleri doğrudan güncelle
        # Maksimum hızı (PWM) 70.0 ile sınırlandırıyoruz (Güvenlik gereği)
        pid_ctrl.sp_l = max(-70.0, min(70.0, hedef_sol))
        pid_ctrl.sp_r = max(-70.0, min(70.0, hedef_sag))

        # Kalp atışı (Heartbeat) güncellemesi: İletişim koparsa sistem durur
        pid_ctrl.last_heartbeat = time.time()

    def stop_motors(self):
        """Acil durumda veya kapanışta motorları tamamen durdurur."""
        pid_ctrl.sp_l = 0.0
        pid_ctrl.sp_r = 0.0
        pid_ctrl.emergency_stop.set()
        self.get_logger().warn('🛑 Motorlar Güvenle Durduruldu.')


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('⚠️ Kullanıcı komutuyla sistem kapatılıyor...')
    finally:
        # Donanımı ve thread'leri kapat
        pid_ctrl.system_running = False
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()