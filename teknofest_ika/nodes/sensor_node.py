import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import teknofest_ika.modules.serial_comms as serial_tool


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.get_logger().info('🧭 Sensor Node başlatıldı. IMU ve UART aktif.')

        # 1. Donanım Kurulumu
        try:
            self.imu = serial_tool.IMU()
            self.uart = serial_tool.UART()
        except Exception as e:
            self.get_logger().error(f'Donanım başlatılamadı: {e}')

        # 2. Yayıncılar (Publishers)
        # Eğim verilerini (Roll, Pitch) yayınlar
        self.imu_publisher = self.create_publisher(Float32MultiArray, 'imu_data', 10)
        # Sistem durumunu (Hata, Uyarı vb.) yayınlar
        self.status_publisher = self.create_publisher(String, 'system_status', 10)

        # 3. Sensör Okuma Döngüsü (50Hz - Zeynep'in orijinal ayarı)
        self.timer = self.create_timer(0.02, self.sensor_loop)

    def sensor_loop(self):
        try:
            # IMU verilerini oku
            roll, pitch = self.imu.get_angles(0.02)

            # Veriyi ROS ağına bas
            imu_msg = Float32MultiArray()
            imu_msg.data = [float(roll), float(pitch)]
            self.imu_publisher.publish(imu_msg)

            # Güvenlik Kontrolü (Zeynep'in Anti-Rollover mantığı)
            if abs(roll) > 45 or abs(pitch) > 45:
                status_msg = String()
                status_msg.data = "CRITICAL_ANGLE"
                self.status_publisher.publish(status_msg)
                self.get_logger().warn('⚠️ KRİTİK EĞİM ALGILANDI!')

        except Exception as e:
            self.get_logger().error(f'Sensör döngü hatası: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Sensörler kapatılıyor...')
    finally:
        node.uart.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()