import time
import math
from enum import Enum
from unittest.mock import MagicMock

# --- DONANIM TAKLİT SİSTEMİ (MAGICMOCK) ---
try:
    import smbus2
    import serial
    HARDWARE_MODE = True
except ImportError:
    print("UYARI: smbus2 veya pyserial bulunamadı! MagicMock Simülasyonu aktif.")
    HARDWARE_MODE = False
    
    # Donanım kütüphanelerini MagicMock ile simüle ediyoruz
    smbus2 = MagicMock()
    serial = MagicMock()
    
    # SMBus(1) çağrıldığında hata vermemesi için bir nesne döndürmesini sağlıyoruz
    smbus2.SMBus.return_value = MagicMock()
    # serial.Serial çağrıldığında in_waiting gibi niteliklere sahip bir mock döndürür
    serial.Serial.return_value = MagicMock(in_waiting=0)

# --- CRC8 CALCULATION ---
def calculate_crc8(data: str):
    crc = 0x00
    for byte in data.encode():
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

# --- SYSTEM STATES ---
class SystemState(Enum):
    INIT = 0
    RUNNING = 1
    WARNING = 2
    FAILSAFE = 3
    STOPPED = 4

# --- IMU CLASS (MPU6050) ---
class IMU:
    def __init__(self, bus_id=1, address=0x68, alpha=0.98):
        self.address = address
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self.gx_offset = 0
        self.gy_offset = 0
        self.ax_filt, self.ay_filt, self.az_filt = 0, 0, 0

        try:
            self.bus = smbus2.SMBus(bus_id)
            if HARDWARE_MODE:
                self.bus.write_byte_data(self.address, 0x6B, 0x00)
            print("IMU Calibration starting... Please do not move the robot.")
            self.gyro_calibration()
        except Exception as e:
            if HARDWARE_MODE:
                raise RuntimeError(f"IMU Connection Error: {e}")

    def read_word(self, reg):
        if not HARDWARE_MODE: return 0
        try:
            high = self.bus.read_byte_data(self.address, reg)
            low = self.bus.read_byte_data(self.address, reg + 1)
            value = (high << 8) | low
            return value - 65536 if value >= 0x8000 else value
        except:
            return 0

    def gyro_calibration(self, samples=100):
        if not HARDWARE_MODE:
            print("Calibration Skipped (Simulation Mode).")
            return
        gx_t, gy_t = 0, 0
        for _ in range(samples):
            gx_t += self.read_word(0x43)
            gy_t += self.read_word(0x45)
            time.sleep(0.005)
        self.gx_offset = gx_t / samples
        self.gy_offset = gy_t / samples
        print(f"Calibration Complete. Offsets: GX:{self.gx_offset:.2f} GY:{self.gy_offset:.2f}")

    def get_angles(self, dt):
        dt = min(dt, 0.1)
        raw_ax = self.read_word(0x3B)
        raw_ay = self.read_word(0x3D)
        raw_az = self.read_word(0x3F)

        # Filtreleme
        self.ax_filt = (raw_ax * 0.1) + (self.ax_filt * 0.9)
        self.ay_filt = (raw_ay * 0.1) + (self.ay_filt * 0.9)
        self.az_filt = (raw_az * 0.1) + (self.az_filt * 0.9)

        ax, ay, az = self.ax_filt / 16384.0, self.ay_filt / 16384.0, self.az_filt / 16384.0
        gx = (self.read_word(0x43) - self.gx_offset) / 131.0
        gy = (self.read_word(0x45) - self.gy_offset) / 131.0

        if not HARDWARE_MODE:
            accel_roll, accel_pitch = 0.0, 0.0
        else:
            accel_roll = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))
            accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        self.roll = self.alpha * (self.roll + gx * dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * accel_pitch
        return self.roll, self.pitch

# --- UART COMMUNICATION ---
class UART:
    def __init__(self, port="/dev/ttyS3", baud=115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01, write_timeout=0.01)
            if HARDWARE_MODE:
                self.ser.reset_input_buffer()
        except Exception as e:
            print(f"UART Init Error: {e}")

    def send(self, message):
        try:
            crc = calculate_crc8(message)
            packet = f"<{message}|{crc:02X}>\n"
            if HARDWARE_MODE:
                self.ser.write(packet.encode())
        except: pass

    def receive(self):
        if HARDWARE_MODE and self.ser.in_waiting > 0:
            try:
                lines = self.ser.readlines()
                if not lines: return None
                line = lines[-1].decode(errors="ignore").strip()
                if line.startswith("<") and "|" in line and line.endswith(">"):
                    content = line[1:-1]
                    msg, crc_rx = content.rsplit("|", 1)
                    if f"{calculate_crc8(msg):02X}" == crc_rx.upper():
                        return msg
            except: pass
        return None

    def close(self):
        if HARDWARE_MODE:
            self.ser.close()

# --- MAIN LOOP ---
def main():
    state = SystemState.INIT
    last_state = None
    period = 1.0 / 50
    last_comm_time = time.perf_counter()
    start_time = time.perf_counter()

    try:
        imu = IMU()
        uart = UART()
    except Exception as e:
        print(f"Critical Init Error: {e}")
        return

    last_time = time.perf_counter()
    print("Helmet Control System Ready...")

    try:
        while True:
            loop_start = time.perf_counter()
            dt = loop_start - last_time
            last_time = loop_start

            try:
                roll, pitch = imu.get_angles(dt)
                cmd = uart.receive()
                if cmd:
                    last_comm_time = time.perf_counter()
                    if cmd == "STOP": state = SystemState.STOPPED

                # Güvenlik Kontrolleri
                if abs(roll) > 45 or abs(pitch) > 45:
                    state = SystemState.FAILSAFE

                # İletişim Kaybı (3sn sonra kontrol başlar)
                if (time.perf_counter() - start_time) > 3.0:
                    if (time.perf_counter() - last_comm_time) > 1.0:
                        if state not in (SystemState.STOPPED, SystemState.FAILSAFE):
                            print("Communication Lost!")
                            state = SystemState.FAILSAFE

                # Durum Yönetimi
                if state == SystemState.INIT: state = SystemState.RUNNING
                elif state == SystemState.RUNNING:
                    if abs(roll) > 30 or abs(pitch) > 30: state = SystemState.WARNING
                elif state == SystemState.WARNING:
                    if abs(roll) < 25 and abs(pitch) < 25: state = SystemState.RUNNING

                # Komut Gönderimi
                if state != last_state:
                    if state in (SystemState.FAILSAFE, SystemState.STOPPED):
                        uart.send("MOTOR_KAPAT")
                    elif state == SystemState.WARNING:
                        uart.send("YAVASLA")
                    last_state = state

                telemetry = f"R:{roll:.1f};P:{pitch:.1f};S:{state.name}"
                uart.send(telemetry)

            except Exception as e:
                print(f"Loop Error: {e}")
                uart.send("MOTOR_KAPAT")
                state = SystemState.FAILSAFE

            wait = period - (time.perf_counter() - loop_start)
            if wait > 0: time.sleep(wait)

    except KeyboardInterrupt:
        print("\nSystem shut down by user.")
    finally:
        uart.send("MOTOR_KAPAT")
        uart.close()

if __name__ == "__main__":
    main()