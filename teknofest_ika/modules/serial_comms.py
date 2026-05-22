import asyncio
import time
import math
from enum import Enum
from unittest.mock import MagicMock

DEBUG = False

# --- DONANIM KONTROLÜ ---
try:
    import smbus2
    import serial
    HARDWARE_MODE = True
except ImportError:
    print("UYARI: Donanım kütüphaneleri eksik! Simülasyon modu aktif.")
    HARDWARE_MODE = False
    smbus2 = MagicMock()
    serial = MagicMock()
    smbus2.SMBus.return_value = MagicMock()
    serial.Serial.return_value = MagicMock(in_waiting=0)

# --- CRC8 HESAPLAMA ---
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

class SystemState(Enum):
    INIT = 0
    RUNNING = 1
    WARNING = 2
    FAILSAFE = 3
    STOPPED = 4

# --- ASENKRON IMU SINIFI ---
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
        except Exception as e:
            if HARDWARE_MODE: print(f"IMU Hatası: {e}")

    async def calibrate(self, samples=100):
        """Asenkron kalibrasyon: Diğer görevleri bloklamaz."""
        if not HARDWARE_MODE: return
        print("IMU Kalibrasyonu başlıyor... Robotu oynatmayın.")
        gx_t, gy_t = 0, 0
        for _ in range(samples):
            gx_t += self.read_word(0x43)
            gy_t += self.read_word(0x45)
            await asyncio.sleep(0.005) 
        self.gx_offset = gx_t / samples
        self.gy_offset = gy_t / samples
        print(f"Kalibrasyon Tamam. Ofsetler: GX:{self.gx_offset:.2f}")

    def read_word(self, reg):
        if not HARDWARE_MODE: return 0
        try:
            high = self.bus.read_byte_data(self.address, reg)
            low = self.bus.read_byte_data(self.address, reg + 1)
            val = (high << 8) | low
            return val - 65536 if val >= 0x8000 else val
        except: return 0

    def update_angles(self, dt):
        """Açıları günceller."""
        raw_ax = self.read_word(0x3B)
        raw_ay = self.read_word(0x3D)
        raw_az = self.read_word(0x3F)

        # Basit LPF (Düşük Geçiren Filtre)
        self.ax_filt = (raw_ax * 0.1) + (self.ax_filt * 0.9)
        self.ay_filt = (raw_ay * 0.1) + (self.ay_filt * 0.9)
        self.az_filt = (raw_az * 0.1) + (self.az_filt * 0.9)

        ax, ay, az = self.ax_filt / 16384.0, self.ay_filt / 16384.0, self.az_filt / 16384.0
        gx = (self.read_word(0x43) - self.gx_offset) / 131.0
        gy = (self.read_word(0x45) - self.gy_offset) / 131.0

        accel_roll = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az))) if HARDWARE_MODE else 0
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az))) if HARDWARE_MODE else 0

        self.roll = self.alpha * (self.roll + gx * dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * accel_pitch
        return self.roll, self.pitch

# --- ASENKRON UART SINIFI ---
class UART:
    def __init__(self, port="/dev/ttyS3", baud=115200):
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0)
        except Exception as e:
            print(f"UART Hata: {e}")

    def send(self, message):
        if not self.ser: return
        try:
            packet = f"<{message}|{calculate_crc8(message):02X}>\n"
            if HARDWARE_MODE: self.ser.write(packet.encode())
        except: pass

    async def receive(self):
        """Asenkron veri okuma: Veri yoksa beklemez, geçer."""
        if HARDWARE_MODE and self.ser and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line.startswith("<") and "|" in line:
                    content = line[1:-1]
                    msg, crc_rx = content.rsplit("|", 1)
                    if f"{calculate_crc8(msg):02X}" == crc_rx.upper():
                        return msg
            except: pass
        return None
    
class HelmetSystem:
    def __init__(self):
        self.state = SystemState.INIT
        self.imu = IMU()
        self.uart = UART()
        self.last_comm_time = time.perf_counter()
        self.running = True

    async def run(self):
        # 1. Adım: Kalibrasyon (Diğer işleri engellemez)
        await self.imu.calibrate()
        self.state = SystemState.RUNNING
        last_time = time.perf_counter()

        while self.running:
            loop_start = time.perf_counter()
            dt = loop_start - last_time
            last_time = loop_start

            # --- 1. SENSÖR VE HABERLEŞME ---
            roll, pitch = self.imu.update_angles(dt)
            cmd = await self.uart.receive()

            if DEBUG:
                print(f"DEBUG: Durum={self.state.name} | Roll={roll:.1f} | Pitch={pitch:.1f}")

            # --- 2. DURUM MANTIĞI ---
            if cmd:
                self.last_comm_time = time.perf_counter()
                if cmd == "STOP":
                    self.state = SystemState.STOPPED
                elif cmd == "RESUME" and self.state == SystemState.STOPPED:
                    self.state = SystemState.RUNNING

            # FAILSAFE'den çıkış: açı normale dönerse ve RESUME komutu gelirse
            if self.state == SystemState.FAILSAFE:
                if abs(roll) < 30 and abs(pitch) < 30 and cmd == "RESUME":
                    self.state = SystemState.RUNNING

            # Güvenlik Sınırı (45 Derece) - kademeli uyarı sistemi
            if abs(roll) > 45 or abs(pitch) > 45:
                self.state = SystemState.FAILSAFE
            elif abs(roll) > 35 or abs(pitch) > 35:
                if self.state == SystemState.RUNNING:
                    self.state = SystemState.WARNING
            else:
                if self.state == SystemState.WARNING:
                    self.state = SystemState.RUNNING

            # İletişim Kaybı Kontrolü (1 saniye)
            if (time.perf_counter() - self.last_comm_time) > 1.0:
                if self.state not in (SystemState.STOPPED, SystemState.FAILSAFE):
                    self.state = SystemState.FAILSAFE

            # --- 3. ÇIKTI ÜRETME ---
            if self.state in (SystemState.FAILSAFE, SystemState.STOPPED):
                self.uart.send("MOTOR_KAPAT")
            
            telemetry = f"R:{roll:.1f};P:{pitch:.1f};S:{self.state.name}"
            self.uart.send(telemetry)

            # --- 4. ASENKRON BEKLEME ---
            # 50Hz (0.02s) döngü hızı sağlar, CPU'yu yormaz ve paralel görevlere izin verir
            await asyncio.sleep(0.02)

async def main():
    system = HelmetSystem()
    try:
        await system.run()
    except KeyboardInterrupt:
        print("\nKullanıcı durdurdu.")
    finally:
        system.uart.send("MOTOR_KAPAT")

if __name__ == "__main__":
    asyncio.run(main())