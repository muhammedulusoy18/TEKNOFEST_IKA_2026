import asyncio
import os
from unittest.mock import MagicMock

# ============================================================
# DONANIM SOYUTLAMA
# Khadas VIM3: gpiochip1 kullanır (NOT gpiochip0!)
# GPIO library: python3-gpiod (v1 veya v2 API)
# VIM3 /boot/env.txt: overlays=pwm_ao_a pwm_f i2c3 uart3
# ============================================================
try:
    import gpiod
    # gpiod v2 API kontrolü
    _GPIOD_V2 = hasattr(gpiod, 'request_lines')
    GPIOD_AVAILABLE = True
except ImportError:
    gpiod = MagicMock()
    gpiod.LINE_REQ_DIR_OUT = 1
    gpiod.LINE_REQ_DIR_IN = 2
    GPIOD_AVAILABLE = False
    _GPIOD_V2 = False

# ============================================================
# KHADAS VIM3 — PIN ve PWM REFERANSI
# ============================================================
# PWM:
#   Sol Motor  → /sys/class/pwm/pwmchip0/pwm0 (pwm_ao_a overlay)
#   Sağ Motor  → /sys/class/pwm/pwmchip4/pwm0 (pwm_f overlay)
#
# GPIO (40-pin Header):
#   Önerilen yön pinleri (VIM3 GPIOH_6, GPIOH_7 vb.):
#   VIM3 Pinout: https://docs.khadas.com/products/sbc/vim3/hardware/gpio
#   Sol İleri  → GPIO pin 35 (GPIOH_6)
#   Sol Geri   → GPIO pin 37 (GPIOH_7)
#   Sağ İleri  → GPIO pin 15 (GPIOAO_5)
#   Sağ Geri   → GPIO pin 16 (GPIOAO_6)
#
# NOT: gpiochip0 = Raspberry Pi, gpiochip1 = Khadas VIM3!
# ============================================================

# VIM3'de PWM chip numaraları:
VIM3_PWM_LEFT_CHIP  = 0   # /sys/class/pwm/pwmchip0 (pwm_ao_a)
VIM3_PWM_RIGHT_CHIP = 4   # /sys/class/pwm/pwmchip4 (pwm_f)
VIM3_GPIO_CHIP      = "gpiochip1"  # VIM3'e özgü

# VIM3 GPIO Pin numaraları (40-pin header sırası değil, kernel GPIO numarası)
VIM3_LEFT_FWD_GPIO  = 228   # GPIOH_6  → Header Pin 35
VIM3_LEFT_BWD_GPIO  = 229   # GPIOH_7  → Header Pin 37
VIM3_RIGHT_FWD_GPIO = 501   # GPIOAO_5 → Header Pin 15
VIM3_RIGHT_BWD_GPIO = 502   # GPIOAO_6 → Header Pin 16


class MotorDriver:
    """
    Tek motor için GPIO yön + PWM hız kontrolü.
    Khadas VIM3 ile hem gerçek donanımda hem simülasyonda çalışır.
    """
    def __init__(self, forward_pin, backward_pin, pwm_chip, name="motor"):
        self.name = name
        self.period = 50000        # 50ms → 20kHz PWM frekansı
        self.active_speed = 0
        self.last_speed = None

        # Gerçek donanım mı, simülasyon mu?
        pwm_chip_path = f"/sys/class/pwm/pwmchip{pwm_chip}"
        self.simulated = not os.path.exists(pwm_chip_path)
        self.pwm_path = f"{pwm_chip_path}/pwm0"

        if self.simulated:
            print(f"[MOTOR-{name}] Simülasyon modu aktif (pwmchip{pwm_chip} bulunamadı).")

        # GPIO başlatma
        self.forward = MagicMock()
        self.backward = MagicMock()
        self._init_gpio(forward_pin, backward_pin)

        # PWM başlatma
        self._setup_pwm(pwm_chip)

    def _init_gpio(self, forward_pin, backward_pin):
        """gpiod v1/v2 API farkını halleder."""
        if not GPIOD_AVAILABLE:
            return

        try:
            if _GPIOD_V2:
                # gpiod v2 API (Ubuntu 24.04 varsayılanı)
                chip = gpiod.Chip(VIM3_GPIO_CHIP)
                request = gpiod.request_lines(
                    VIM3_GPIO_CHIP,
                    consumer=self.name,
                    config={
                        (forward_pin, backward_pin): gpiod.LineSettings(
                            direction=gpiod.line.Direction.OUTPUT,
                            output_value=gpiod.line.Value.INACTIVE
                        )
                    }
                )
                self._gpio_request = request
                self._fwd_pin  = forward_pin
                self._bwd_pin  = backward_pin
                self._use_v2   = True
            else:
                # gpiod v1 API
                chip = gpiod.Chip(VIM3_GPIO_CHIP)
                self.forward  = chip.get_line(forward_pin)
                self.backward = chip.get_line(backward_pin)
                self.forward.request(consumer=self.name, type=gpiod.LINE_REQ_DIR_OUT)
                self.backward.request(consumer=self.name, type=gpiod.LINE_REQ_DIR_OUT)
                self._use_v2 = False
        except Exception as e:
            if not self.simulated:
                print(f"[GPIO HATA] {self.name}: {e}")
                print(f"[GPIO İPUCU] VIM3'de /boot/env.txt'e 'overlays=uart3 pwm_ao_a pwm_f i2c3' ekleyin.")
            self._use_v2 = False

    def _gpio_write(self, fwd_val: int, bwd_val: int):
        """GPIO pinlerine değer yaz (v1/v2 uyumlu)."""
        try:
            if hasattr(self, '_use_v2') and self._use_v2:
                self._gpio_request.set_values({
                    self._fwd_pin: gpiod.line.Value.ACTIVE if fwd_val else gpiod.line.Value.INACTIVE,
                    self._bwd_pin: gpiod.line.Value.ACTIVE if bwd_val else gpiod.line.Value.INACTIVE,
                })
            else:
                self.forward.set_value(fwd_val)
                self.backward.set_value(bwd_val)
        except Exception as e:
            if not self.simulated:
                print(f"[GPIO YAZMA HATA] {self.name}: {e}")

    def _setup_pwm(self, pwm_chip):
        """PWM kanalını dışa aç ve başlangıç değerlerini yaz."""
        pwm_chip_path = f"/sys/class/pwm/pwmchip{pwm_chip}"
        if not self.simulated and not os.path.exists(self.pwm_path):
            try:
                with open(f"{pwm_chip_path}/export", "w") as f:
                    f.write("0")
            except Exception as e:
                print(f"[PWM EXPORT HATA] {self.name}: {e}")

        self._write_pwm("enable", 0)
        self._write_pwm("period", self.period)
        self._write_pwm("duty_cycle", 0)
        self._write_pwm("enable", 1)

    def _write_pwm(self, attribute, value):
        """PWM sysfs dosyasına değer yaz (simülasyonda atla)."""
        if self.simulated:
            return
        try:
            with open(f"{self.pwm_path}/{attribute}", "w") as f:
                f.write(str(value))
        except Exception as e:
            print(f"[PWM YAZMA HATA] {self.name}/{attribute}: {e}")

    def set_speed(self, speed: float):
        """
        Motoru verilen hızda çalıştır.
        speed > 0: İleri, speed < 0: Geri, speed == 0: Dur
        Aralık: -100 ile 100 (PWM %)
        """
        if speed == 0:
            self._gpio_write(0, 0)
            self._write_pwm("duty_cycle", 0)
            self.last_speed = 0
            return

        # Ölü bölge: çok düşük PWM motorları kıpırdatmaz, akım çeker
        if 0 < abs(speed) < 15:
            speed = 15 if speed > 0 else -15

        if speed == self.last_speed:
            return
        self.last_speed = speed

        # Önce her iki yön pimini sıfırla (kısa devre önlemi)
        self._gpio_write(0, 0)

        if speed > 0:
            self._gpio_write(1, 0)
        else:
            self._gpio_write(0, 1)

        duty = int(self.period * abs(speed) / 100.0)
        self._write_pwm("duty_cycle", duty)

    def cleanup(self):
        """Donanımı güvenle kapat."""
        self._write_pwm("enable", 0)
        try:
            if hasattr(self, '_use_v2') and self._use_v2:
                self._gpio_request.release()
            else:
                self.forward.release()
                self.backward.release()
        except Exception:
            pass


class SixWheelRobot:
    """
    Diferansiyel sürüşlü 6 tekerlekli robot için yüksek seviye kontrol.
    Sol ve sağ motor grupları aynı sinyalle sürülür.
    """
    def __init__(self, left: MotorDriver, right: MotorDriver):
        self.left = left
        self.right = right
        self.task = None
        self.emergency = False

    async def hard_stop(self):
        """Yazılımsal hata veya limit aşımında sistemi kilitler."""
        if self.task and not self.task.done():
            self.task.cancel()
        self.left.active_speed = 0
        self.right.active_speed = 0
        self.left.set_speed(0)
        self.right.set_speed(0)
        self.emergency = True
        print("\n[!!!] YAZILIMSAL KRİTİK DURUŞ (SİSTEM KİLİTLENDİ)")

    async def stop(self):
        """Yumuşak operasyonel duruş."""
        print("\n[DUR] - Görev tamamlandı.")
        if self.task and not self.task.done():
            self.task.cancel()
        self.left.active_speed = 0
        self.right.active_speed = 0
        self.left.set_speed(0)
        self.right.set_speed(0)

    async def _move_task(self, left_target: float, right_target: float, step: float = 4):
        """Rampa hız değişimi (ani akım çekişini önler)."""
        # Yazılımsal limit kontrolü
        if abs(left_target) > 100 or abs(right_target) > 100:
            print(f"\n[GÜVENLİK İHLALİ] Hız sınırı aşıldı! L:{left_target} R:{right_target}")
            await self.hard_stop()
            return

        while not self.emergency:
            changed = False
            for motor, target in [(self.left, left_target), (self.right, right_target)]:
                diff = target - motor.active_speed
                if abs(diff) > 0.1:
                    if abs(diff) < step:
                        motor.active_speed = target
                    else:
                        motor.active_speed += step if diff > 0 else -step
                    motor.set_speed(motor.active_speed)
                    changed = True

            if changed:
                print(f"  >> Sol %{self.left.active_speed:.1f} | Sağ %{self.right.active_speed:.1f}")
            else:
                break
            await asyncio.sleep(0.05)

    async def _start_action(self, left_speed: float, right_speed: float):
        """Yön değişiminde motorları dinlendirir, sonra harekete geçer."""
        if self.left.active_speed != 0 or self.right.active_speed != 0:
            print("\n[BİLGİ] Yön değişimi algılandı. Motorlar dinlendiriliyor...")
            if self.task and not self.task.done():
                self.task.cancel()
            self.left.set_speed(0)
            self.right.set_speed(0)
            self.left.active_speed = 0
            self.right.active_speed = 0
            await asyncio.sleep(0.5)

        if not self.emergency:
            self.task = asyncio.create_task(self._move_task(left_speed, right_speed))
            await self.task
        else:
            print("[UYARI] Robot yazılımsal kilit modunda, hareket reddedildi!")

    async def forward(self, speed: float):
        print(f"\n[İLERİ] %{speed}")
        await self._start_action(speed, speed)

    async def backward(self, speed: float):
        print(f"\n[GERİ] %{speed}")
        await self._start_action(-speed, -speed)

    async def right_turn(self, speed: float):
        print(f"\n[SAĞ TANK DÖNÜŞÜ] %{speed}")
        await self._start_action(speed, -speed)

    async def left_turn(self, speed: float):
        print(f"\n[SOL TANK DÖNÜŞÜ] %{speed}")
        await self._start_action(-speed, speed)


async def main():
    """Donanım test senaryosu (Khadas VIM3 üzerinde çalıştır)."""
    # VIM3 GPIO kernel numaraları
    left  = MotorDriver(VIM3_LEFT_FWD_GPIO,  VIM3_LEFT_BWD_GPIO,  pwm_chip=VIM3_PWM_LEFT_CHIP,  name="SOL")
    right = MotorDriver(VIM3_RIGHT_FWD_GPIO, VIM3_RIGHT_BWD_GPIO, pwm_chip=VIM3_PWM_RIGHT_CHIP, name="SAG")
    robot = SixWheelRobot(left, right)

    print("\n=== IKA 2026 YAZILIMSAL TEST SENARYOSU BAŞLIYOR ===")

    try:
        # 1. İleri Gidiş
        await robot.forward(50)
        await asyncio.sleep(1)
        await robot.stop()

        # 2. Geri Gidiş
        await robot.backward(50)
        await asyncio.sleep(1)
        await robot.stop()

        # 3. Sağ Tank Dönüşü
        await robot.right_turn(45)
        await asyncio.sleep(0.5)
        await robot.stop()

        # 4. Sol Tank Dönüşü
        await robot.left_turn(45)
        await asyncio.sleep(0.5)
        await robot.stop()

        # 5. FİNAL: Yazılımsal Güvenlik Testi
        print("\n--- TEST SONU: YAZILIMSAL LİMİT DENETİMİ TEST EDİLİYOR ---")
        await robot.forward(101)

    except asyncio.CancelledError:
        pass  # hard_stop zaten halletti.

    finally:
        left.cleanup()
        right.cleanup()
        print("\n[PROGRAM SONLANDI]")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[PROGRAM KAPATILDI]")
