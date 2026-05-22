import asyncio
import os
from unittest.mock import MagicMock

try:
    import gpiod
except ImportError:
    gpiod = MagicMock()
    gpiod.LINE_REQ_DIR_OUT = 1
    gpiod.LINE_REQ_DIR_IN = 2

class MotorDriver:
    def __init__(self, forward_pin, backward_pin, pwm_no, name="motor"):
        self.name = name
        self.period = 50000
        self.active_speed = 0
        self.last_speed = None       
        self.simulated = not os.path.exists("/sys/class/pwm/pwmchip0")
        self.pwm_path = f"/sys/class/pwm/pwmchip0/pwm{pwm_no}"

        try:
            self.chip = gpiod.Chip("gpiochip0")
            self.forward = self.chip.get_line(forward_pin)
            self.backward = self.chip.get_line(backward_pin)
            self.forward.request(name, gpiod.LINE_REQ_DIR_OUT)
            self.backward.request(name, gpiod.LINE_REQ_DIR_OUT)
        except Exception as e:
            if not self.simulated: print(f"[GPIO HATA] {self.name}: {e}")
            self.forward = MagicMock()
            self.backward = MagicMock()

        self._setup_pwm(pwm_no)

    def _setup_pwm(self, pwm_no):
        if not self.simulated and not os.path.exists(self.pwm_path):
            try:
                with open("/sys/class/pwm/pwmchip0/export", "w") as f:
                    f.write(str(pwm_no))
            except Exception as e:
                print(f"[PWM EXPORT HATA] {e}")

        self._write_pwm("enable", 0)
        self._write_pwm("period", self.period)
        self._write_pwm("duty_cycle", 0)
        self._write_pwm("enable", 1)

    def _write_pwm(self, file, value):
        if self.simulated: return
        try:
            with open(f"{self.pwm_path}/{file}", "w") as f:
                f.write(str(value))
        except Exception as e:
            print(f"[PWM YAZMA HATA] {e}")

    def set_speed(self, speed):
        if speed == 0:
            self.forward.set_value(0)
            self.backward.set_value(0)
            self._write_pwm("duty_cycle", 0)
            self.last_speed = 0
            return

        if 0 < abs(speed) < 15:
            speed = 15 if speed > 0 else -15

        if speed == self.last_speed: return
        self.last_speed = speed

        self.forward.set_value(0)
        self.backward.set_value(0)

        if speed > 0:
            self.forward.set_value(1)
        elif speed < 0:
            self.backward.set_value(1)

        duty = int(self.period * abs(speed) / 100)
        self._write_pwm("duty_cycle", duty)

    def cleanup(self):
        self._write_pwm("enable", 0)
        try:
            self.forward.release()
            self.backward.release()
        except: pass

class SixWheelRobot:
    def __init__(self, left, right):
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

    async def _move_task(self, left_target, right_target, step=4):
        # YAZILIMSAL LİMİT KONTROLÜ
        if abs(left_target) > 100 or abs(right_target) > 100:
            print(f"\n[GÜVENLİK İHLALİ] Hız sınırı aşıldı! Hedef: {left_target}")
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

    async def _start_action(self, left_speed, right_speed):
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

    async def forward(self, speed):
        print(f"\n[İLERİ] %{speed}")
        await self._start_action(speed, speed)

    async def backward(self, speed):
        print(f"\n[GERİ] %{speed}")
        await self._start_action(-speed, -speed)

    async def right_turn(self, speed):
        print(f"\n[SAĞ TANK DÖNÜŞÜ] %{speed}")
        await self._start_action(speed, -speed)

    async def left_turn(self, speed):
        print(f"\n[SOL TANK DÖNÜŞÜ] %{speed}")
        await self._start_action(-speed, speed)

async def main():
    left = MotorDriver(17, 18, 0, "SOL")
    right = MotorDriver(27, 22, 1, "SAG")
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

        # 3.  sağ  Tank DönüşlerisS
        await robot.right_turn(45)
        await asyncio.sleep(0.5)
        await robot.stop()

        # 4 sol tank dönüşü:
        await robot.left_turn(45)
        await asyncio.sleep(0.5)
        await robot.stop()

        # 4. FİNAL: Yazılımsal Güvenlik Testi (%101 Hız denemesi)
        print("\n--- TEST SONU: YAZILIMSAL LİMİT DENETİMİ TEST EDİLİYOR ---")
        await robot.forward(101)

    except asyncio.CancelledError:
        pass  # ← hard_stop zaten halletti.

    finally:
          left.cleanup()
          right.cleanup()
          print("\n[PROGRAM SONLANDI]")
        
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[PROGRAM KAPATILDI]")
 
       



