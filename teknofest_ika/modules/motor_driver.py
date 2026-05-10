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
        self.period = 50000          # 20kHz PWM
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
        speed = max(min(speed, 100), -100)
        # Deadzone: Motorların vınlamasını engeller
        if 0 < abs(speed) < 15:
            speed = 15 if speed > 0 else -15

        if speed == self.last_speed: return
        self.last_speed = speed

        # Pinleri temizle ve yönü ata
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
    def __init__(self, left, right, stop_pin=21):
        self.left = left
        self.right = right
        self.task = None
        self.emergency = False

        try:
            self.stop_button = left.chip.get_line(stop_pin)
            self.stop_button.request("E_STOP", gpiod.LINE_REQ_DIR_IN)
        except:
            self.stop_button = MagicMock()

    async def hard_stop(self):
        """Her şeyi anında kesen acil durum durdurması."""
        if self.task and not self.task.done():
            self.task.cancel()
        
        self.left.active_speed = 0 
        self.right.active_speed = 0
        self.left.set_speed(0)
        self.right.set_speed(0)
        print("\n[!!!] SERT DURUŞ YAPILDI")

    async def stop(self):
        """Yavaşlamadan anında duruş sağlar."""
        print("\n[DUR] - Komut Geldi, Anında Frenleniyor...")
        await self.hard_stop()

    async def monitor_estop(self):
        while True:
            try:
                if self.stop_button.get_value() == 1:
                    if not self.emergency:
                        self.emergency = True
                        await self.hard_stop()
                else:
                    self.emergency = False
            except: pass
            await asyncio.sleep(0.01)

    async def _move_task(self, left_target, right_target, step=4):
        """Sadece hızlanırken rampa yapar."""
        while not self.emergency:
            changed = False
            for motor, target in [(self.left, left_target), (self.right, right_target)]:
                
                # Mevcut hız ile hedef arasındaki fark
                diff = target - motor.active_speed
                
                if abs(diff) > 0.1:
                    # Hedefe doğru adım at (Rampa)
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
        if self.task and not self.task.done():
            self.task.cancel()
            try: await self.task
            except asyncio.CancelledError: pass

        if not self.emergency:
            self.task = asyncio.create_task(self._move_task(left_speed, right_speed))
            await self.task

    async def forward(self, speed):
        print(f"\n[İLERİ] %{speed}")
        await self.start_action(speed, speed)

    async def backward(self, speed):
        print(f"\n[GERİ] %{speed}")
        await self.start_action(-speed, -speed)

    async def left_turn(self, speed):
        print(f"\n[SOL TANK] %{speed}")
        await self.start_action(-speed, speed)

    async def right_turn(self, speed):
        print(f"\n[SAĞ TANK] %{speed}")
        await self.start_action(speed, -speed)

    async def start_action(self, l, r):
        await self._start_action(l, r)

# --- ANA ÇALIŞTIRMA ---
async def main():
    left = MotorDriver(17, 18, 0, "SOL")
    right = MotorDriver(27, 22, 1, "SAG")
    robot = SixWheelRobot(left, right)

    # E-Stop dinleyicisini arka planda başlat
    asyncio.create_task(robot.monitor_estop())
    
    print("\n=== IKA 2026 TAM DENETİM TESTİ BAŞLIYOR ===")

    try:
        # 1. TEST: İLERİ GİDİŞ
        print("\n[TEST 1] İleri Hareket Denetleniyor...")
        await robot.forward(50)
        await asyncio.sleep(1.5) # 1.5 saniye boyunca %50 hızda git
        await robot.stop()       # Sert fren yap
        await asyncio.sleep(1)

        
        # 2. TEST: GERİ GİDİŞ
        print("\n[TEST 2] Geri Hareket Denetleniyor...")
        await robot.backward(40)
        await asyncio.sleep(1.5)
        await robot.stop()
        await asyncio.sleep(1)

        # 3. TEST: SAĞA TANK DÖNÜŞÜ
        print("\n[TEST 3] Sağa Tank Dönüşü Denetleniyor...")
        await robot.right_turn(30)
        await asyncio.sleep(1.5)
        await robot.stop()
        await asyncio.sleep(1)

        # 4. TEST: SOLA TANK DÖNÜŞÜ
        print("\n[TEST 4] Sola Tank Dönüşü Denetleniyor...")
        await robot.left_turn(30)
        await asyncio.sleep(1.5)
        await robot.stop()
        await asyncio.sleep(1)

        # 5. TEST: ACİL DURDURMA (E-STOP) AKTİF Mİ?
        print("\n[TEST 5] E-Stop Mekanizması Denetleniyor...")
        print(">> Robot hızlanırken sanal butona basılacak...")
        
        asyncio.create_task(robot.forward(100)) 
        await asyncio.sleep(0.4) 
        robot.stop_button.get_value.return_value = 1 
        

        await asyncio.sleep(1)
        print("\n=== TEST TAMAMLANDI )")

    finally:
        await robot.hard_stop()
        left.cleanup()
        right.cleanup()
        
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[PROGRAM KAPATILDI]")
 
       



