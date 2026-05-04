import asyncio
import os
from unittest.mock import MagicMock

# --- DONANIM YAPILANDIRMASI ---
try:
    import gpiod
except ImportError:
    gpiod = MagicMock()
    gpiod.Chip.return_value.get_line.return_value = MagicMock()

class MotorDriver:
    def __init__(self, forward_pin, backward_pin, pwm_no=0, name="motor"):
        self.name = name
        self.period = 50000  # 20 kHz (50.000 ns)
        self.active_speed = 0
        self.is_simulated = not os.path.exists("/sys/class/pwm/pwmchip0")

        # GPIO Ayarları
        self.chip = gpiod.Chip("gpiochip0")
        self.lines = {
            "forward": self.chip.get_line(forward_pin),
            "backward": self.chip.get_line(backward_pin)
        }
        for line in self.lines.values():
            line.request(consumer=name, type=gpiod.LINE_REQ_DIR_OUT)

        self.pwm_path = f"/sys/class/pwm/pwmchip0/pwm{pwm_no}"
        self._setup_pwm(pwm_no)

    def _setup_pwm(self, pwm_no):
        if not self.is_simulated and not os.path.exists(self.pwm_path):
            try:
                with open("/sys/class/pwm/pwmchip0/export", "w") as f: f.write(str(pwm_no))
            except: pass
        self._write_pwm("enable", 0)
        self._write_pwm("period", self.period)
        self._write_pwm("duty_cycle", 0)
        self._write_pwm("enable", 1)

    def _write_pwm(self, file, value):
        if not self.is_simulated:
            try:
                with open(f"{self.pwm_path}/{file}", "w") as f: f.write(str(value))
            except: pass

class SixWheelRobot:
    def __init__(self, left, right, relay):
        self.left = left
        self.right = right
        self.relay = relay
        self._move_task = None

    async def _accelerate(self, target, direction, step=5):
        """Sadece kalkışta senkron rampa yapar."""
        for m in [self.left, self.right]:
            m.lines["forward"].set_value(1 if direction == "forward" else 0)
            m.lines["backward"].set_value(0 if direction == "forward" else 1)

           
        while abs(target - self.left.active_speed) > 0.1:
            new_speed = self.left.active_speed + step
            if new_speed > target: new_speed = target
            duty = int(self.left.period * (new_speed / 100))
            self.left._write_pwm("duty_cycle", duty)
            self.right._write_pwm("duty_cycle", duty)
            self.left.active_speed = self.right.active_speed = new_speed
            print(f"[ROBOT] Hız: %{new_speed:.1f}")
            await asyncio.sleep(0.01)

    async def move_forward(self, speed):
        print(f"\n--- HAREKET: %{speed} ---")
        self.relay.forward_mode()
        self._move_task = asyncio.create_task(self._accelerate(speed, "forward"))
        await self._move_task

    async def stop(self):
      
        print("\n--- ACİL SENKRON FRENLEME TETİKLENDİ ---")
        
        if self._move_task and not self._move_task.done():
            self._move_task.cancel()

        # 1. SENKRON KİLİTLEME: Tüm pinleri aynı anda HIGH yap
        self.left.lines["forward"].set_value(1)
        self.left.lines["backward"].set_value(1)
        self.right.lines["forward"].set_value(1)
        self.right.lines["backward"].set_value(1)
        
        self.left._write_pwm("duty_cycle", self.left.period)
        self.right._write_pwm("duty_cycle", self.right.period)
        
        self.left.active_speed = self.right.active_speed = 0
        print("[ROBOT] >>> TAM SENKRON AKTİF FRENLEME YAPILDI <<<")
        print("[ROBOT] DURUM: KİLİTLİ | GÜÇ: %100 | FREKANS: 20kHz")

# --- ÇALIŞTIRMA ---
async def main():
    relay = MagicMock()
    relay.forward_mode = lambda: print("[RÖLE] İleri Mod")
    
    robot = SixWheelRobot(
        MotorDriver(17, 18, 0, "SOL"), 
        MotorDriver(22, 23, 1, "SAG"), 
        relay
    )

    await robot.move_forward(60)
    await asyncio.sleep(2)
    await robot.stop()

if __name__ == "__main__":
    asyncio.run(main())