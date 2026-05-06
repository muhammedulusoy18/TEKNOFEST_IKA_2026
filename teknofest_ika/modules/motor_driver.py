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
        self.period = 50000  # 20 kHz
        self.active_speed = 0
        self.is_simulated = not os.path.exists("/sys/class/pwm/pwmchip0")

        try:
            self.chip = gpiod.Chip("gpiochip0")
            self.lines = {
                "forward": self.chip.get_line(forward_pin),
                "backward": self.chip.get_line(backward_pin)
            }
            for line in self.lines.values():
                line.request(consumer=name, type=gpiod.LINE_REQ_DIR_OUT)
        except Exception:
            self.lines = {"forward": MagicMock(), "backward": MagicMock()}

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
    def __init__(self, left_motor, right_motor):
        self.left = left_motor
        self.right = right_motor
        self._current_task = None
        self.MAX_SPEED = 100.0
        self.MIN_SPEED = 0.0

    async def _execute_movement(self, left_target, right_target, step=5):
        """Hızlanma ve yavaşlamayı kademeli yapar ve terminale yazdırır."""
        left_target = max(min(left_target, self.MAX_SPEED), -self.MAX_SPEED)
        right_target = max(min(right_target, self.MAX_SPEED), -self.MAX_SPEED)

        while True:
            changed = False
            for m, target in [(self.left, left_target), (self.right, right_target)]:
                if abs(m.active_speed - target) > 0.1:
                    if m.active_speed < target:
                        m.active_speed = min(m.active_speed + step, target)
                    else:
                        m.active_speed = max(m.active_speed - step, target)
                    
                    m.lines["forward"].set_value(1 if m.active_speed > 0 else 0)
                    m.lines["backward"].set_value(1 if m.active_speed < 0 else 0)
                    
                    duty = int(m.period * (abs(m.active_speed) / 100))
                    m._write_pwm("duty_cycle", duty)
                    changed = True
            
            if changed:
                # Terminalde hız değişimini takip etmek için eklenen kısım
                print(f"   >> Hız Güncellendi: Sol %{self.left.active_speed:.1f} | Sağ %{self.right.active_speed:.1f}")
            else:
                break
                
            await asyncio.sleep(0.05) # Terminal takibi için ideal süre

    async def _start_task(self, left_speed, right_speed):
        if self._current_task and not self._current_task.done():
            self._current_task.cancel()
            try: await self._current_task
            except asyncio.CancelledError: pass
        
        self._current_task = asyncio.create_task(self._execute_movement(left_speed, right_speed))
        await self._current_task

    async def move_forward(self, speed):
        print(f"\n[HAREKET] Hedef İleri %{speed}")
        await self._start_task(speed, speed)

    async def move_backward(self, speed):
        print(f"\n[HAREKET] Hedef Geri %{speed}")
        await self._start_task(-speed, -speed)

    async def turn_left_tank(self, speed):
        print(f"\n[DÖNÜŞ] Hedef Tank Sola %{speed}")
        await self._start_task(-speed, speed)

    async def turn_right_tank(self, speed):
        print(f"\n[DÖNÜŞ] Hedef Tank Sağa %{speed}")
        await self._start_task(speed, -speed)

    async def stop(self):
        print("\n[SİSTEM] Durduruluyor...")
        await self._start_task(0, 0)

async def main():
    sol = MotorDriver(forward_pin=17, backward_pin=18, pwm_no=0, name="Sol_Grup")
    sag = MotorDriver(forward_pin=27, backward_pin=22, pwm_no=1, name="Sag_Grup")
    ika = SixWheelRobot(left_motor=sol, right_motor=sag)
    
    print("--- IKA_2026 Kontrol Sistemi Başlatıldı ---")

    try:
        # Örnek Senaryo
        print("ileri test başlatıldı")
        await ika.move_forward(50)
        await asyncio.sleep(1)

        print("tank dönüş sistemı başlatıldı")
        await ika.turn_right_tank(40)
        await asyncio.sleep(1)

        print(" geri  test başlatıldı")
        await ika.move_backward(50)
        await asyncio.sleep(1)

        
        await ika.stop()
    finally:
        await ika.stop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nKullanıcı durdurdu.")
       



