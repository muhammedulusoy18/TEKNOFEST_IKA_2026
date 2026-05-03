import gpiod
import asyncio
import os
import time

class MotorDriver:
    def __init__(self, forward_pin, backward_pin, pwm_no=0, name="motor"):
        self.name = name
        self.period = 500000  # 5 kHz
        self.active_speed = 0
        self.MAX_SPEED = 80
        self._ramp_task = None  # Hız rampası için asenkron görev

        # GPIO Setup
        self.chip = gpiod.Chip("gpiochip0")
        self.forward_line = self.chip.get_line(forward_pin)
        self.backward_line = self.chip.get_line(backward_pin)
        self.forward_line.request(consumer=name, type=gpiod.LINE_REQ_DIR_OUT)
        self.backward_line.request(consumer=name, type=gpiod.LINE_REQ_DIR_OUT)

        # PWM Setup
        self.pwm_chip = "/sys/class/pwm/pwmchip0"
        self.pwm_no = pwm_no
        self.pwm_path = f"{self.pwm_chip}/pwm{self.pwm_no}"
        self._init_pwm()

    def _init_pwm(self):
        if not os.path.exists(self.pwm_path):
            with open(f"{self.pwm_chip}/export", "w") as f:
                f.write(str(self.pwm_no))
        self._write("enable", 0)
        self._write("period", self.period)
        self._write("duty_cycle", 0)
        self._write("enable", 1)

    def _write(self, file, value):
        try:
            with open(f"{self.pwm_path}/{file}", "w") as f:
                f.write(str(value))
        except Exception as e:
            print(f"[{self.name}] PWM Error: {e}")

    def _set_speed_raw(self, speed):
        """Matematiksel taşma korumalı hız ayarı."""
        # Üst limit kontrolü (Overflow önleyici)
        speed = max(0, min(self.MAX_SPEED, speed))
        duty = int(self.period * speed / 100)
        self._write("duty_cycle", duty)
        self.active_speed = speed

    async def speed_ramp(self, target, step=4, delay=0.01):
        """Asenkron Rampa: Sistemi kilitlemeden hızı değiştirir."""
        # Eğer hali hazırda bir rampa görevi varsa iptal et (Yeni komut önceliklidir)
        if self._ramp_task and not self._ramp_task.done():
            self._ramp_task.cancel()

        target = max(0, min(target, self.MAX_SPEED))
        
        while abs(self.active_speed - target) > 0.1:
            if self.active_speed < target:
                self.active_speed = min(self.active_speed + step, target) # Hedefi aşma koruması
            else:
                self.active_speed = max(self.active_speed - step, target) # Hedefin altına düşme koruması
            
            self._set_speed_raw(self.active_speed)
            await asyncio.sleep(delay) # Sistemi kitlemeyen asenkron bekleme

    def set_direction(self, direction):
        """Yönü anlık değiştirir, bloklama yapmaz."""
        if direction == "forward":
            self.backward_line.set_value(0)
            self.forward_line.set_value(1)
        elif direction == "backward":
            self.forward_line.set_value(0)
            self.backward_line.set_value(1)
        else:
            self.forward_line.set_value(0)
            self.backward_line.set_value(0)

    async def move(self, speed, direction="forward"):
        self.set_direction(direction)
        self._ramp_task = asyncio.create_task(self.speed_ramp(speed))

    def instant_stop(self):
        """Acil fren: PWM ve GPIO anında kesilir."""
        if self._ramp_task:
            self._ramp_task.cancel()
        self._write("duty_cycle", 0)
        self.forward_line.set_value(0)
        self.backward_line.set_value(0)
        self.active_speed = 0

class SixWheelRobot:
    def __init__(self, left_motor, right_motor, relay):
        self.left = left_motor
        self.right = right_motor
        self.relay = relay
        self.last_command = time.time()

    async def forward(self, speed):
        self.last_command = time.time()
        self.relay.forward_mode()
        # İki motoru aynı anda asenkron başlat
        await asyncio.gather(
            self.left.move(speed, "forward"),
            self.right.move(speed, "forward")
        )

    async def backward(self, speed):
        self.last_command = time.time()
        # Röle ve dişli sağlığı için çok kısa bir duraksama yeterli (0.2s yerine asenkron 0.1s)
        self.left.instant_stop()
        self.right.instant_stop()
        await asyncio.sleep(0.1) 
        
        self.relay.backward_mode()
        await asyncio.gather(
            self.left.move(speed, "backward"),
            self.right.move(speed, "backward")
        )

    async def spin_right(self, speed):
        """Gerçek Tank Dönüşü: Sol ileri, Sağ geri."""
        self.last_command = time.time()
        self.relay.forward_mode() 
        await asyncio.gather(
            self.left.move(speed, "forward"),
            self.right.move(speed, "backward")
        )

    async def spin_left(self, speed):
        """Gerçek Tank Dönüşü: Sağ ileri, Sol geri."""
        self.last_command = time.time()
        self.relay.forward_mode()
        await asyncio.gather(
            self.left.move(speed, "backward"),
            self.right.move(speed, "forward")
        )

    async def stop(self):
        """Yumuşak duruş."""
        await asyncio.gather(
            self.left.speed_ramp(0, step=8),
            self.right.speed_ramp(0, step=8)
        )
        self.left.set_direction("stop")
        self.right.set_direction("stop")

    def emergency_stop(self):
        self.left.instant_stop()
        self.right.instant_stop()
        print("!!! ACİL DURDURMA !!!")