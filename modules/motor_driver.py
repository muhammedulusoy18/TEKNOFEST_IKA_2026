import gpiod
import time
import os


class MotorDriver:
    def __init__(self, forward_pin, backward_pin, pwm_no=0, name="motor"):
        self.name = name
        self.period = 200000  # 5 kHz
        self.active_speed = 0


        self.MAX_SPEED = 80

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
        time.sleep(0.1)
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

    def _set_speed(self, speed):
        # Software Safety Limit (BMS Protection)
        speed = max(0, min(self.MAX_SPEED, speed))
        duty = int(self.period * speed / 100)
        self._write("duty_cycle", duty)
        self.active_speed = speed

    def speed_ramp(self, target, step=2, delay=0.03):

        target = min(target, self.MAX_SPEED)

        if self.active_speed < target:
            while self.active_speed < target:
                self.active_speed += step
                self._set_speed(self.active_speed)
                time.sleep(delay)
        else:
            while self.active_speed > target:
                self.active_speed -= step
                self._set_speed(self.active_speed)
                time.sleep(delay)

    def instant_stop(self):
        """Emergency brake: Cuts energy without waiting for the ramp."""
        self._write("duty_cycle", 0)
        self.forward_line.set_value(0)
        self.backward_line.set_value(0)
        self.active_speed = 0

    def stop(self):
        """Normal stop: Uses ramp for smooth deceleration."""
        self.speed_ramp(0, step=5)
        self.forward_line.set_value(0)
        self.backward_line.set_value(0)

    def move(self, speed, direction="forward"):
        if direction == "forward":
            self.backward_line.set_value(0)
            self.forward_line.set_value(1)
        else:
            self.forward_line.set_value(0)
            self.backward_line.set_value(1)
        self.speed_ramp(speed)

    def cleanup(self):
        self.instant_stop()
        self._write("enable", 0)
        self.forward_line.release()
        self.backward_line.release()
        self.chip.close()


class SixWheelRobot:
    def __init__(self, left_motor, right_motor, relay):
        self.left = left_motor
        self.right = right_motor
        self.relay = relay
        self.last_command = time.time()

    def emergency_stop(self):
        self.left.instant_stop()
        self.right.instant_stop()
        print("!!! EMERGENCY STOP TRIGGERED !!!")

    def failsafe_check(self):
        """Stops the robot if communication is lost."""
        if time.time() - self.last_command > 1.5:
            print("CONNECTION LOST: FAILSAFE ACTIVE")
            self.stop()

    def stop(self):
        self.left.stop()
        self.right.stop()

    def forward(self, speed):
        self.last_command = time.time()
        self.relay.forward_mode()
        self.left.move(speed, "forward")
        self.right.move(speed, "forward")

    def backward(self, speed):
        self.last_command = time.time()
        self.stop()  # Stop first, then change direction (For Relay and Gearbox health)
        time.sleep(0.2)
        self.relay.backward_mode()
        self.left.move(speed, "backward")
        self.right.move(speed, "backward")

    def turn_right(self, speed, sharpness=0.4):
        """
        Differential Turn: Slows down the inner wheels,
        keeps the outer wheels at the main speed.
        """
        self.last_command = time.time()
        self.relay.forward_mode()
        self.left.move(speed, "forward")
        self.right.move(speed * (1 - sharpness), "forward")

    def spin_right(self, speed):
        """Tank Turn: Spins in place to the right."""
        self.last_command = time.time()
        self.relay.forward_mode()
        self.left.move(speed, "forward")
        self.right.move(speed, "backward")

    def turn_left(self, speed, sharpness=0.4):

        self.last_command = time.time()
        self.relay.forward_mode()
        self.left.move(speed * (1 - sharpness), "forward")
        self.right.move(speed, "forward")

    def spin_left(self, speed):

        self.last_command = time.time()
        self.relay.forward_mode()
        self.left.move(speed, "backward")
        self.right.move(speed, "forward")

    def cleanup(self):
        self.left.cleanup()
        self.right.cleanup()
        self.relay.cleanup()