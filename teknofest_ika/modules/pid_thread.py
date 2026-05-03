import threading
import time
import random
from enum import Enum

# PARAMETRELER
SIMULATION_MODE   = True   # Khadas'a yüklerken bunu False yap!
LOOP_DT           = 0.02
MAX_PWM           = 70.0
MIN_PWM           = 0.0
HEARTBEAT_TIMEOUT = 1.0
kS                = 10.0
kV                = 0.6
PWM_SLEW_RATE     = 120.0
MEAS_ALPHA        = 0.7

# DURUM MAKİNESİ
class State(Enum):
    RUN     = 0
    ESTOP   = 1
    TIMEOUT = 2
    LASER   = 3

# 2-DOF PID
class PID2DOF:
    def __init__(self, kp, ki, kd, beta=0.7, kaw=0.5, d_alpha=0.7):
        self.kp      = kp
        self.ki      = ki
        self.kd      = kd
        self.beta    = beta
        self.kaw     = kaw
        self.d_alpha = d_alpha
        self.integral  = 0.0
        self.prev_meas = 0.0
        self.prev_d    = 0.0

    def compute(self, setpoint, meas, dt, u_min, u_max, u_ff=0.0):
        p = self.kp * (self.beta * setpoint - meas)
        raw_d = -(meas - self.prev_meas) / dt
        d_f   = self.d_alpha * self.prev_d + (1 - self.d_alpha) * raw_d
        d     = self.kd * d_f
        self.prev_d    = d_f
        self.prev_meas = meas

        u_unsat = u_ff + p + self.integral + d
        u_sat   = max(u_min, min(u_max, u_unsat))
        aw      = self.kaw * (u_sat - u_unsat)
        self.integral += self.ki * (setpoint - meas) * dt + aw

        u = max(u_min, min(u_max, u_ff + p + self.integral + d))
        return u

    def reset(self):
        self.integral  = 0.0
        self.prev_meas = 0.0
        self.prev_d    = 0.0

# YARDIMCI FONKSİYONLAR
def motor_model(pwm):
    if pwm < kS: return 0.0
    return (pwm - kS) * kV

def motor_model_inv(speed):
    if speed <= 0: return 0.0
    return speed / kV + kS

def slew_limit(current, target, rate, dt):
    delta = rate * dt
    if target > current: return min(current + delta, target)
    else: return max(current - delta, target)

def lowpass(prev, new_val, alpha):
    return alpha * prev + (1 - alpha) * new_val

# GLOBAL DURUM
left_speed  = 0.0
right_speed = 0.0
left_pwm    = 0.0
right_pwm   = 0.0

sp_l = 0.0
sp_r = 0.0

last_heartbeat = time.time()
state          = State.RUN
prev_state     = None

lock = threading.Lock()
emergency_stop = threading.Event()
laser_active   = threading.Event()

system_running = True

# SAFETY THREAD
def safety_thread():
    global left_pwm, right_pwm
    estop_printed = False

    try:
        while system_running:
            hb = last_heartbeat

            if emergency_stop.is_set():
                with lock:
                    left_pwm = right_pwm = 0.0
                if not estop_printed:
                    print("⚠️  E-STOP AKTİF - Motorlar güvenle kilitlendi.")
                    estop_printed = True
            else:
                estop_printed = False

            if not SIMULATION_MODE and (time.time() - hb > HEARTBEAT_TIMEOUT):
                with lock:
                    left_pwm = right_pwm = 0.0
                if not estop_printed:
                    print("⚠️  HABERLEŞME KESİLDİ")
                    estop_printed = True

            if laser_active.is_set():
                with lock:
                    left_pwm = right_pwm = 0.0

            time.sleep(0.02)

    except Exception as e:
        print("❌ SAFETY CRASH:", e)
        emergency_stop.set()

# ANA KONTROL
def main():
    global left_speed, right_speed, left_pwm, right_pwm
    global last_heartbeat, state, prev_state
    global sp_l, sp_r

    pid_l = PID2DOF(1.2, 0.4, 0.15)
    pid_r = PID2DOF(1.2, 0.4, 0.15)

    filt_l = 0.0
    filt_r = 0.0
    last_print_time = time.time()

    try:
        while system_running:
            t0 = time.time()

            if emergency_stop.is_set():
                state = State.ESTOP
            elif not SIMULATION_MODE and (time.time() - last_heartbeat > HEARTBEAT_TIMEOUT):
                state = State.TIMEOUT
            elif laser_active.is_set():
                state = State.LASER
            else:
                state = State.RUN

            if state != prev_state:
                print(f" STATE: {state.name}")
                if state == State.RUN:
                    pid_l.reset()
                    pid_r.reset()
                prev_state = state

            with lock:
                lp = left_pwm
                rp = right_pwm

            if state != State.RUN:
                left_speed  *= 0.9
                right_speed *= 0.9
            else:
                left_speed  += 0.1 * (motor_model(lp) - 0.03 * left_speed)
                right_speed += 0.1 * (motor_model(rp) - 0.03 * right_speed)

            raw_l = left_speed  + random.uniform(-0.2, 0.2)
            raw_r = right_speed + random.uniform(-0.2, 0.2)

            filt_l = lowpass(filt_l, raw_l, MEAS_ALPHA)
            filt_r = lowpass(filt_r, raw_r, MEAS_ALPHA)

            if state == State.RUN:
                last_heartbeat = time.time()
                ff_l = motor_model_inv(sp_l)
                ff_r = motor_model_inv(sp_r)
                u_l = pid_l.compute(sp_l, filt_l, LOOP_DT, MIN_PWM, MAX_PWM, ff_l)
                u_r = pid_r.compute(sp_r, filt_r, LOOP_DT, MIN_PWM, MAX_PWM, ff_r)
                new_lp = slew_limit(lp, u_l, PWM_SLEW_RATE, LOOP_DT)
                new_rp = slew_limit(rp, u_r, PWM_SLEW_RATE, LOOP_DT)
            else:
                new_lp = slew_limit(lp, 0.0, PWM_SLEW_RATE, LOOP_DT)
                new_rp = slew_limit(rp, 0.0, PWM_SLEW_RATE, LOOP_DT)

            with lock:
                left_pwm  = new_lp
                right_pwm = new_rp

            if time.time() - last_print_time > 0.5:
                if state == State.RUN:
                    print(f"L:{filt_l:6.1f} PWM:{left_pwm:5.1f} | R:{filt_r:6.1f} PWM:{right_pwm:5.1f} | {state.name}")
                last_print_time = time.time()

            elapsed = time.time() - t0
            time.sleep(max(0, LOOP_DT - elapsed))

        print("[BİLGİ] Motor kontrol ve güvenlik zincirleri başarıyla kırıldı ve durduruldu.")

    except KeyboardInterrupt:
        print("\n Kapatılıyor...")
        emergency_stop.set()
        time.sleep(0.2)
        print("✅ Güvenli kapanış.")

if __name__ == "__main__":
    threading.Thread(target=safety_thread, daemon=False).start()
    main()