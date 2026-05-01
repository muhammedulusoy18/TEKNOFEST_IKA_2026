import threading
import time
import random
from enum import Enum


# PARAMETRELER

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

        # I (back-calculation anti-windup)
        u_unsat = u_ff + p + self.integral + d
        u_sat   = max(u_min, min(u_max, u_unsat))
        aw      = self.kaw * (u_sat - u_unsat)
        self.integral += self.ki * (setpoint - meas) * dt + aw

        u = max(u_min, min(u_max, u_ff + p + self.integral + d))
        return u

    def reset(self):
        self.integral  = 0.0#(Hata Birikimini Temizle)
        self.prev_meas = 0.0#(Hız Hafızasını Sıfırla)
        self.prev_d    = 0.0#Filtre Hafızasını Temizle)



# YARDIMCI FONKSİYONLAR

def motor_model(pwm):
    if pwm < kS:
        return 0.0
    return (pwm - kS) * kV

def motor_model_inv(speed):
    if speed <= 0:
        return 0.0
    return speed / kV + kS

def slew_limit(current, target, rate, dt):
    delta = rate * dt
    if target > current:
        return min(current + delta, target)
    else:
        return max(current - delta, target)

def lowpass(prev, new_val, alpha):
    return alpha * prev + (1 - alpha) * new_val



# Şartname 6.10: stop noktasında 2s zorunlu bekleme

def ramp_stop_and_wait(lock, pwm_ref):
    print("🛑 STOP - 2 saniye bekleniyor...")
    with lock:
        pwm_ref[0] = 0.0
        pwm_ref[1] = 0.0
    time.sleep(2.0)
    print("▶️  Devam ediliyor...")



# GLOBAL DURUM

left_speed  = 0.0
right_speed = 0.0
left_pwm    = 0.0
right_pwm   = 0.0

last_heartbeat = time.time()
state          = State.RUN
prev_state     = None

lock = threading.Lock()

# Şartname 7.8: thread-safe Event
emergency_stop = threading.Event()
laser_active   = threading.Event()



# SAFETY THREAD
# Şartname 7.8: ana kontrolden tamamen bağımsız

def safety_thread():
    global left_pwm, right_pwm

    try:
        while True:
            hb = last_heartbeat  # GIL korumalı

            if emergency_stop.is_set():
                with lock:
                    left_pwm = right_pwm = 0.0
                print("⚠️  E-STOP AKTİF")

            elif time.time() - hb > HEARTBEAT_TIMEOUT:
                with lock:
                    left_pwm = right_pwm = 0.0
                print("⚠️  HABERLEŞME KESİLDİ")

            elif laser_active.is_set():
                with lock:
                    left_pwm = right_pwm = 0.0

            time.sleep(0.02)

    except Exception as e:
        print("❌ SAFETY CRASH:", e)
        emergency_stop.set()



# ANA KONTROL

def main():
    global left_speed, right_speed
    global left_pwm, right_pwm
    global last_heartbeat, state, prev_state

    pid_l = PID2DOF(1.2, 0.4, 0.15)
    pid_r = PID2DOF(1.2, 0.4, 0.15)

    filt_l = 0.0
    filt_r = 0.0
    sp_l   = 100.0
    sp_r   = 100.0

    try:
        while True:
            t0 = time.time()

            # -------- STATE MACHINE --------
            if emergency_stop.is_set():
                state = State.ESTOP
            elif time.time() - last_heartbeat > HEARTBEAT_TIMEOUT:
                state = State.TIMEOUT
            elif laser_active.is_set():
                state = State.LASER
            else:
                state = State.RUN

            if state != prev_state:
                print(f" STATE: {state.name}")
                # State değişince PID sıfırla
                if state == State.RUN:
                    pid_l.reset()
                    pid_r.reset()
                prev_state = state

            # -------- SENSOR (SIM) --------
            with lock:
                lp = left_pwm
                rp = right_pwm

            if state != State.RUN:
                left_speed  *= 0.9
                right_speed *= 0.9
            else:
                left_speed  += 0.1 * (motor_model(lp) - 0.03 * left_speed)
                right_speed += 0.1 * (motor_model(rp) - 0.03 * right_speed)

            #  noise sadece ham ölçüme, speed temiz kalır
            raw_l = left_speed  + random.uniform(-0.2, 0.2)
            raw_r = right_speed + random.uniform(-0.2, 0.2)

            filt_l = lowpass(filt_l, raw_l, MEAS_ALPHA)
            filt_r = lowpass(filt_r, raw_r, MEAS_ALPHA)

            # -------- CONTROL --------
            if state == State.RUN:
                last_heartbeat = time.time()

                ff_l = motor_model_inv(sp_l)
                ff_r = motor_model_inv(sp_r)

                u_l = pid_l.compute(sp_l, filt_l, LOOP_DT,
                                    MIN_PWM, MAX_PWM, ff_l)
                u_r = pid_r.compute(sp_r, filt_r, LOOP_DT,
                                    MIN_PWM, MAX_PWM, ff_r)

                new_lp = slew_limit(lp, u_l, PWM_SLEW_RATE, LOOP_DT)
                new_rp = slew_limit(rp, u_r, PWM_SLEW_RATE, LOOP_DT)

            else:
                #  Anında sıfır değil, slew ile yumuşak durma
                new_lp = slew_limit(lp, 0.0, PWM_SLEW_RATE, LOOP_DT)
                new_rp = slew_limit(rp, 0.0, PWM_SLEW_RATE, LOOP_DT)

            with lock:
                left_pwm  = new_lp
                right_pwm = new_rp

            print(
                f"L:{filt_l:6.1f} PWM:{left_pwm:5.1f} | "
                f"R:{filt_r:6.1f} PWM:{right_pwm:5.1f} | "
                f"{state.name}"
            )

            # -------- TIMING --------
            elapsed = time.time() - t0
            time.sleep(max(0, LOOP_DT - elapsed))

    except KeyboardInterrupt:
        print("\n Kapatılıyor...")
        emergency_stop.set()
        time.sleep(0.2)
        print("✅ Güvenli kapanış.")



# START

if __name__ == "__main__":
    threading.Thread(target=safety_thread, daemon=False).start()
    main()
