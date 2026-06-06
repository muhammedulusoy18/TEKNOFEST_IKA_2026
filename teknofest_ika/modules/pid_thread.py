"""
pid_thread.py — IKA 2026 Hız-PID Kontrol Katmanı
=================================================
Khadas VIM3 üzerinde gerçek PWM + simülasyon modunu destekler.
Thread-safe global setpoint erişimi lock ile korunur.
"""

import threading
import time
import random
import os
from enum import Enum
from unittest.mock import MagicMock

# --- DONANIM SOYUTLAMA ---
try:
    import gpiod
    GPIOD_AVAILABLE = True
except ImportError:
    gpiod = MagicMock()
    GPIOD_AVAILABLE = False

# ============================================================
# KONFİGÜRASYON
# ============================================================
# Gerçek donanım varsa (VIM3 + pwmchip0) otomatik False olur
SIMULATION_MODE   = not os.path.exists("/sys/class/pwm/pwmchip0")

LOOP_DT           = 0.02          # 50 Hz kontrol döngüsü
MAX_PWM           = 70.0          # Maksimum PWM değeri (güvenlik sınırı)
MIN_PWM           = 0.0
HEARTBEAT_TIMEOUT = 1.0           # 1 saniye heartbeat yok → dur
kS                = 10.0          # Motor statik sürtünme eşiği (PWM)
kV                = 0.6           # Motor kazancı (hız/pwm)
PWM_SLEW_RATE     = 120.0         # PWM/s rampa hızı
MEAS_ALPHA        = 0.7           # Ölçüm düşük geçiren filtresi

# VIM3 PWM sysfs yolları
VIM3_PWM_LEFT_PATH  = "/sys/class/pwm/pwmchip0/pwm0"   # pwm_ao_a
VIM3_PWM_RIGHT_PATH = "/sys/class/pwm/pwmchip4/pwm0"   # pwm_f
PWM_PERIOD_NS       = 50000                             # 50µs → 20kHz

# ============================================================
# DURUM MAKİNESİ
# ============================================================
class State(Enum):
    RUN     = 0
    ESTOP   = 1
    TIMEOUT = 2
    LASER   = 3


# ============================================================
# 2-DOF PID SINIFI
# ============================================================
class PID2DOF:
    """
    2 serbestlik dereceli PID:
    - Beta: Referans ağırlıklaması (set-point kick'i azaltır)
    - Anti-windup: Saturasyon sırasında integral birikimini önler
    - D-filtresi: Gürültüyü bastırır
    """
    def __init__(self, kp: float, ki: float, kd: float,
                 beta: float = 0.7, kaw: float = 0.5, d_alpha: float = 0.7):
        self.kp      = kp
        self.ki      = ki
        self.kd      = kd
        self.beta    = beta
        self.kaw     = kaw
        self.d_alpha = d_alpha
        self.integral  = 0.0
        self.prev_meas = 0.0
        self.prev_d    = 0.0

    def compute(self, setpoint: float, meas: float, dt: float,
                u_min: float, u_max: float, u_ff: float = 0.0) -> float:
        if dt <= 0:
            return 0.0

        # Oransal (Beta ağırlıklı — set-point kick'i azaltır)
        p = self.kp * (self.beta * setpoint - meas)

        # Türev (Ölçüm üzerinden, filtreli)
        raw_d = -(meas - self.prev_meas) / dt
        d_f   = self.d_alpha * self.prev_d + (1 - self.d_alpha) * raw_d
        d     = self.kd * d_f
        self.prev_d    = d_f
        self.prev_meas = meas

        # Anti-Windup
        u_unsat = u_ff + p + self.integral + d
        u_sat   = max(u_min, min(u_max, u_unsat))
        aw      = self.kaw * (u_sat - u_unsat)

        # Integral güncelleme
        self.integral += self.ki * (setpoint - meas) * dt + aw

        return u_sat

    def reset(self):
        self.integral  = 0.0
        self.prev_meas = 0.0
        self.prev_d    = 0.0


# ============================================================
# MODEL YARDIMCILARI
# ============================================================
def motor_model(pwm: float) -> float:
    """PWM → Simüle edilmiş hız (doğrusal model)."""
    if pwm < kS:
        return 0.0
    return (pwm - kS) * kV

def motor_model_inv(speed: float) -> float:
    """Hız → İleri besleme PWM (feedforward)."""
    if speed <= 0:
        return 0.0
    return speed / kV + kS

def slew_limit(current: float, target: float, rate: float, dt: float) -> float:
    """PWM'in ani değişimini sınırlar (motor koruması)."""
    delta = rate * dt
    if target > current:
        return min(current + delta, target)
    else:
        return max(current - delta, target)

def lowpass(prev: float, new_val: float, alpha: float) -> float:
    return alpha * prev + (1 - alpha) * new_val


# ============================================================
# PWM DONANIM YAZICI
# ============================================================
def _write_pwm_file(path: str, value) -> bool:
    """sysfs PWM dosyasına değer yazar. Başarısızsa False döner."""
    try:
        with open(path, "w") as f:
            f.write(str(value))
        return True
    except Exception as e:
        print(f"[PWM HATA] {path}: {e}")
        return False

def _setup_pwm_channel(chip_no: int) -> bool:
    """PWM kanalını dışa açar ve yapılandırır. Khadas VIM3 uyumlu."""
    if SIMULATION_MODE:
        return False
    chip_path = f"/sys/class/pwm/pwmchip{chip_no}"
    pwm_path  = f"{chip_path}/pwm0"
    if not os.path.exists(chip_path):
        print(f"[PWM UYARI] pwmchip{chip_no} bulunamadı. Overlay aktif mi?")
        print("[PWM İPUCU] /boot/env.txt → overlays=pwm_ao_a pwm_f uart3 i2c3")
        return False
    if not os.path.exists(pwm_path):
        _write_pwm_file(f"{chip_path}/export", "0")
        time.sleep(0.05)
    _write_pwm_file(f"{pwm_path}/enable",     "0")
    _write_pwm_file(f"{pwm_path}/period",     str(PWM_PERIOD_NS))
    _write_pwm_file(f"{pwm_path}/duty_cycle", "0")
    _write_pwm_file(f"{pwm_path}/enable",     "1")
    return True

def write_motor_pwm(pwm_path: str, duty: float):
    """PWM duty cycle'ı günceller. duty: 0-MAX_PWM aralığında."""
    if SIMULATION_MODE:
        return
    duty_ns = int(PWM_PERIOD_NS * abs(duty) / MAX_PWM)
    _write_pwm_file(f"{pwm_path}/duty_cycle", str(duty_ns))


# ============================================================
# GLOBAL DURUM (Thread-safe erişim için lock zorunlu)
# ============================================================
_lock          = threading.Lock()

_sp_l:   float = 0.0   # Sol motor setpoint (sadece _lock ile oku/yaz)
_sp_r:   float = 0.0   # Sağ motor setpoint (sadece _lock ile oku/yaz)

left_speed:  float = 0.0
right_speed: float = 0.0
left_pwm:    float = 0.0
right_pwm:   float = 0.0

_last_heartbeat: float = time.time()
state:      State = State.RUN
prev_state: State = None

emergency_stop  = threading.Event()
laser_active    = threading.Event()
system_running  = True


# ============================================================
# THREAD-SAFE SETPOINT ERİŞİCİLERİ
# ============================================================
def get_setpoints() -> tuple:
    """Güvenli setpoint okuma."""
    with _lock:
        return _sp_l, _sp_r

def set_setpoints(l: float, r: float):
    """Güvenli setpoint yazma."""
    global _sp_l, _sp_r
    with _lock:
        _sp_l = max(-MAX_PWM, min(MAX_PWM, l))
        _sp_r = max(-MAX_PWM, min(MAX_PWM, r))

def update_heartbeat():
    """Son komut zamanını günceller (timeout koruması için)."""
    global _last_heartbeat
    with _lock:
        _last_heartbeat = time.time()

# Geriye dönük uyumluluk için property benzeri global erişim
# (vehicle_manager.py'de pid_ctrl.sp_l = x şeklinde kullanılıyor)
# Bu bir anti-pattern ama mevcut kodu kırmamak için tutuldu.
class _SetpointProxy:
    """pid_ctrl.sp_l = 50.0 yazımını thread-safe yapar."""
    @property
    def sp_l(self): return get_setpoints()[0]
    @sp_l.setter
    def sp_l(self, v): set_setpoints(v, get_setpoints()[1])

    @property
    def sp_r(self): return get_setpoints()[1]
    @sp_r.setter
    def sp_r(self, v): set_setpoints(get_setpoints()[0], v)


# Module-level proxy — `import pid_thread as pid_ctrl; pid_ctrl.sp_l = 50` çalışır
# ancak doğrudan global değişken ataması thread-safe değildir.
# Kullanım: pid_ctrl.set_setpoints(l, r) tercih edilmeli.
sp_l = 0.0  # Legacy — doğrudan kullanmaktan kaçının
sp_r = 0.0  # Legacy — doğrudan kullanmaktan kaçının
last_heartbeat = time.time()  # Legacy


# ============================================================
# GÜVENLİK THREAD'İ
# ============================================================
def safety_thread():
    """
    50Hz'de çalışır. E-Stop, heartbeat timeout ve laser durumlarını izler.
    Kritik durumlarda motorları anında durdurur.
    """
    global left_pwm, right_pwm, state, system_running
    estop_printed = False

    try:
        while system_running:
            with _lock:
                hb = _last_heartbeat

            # E-Stop (acil durdurma butonu veya yazılım tetikleyicisi)
            if emergency_stop.is_set():
                with _lock:
                    left_pwm = right_pwm = 0.0
                    state = State.ESTOP
                if not estop_printed:
                    print("\n[SAFETY] ⛔ GÜVENLİK TETİKLENDİ - Motorlar durduruldu.")
                    estop_printed = True

            # Heartbeat Timeout (komünikasyon kesildi)
            elif (time.time() - hb) > HEARTBEAT_TIMEOUT:
                with _lock:
                    left_pwm = right_pwm = 0.0
                    state = State.TIMEOUT
                if not estop_printed:
                    print("\n[SAFETY] ⏱ HEARTBEAT ZAMAN AŞIMI - Motorlar durduruldu.")
                    estop_printed = True

            # Laser Aktif (atış sırasında yavaş)
            elif laser_active.is_set():
                with _lock:
                    state = State.LASER
                estop_printed = False

            else:
                with _lock:
                    if state != State.RUN:
                        state = State.RUN
                estop_printed = False

            time.sleep(0.05)

    except Exception as e:
        print(f"[ERROR] Safety Thread hatası: {e}")
        emergency_stop.set()


# ============================================================
# ANA PID DÖNGÜSÜ
# ============================================================
def main():
    """
    50Hz PID kontrol döngüsü.
    Gerçek donanımda: sysfs PWM yazılır.
    Simülasyonda: Motor modeli hesaplanır, konsola yazdırılır.
    """
    global left_speed, right_speed, left_pwm, right_pwm
    global last_heartbeat, state, prev_state, sp_l, sp_r, system_running

    # PWM kanallarını hazırla
    left_pwm_ready  = _setup_pwm_channel(0)   # VIM3: pwmchip0 → pwm_ao_a
    right_pwm_ready = _setup_pwm_channel(4)   # VIM3: pwmchip4 → pwm_f

    if not SIMULATION_MODE and not (left_pwm_ready and right_pwm_ready):
        print("[UYARI] PWM donanım kurulumu başarısız, simülasyon moduna geçildi.")

    # Simülasyon için mock sürücüler
    mock_driver_l = MagicMock(name="Sol_Driver_Sim")
    mock_driver_r = MagicMock(name="Sag_Driver_Sim")

    pid_l = PID2DOF(1.2, 0.4, 0.15)
    pid_r = PID2DOF(1.2, 0.4, 0.15)

    filt_l = filt_r = 0.0
    last_print_time = time.time()

    sim_status = "SİMÜLASYON" if SIMULATION_MODE else "DONANIM"
    print(f"\n[INFO] PID Kontrol döngüsü başladı [{sim_status} MODU]. Durdurmak için Ctrl+C.")

    try:
        while system_running:
            t0 = time.time()

            # --- Durum okuma (thread-safe) ---
            with _lock:
                current_state = state

            if current_state != prev_state:
                print(f"\n[SISTEM] Yeni Durum: {current_state.name}")
                if current_state == State.RUN:
                    pid_l.reset()
                    pid_r.reset()
                prev_state = current_state

            # --- Setpoint okuma (thread-safe) ---
            cur_sp_l, cur_sp_r = get_setpoints()
            # Legacy uyumluluk: modül seviyesi sp_l/sp_r da okunur
            cur_sp_l = cur_sp_l if cur_sp_l != 0.0 else sp_l
            cur_sp_r = cur_sp_r if cur_sp_r != 0.0 else sp_r

            with _lock:
                lp, rp = left_pwm, right_pwm

            # --- Simülasyon: Fizik Modeli ---
            if SIMULATION_MODE:
                if current_state != State.RUN:
                    left_speed  *= 0.8
                    right_speed *= 0.8
                else:
                    left_speed  += 0.1 * (motor_model(lp) - 0.03 * left_speed)
                    right_speed += 0.1 * (motor_model(rp) - 0.03 * right_speed)

                raw_l = left_speed  + random.uniform(-0.1, 0.1)
                raw_r = right_speed + random.uniform(-0.1, 0.1)
            else:
                # Gerçek donanım: encoder yoksa setpoint=ölçüm varsay
                raw_l = lp * kV
                raw_r = rp * kV

            filt_l = lowpass(filt_l, raw_l, MEAS_ALPHA)
            filt_r = lowpass(filt_r, raw_r, MEAS_ALPHA)

            # --- Kontrol Hesabı ---
            if current_state == State.RUN:
                with _lock:
                    _last_heartbeat = time.time()
                    last_heartbeat  = _last_heartbeat  # legacy

                ff_l = motor_model_inv(cur_sp_l)
                ff_r = motor_model_inv(cur_sp_r)

                u_l = pid_l.compute(cur_sp_l, filt_l, LOOP_DT, MIN_PWM, MAX_PWM, ff_l)
                u_r = pid_r.compute(cur_sp_r, filt_r, LOOP_DT, MIN_PWM, MAX_PWM, ff_r)

                new_lp = slew_limit(lp, u_l, PWM_SLEW_RATE, LOOP_DT)
                new_rp = slew_limit(rp, u_r, PWM_SLEW_RATE, LOOP_DT)

            elif current_state == State.LASER:
                # Laser aktifken motorlar %30'a düşer (titreşim azaltma)
                new_lp = slew_limit(lp, cur_sp_l * 0.3, PWM_SLEW_RATE, LOOP_DT)
                new_rp = slew_limit(rp, cur_sp_r * 0.3, PWM_SLEW_RATE, LOOP_DT)

            else:
                # ESTOP veya TIMEOUT: sıfıra in
                new_lp = slew_limit(lp, 0.0, PWM_SLEW_RATE, LOOP_DT)
                new_rp = slew_limit(rp, 0.0, PWM_SLEW_RATE, LOOP_DT)

            # --- PWM Güncelleme ---
            with _lock:
                left_pwm, right_pwm = new_lp, new_rp

            if SIMULATION_MODE:
                mock_driver_l.write(left_pwm)
                mock_driver_r.write(right_pwm)
            else:
                write_motor_pwm(VIM3_PWM_LEFT_PATH,  left_pwm)
                write_motor_pwm(VIM3_PWM_RIGHT_PATH, right_pwm)

            # --- Konsol Çıktısı (0.5s'de bir) ---
            if time.time() - last_print_time > 0.5:
                mode = "SIM" if SIMULATION_MODE else "HW"
                print(f"[{mode}] L_HIZ:{filt_l:5.1f} L_PWM:{left_pwm:4.1f} || "
                      f"R_HIZ:{filt_r:5.1f} R_PWM:{right_pwm:4.1f} | {current_state.name}")
                last_print_time = time.time()

            # --- Zamanlama ---
            elapsed = time.time() - t0
            time.sleep(max(0, LOOP_DT - elapsed))

    except KeyboardInterrupt:
        print("\n[INFO] PID döngüsü kapatılıyor...")
        emergency_stop.set()
        system_running = False
        # PWM'i güvenle sıfırla
        if not SIMULATION_MODE:
            write_motor_pwm(VIM3_PWM_LEFT_PATH,  0)
            write_motor_pwm(VIM3_PWM_RIGHT_PATH, 0)
        time.sleep(0.5)
        print("[OK] Sistem güvenli duruma alındı.")


if __name__ == "__main__":
    t_safety = threading.Thread(target=safety_thread, daemon=True)
    t_safety.start()
    main()