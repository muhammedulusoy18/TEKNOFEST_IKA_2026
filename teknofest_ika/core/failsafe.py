"""
failsafe.py — IKA 2026 Güvenlik Kalkanı
=========================================
Araç durumu izleme, acil durdurma ve yeniden başlatma mantığı.
Bu modül hem ROS-dışı hem ROS modunda kullanılabilir.
"""

import time
import threading
from enum import Enum


# ============================================================
# GÜVENLİK SINIR DEĞERLERİ
# ============================================================
ROLL_WARN_DEG   = 35.0   # Bu açıyı geçince uyarı
ROLL_STOP_DEG   = 45.0   # Bu açıyı geçince acil dur
PITCH_WARN_DEG  = 35.0
PITCH_STOP_DEG  = 45.0
COMM_TIMEOUT_S  = 2.0    # İletişim kesilince dur (saniye)
BATTERY_MIN_PCT = 10     # Kritik batarya seviyesi (%)
CPU_WARN_PCT    = 90     # Yüksek CPU uyarısı (%)
TEMP_WARN_C     = 75     # Yüksek Khadas sıcaklığı (°C)
TEMP_STOP_C     = 90     # Kritik Khadas sıcaklığı — işlemci koruması (°C)


class FailsafeReason(Enum):
    NONE           = "normal"
    ROLL_LIMIT     = "yüksek_roll_açısı"
    PITCH_LIMIT    = "yüksek_pitch_açısı"
    COMM_LOST      = "haberleşme_kesildi"
    BATTERY_LOW    = "batarya_kritik"
    TEMP_CRITICAL  = "sıcaklık_kritik"
    MANUAL_ESTOP   = "manuel_acil_dur"
    WATCHDOG       = "watchdog_timeout"


class FailsafeState(Enum):
    NORMAL   = 0
    WARNING  = 1
    FAILSAFE = 2
    LOCKED   = 3   # Manuel müdahale gerektirir


class FailsafeGuard:
    """
    Araç güvenlik izleyicisi.
    Thread-safe olarak her yerden sorgulabilir.
    
    Kullanım:
        guard = FailsafeGuard()
        guard.update_imu(roll=5.0, pitch=3.0)
        guard.heartbeat()
        if guard.is_safe():
            # motoru çalıştır
    """

    def __init__(self):
        self._lock  = threading.Lock()
        self._state = FailsafeState.NORMAL
        self._reason: FailsafeReason = FailsafeReason.NONE

        self._last_heartbeat = time.monotonic()
        self._roll  = 0.0
        self._pitch = 0.0
        self._battery_pct = 100
        self._temp_c = 0.0

        # Geri çağrı listesi: durum değişince çağrılır
        self._callbacks: list = []

        print("[FAILSAFE] Güvenlik kalkanı aktif.")

    # ── Güncelleyiciler ──────────────────────────────────────

    def update_imu(self, roll: float, pitch: float):
        with self._lock:
            self._roll  = roll
            self._pitch = pitch
        self._evaluate()

    def update_battery(self, pct: int):
        with self._lock:
            self._battery_pct = pct
        self._evaluate()

    def update_temp(self, temp_c: float):
        with self._lock:
            self._temp_c = temp_c
        self._evaluate()

    def heartbeat(self):
        """Her başarılı haberleşme paketinde çağrılmalı."""
        with self._lock:
            self._last_heartbeat = time.monotonic()

    def trigger_estop(self):
        """Fiziksel E-Stop butonuna basıldığında veya GUI'den tetiklendiğinde."""
        self._set_state(FailsafeState.LOCKED, FailsafeReason.MANUAL_ESTOP)

    def reset(self) -> bool:
        """
        LOCKED → NORMAL geçişi (sadece araç güvenli pozisyondaysa).
        Otomatik reset için tüm koşulların sağlanması gerekir.
        """
        with self._lock:
            roll  = self._roll
            pitch = self._pitch
            reason = self._reason

        if reason == FailsafeReason.MANUAL_ESTOP:
            # Manuel E-Stop'tan çıkmak için fiziksel onay gerekir
            # Bu kararı GUI/operatör verir, burada False döndür
            print("[FAILSAFE] Manuel E-Stop sıfırlama fiziksel onay gerektirir.")
            return False

        if abs(roll) < ROLL_WARN_DEG and abs(pitch) < PITCH_WARN_DEG:
            with self._lock:
                self._state  = FailsafeState.NORMAL
                self._reason = FailsafeReason.NONE
            print("[FAILSAFE] Güvenlik durumu NORMAL'e döndü.")
            return True

        print(f"[FAILSAFE] Reset reddedildi: Roll={roll:.1f}° Pitch={pitch:.1f}°")
        return False

    def force_reset(self):
        """Operatör tarafından zorla sıfırlama (LOCKED dahil)."""
        with self._lock:
            self._state  = FailsafeState.NORMAL
            self._reason = FailsafeReason.NONE
            self._last_heartbeat = time.monotonic()
        print("[FAILSAFE] ⚠ ZORLA sıfırlama yapıldı.")

    # ── Sorgulayıcılar ──────────────────────────────────────

    def is_safe(self) -> bool:
        """Motorların çalışmasına izin var mı?"""
        with self._lock:
            return self._state == FailsafeState.NORMAL

    def is_warning(self) -> bool:
        with self._lock:
            return self._state == FailsafeState.WARNING

    def get_state(self) -> FailsafeState:
        with self._lock:
            return self._state

    def get_reason(self) -> str:
        with self._lock:
            return self._reason.value

    def get_summary(self) -> dict:
        with self._lock:
            return {
                "state":   self._state.name,
                "reason":  self._reason.value,
                "roll":    round(self._roll, 1),
                "pitch":   round(self._pitch, 1),
                "battery": self._battery_pct,
                "temp":    round(self._temp_c, 1),
            }

    # ── Geri çağrılar ───────────────────────────────────────

    def on_state_change(self, callback):
        """Durum değişince çağrılacak fonksiyonu kaydet: fn(state, reason)"""
        self._callbacks.append(callback)

    # ── İç Mantık ──────────────────────────────────────────

    def _evaluate(self):
        """Tüm sensör verilerini kontrol et, durumu güncelle."""
        with self._lock:
            roll    = self._roll
            pitch   = self._pitch
            battery = self._battery_pct
            temp    = self._temp_c
            elapsed = time.monotonic() - self._last_heartbeat
            old_state = self._state

        # LOCKED durumu sadece force_reset veya reset() ile çıkılabilir
        if old_state == FailsafeState.LOCKED:
            return

        new_state  = FailsafeState.NORMAL
        new_reason = FailsafeReason.NONE

        # Öncelik sırası: Kritik → Uyarı
        if abs(roll) >= ROLL_STOP_DEG:
            new_state, new_reason = FailsafeState.FAILSAFE, FailsafeReason.ROLL_LIMIT
        elif abs(pitch) >= PITCH_STOP_DEG:
            new_state, new_reason = FailsafeState.FAILSAFE, FailsafeReason.PITCH_LIMIT
        elif elapsed > COMM_TIMEOUT_S:
            new_state, new_reason = FailsafeState.FAILSAFE, FailsafeReason.COMM_LOST
        elif temp >= TEMP_STOP_C:
            new_state, new_reason = FailsafeState.FAILSAFE, FailsafeReason.TEMP_CRITICAL
        elif battery <= BATTERY_MIN_PCT:
            new_state, new_reason = FailsafeState.FAILSAFE, FailsafeReason.BATTERY_LOW
        elif abs(roll) >= ROLL_WARN_DEG or abs(pitch) >= PITCH_WARN_DEG:
            new_state, new_reason = FailsafeState.WARNING, FailsafeReason.ROLL_LIMIT
        elif temp >= TEMP_WARN_C:
            new_state, new_reason = FailsafeState.WARNING, FailsafeReason.TEMP_CRITICAL

        with self._lock:
            if new_state != old_state:
                self._state  = new_state
                self._reason = new_reason
                print(f"[FAILSAFE] Durum: {old_state.name} → {new_state.name} ({new_reason.value})")
                for cb in self._callbacks:
                    try:
                        cb(new_state, new_reason)
                    except Exception as e:
                        print(f"[FAILSAFE] Callback hatası: {e}")


# ============================================================
# GLOBAL ÖRNEK (Opsiyonel — modül seviyesi singleton)
# ============================================================
_guard: FailsafeGuard | None = None

def get_guard() -> FailsafeGuard:
    """Global FailsafeGuard singleton'ı döndürür."""
    global _guard
    if _guard is None:
        _guard = FailsafeGuard()
    return _guard
