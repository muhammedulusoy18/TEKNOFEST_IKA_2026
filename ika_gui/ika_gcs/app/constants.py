"""
constants.py — IKA GCS Sabit Değerleri
"""

# ============================================================
# BAĞLANTI
# ============================================================
DEFAULT_TCP_HOST    = "192.168.1.100"   # Khadas VIM3 IP (aynı ağda)
DEFAULT_TCP_PORT    = 9000              # TCP komut portu
DEFAULT_UDP_PORT    = 9001             # UDP telemetri portu

# ============================================================
# TELEMETRY SINIR DEĞERLERİ
# ============================================================
BATTERY_LOW_PCT     = 20    # Düşük batarya uyarısı (%)
BATTERY_CRITICAL_PCT= 10    # Kritik batarya (%)
TEMP_WARN_C         = 75.0  # Yüksek sıcaklık uyarısı (°C)
TEMP_CRITICAL_C     = 90.0  # Kritik sıcaklık (°C)
CPU_WARN_PCT        = 90    # Yüksek CPU yük uyarısı (%)
ROLL_WARN_DEG       = 35.0  # Eğim uyarı açısı (°)
ROLL_STOP_DEG       = 45.0  # Eğim kritik açısı (°)

# ============================================================
# UYGULAMA
# ============================================================
APP_NAME            = "IKA GCS"
APP_VERSION         = "1.0.0"
WINDOW_TITLE        = f"IKA GCS v{APP_VERSION} — TEKNOFEST 2026"
WINDOW_MIN_W        = 1280
WINDOW_MIN_H        = 720
