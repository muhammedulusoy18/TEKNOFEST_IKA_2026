import sys
from unittest.mock import MagicMock

#  WINDOWS TEST
try:
    import gpiod
except ImportError:
    print("[SİSTEM] 'gpiod' bulunamadı! Windows Simülasyon Modu aktif.")
    mock_gpiod = MagicMock()
    sys.modules["gpiod"] = mock_gpiod
    mock_gpiod.line_request.return_value = MagicMock()

from core.vehicle_manager import VehicleManager

if __name__ == "__main__":
    try:
        arac = VehicleManager()
        arac.run()
    except Exception as e:
        print(f"[KRİTİK HATA] Sistem başlatılamadı: {e}")