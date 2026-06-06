import sys
import os
import traceback
from unittest.mock import MagicMock

# 1. Yol Tanımlamaları: Python'ın yan klasörleri (core, modules) görmesini sağlar
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

# 2. Donanım Simülasyonu: gpiod yoksa sahtesini (Mock) oluşturur
try:
    import gpiod
except ImportError:	
    print("[SİSTEM] 'gpiod' bulunamadı! Simülasyon Modu aktif.")
    mock_gpiod = MagicMock()
    sys.modules["gpiod"] = mock_gpiod
    mock_gpiod.line_request.return_value = MagicMock()

# 3. Kendi Modüllerimizi İçe Aktarma
# Not: Bunlar sys.path ayarından SONRA gelmelidir
try:
    from teknofest_ika.core.vehicle_manager import VehicleManager
except ImportError as e:
    print(f"[HATA] Modüller içe aktarılamadı: {e}")
    print("[İPUCU] Paketi ROS 2 workspace içinde 'colcon build' ile derleyin veya")
    print("        'python -m teknofest_ika.run_ika' şeklinde çalıştırın.")
    sys.exit(1)

def main():
    print("--- TEKNOFEST IKA 2026 SİSTEMİ BAŞLATILIYOR ---")
    
    try:
        # Araç yönetim merkezini kur (PerceptionUnit + CameraHandler dahili olarak oluşturulur)
        arac = VehicleManager()
        
        print("[BİLGİ] Araç döngüsü başlatılıyor...")
        
        # 4. Aracı Çalıştır (kendi döngüsü var, bloklar)
        arac.run()

    except KeyboardInterrupt:
        print("\n[BİLGİ] Kullanıcı komutuyla sistem kapatılıyor...")
    except Exception as e:
        print(f"[KRİTİK HATA] Çalışma zamanı hatası: {e}")
        traceback.print_exc()

if __name__ == '__main__':
    main()
