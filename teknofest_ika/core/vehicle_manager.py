"""
vehicle_manager.py — IKA 2026 Araç Yönetim Merkezi
=====================================================
Tüm alt sistemleri koordine eden ana orkestratör.
- Kamera → Algılama → Karar → Motor zinciri
- FailsafeGuard entegrasyonu (IMU bazlı acil durdurma)
- Manuel override ile otonom mod arasında geçiş
- Khadas VIM3 için optimize edilmiş thread mimarisi
"""

import cv2
import threading
import teknofest_ika.modules.pid_thread as pid_ctrl
from teknofest_ika.modules.perception import PerceptionUnit
from teknofest_ika.utils.camera_handler import CameraHandler
from teknofest_ika.core.failsafe import FailsafeGuard


class VehicleManager:
    """
    Ana araç kontrol sınıfı.

    Durum Makinesi:
        DRIVING         → Otonom sürüş (şerit takibi + engel kaçınma)
        AIMING          → Hedefe nişan alma
        SHOOTING        → Lazer atışı
        SLIDING_OBSTACLE→ Hareketli engel kaçınma
    """

    def __init__(self):
        print("[VehicleManager] Alt sistemler başlatılıyor...")

        # Algılama ve Kamera
        self.perception = PerceptionUnit()
        self.camera     = CameraHandler(camera_source=0)

        # Güvenlik Kalkanı
        self.guard = FailsafeGuard()
        self.guard.on_state_change(self._on_failsafe_change)

        # Durum
        self.is_running      = True
        self.system_state    = "DRIVING"
        self.manual_override = False

        # PID + Güvenlik thread'leri başlat
        print("[VehicleManager] PID motor denetleyicisi ve güvenlik kalkanı aktif ediliyor...")
        threading.Thread(target=pid_ctrl.safety_thread, daemon=True, name="safety_thread").start()
        threading.Thread(target=pid_ctrl.main,          daemon=True, name="pid_thread").start()

        print("[VehicleManager] ✅ Tüm sistemler hazır.")

    def run(self):
        print("=" * 54)
        print("  TEKNOFEST 2026 IKA — ARAÇ YÖNETİM SİSTEMİ")
        print("=" * 54)
        print("KOMUTLAR: W(İleri) | X(Geri) | A(Sol) | D(Sağ)")
        print("          Space(Otonomi) | 8(AIMING) | 9(DRIVING) | Q(Çıkış)")

        try:
            while self.is_running:
                frame = self.camera.get_frame()
                if frame is None:
                    continue

                # 1. Otonom Algılama
                decision, output_frame, new_state = self.perception.process_frame(
                    frame, self.system_state
                )
                self.system_state = new_state

                # 2. Failsafe Kontrolü
                if not self.guard.is_safe():
                    # Acil durumda motoru durdur, otonom moda geçme
                    reason = self.guard.get_reason()
                    pid_ctrl.set_setpoints(0.0, 0.0)
                    cv2.putText(output_frame,
                                f"FAILSAFE: {reason.upper()}",
                                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.imshow("TEKNOFEST IKA 2026", output_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.is_running = False
                    continue

                # 3. Klavye Girişi
                key = cv2.waitKey(1) & 0xFF

                if key != 255:
                    self._handle_keyboard(key)
                else:
                    # Tuşa basılmıyorsa ve manuel moddaysa fren
                    if self.manual_override:
                        pid_ctrl.set_setpoints(0.0, 0.0)

                # 4. Motor Kontrolü
                if not self.manual_override:
                    self._apply_kinematics(decision)

                # 5. Heartbeat güncelle (timeout koruması)
                self.guard.heartbeat()
                pid_ctrl.update_heartbeat()

                cv2.imshow("TEKNOFEST IKA 2026", output_frame)

        except Exception as e:
            import traceback
            print(f"[HATA] VehicleManager çalışma zamanı hatası: {e}")
            traceback.print_exc()
        finally:
            self._cleanup()

    def _handle_keyboard(self, key: int):
        """ASCII tuş kodlarına göre manuel kontrol."""

        # W / w — İleri (119/87)
        if key in (119, 87):
            self.manual_override = True
            pid_ctrl.set_setpoints(50.0, 50.0)
            print(">>> MANUEL: İLERİ")

        # X / x — Geri (120/88)
        elif key in (120, 88):
            self.manual_override = True
            pid_ctrl.set_setpoints(-50.0, -50.0)
            print(">>> MANUEL: GERİ")

        # A / a — Sol Tank Dönüşü (97/65)
        elif key in (97, 65):
            self.manual_override = True
            pid_ctrl.set_setpoints(-50.0, 50.0)
            print(">>> MANUEL: SOL DÖNÜŞ")

        # D / d — Sağ Tank Dönüşü (100/68)
        elif key in (100, 68):
            self.manual_override = True
            pid_ctrl.set_setpoints(50.0, -50.0)
            print(">>> MANUEL: SAĞ DÖNÜŞ")

        # Space — Otonomi (32)
        elif key == 32:
            self.manual_override = False
            pid_ctrl.set_setpoints(0.0, 0.0)
            print(">>> MOD: OTONOMİ")

        # 8 — Nişan Alma modu
        elif key == 56:
            self.system_state = "AIMING"
            print(">>> DURUM: NİŞAN ALMA")

        # 9 — Sürüş moduna dön
        elif key == 57:
            self.system_state = "DRIVING"
            print(">>> DURUM: SÜRÜŞ")

        # R — Failsafe sıfırla
        elif key in (114, 82):
            if self.guard.reset():
                print(">>> FAILSAFE: SIFIRLANDI")
            else:
                print(">>> FAILSAFE: Sıfırlama reddedildi (koşullar sağlanmadı)")

        # Q / q — Çıkış (113/81)
        elif key in (113, 81):
            self.is_running = False

    def _apply_kinematics(self, cmd_dict: dict):
        """
        Otonom karar komutunu sol/sağ motor setpointlerine çevirir.
        Diferansiyel sürüş kinematik dönüşümü.
        """
        steer    = cmd_dict.get("steer", 0)
        throttle = cmd_dict.get("throttle", 0)
        sp_l = float(max(-70.0, min(70.0, throttle + steer)))
        sp_r = float(max(-70.0, min(70.0, throttle - steer)))
        pid_ctrl.set_setpoints(sp_l, sp_r)

    def _on_failsafe_change(self, state, reason):
        """FailsafeGuard durum değişimi geri çağrısı."""
        from teknofest_ika.core.failsafe import FailsafeState
        if state in (FailsafeState.FAILSAFE, FailsafeState.LOCKED):
            # Acil durdurma
            pid_ctrl.emergency_stop.set()
            pid_ctrl.set_setpoints(0.0, 0.0)
            print(f"[VehicleManager] ⛔ FAILSAFE TETİKLENDİ: {reason.value}")
        elif state == FailsafeState.NORMAL:
            # Güvenli duruma döndü — E-Stop event'ini temizle
            pid_ctrl.emergency_stop.clear()
            print("[VehicleManager] ✅ Failsafe kaldırıldı.")

    def _cleanup(self):
        print("[VehicleManager] Sistem kapatılıyor...")
        pid_ctrl.emergency_stop.set()
        pid_ctrl.set_setpoints(0.0, 0.0)
        pid_ctrl.system_running = False
        self.camera.stop()
        cv2.destroyAllWindows()
        print("[VehicleManager] ✅ Tüm kaynaklar serbest bırakıldı.")