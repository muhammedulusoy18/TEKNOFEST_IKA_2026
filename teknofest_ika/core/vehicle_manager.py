import cv2
import threading
import teknofest_ika.modules.pid_thread as pid_ctrl
from teknofest_ika.modules.perception import PerceptionUnit
from teknofest_ika.utils.camera_handler import CameraHandler


class VehicleManager:
    def __init__(self):
        self.perception = PerceptionUnit()
        self.camera = CameraHandler(camera_source=0)
        self.is_running = True
        self.system_state = "DRIVING"
        self.manual_override = False

        print("[SİSTEM] Güvenlik kalkanları ve PID motor denetleyicisi aktif ediliyor...")
        threading.Thread(target=pid_ctrl.safety_thread, daemon=True).start()
        threading.Thread(target=pid_ctrl.main, daemon=True).start()

    def run(self):
        print("==================================================")
        print("  TEKNOFEST 2026 IKA - ARAÇ YÖNETİM SİSTEMİ")
        print("==================================================")
        print("KOMUTLAR: W(İleri) | X(Geri) | A(Sol) | D(Sağ) | Space(Otonomi)")

        try:
            while self.is_running:
                frame = self.camera.get_frame()
                if frame is None: continue

                # 1. Otonomi Kararı (Arka planda hep çalışır)
                decision, output_frame, new_state = self.perception.process_frame(frame, self.system_state)
                self.system_state = new_state

                # 2. Klavye Okuma
                key = cv2.waitKey(1) & 0xFF

                if key != 255:
                    self._handle_keyboard(key)
                else:
                    # EĞER TUŞA BASILMIYORSA VE MANUEL MODDAYSAK FREN YAP!
                    if self.manual_override:
                        pid_ctrl.sp_l, pid_ctrl.sp_r = 0.0, 0.0
                # Tuşa basıldıysa işleme al

                # 3. Motor Kontrolü
                # Eğer manuel override varsa otonomi komutlarını yoksay
                if not self.manual_override:
                    self._apply_kinematics(decision)

                cv2.imshow("TEKNOFEST IKA - Muhammed Ulusoy", output_frame)

        except Exception as e:
            print(f"[HATA] Beklenmedik hata: {e}")
        finally:
            self._cleanup()

    def _handle_keyboard(self, key):
        """Doğrudan 119 gibi ASCII kodları üzerinden kontrol."""

        # SÜREKLİ TAKİP İÇİN (Çalıştığını buradan teyit et kanka)
        if key != 255:
            print(f">>> TUS ALGILANDI: {key}")

        # W - İLERİ (119: w, 87: W)
        if key == 119 or key == 87:
            self.manual_override = True
            pid_ctrl.sp_l, pid_ctrl.sp_r = 50.0, 50.0
            print(">>> MANUEL KOMUT: İLERİ GİDİYOR")

        # X - GERİ (120: x, 88: X)
        elif key == 120 or key == 88:
            self.manual_override = True
            pid_ctrl.sp_l, pid_ctrl.sp_r = -50.0, -50.0
            print(">>> MANUEL KOMUT: GERİ GİDİYOR")

        # A - SOL TANK (97: a, 65: A)
        elif key == 97 or key == 65:
            self.manual_override = True
            pid_ctrl.sp_l, pid_ctrl.sp_r = -50.0, 50.0
            print(">>> MANUEL KOMUT: SOLA DÖNÜŞ")

        # D - SAĞ TANK (100: d, 68: D)
        elif key == 100 or key == 68:
            self.manual_override = True
            pid_ctrl.sp_l, pid_ctrl.sp_r = 50.0, -50.0
            print(">>> MANUEL KOMUT: SAĞA DÖNÜŞ")

        # SPACE - OTONOMİYE DÖN (32: Space)
        elif key == 32:
            self.manual_override = False
            pid_ctrl.sp_l, pid_ctrl.sp_r = 0.0, 0.0
            print(">>> MOD: OTONOMİYE DEVREDİLDİ")

        # MOD DEĞİŞTİRME (56: 8, 57: 9)
        elif key == 56:  # 8
            self.system_state = "AIMING"
            print(">>> DURUM: NISAN ALMA")
        elif key == 57:  # 9
            self.system_state = "DRIVING"
            print(">>> DURUM: SURUS")

        # Q - ÇIKIŞ (113: q, 81: Q)
        elif key == 113 or key == 81:
            self.is_running = False

    def _apply_kinematics(self, cmd_dict):
        # Otonomi komutlarını motor setpointlerine çevirir
        steer = cmd_dict.get("steer", 0)
        throttle = cmd_dict.get("throttle", 0)
        pid_ctrl.sp_l = max(-70.0, min(70.0, throttle + steer))
        pid_ctrl.sp_r = max(-70.0, min(70.0, throttle - steer))

    def _cleanup(self):
        print("[SİSTEM] Kapatılıyor...")
        pid_ctrl.emergency_stop.set()
        self.camera.stop()
        cv2.destroyAllWindows()