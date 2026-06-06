"""
perception.py — IKA 2026 Algılama Birimi
==========================================
Durum makinesi tabanlı görüntü işleme:
  DRIVING         → Şerit takibi + sabit engel kaçınma
  AIMING          → Dairesel hedef tespiti ve nişan alma
  SHOOTING        → Lazer atışı sayacı
  SLIDING_OBSTACLE→ MOG2 hareketli engel kaçınma

Khadas VIM3 NPU'su için notlar:
  - Ağır OpenCV işlemleri (MOG2, HoughCircles) her karede değil,
    belirli kare aralıklarında çalıştırılır (performans optimizasyonu).
  - Tesseract varsa her 10 karede bir çalışır.
"""

import cv2
import numpy as np
import time
import os

# ============================================================
# TESSERACT OCR YAPILLANDIRMASI
# Khadas VIM3 Linux: sudo apt install tesseract-ocr
# ============================================================
try:
    import pytesseract
    if os.name == 'nt':
        # Windows geliştirme ortamı
        pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'
    OCR_AVAILABLE = True
except ImportError:
    OCR_AVAILABLE = False


class PerceptionUnit:
    """
    Kamera görüntüsünü işleyen ve sürüş kararları üreten algılama birimi.
    
    Çıktı formatı: {"throttle": float, "steer": float}
      throttle: -70 ~ 70 (negatif = geri)
      steer:    -50 ~ 50 (negatif = sol, pozitif = sağ)
    """

    def __init__(self):
        # ── Durum ──────────────────────────────────────────
        self.laser_active  = False
        self.laser_on_time = 0
        self.frame_count   = 0
        self.target_number = "8"   # OCR'ın arayacağı hedef sayı

        # ── Hareketli Engel (MOG2) ─────────────────────────
        # history=100, varThreshold=50 → VIM3'de dengeli performans
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=100, varThreshold=50, detectShadows=False
        )

        # ── Renk Aralıkları (HSV) ──────────────────────────
        # Turuncu Koni / Sabit Engel
        self.lower_orange = np.array([5,  130, 100])
        self.upper_orange = np.array([25, 255, 255])

        # Sarı Şerit (asfalt yolu için tipik)
        self.lower_yellow = np.array([20, 100,  80])
        self.upper_yellow = np.array([35, 255, 255])

        # Beyaz Şerit
        self.lower_white  = np.array([0,   0, 200])
        self.upper_white  = np.array([180, 30, 255])

        # ── Şerit Takibi: Son geçerli değer ───────────────
        self._last_lane_steer = 0
        self._lane_lost_count = 0
        LANE_LOST_MAX = 10   # Bu kadar karede şerit bulunamazsa throttle kesilir

        self.LANE_LOST_MAX = LANE_LOST_MAX
        print(f"[Perception] Başlatıldı. OCR: {'AKTIF' if OCR_AVAILABLE else 'KAPALI'}")

    # ============================================================
    # ANA İŞLEME DÖNGÜSÜ
    # ============================================================
    def process_frame(self, frame, current_state: str):
        """
        Ana giriş noktası. VehicleManager her karede çağırır.
        
        Dönüş: (decision_dict, annotated_frame, new_state)
        """
        if frame is None:
            return {"throttle": 0, "steer": 0}, frame, current_state

        self.frame_count += 1
        frame        = cv2.resize(frame, (640, 480))
        output_frame = frame.copy()

        # ── MOG2: Hareketli Engel Algılaması ──────────────
        # Her karede çalışması zorunlu (arka plan modeli öğrenir)
        gray_frame   = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_gray = cv2.GaussianBlur(gray_frame, (5, 5), 0)
        fg_mask      = self.bg_subtractor.apply(blurred_gray)

        if current_state in ("DRIVING", "SLIDING_OBSTACLE"):
            contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            moving_obstacle = False
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 3000:
                    moving_obstacle = True
                    current_state = "SLIDING_OBSTACLE"
            if not moving_obstacle and current_state == "SLIDING_OBSTACLE":
                current_state = "DRIVING"

        # ── Durum Makinesi Yönlendirmesi ──────────────────
        if current_state == "DRIVING":
            decision, output_frame, new_state = self._autonomous_drive(frame, output_frame)

        elif current_state == "AIMING":
            decision, output_frame, new_state = self._aim_target(frame, output_frame)

        elif current_state == "SHOOTING":
            decision, output_frame, new_state = self._handle_shooting(frame, output_frame)

        elif current_state == "SLIDING_OBSTACLE":
            decision, output_frame = self._avoid_sliding_obstacle(fg_mask, output_frame)
            new_state = "SLIDING_OBSTACLE"

        else:
            decision  = {"throttle": 0, "steer": 0}
            new_state = "DRIVING"

        self._draw_hud(output_frame, new_state, decision)
        return decision, output_frame, new_state

    # ============================================================
    # OTONOM SÜRÜŞ (DRIVING)
    # ============================================================
    def _autonomous_drive(self, frame, draw_frame):
        """
        1. OCR ile hedef sayı tespiti (her 10 karede)
        2. Şerit takibi (sarı/beyaz çizgi merkezleme)
        3. Turuncu koni/engel kaçınma
        Öncelik: Hedef Tespiti > Koni Kaçınma > Şerit Takibi
        """
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        # 1. HEDEF SAYISI TESPİTİ (Dairesel tabela OCR)
        if self.frame_count % 10 == 0 and OCR_AVAILABLE:
            circles = cv2.HoughCircles(
                blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=150,
                param1=50, param2=40, minRadius=30, maxRadius=150
            )
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    roi = gray[
                        max(0, i[1] - i[2]): min(480, i[1] + i[2]),
                        max(0, i[0] - i[2]): min(640, i[0] + i[2])
                    ]
                    if roi.size > 0:
                        roi_t = cv2.threshold(
                            roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
                        )[1]
                        text = pytesseract.image_to_string(
                            roi_t, config='--psm 10 -c tessedit_char_whitelist=0123456789'
                        ).strip()
                        if self.target_number in text:
                            cv2.circle(draw_frame, (i[0], i[1]), i[2], (0, 255, 0), 3)
                            return {"throttle": 0, "steer": 0}, draw_frame, "AIMING"

        # 2. SABİT ENGEL (Turuncu Koni) TESPİTİ
        hsv      = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_org = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        contours, _ = cv2.findContours(mask_org, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cone_steer = 0
        for cnt in contours:
            if cv2.contourArea(cnt) > 800:
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2
                cv2.rectangle(draw_frame, (x, y), (x + w, y + h), (0, 165, 255), 2)
                cv2.putText(draw_frame, "KONİ", (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
                # Koni sağda → sola dön, solda → sağa dön
                cone_steer = -40 if cx > 320 else 40

        if cone_steer != 0:
            self._lane_lost_count = 0
            return {"throttle": 25, "steer": cone_steer}, draw_frame, "DRIVING"

        # 3. ŞERİT TAKİBİ (Sarı + Beyaz Çizgi)
        lane_steer = self._detect_lane(frame, draw_frame)
        return {"throttle": 30, "steer": lane_steer}, draw_frame, "DRIVING"

    def _detect_lane(self, frame, draw_frame) -> int:
        """
        Sarı ve beyaz şerit maskeleri kullanarak yol merkezini hesaplar.
        Görüntünün alt yarısına (ROI) odaklanır.
        Dönüş: steer değeri (-50 ~ 50)
        """
        h, w = frame.shape[:2]
        roi_top = h // 2   # Görüntünün sadece alt yarısı

        # ROI maskesi
        roi = frame[roi_top:h, 0:w]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Sarı + Beyaz şerit maskesi
        mask_y = cv2.inRange(hsv_roi, self.lower_yellow, self.upper_yellow)
        mask_w = cv2.inRange(hsv_roi, self.lower_white,  self.upper_white)
        lane_mask = cv2.bitwise_or(mask_y, mask_w)

        # Morfolojik temizleme
        kernel    = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN,  kernel)

        # Şerit noktalarının ağırlık merkezi (centroid)
        moments = cv2.moments(lane_mask)
        if moments["m00"] > 500:   # Yeterli şerit pikseli var
            cx_lane = int(moments["m10"] / moments["m00"])
            cy_lane = int(moments["m01"] / moments["m00"]) + roi_top

            # Görüntü merkezine göre hata
            center_x = w // 2
            error    = cx_lane - center_x

            # Şerit merkezi görselleştir
            cv2.circle(draw_frame, (cx_lane, cy_lane), 8, (0, 255, 255), -1)
            cv2.line(draw_frame, (center_x, h - 20), (cx_lane, cy_lane), (0, 255, 255), 2)
            cv2.putText(draw_frame, f"Serit err:{error:+d}", (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # P kontrol (oran: 320 piksel → 50 steer)
            steer = int(np.clip(error * 0.15, -50, 50))
            self._last_lane_steer = steer
            self._lane_lost_count = 0
            return steer
        else:
            # Şerit bulunamadı
            self._lane_lost_count += 1
            cv2.putText(draw_frame, "SErit YOK!", (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            # Son bilinen yönde devam et (max 10 kare)
            if self._lane_lost_count <= self.LANE_LOST_MAX:
                return self._last_lane_steer
            else:
                return 0   # Şerit çok uzun süredir kayıp → düz git

    # ============================================================
    # NİŞAN ALMA (AIMING)
    # ============================================================
    def _aim_target(self, frame, draw_frame):
        """
        Dairesel hedefi bulup merkezleme.
        Hata < 20 piksel olduğunda SHOOTING'e geç.
        """
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 1.5)

        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=100,
            param1=50, param2=25, minRadius=15, maxRadius=120
        )

        steer = 0
        if circles is not None:
            i       = np.uint16(np.around(circles))[0, 0]
            error_x = int(i[0]) - 320
            cv2.circle(draw_frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.circle(draw_frame, (i[0], i[1]), 5,    (0, 255, 0), -1)
            cv2.putText(draw_frame, f"Hata: {error_x:+d}px", (i[0] + 10, i[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if abs(error_x) < 20:
                # Hedef merkezlendi → atış başlat
                self.laser_on_time = time.time()
                return {"throttle": 0, "steer": 0}, draw_frame, "SHOOTING"
            else:
                # Oransal düzeltme
                steer = int(np.clip(error_x * 0.4, -50, 50))

        else:
            cv2.putText(draw_frame, "HEDEF ARANIYOR...", (200, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)

        return {"throttle": 0, "steer": steer}, draw_frame, "AIMING"

    # ============================================================
    # ATEŞ (SHOOTING)
    # ============================================================
    def _handle_shooting(self, frame, draw_frame):
        """
        2 saniyelik lazer atışı. Süre dolunca DRIVING'e döner.
        Gerçek lazer çıkışı: sensor_node veya motor_node üzerinden GPIO.
        """
        self.laser_active = True
        elapsed = time.time() - self.laser_on_time

        if elapsed > 2.0:
            self.laser_active = False
            return {"throttle": 0, "steer": 0}, draw_frame, "DRIVING"

        # Atış görsel efekti
        remaining = 2.0 - elapsed
        cv2.putText(draw_frame, f"!!! ATES !!! {remaining:.1f}s",
                    (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)
        # Kırmızı çerçeve
        cv2.rectangle(draw_frame, (5, 5), (635, 475), (0, 0, 255), 4)
        return {"throttle": 0, "steer": 0}, draw_frame, "SHOOTING"

    # ============================================================
    # HAREKETLİ ENGEL KAÇINMA (SLIDING_OBSTACLE)
    # ============================================================
    def _avoid_sliding_obstacle(self, fg_mask, draw_frame):
        """
        MOG2 ön plan maskesindeki en büyük hareketi takip eder.
        Engel sağda → sola dön, solda → sağa dön.
        """
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        steer = 0

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 3000:
                x, y, w, h = cv2.boundingRect(largest)
                cx_obj = x + w // 2
                cv2.rectangle(draw_frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                cv2.putText(draw_frame, "HAREKET! KACINIYOR",
                            (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                steer = -50 if cx_obj > 320 else 50

        return {"throttle": 20, "steer": steer}, draw_frame

    # ============================================================
    # HUD (Heads-Up Display) OVERLAY
    # ============================================================
    def _draw_hud(self, frame, state: str, decision: dict):
        """
        Kare üzerine durum bilgisi, steer/throttle ve kare sayacı yazar.
        """
        # Üst bilgi şeridi
        cv2.rectangle(frame, (0, 0), (640, 70), (0, 0, 0), -1)
        cv2.rectangle(frame, (0, 0), (640, 70), (30, 60, 100), 1)

        # Durum
        state_colors = {
            "DRIVING":          (0, 255, 0),
            "AIMING":           (0, 255, 255),
            "SHOOTING":         (0, 0, 255),
            "SLIDING_OBSTACLE": (0, 165, 255),
        }
        color = state_colors.get(state, (200, 200, 200))
        cv2.putText(frame, f"DURUM: {state}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Steer / Throttle bar
        throttle = decision.get("throttle", 0)
        steer    = decision.get("steer", 0)
        cv2.putText(frame, f"Steer:{steer:+4d}  Throttle:{throttle:+4d}",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # Kare sayacı (sağ üst)
        cv2.putText(frame, f"F#{self.frame_count}", (570, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)

        # OCR durumu
        if not OCR_AVAILABLE:
            cv2.putText(frame, "OCR: KAPALI", (480, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 200), 1)