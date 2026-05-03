import cv2
import numpy as np
import time
import os

# Tesseract OCR Yapılandırması (Cross-Platform)
try:
    import pytesseract

    if os.name == 'nt':
        pytesseract.pytesseract.tesseract_cmd = r'C:\Program Files\Tesseract-OCR\tesseract.exe'
    OCR_AVAILABLE = True
except ImportError:
    OCR_AVAILABLE = False


class PerceptionUnit:
    def __init__(self):
        # 1. Durum ve Bayraklar
        self.laser_active = False
        self.laser_on_time = 0

        # 2. Hareketli Engel Tespiti (MOG2)
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=100, varThreshold=50, detectShadows=False)

        # 3. Koni/Engel Renk Aralıkları (Hsv)
        self.lower_orange = np.array([0, 130, 100])
        self.upper_orange = np.array([25, 255, 255])

        self.target_number = "8"
        self.frame_count = 0

    def process_frame(self, frame, current_state):
        if frame is None:
            return {"throttle": 0, "steer": 0}, frame, current_state

        self.frame_count += 1
        frame = cv2.resize(frame, (640, 480))
        output_frame = frame.copy()

        # MOG2 ARKA PLAN ÖĞRENMESİ (Her karede çalışmak ZORUNDA)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred_gray = cv2.GaussianBlur(gray_frame, (5, 5), 0)
        fg_mask = self.bg_subtractor.apply(blurred_gray)

        # HAREKETLİ ENGEL REFLEKSİ (Öncelikli Güvenlik Kontrolü)
        if current_state in ["DRIVING", "SLIDING_OBSTACLE"]:
            contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            moving_obstacle_found = False

            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 3000:
                    moving_obstacle_found = True
                    current_state = "SLIDING_OBSTACLE"

            if not moving_obstacle_found and current_state == "SLIDING_OBSTACLE":
                current_state = "DRIVING"

        # --- DURUM MAKİNESİ (FSM) YÖNLENDİRMESİ ---
        if current_state == "DRIVING":
            decision, output_frame, new_state = self.__autonomous_drive(frame, output_frame)
        elif current_state == "AIMING":
            decision, output_frame, new_state = self.__aim_target(frame, output_frame)
        elif current_state == "SHOOTING":
            decision, output_frame, new_state = self.__handle_shooting(frame, output_frame)
        elif current_state == "SLIDING_OBSTACLE":
            decision, output_frame = self.__avoid_sliding_obstacle(fg_mask, output_frame)
            new_state = "SLIDING_OBSTACLE"
        else:
            decision = {"throttle": 0, "steer": 0}
            new_state = "DRIVING"

        self._draw_status(output_frame, new_state, decision)

        return decision, output_frame, new_state

    def __autonomous_drive(self, frame, draw_frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        # 1. TABELA VE HEDEF KONTROLÜ
        if self.frame_count % 10 == 0 and OCR_AVAILABLE:
            circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=150,
                                       param1=50, param2=40, minRadius=30, maxRadius=150)
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    roi = gray[max(0, i[1] - i[2]):min(480, i[1] + i[2]),
                    max(0, i[0] - i[2]):min(640, i[0] + i[2])]
                    if roi.size > 0:
                        roi_t = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
                        text = pytesseract.image_to_string(roi_t,
                                                           config='--psm 10 -c tessedit_char_whitelist=0123456789').strip()
                        if self.target_number in text:
                            return {"throttle": 0, "steer": 0}, draw_frame, "AIMING"

        # 2. SABİT ENGEL ALGILAMA
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        steer = 0
        for cnt in contours:
            if cv2.contourArea(cnt) > 800:
                x, y, w, h = cv2.boundingRect(cnt)
                center_x = x + (w // 2)
                cv2.rectangle(draw_frame, (x, y), (x + w, y + h), (0, 165, 255), 2)
                steer = -40 if center_x > 320 else 40

        return {"throttle": 30, "steer": steer}, draw_frame, "DRIVING"

    def __aim_target(self, frame, draw_frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(cv2.GaussianBlur(gray, (7, 7), 1.5), cv2.HOUGH_GRADIENT,
                                   dp=1, minDist=100, param1=50, param2=25, minRadius=15, maxRadius=120)

        steer = 0
        if circles is not None:
            i = np.uint16(np.around(circles))[0, 0]
            error_x = int(i[0]) - 320
            cv2.circle(draw_frame, (i[0], i[1]), i[2], (0, 255, 0), 2)

            if abs(error_x) < 20:
                self.laser_on_time = time.time()
                return {"throttle": 0, "steer": 0}, draw_frame, "SHOOTING"
            else:
                steer = int(np.clip(error_x * 0.4, -50, 50))

        return {"throttle": 0, "steer": steer}, draw_frame, "AIMING"

    def __handle_shooting(self, frame, draw_frame):
        self.laser_active = True
        elapsed = time.time() - self.laser_on_time

        if elapsed > 2.0:
            self.laser_active = False
            return {"throttle": 0, "steer": 0}, draw_frame, "DRIVING"

        cv2.putText(draw_frame, "!!! ATES !!!", (230, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)
        return {"throttle": 0, "steer": 0}, draw_frame, "SHOOTING"

    def __avoid_sliding_obstacle(self, fg_mask, draw_frame):
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        steer = 0
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 3000:
                x, y, w, h = cv2.boundingRect(largest)
                cv2.rectangle(draw_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                steer = -50 if (x + w // 2) > 320 else 50

        return {"throttle": 20, "steer": steer}, draw_frame

    def _draw_status(self, frame, state, decision):
        cv2.putText(frame, f"Muhammed Ulusoy | {state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"S: {decision['steer']} T: {decision['throttle']}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)