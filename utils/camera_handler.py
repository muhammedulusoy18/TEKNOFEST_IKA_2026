import cv2
import threading
import time


class CameraHandler:
    def __init__(self, camera_source=0, width=640, height=480, fps=30):
        # camera_source: Kendi bilgisayarın için 0, Khadas USB kamera için genelde 0 veya 1
        self.camera_source = camera_source
        self.width = width
        self.height = height
        self.fps = fps

        self.cap = cv2.VideoCapture(self.camera_source)
        self._set_camera_params()

        self.grabbed, self.frame = self.cap.read()
        self.running = True

        # Thread güvenliği için kilit mekanizması
        self.lock = threading.Lock()

        # Arka planda kamerayı sürekli okuyan bağımsız thread
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()
        print(f"[KAMERA] {camera_source} portundan {width}x{height} çözünürlükte başlatıldı.")

    def _set_camera_params(self):
        """Donanımsal kamera ayarlarını zorlar (Khadas USB bant genişliği için kritik)"""
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        # Linux üzerinde USB kameranın NPU'yu boğmasını engellemek için MJPG formatı:
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    def _update(self):
        """Asıl sihrin olduğu yer: Ana döngüyü bloklamadan en taze kareyi sürekli alır."""
        while self.running:
            grabbed, frame = self.cap.read()

            with self.lock:
                self.grabbed = grabbed
                if grabbed:
                    self.frame = frame

            # Kamera kablosu sarsıntıdan çıkarsa/temassızlık yaparsa oto-reconnect
            if not grabbed:
                print("[KAMERA UYARISI] Görüntü alınamıyor! Yeniden bağlanılıyor...")
                time.sleep(1)
                self.cap.release()
                self.cap = cv2.VideoCapture(self.camera_source)
                self._set_camera_params()

    def get_frame(self):
        """perception.py'nin görüntüyü beklemeden kopyalayıp alacağı fonksiyon."""
        with self.lock:
            if self.frame is not None:
                return self.frame.copy()
        return None

    def stop(self):
        """Sistemi ve thread'i güvenlice kapatır."""
        self.running = False
        self.thread.join()
        self.cap.release()
        print("[KAMERA] Donanım bağlantısı kesildi.")