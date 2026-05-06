import json
import socket
from PyQt5.QtCore import QThread, pyqtSignal

# ============================================================
# PORT IN USE CHECK
# ============================================================
def is_port_in_use(ip, port):
    """
    UDP portunu test eder. Port doluysa True döner.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.bind((ip, port))
            return False  # Port boş
        except OSError:
            return True   # Port dolu → başka thread kullanıyor


class TelemetryWorker(QThread):
    telemetry = pyqtSignal(dict)
    link = pyqtSignal(dict)

    def __init__(self, bind_ip="0.0.0.0", port=9001, parent=None):
        super().__init__(parent)
        self.bind_ip = bind_ip
        self.port = port
        self._running = True
        self.sock = None

    def stop(self):
        self._running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass

    # ============================================================
    # THREAD RUN
    # ============================================================
    def run(self):

        print(f"[Telemetry] Starting on {self.bind_ip}:{self.port}")

        # 1) PORT ÇAKIŞMASI ÖNLEME
        if is_port_in_use(self.bind_ip, self.port):
            print(f"[ERROR] Port {self.port} already in use! Telemetry aborted.")
            return

        # 2) SOCKET
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.bind_ip, self.port))
            self.sock.settimeout(1.0)
            print("[Telemetry] Socket opened successfully.")
        except Exception as e:
            print(f"[ERROR] Cannot open socket: {e}")
            return

        # 3) MAIN LOOP
        while self._running:
            try:
                data, addr = self.sock.recvfrom(8192)

                print(f"\n===== RECV TELEMETRY FROM {addr[0]}:{addr[1]} =====")
                print("RAW DATA:", data)

                try:
                    payload = json.loads(data.decode("utf-8", errors="ignore"))
                    print("DECODED JSON:", payload)

                    self.telemetry.emit(payload)
                    self.link.emit({"udp_from": f"{addr[0]}:{addr[1]}"})

                except Exception as json_err:
                    print("[ERROR] INVALID JSON →", json_err)

            except socket.timeout:
                continue

            except Exception as e:
                print(f"[ERROR] Telemetry loop error: {e}")
                break

        print("[Telemetry] Stopped.")

        # 4) CLEANUP
        try:
            if self.sock:
                self.sock.close()
        except:
            pass