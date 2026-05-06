import json
import socket
from PyQt5.QtCore import QThread, pyqtSignal


class TcpClient(QThread):
    status = pyqtSignal(str)      # "CONNECTED" / "DISCONNECTED" / "ERROR:..."
    received = pyqtSignal(dict)   # gelen JSON
    ack = pyqtSignal(dict)        # ACK paketleri

    def __init__(self, host="127.0.0.1", port=9000, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self._running = True
        self._sock = None

    def connect_to_server(self):
        # Thread run() içinde connect edilecek
        if not self.isRunning():
            self.start()

    def disconnect_from_server(self):
        self._running = False
        try:
            if self._sock:
                self._sock.shutdown(socket.SHUT_RDWR)
                self._sock.close()
        except Exception:
            pass
        self._sock = None

    def send_json(self, payload: dict):
        try:
            if not self._sock:
                self.status.emit("DISCONNECTED")
                return False
            msg = (json.dumps(payload) + "\n").encode("utf-8")
            self._sock.sendall(msg)
            return True
        except Exception as e:
            self.status.emit(f"ERROR:{e}")
            return False

    def run(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(5.0)
            self._sock.connect((self.host, self.port))
            self._sock.settimeout(None)
            self.status.emit("CONNECTED")
        except Exception as e:
            self.status.emit(f"ERROR:{e}")
            self._sock = None
            return

        buff = b""
        while self._running and self._sock:
            try:
                chunk = self._sock.recv(4096)
                if not chunk:
                    break
                buff += chunk
                while b"\n" in buff:
                    line, buff = buff.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        obj = json.loads(line.decode("utf-8", errors="ignore"))
                        self.received.emit(obj)
                        if obj.get("type") == "ACK":
                            self.ack.emit(obj)
                    except Exception:
                        continue
            except Exception:
                break

        self.status.emit("DISCONNECTED")
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass
        self._sock = None
