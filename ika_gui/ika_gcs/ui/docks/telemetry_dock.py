from PyQt5.QtWidgets import QDockWidget, QWidget, QVBoxLayout
from ui.widgets.indicator import Indicator


class TelemetryDock(QDockWidget):
    def __init__(self, parent=None):
        super().__init__("Telemetri", parent)

        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        self.ind_batt = Indicator("Batarya Seviyesi", "—%", accent="yellow")
        self.ind_batt_temp = Indicator("Batarya Sıcaklığı", "— °C", accent="red")
        self.ind_humidity = Indicator("Nem Oranı", "—%", accent="cyan")
        self.ind_speed = Indicator("Hız", "— km/h", accent="blue")
        self.ind_imu = Indicator("IMU (Roll / Pitch / Yaw)", "— / — / —", accent="purple")
        self.ind_warn = Indicator("Uyarı", "Yok", accent="orange")

        layout.addWidget(self.ind_batt)
        layout.addWidget(self.ind_batt_temp)
        layout.addWidget(self.ind_humidity)
        layout.addWidget(self.ind_speed)
        layout.addWidget(self.ind_imu)
        layout.addWidget(self.ind_warn)
        layout.addStretch(1)

        self.setWidget(w)

    def update_from_telemetry(self, t: dict):
        # 🔋 Batarya yüzdesi
        if "batt" in t:
            batt_val = t["batt"]
            self.ind_batt.setValue(f"%{batt_val}")

            if batt_val < 20:
                accent = "red"
            elif batt_val < 40:
                accent = "orange"
            else:
                accent = "yellow"

            self.ind_batt.setAccent(accent)

        # 🌡 Batarya sıcaklığı
        if "batt_temp" in t:
            temp = t["batt_temp"]
            self.ind_batt_temp.setValue(f"{temp} °C")

            if temp > 60:
                accent = "red"
            elif temp > 45:
                accent = "orange"
            else:
                accent = "yellow"

            self.ind_batt_temp.setAccent(accent)

        # 💧 Nem oranı
        if "humidity" in t:
            self.ind_humidity.setValue(f"%{t['humidity']}")
            self.ind_humidity.setAccent("cyan")

        # ⚡ Hız
        if "speed" in t:
            spd = t["speed"]
            self.ind_speed.setValue(f"{spd} km/h")
            self.ind_speed.setAccent("blue")

        # 🎛 IMU — Roll / Pitch / Yaw
        if any(k in t for k in ("roll", "pitch", "yaw")):
            r = t.get("roll", "—")
            p = t.get("pitch", "—")
            y = t.get("yaw", "—")
            self.ind_imu.setValue(f"{r} / {p} / {y}")
            self.ind_imu.setAccent("purple")

        # ⚠ Uyarı
        if "warn" in t:
            warn_txt = t["warn"]

            if not warn_txt or warn_txt == "OK":
                self.ind_warn.setValue("Yok")
                self.ind_warn.setAccent("orange")
            else:
                self.ind_warn.setValue(str(warn_txt))
                self.ind_warn.setAccent("red")