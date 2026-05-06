# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QFrame, QGridLayout, QProgressBar,
    QScrollArea, QSizePolicy, QTextEdit
)
from PyQt5.QtCore import Qt, QDateTime
from PyQt5.QtGui import QTextCursor, QColor, QTextCharFormat, QFont

from utils.logger import log as file_log

MAX_LINES = 30
LEVEL_COLORS = {
    "INFO":    "#60a5fa",
    "WARNING": "#fbbf24",
    "ERROR":   "#f87171",
    "SUCCESS": "#4ade80",
    "DEBUG":   "#9ca3af",
}


class LogPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("LogPanel")
        self._line_count = 0
        self._linked: list = []

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        title = QLabel("● SİSTEM LOGU")
        title.setObjectName("LogTitle")
        self.btn_clear = QPushButton("Temizle")
        self.btn_clear.setObjectName("LogClearButton")
        self.btn_clear.setFixedHeight(22)
        self.btn_clear.clicked.connect(self.clear_logs)
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.btn_clear)
        layout.addLayout(header)

        self.log_view = QTextEdit()
        self.log_view.setObjectName("LogView")
        self.log_view.setReadOnly(True)
        self.log_view.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.log_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.log_view.setLineWrapMode(QTextEdit.WidgetWidth)
        self.log_view.setFixedHeight(105)
        self.log_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout.addWidget(self.log_view)

    def add_log(self, message: str, level: str = "INFO"):
        # Dosyaya kaydet
        file_log(message, level)

        # Ekrana yaz
        level = level.upper()
        color = LEVEL_COLORS.get(level, "#9ca3af")
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        cursor = self.log_view.textCursor()
        cursor.movePosition(QTextCursor.End)
        self._append_colored(cursor, f"[{timestamp}] ", "#4b5563")
        self._append_colored(cursor, f"{level:<8}", color)
        self._append_colored(cursor, f"{message}\n", "#d1d5db")
        self.log_view.setTextCursor(cursor)
        self.log_view.ensureCursorVisible()
        self._line_count += 1
        if self._line_count > MAX_LINES:
            self._trim_oldest_line()

    def link_to(self, other):
        if other not in self._linked:
            self._linked.append(other)
        if self not in other._linked:
            other._linked.append(self)

    def clear_logs(self):
        self.log_view.clear()
        self._line_count = 0
        for panel in list(self._linked):
            panel._linked = [p for p in panel._linked if p is not self]
            panel.clear_logs()
            panel._linked.append(self)

    def log_connected(self, host, port, ping=None):
        ping_str = f" · {ping}ms" if ping is not None else ""
        self.add_log(f"Bağlantı kuruldu → {host}:{port}{ping_str}", "SUCCESS")

    def log_disconnected(self):
        self.add_log("Bağlantı kesildi", "ERROR")

    def log_warning(self, msg):   self.add_log(msg, "WARNING")
    def log_error(self, msg):     self.add_log(msg, "ERROR")
    def log_mode_change(self, m): self.add_log(f"Mod değişti → {m}", "INFO")
    def log_laser(self, msg):     self.add_log(msg, "INFO")
    def log_speed(self, v):       self.add_log(f"Hız güncellendi → %{v}", "DEBUG")
    def log_reset(self):          self.add_log("Panel sıfırlandı", "DEBUG")

    def _append_colored(self, cursor, text, hex_color):
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(hex_color))
        fmt.setFont(QFont("Courier New", 10))
        cursor.insertText(text, fmt)

    def _trim_oldest_line(self):
        cursor = self.log_view.textCursor()
        cursor.movePosition(QTextCursor.Start)
        cursor.movePosition(QTextCursor.EndOfLine, QTextCursor.KeepAnchor)
        cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor)
        cursor.removeSelectedText()


class LeftStatusPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("LeftStatusPanel")
        self.setFixedWidth(330)

        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QFrame.NoFrame)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.scroll.setObjectName("LeftScrollArea")

        self.container = QWidget()
        self.container.setObjectName("LeftStatusContainer")
        self.container.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Minimum)

        self.main_layout = QVBoxLayout(self.container)
        self.main_layout.setContentsMargins(14, 10, 14, 6)
        self.main_layout.setSpacing(6)

        self.scroll.setWidget(self.container)
        outer_layout.addWidget(self.scroll)

        self.build_connection()
        self.build_telemetry()
        self.build_imu()
        self.build_khadas_health()
        self.build_log()

        self.main_layout.addStretch(1)
        self.setStyleSheet(self.styles())

    def section_title(self, title, color="#22c55e"):
        label = QLabel(f"● {title}")
        label.setObjectName("SectionTitle")
        label.setStyleSheet(f"color:{color};")
        return label

    def build_connection(self):
        self.main_layout.addWidget(self.section_title("BAĞLANTI"))
        form = QGridLayout()
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(6)
        form.setColumnStretch(0, 0)
        form.setColumnStretch(1, 1)
        self.host_input = QLineEdit("127.0.0.1")
        self.port_input = QLineEdit("9000")
        self.host_input.setObjectName("PanelInput")
        self.port_input.setObjectName("PanelInput")
        form.addWidget(QLabel("Host"), 0, 0)
        form.addWidget(self.host_input, 0, 1)
        form.addWidget(QLabel("TCP Port"), 1, 0)
        form.addWidget(self.port_input, 1, 1)
        self.main_layout.addLayout(form)

        btns = QHBoxLayout()
        btns.setSpacing(8)
        self.btn_connect    = QPushButton("TCP Bağlan")
        self.btn_disconnect = QPushButton("Kes")
        self.btn_connect.setObjectName("ConnectButton")
        self.btn_disconnect.setObjectName("DisconnectButton")
        self.btn_connect.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_disconnect.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        btns.addWidget(self.btn_connect)
        btns.addWidget(self.btn_disconnect)
        self.main_layout.addLayout(btns)

        self.connection_status = QLabel("● Bağlantı yok")
        self.connection_status.setObjectName("ConnectionStatus")
        self.connection_status.setProperty("status", "disconnected")
        self.main_layout.addWidget(self.connection_status)

    def build_telemetry(self):
        self.main_layout.addWidget(self.section_title("TELEMETRİ"))
        self.warning_banner = QLabel("")
        self.warning_banner.setObjectName("WarningBanner")
        self.warning_banner.setWordWrap(True)
        self.warning_banner.hide()
        self.main_layout.addWidget(self.warning_banner)

        self.battery_card      = self.metric_card("Batarya Seviyesi",  "--", "%",    0, "#fbbf24")
        self.battery_temp_card = self.metric_card("Batarya Sıcaklığı", "--", "°C",   0, "#22c55e")
        self.humidity_card     = self.metric_card("Nem Oranı",         "--", "%",    0, "#3b82f6")
        self.speed_card        = self.metric_card("Hız",               "--", "km/h", 0, "#9ca3af")
        for card in [self.battery_card, self.battery_temp_card,
                     self.humidity_card, self.speed_card]:
            self.main_layout.addWidget(card)

    def metric_card(self, title, value, unit, progress, color):
        card = QFrame()
        card.setObjectName("MetricCard")
        card.setFixedHeight(52)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(12, 6, 12, 6)
        layout.setSpacing(3)
        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        title_label = QLabel(title)
        title_label.setObjectName("MetricTitle")
        value_label = QLabel(f"{value} <span style='font-size:11px;color:#8b949e;'>{unit}</span>")
        value_label.setObjectName("MetricValue")
        value_label.setTextFormat(Qt.RichText)
        value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        top.addWidget(title_label, alignment=Qt.AlignVCenter)
        top.addStretch()
        top.addWidget(value_label)
        bar = QProgressBar()
        bar.setRange(0, 100)
        bar.setValue(progress)
        bar.setTextVisible(False)
        bar.setObjectName("MetricProgress")
        bar.setFixedHeight(3)
        bar.setStyleSheet(f"QProgressBar::chunk {{ background-color: {color}; border-radius: 2px; }}")
        layout.addLayout(top)
        layout.addWidget(bar)
        card.value_label  = value_label
        card.progress_bar = bar
        return card

    def build_imu(self):
        title = QLabel("IMU (Roll / Pitch / Yaw)")
        title.setObjectName("SmallTitle")
        self.main_layout.addWidget(title)
        grid = QGridLayout()
        grid.setSpacing(7)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        self.roll_box  = self.imu_box("Roll",  "--")
        self.pitch_box = self.imu_box("Pitch", "--")
        self.yaw_box   = self.imu_box("Yaw",   "--")
        grid.addWidget(self.roll_box,  0, 0)
        grid.addWidget(self.pitch_box, 0, 1)
        grid.addWidget(self.yaw_box,   0, 2)
        self.main_layout.addLayout(grid)

    def imu_box(self, title, value):
        box = QFrame()
        box.setObjectName("ImuBox")
        box.setFixedHeight(48)
        box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QVBoxLayout(box)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(1)
        title_label = QLabel(title)
        title_label.setObjectName("ImuTitle")
        title_label.setAlignment(Qt.AlignCenter)
        value_label = QLabel(value)
        value_label.setObjectName("ImuValue")
        value_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        layout.addWidget(value_label)
        box.value_label = value_label
        return box

    def build_khadas_health(self):
        self.main_layout.addWidget(self.section_title("KHADAS HEALTH"))
        grid = QGridLayout()
        grid.setSpacing(6)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        self.cpu_card  = self.health_card("CPU",      "--",  0,  "#fbbf24")
        self.temp_card = self.health_card("Sıcaklık", "--",  0,  "#22c55e")
        self.ram_card  = self.health_card("RAM",      "--",  0,  "#3b82f6")
        self.disk_card = self.health_card("Disk",     "--",  0,  "#9ca3af")
        grid.addWidget(self.cpu_card,  0, 0)
        grid.addWidget(self.temp_card, 0, 1)
        grid.addWidget(self.ram_card,  1, 0)
        grid.addWidget(self.disk_card, 1, 1)
        self.main_layout.addLayout(grid)

    def health_card(self, title, value, progress, color):
        card = QFrame()
        card.setObjectName("HealthCard")
        card.setFixedHeight(62)
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QVBoxLayout(card)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(2)
        title_label = QLabel(title)
        title_label.setObjectName("HealthTitle")
        value_label = QLabel(value)
        value_label.setObjectName("HealthValue")
        value_label.setStyleSheet(f"color:{color};")
        bar = QProgressBar()
        bar.setRange(0, 100)
        bar.setValue(progress)
        bar.setTextVisible(False)
        bar.setObjectName("HealthProgress")
        bar.setFixedHeight(3)
        bar.setStyleSheet(f"QProgressBar::chunk {{ background-color: {color}; border-radius: 2px; }}")
        layout.addWidget(title_label)
        layout.addWidget(value_label)
        layout.addWidget(bar)
        card.value_label  = value_label
        card.progress_bar = bar
        return card

    def build_log(self):
        self.log_panel = LogPanel()
        self.main_layout.addWidget(self.log_panel)

    # ── Güncelleme ───────────────────────────────

    def update_connection(self, connected=True, ping=None):
        if connected:
            ping_str = f" · {ping}ms ping" if ping is not None else ""
            self.connection_status.setText(f"● Bağlı{ping_str}")
            self.connection_status.setProperty("status", "connected")
            self.log_panel.log_connected(self.host_input.text(), int(self.port_input.text()), ping)
        else:
            self.connection_status.setText("● Bağlantı yok")
            self.connection_status.setProperty("status", "disconnected")
            self.log_panel.log_disconnected()
        self.connection_status.style().unpolish(self.connection_status)
        self.connection_status.style().polish(self.connection_status)

    def update_telemetry(self, battery=None, battery_temp=None, humidity=None,
                         speed=None, roll=None, pitch=None, yaw=None, warning=None):
        warning_text = None
        if battery is not None:
            battery = int(float(battery))
            self.set_metric(self.battery_card, battery, "%", battery)
            if battery <= 20:
                warning_text = f"Batarya düşük — %{battery} kaldı"
                self.log_panel.log_warning(f"Batarya kritik: %{battery}")
        if battery_temp is not None:
            battery_temp = int(float(battery_temp))
            self.set_metric(self.battery_temp_card, battery_temp, "°C", min(battery_temp, 100))
            if battery_temp >= 60:
                self.log_panel.log_warning(f"Batarya sıcaklığı yüksek: {battery_temp}°C")
        if humidity is not None:
            humidity = int(float(humidity))
            self.set_metric(self.humidity_card, humidity, "%", humidity)
        if speed is not None:
            speed_value = float(speed)
            self.set_metric(self.speed_card, speed_value, "km/h", min(int(speed_value * 10), 100))
        if roll is not None:
            self.roll_box.value_label.setText(str(roll))
        if pitch is not None:
            self.pitch_box.value_label.setText(str(pitch))
        if yaw is not None:
            self.yaw_box.value_label.setText(str(yaw))
        if warning and str(warning).lower() != "yok":
            warning_text = str(warning)
        if warning_text:
            self.warning_banner.setText(f"!   {warning_text}")
            self.warning_banner.show()
        else:
            self.warning_banner.hide()

    def update_khadas_health(self, cpu=None, temp=None, ram=None, disk=None):
        if cpu is not None:
            cpu = int(float(cpu))
            self.set_health(self.cpu_card, f"{cpu}%", cpu)
            if cpu >= 90:
                self.log_panel.log_warning(f"CPU yüksek: %{cpu}")
        if temp is not None:
            temp = int(float(temp))
            self.set_health(self.temp_card, f"{temp}°C", min(temp, 100))
            if temp >= 75:
                self.log_panel.log_warning(f"Khadas sıcaklığı kritik: {temp}°C")
        if ram is not None:
            self.set_health(self.ram_card, f"{int(float(ram))}%", int(float(ram)))
        if disk is not None:
            self.set_health(self.disk_card, f"{int(float(disk))}%", int(float(disk)))

    def log_mode_change(self, mode: str):
        self.log_panel.log_mode_change(mode)

    def set_metric(self, card, value, unit, progress):
        card.value_label.setText(
            f"{value} <span style='font-size:11px;color:#8b949e;'>{unit}</span>"
        )
        card.progress_bar.setValue(max(0, min(100, int(progress))))

    def set_health(self, card, value, progress):
        card.value_label.setText(str(value))
        card.progress_bar.setValue(max(0, min(100, int(progress))))

    def styles(self):
        return """
        #LeftStatusPanel { background-color: #0f1724; }
        #LeftScrollArea { background-color: #0f1724; border-right: 1px solid #1f2a3d; }
        #LeftStatusContainer { background-color: #0f1724; }
        QScrollArea { border: none; }
        QScrollBar:vertical { background: #0f1724; width: 6px; margin: 0px; }
        QScrollBar::handle:vertical { background: #263244; border-radius: 3px; min-height: 30px; }
        QScrollBar::handle:vertical:hover { background: #334155; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }
        QLabel { color: #d8dee9; font-size: 12px; }
        #SectionTitle { font-size: 13px; font-weight: 800; letter-spacing: 1px; padding-bottom: 3px; border-bottom: 1px solid #263244; }
        #SmallTitle { color: #8b949e; font-size: 11px; font-weight: 700; }
        #PanelInput { background-color: #1a2232; color: #e5e7eb; border: 1px solid #374151; border-radius: 8px; padding: 6px 10px; font-size: 13px; font-weight: 700; }
        #PanelInput:focus { border: 1px solid #0ea5e9; }
        QPushButton { border-radius: 8px; padding: 7px; font-weight: 800; font-size: 12px; }
        #ConnectButton { background-color: #065f2f; color: #4ade80; border: 1px solid #22c55e; }
        #ConnectButton:hover { background-color: #047857; }
        #DisconnectButton { background-color: #1a2232; color: #9ca3af; border: 1px solid #374151; }
        #DisconnectButton:hover { border: 1px solid #ef4444; color: #ef4444; }
        #ConnectionStatus { background-color: rgba(34,197,94,0.10); color: #4ade80; border: 1px solid rgba(34,197,94,0.35); border-radius: 8px; padding: 7px 10px; font-weight: 800; }
        #ConnectionStatus[status="disconnected"] { background-color: rgba(239,68,68,0.10); color: #ff3b4f; border: 1px solid rgba(239,68,68,0.45); }
        #WarningBanner { background-color: rgba(239,68,68,0.12); color: #ff6b6b; border: 1px solid rgba(239,68,68,0.45); border-radius: 8px; padding: 6px 10px; font-weight: 800; }
        #MetricCard { background-color: #171f2e; border: 1px solid #2d3748; border-radius: 8px; }
        #MetricTitle { color: #8b949e; font-size: 11px; font-weight: 700; }
        #MetricValue { color: #e5e7eb; font-size: 18px; font-weight: 900; }
        #MetricProgress { background-color: #2a3344; border: none; border-radius: 2px; max-height: 3px; }
        #ImuBox { background-color: #171f2e; border: 1px solid #2d3748; border-radius: 8px; }
        #ImuTitle { color: #8b949e; font-size: 10px; font-weight: 700; }
        #ImuValue { color: #e5e7eb; font-size: 14px; font-weight: 900; }
        #HealthCard { background-color: #171f2e; border: 1px solid #2d3748; border-radius: 8px; }
        #HealthTitle { color: #8b949e; font-size: 10px; font-weight: 700; }
        #HealthValue { font-size: 15px; font-weight: 900; }
        #HealthProgress { background-color: #2a3344; border: none; border-radius: 2px; max-height: 3px; }
        #LogTitle { color: #22c55e; font-size: 13px; font-weight: 800; letter-spacing: 1px; padding-bottom: 3px; border-bottom: 1px solid #263244; }
        #LogClearButton { background-color: #1a2232; color: #6b7280; border: 1px solid #2d3748; border-radius: 6px; padding: 2px 10px; font-size: 11px; font-weight: 700; }
        #LogClearButton:hover { color: #f87171; border: 1px solid rgba(239,68,68,0.45); }
        #LogView { background-color: #0b1120; border: 1px solid #1f2a3d; border-radius: 8px; padding: 6px 8px; font-family: "Courier New", monospace; font-size: 11px; color: #d1d5db; }
        """