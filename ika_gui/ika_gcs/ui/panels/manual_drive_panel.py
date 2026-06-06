from ui.widgets.joystick import JoystickWidget
from PyQt5.QtWidgets import (
    QFrame, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton, QSizePolicy
)
from PyQt5.QtCore import Qt, pyqtSignal
 
 
class ManualDrivePanel(QFrame):
    # fix #2: log sinyalleri — MainWindow bunları log_panel'e bağlar
    sig_mode_changed = pyqtSignal(str)   # "AKTİF" / "PASİF"
    sig_speed_changed = pyqtSignal(int)  # 0-100
    sig_joystick_move = pyqtSignal(float, float) # (x, y)
 
    def __init__(self):
        super().__init__()
        self.setObjectName("manualDrivePanel")
 
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)
 
        # Başlık
        title = QLabel("● MANUEL SÜRÜŞ")
        title.setObjectName("RightSectionTitle")
        layout.addWidget(title)
 
        # Joystick
        self.joystick = JoystickWidget(170)
        self.joystick.setObjectName("manualJoystick")
        layout.addWidget(self.joystick, alignment=Qt.AlignCenter)
        self.joystick.valueChanged.connect(self.on_joystick_move)
 
        # Hız bölümü
        speed_title = QLabel("HIZ KONTROLÜ")
        speed_title.setObjectName("RightSubLabel")
        layout.addWidget(speed_title)
 
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setObjectName("manualSpeedSlider")
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(100)
        self.speed_slider.setValue(60)
        layout.addWidget(self.speed_slider)
 
        self.speed_label = QLabel("Hız: %60")
        self.speed_label.setObjectName("RightInfoLabel")
        layout.addWidget(self.speed_label)
 
        self.speed_slider.valueChanged.connect(self._on_speed_changed)
 
        # Manuel mod butonu
        self.manual_enabled = True
        self.manual_button = QPushButton("MANUEL MOD: AKTİF")
        self.manual_button.setObjectName("manualModeButton")
        self.manual_button.setProperty("state", "active")
        self.manual_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.manual_button.setFixedHeight(38)
        layout.addWidget(self.manual_button)
        self.manual_button.clicked.connect(self.toggle_manual_mode)
 
        self.mode_label = QLabel("Joystick ile kontrol")
        self.mode_label.setObjectName("RightInfoLabel")
        layout.addWidget(self.mode_label)
 
        layout.addStretch()
        self.setStyleSheet(self._styles())
 
    def _on_speed_changed(self, value):
        self.speed_label.setText(f"Hız: %{value}")
        self.sig_speed_changed.emit(value)  # log sinyali
 
    def get_speed(self):
        return self.speed_slider.value() / 100.0
 
    def toggle_manual_mode(self):
        self.manual_enabled = not self.manual_enabled
        if self.manual_enabled:
            self.manual_button.setText("MANUEL MOD: AKTİF")
            self.manual_button.setProperty("state", "active")
            self.mode_label.setText("Joystick ile kontrol")
            self.sig_mode_changed.emit("MANUEL AKTİF")
        else:
            self.manual_button.setText("MANUEL MOD: PASİF")
            self.manual_button.setProperty("state", "passive")
            self.mode_label.setText("Manuel sürüş devre dışı")
            self.sig_mode_changed.emit("MANUEL PASİF")
        self.manual_button.style().unpolish(self.manual_button)
        self.manual_button.style().polish(self.manual_button)
        self.manual_button.update()
 
    def on_joystick_move(self, x, y):
        if not self.manual_enabled:
            return
        speed = self.get_speed()
        self.sig_joystick_move.emit(x * speed, y * speed)
 
    def _styles(self):
        return """
        #manualDrivePanel {
            background-color: #111827;
            border: 1px solid #1f2a3d;
            border-radius: 10px;
        }
        #RightSectionTitle {
            color: #22c55e;
            font-size: 13px;
            font-weight: 800;
            letter-spacing: 1px;
            padding-bottom: 6px;
            border-bottom: 1px solid #263244;
        }
        #RightSubLabel {
            color: #8b949e;
            font-size: 11px;
            font-weight: 700;
            letter-spacing: 0.5px;
        }
        #RightInfoLabel { color: #8b949e; font-size: 11px; }
 
        QSlider#manualSpeedSlider::groove:horizontal {
            height: 4px;
            background: #2a3344;
            border-radius: 2px;
        }
        QSlider#manualSpeedSlider::handle:horizontal {
            background: #3b82f6;
            border: 2px solid #1d4ed8;
            width: 14px;
            height: 14px;
            margin: -5px 0;
            border-radius: 7px;
        }
        QSlider#manualSpeedSlider::sub-page:horizontal {
            background: #3b82f6;
            border-radius: 2px;
        }
 
        QPushButton#manualModeButton[state="active"] {
            background-color: #14532d;
            color: #4ade80;
            border: 1px solid #22c55e;
            border-radius: 8px;
            font-size: 12px;
            font-weight: 900;
            padding: 8px;
        }
        QPushButton#manualModeButton[state="active"]:hover {
            background-color: #166534;
        }
        QPushButton#manualModeButton[state="passive"] {
            background-color: #1a2232;
            color: #6b7280;
            border: 1px solid #374151;
            border-radius: 8px;
            font-size: 12px;
            font-weight: 900;
            padding: 8px;
        }
        QPushButton#manualModeButton[state="passive"]:hover {
            border: 1px solid #4b5563;
            color: #9ca3af;
        }
        """
 