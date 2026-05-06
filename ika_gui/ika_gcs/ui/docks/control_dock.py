from PyQt5.QtWidgets import (
    QDockWidget, QWidget, QVBoxLayout, QGroupBox,
    QLabel, QPushButton
)
from PyQt5.QtCore import Qt
from ui.widgets.joystick import JoystickWidget


class ControlDock(QDockWidget):
    def __init__(self, parent=None):
        super().__init__("Kontrol Paneli", parent)
        self.setAllowedAreas(Qt.RightDockWidgetArea)
        self.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)

        container = QWidget()
        root = QVBoxLayout(container)
        root.setSpacing(14)

        # ============================================================
        # 1) OTONOM SÜRÜŞ BLOĞU
        # ============================================================
        gb_auto = QGroupBox("Otonom Sürüş")
        auto_layout = QVBoxLayout(gb_auto)

        self.btn_auto_on = QPushButton("Otonom Sürüşü Aktif Et")
        self.btn_auto_off = QPushButton("Otonom Sürüşü Pasif Et")

        # 🚨 ACİL STOP
        self.btn_estop = QPushButton("🚨 ACİL STOP")
        self.btn_estop.setStyleSheet(
            "background:#b30000; color:white; font-weight:bold; padding:10px;"
        )

        auto_layout.addWidget(self.btn_auto_on)
        auto_layout.addWidget(self.btn_auto_off)
        auto_layout.addWidget(self.btn_estop)

        # ============================================================
        # 2) MANUEL SÜRÜŞ BLOĞU
        # ============================================================
        gb_manual = QGroupBox("Manuel Sürüş")
        manual_layout = QVBoxLayout(gb_manual)

        self.btn_manual_on = QPushButton("Manuel Sürüşü Aktif Et")
        self.btn_manual_off = QPushButton("Manuel Sürüşü Pasif Et")

        # 🕹 JOYSTICK
        manual_layout.addWidget(QLabel("Joystick"))
        self.joystick = JoystickWidget(size=180)
        self.joystick.valueChanged.connect(self._on_joystick_value_changed)

        manual_layout.addWidget(self.btn_manual_on)
        manual_layout.addWidget(self.btn_manual_off)
        manual_layout.addWidget(self.joystick, alignment=Qt.AlignCenter)

        # ============================================================
        # ANA PANEL TOPLAMA
        # ============================================================
        root.addWidget(gb_auto)
        root.addWidget(gb_manual)
        root.addStretch(1)

        self.setWidget(container)

        # ============================================================
        # CALLBACK BAĞLAMA
        # ============================================================
        self.btn_auto_on.clicked.connect(self._cb_auto_on)
        self.btn_auto_off.clicked.connect(self._cb_auto_off)
        self.btn_manual_on.clicked.connect(self._cb_manual_on)
        self.btn_manual_off.clicked.connect(self._cb_manual_off)
        self.btn_estop.clicked.connect(self._cb_estop)

    # ============================================================
    # CALLBACK WRAPPERS
    # ============================================================
    def _cb_auto_on(self):
        if hasattr(self.parent(), "on_drive_auto_enable"):
            self.parent().on_drive_auto_enable()

    def _cb_auto_off(self):
        if hasattr(self.parent(), "on_drive_auto_disable"):
            self.parent().on_drive_auto_disable()

    def _cb_manual_on(self):
        if hasattr(self.parent(), "on_drive_manual_enable"):
            self.parent().on_drive_manual_enable()

    def _cb_manual_off(self):
        if hasattr(self.parent(), "on_drive_manual_disable"):
            self.parent().on_drive_manual_disable()

    def _cb_estop(self):
        if hasattr(self.parent(), "on_estop"):
            self.parent().on_estop()

    def _on_joystick_value_changed(self, x, y):
        if hasattr(self.parent(), "on_joystick"):
            self.parent().on_joystick(x, y)

    # ============================================================
    # ENABLE/DISABLE
    # ============================================================
    def set_enabled(self, enabled: bool):
        self.btn_auto_on.setEnabled(enabled)
        self.btn_auto_off.setEnabled(enabled)
        self.btn_manual_on.setEnabled(enabled)
        self.btn_manual_off.setEnabled(enabled)
        self.btn_estop.setEnabled(enabled)
        self.joystick.setEnabled(enabled)