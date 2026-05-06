from PyQt5.QtWidgets import QToolBar, QWidget, QHBoxLayout, QPushButton
from PyQt5.QtCore import QTimer


class EStopButton(QPushButton):
    def __init__(self, text="⛔ ACİL STOP", parent=None):
        super().__init__(text, parent)
        self.setObjectName("EStopButton")
        self.setFixedHeight(36)
        self.setMinimumWidth(140)

        self._pulse_step = 0
        self._pulse_up   = True
        self._timer = QTimer(self)
        self._timer.setInterval(30)
        self._timer.timeout.connect(self._pulse_tick)
        self._timer.start()

        self.setStyleSheet(self._base_style(255))

    def _pulse_tick(self):
        step = 4
        if self._pulse_up:
            self._pulse_step += step
            if self._pulse_step >= 135:
                self._pulse_up = False
        else:
            self._pulse_step -= step
            if self._pulse_step <= 0:
                self._pulse_up = True

        brightness = 120 + self._pulse_step
        self.setStyleSheet(self._base_style(brightness))

    def _base_style(self, brightness: int) -> str:
        r = brightness
        border_alpha = int((brightness - 120) / 135 * 200 + 55)
        return f"""
        QPushButton#EStopButton {{
            background-color: rgb({r}, 0, 0);
            color: white;
            border: 2px solid rgba(255, 80, 80, {border_alpha});
            border-radius: 8px;
            font-size: 13px;
            font-weight: 900;
            letter-spacing: 1px;
            padding: 0 16px;
        }}
        QPushButton#EStopButton:hover {{
            background-color: rgb(255, 30, 30);
            border: 2px solid rgba(255, 120, 120, 255);
        }}
        QPushButton#EStopButton:pressed {{
            background-color: rgb(180, 0, 0);
        }}
        """


def build_toolbar(main_window) -> QToolBar:
    toolbar = QToolBar()
    toolbar.setMovable(False)
    toolbar.setFloatable(False)
    toolbar.setObjectName("MainToolbar")
    toolbar.setStyleSheet("""
        QToolBar#MainToolbar {
            background-color: #080e18;
            border-bottom: 1px solid #1f2a3d;
            spacing: 0px;
            padding: 4px 10px;
        }
    """)

    container = QWidget()
    layout = QHBoxLayout(container)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(8)

    btn_estop = EStopButton("⛔  ACİL STOP")
    btn_estop.clicked.connect(main_window.on_estop)
    layout.addWidget(btn_estop)

    layout.addSpacing(12)

    btn_control = QPushButton("KONTROL")
    btn_control.setObjectName("NavButton")
    btn_control.setProperty("active", "true")
    btn_control.setFixedHeight(34)
    btn_control.setMinimumWidth(90)
    btn_control.clicked.connect(lambda: main_window.switch_page(0))

    btn_mission = QPushButton("MISSION")
    btn_mission.setObjectName("NavButton")
    btn_mission.setProperty("active", "false")
    btn_mission.setFixedHeight(34)
    btn_mission.setMinimumWidth(90)
    btn_mission.clicked.connect(lambda: main_window.switch_page(1))

    layout.addWidget(btn_control)
    layout.addWidget(btn_mission)

    layout.addStretch()

    toolbar.addWidget(container)

    nav_style = """
        QPushButton#NavButton {
            background-color: transparent;
            color: #6b7280;
            border: 1px solid transparent;
            border-radius: 7px;
            font-size: 12px;
            font-weight: 800;
            letter-spacing: 0.5px;
            padding: 0 12px;
        }
        QPushButton#NavButton:hover {
            color: #d8dee9;
            border: 1px solid #2d3748;
        }
        QPushButton#NavButton[active="true"] {
            color: #e5e7eb;
            background-color: #1a2232;
            border: 1px solid #22c55e;
        }
    """
    btn_control.setStyleSheet(nav_style)
    btn_mission.setStyleSheet(nav_style)

    main_window.btn_control = btn_control
    main_window.btn_mission = btn_mission

    return toolbar