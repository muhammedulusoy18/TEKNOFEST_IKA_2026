from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QSizePolicy, QFrame, QLabel
from PyQt5.QtCore import Qt

from ui.panels.left_status_panel import LeftStatusPanel
from ui.panels.autonomy_panel import AutonomyPanel
from ui.panels.manual_drive_panel import ManualDrivePanel
from ui.panels.laser_control_panel import LaserControlPanel

from ui.widgets.video_wall import VideoWall
from ui.widgets.joystick import JoystickWidget


class MissionPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        root = QHBoxLayout(self)
        root.setSpacing(12)
        root.setContentsMargins(0, 0, 0, 0)

        # ----------------------------------------------------
        # SOL PANEL
        # ----------------------------------------------------
        self.left_panel = LeftStatusPanel(self)

        left_col = QVBoxLayout()
        left_col.setSpacing(0)
        left_col.setContentsMargins(0, 0, 0, 0)
        left_col.addWidget(self.left_panel)

        # ----------------------------------------------------
        # ORTA PANEL
        # ----------------------------------------------------
        center_col = QVBoxLayout()
        center_col.setSpacing(12)

        self.laser_camera = VideoWall(camera_count=1)
        self.laser_camera.setObjectName("laserCameraWall")
        self.laser_camera.setMinimumHeight(350)
        self.laser_camera.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        center_col.addWidget(self.laser_camera)

        bottom_row = QHBoxLayout()
        bottom_row.setSpacing(12)

        # ----------------------------------------------------
        # NİŞAN JOYSTICK
        # ----------------------------------------------------
        self.aim_joystick_frame = QFrame()
        self.aim_joystick_frame.setObjectName("JoystickFrame")
        self.aim_joystick_frame.setStyleSheet("""
            #JoystickFrame {
                background-color: #0f1724;
                border: 1px solid #1f2a3d;
                border-radius: 10px;
            }
        """)

        aim_layout = QVBoxLayout(self.aim_joystick_frame)
        aim_layout.setContentsMargins(12, 12, 12, 12)
        aim_layout.setSpacing(10)

        aim_title = QLabel("● NİŞAN KONTROLÜ")
        aim_title.setStyleSheet("""
            color: #22c55e;
            font-size: 13px;
            font-weight: 800;
            letter-spacing: 1px;
            padding-bottom: 6px;
            border-bottom: 1px solid #263244;
            border-left: none;
            border-right: none;
            border-top: none;
            border-radius: 0px;
        """)
        aim_layout.addWidget(aim_title)

        self.aim_joystick = JoystickWidget(220)
        aim_layout.addWidget(self.aim_joystick, 0, alignment=Qt.AlignCenter)

        # ----------------------------------------------------
        # LAZER PANEL
        # ----------------------------------------------------
        self.laser_panel = LaserControlPanel()

        bottom_row.addWidget(self.aim_joystick_frame, 1)
        bottom_row.addWidget(self.laser_panel, 1)

        center_col.addLayout(bottom_row)

        # ----------------------------------------------------
        # SAĞ PANEL
        # ----------------------------------------------------
        self.autonomy = AutonomyPanel()
        self.manual_drive = ManualDrivePanel()

        right_col = QVBoxLayout()
        right_col.setSpacing(12)
        right_col.addWidget(self.autonomy)
        right_col.addWidget(self.manual_drive)
        right_col.addStretch()

        # ----------------------------------------------------
        # ANA LAYOUT
        # ----------------------------------------------------
        root.addLayout(left_col, 1)
        root.addLayout(center_col, 4)
        root.addLayout(right_col, 1)