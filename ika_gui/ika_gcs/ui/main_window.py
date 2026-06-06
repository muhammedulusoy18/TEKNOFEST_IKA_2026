# -*- coding: utf-8 -*-
from workers.gcs_ros_node import GCSROSWorker

from PyQt5.QtWidgets import (
    QMainWindow, QWidget,
    QHBoxLayout, QVBoxLayout, QSizePolicy,
    QStackedWidget
)

from ui.toolbar import build_toolbar
from ui.status_bar import build_status_bar

from ui.panels.left_status_panel import LeftStatusPanel
from ui.panels.autonomy_panel import AutonomyPanel
from ui.panels.manual_drive_panel import ManualDrivePanel

from ui.widgets.video_wall import VideoWall
from ui.widgets.map_widget import MapWidget
from ui.widgets.toast import Toast

from ui.pages.mission_page import MissionPage


class MainWindow(QMainWindow):
    def __init__(self, app):
        super().__init__()
        self.app = app

        self.setWindowTitle("IKA GCS • PyQt5")
        self.resize(1800, 1000)

        self.toast = Toast(self)
        self.btn_control = None
        self.btn_mission = None

        central = QWidget(self)
        self.setCentralWidget(central)

        outer = QVBoxLayout(central)
        outer.setSpacing(0)
        outer.setContentsMargins(10, 10, 10, 10)

        self.pages = QStackedWidget()
        outer.addWidget(self.pages)

        # ── CONTROL PAGE ─────────────────────────────
        self.control_page = QWidget()
        self.pages.addWidget(self.control_page)

        root = QHBoxLayout(self.control_page)
        root.setSpacing(12)
        root.setContentsMargins(0, 0, 0, 0)

        self.left_panel = LeftStatusPanel(self)

        left_col = QVBoxLayout()
        left_col.setSpacing(0)
        left_col.setContentsMargins(0, 0, 0, 0)
        left_col.addWidget(self.left_panel)

        center_col = QVBoxLayout()
        center_col.setSpacing(12)

        self.cam_main = VideoWall()
        self.cam_main.setMinimumHeight(350)
        self.cam_main.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        center_col.addWidget(self.cam_main)

        self.map = MapWidget()
        self.map.setMinimumHeight(200)
        self.map.setMaximumHeight(450)
        self.map.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        center_col.addWidget(self.map)

        self.autonomy     = AutonomyPanel()
        self.manual_drive = ManualDrivePanel()

        right_col = QVBoxLayout()
        right_col.setSpacing(12)
        right_col.addWidget(self.autonomy)
        right_col.addWidget(self.manual_drive)
        right_col.addStretch()

        root.addLayout(left_col, 1)
        root.addLayout(center_col, 4)
        root.addLayout(right_col, 1)

        # ── MISSION PAGE ─────────────────────────────
        self.mission_page = MissionPage(self)
        self.pages.addWidget(self.mission_page)

        # ── TOOLBAR + STATUSBAR ───────────────────────
        self.toolbar = build_toolbar(self)
        self.addToolBar(self.toolbar)
        build_status_bar(self)

        self.switch_page(0)

        # ── ROS 2 WORKER ─────────────────────────────
        self.ros_worker = GCSROSWorker()
        self.ros_worker.sig_telemetry.connect(self.on_telemetry)
        self.ros_worker.sig_image.connect(self.cam_main.cam1.set_frame)
        self.ros_worker.sig_status.connect(self.on_ros_status)
        
        self.left_panel.btn_connect.clicked.connect(self.ros_connect)
        self.left_panel.btn_disconnect.clicked.connect(self.ros_disconnect)

        self.mission_page.left_panel.btn_connect.clicked.connect(self.ros_connect)
        self.mission_page.left_panel.btn_disconnect.clicked.connect(self.ros_disconnect)

        # ── Buton log bağlantıları ────────────────────

        # Control — Manuel sürüş
        self.manual_drive.sig_mode_changed.connect(
            lambda m: self.left_panel.log_mode_change(m)
        )
        self.manual_drive.sig_speed_changed.connect(
            lambda v: self.left_panel.log_panel.log_speed(v)
        )
        self.manual_drive.sig_joystick_move.connect(self._send_joystick_cmd)

        # Control — Otonom: sayfa geçişi + log + toast
        self.autonomy.sig_autonomy_toggled.connect(self._on_autonomy_control)

        # Mission — Manuel sürüş
        self.mission_page.manual_drive.sig_mode_changed.connect(
            lambda m: self.mission_page.left_panel.log_mode_change(m)
        )
        self.mission_page.manual_drive.sig_speed_changed.connect(
            lambda v: self.mission_page.left_panel.log_panel.log_speed(v)
        )

        # Mission — Otonom: sayfa geçişi + log + toast
        self.mission_page.autonomy.sig_autonomy_toggled.connect(self._on_autonomy_mission)

        # Mission — Lazer
        self.mission_page.laser_panel.sig_laser_toggled.connect(
            lambda m: self.mission_page.left_panel.log_panel.log_laser(m)
        )
        self.mission_page.laser_panel.sig_lock_toggled.connect(
            lambda m: self.mission_page.left_panel.log_panel.log_laser(m)
        )
        self.mission_page.laser_panel.sig_reset.connect(
            self.mission_page.left_panel.log_panel.log_reset
        )

        # Log panellerini birbirine bağla
        self.left_panel.log_panel.link_to(self.mission_page.left_panel.log_panel)

    # ── Joystick ROS Handler ──────────────────────────

    def _send_joystick_cmd(self, x, y):
        # y pozitif = ileri, x pozitif = sağ (manual_drive_panel'de speed çarpıldı)
        self.ros_worker.send_move(y, -x) # Twist: linear_x (ileri y), angular_z (sol pozitif -x)

    # ── Otonom geçiş handler'ları ─────────────────────

    def _on_autonomy_control(self, active: bool):
        """Control sayfasındaki otonom butonu — onay diyaloğundan sonra gelir."""
        mode = "AUTONOMOUS" if active else "MANUAL"
        self.ros_worker.send_drive_mode(mode)
        if active:
            self.switch_page(1)
            self.toast.show_message("✓ Otonom mod aktif — Mission ekranına geçildi")
            self.left_panel.log_mode_change("OTONOM AKTİF")
            self.mission_page.left_panel.log_mode_change("OTONOM AKTİF")
            self.mission_page.autonomy.set_active(True)
        else:
            self.toast.show_message("Otonom mod pasif")
            self.left_panel.log_mode_change("OTONOM PASİF")
            self.mission_page.autonomy.set_active(False)

    def _on_autonomy_mission(self, active: bool):
        """Mission sayfasındaki otonom butonu."""
        mode = "AUTONOMOUS" if active else "MANUAL"
        self.ros_worker.send_drive_mode(mode)
        if not active:
            self.switch_page(0)
            self.toast.show_message("Otonom mod pasif — Kontrol ekranına geçildi")
            self.mission_page.left_panel.log_mode_change("OTONOM PASİF")
            self.left_panel.log_mode_change("OTONOM PASİF")
            self.autonomy.set_active(False)
        else:
            self.toast.show_message("✓ Otonom mod aktif")
            self.mission_page.left_panel.log_mode_change("OTONOM AKTİF")
            self.autonomy.set_active(True)

    # ── Sayfa geçişi ──────────────────────────────────

    def switch_page(self, index: int):
        self.pages.setCurrentIndex(index)
        if self.btn_control and self.btn_mission:
            if index == 0:
                self.btn_control.setProperty("active", "true")
                self.btn_mission.setProperty("active", "false")
            else:
                self.btn_control.setProperty("active", "false")
                self.btn_mission.setProperty("active", "true")
            for btn in [self.btn_control, self.btn_mission]:
                btn.style().unpolish(btn)
                btn.style().polish(btn)
                btn.update()

    def on_telemetry(self, t: dict):
        for panel in [self.left_panel, self.mission_page.left_panel]:
            panel.update_telemetry(
                battery=t.get("battery"),
                battery_temp=t.get("battery_temp"),
                humidity=t.get("humidity"),
                speed=t.get("speed"),
                roll=t.get("roll"),
                pitch=t.get("pitch"),
                yaw=t.get("yaw"),
                warning=t.get("warning", "Yok")
            )
            panel.update_khadas_health(
                cpu=t.get("cpu"),
                temp=t.get("temp"),
                ram=t.get("ram"),
                disk=t.get("disk")
            )

        self.map.update_gps(t)

        if hasattr(self.mission_page, "laser_panel"):
            self.mission_page.laser_panel.update_data(
                distance=t.get("distance", 0.0),
                yaw=t.get("yaw", 0.0),
                pitch=t.get("pitch", 0.0)
            )

    def ros_connect(self):
        if not self.ros_worker.isRunning():
            self.ros_worker.start()

    def ros_disconnect(self):
        if self.ros_worker.isRunning():
            self.ros_worker.stop()
            self.ros_worker.wait()
        self.left_panel.update_connection(False)
        self.mission_page.left_panel.update_connection(False)

    def on_ros_status(self, msg):
        self.toast.show_message(f"ROS 2: {msg}")
        if "CONNECTED" in msg:
            self.left_panel.update_connection(True, ping=1)
            self.mission_page.left_panel.update_connection(True, ping=1)
        elif "DISCONNECTED" in msg or "ERROR" in msg:
            self.left_panel.update_connection(False)
            self.mission_page.left_panel.update_connection(False)

    def change_theme(self):
        pass

    def on_estop(self):
        self.toast.show_message("⚠ ACİL STOP TETİKLENDİ!")
        self.left_panel.log_panel.add_log("ACİL STOP TETİKLENDİ!", "ERROR")
        self.mission_page.left_panel.log_panel.add_log("ACİL STOP TETİKLENDİ!", "ERROR")
        self.ros_worker.send_estop()
        print("ACİL STOP BASILDI (ROS Twist -> 0)")