# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QFrame, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QSizePolicy
)
from PyQt5.QtCore import pyqtSignal, Qt


class LaserControlPanel(QFrame):
    sig_laser_toggled = pyqtSignal(str)
    sig_lock_toggled  = pyqtSignal(str)
    sig_reset         = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("laserControlPanel")

        self.laser_enabled = False
        self.target_locked = False
        self.distance      = 0.0

        self._shot_count = 0
        self._hit_count  = 0

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 14, 16, 14)
        root.setSpacing(10)

        title = QLabel("● LAZER KONTROL")
        title.setObjectName("RightSectionTitle")
        root.addWidget(title)

        self.lbl_status   = self._info_row("Durum",   "PASİF")
        self.lbl_lock     = self._info_row("Kilit",   "YOK")
        self.lbl_distance = self._info_row("Mesafe",  "0.0 m")
        self.lbl_angles   = self._info_row("Açı",     "Yaw 0.0° · Pitch 0.0°")

        for row in [self.lbl_status, self.lbl_lock, self.lbl_distance, self.lbl_angles]:
            root.addWidget(row)

        # ── İsabet sayacı ──────────────────────────
        counter_frame = QFrame()
        counter_frame.setObjectName("CounterFrame")
        counter_layout = QHBoxLayout(counter_frame)
        counter_layout.setContentsMargins(10, 8, 10, 8)
        counter_layout.setSpacing(0)

        self._shot_lbl  = self._counter_box("ATEŞ", "0")
        self._hit_lbl   = self._counter_box("İSABET", "0")
        self._ratio_lbl = self._counter_box("ORAN", "-%")

        counter_layout.addWidget(self._shot_lbl)
        counter_layout.addWidget(self._make_divider())
        counter_layout.addWidget(self._hit_lbl)
        counter_layout.addWidget(self._make_divider())
        counter_layout.addWidget(self._ratio_lbl)

        root.addWidget(counter_frame)

        # Butonlar
        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        self.btn_toggle = QPushButton("LAZER AÇ")
        self.btn_toggle.setObjectName("laserPrimaryButton")
        self.btn_toggle.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_toggle.setFixedHeight(36)
        self.btn_toggle.clicked.connect(self.toggle_laser)

        self.btn_lock = QPushButton("HEDEF KİLİTLE")
        self.btn_lock.setObjectName("laserSecondaryButton")
        self.btn_lock.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_lock.setFixedHeight(36)
        self.btn_lock.clicked.connect(self.toggle_lock)

        btn_row.addWidget(self.btn_toggle)
        btn_row.addWidget(self.btn_lock)
        root.addLayout(btn_row)

        self.btn_reset = QPushButton("SIFIRLA")
        self.btn_reset.setObjectName("laserResetButton")
        self.btn_reset.setFixedHeight(32)
        self.btn_reset.clicked.connect(self.reset_panel)
        root.addWidget(self.btn_reset)

        root.addStretch()
        self.setStyleSheet(self._styles())

    def _info_row(self, label_text, value_text):
        row = QFrame()
        row.setObjectName("LaserInfoRow")
        layout = QHBoxLayout(row)
        layout.setContentsMargins(10, 5, 10, 5)
        layout.setSpacing(0)
        lbl = QLabel(label_text)
        lbl.setObjectName("LaserInfoKey")
        val = QLabel(value_text)
        val.setObjectName("LaserInfoVal")
        layout.addWidget(lbl)
        layout.addStretch()
        layout.addWidget(val)
        row.val_label = val
        return row

    def _counter_box(self, title: str, value: str) -> QFrame:
        box = QFrame()
        box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QVBoxLayout(box)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        title_lbl = QLabel(title)
        title_lbl.setObjectName("CounterTitle")
        title_lbl.setAlignment(Qt.AlignCenter)

        val_lbl = QLabel(value)
        val_lbl.setObjectName("CounterValue")
        val_lbl.setAlignment(Qt.AlignCenter)

        layout.addWidget(title_lbl)
        layout.addWidget(val_lbl)
        box.val_label = val_lbl
        return box

    def _make_divider(self) -> QFrame:
        div = QFrame()
        div.setFixedWidth(1)
        div.setStyleSheet("background-color: #2d3748;")
        return div

    def toggle_laser(self):
        self.laser_enabled = not self.laser_enabled
        if self.laser_enabled:
            self._shot_count += 1
            if self.target_locked:
                self._hit_count += 1
            self._update_counter()
            self.lbl_status.val_label.setText("AKTİF")
            self.lbl_status.val_label.setStyleSheet("color: #4ade80; font-weight: 800;")
            self.btn_toggle.setText("LAZER KAPAT")
            self.sig_laser_toggled.emit("Lazer AKTİF")
        else:
            self.lbl_status.val_label.setText("PASİF")
            self.lbl_status.val_label.setStyleSheet("color: #9ca3af; font-weight: 800;")
            self.btn_toggle.setText("LAZER AÇ")
            self.sig_laser_toggled.emit("Lazer PASİF")

    def toggle_lock(self):
        self.target_locked = not self.target_locked
        if self.target_locked:
            self.lbl_lock.val_label.setText("AKTİF")
            self.lbl_lock.val_label.setStyleSheet("color: #f87171; font-weight: 800;")
            self.btn_lock.setText("KİLİDİ AÇ")
            self.sig_lock_toggled.emit("Hedef kilitlendi")
        else:
            self.lbl_lock.val_label.setText("YOK")
            self.lbl_lock.val_label.setStyleSheet("color: #9ca3af; font-weight: 800;")
            self.btn_lock.setText("HEDEF KİLİTLE")
            self.sig_lock_toggled.emit("Kilit açıldı")

    def reset_panel(self):
        self.laser_enabled = False
        self.target_locked = False
        self.distance      = 0.0
        self._shot_count   = 0
        self._hit_count    = 0

        self.lbl_status.val_label.setText("PASİF")
        self.lbl_status.val_label.setStyleSheet("color: #9ca3af; font-weight: 800;")
        self.lbl_lock.val_label.setText("YOK")
        self.lbl_lock.val_label.setStyleSheet("color: #9ca3af; font-weight: 800;")
        self.lbl_distance.val_label.setText("0.0 m")
        self.lbl_angles.val_label.setText("Yaw 0.0° · Pitch 0.0°")
        self.btn_toggle.setText("LAZER AÇ")
        self.btn_lock.setText("HEDEF KİLİTLE")
        self._update_counter()
        self.sig_reset.emit()

    def _update_counter(self):
        self._shot_lbl.val_label.setText(str(self._shot_count))
        self._hit_lbl.val_label.setText(str(self._hit_count))

        if self._shot_count > 0:
            ratio = int(self._hit_count / self._shot_count * 100)
            color = "#4ade80" if ratio >= 70 else "#fbbf24" if ratio >= 40 else "#f87171"
            self._ratio_lbl.val_label.setText(f"%{ratio}")
            self._ratio_lbl.val_label.setStyleSheet(
                f"color: {color}; font-size: 16px; font-weight: 900;"
            )
        else:
            self._ratio_lbl.val_label.setText("-%")
            self._ratio_lbl.val_label.setStyleSheet(
                "color: #6b7280; font-size: 16px; font-weight: 900;"
            )

    def update_data(self, distance=0.0, yaw=0.0, pitch=0.0):
        self.distance = float(distance)
        self.lbl_distance.val_label.setText(f"{self.distance:.1f} m")
        self.lbl_angles.val_label.setText(f"Yaw {yaw:.1f}° · Pitch {pitch:.1f}°")

    def _styles(self):
        return """
        #laserControlPanel {
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
        #LaserInfoRow {
            background-color: #171f2e;
            border: 1px solid #2d3748;
            border-radius: 7px;
        }
        #LaserInfoKey { color: #8b949e; font-size: 11px; font-weight: 700; }
        #LaserInfoVal { color: #9ca3af; font-size: 12px; font-weight: 800; }

        #CounterFrame {
            background-color: #0f1724;
            border: 1px solid #2d3748;
            border-radius: 8px;
        }
        #CounterTitle {
            color: #6b7280;
            font-size: 10px;
            font-weight: 700;
            letter-spacing: 1px;
        }
        #CounterValue {
            color: #e5e7eb;
            font-size: 20px;
            font-weight: 900;
        }

        QPushButton { border-radius: 8px; font-weight: 900; font-size: 12px; }

        #laserPrimaryButton {
            background-color: #7f1d1d;
            color: #fca5a5;
            border: 1px solid #ef4444;
        }
        #laserPrimaryButton:hover { background-color: #991b1b; }

        #laserSecondaryButton {
            background-color: #1e3a5f;
            color: #60a5fa;
            border: 1px solid #3b82f6;
        }
        #laserSecondaryButton:hover { background-color: #1d4ed8; color: #bfdbfe; }

        #laserResetButton {
            background-color: #1a2232;
            color: #6b7280;
            border: 1px solid #374151;
        }
        #laserResetButton:hover { color: #f87171; border: 1px solid rgba(239,68,68,0.5); }
        """