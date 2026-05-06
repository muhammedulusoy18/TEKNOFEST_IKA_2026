# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QFrame, QVBoxLayout, QLabel, QPushButton,
    QSizePolicy, QDialog, QHBoxLayout, QWidget
)
from PyQt5.QtCore import pyqtSignal, Qt, QTimer
from PyQt5.QtGui import QFont


class AutonomyConfirmDialog(QDialog):
    """
    Otonom moda geçiş için onay + geri sayım diyaloğu.
    Onaylanırsa 3...2...1 sayar, sonra confirmed sinyali yayar.
    """
    confirmed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.Dialog)
        self.setModal(True)
        self.setFixedSize(420, 280)
        self.setAttribute(Qt.WA_TranslucentBackground, False)
        self._countdown = 3
        self._timer = None
        self._counting = False
        self._build_ui()
        self._center_on_parent()

    def _center_on_parent(self):
        if self.parent():
            pg = self.parent().geometry()
            x = pg.x() + (pg.width()  - self.width())  // 2
            y = pg.y() + (pg.height() - self.height()) // 2
            self.move(x, y)

    def _build_ui(self):
        self.setStyleSheet("""
            QDialog {
                background-color: #0f1724;
                border: 1px solid #ef4444;
                border-radius: 14px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(30, 28, 30, 24)
        layout.setSpacing(12)

        # Uyarı ikonu + başlık
        title = QLabel("⚠  OTONOM MOD ONAYI")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            color: #fbbf24;
            font-size: 15px;
            font-weight: 900;
            letter-spacing: 2px;
        """)
        layout.addWidget(title)

        # Açıklama
        desc = QLabel(
            "Araç otonom moda geçecek.\n"
            "Tüm manuel kontroller devre dışı kalacak.\n"
            "Onaylamak için BAŞLAT butonuna basın."
        )
        desc.setAlignment(Qt.AlignCenter)
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #9ca3af; font-size: 12px; line-height: 1.6;")
        layout.addWidget(desc)

        # Geri sayım etiketi
        self.countdown_label = QLabel("")
        self.countdown_label.setAlignment(Qt.AlignCenter)
        self.countdown_label.setStyleSheet("""
            color: #ef4444;
            font-size: 42px;
            font-weight: 900;
        """)
        self.countdown_label.setFixedHeight(60)
        layout.addWidget(self.countdown_label)

        # Butonlar
        btn_row = QHBoxLayout()
        btn_row.setSpacing(12)

        self.btn_cancel = QPushButton("İPTAL")
        self.btn_cancel.setFixedHeight(40)
        self.btn_cancel.setStyleSheet("""
            QPushButton {
                background-color: #1a2232;
                color: #9ca3af;
                border: 1px solid #374151;
                border-radius: 8px;
                font-size: 13px;
                font-weight: 800;
            }
            QPushButton:hover {
                border: 1px solid #ef4444;
                color: #ef4444;
            }
        """)
        self.btn_cancel.clicked.connect(self._on_cancel)

        self.btn_confirm = QPushButton("BAŞLAT  →  3")
        self.btn_confirm.setFixedHeight(40)
        self.btn_confirm.setStyleSheet("""
            QPushButton {
                background-color: #14532d;
                color: #4ade80;
                border: 1px solid #22c55e;
                border-radius: 8px;
                font-size: 13px;
                font-weight: 900;
            }
            QPushButton:hover { background-color: #166534; }
            QPushButton:disabled {
                background-color: #052e16;
                color: #22c55e;
                border: 1px solid #166534;
            }
        """)
        self.btn_confirm.clicked.connect(self._on_confirm)

        btn_row.addWidget(self.btn_cancel)
        btn_row.addWidget(self.btn_confirm)
        layout.addLayout(btn_row)

    def _on_confirm(self):
        if self._counting:
            return
        self._counting = True
        self._countdown = 3
        self.btn_confirm.setEnabled(False)
        self.btn_cancel.setEnabled(False)
        self._tick()

    def _tick(self):
        if self._countdown <= 0:
            self.confirmed.emit()
            self.accept()
            return

        self.countdown_label.setText(str(self._countdown))
        self.btn_confirm.setText(f"BAŞLATILIYOR...  {self._countdown}")
        self._countdown -= 1
        self._timer = QTimer.singleShot(1000, self._tick)

    def _on_cancel(self):
        if self._timer:
            pass
        self._counting = False
        self.reject()


class AutonomyPanel(QFrame):
    sig_autonomy_toggled = pyqtSignal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("autonomyPanel")
        self._active = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)

        title = QLabel("● OTONOM SÜRÜŞ")
        title.setObjectName("RightSectionTitle")
        layout.addWidget(title)

        self.autonomy_button = QPushButton("OTONOM MOD")
        self.autonomy_button.setObjectName("autonomyButton")
        self.autonomy_button.setProperty("state", "inactive")
        self.autonomy_button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.autonomy_button.setFixedHeight(40)
        self.autonomy_button.clicked.connect(self._on_button_clicked)
        layout.addWidget(self.autonomy_button)

        self.status_label = QLabel("Durum: Bekleniyor")
        self.status_label.setObjectName("RightInfoLabel")
        layout.addWidget(self.status_label)

        layout.addStretch()
        self.setStyleSheet(self._styles())

    def _on_button_clicked(self):
        if not self._active:
            # Otonom moda GEÇİŞ → onay diyaloğu aç
            dlg = AutonomyConfirmDialog(self.window())
            dlg.confirmed.connect(self._activate)
            dlg.exec_()
        else:
            # Otonom moddan ÇIKIŞ → direkt
            self._toggle()

    def _activate(self):
        """Onay diyaloğundan sonra çağrılır."""
        if not self._active:
            self._toggle()

    def _toggle(self):
        self._active = not self._active
        if self._active:
            self.autonomy_button.setText("OTONOM MOD: AKTİF")
            self.autonomy_button.setProperty("state", "active")
            self.status_label.setText("Durum: Otonom aktif")
        else:
            self.autonomy_button.setText("OTONOM MOD: PASİF")
            self.autonomy_button.setProperty("state", "inactive")
            self.status_label.setText("Durum: Otonom pasif")

        self.autonomy_button.style().unpolish(self.autonomy_button)
        self.autonomy_button.style().polish(self.autonomy_button)
        self.autonomy_button.update()
        self.sig_autonomy_toggled.emit(self._active)

    def set_active(self, active: bool):
        """Dışarıdan senkronizasyon için — private _toggle yerine bunu kullan."""
        if self._active != active:
            self._toggle()

    def _styles(self):
        return """
        #autonomyPanel {
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
        QPushButton#autonomyButton[state="inactive"] {
            background-color: #7f1d1d;
            color: #fca5a5;
            border: 1px solid #ef4444;
            border-radius: 8px;
            font-size: 13px;
            font-weight: 900;
            padding: 8px;
        }
        QPushButton#autonomyButton[state="inactive"]:hover {
            background-color: #991b1b;
        }
        QPushButton#autonomyButton[state="active"] {
            background-color: #14532d;
            color: #4ade80;
            border: 1px solid #22c55e;
            border-radius: 8px;
            font-size: 13px;
            font-weight: 900;
            padding: 8px;
        }
        QPushButton#autonomyButton[state="active"]:hover {
            background-color: #166534;
        }
        #RightInfoLabel { color: #8b949e; font-size: 11px; }
        """