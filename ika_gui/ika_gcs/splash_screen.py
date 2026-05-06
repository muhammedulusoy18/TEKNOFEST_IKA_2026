# -*- coding: utf-8 -*-
import os
import sys
from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QLabel,
    QPushButton, QLineEdit, QGraphicsOpacityEffect, QApplication
)
from PyQt5.QtCore import (
    Qt, QTimer, QPropertyAnimation, QEasingCurve, pyqtSignal, QByteArray
)
from PyQt5.QtGui import QPixmap, QPainter
from PyQt5.QtSvg import QSvgWidget

CORRECT_PASSWORD = "alpagu"

LOGO_SVG = """
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 680 520">
  <defs>
    <linearGradient id="body_grad" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#1e293b"/>
      <stop offset="100%" stop-color="#0f172a"/>
    </linearGradient>
    <linearGradient id="helmet_grad" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#334155"/>
      <stop offset="100%" stop-color="#1e293b"/>
    </linearGradient>
    <linearGradient id="visor_grad" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0%" stop-color="#22d3ee" stop-opacity="0.9"/>
      <stop offset="100%" stop-color="#0891b2" stop-opacity="0.7"/>
    </linearGradient>
    <linearGradient id="wheel_grad" x1="0" y1="0" x2="0" y2="1">
      <stop offset="0%" stop-color="#374151"/>
      <stop offset="100%" stop-color="#111827"/>
    </linearGradient>
    <linearGradient id="cannon_grad" x1="0" y1="0" x2="1" y2="0">
      <stop offset="0%" stop-color="#475569"/>
      <stop offset="100%" stop-color="#1e293b"/>
    </linearGradient>
    <clipPath id="stripe_clip">
      <rect x="370" y="155" width="60" height="110"/>
    </clipPath>
  </defs>

  <polygon points="340,30 490,115 490,285 340,370 190,285 190,115"
           fill="none" stroke="#22c55e" stroke-width="1.5" opacity="0.25"/>
  <polygon points="340,52 472,127 472,277 340,352 208,277 208,127"
           fill="none" stroke="#22c55e" stroke-width="0.7" opacity="0.15"/>

  <rect x="230" y="155" width="200" height="110" rx="8" fill="url(#body_grad)" stroke="#334155" stroke-width="1.5"/>
  <path d="M245,155 Q260,110 340,100 Q420,110 435,155 Z" fill="url(#helmet_grad)" stroke="#475569" stroke-width="1.2"/>
  <path d="M268,145 Q290,128 340,124 Q390,128 412,145 L408,155 Q385,140 340,137 Q295,140 272,155 Z" fill="url(#visor_grad)" opacity="0.85"/>
  <path d="M278,138 Q300,130 330,128 L328,133 Q305,135 283,143 Z" fill="#ffffff" opacity="0.3"/>

  <rect x="305" y="120" width="70" height="38" rx="5" fill="#263048" stroke="#334155" stroke-width="1"/>
  <rect x="375" y="130" width="55" height="10" rx="3" fill="url(#cannon_grad)" stroke="#475569" stroke-width="0.8"/>
  <rect x="426" y="128" width="8" height="14" rx="2" fill="#1e293b" stroke="#334155" stroke-width="0.8"/>

  <g clip-path="url(#stripe_clip)">
    <line x1="370" y1="155" x2="430" y2="265" stroke="#eab308" stroke-width="8"/>
    <line x1="383" y1="155" x2="430" y2="237" stroke="#eab308" stroke-width="8"/>
    <line x1="357" y1="155" x2="430" y2="293" stroke="#eab308" stroke-width="8"/>
    <line x1="344" y1="162" x2="430" y2="320" stroke="#eab308" stroke-width="8"/>
  </g>

  <rect x="230" y="175" width="28" height="80" rx="4" fill="#15803d" stroke="#16a34a" stroke-width="1"/>

  <line x1="348" y1="100" x2="348" y2="76" stroke="#94a3b8" stroke-width="2" stroke-linecap="round"/>
  <line x1="358" y1="100" x2="358" y2="80" stroke="#94a3b8" stroke-width="2" stroke-linecap="round"/>
  <circle cx="348" cy="74" r="3" fill="#22c55e"/>
  <circle cx="358" cy="78" r="2.5" fill="#22c55e" opacity="0.7"/>

  <circle cx="262" cy="278" r="26" fill="url(#wheel_grad)" stroke="#475569" stroke-width="2"/>
  <circle cx="262" cy="278" r="14" fill="#0f172a" stroke="#334155" stroke-width="1.5"/>
  <circle cx="262" cy="278" r="5" fill="#475569"/>

  <circle cx="418" cy="278" r="26" fill="url(#wheel_grad)" stroke="#475569" stroke-width="2"/>
  <circle cx="418" cy="278" r="14" fill="#0f172a" stroke="#334155" stroke-width="1.5"/>
  <circle cx="418" cy="278" r="5" fill="#475569"/>

  <line x1="262" y1="265" x2="290" y2="255" stroke="#64748b" stroke-width="3" stroke-linecap="round"/>
  <line x1="418" y1="265" x2="390" y2="255" stroke="#64748b" stroke-width="3" stroke-linecap="round"/>
  <line x1="262" y1="252" x2="290" y2="265" stroke="#64748b" stroke-width="2" stroke-linecap="round"/>
  <line x1="418" y1="252" x2="390" y2="265" stroke="#64748b" stroke-width="2" stroke-linecap="round"/>

  <rect x="345" y="136" width="30" height="5" rx="2" fill="#eab308" opacity="0.9"/>

  <line x1="190" y1="115" x2="210" y2="115" stroke="#22c55e" stroke-width="1.5" opacity="0.6"/>
  <line x1="190" y1="115" x2="190" y2="135" stroke="#22c55e" stroke-width="1.5" opacity="0.6"/>
  <line x1="490" y1="115" x2="470" y2="115" stroke="#22c55e" stroke-width="1.5" opacity="0.6"/>
  <line x1="490" y1="115" x2="490" y2="135" stroke="#22c55e" stroke-width="1.5" opacity="0.6"/>
  <line x1="190" y1="285" x2="210" y2="285" stroke="#22c55e" stroke-width="1.5" opacity="0.6"/>
  <line x1="190" y1="285" x2="190" y2="265" stroke="#22c55e" stroke-width="1.5" opacity="0.6"/>
  <line x1="490" y1="285" x2="470" y2="285" stroke="#22c55e" stroke-width="1.5" opacity="0.6"/>
  <line x1="490" y1="285" x2="490" y2="265" stroke="#22c55e" stroke-width="1.5" opacity="0.6"/>

  <text x="340" y="420" text-anchor="middle"
        font-family="monospace" font-size="48" font-weight="bold"
        fill="#22c55e" letter-spacing="12">M&#304;&#286;FER</text>

  <text x="340" y="460" text-anchor="middle"
        font-family="monospace" font-size="16"
        fill="#3b82f6" letter-spacing="5" opacity="0.85">ALPAGU TEAM &#8226; IKA</text>
</svg>
"""


class CloseButton(QPushButton):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(28, 28)
        self.setCursor(Qt.PointingHandCursor)
        self.setStyleSheet("""
            QPushButton {
                background-color: rgba(239, 68, 68, 0.15);
                border: 1px solid rgba(239, 68, 68, 0.25);
                border-radius: 14px;
            }
            QPushButton:hover {
                background-color: #ef4444;
                border: 1px solid #ef4444;
            }
            QPushButton:pressed {
                background-color: #b91c1c;
            }
        """)

    def paintEvent(self, event):
        super().paintEvent(event)
        from PyQt5.QtGui import QPainter, QPen, QColor
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        if self.underMouse():
            painter.setPen(QPen(QColor("#ffffff"), 2, Qt.SolidLine, Qt.RoundCap))
        else:
            painter.setPen(QPen(QColor("#6b7280"), 1.5, Qt.SolidLine, Qt.RoundCap))
        cx, cy = self.width() // 2, self.height() // 2
        offset = 5
        painter.drawLine(cx - offset, cy - offset, cx + offset, cy + offset)
        painter.drawLine(cx + offset, cy - offset, cx - offset, cy + offset)
        painter.end()


class SplashScreen(QWidget):
    login_success = pyqtSignal()

    def __init__(self, vehicle_image_path: str = "assets/vehicle.jpg"):
        super().__init__()
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.setFixedSize(1100, 660)
        self._vehicle_path = vehicle_image_path
        self._center_on_screen()
        self._build_ui()

        self._opacity_effect = QGraphicsOpacityEffect(self)
        self.setGraphicsEffect(self._opacity_effect)
        self._opacity_effect.setOpacity(0.0)
        self._anim = QPropertyAnimation(self._opacity_effect, b"opacity")
        self._anim.setDuration(800)
        self._anim.setStartValue(0.0)
        self._anim.setEndValue(1.0)
        self._anim.setEasingCurve(QEasingCurve.InOutQuad)
        self._anim.start()

    def _center_on_screen(self):
        screen = QApplication.primaryScreen().geometry()
        self.move(
            (screen.width()  - self.width())  // 2,
            (screen.height() - self.height()) // 2,
        )

    def _build_ui(self):
        self.setStyleSheet("background-color: #050d1a;")

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── SOL: Araç görseli + takım adı ─────────────
        left = QWidget()
        left.setFixedWidth(620)
        left.setFixedHeight(660)
        left.setStyleSheet("background-color: #070f1e;")
        root.addWidget(left)

        self.vehicle_bg = QLabel(left)
        self.vehicle_bg.setGeometry(0, 0, 620, 660)
        self.vehicle_bg.setAlignment(Qt.AlignCenter)
        self._load_vehicle(self.vehicle_bg, 620, 660, opacity=0.13)

        team = QLabel("ALPAGU TEAM", left)
        team.setGeometry(0, 50, 620, 70)
        team.setAlignment(Qt.AlignCenter)
        team.setStyleSheet("""
            color: rgba(34, 197, 94, 230);
            font-size: 42px;
            font-weight: 900;
            letter-spacing: 12px;
            background: transparent;
        """)

        sub = QLabel("IKA GROUND CONTROL STATION", left)
        sub.setGeometry(0, 122, 620, 28)
        sub.setAlignment(Qt.AlignCenter)
        sub.setStyleSheet("""
            color: rgba(96, 165, 250, 180);
            font-size: 13px;
            font-weight: 700;
            letter-spacing: 5px;
            background: transparent;
        """)

        ver = QLabel("v1.0.0  •  TEKNOFEST 2026", left)
        ver.setGeometry(0, 618, 620, 28)
        ver.setAlignment(Qt.AlignCenter)
        ver.setStyleSheet("""
            color: rgba(75, 85, 99, 180);
            font-size: 11px;
            font-weight: 600;
            letter-spacing: 2px;
            background: transparent;
        """)

        # ── SAĞ: Login paneli ──────────────────────────
        right = QWidget()
        right.setFixedWidth(480)
        right.setStyleSheet("background-color: #080e18; border-left: 1px solid #1f2a3d;")
        root.addWidget(right)

        r_layout = QVBoxLayout(right)
        r_layout.setContentsMargins(40, 0, 40, 0)
        r_layout.setSpacing(0)

        # Sağ üst köşe kapatma butonu
        top_bar = QHBoxLayout()
        top_bar.setContentsMargins(0, 14, 0, 0)
        top_bar.addStretch()
        self.close_btn = CloseButton()
        self.close_btn.clicked.connect(QApplication.quit)
        top_bar.addWidget(self.close_btn)
        r_layout.addLayout(top_bar)

        r_layout.addStretch(1)

        # SVG Logo — büyütüldü
        self.logo_svg = QSvgWidget()
        self.logo_svg.load(QByteArray(LOGO_SVG.encode("utf-8")))
        self.logo_svg.setFixedSize(360, 290)
        self.logo_svg.setStyleSheet("background: transparent;")
        r_layout.addWidget(self.logo_svg, 0, Qt.AlignHCenter)
        r_layout.addSpacing(16)

        title = QLabel("Güvenli Giriş")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            color: #e5e7eb;
            font-size: 22px;
            font-weight: 800;
            background: transparent;
        """)
        r_layout.addWidget(title)
        r_layout.addSpacing(4)

        desc = QLabel("Sisteme erişmek için şifrenizi girin")
        desc.setAlignment(Qt.AlignCenter)
        desc.setStyleSheet("""
            color: #6b7280;
            font-size: 12px;
            background: transparent;
        """)
        r_layout.addWidget(desc)
        r_layout.addSpacing(20)

        pw_label = QLabel("SİSTEM ŞİFRESİ")
        pw_label.setStyleSheet("""
            color: #9ca3af;
            font-size: 11px;
            font-weight: 700;
            letter-spacing: 1.5px;
            background: transparent;
        """)
        r_layout.addWidget(pw_label)
        r_layout.addSpacing(8)

        self.pw_input = QLineEdit()
        self.pw_input.setEchoMode(QLineEdit.Password)
        self.pw_input.setPlaceholderText("••••••••••")
        self.pw_input.setFixedHeight(46)
        self.pw_input.setStyleSheet("""
            QLineEdit {
                background-color: #111827;
                color: #e5e7eb;
                border: 1px solid #374151;
                border-radius: 10px;
                padding: 0 16px;
                font-size: 18px;
                letter-spacing: 4px;
            }
            QLineEdit:focus {
                border: 1px solid #22c55e;
                background-color: #131f30;
            }
        """)
        self.pw_input.returnPressed.connect(self._check_password)
        r_layout.addWidget(self.pw_input)
        r_layout.addSpacing(10)

        self.pw_error = QLabel("")
        self.pw_error.setAlignment(Qt.AlignCenter)
        self.pw_error.setFixedHeight(18)
        self.pw_error.setStyleSheet("""
            color: #ef4444;
            font-size: 12px;
            font-weight: 600;
            background: transparent;
        """)
        r_layout.addWidget(self.pw_error)
        r_layout.addSpacing(10)

        self.pw_btn = QPushButton("SİSTEME GİR")
        self.pw_btn.setFixedHeight(46)
        self.pw_btn.setStyleSheet("""
            QPushButton {
                background-color: #15803d;
                color: #dcfce7;
                border: none;
                border-radius: 10px;
                font-size: 14px;
                font-weight: 900;
                letter-spacing: 2px;
            }
            QPushButton:hover { background-color: #16a34a; }
            QPushButton:pressed { background-color: #166534; }
        """)
        self.pw_btn.clicked.connect(self._check_password)
        r_layout.addWidget(self.pw_btn)

        r_layout.addStretch(1)

        footer = QLabel("ALPAGU TEAM  •  TEKNOFEST IKA 2026")
        footer.setAlignment(Qt.AlignCenter)
        footer.setStyleSheet("""
            color: #374151;
            font-size: 11px;
            letter-spacing: 1px;
            background: transparent;
        """)
        r_layout.addWidget(footer)
        r_layout.addSpacing(20)

    def _load_vehicle(self, label: QLabel, w: int, h: int, opacity: float = 0.13):
        paths = [
            self._vehicle_path,
            os.path.join(os.path.dirname(os.path.abspath(__file__)), self._vehicle_path),
            os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])), self._vehicle_path),
        ]
        pixmap = QPixmap()
        for p in paths:
            if os.path.exists(p):
                pixmap = QPixmap(p)
                break

        if pixmap.isNull():
            label.setText("")
            return

        scaled = pixmap.scaled(w, h, Qt.KeepAspectRatioByExpanding, Qt.SmoothTransformation)
        x_offset = (scaled.width()  - w) // 2
        y_offset = (scaled.height() - h) // 2
        cropped = scaled.copy(x_offset, y_offset, w, h)

        result = QPixmap(cropped.size())
        result.fill(Qt.transparent)
        painter = QPainter(result)
        painter.setOpacity(opacity)
        painter.drawPixmap(0, 0, cropped)
        painter.end()

        label.setPixmap(result)

    def _check_password(self):
        if self.pw_input.text().strip() == CORRECT_PASSWORD:
            self.pw_error.setText("")
            self._fade_out()
        else:
            self.pw_error.setText("✕  Hatalı şifre, tekrar deneyin")
            self.pw_input.clear()
            self.pw_input.setFocus()
            self._shake()

    def _shake(self):
        orig_x = self.x()
        for i, dx in enumerate([10, -10, 7, -7, 4, -4, 0]):
            QTimer.singleShot(i * 45, lambda d=dx, ox=orig_x: self.move(ox + d, self.y()))

    def _fade_out(self):
        self._fade_out_anim = QPropertyAnimation(self._opacity_effect, b"opacity")
        self._fade_out_anim.setDuration(500)
        self._fade_out_anim.setStartValue(1.0)
        self._fade_out_anim.setEndValue(0.0)
        self._fade_out_anim.setEasingCurve(QEasingCurve.InOutQuad)
        self._fade_out_anim.finished.connect(self._on_done)
        self._fade_out_anim.start()

    def _on_done(self):
        self.close()
        self.login_success.emit()