from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont
from PyQt5.QtWidgets import QWidget
import math


class GyroWidget(QWidget):
    def __init__(self, size=260, parent=None):
        super().__init__(parent)
        self.setMinimumSize(size, size)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.bg = QColor("#0b1020")
        self.card = QColor("#111a2e")
        self.grid = QColor("#22345a")
        self.accent = QColor("#22d3ee")
        self.accent2 = QColor("#3b82f6")
        self.text = QColor("#e6eaf2")
        self.muted = QColor("#94a3b8")
        self.warn = QColor("#facc15")

    def set_attitude(self, roll=0.0, pitch=0.0, yaw=0.0):
        self.roll = float(roll)
        self.pitch = float(pitch)
        self.yaw = float(yaw)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        w = self.width()
        h = self.height()
        cx = w / 2
        cy = h / 2
        r = min(w, h) * 0.34

        # dış halka
        p.setPen(QPen(self.grid, 2))
        p.setBrush(QBrush(self.card))
        p.drawEllipse(int(cx - r), int(cy - r), int(r * 2), int(r * 2))

        # dış işaret halkası
        p.setPen(QPen(self.accent2, 2))
        for deg in range(0, 360, 30):
            rad = math.radians(deg)
            x1 = cx + math.cos(rad) * (r + 6)
            y1 = cy + math.sin(rad) * (r + 6)
            x2 = cx + math.cos(rad) * (r + 16)
            y2 = cy + math.sin(rad) * (r + 16)
            p.drawLine(int(x1), int(y1), int(x2), int(y2))

        # pitch horizon çizgileri (roll ile döndürülmüş hissi)
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll)

        pitch_offset = max(-40, min(40, self.pitch * 1.4))

        p.setPen(QPen(self.grid, 1))
        for y in range(-60, 61, 15):
            yy = y + pitch_offset
            p.drawLine(int(-r * 0.75), int(yy), int(r * 0.75), int(yy))

        # ana horizon
        p.setPen(QPen(self.accent, 2))
        p.drawLine(int(-r * 0.9), int(pitch_offset), int(r * 0.9), int(pitch_offset))
        p.restore()

        # merkez nişangah
        p.setPen(QPen(self.accent, 2))
        p.drawLine(int(cx - 28), int(cy), int(cx - 8), int(cy))
        p.drawLine(int(cx + 8), int(cy), int(cx + 28), int(cy))
        p.drawLine(int(cx), int(cy - 10), int(cx), int(cy + 10))

        # merkez halka
        p.setPen(QPen(self.text, 2))
        p.setBrush(QBrush(QColor("#dbeafe")))
        p.drawEllipse(int(cx - 8), int(cy - 8), 16, 16)

        # yaw pointer
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(self.warn))
        tri = [
            (cx, cy - r - 18),
            (cx - 7, cy - r - 4),
            (cx + 7, cy - r - 4),
        ]
        from PyQt5.QtGui import QPolygonF
        from PyQt5.QtCore import QPointF
        p.drawPolygon(QPolygonF([QPointF(x, y) for x, y in tri]))

        # yazılar
        p.setPen(self.text)
        font = QFont()
        font.setPointSize(10)
        font.setBold(True)
        p.setFont(font)
        p.drawText(14, 24, "JİROSKOP")

        font.setPointSize(9)
        font.setBold(False)
        p.setFont(font)
        p.setPen(self.muted)
        p.drawText(14, h - 36, f"Roll : {self.roll:.1f}°")
        p.drawText(14, h - 20, f"Pitch: {self.pitch:.1f}°")
        p.drawText(w - 95, h - 20, f"Yaw: {self.yaw:.1f}°")