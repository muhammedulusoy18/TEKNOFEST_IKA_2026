from PyQt5.QtCore import Qt, pyqtSignal, QPointF, QRectF, QTimer
from PyQt5.QtGui import (
    QPainter, QPen, QBrush, QColor, QRadialGradient, QPolygonF
)
from PyQt5.QtWidgets import QWidget
import math


class JoystickWidget(QWidget):
    """
    Mouse ile kontrol edilen joystick
    - -1..+1 arası x,y üretir
    - Deadzone vardır
    - Yumuşak renk animasyonlu cyber joystick
    """

    valueChanged = pyqtSignal(float, float)

    def __init__(self, size=180, parent=None):
        super().__init__(parent)

        self.setFixedSize(size, size)

        self._center = QPointF(size / 2, size / 2)
        self._radius = size / 2 - 10
        self._knob_radius = max(14, int(size * 0.12))
        self._knob_pos = QPointF(self._center)

        self._dragging = False
        self.deadzone = 0.05

        # -----------------------------
        # Static theme colors
        # -----------------------------
        self.bg_outer = QColor("#08101f")
        self.bg_inner = QColor("#111a2e")
        self.ring_base = QColor("#1b2744")
        self.crosshair = QColor("#2a3b63")
        self.knob_edge = QColor("#ecf5ff")
        self.knob_mid = QColor("#a9b8cc")
        self.knob_core = QColor("#dbeafe")

        # -----------------------------
        # Animation state
        # -----------------------------
        self._phase = 0.0
        self._pulse = 0.0
        self._glow_boost = 0.0

        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._tick_animation)
        self._anim_timer.start(30)

    # =====================================================
    # ANIMATION
    # =====================================================

    def _tick_animation(self):
        self._phase += 0.02
        self._pulse += 0.08

        if self._phase > math.tau:
            self._phase -= math.tau

        if self._pulse > math.tau:
            self._pulse -= math.tau

        # dragging sonrası glow'u yavaşça geri düşür
        self._glow_boost *= 0.94
        if self._glow_boost < 0.01:
            self._glow_boost = 0.0

        self.update()

    def _blend(self, c1: QColor, c2: QColor, t: float) -> QColor:
        t = max(0.0, min(1.0, t))
        r = int(c1.red() + (c2.red() - c1.red()) * t)
        g = int(c1.green() + (c2.green() - c1.green()) * t)
        b = int(c1.blue() + (c2.blue() - c1.blue()) * t)
        a = int(c1.alpha() + (c2.alpha() - c1.alpha()) * t)
        return QColor(r, g, b, a)

    def _animated_color(self) -> QColor:
        """
        cyan -> blue -> purple -> cyan
        """
        cyan = QColor("#22d3ee")
        blue = QColor("#3b82f6")
        purple = QColor("#a855f7")

        cycle = (math.sin(self._phase) + 1.0) / 2.0
        cycle2 = (math.sin(self._phase + 2.1) + 1.0) / 2.0

        if cycle < 0.5:
            t = cycle / 0.5
            return self._blend(cyan, blue, t)
        else:
            t = (cycle - 0.5) / 0.5
            mid = self._blend(blue, purple, t)
            return self._blend(mid, cyan, cycle2 * 0.35)

    def _secondary_color(self) -> QColor:
        base = self._animated_color()
        return self._blend(base, QColor("#ffffff"), 0.25)

    # =====================================================
    # DRAW
    # =====================================================

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        accent = self._animated_color()
        accent2 = self._secondary_color()

        self._draw_base(p)
        self._draw_pulse_ring(p, accent)
        self._draw_outer_ring(p, accent, accent2)
        self._draw_inner_ring(p, accent)
        self._draw_crosshair(p, accent)
        self._draw_arrows(p, accent)
        self._draw_direction_line(p, accent)
        self._draw_knob(p, accent, accent2)

    def _draw_base(self, p: QPainter):
        base_grad = QRadialGradient(self._center, self._radius * 1.1)
        base_grad.setColorAt(0.0, QColor("#13203a"))
        base_grad.setColorAt(0.55, self.bg_inner)
        base_grad.setColorAt(1.0, self.bg_outer)

        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(base_grad))
        p.drawEllipse(self._center, self._radius, self._radius)

        p.setPen(QPen(QColor("#22345a"), 1.3))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(self._center, self._radius, self._radius)

    def _draw_pulse_ring(self, p: QPainter, accent: QColor):
        pulse = (math.sin(self._pulse) + 1.0) / 2.0
        extra = 4 + pulse * 4
        alpha = int(26 + pulse * 40 + self._glow_boost * 55)

        glow_color = QColor(accent)
        glow_color.setAlpha(alpha)

        p.setPen(QPen(glow_color, 3))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(self._center, self._radius - 3 + extra * 0.15, self._radius - 3 + extra * 0.15)

    def _draw_outer_ring(self, p: QPainter, accent: QColor, accent2: QColor):
        outer_r = self._radius - 4

        p.setPen(QPen(self.ring_base, 10))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(self._center, outer_r, outer_r)

        arc_r = self._radius - 5
        arc_rect = QRectF(
            self._center.x() - arc_r,
            self._center.y() - arc_r,
            arc_r * 2,
            arc_r * 2
        )

        segments = [
            (12, 48, accent),
            (82, 36, accent2),
            (150, 42, accent),
            (225, 54, accent2),
            (307, 30, accent),
        ]

        width = 3.2 + self._glow_boost * 1.0

        for start_deg, span_deg, color in segments:
            c = QColor(color)
            c.setAlpha(245)
            pen = QPen(c, width)
            pen.setCapStyle(Qt.RoundCap)
            p.setPen(pen)
            p.drawArc(arc_rect, int(start_deg * 16), int(-span_deg * 16))

    def _draw_inner_ring(self, p: QPainter, accent: QColor):
        inner_r = self._radius * 0.68

        p.setPen(QPen(QColor("#173056"), 2))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(self._center, inner_r, inner_r)

        faint = QColor(accent)
        faint.setAlpha(70 + int(self._glow_boost * 50))

        p.setPen(QPen(faint, 1.6))
        p.drawEllipse(self._center, inner_r - 10, inner_r - 10)

    def _draw_crosshair(self, p: QPainter, accent: QColor):
        line_color = QColor(self.crosshair)
        p.setPen(QPen(line_color, 1))

        gap = self._knob_radius + 10
        r = self._radius - 18

        p.drawLine(
            QPointF(self._center.x(), self._center.y() - r),
            QPointF(self._center.x(), self._center.y() - gap)
        )
        p.drawLine(
            QPointF(self._center.x(), self._center.y() + gap),
            QPointF(self._center.x(), self._center.y() + r)
        )
        p.drawLine(
            QPointF(self._center.x() - r, self._center.y()),
            QPointF(self._center.x() - gap, self._center.y())
        )
        p.drawLine(
            QPointF(self._center.x() + gap, self._center.y()),
            QPointF(self._center.x() + r, self._center.y())
        )

        faint = QColor(accent)
        faint.setAlpha(90)
        p.setPen(QPen(faint, 1.1))
        p.drawEllipse(self._center, 11, 11)

    def _draw_arrows(self, p: QPainter, accent: QColor):
        p.setPen(Qt.NoPen)

        arrow = QColor(accent)
        arrow.setAlpha(220)
        p.setBrush(QBrush(arrow))

        size = 7
        dist = self._radius - 19

        self._draw_triangle(p, QPointF(self._center.x(), self._center.y() - dist), size, "up")
        self._draw_triangle(p, QPointF(self._center.x(), self._center.y() + dist), size, "down")
        self._draw_triangle(p, QPointF(self._center.x() - dist, self._center.y()), size, "left")
        self._draw_triangle(p, QPointF(self._center.x() + dist, self._center.y()), size, "right")

    def _draw_triangle(self, p: QPainter, center: QPointF, size: int, direction: str):
        x = center.x()
        y = center.y()

        if direction == "up":
            poly = QPolygonF([
                QPointF(x, y - size),
                QPointF(x - size, y + size * 0.7),
                QPointF(x + size, y + size * 0.7),
            ])
        elif direction == "down":
            poly = QPolygonF([
                QPointF(x, y + size),
                QPointF(x - size, y - size * 0.7),
                QPointF(x + size, y - size * 0.7),
            ])
        elif direction == "left":
            poly = QPolygonF([
                QPointF(x - size, y),
                QPointF(x + size * 0.7, y - size),
                QPointF(x + size * 0.7, y + size),
            ])
        else:
            poly = QPolygonF([
                QPointF(x + size, y),
                QPointF(x - size * 0.7, y - size),
                QPointF(x - size * 0.7, y + size),
            ])

        p.drawPolygon(poly)

    def _draw_direction_line(self, p: QPainter, accent: QColor):
        dx = self._knob_pos.x() - self._center.x()
        dy = self._knob_pos.y() - self._center.y()
        dist = math.hypot(dx, dy)

        if dist < 1:
            return

        line = QColor(accent)
        line.setAlpha(180 + int(self._glow_boost * 50))

        pen = QPen(line, 2.2 + self._glow_boost * 0.8)
        pen.setCapStyle(Qt.RoundCap)
        p.setPen(pen)
        p.drawLine(self._center, self._knob_pos)

    def _draw_knob(self, p: QPainter, accent: QColor, accent2: QColor):
        # glow
        glow_r = self._knob_radius * (1.8 + self._glow_boost * 0.25)
        glow = QRadialGradient(self._knob_pos, glow_r)
        glow.setColorAt(0.0, QColor(accent.red(), accent.green(), accent.blue(), 95 + int(self._glow_boost * 60)))
        glow.setColorAt(0.45, QColor(accent.red(), accent.green(), accent.blue(), 35))
        glow.setColorAt(1.0, QColor(accent.red(), accent.green(), accent.blue(), 0))

        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(glow))
        p.drawEllipse(self._knob_pos, glow_r, glow_r)

        # outer shell
        outer_grad = QRadialGradient(
            self._knob_pos.x() - self._knob_radius * 0.35,
            self._knob_pos.y() - self._knob_radius * 0.35,
            self._knob_radius * 1.6
        )
        outer_grad.setColorAt(0.0, self.knob_edge)
        outer_grad.setColorAt(0.42, self.knob_mid)
        outer_grad.setColorAt(1.0, QColor("#6b7c93"))

        edge = QColor(accent2)
        edge.setAlpha(240)
        p.setPen(QPen(edge, 1.6))
        p.setBrush(QBrush(outer_grad))
        p.drawEllipse(self._knob_pos, self._knob_radius, self._knob_radius)

        # core
        core_r = self._knob_radius * 0.46
        core_grad = QRadialGradient(
            self._knob_pos.x() - core_r * 0.25,
            self._knob_pos.y() - core_r * 0.25,
            core_r * 1.4
        )
        core_grad.setColorAt(0.0, QColor("#ffffff"))
        core_grad.setColorAt(0.35, QColor(accent2))
        core_grad.setColorAt(1.0, QColor(accent))

        p.setPen(QPen(QColor("#dbeafe"), 1))
        p.setBrush(QBrush(core_grad))
        p.drawEllipse(self._knob_pos, core_r, core_r)

        # center shine
        shine_r = core_r * 0.35
        shine = QRadialGradient(
            self._knob_pos.x() - shine_r * 0.2,
            self._knob_pos.y() - shine_r * 0.2,
            shine_r * 1.3
        )
        shine.setColorAt(0.0, QColor(255, 255, 255, 220))
        shine.setColorAt(1.0, QColor(255, 255, 255, 0))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(shine))
        p.drawEllipse(self._knob_pos, shine_r, shine_r)

    # =====================================================
    # MOUSE EVENTS
    # =====================================================

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._glow_boost = 1.0
            self._set_knob(event.pos())

    def mouseMoveEvent(self, event):
        if self._dragging:
            self._glow_boost = min(1.0, self._glow_boost + 0.08)
            self._set_knob(event.pos())

    def mouseReleaseEvent(self, event):
        if self._dragging and event.button() == Qt.LeftButton:
            self._dragging = False
            self._knob_pos = QPointF(self._center)
            self.update()
            self.valueChanged.emit(0.0, 0.0)

    # =====================================================
    # JOYSTICK LOGIC
    # =====================================================

    def _set_knob(self, pos):
        dx = pos.x() - self._center.x()
        dy = pos.y() - self._center.y()

        max_r = self._radius - self._knob_radius - 6
        dist2 = dx * dx + dy * dy

        if dist2 > max_r * max_r:
            dist = math.sqrt(dist2)
            dx = dx / dist * max_r
            dy = dy / dist * max_r

        self._knob_pos = QPointF(
            self._center.x() + dx,
            self._center.y() + dy
        )

        self.update()

        nx = dx / max_r if max_r else 0.0
        ny = -(dy / max_r) if max_r else 0.0

        if abs(nx) < self.deadzone:
            nx = 0.0

        if abs(ny) < self.deadzone:
            ny = 0.0

        self.valueChanged.emit(float(nx), float(ny))