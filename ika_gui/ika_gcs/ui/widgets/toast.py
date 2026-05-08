from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QLabel


class Toast(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("Toast")
        self.setAlignment(Qt.AlignCenter)
        self.hide()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.hide)

    def show_message(self, text: str, kind: str = "info", ms: int = 1600):
        # kind: info / warn / danger / ok
        self.setProperty("kind", kind)
        self.setText(text)

        # QSS property refresh
        self.style().unpolish(self)
        self.style().polish(self)

        self.adjustSize()

        p = self.parent()
        if p:
            # Parent içine göre konumlandır
            x = (p.width() - self.width()) // 2
            y = 58
            self.move(x, y)

        self.show()
        self.raise_()
        self._timer.start(ms)
