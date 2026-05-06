from PyQt5.QtWidgets import QDockWidget, QWidget, QVBoxLayout, QPlainTextEdit


class LogDock(QDockWidget):
    def __init__(self, parent=None):
        super().__init__("Logs", parent)
        w = QWidget()
        layout = QVBoxLayout(w)

        self.box = QPlainTextEdit()
        self.box.setReadOnly(True)
        self.box.setMaximumBlockCount(1000)

        layout.addWidget(self.box)
        self.setWidget(w)

    def append_log(self, level: str, msg: str):
        self.box.appendPlainText(f"[{level}] {msg}")
