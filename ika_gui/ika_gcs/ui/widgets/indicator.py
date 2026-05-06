from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel


class Indicator(QFrame):
    def __init__(self, label: str, value: str, accent: str = "blue"):
        super().__init__()

        self.setObjectName("Indicator")

        layout = QHBoxLayout()
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(10)

        self.lbl = QLabel(label)
        self.lbl.setObjectName("IndicatorLabel")

        self.val = QLabel(value)
        self.val.setObjectName("IndicatorValue")

        layout.addWidget(self.lbl)
        layout.addStretch(1)
        layout.addWidget(self.val)

        self.setLayout(layout)

        self.setAccent(accent)

    def refreshStyle(self):
        # Ana widget stil yenile
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()

        # Label stil yenile
        self.lbl.style().unpolish(self.lbl)
        self.lbl.style().polish(self.lbl)
        self.lbl.update()

        # Value stil yenile
        self.val.style().unpolish(self.val)
        self.val.style().polish(self.val)
        self.val.update()

    def setValue(self, value: str):
        self.val.setText(value)
        self.refreshStyle()

    def setAccent(self, accent: str):
        self.setProperty("accent", str(accent).strip().lower())
        self.refreshStyle()

    def text(self):
        return self.val.text()

    def labelText(self):
        return self.lbl.text()