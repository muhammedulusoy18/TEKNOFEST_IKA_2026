from PyQt5.QtWidgets import QPushButton


class EStopButton(QPushButton):
    def __init__(self):
        super().__init__("ACİL STOP")
        self.setObjectName("EStopButton")
        self.setToolTip("Acil durdurma (UI testi). Sonraki adım: TCP+ACK.")
