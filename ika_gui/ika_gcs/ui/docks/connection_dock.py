from PyQt5.QtWidgets import QDockWidget, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QHBoxLayout


class ConnectionDock(QDockWidget):
    def __init__(self, parent=None):
        super().__init__("Connection", parent)

        w = QWidget()
        layout = QVBoxLayout(w)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Host"))
        self.host = QLineEdit("127.0.0.1")
        row1.addWidget(self.host)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("TCP Port"))
        self.port = QLineEdit("9000")
        row2.addWidget(self.port)

        self.btn_connect = QPushButton("TCP Connect")
        self.btn_disconnect = QPushButton("Disconnect")

        layout.addLayout(row1)
        layout.addLayout(row2)
        layout.addWidget(self.btn_connect)
        layout.addWidget(self.btn_disconnect)
        layout.addStretch(1)

        self.setWidget(w)
