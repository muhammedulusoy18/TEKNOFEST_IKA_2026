from PyQt5.QtWidgets import QFrame, QVBoxLayout, QLabel


class VideoView(QFrame):
    def __init__(self, title="CAMERA"):
        super().__init__()
        self.setObjectName("VideoCard")
        self.setFrameShape(QFrame.StyledPanel)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        self.title = QLabel(title)
        self.title.setObjectName("VideoTitle")

        self.canvas = QLabel("NO SIGNAL")
        self.canvas.setObjectName("VideoCanvas")
        self.canvas.setMinimumHeight(140)

        layout.addWidget(self.title)
        layout.addWidget(self.canvas)
        self.setLayout(layout)

        self.setStyleSheet("""
            #VideoCard {
                background-color: #0f1724;
                border: 1px solid #1f2a3d;
                border-radius: 10px;
            }
            #VideoTitle {
                color: #8b949e;
                font-size: 11px;
                font-weight: 700;
                letter-spacing: 1px;
            }
            #VideoCanvas {
                background-color: #111827;
                border: 1px solid #1f2a3d;
                border-radius: 8px;
                color: #374151;
                font-size: 13px;
                font-weight: 700;
                qproperty-alignment: AlignCenter;
            }
        """)

    def set_frame(self, pixmap):
        from PyQt5.QtCore import Qt
        scaled = pixmap.scaled(self.canvas.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.canvas.setPixmap(scaled)