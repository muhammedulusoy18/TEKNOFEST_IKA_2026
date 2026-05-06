from PyQt5.QtWidgets import QWidget, QGridLayout
from ui.widgets.video_view import VideoView


class VideoWall(QWidget):
    def __init__(self, camera_count=2):
        super().__init__()
        self.setStyleSheet("background-color: #0f1724; border-radius: 10px;")

        self.camera_count = camera_count

        self.layout = QGridLayout(self)
        self.layout.setContentsMargins(10, 10, 10, 10)
        self.layout.setSpacing(10)

        self.cam1 = VideoView(title="ANA KAMERA")

        if self.camera_count == 2:
            self.cam2 = VideoView(title="KAMERA 2")
        else:
            self.cam2 = None

        self.main_cam = 1

        if self.camera_count == 2:
            self.cam1.mousePressEvent = self.cam1_clicked
            self.cam2.mousePressEvent = self.cam2_clicked

        self.build_layout()

    def build_layout(self):
        while self.layout.count():
            item = self.layout.takeAt(0)
            if item.widget():
                item.widget().setParent(None)

        if self.camera_count == 1:
            self.layout.addWidget(self.cam1, 0, 0, 1, 1)
            return

        if self.main_cam == 1:
            self.layout.addWidget(self.cam1, 0, 0, 2, 2)
            self.layout.addWidget(self.cam2, 0, 2)
        else:
            self.layout.addWidget(self.cam2, 0, 0, 2, 2)
            self.layout.addWidget(self.cam1, 0, 2)

        self.layout.setColumnStretch(0, 3)
        self.layout.setColumnStretch(1, 3)
        self.layout.setColumnStretch(2, 2)

    def swap(self):
        if self.camera_count != 2:
            return
        self.main_cam = 2 if self.main_cam == 1 else 1
        self.build_layout()

    def cam1_clicked(self, event):
        if self.main_cam != 1:
            self.swap()

    def cam2_clicked(self, event):
        if self.main_cam != 2:
            self.swap()