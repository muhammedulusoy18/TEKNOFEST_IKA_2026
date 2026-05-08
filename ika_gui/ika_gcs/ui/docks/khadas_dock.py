from PyQt5.QtWidgets import QDockWidget, QWidget, QVBoxLayout
from ui.widgets.indicator import Indicator


class KhadasDock(QDockWidget):
    def __init__(self, parent=None):
        super().__init__("Khadas Health", parent)

        container = QWidget()

        layout = QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # --------------------------
        # KHADAS HEALTH GÖSTERGELERİ
        # --------------------------
        self.ind_cpu  = Indicator("CPU Kullanımı", "--%", accent="blue")
        self.ind_temp = Indicator("Sıcaklık", "-- °C", accent="red")
        self.ind_ram  = Indicator("RAM Kullanımı", "--%", accent="purple")
        self.ind_disk = Indicator("Disk Kullanımı", "--%", accent="orange")

        layout.addWidget(self.ind_cpu)
        layout.addWidget(self.ind_temp)
        layout.addWidget(self.ind_ram)
        layout.addWidget(self.ind_disk)

        # Widgetları yukarı sabitle
        layout.addStretch()

        container.setLayout(layout)
        self.setWidget(container)

    # -------------------------------------------------
    # KHADAS HEALTH VERİ GÜNCELLEME
    # -------------------------------------------------
    def update_from_health(self, t: dict):

        # Telemetry paketinde khadas verisi olabilir
        data = t.get("khadas", t)

        # CPU
        if "cpu" in data:
            self.ind_cpu.val.setText(f"%{data['cpu']}")

        # Sıcaklık
        if "temp" in data:
            self.ind_temp.val.setText(f"{data['temp']} °C")

        # RAM
        if "ram" in data:
            self.ind_ram.val.setText(f"%{data['ram']}")

        # Disk
        if "disk" in data:
            self.ind_disk.val.setText(f"%{data['disk']}")