from PyQt5.QtWidgets import QDockWidget, QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox
from ui.widgets.estop_button import EStopButton


class AutonomyDock(QDockWidget):
    def __init__(self, parent=None):
        super().__init__("Autonomy", parent)
        self._enabled = True

        w = QWidget()
        layout = QVBoxLayout(w)

        # Scenario select
        self.scenario = QComboBox()
        self.scenario.addItems(["Scenario A", "Scenario B", "Scenario C"])
        self.scenario.currentIndexChanged.connect(self._on_scenario_changed)

        # Control buttons
        self.btn_start = QPushButton("Start")
        self.btn_pause = QPushButton("Pause")
        self.btn_abort = QPushButton("Abort")

        self.btn_start.clicked.connect(self._on_start)
        self.btn_pause.clicked.connect(self._on_pause)
        self.btn_abort.clicked.connect(self._on_abort)

        # E-STOP button
        self.estop = EStopButton()
        if parent is not None and hasattr(parent, "on_estop"):
            self.estop.clicked.connect(parent.on_estop)

        # Layout
        layout.addWidget(QLabel("Scenario"))
        layout.addWidget(self.scenario)
        layout.addWidget(self.btn_start)
        layout.addWidget(self.btn_pause)
        layout.addWidget(self.btn_abort)
        layout.addSpacing(10)
        layout.addWidget(self.estop)  # New position
        layout.addStretch(1)

        self.setWidget(w)

    # Callbacks
    def _on_start(self):
        if hasattr(self.parent(), "on_autonomy_start"):
            self.parent().on_autonomy_start()

    def _on_pause(self):
        if hasattr(self.parent(), "on_autonomy_pause"):
            self.parent().on_autonomy_pause()

    def _on_abort(self):
        if hasattr(self.parent(), "on_autonomy_abort"):
            self.parent().on_autonomy_abort()

    def _on_scenario_changed(self, index):
        name = self.scenario.itemText(index)
        if hasattr(self.parent(), "on_autonomy_scenario"):
            self.parent().on_autonomy_scenario(name)

    # Enable/disable UI
    def set_enabled(self, enabled: bool):
        self._enabled = enabled
        self.scenario.setEnabled(enabled)
        self.btn_start.setEnabled(enabled)
        self.btn_pause.setEnabled(enabled)
        self.btn_abort.setEnabled(enabled)

        # E-STOP always active
        self.estop.setEnabled(True)