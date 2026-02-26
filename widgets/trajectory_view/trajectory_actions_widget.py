from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QCheckBox, QSlider, QLabel
)
from PyQt6.QtCore import Qt, pyqtSignal


class TrajectoryActionsWidget(QWidget):
    """Widget for trajectory playback actions and timeline control."""

    compute_requested = pyqtSignal()
    play_requested = pyqtSignal()
    pause_requested = pyqtSignal()
    stop_requested = pyqtSignal()
    reverse_toggled = pyqtSignal(bool)
    loop_toggled = pyqtSignal(bool)
    time_value_changed = pyqtSignal(float)

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self._time_range = (0.0, 10.0)

        self.btn_compute = QPushButton("Calculer")

        self.btn_play = QPushButton("Démarrer")
        self.btn_pause = QPushButton("Pause")
        self.btn_stop = QPushButton("Stop")

        self.cb_reverse = QCheckBox("Inverser à la fin")
        self.cb_loop = QCheckBox("Boucle")

        self.time_slider = QSlider(Qt.Orientation.Horizontal)
        self.time_label = QLabel("Temps : 0.00 s")

        self._setup_ui()
        self._setup_connections()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        title = QLabel("Simuler la trajectoire")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(title)

        row_actions = QHBoxLayout()
        row_actions.addWidget(self.btn_compute)
        row_actions.addWidget(self.btn_play)
        row_actions.addWidget(self.btn_pause)
        row_actions.addWidget(self.btn_stop)
        row_actions.addStretch()
        row_actions.addWidget(self.cb_reverse)
        row_actions.addWidget(self.cb_loop)

        row_timeline = QHBoxLayout()
        self.time_slider.setRange(0, 1000)
        self.time_slider.setValue(0)
        row_timeline.addWidget(self.time_label)
        row_timeline.addWidget(self.time_slider)

        layout.addLayout(row_actions)
        layout.addLayout(row_timeline)

    def _setup_connections(self) -> None:
        self.btn_compute.clicked.connect(self.compute_requested.emit)
        self.btn_play.clicked.connect(self.play_requested.emit)
        self.btn_pause.clicked.connect(self.pause_requested.emit)
        self.btn_stop.clicked.connect(self.stop_requested.emit)
        self.cb_reverse.toggled.connect(self.reverse_toggled.emit)
        self.cb_loop.toggled.connect(self.loop_toggled.emit)
        self.time_slider.valueChanged.connect(self._on_slider_changed)

    def _on_slider_changed(self, value: int) -> None:
        time_value = self._slider_to_time(value)
        self.time_label.setText(f"Temps : {time_value:.2f} s")
        self.time_value_changed.emit(time_value)

    def _slider_to_time(self, slider_value: int) -> float:
        min_t, max_t = self._time_range
        if max_t == min_t:
            return min_t
        return min_t + (slider_value / 1000.0) * (max_t - min_t)

    def _time_to_slider(self, time_value: float) -> int:
        min_t, max_t = self._time_range
        if max_t == min_t:
            return 0
        ratio = (time_value - min_t) / (max_t - min_t)
        return int(round(max(0.0, min(1.0, ratio)) * 1000))

    def set_time_range(self, min_t: float, max_t: float) -> None:
        self._time_range = (min_t, max_t)
        self.set_time_value(min_t)

    def set_time_value(self, time_value: float) -> None:
        self.time_slider.blockSignals(True)
        self.time_slider.setValue(self._time_to_slider(time_value))
        self.time_slider.blockSignals(False)
        self.time_label.setText(f"Temps : {time_value:.2f} s")

    def set_editing_locked(self, locked: bool) -> None:
        enabled = not locked
        self.btn_compute.setEnabled(enabled)
        self.btn_play.setEnabled(enabled)
        self.btn_pause.setEnabled(enabled)
        self.btn_stop.setEnabled(enabled)
        self.cb_reverse.setEnabled(enabled)
        self.cb_loop.setEnabled(enabled)
        self.time_slider.setEnabled(enabled)
