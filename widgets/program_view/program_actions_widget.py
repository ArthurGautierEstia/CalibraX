from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import QCheckBox, QComboBox, QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget

from models.robot_program import ProgramCompensationOutputMode


class ProgramActionsWidget(QWidget):
    recompute_requested = pyqtSignal()
    export_requested = pyqtSignal()
    display_options_changed = pyqtSignal()
    play_requested = pyqtSignal()
    pause_requested = pyqtSignal()
    stop_requested = pyqtSignal()
    restart_requested = pyqtSignal()
    time_value_changed = pyqtSignal(float)
    clear_requested = pyqtSignal()
    

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        self._time_range = (0.0, 0.0)
        self.btn_recompute = QPushButton("Recalculer")
        self.btn_clear = QPushButton("Effacer trajectoire")
        self.btn_export = QPushButton("Exporter programme compense")
        self.btn_play = QPushButton("Demarrer")
        self.btn_pause = QPushButton("Pause")
        self.btn_stop = QPushButton("Stop")
        self.btn_restart = QPushButton("Restart")
        self.time_slider = QSlider(Qt.Orientation.Horizontal)
        self.time_label = QLabel("Temps : 0.00 s")
        self.status_label = QLabel("")
        self._setup_ui()
        self._setup_connections()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        row_buttons = QHBoxLayout()
        row_buttons.addWidget(self.btn_recompute)
        row_buttons.addWidget(self.btn_clear)
        row_buttons.addWidget(self.btn_export)
        row_buttons.addWidget(self.btn_play)
        row_buttons.addWidget(self.btn_pause)
        row_buttons.addWidget(self.btn_stop)
        row_buttons.addWidget(self.btn_restart)
        row_buttons.addStretch()
        layout.addLayout(row_buttons)

        row_timeline = QHBoxLayout()
        self.time_slider.setRange(0, 1000)
        self.time_slider.setValue(0)
        row_timeline.addWidget(self.time_label)
        row_timeline.addWidget(self.time_slider)
        layout.addLayout(row_timeline)

        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

    def _setup_connections(self) -> None:
        self.btn_recompute.clicked.connect(self.recompute_requested.emit)
        self.btn_clear.clicked.connect(self.clear_requested.emit)
        self.btn_export.clicked.connect(self.export_requested.emit)
        self.btn_play.clicked.connect(self.play_requested.emit)
        self.btn_pause.clicked.connect(self.pause_requested.emit)
        self.btn_stop.clicked.connect(self.stop_requested.emit)
        self.btn_restart.clicked.connect(self.restart_requested.emit)
        self.time_slider.valueChanged.connect(self._on_slider_changed)

    def _on_slider_changed(self, value: int) -> None:
        time_value = self._slider_to_time(value)
        self.time_label.setText(f"Temps : {time_value:.2f} s")
        self.time_value_changed.emit(time_value)

    def selected_output_mode(self) -> ProgramCompensationOutputMode:
        return ProgramCompensationOutputMode.CARTESIAN

    def is_compensated_display(self) -> bool:
        return False

    def set_status_text(self, text: str) -> None:
        self.status_label.setText(text)

    def set_export_enabled(self, enabled: bool) -> None:
        self.btn_export.setEnabled(bool(enabled))

    def set_time_range(self, min_t: float, max_t: float) -> None:
        self._time_range = (float(min_t), float(max_t))
        self.set_time_value(min_t)

    def set_time_value(self, time_value: float) -> None:
        self.time_slider.blockSignals(True)
        self.time_slider.setValue(self._time_to_slider(time_value))
        self.time_slider.blockSignals(False)
        self.time_label.setText(f"Temps : {float(time_value):.2f} s")

    def _slider_to_time(self, slider_value: int) -> float:
        min_t, max_t = self._time_range
        if max_t == min_t:
            return min_t
        return min_t + (float(slider_value) / 1000.0) * (max_t - min_t)

    def _time_to_slider(self, time_value: float) -> int:
        min_t, max_t = self._time_range
        if max_t == min_t:
            return 0
        ratio = (float(time_value) - min_t) / (max_t - min_t)
        return int(round(max(0.0, min(1.0, ratio)) * 1000.0))
