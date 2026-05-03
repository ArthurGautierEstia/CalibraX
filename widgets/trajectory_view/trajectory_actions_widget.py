from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget


class TrajectoryActionsWidget(QWidget):
    """Widget for trajectory playback actions and timeline control."""

    compute_requested = pyqtSignal()
    play_requested = pyqtSignal()
    pause_requested = pyqtSignal()
    stop_requested = pyqtSignal()
    home_position_requested = pyqtSignal()
    export_trajectory_requested = pyqtSignal()
    reverse_toggled = pyqtSignal(bool)
    loop_toggled = pyqtSignal(bool)
    time_value_changed = pyqtSignal(float)

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self._time_range = (0.0, 10.0)
        self._editing_locked = False
        self._analysis_pending = False

        self.btn_compute = QPushButton("Calculer")
        self.btn_home = QPushButton("Aller Home")

        self.btn_play = QPushButton("Démarrer")
        self.btn_pause = QPushButton("Pause")
        self.btn_stop = QPushButton("Stop")
        self.btn_export_trajectory = QPushButton("Exporter trajectoire CSV")

        # Keep these options in code, but hide them from UI for now.
        self.cb_reverse = QCheckBox("Inverser à la fin")
        self.cb_loop = QCheckBox("Boucle")
        self.cb_reverse.setVisible(False)
        self.cb_loop.setVisible(False)

        self.time_slider = QSlider(Qt.Orientation.Horizontal)
        self.time_label = QLabel("Temps : 0.00 s")
        self.issues_label = QLabel("")
        self.warnings_label = QLabel("")

        self._setup_ui()
        self._setup_connections()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        title = QLabel("Simuler la trajectoire")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(title)

        row_actions = QHBoxLayout()
        row_actions.addWidget(self.btn_compute)
        row_actions.addWidget(self.btn_home)
        row_actions.addWidget(self.btn_play)
        row_actions.addWidget(self.btn_pause)
        row_actions.addWidget(self.btn_stop)
        row_actions.addWidget(self.btn_export_trajectory)
        row_actions.addStretch()

        row_timeline = QHBoxLayout()
        self.time_slider.setRange(0, 1000)
        self.time_slider.setValue(0)
        row_timeline.addWidget(self.time_label)
        row_timeline.addWidget(self.time_slider)

        layout.addLayout(row_actions)
        layout.addLayout(row_timeline)

        self.issues_label.setWordWrap(True)
        self.issues_label.setStyleSheet("color: #d9534f; font-weight: bold;")
        self.issues_label.hide()
        layout.addWidget(self.issues_label)

        self.warnings_label.setWordWrap(True)
        self.warnings_label.setStyleSheet("color: #f0ad4e; font-weight: bold;")
        self.warnings_label.hide()
        layout.addWidget(self.warnings_label)

    def _setup_connections(self) -> None:
        self.btn_compute.clicked.connect(self.compute_requested.emit)
        self.btn_export_trajectory.clicked.connect(self.export_trajectory_requested.emit)
        self.btn_home.clicked.connect(self.home_position_requested.emit)
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

    def set_issue_messages(self, messages: list[str]) -> None:
        if not messages:
            self.issues_label.setText("")
            self.issues_label.hide()
            return
        self.issues_label.setText("Problemes detectes: " + " | ".join(messages))
        self.issues_label.show()

    def set_warning_messages(self, messages: list[str]) -> None:
        if not messages:
            self.warnings_label.setText("")
            self.warnings_label.hide()
            return
        self.warnings_label.setText("Warnings: " + " | ".join(messages))
        self.warnings_label.show()

    def set_editing_locked(self, locked: bool) -> None:
        self._editing_locked = bool(locked)
        self._refresh_enabled_state()

    def set_analysis_pending(self, pending: bool) -> None:
        self._analysis_pending = bool(pending)
        self._refresh_enabled_state()

    def _refresh_enabled_state(self) -> None:
        editing_enabled = not self._editing_locked
        playback_enabled = editing_enabled and not self._analysis_pending
        self.btn_compute.setEnabled(editing_enabled)
        self.btn_export_trajectory.setEnabled(playback_enabled)
        self.btn_home.setEnabled(editing_enabled)
        self.btn_play.setEnabled(playback_enabled)
        self.btn_pause.setEnabled(playback_enabled)
        self.btn_stop.setEnabled(playback_enabled)
        self.cb_reverse.setEnabled(playback_enabled)
        self.cb_loop.setEnabled(playback_enabled)
        self.time_slider.setEnabled(playback_enabled)
