from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from models.robot_program import ProgramCompensationOutputMode


class ProgramActionsWidget(QWidget):
    recompute_requested = pyqtSignal()
    export_requested = pyqtSignal()
    display_options_changed = pyqtSignal()
    compute_compensation_requested = pyqtSignal()
    clear_requested = pyqtSignal()
    trajectory_visibility_changed = pyqtSignal()

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.btn_recompute = QPushButton("Simuler")
        self.btn_export = QPushButton("Exporter programme compense")
        self.btn_compute_compensation = QPushButton("Calculer compensation")
        self.cb_show_theoretical = QCheckBox("Afficher theorique")
        self.cb_show_measured = QCheckBox("Afficher reelle")
        self.cb_show_compensated = QCheckBox("Afficher compensee")
        self.cb_show_theoretical.setChecked(True)
        self.cb_show_measured.setChecked(False)
        self.cb_show_compensated.setChecked(False)
        self.cb_show_compensated.setEnabled(False)
        self.status_label = QLabel("")
        self._setup_ui()
        self._setup_connections()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        row_buttons = QHBoxLayout()
        row_buttons.addWidget(self.btn_recompute, 1)
        row_buttons.addWidget(self.btn_compute_compensation, 1)
        row_buttons.addWidget(self.btn_export, 1)
        layout.addLayout(row_buttons)

        row_visibility = QHBoxLayout()
        row_visibility.addWidget(self.cb_show_theoretical)
        row_visibility.addWidget(self.cb_show_measured)
        row_visibility.addWidget(self.cb_show_compensated)
        row_visibility.addStretch()
        layout.addLayout(row_visibility)

        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

    def _setup_connections(self) -> None:
        self.btn_recompute.clicked.connect(self.recompute_requested.emit)
        self.btn_export.clicked.connect(self.export_requested.emit)
        self.btn_compute_compensation.clicked.connect(self.compute_compensation_requested.emit)
        self.cb_show_theoretical.stateChanged.connect(lambda _: self.trajectory_visibility_changed.emit())
        self.cb_show_measured.stateChanged.connect(lambda _: self.trajectory_visibility_changed.emit())
        self.cb_show_compensated.stateChanged.connect(lambda _: self.trajectory_visibility_changed.emit())

    def selected_output_mode(self) -> ProgramCompensationOutputMode:
        return ProgramCompensationOutputMode.CARTESIAN

    def is_compensated_display(self) -> bool:
        return False

    def set_status_text(self, text: str) -> None:
        self.status_label.setText(text)

    def set_export_enabled(self, enabled: bool) -> None:
        self.btn_export.setEnabled(bool(enabled))

    def set_compensation_enabled(self, enabled: bool) -> None:
        """Active/desactive le bouton de calcul de compensation."""
        self.btn_compute_compensation.setEnabled(enabled)

    def set_simulation_enabled(self, enabled: bool) -> None:
        self.btn_recompute.setEnabled(bool(enabled))

    def is_theoretical_visible(self) -> bool:
        return self.cb_show_theoretical.isChecked()

    def is_measured_visible(self) -> bool:
        return self.cb_show_measured.isChecked()

    def is_compensated_visible(self) -> bool:
        return self.cb_show_compensated.isChecked()

    def set_compensated_checkbox_enabled(self, enabled: bool) -> None:
        """Active/desactive la checkbox compensee. La decoche si on la desactive."""
        if not enabled:
            self.cb_show_compensated.blockSignals(True)
            self.cb_show_compensated.setChecked(False)
            self.cb_show_compensated.blockSignals(False)
        self.cb_show_compensated.setEnabled(enabled)
