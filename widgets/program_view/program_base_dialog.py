from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QLabel,
    QVBoxLayout,
    QWidget,
)

from models.robot_program import ProgramBaseSource
from models.types import Pose6
from utils.reference_frame_utils import matrix_to_pose, pose_to_matrix


class ProgramBaseDialog(QDialog):
    base_pose_preview_changed = pyqtSignal(Pose6)

    _AXIS_LABELS = ("X", "Y", "Z", "A", "B", "C")
    _AXIS_LIMITS = (
        (-100000.0, 100000.0),
        (-100000.0, 100000.0),
        (-100000.0, 100000.0),
        (-360.0, 360.0),
        (-360.0, 360.0),
        (-360.0, 360.0),
    )

    def __init__(
        self,
        base_source: ProgramBaseSource,
        manual_base: Pose6,
        base_offset: Pose6,
        workpiece_frame_in_robot: Pose6,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Editer la base programme")
        self.setMinimumWidth(340)

        self._workpiece_frame_in_robot: Pose6 = workpiece_frame_in_robot.copy()
        self._manual_spinboxes: list[QDoubleSpinBox] = []
        self._offset_spinboxes: list[QDoubleSpinBox] = []
        self._workpiece_labels: list[QLabel] = []

        self.button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Close
        )

        self._setup_ui(base_source, manual_base, base_offset)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        for button in self.button_box.buttons():
            button.setAutoDefault(False)
            button.setDefault(False)

    def _setup_ui(
        self,
        base_source: ProgramBaseSource,
        manual_base: Pose6,
        base_offset: Pose6,
    ) -> None:
        layout = QVBoxLayout(self)

        # --- Source combo ---
        source_form = QFormLayout()
        self._source_combo = QComboBox(self)
        self._source_combo.addItem("Manuel", ProgramBaseSource.MANUAL)
        self._source_combo.addItem("Repère pièce", ProgramBaseSource.WORKPIECE)
        if base_source == ProgramBaseSource.WORKPIECE:
            self._source_combo.setCurrentIndex(1)
        source_form.addRow("Source", self._source_combo)
        layout.addLayout(source_form)

        # --- Base (Manuel) ---
        self._manual_group = QGroupBox("Base manuelle")
        manual_form = QFormLayout(self._manual_group)
        for axis_index, axis_label in enumerate(self._AXIS_LABELS):
            spinbox = QDoubleSpinBox(self)
            lo, hi = self._AXIS_LIMITS[axis_index]
            spinbox.setRange(lo, hi)
            spinbox.setDecimals(3)
            spinbox.setSingleStep(1.0 if axis_index < 3 else 0.1)
            spinbox.setSuffix(" mm" if axis_index < 3 else " deg")
            spinbox.setKeyboardTracking(False)
            spinbox.setValue(manual_base.to_list()[axis_index])
            spinbox.valueChanged.connect(self._emit_preview_changed)
            manual_form.addRow(axis_label, spinbox)
            self._manual_spinboxes.append(spinbox)
        layout.addWidget(self._manual_group)

        # --- Base (Repère pièce, read-only) ---
        self._workpiece_group = QGroupBox("Repère pièce dans repère robot (référence)")
        workpiece_form = QFormLayout(self._workpiece_group)
        wp_values = self._workpiece_frame_in_robot.to_list()
        for axis_index, axis_label in enumerate(self._AXIS_LABELS):
            suffix = " mm" if axis_index < 3 else " deg"
            label = QLabel(f"{wp_values[axis_index]:.3f}{suffix}", self)
            label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            workpiece_form.addRow(axis_label, label)
            self._workpiece_labels.append(label)
        layout.addWidget(self._workpiece_group)

        # --- Offset ---
        offset_group = QGroupBox("Offset")
        offset_form = QFormLayout(offset_group)
        for axis_index, axis_label in enumerate(self._AXIS_LABELS):
            spinbox = QDoubleSpinBox(self)
            lo, hi = self._AXIS_LIMITS[axis_index]
            spinbox.setRange(lo, hi)
            spinbox.setDecimals(3)
            spinbox.setSingleStep(1.0 if axis_index < 3 else 0.1)
            spinbox.setSuffix(" mm" if axis_index < 3 else " deg")
            spinbox.setKeyboardTracking(False)
            spinbox.setValue(base_offset.to_list()[axis_index])
            spinbox.valueChanged.connect(self._emit_preview_changed)
            offset_form.addRow(axis_label, spinbox)
            self._offset_spinboxes.append(spinbox)
        layout.addWidget(offset_group)

        layout.addWidget(self.button_box)

        self._source_combo.currentIndexChanged.connect(self._on_source_changed)
        self._update_source_visibility()

    def _on_source_changed(self, _index: int) -> None:
        self._update_source_visibility()
        self._emit_preview_changed(0.0)

    def _update_source_visibility(self) -> None:
        is_manual = self._current_source() == ProgramBaseSource.MANUAL
        self._manual_group.setVisible(is_manual)
        self._workpiece_group.setVisible(not is_manual)

    def _current_source(self) -> ProgramBaseSource:
        return self._source_combo.currentData()

    def _get_manual_base(self) -> Pose6:
        return Pose6.from_values([float(s.value()) for s in self._manual_spinboxes])

    def _get_offset(self) -> Pose6:
        return Pose6.from_values([float(s.value()) for s in self._offset_spinboxes])

    def _effective_base(self) -> Pose6:
        source = self._current_source()
        base = self._get_manual_base() if source == ProgramBaseSource.MANUAL else self._workpiece_frame_in_robot
        offset = self._get_offset()
        if offset == Pose6.zeros():
            return base.copy()
        return matrix_to_pose(pose_to_matrix(base) @ pose_to_matrix(offset))

    def get_base_config(self) -> tuple[ProgramBaseSource, Pose6, Pose6]:
        """Retourne (source, base_manuelle, offset)."""
        return self._current_source(), self._get_manual_base(), self._get_offset()

    def _emit_preview_changed(self, _value: float = 0.0) -> None:
        self.base_pose_preview_changed.emit(self._effective_base())

    def keyPressEvent(self, event) -> None:  # type: ignore[override]
        if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            focused_widget = self.focusWidget()
            if isinstance(focused_widget, QDoubleSpinBox):
                focused_widget.interpretText()
                event.accept()
                return
        super().keyPressEvent(event)
