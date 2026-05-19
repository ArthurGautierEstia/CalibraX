from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QVBoxLayout,
    QWidget,
)

from models.types import Pose6


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

    def __init__(self, base_pose: Pose6, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Editer la base programme")
        self.setMinimumWidth(320)

        self._spinboxes: list[QDoubleSpinBox] = []
        self.button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Close
        )

        self._setup_ui(base_pose)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)

    def _setup_ui(self, base_pose: Pose6) -> None:
        layout = QVBoxLayout(self)
        form_layout = QFormLayout()

        for axis_index, axis_label in enumerate(self._AXIS_LABELS):
            spinbox = QDoubleSpinBox(self)
            minimum, maximum = self._AXIS_LIMITS[axis_index]
            spinbox.setRange(minimum, maximum)
            spinbox.setDecimals(3)
            spinbox.setSingleStep(1.0 if axis_index < 3 else 0.1)
            spinbox.setSuffix(" mm" if axis_index < 3 else " deg")
            spinbox.setKeyboardTracking(False)
            spinbox.setValue(base_pose.to_list()[axis_index])
            spinbox.valueChanged.connect(self._emit_preview_changed)
            form_layout.addRow(axis_label, spinbox)
            self._spinboxes.append(spinbox)

        layout.addLayout(form_layout)
        layout.addWidget(self.button_box)

    def get_base_pose(self) -> Pose6:
        values = [float(spinbox.value()) for spinbox in self._spinboxes]
        return Pose6.from_values(values)

    def _emit_preview_changed(self, _value: float) -> None:
        self.base_pose_preview_changed.emit(self.get_base_pose())
