"""Widget réutilisable pour éditer une Pose6 (6 spinboxes X Y Z A B C)."""
from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QDoubleSpinBox, QFormLayout, QGroupBox, QWidget

from models.types.pose6 import Pose6


class Pose6EditorWidget(QGroupBox):
    pose_changed = pyqtSignal()

    LABELS = ("X (mm)", "Y (mm)", "Z (mm)", "A (°)", "B (°)", "C (°)")

    def __init__(self, title: str = "", parent: QWidget | None = None) -> None:
        super().__init__(title, parent)
        self._spinboxes: list[QDoubleSpinBox] = []
        self._building = False
        self._setup_ui()

    def _setup_ui(self) -> None:
        form = QFormLayout(self)
        form.setSpacing(4)
        for label in self.LABELS:
            sb = QDoubleSpinBox()
            sb.setRange(-99999.0, 99999.0)
            sb.setDecimals(3)
            sb.setSingleStep(1.0)
            sb.valueChanged.connect(self._on_changed)
            form.addRow(label, sb)
            self._spinboxes.append(sb)

    def _on_changed(self) -> None:
        if not self._building:
            self.pose_changed.emit()

    def get_pose(self) -> Pose6:
        vals = [sb.value() for sb in self._spinboxes]
        return Pose6(*vals)

    def set_pose(self, pose: Pose6) -> None:
        self._building = True
        for sb, val in zip(self._spinboxes, pose.to_list()):
            sb.setValue(val)
        self._building = False
