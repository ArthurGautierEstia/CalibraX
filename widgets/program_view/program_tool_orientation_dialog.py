from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QLabel,
    QVBoxLayout,
    QWidget,
)

from models.types import Pose6


class ProgramToolOrientationDialog(QDialog):
    _AXIS_LABELS = ("A", "B", "C")

    def __init__(
        self,
        initial_orientation: Pose6 | None,
        points_count: int,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Editer l'orientation de l'outil")
        self.setMinimumWidth(340)

        self._spinboxes: list[QDoubleSpinBox] = []
        self.button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )

        self._setup_ui(initial_orientation, points_count)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        for button in self.button_box.buttons():
            button.setAutoDefault(False)
            button.setDefault(False)

    def _setup_ui(self, initial_orientation: Pose6 | None, points_count: int) -> None:
        layout = QVBoxLayout(self)
        info_label = QLabel(
            f"Modification de l'orientation A/B/C sur {points_count} point(s) cartesien(s) lineaire(s)."
        )
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        if initial_orientation is None:
            mixed_label = QLabel(
                "Les valeurs actuelles ne sont pas identiques sur tous les points. "
                "Renseignez les nouvelles valeurs a appliquer."
            )
            mixed_label.setWordWrap(True)
            layout.addWidget(mixed_label)

        form_layout = QFormLayout()
        orientation_values = (
            (initial_orientation.a, initial_orientation.b, initial_orientation.c)
            if initial_orientation is not None
            else (0.0, 0.0, 0.0)
        )
        for axis_index, axis_label in enumerate(self._AXIS_LABELS):
            spinbox = QDoubleSpinBox(self)
            spinbox.setRange(-360.0, 360.0)
            spinbox.setDecimals(3)
            spinbox.setSingleStep(0.1)
            spinbox.setSuffix(" deg")
            spinbox.setKeyboardTracking(False)
            spinbox.setValue(float(orientation_values[axis_index]))
            form_layout.addRow(axis_label, spinbox)
            self._spinboxes.append(spinbox)

        layout.addLayout(form_layout)
        layout.addWidget(self.button_box)

    def get_orientation(self) -> Pose6:
        return Pose6(
            0.0,
            0.0,
            0.0,
            float(self._spinboxes[0].value()),
            float(self._spinboxes[1].value()),
            float(self._spinboxes[2].value()),
        )

    def keyPressEvent(self, event) -> None:  # type: ignore[override]
        if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            focused_widget = self.focusWidget()
            if isinstance(focused_widget, QDoubleSpinBox):
                focused_widget.interpretText()
                event.accept()
                return
        super().keyPressEvent(event)
