from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QDoubleSpinBox, QHBoxLayout, QGroupBox
)
from PyQt6.QtCore import pyqtSignal


class MgiJointWeightsWidget(QWidget):
    """
    Widget permettant de définir les poids des joints pour la sélection de la meilleure solution MGI
    """

    # Émis à chaque modification
    # list[float]
    weights_changed = pyqtSignal(list)

    def __init__(self, weights: list[float], parent=None):
        super().__init__(parent)

        self._weights = list(weights)
        self._spin_boxes: list[QDoubleSpinBox] = []

        self._init_ui()

    # ---------------------------------------------------------
    # UI
    # ---------------------------------------------------------

    def _init_ui(self):
        layout = QVBoxLayout(self)
        group_box = QGroupBox("Poids des joints")
        group_layout = QVBoxLayout(group_box)

        description = QLabel("Poids utilisés pour calculer la distance entre la position actuelle et les solutions MGI.\n"
                           "Un poids plus élevé pénalise davantage les mouvements de ce joint.")
        description.setWordWrap(True)
        group_layout.addWidget(description)

        for i in range(6):
            joint_layout = QHBoxLayout()

            label = QLabel(f"Joint {i+1}:")
            joint_layout.addWidget(label)

            spin_box = QDoubleSpinBox()
            spin_box.setRange(0.0, 100.0)
            spin_box.setValue(self._weights[i])
            spin_box.setSingleStep(0.1)
            spin_box.setDecimals(2)
            spin_box.valueChanged.connect(
                lambda value, idx=i: self._on_weight_changed(idx, value)
            )

            self._spin_boxes.append(spin_box)
            joint_layout.addWidget(spin_box)

            group_layout.addLayout(joint_layout)

        group_layout.addStretch()
        layout.addWidget(group_box)

    # ---------------------------------------------------------
    # Internals
    # ---------------------------------------------------------

    def _on_weight_changed(self, index: int, value: float):
        self._weights[index] = value
        self.weights_changed.emit(self._weights.copy())

    # ---------------------------------------------------------
    # Public API
    # ---------------------------------------------------------

    def set_weights(self, weights: list[float]):
        if len(weights) >= 6:
            self._weights = list(weights[:6])
            for i in range(6):
                self._spin_boxes[i].blockSignals(True)
                self._spin_boxes[i].setValue(self._weights[i])
                self._spin_boxes[i].blockSignals(False)
