from typing import List
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QDoubleSpinBox, QSizePolicy
)
from PyQt6.QtCore import Qt, pyqtSignal


class JogJointControlWidget(QWidget):
    """Widget pour le contrôle Jog articulaire (robot + axes externes E1/E2/…)."""

    # Signaux robot (indices 0-5)
    jog_joint_minus_pressed = pyqtSignal(int)
    jog_joint_minus_released = pyqtSignal(int)
    jog_joint_plus_pressed = pyqtSignal(int)
    jog_joint_plus_released = pyqtSignal(int)
    delta_changed = pyqtSignal(float)

    # Signaux axes externes : (axis_id, joint_index, direction +1/-1)
    jog_external_minus_pressed = pyqtSignal(str, int)
    jog_external_minus_released = pyqtSignal(str, int)
    jog_external_plus_pressed = pyqtSignal(str, int)
    jog_external_plus_released = pyqtSignal(str, int)

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.delta_input = QDoubleSpinBox()
        self.jog_joint_buttons_minus: List[QPushButton] = []
        self.jog_joint_buttons_plus: List[QPushButton] = []
        # Boutons axes externes : liste de (btn_minus, btn_plus)
        self._ext_buttons: List[tuple[QPushButton, QPushButton]] = []

        self._groupbox_layout: QVBoxLayout | None = None
        self._ext_section_start_index: int = 0
        self.setup_ui()

    def setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        groupbox = QGroupBox("Jog Articulaire")
        groupbox_layout = QVBoxLayout()
        self._groupbox_layout = groupbox_layout

        groupbox_layout.addWidget(QLabel())

        # Delta
        row_layout = QHBoxLayout()
        label = QLabel("Delta")
        label.setFixedWidth(80)
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.delta_input.setValue(0.0)
        self.delta_input.setRange(0, 10)
        self.delta_input.setDecimals(2)
        self.delta_input.setSingleStep(0.1)
        self.delta_input.setFixedWidth(101)
        self.delta_input.valueChanged.connect(self.delta_changed.emit)

        row_layout.addWidget(label)
        row_layout.addWidget(self.delta_input)
        row_layout.addStretch()
        groupbox_layout.addLayout(row_layout)

        # 6 joints robot
        for i in range(6):
            row_layout = QHBoxLayout()
            label = QLabel(f"q{i+1}")
            label.setFixedWidth(80)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)

            btn_minus = QPushButton("-")
            btn_minus.pressed.connect(lambda idx=i: self.jog_joint_minus_pressed.emit(idx))
            btn_minus.released.connect(lambda idx=i: self.jog_joint_minus_released.emit(idx))
            btn_minus.setFixedWidth(48)

            btn_plus = QPushButton("+")
            btn_plus.pressed.connect(lambda idx=i: self.jog_joint_plus_pressed.emit(idx))
            btn_plus.released.connect(lambda idx=i: self.jog_joint_plus_released.emit(idx))
            btn_plus.setFixedWidth(48)

            row_layout.addWidget(label)
            row_layout.addWidget(btn_minus)
            row_layout.addWidget(btn_plus)
            row_layout.addStretch()
            groupbox_layout.addLayout(row_layout)
            self.jog_joint_buttons_minus.append(btn_minus)
            self.jog_joint_buttons_plus.append(btn_plus)

        self._ext_section_start_index = groupbox_layout.count()

        groupbox.setLayout(groupbox_layout)
        layout.addWidget(groupbox)
        self.setLayout(layout)

    def set_delta(self, value: float):
        self.delta_input.blockSignals(True)
        self.delta_input.setValue(value)
        self.delta_input.blockSignals(False)

    # ------------------------------------------------------------------
    # Axes externes dynamiques
    # ------------------------------------------------------------------

    def set_external_axes(self, axes_info: list[tuple[str, int, str, str]]) -> None:
        """Reconstruit les lignes de jog pour les axes externes.

        axes_info: liste de (axis_id, joint_index, label "E1", unit "mm"/"°")
        """
        if self._groupbox_layout is None:
            return

        # Supprimer les anciennes lignes axes externes
        for btn_m, btn_p in self._ext_buttons:
            btn_m.deleteLater()
            btn_p.deleteLater()
        self._ext_buttons.clear()
        # Supprimer les layouts correspondants
        while self._groupbox_layout.count() > self._ext_section_start_index:
            item = self._groupbox_layout.takeAt(self._ext_section_start_index)
            if item and item.layout():
                while item.layout().count():
                    w = item.layout().takeAt(0).widget()
                    if w:
                        w.deleteLater()

        # Ajouter les nouvelles
        for axis_id, joint_index, label_text, unit in axes_info:
            row = QHBoxLayout()
            lbl = QLabel(f"{label_text} ({unit})")
            lbl.setFixedWidth(80)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

            btn_m = QPushButton("-")
            btn_m.setFixedWidth(48)
            btn_m.pressed.connect(
                lambda aid=axis_id, ji=joint_index: self.jog_external_minus_pressed.emit(aid, ji)
            )
            btn_m.released.connect(
                lambda aid=axis_id, ji=joint_index: self.jog_external_minus_released.emit(aid, ji)
            )

            btn_p = QPushButton("+")
            btn_p.setFixedWidth(48)
            btn_p.pressed.connect(
                lambda aid=axis_id, ji=joint_index: self.jog_external_plus_pressed.emit(aid, ji)
            )
            btn_p.released.connect(
                lambda aid=axis_id, ji=joint_index: self.jog_external_plus_released.emit(aid, ji)
            )

            row.addWidget(lbl)
            row.addWidget(btn_m)
            row.addWidget(btn_p)
            row.addStretch()
            self._groupbox_layout.addLayout(row)
            self._ext_buttons.append((btn_m, btn_p))
    