from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLayout,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from mgi import MgiConfigKey
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.robot_model import RobotModel


class TrajectoryKeypointDialog(QDialog):
    """Dialog to edit a single trajectory keypoint."""

    CONFIG_ORDER = [
        MgiConfigKey.FUN,
        MgiConfigKey.FUF,
        MgiConfigKey.FDN,
        MgiConfigKey.FDF,
        MgiConfigKey.BUN,
        MgiConfigKey.BUF,
        MgiConfigKey.BDN,
        MgiConfigKey.BDF,
    ]

    def __init__(self, robot_model: RobotModel, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Point cle")
        self.robot_model = robot_model

        self.target_type_combo = QComboBox()
        self.use_current_target_btn = QPushButton("Valeurs courantes")
        self.target_stack = QStackedWidget()

        self.cartesian_target_spins: list[QDoubleSpinBox] = []
        self.joint_target_spins: list[QDoubleSpinBox] = []

        self.mode_combo = QComboBox()
        self.speed_spin = QDoubleSpinBox()
        self.speed_unit_label = QLabel("m/s")
        self.speed_unit_label.setMinimumWidth(self.speed_unit_label.sizeHint().width())
        self._ptp_speed_percent = 75.0
        self._linear_speed_mps = 0.5
        self._last_mode = KeypointMotionMode.PTP

        self.cubic_group = QGroupBox("Vecteurs de la cubique")
        self.cubic_vector_1: list[QDoubleSpinBox] = []
        self.cubic_vector_2: list[QDoubleSpinBox] = []

        self.favorite_config_combo = QComboBox()
        self.config_checkboxes: list[QCheckBox] = []
        self.config_hint_label = QLabel("En mode articulaire, la configuration est deduite automatiquement depuis J1..J6.")

        tmpLabel = QLabel("XX")
        self.targetLblMinWidth = tmpLabel.sizeHint().width()

        self._setup_ui()
        self._setup_connections()
        self.load_keypoint(TrajectoryKeypoint())

    @staticmethod
    def _make_spin(minimum: float, maximum: float, decimals: int = 3, step: float = 0.1) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        return spin

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSizeConstraint(QLayout.SizeConstraint.SetMinAndMaxSize)

        target_mode_group = QGroupBox("Type de target")
        target_mode_layout = QHBoxLayout()
        self.target_type_combo.addItem("Cartesienne (X Y Z A B C)", KeypointTargetType.CARTESIAN.value)
        self.target_type_combo.addItem("Articulaire (J1 J2 J3 J4 J5 J6)", KeypointTargetType.JOINT.value)
        target_mode_layout.addWidget(self.target_type_combo)
        target_mode_layout.addWidget(self.use_current_target_btn)
        target_mode_group.setLayout(target_mode_layout)
        layout.addWidget(target_mode_group)

        self.target_stack.addWidget(self._build_cartesian_target_group())
        self.target_stack.addWidget(self._build_joint_target_group())
        layout.addWidget(self.target_stack)

        motion_group = QGroupBox("Type de mouvement")
        motion_layout = QHBoxLayout()
        self.mode_combo.addItems([KeypointMotionMode.PTP.value, KeypointMotionMode.LINEAR.value, KeypointMotionMode.CUBIC.value])
        motion_layout.addWidget(QLabel("Mode"))
        motion_layout.addWidget(self.mode_combo)
        motion_layout.addStretch()
        motion_group.setLayout(motion_layout)
        layout.addWidget(motion_group)

        cubic_layout = QGridLayout()
        cubic_layout.addWidget(QLabel("Vecteur Out"), 0, 0)
        for i in range(3):
            spin = self._make_spin(-10000.0, 10000.0, 3, 0.1)
            self.cubic_vector_1.append(spin)
            cubic_layout.addWidget(spin, 0, i + 1)

        cubic_layout.addWidget(QLabel("Vecteur In"), 1, 0)
        for i in range(3):
            spin = self._make_spin(-10000.0, 10000.0, 3, 0.1)
            self.cubic_vector_2.append(spin)
            cubic_layout.addWidget(spin, 1, i + 1)
        self.cubic_group.setLayout(cubic_layout)
        layout.addWidget(self.cubic_group)

        config_group = QGroupBox("Configurations")
        config_layout = QVBoxLayout()

        favorite_row = QHBoxLayout()
        favorite_row.addWidget(QLabel("Configuration favorite"))
        for key in self.CONFIG_ORDER:
            self.favorite_config_combo.addItem(key.name, key.name)
        favorite_row.addWidget(self.favorite_config_combo)
        config_layout.addLayout(favorite_row)

        config_grid = QGridLayout()
        for idx, key in enumerate(self.CONFIG_ORDER):
            row = 0 if idx < 4 else 1
            col = idx % 4
            cb = QCheckBox(key.name)
            cb.setChecked(True)
            self.config_checkboxes.append(cb)
            config_grid.addWidget(cb, row, col)
        config_layout.addLayout(config_grid)

        self.config_hint_label.setWordWrap(True)
        self.config_hint_label.setMinimumHeight(self.config_hint_label.fontMetrics().lineSpacing() * 2 + 4)
        self.config_hint_label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        config_layout.addWidget(self.config_hint_label)
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)

        speed_group = QGroupBox("Vitesse")
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(self.speed_unit_label)
        speed_layout.addWidget(self.speed_spin)
        speed_layout.addStretch()
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _build_cartesian_target_group(self) -> QGroupBox:
        group = QGroupBox("Target cartesienne (X Y Z A B C)")
        grid = QGridLayout()
        labels = ["X", "Y", "Z", "A", "B", "C"]
        for i, label in enumerate(labels):
            row = 0 if i < 3 else 1
            col = (i % 3) * 2
            lbl = QLabel(label)
            lbl.setFixedWidth(self.targetLblMinWidth)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            
            grid.addWidget(lbl, row, col)
            spin = self._make_spin(-10000.0, 10000.0, 3, 0.1)
            self.cartesian_target_spins.append(spin)
            grid.addWidget(spin, row, col + 1)
        group.setLayout(grid)
        return group

    def _build_joint_target_group(self) -> QGroupBox:
        group = QGroupBox("Target articulaire (J1 J2 J3 J4 J5 J6)")
        grid = QGridLayout()
        axis_limits = self.robot_model.get_axis_limits()
        for i in range(6):
            row = 0 if i < 3 else 1
            col = (i % 3) * 2
            lbl = QLabel(f"J{i + 1}")
            lbl.setFixedWidth(self.targetLblMinWidth)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)

            grid.addWidget(lbl, row, col)
            axis_min, axis_max = axis_limits[i]
            spin = self._make_spin(axis_min, axis_max, 3, 0.1)
            self.joint_target_spins.append(spin)
            grid.addWidget(spin, row, col + 1)
        group.setLayout(grid)
        return group

    def _setup_connections(self) -> None:
        self.target_type_combo.currentIndexChanged.connect(self._on_target_type_changed)
        self.use_current_target_btn.clicked.connect(self._on_use_current_target_clicked)
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        self.favorite_config_combo.currentIndexChanged.connect(self._on_favorite_changed)
        for cb in self.config_checkboxes:
            cb.toggled.connect(self._on_config_checkbox_toggled)
        for spin in self.joint_target_spins:
            spin.valueChanged.connect(self._on_joint_target_changed)

    def _on_use_current_target_clicked(self) -> None:
        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            current_target = list(self.robot_model.get_tcp_pose())
            for i, spin in enumerate(self.cartesian_target_spins):
                spin.setValue(current_target[i] if i < len(current_target) else 0.0)
            return

        current_joints = list(self.robot_model.get_joints())
        for i, spin in enumerate(self.joint_target_spins):
            spin.setValue(current_joints[i] if i < len(current_joints) else 0.0)

    def _current_target_type(self) -> KeypointTargetType:
        return KeypointTargetType(self.target_type_combo.currentData())

    def _current_mode(self) -> KeypointMotionMode:
        return KeypointMotionMode(self.mode_combo.currentText())

    def _current_favorite_config(self) -> MgiConfigKey:
        return MgiConfigKey[self.favorite_config_combo.currentData()]

    def _checkbox_for_config(self, key: MgiConfigKey) -> QCheckBox:
        idx = self.CONFIG_ORDER.index(key)
        return self.config_checkboxes[idx]

    def _selected_allowed_configs(self) -> list[MgiConfigKey]:
        selected: list[MgiConfigKey] = []
        for key, cb in zip(self.CONFIG_ORDER, self.config_checkboxes):
            if cb.isChecked():
                selected.append(key)
        return selected

    def _set_favorite_combo(self, key: MgiConfigKey) -> None:
        idx = self.favorite_config_combo.findData(key.name)
        if idx >= 0:
            self.favorite_config_combo.blockSignals(True)
            self.favorite_config_combo.setCurrentIndex(idx)
            self.favorite_config_combo.blockSignals(False)

    def _store_current_speed(self) -> None:
        current = float(self.speed_spin.value())
        if self._last_mode == KeypointMotionMode.PTP:
            self._ptp_speed_percent = self._clamp(current, 0.0, 100.0)
        else:
            self._linear_speed_mps = self._clamp(current, 0.0, 2.0)

    def _try_minimize_window_size(self) -> None:
        self.setMinimumSize(0, 0)
        self.adjustSize()
        QTimer.singleShot(0, self.adjustSize)

    def _on_mode_changed(self, _mode: str) -> None:
        self._store_current_speed()
        self._last_mode = self._current_mode()
        self._update_cubic_visibility()
        self._update_speed_editor()
        self._try_minimize_window_size()

    def _on_target_type_changed(self, _idx: int) -> None:
        self._update_target_editors()
        self._try_minimize_window_size()

    def _on_joint_target_changed(self, _value: float) -> None:
        if self._current_target_type() == KeypointTargetType.JOINT:
            self._sync_joint_mode_configs()

    def _on_config_checkbox_toggled(self, _checked: bool) -> None:
        if self._current_target_type() != KeypointTargetType.CARTESIAN:
            return

        selected = self._selected_allowed_configs()
        if not selected:
            favorite = self._current_favorite_config()
            cb = self._checkbox_for_config(favorite)
            cb.blockSignals(True)
            cb.setChecked(True)
            cb.blockSignals(False)
            return

        favorite = self._current_favorite_config()
        if favorite not in selected:
            self._set_favorite_combo(selected[0])

    def _on_favorite_changed(self, _idx: int) -> None:
        if self._current_target_type() != KeypointTargetType.CARTESIAN:
            return
        favorite = self._current_favorite_config()
        cb = self._checkbox_for_config(favorite)
        if not cb.isChecked():
            cb.blockSignals(True)
            cb.setChecked(True)
            cb.blockSignals(False)

    def _sync_joint_mode_configs(self) -> None:
        joint_target = [spin.value() for spin in self.joint_target_spins]
        deduced = TrajectoryKeypoint.identify_config_from_joint_target(
            joint_target,
            self.robot_model.get_config_identifier(),
        )
        self._set_favorite_combo(deduced)
        for key, cb in zip(self.CONFIG_ORDER, self.config_checkboxes):
            cb.blockSignals(True)
            cb.setChecked(key == deduced)
            cb.blockSignals(False)

    def _update_target_editors(self) -> None:
        is_joint = self._current_target_type() == KeypointTargetType.JOINT
        self.target_stack.setCurrentIndex(1 if is_joint else 0)

        for cb in self.config_checkboxes:
            cb.setEnabled(not is_joint)
        self.favorite_config_combo.setEnabled(not is_joint)
        self.config_hint_label.setText(
            "En mode articulaire, la configuration est deduite automatiquement depuis J1..J6."
            if is_joint else ""
        )

        if is_joint:
            self._sync_joint_mode_configs()

    def _update_cubic_visibility(self) -> None:
        self.cubic_group.setVisible(self._current_mode() == KeypointMotionMode.CUBIC)

    def _update_speed_editor(self) -> None:
        if self._current_mode() == KeypointMotionMode.PTP:
            self.speed_unit_label.setText("%")
            self.speed_spin.setRange(0.0, 100.0)
            self.speed_spin.setDecimals(1)
            self.speed_spin.setSingleStep(1.0)
            self.speed_spin.setValue(self._clamp(self._ptp_speed_percent, 0.0, 100.0))
        else:
            self.speed_unit_label.setText("m/s")
            self.speed_spin.setRange(0.0, 2.0)
            self.speed_spin.setDecimals(3)
            self.speed_spin.setSingleStep(0.01)
            self.speed_spin.setValue(self._clamp(self._linear_speed_mps, 0.0, 2.0))

    def load_keypoint(self, keypoint: TrajectoryKeypoint) -> None:
        target_type_idx = self.target_type_combo.findData(keypoint.target_type.value)
        if target_type_idx >= 0:
            self.target_type_combo.blockSignals(True)
            self.target_type_combo.setCurrentIndex(target_type_idx)
            self.target_type_combo.blockSignals(False)

        for i, spin in enumerate(self.cartesian_target_spins):
            spin.setValue(keypoint.cartesian_target[i])
        for i, spin in enumerate(self.joint_target_spins):
            spin.setValue(keypoint.joint_target[i])

        self._ptp_speed_percent = keypoint.ptp_speed_percent
        self._linear_speed_mps = keypoint.linear_speed_mps

        self.mode_combo.blockSignals(True)
        self.mode_combo.setCurrentText(keypoint.mode.value)
        self.mode_combo.blockSignals(False)
        self._last_mode = keypoint.mode

        for i, spin in enumerate(self.cubic_vector_1):
            spin.setValue(keypoint.cubic_vectors[0][i])
        for i, spin in enumerate(self.cubic_vector_2):
            spin.setValue(keypoint.cubic_vectors[1][i])

        allowed = set(keypoint.allowed_configs)
        for key, cb in zip(self.CONFIG_ORDER, self.config_checkboxes):
            cb.setChecked(key in allowed)
        self._set_favorite_combo(keypoint.favorite_config)

        self._update_target_editors()
        self._update_cubic_visibility()
        self._update_speed_editor()
        self._try_minimize_window_size()

    def get_keypoint(self) -> TrajectoryKeypoint:
        self._store_current_speed()
        target_type = self._current_target_type()
        mode = self._current_mode()

        return TrajectoryKeypoint(
            target_type=target_type,
            cartesian_target=[spin.value() for spin in self.cartesian_target_spins],
            joint_target=[spin.value() for spin in self.joint_target_spins],
            mode=mode,
            cubic_vectors=[
                [spin.value() for spin in self.cubic_vector_1],
                [spin.value() for spin in self.cubic_vector_2],
            ],
            allowed_configs=self._selected_allowed_configs(),
            favorite_config=self._current_favorite_config(),
            ptp_speed_percent=self._ptp_speed_percent,
            linear_speed_mps=self._linear_speed_mps,
        )
