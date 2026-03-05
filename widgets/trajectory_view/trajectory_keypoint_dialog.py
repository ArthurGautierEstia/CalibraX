from __future__ import annotations

import math
from typing import Optional

from PyQt6.QtCore import pyqtSignal
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
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

import utils.math_utils as math_utils
from utils.mgi import MgiConfigKey
from utils.trajectory_keypoint_utils import resolve_keypoint_xyz
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.trajectory_result import TrajectoryResult
from models.robot_model import RobotModel
from widgets.cartesian_control_view.cartesian_control_widget import CartesianControlWidget
from widgets.joint_control_view.joints_control_widget import JointsControlWidget


class TrajectoryKeypointDialog(QDialog):
    """Dialog to edit a single trajectory keypoint."""
    updateRobotGhostRequested = pyqtSignal(object)
    previewKeypointChanged = pyqtSignal(object)

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
        self.setWindowTitle("Point clé")
        self._dialog_min_width = 620
        self.setMinimumWidth(self._dialog_min_width)
        self.robot_model = robot_model

        self.target_type_combo = QComboBox()
        self.use_current_target_btn = QPushButton("Valeurs courantes")
        self.use_home_target_btn = QPushButton("Position home")
        self.target_stack = QStackedWidget()
        self.main_tabs = QTabWidget()
        self.target_tab = QWidget()
        self.cubic_tab = QWidget()
        self.config_tab = QWidget()

        self.cartesian_target_widget = CartesianControlWidget(compact=True)
        self.joint_target_widget = JointsControlWidget(compact=True)
        self.cartesian_error_label = QLabel("")

        self.mode_combo = QComboBox()
        self.speed_spin = QDoubleSpinBox()
        self.speed_unit_label = QLabel("m/s")
        self.speed_unit_label.setMinimumWidth(self.speed_unit_label.sizeHint().width())
        self._ptp_speed_percent = 75.0
        self._linear_speed_mps = 0.5
        self._last_mode = KeypointMotionMode.PTP
        self._last_target_type = KeypointTargetType.CARTESIAN
        self._suspend_preview_emission = False

        self.cubic_group = QGroupBox("Vecteurs du segment cubique (entrant)")
        self.cubic_vector_1: list[QDoubleSpinBox] = []
        self.cubic_vector_2: list[QDoubleSpinBox] = []
        self.cubic_amplitude_1 = QDoubleSpinBox()
        self.cubic_amplitude_2 = QDoubleSpinBox()
        self.cubic_auto_start_btn = QPushButton("Auto (segment precedent)")
        self.cubic_auto_end_btn = QPushButton("Auto (segment suivant)")
        self.cubic_auto_update_adjacent_checkbox = QCheckBox(
            "Mettre a jour automatiquement les tangentes des segments adjacents"
        )
        self.cubic_auto_update_adjacent_checkbox.setChecked(True)
        self.cubic_hint_label = QLabel(
            "Direction : le triplet X/Y/Z est normalise automatiquement. "
            "Amplitude : min 0%, pas de maximum. "
            "100% correspond a la distance entre le point precedent et ce point."
        )

        self._context_keypoints: list[TrajectoryKeypoint] = []
        self._context_edited_row_index: int | None = None
        self._context_trajectory_result: TrajectoryResult | None = None

        self.config_checkboxes: list[QCheckBox] = []
        self.config_hint_label = QLabel("En mode articulaire, la configuration est deduite automatiquement depuis J1..J6.")

        self._setup_ui()
        self._setup_connections()
        self.load_keypoint(
            TrajectoryKeypoint(
                cartesian_target=list(self.robot_model.get_tcp_pose()),
                joint_target=list(self.robot_model.get_joints()),
            )
        )

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
        layout.setSizeConstraint(QLayout.SizeConstraint.SetDefaultConstraint)

        top_controls_layout = QHBoxLayout()
        self.target_type_combo.addItem("Cartesienne (X Y Z A B C)", KeypointTargetType.CARTESIAN.value)
        self.target_type_combo.addItem("Articulaire (J1 J2 J3 J4 J5 J6)", KeypointTargetType.JOINT.value)
        self.mode_combo.addItems([KeypointMotionMode.PTP.value, KeypointMotionMode.LINEAR.value, KeypointMotionMode.CUBIC.value])
        top_controls_layout.addWidget(QLabel("Type de target"))
        top_controls_layout.addWidget(self.target_type_combo)
        top_controls_layout.addSpacing(10)
        top_controls_layout.addWidget(QLabel("Mode"))
        top_controls_layout.addWidget(self.mode_combo)
        top_controls_layout.addSpacing(10)
        top_controls_layout.addWidget(QLabel("Vitesse"))
        top_controls_layout.addWidget(self.speed_spin)
        top_controls_layout.addWidget(self.speed_unit_label)
        top_controls_layout.addStretch()
        layout.addLayout(top_controls_layout)

        self.target_stack.addWidget(self._build_cartesian_target_group())
        self.target_stack.addWidget(self._build_joint_target_group())

        target_tab_layout = QVBoxLayout(self.target_tab)
        target_actions_layout = QHBoxLayout()
        target_actions_layout.addWidget(self.use_current_target_btn)
        target_actions_layout.addWidget(self.use_home_target_btn)
        target_actions_layout.addStretch()
        target_tab_layout.addLayout(target_actions_layout)
        target_tab_layout.addWidget(self.target_stack)

        cubic_tab_layout = QVBoxLayout(self.cubic_tab)

        cubic_layout = QVBoxLayout()

        cubic_label_width = 120

        start_group = QGroupBox("Debut de segment")
        start_layout = QGridLayout()
        start_direction_label = QLabel("Direction (X/Y/Z)")
        start_direction_label.setMinimumWidth(cubic_label_width)
        start_layout.addWidget(start_direction_label, 0, 0)
        for i in range(3):
            spin = self._make_spin(-1000.0, 1000.0, 3, 0.1)
            self.cubic_vector_1.append(spin)
            start_layout.addWidget(spin, 0, i + 1)

        start_amp_label = QLabel("Amplitude (%)")
        start_amp_label.setMinimumWidth(cubic_label_width)
        start_layout.addWidget(start_amp_label, 1, 0)
        self.cubic_amplitude_1.setRange(0.0, 1_000_000.0)
        self.cubic_amplitude_1.setDecimals(3)
        self.cubic_amplitude_1.setSingleStep(1.0)
        start_layout.addWidget(self.cubic_amplitude_1, 1, 1)

        self.cubic_auto_start_btn.setToolTip(
            "Calcule la tangente de debut du segment courant a partir du segment precedent."
        )
        start_auto_label = QLabel()
        start_auto_label.setMinimumWidth(cubic_label_width)
        start_layout.addWidget(start_auto_label, 2, 0)
        start_layout.addWidget(self.cubic_auto_start_btn, 2, 1)
        start_layout.setColumnStretch(4, 1)
        start_group.setLayout(start_layout)
        cubic_layout.addWidget(start_group)

        end_group = QGroupBox("Fin de segment")
        end_layout = QGridLayout()
        end_direction_label = QLabel("Direction (X/Y/Z)")
        end_direction_label.setMinimumWidth(cubic_label_width)
        end_layout.addWidget(end_direction_label, 0, 0)
        for i in range(3):
            spin = self._make_spin(-1000.0, 1000.0, 3, 0.1)
            self.cubic_vector_2.append(spin)
            end_layout.addWidget(spin, 0, i + 1)

        end_amp_label = QLabel("Amplitude (%)")
        end_amp_label.setMinimumWidth(cubic_label_width)
        end_layout.addWidget(end_amp_label, 1, 0)
        self.cubic_amplitude_2.setRange(0.0, 1_000_000.0)
        self.cubic_amplitude_2.setDecimals(3)
        self.cubic_amplitude_2.setSingleStep(1.0)
        end_layout.addWidget(self.cubic_amplitude_2, 1, 1)

        self.cubic_auto_end_btn.setToolTip(
            "Calcule la tangente de fin du segment courant a partir du segment suivant."
        )
        end_auto_label = QLabel()
        end_auto_label.setMinimumWidth(cubic_label_width)
        end_layout.addWidget(end_auto_label, 2, 0)
        end_layout.addWidget(self.cubic_auto_end_btn, 2, 1)
        end_layout.setColumnStretch(4, 1)
        end_group.setLayout(end_layout)
        cubic_layout.addWidget(end_group)

        cubic_group_layout = QVBoxLayout()
        cubic_group_layout.addLayout(cubic_layout)
        self.cubic_hint_label.setWordWrap(True)
        self.cubic_hint_label.setStyleSheet("color: #b0b0b0;")
        cubic_group_layout.addWidget(self.cubic_hint_label)
        self.cubic_group.setLayout(cubic_group_layout)

        self.cubic_auto_update_adjacent_checkbox.setToolTip(
            "En edition, ajuste automatiquement les tangentes des segments cubiques precedent/suivant."
        )
        cubic_tab_layout.addWidget(self.cubic_group)
        cubic_tab_layout.addStretch()

        config_group = QGroupBox("Configurations")
        config_layout = QVBoxLayout()

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
        config_layout.addStretch()
        config_group.setLayout(config_layout)

        config_tab_layout = QVBoxLayout(self.config_tab)
        config_tab_layout.addWidget(config_group)

        self.main_tabs.addTab(self.target_tab, "Target")
        self.main_tabs.addTab(self.cubic_tab, "Cubique")
        self.main_tabs.addTab(self.config_tab, "Configs")
        layout.addWidget(self.main_tabs)
        layout.addWidget(self.cubic_auto_update_adjacent_checkbox)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _build_cartesian_target_group(self) -> QGroupBox:
        group = QGroupBox("Target cartesienne (X Y Z A B C)")
        layout = QVBoxLayout()

        # Keep historical wide limits from previous dialog spinboxes.
        layout.addWidget(self.cartesian_target_widget)

        self.cartesian_error_label.setStyleSheet("color: #d9534f;")
        self.cartesian_error_label.setWordWrap(True)
        self.cartesian_error_label.setMinimumHeight(self.cartesian_error_label.fontMetrics().lineSpacing() * 2 + 4)
        self.cartesian_error_label.setText("")
        layout.addWidget(self.cartesian_error_label)

        group.setLayout(layout)
        return group

    def _build_joint_target_group(self) -> QGroupBox:
        group = QGroupBox("Target articulaire (J1 J2 J3 J4 J5 J6)")
        layout = QVBoxLayout()

        self.joint_target_widget.update_axis_limits(self.robot_model.get_axis_limits())
        self.joint_target_widget.btn_home_position.hide()
        self.joint_target_widget.btn_position_zero.hide()
        self.joint_target_widget.btn_position_transport.hide()
        layout.addWidget(self.joint_target_widget)

        group.setLayout(layout)
        return group

    def _setup_connections(self) -> None:
        self.target_type_combo.currentIndexChanged.connect(self._on_target_type_changed)
        self.use_current_target_btn.clicked.connect(self._on_use_current_target_clicked)
        self.use_home_target_btn.clicked.connect(self._on_use_home_target_clicked)
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        self.speed_spin.valueChanged.connect(self._on_speed_changed)
        for cb in self.config_checkboxes:
            cb.toggled.connect(self._on_config_checkbox_toggled)
        for spin in self.cubic_vector_1:
            spin.valueChanged.connect(self._on_cubic_vectors_changed)
        for spin in self.cubic_vector_2:
            spin.valueChanged.connect(self._on_cubic_vectors_changed)
        self.cubic_amplitude_1.valueChanged.connect(self._on_cubic_vectors_changed)
        self.cubic_amplitude_2.valueChanged.connect(self._on_cubic_vectors_changed)
        self.cubic_auto_start_btn.clicked.connect(self._on_auto_start_tangent_clicked)
        self.cubic_auto_end_btn.clicked.connect(self._on_auto_end_tangent_clicked)
        self.joint_target_widget.joint_value_changed.connect(self._on_joint_target_changed)
        self.cartesian_target_widget.cartesian_value_changed.connect(self._on_cartesian_target_changed)

    def _on_use_current_target_clicked(self) -> None:
        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            self.cartesian_target_widget.set_all_cartesian(list(self.robot_model.get_tcp_pose()))
            self._emit_ghost_update()
            return

        self.joint_target_widget.set_all_joints(list(self.robot_model.get_joints()))
        self._emit_ghost_update()

    def _on_use_home_target_clicked(self) -> None:
        home_joints = list(self.robot_model.get_home_position())
        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            fk_result = self.robot_model.compute_fk_joints(home_joints)
            if fk_result is not None:
                _, _, home_pose, _, _ = fk_result
                self.cartesian_target_widget.set_all_cartesian([float(v) for v in home_pose[:6]])
            else:
                self.cartesian_target_widget.set_all_cartesian(list(self.robot_model.get_tcp_pose()))
            self._emit_ghost_update()
            return

        self.joint_target_widget.set_all_joints(home_joints)
        self._emit_ghost_update()

    def _compute_joint_values_from_cartesian_target(self) -> list[float]:
        target = self.cartesian_target_widget.get_cartesian_values()
        mgi_result = self.robot_model.compute_ik_target(target)
        valid_solutions = mgi_result.get_valid_solutions()
        if not valid_solutions:
            self._set_cartesian_error("Aucune solution valide pour la target cartesienne.")
            return []

        selected_allowed = set(self._selected_allowed_configs())
        if not selected_allowed:
            self._set_cartesian_error("Aucune configuration autorisee selectionnee.")
            return []

        allowed_valid = {
            key: solution
            for key, solution in valid_solutions.items()
            if key in selected_allowed and len(solution.joints) >= 6
        }
        if not allowed_valid:
            self._set_cartesian_error("Aucune solution valide dans les configurations autorisees.")
            return []

        reference_joints = [float(v) for v in self.joint_target_widget.get_all_joints()[:6]]
        while len(reference_joints) < 6:
            reference_joints.append(0.0)
        reference_joints_rad = [math.radians(v) for v in reference_joints]

        weights = [float(v) for v in self.robot_model.get_joint_weights()[:6]]
        while len(weights) < 6:
            weights.append(1.0)

        best_joints: list[float] | None = None
        best_distance = float("inf")
        for solution in allowed_valid.values():
            solution_joints = [float(v) for v in solution.joints[:6]]
            while len(solution_joints) < 6:
                solution_joints.append(0.0)
            solution_joints_rad = [math.radians(v) for v in solution_joints]
            distance = sum(weights[i] * (reference_joints_rad[i] - solution_joints_rad[i]) ** 2 for i in range(6))
            if distance < best_distance:
                best_distance = distance
                best_joints = solution_joints

        if best_joints is not None:
            self._set_cartesian_error("")
            return best_joints

        self._set_cartesian_error("Aucune solution exploitable pour la target cartesienne.")
        return []

    def _sync_joint_target_from_cartesian_target(self) -> None:
        joints = self._compute_joint_values_from_cartesian_target()
        if len(joints) >= 6:
            self.joint_target_widget.set_all_joints(joints)

    def _sync_cartesian_target_from_joint_target(self) -> None:
        fk_result = self.robot_model.compute_fk_joints(self.joint_target_widget.get_all_joints())
        if fk_result is None:
            return
        _, _, pose, _, _ = fk_result
        if len(pose) >= 6:
            self.cartesian_target_widget.set_all_cartesian([float(v) for v in pose[:6]])
            self._set_cartesian_error("")

    def _emit_live_preview(self) -> None:
        if self._suspend_preview_emission:
            return
        self.previewKeypointChanged.emit(self.get_keypoint())

    def _emit_ghost_update(self) -> None:
        self._emit_live_preview()
        payload = self._build_ghost_payload_and_sync_targets()
        self.updateRobotGhostRequested.emit(payload)

    def _build_ghost_payload_and_sync_targets(self) -> dict:
        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            joints = self._compute_joint_values_from_cartesian_target()
            if len(joints) >= 6:
                self.joint_target_widget.set_all_joints(joints)
            return {"joints": joints}

        joints = [float(v) for v in self.joint_target_widget.get_all_joints()[:6]]
        payload: dict = {"joints": joints}
        if len(joints) < 6:
            return payload

        fk_result = self.robot_model.compute_fk_joints(joints)
        if fk_result is None:
            return payload
        _, corrected_matrices, pose, _, _ = fk_result
        if len(pose) >= 6:
            self.cartesian_target_widget.set_all_cartesian([float(v) for v in pose[:6]])
            self._set_cartesian_error("")
        payload["corrected_matrices"] = corrected_matrices
        return payload

    def _set_cartesian_error(self, message: str) -> None:
        self.cartesian_error_label.setText(message)

    def _current_target_type(self) -> KeypointTargetType:
        return KeypointTargetType(self.target_type_combo.currentData())

    def _current_mode(self) -> KeypointMotionMode:
        return KeypointMotionMode(self.mode_combo.currentText())

    def _checkbox_for_config(self, key: MgiConfigKey) -> QCheckBox:
        idx = self.CONFIG_ORDER.index(key)
        return self.config_checkboxes[idx]

    def _selected_allowed_configs(self) -> list[MgiConfigKey]:
        selected: list[MgiConfigKey] = []
        for key, cb in zip(self.CONFIG_ORDER, self.config_checkboxes):
            if cb.isChecked():
                selected.append(key)
        return selected

    def set_keypoint_context(
        self,
        keypoints: list[TrajectoryKeypoint],
        edited_row_index: int | None,
        trajectory_result: TrajectoryResult | None = None,
    ) -> None:
        self._context_keypoints = list(keypoints)
        self._context_edited_row_index = edited_row_index if isinstance(edited_row_index, int) else None
        self._context_trajectory_result = trajectory_result
        is_editing_existing_point = (
            self._context_edited_row_index is not None
            and 0 <= self._context_edited_row_index < len(self._context_keypoints)
        )
        self.cubic_auto_update_adjacent_checkbox.setEnabled(is_editing_existing_point)
        self._update_auto_tangent_buttons_state()

    def should_auto_update_adjacent_cubic_tangents(self) -> bool:
        return self.cubic_auto_update_adjacent_checkbox.isEnabled() and self.cubic_auto_update_adjacent_checkbox.isChecked()

    def _resolve_current_target_xyz(self) -> list[float] | None:
        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            target = self.cartesian_target_widget.get_cartesian_values()
            if len(target) < 3:
                return None
            return [float(target[0]), float(target[1]), float(target[2])]

        fk_result = self.robot_model.compute_fk_joints(self.joint_target_widget.get_all_joints())
        if fk_result is None:
            return None
        _, _, pose, _, _ = fk_result
        if len(pose) < 3:
            return None
        return [float(pose[0]), float(pose[1]), float(pose[2])]

    def _get_previous_segment_end_tangent_for_auto(self) -> list[float] | None:
        row = self._context_edited_row_index
        if row is None or row < 1:
            return None
        trajectory_result = self._context_trajectory_result
        if trajectory_result is None:
            return None
        previous_segment_index = row - 1
        if previous_segment_index >= len(trajectory_result.segments):
            return None
        previous_segment = trajectory_result.segments[previous_segment_index]
        direction = [float(v) for v in previous_segment.in_direction[:3]]
        if len(direction) < 3:
            return None
        return direction

    def _get_next_segment_start_tangent_for_auto(self) -> list[float] | None:
        row = self._context_edited_row_index
        if row is None:
            return None
        trajectory_result = self._context_trajectory_result
        if trajectory_result is None:
            return None
        next_segment_index = row + 1
        if next_segment_index >= len(trajectory_result.segments):
            return None
        next_segment = trajectory_result.segments[next_segment_index]
        direction = [float(v) for v in next_segment.out_direction[:3]]
        if len(direction) < 3:
            return None
        return direction

    def _update_auto_tangent_buttons_state(self) -> None:
        is_cubic = self._current_mode() == KeypointMotionMode.CUBIC
        self.cubic_auto_start_btn.setEnabled(
            is_cubic and self._get_previous_segment_end_tangent_for_auto() is not None
        )
        self.cubic_auto_end_btn.setEnabled(
            is_cubic and self._get_next_segment_start_tangent_for_auto() is not None
        )

    def _apply_cubic_start_tangent_direction(self, direction_xyz: list[float], emit_preview: bool = True) -> None:
        for idx, spin in enumerate(self.cubic_vector_1):
            spin.blockSignals(True)
            spin.setValue(float(direction_xyz[idx]))
            spin.blockSignals(False)
        if emit_preview:
            self._emit_live_preview()

    def _apply_cubic_end_tangent_direction(self, direction_xyz: list[float], emit_preview: bool = True) -> None:
        for idx, spin in enumerate(self.cubic_vector_2):
            spin.blockSignals(True)
            spin.setValue(float(direction_xyz[idx]))
            spin.blockSignals(False)
        if emit_preview:
            self._emit_live_preview()

    def _compute_end_tangent_from_first_tangent_look_at(
        self,
        start_direction_xyz: list[float],
    ) -> list[float] | None:
        row = self._context_edited_row_index
        if row is None or row < 1 or (row - 1) >= len(self._context_keypoints):
            return None

        previous_point_xyz = resolve_keypoint_xyz(self.robot_model, self._context_keypoints[row - 1])
        current_point_xyz = self._resolve_current_target_xyz()
        if previous_point_xyz is None or current_point_xyz is None:
            return None

        look_at_start_point = [
            previous_point_xyz[0] + float(start_direction_xyz[0]),
            previous_point_xyz[1] + float(start_direction_xyz[1]),
            previous_point_xyz[2] + float(start_direction_xyz[2]),
        ]
        return [
            look_at_start_point[0] - current_point_xyz[0],
            look_at_start_point[1] - current_point_xyz[1],
            look_at_start_point[2] - current_point_xyz[2],
        ]

    def _compute_start_tangent_from_end_tangent_look_at(
        self,
        end_direction_xyz: list[float],
    ) -> list[float] | None:
        row = self._context_edited_row_index
        if row is None or (row + 1) >= len(self._context_keypoints):
            return None

        current_point_xyz = self._resolve_current_target_xyz()
        next_point_xyz = resolve_keypoint_xyz(self.robot_model, self._context_keypoints[row + 1])
        if current_point_xyz is None or next_point_xyz is None:
            return None

        look_at_end_point = [
            next_point_xyz[0] + float(end_direction_xyz[0]),
            next_point_xyz[1] + float(end_direction_xyz[1]),
            next_point_xyz[2] + float(end_direction_xyz[2]),
        ]
        return [
            look_at_end_point[0] - current_point_xyz[0],
            look_at_end_point[1] - current_point_xyz[1],
            look_at_end_point[2] - current_point_xyz[2],
        ]

    def _on_auto_start_tangent_clicked(self) -> None:
        direction = self._get_previous_segment_end_tangent_for_auto()
        if direction is None:
            return
        start_direction = [-float(d) for d in direction[:3]]
        self._apply_cubic_start_tangent_direction(start_direction, emit_preview=False)

        current_end_direction = [float(spin.value()) for spin in self.cubic_vector_2]
        if math_utils.is_near_zero_vector_xyz(current_end_direction):
            end_direction = self._compute_end_tangent_from_first_tangent_look_at(start_direction)
            if end_direction is not None:
                self._apply_cubic_end_tangent_direction(end_direction, emit_preview=False)
        self._emit_live_preview()

    def _on_auto_end_tangent_clicked(self) -> None:
        direction = self._get_next_segment_start_tangent_for_auto()
        if direction is None:
            return
        end_direction = [-float(d) for d in direction[:3]]
        self._apply_cubic_end_tangent_direction(end_direction, emit_preview=False)

        current_start_direction = [float(spin.value()) for spin in self.cubic_vector_1]
        if math_utils.is_near_zero_vector_xyz(current_start_direction):
            start_direction = self._compute_start_tangent_from_end_tangent_look_at(end_direction)
            if start_direction is not None:
                self._apply_cubic_start_tangent_direction(start_direction, emit_preview=False)
        self._emit_live_preview()

    def _store_current_speed(self) -> None:
        current = float(self.speed_spin.value())
        if self._last_mode == KeypointMotionMode.PTP:
            self._ptp_speed_percent = self._clamp(current, 0.0, 100.0)
        else:
            self._linear_speed_mps = self._clamp(current, 0.0, 2.0)

    def _try_minimize_window_size(self) -> None:
        # Keep a stable minimum width and let Qt handle height changes.
        # Forcing adjustSize/resize during combobox switches can trigger
        # Windows setGeometry warnings when the computed height is constrained.
        self.setMinimumWidth(self._dialog_min_width)
        if self.layout() is not None:
            self.layout().activate()

    def _on_mode_changed(self, _mode: str) -> None:
        previous_mode = self._last_mode
        self._store_current_speed()
        self._last_mode = self._current_mode()
        self._update_cubic_visibility()
        self._update_speed_editor()
        if previous_mode != KeypointMotionMode.CUBIC and self._last_mode == KeypointMotionMode.CUBIC:
            self._on_auto_start_tangent_clicked()
            self._on_auto_end_tangent_clicked()
        self._try_minimize_window_size()
        self._emit_ghost_update()

    def _on_target_type_changed(self, _idx: int) -> None:
        previous_type = self._last_target_type
        new_type = self._current_target_type()
        if previous_type != new_type:
            if previous_type == KeypointTargetType.CARTESIAN and new_type == KeypointTargetType.JOINT:
                self._sync_joint_target_from_cartesian_target()
            elif previous_type == KeypointTargetType.JOINT and new_type == KeypointTargetType.CARTESIAN:
                self._sync_cartesian_target_from_joint_target()
        self._update_target_editors()
        self._try_minimize_window_size()
        self._emit_ghost_update()

    def _on_joint_target_changed(self, *_args) -> None:
        if self._current_target_type() == KeypointTargetType.JOINT:
            self._sync_joint_mode_configs()
        self._emit_ghost_update()

    def _on_cartesian_target_changed(self, *_args) -> None:
        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            self._emit_ghost_update()

    def _on_config_checkbox_toggled(self, _checked: bool) -> None:
        if self._current_target_type() != KeypointTargetType.CARTESIAN:
            return

        selected = self._selected_allowed_configs()
        if not selected:
            cb = self._checkbox_for_config(self.CONFIG_ORDER[0])
            cb.blockSignals(True)
            cb.setChecked(True)
            cb.blockSignals(False)
            self._emit_ghost_update()
            return

        self._emit_ghost_update()

    def _on_speed_changed(self, *_args) -> None:
        self._emit_live_preview()

    def _on_cubic_vectors_changed(self, *_args) -> None:
        self._emit_live_preview()

    def _sync_joint_mode_configs(self) -> None:
        joint_target = self.joint_target_widget.get_all_joints()
        deduced = TrajectoryKeypoint.identify_config_from_joint_target(
            joint_target,
            self.robot_model.get_config_identifier(),
        )
        for key, cb in zip(self.CONFIG_ORDER, self.config_checkboxes):
            cb.blockSignals(True)
            cb.setChecked(key == deduced)
            cb.blockSignals(False)

    def _update_target_editors(self) -> None:
        is_joint = self._current_target_type() == KeypointTargetType.JOINT
        self.target_stack.setCurrentIndex(1 if is_joint else 0)

        for cb in self.config_checkboxes:
            cb.setEnabled(not is_joint)
        self.config_hint_label.setText(
            "En mode articulaire, la configuration est deduite automatiquement depuis J1..J6."
            if is_joint else ""
        )

        if is_joint:
            self._set_cartesian_error("")
            self._sync_joint_mode_configs()
        self._last_target_type = KeypointTargetType.JOINT if is_joint else KeypointTargetType.CARTESIAN

    def _update_cubic_visibility(self) -> None:
        is_cubic = self._current_mode() == KeypointMotionMode.CUBIC
        cubic_tab_index = self.main_tabs.indexOf(self.cubic_tab)
        if cubic_tab_index >= 0:
            self.main_tabs.setTabEnabled(cubic_tab_index, is_cubic)
            if hasattr(self.main_tabs, "setTabVisible"):
                self.main_tabs.setTabVisible(cubic_tab_index, is_cubic)
            if is_cubic:
                self.main_tabs.setCurrentWidget(self.cubic_tab)
            elif self.main_tabs.currentIndex() == cubic_tab_index:
                self.main_tabs.setCurrentWidget(self.target_tab)
        self._update_auto_tangent_buttons_state()

    def _update_speed_editor(self) -> None:
        self.speed_spin.blockSignals(True)
        try:
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
        finally:
            self.speed_spin.blockSignals(False)

    def load_keypoint(self, keypoint: TrajectoryKeypoint) -> None:
        self._suspend_preview_emission = True
        target_type_idx = self.target_type_combo.findData(keypoint.target_type.value)
        if target_type_idx >= 0:
            self.target_type_combo.blockSignals(True)
            self.target_type_combo.setCurrentIndex(target_type_idx)
            self.target_type_combo.blockSignals(False)

        self.cartesian_target_widget.set_all_cartesian(keypoint.cartesian_target)
        self.joint_target_widget.set_all_joints(keypoint.joint_target)

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
        self.cubic_amplitude_1.setValue(float(keypoint.cubic_amplitudes_percent[0]))
        self.cubic_amplitude_2.setValue(float(keypoint.cubic_amplitudes_percent[1]))

        allowed = set(keypoint.allowed_configs)
        for key, cb in zip(self.CONFIG_ORDER, self.config_checkboxes):
            cb.setChecked(key in allowed)

        self._update_target_editors()
        self._update_cubic_visibility()
        self._update_speed_editor()
        self._try_minimize_window_size()
        self._suspend_preview_emission = False
        self._emit_ghost_update()

    def get_keypoint(self) -> TrajectoryKeypoint:
        self._store_current_speed()
        target_type = self._current_target_type()
        mode = self._current_mode()

        return TrajectoryKeypoint(
            target_type=target_type,
            cartesian_target=self.cartesian_target_widget.get_cartesian_values(),
            joint_target=self.joint_target_widget.get_all_joints(),
            mode=mode,
            cubic_vectors=[
                [spin.value() for spin in self.cubic_vector_1],
                [spin.value() for spin in self.cubic_vector_2],
            ],
            cubic_amplitudes_percent=[
                self.cubic_amplitude_1.value(),
                self.cubic_amplitude_2.value(),
            ],
            allowed_configs=self._selected_allowed_configs(),
            ptp_speed_percent=self._ptp_speed_percent,
            linear_speed_mps=self._linear_speed_mps,
        )
