from __future__ import annotations

import math
from typing import Optional

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QAbstractItemView,
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
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
    QHeaderView,
)

import utils.math_utils as math_utils
from utils.mgi import MgiConfigKey, MgiResultStatus
from utils.trajectory_status import (
    build_segment_issue_messages,
    build_trajectory_issue_messages,
    join_issue_messages,
)
from utils.trajectory_keypoint_utils import resolve_keypoint_xyz
from models.trajectory_keypoint import (
    ConfigurationPolicy,
    KeypointMotionMode,
    KeypointTargetType,
    TrajectoryKeypoint,
)
from models.reference_frame import ReferenceFrame
from models.trajectory_result import TrajectoryResult
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from utils.reference_frame_utils import convert_pose_from_base_frame, convert_pose_to_base_frame
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

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Point clé")
        self._dialog_min_width = 620
        self.setMinimumWidth(self._dialog_min_width)
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model

        self.target_type_combo = QComboBox()
        self.use_current_target_btn = QPushButton("Valeurs courantes")
        self.use_home_target_btn = QPushButton("Position home")
        self.use_initial_target_btn = QPushButton("Position initiale")
        self.target_stack = QStackedWidget()
        self.main_tabs = QTabWidget()
        self.target_tab = QWidget()
        self.cubic_tab = QWidget()
        self.config_tab = QWidget()

        self.cartesian_target_widget = CartesianControlWidget(compact=True)
        self.joint_target_widget = JointsControlWidget(compact=True)
        self.cartesian_error_label = QLabel("")
        self.cartesian_solutions_table_left = QTableWidget()
        self.cartesian_solutions_table_right = QTableWidget()

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
        self.linear_group = QGroupBox("Tangentes du segment lineaire (entrant)")
        self.cubic_vector_1: list[QDoubleSpinBox] = []
        self.cubic_vector_2: list[QDoubleSpinBox] = []
        self.cubic_amplitude_1 = QDoubleSpinBox()
        self.cubic_amplitude_2 = QDoubleSpinBox()
        self.linear_tangent_ratio_1 = QDoubleSpinBox()
        self.linear_tangent_ratio_2 = QDoubleSpinBox()
        self.linear_tangent_link_checkbox = QCheckBox("Lier debut/fin")
        self.cubic_auto_start_btn = QPushButton("Auto (segment precedent)")
        self.cubic_auto_end_btn = QPushButton("Auto (segment suivant)")
        self.cubic_auto_update_adjacent_checkbox = QCheckBox(
            "Mettre a jour automatiquement les tangentes des segments adjacents"
        )
        self.cubic_auto_update_adjacent_checkbox.setChecked(True)
        self.cubic_hint_label = QLabel(
            "Direction : le triplet X/Y/Z est normalise automatiquement. "
            "Amplitude : longueur reelle de tangente en mm (min 0, pas de maximum)."
        )

        self._context_keypoints: list[TrajectoryKeypoint] = []
        self._context_edited_row_index: int | None = None
        self._context_trajectory_result: TrajectoryResult | None = None
        self._initial_keypoint: TrajectoryKeypoint | None = None

        self.config_policy_combo = QComboBox()
        self.forced_config_combo = QComboBox()
        self.config_hint_label = QLabel("En mode articulaire, la configuration est deduite automatiquement depuis J1..J6.")
        self.current_segment_status_label = QLabel("")
        self.trajectory_status_label = QLabel("")

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
        self.mode_combo.addItem(KeypointMotionMode.PTP.value, KeypointMotionMode.PTP.value)
        self.mode_combo.addItem(KeypointMotionMode.LINEAR.value, KeypointMotionMode.LINEAR.value)
        self.mode_combo.addItem("BEZIER", KeypointMotionMode.CUBIC.value)
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
        target_actions_layout.addWidget(self.use_initial_target_btn)
        target_actions_layout.addStretch()
        target_tab_layout.addLayout(target_actions_layout)
        target_tab_layout.addWidget(self.target_stack)

        cubic_tab_layout = QVBoxLayout(self.cubic_tab)

        linear_layout = QGridLayout()
        linear_label_width = 140

        linear_start_amp_label = QLabel("Amplitude debut (%)")
        linear_start_amp_label.setMinimumWidth(linear_label_width)
        linear_layout.addWidget(linear_start_amp_label, 0, 0)
        self.linear_tangent_ratio_1.setRange(0.0, 1000.0)
        self.linear_tangent_ratio_1.setDecimals(3)
        self.linear_tangent_ratio_1.setSingleStep(1.0)
        linear_layout.addWidget(self.linear_tangent_ratio_1, 0, 1)

        linear_end_amp_label = QLabel("Amplitude fin (%)")
        linear_end_amp_label.setMinimumWidth(linear_label_width)
        linear_layout.addWidget(linear_end_amp_label, 1, 0)
        self.linear_tangent_ratio_2.setRange(0.0, 1000.0)
        self.linear_tangent_ratio_2.setDecimals(3)
        self.linear_tangent_ratio_2.setSingleStep(1.0)
        linear_layout.addWidget(self.linear_tangent_ratio_2, 1, 1)

        linear_layout.addWidget(self.linear_tangent_link_checkbox, 2, 1)
        linear_layout.setColumnStretch(3, 1)
        self.linear_group.setLayout(linear_layout)

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

        start_amp_label = QLabel("Amplitude (mm)")
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

        end_amp_label = QLabel("Amplitude (mm)")
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
        cubic_tab_layout.addWidget(self.linear_group)
        cubic_tab_layout.addWidget(self.cubic_group)
        cubic_tab_layout.addStretch()

        config_group = QGroupBox("Configurations")
        config_layout = QVBoxLayout()

        config_grid = QGridLayout()
        config_grid.addWidget(QLabel("Politique"), 0, 0)
        self.config_policy_combo.addItem("Auto", ConfigurationPolicy.AUTO.value)
        self.config_policy_combo.addItem("Current branch", ConfigurationPolicy.CURRENT_BRANCH.value)
        self.config_policy_combo.addItem("Forced", ConfigurationPolicy.FORCED.value)
        config_grid.addWidget(self.config_policy_combo, 0, 1)

        config_grid.addWidget(QLabel("Config forcee"), 1, 0)
        for key in self.CONFIG_ORDER:
            self.forced_config_combo.addItem(key.name, key.name)
        config_grid.addWidget(self.forced_config_combo, 1, 1)
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

        self.current_segment_status_label.setWordWrap(True)
        self.current_segment_status_label.setStyleSheet("color: #d9534f; font-weight: bold;")
        self.current_segment_status_label.hide()
        layout.addWidget(self.current_segment_status_label)

        self.trajectory_status_label.setWordWrap(True)
        self.trajectory_status_label.setStyleSheet("color: #d9534f; font-weight: bold;")
        self.trajectory_status_label.hide()
        layout.addWidget(self.trajectory_status_label)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _build_cartesian_target_group(self) -> QGroupBox:
        group = QGroupBox("Target cartesienne (X Y Z A B C)")
        layout = QVBoxLayout()

        # Keep historical wide limits from previous dialog spinboxes.
        layout.addWidget(self.cartesian_target_widget)

        tables_layout = QHBoxLayout()
        self._configure_cartesian_solutions_table(self.cartesian_solutions_table_left)
        self._configure_cartesian_solutions_table(self.cartesian_solutions_table_right)
        tables_layout.addWidget(self.cartesian_solutions_table_left)
        tables_layout.addWidget(self.cartesian_solutions_table_right)
        layout.addLayout(tables_layout)

        self.cartesian_error_label.setStyleSheet("color: #d9534f;")
        self.cartesian_error_label.setWordWrap(True)
        self.cartesian_error_label.setMinimumHeight(self.cartesian_error_label.fontMetrics().lineSpacing() * 2 + 4)
        self.cartesian_error_label.setText("")
        layout.addWidget(self.cartesian_error_label)

        group.setLayout(layout)
        return group

    @staticmethod
    def _configure_cartesian_solutions_table(table: QTableWidget) -> None:
        table.setColumnCount(3)
        table.setHorizontalHeaderLabels(["Cfg", "Statut", ""])
        table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        table.verticalHeader().setVisible(False)
        table.setMinimumHeight(145)
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)

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
        self.use_initial_target_btn.clicked.connect(self._on_use_initial_target_clicked)
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        self.speed_spin.valueChanged.connect(self._on_speed_changed)
        self.config_policy_combo.currentIndexChanged.connect(self._on_configuration_policy_changed)
        self.forced_config_combo.currentIndexChanged.connect(self._on_configuration_policy_changed)
        for spin in self.cubic_vector_1:
            spin.valueChanged.connect(self._on_cubic_vectors_changed)
        for spin in self.cubic_vector_2:
            spin.valueChanged.connect(self._on_cubic_vectors_changed)
        self.cubic_amplitude_1.valueChanged.connect(self._on_cubic_vectors_changed)
        self.cubic_amplitude_2.valueChanged.connect(self._on_cubic_vectors_changed)
        self.linear_tangent_ratio_1.valueChanged.connect(self._on_linear_tangent_start_ratio_changed)
        self.linear_tangent_ratio_2.valueChanged.connect(self._on_linear_tangent_end_ratio_changed)
        self.linear_tangent_link_checkbox.toggled.connect(self._on_linear_tangent_link_toggled)
        self.cubic_auto_start_btn.clicked.connect(self._on_auto_start_tangent_clicked)
        self.cubic_auto_end_btn.clicked.connect(self._on_auto_end_tangent_clicked)
        self.joint_target_widget.joint_value_changed.connect(self._on_joint_target_changed)
        self.cartesian_target_widget.cartesian_value_changed.connect(self._on_cartesian_target_changed)
        self.cartesian_target_widget.reference_frame_changed.connect(self._on_cartesian_reference_frame_changed)

    def _robot_base_pose_world(self):
        return self.workspace_model.get_robot_base_transform_world()

    def _get_cartesian_target_in_base_frame(self) -> list[float]:
        return convert_pose_to_base_frame(
            self.cartesian_target_widget.get_cartesian_values(),
            self.cartesian_target_widget.get_reference_frame(),
            self._robot_base_pose_world(),
        )

    def _set_cartesian_target_from_base_pose(self, pose_base: list[float]) -> None:
        self.cartesian_target_widget.set_all_cartesian(
            convert_pose_from_base_frame(
                pose_base,
                self.cartesian_target_widget.get_reference_frame(),
                self._robot_base_pose_world(),
            )
        )

    def _on_use_current_target_clicked(self) -> None:
        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            self._set_cartesian_target_from_base_pose(list(self.robot_model.get_tcp_pose()))
            self._emit_ghost_update()
            return

        self.joint_target_widget.set_all_joints(list(self.robot_model.get_joints()))
        self._emit_ghost_update()

    def _on_use_home_target_clicked(self) -> None:
        home_joints = list(self.robot_model.get_home_position())
        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            fk_result = self.robot_model.compute_fk_joints(home_joints, tool=self.tool_model.get_tool())
            if fk_result is not None:
                _, _, home_pose, _, _ = fk_result
                self._set_cartesian_target_from_base_pose([float(v) for v in home_pose[:6]])
            else:
                self._set_cartesian_target_from_base_pose(list(self.robot_model.get_tcp_pose()))
            self._emit_ghost_update()
            return

        self.joint_target_widget.set_all_joints(home_joints)
        self._emit_ghost_update()

    def _on_use_initial_target_clicked(self) -> None:
        if self._initial_keypoint is None:
            return

        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            self.cartesian_target_widget.set_reference_frame(self._initial_keypoint.cartesian_frame.value)
            self.cartesian_target_widget.set_all_cartesian(list(self._initial_keypoint.cartesian_target))
            self._emit_ghost_update()
            return

        self.joint_target_widget.set_all_joints(list(self._initial_keypoint.joint_target))
        self._emit_ghost_update()

    def _compute_joint_values_from_cartesian_target(self) -> list[float]:
        target = self._get_cartesian_target_in_base_frame()
        mgi_result = self.robot_model.compute_ik_target(target, tool=self.tool_model.get_tool())
        has_valid_raw = bool(mgi_result.get_valid_solutions())
        has_valid_expanded = bool(mgi_result.get_valid_solutions_expanded())
        if not has_valid_raw and not has_valid_expanded:
            self._set_cartesian_error("Aucune solution valide pour la target cartesienne.")
            return []

        selected_allowed = self._resolve_allowed_configs_for_preview()
        if not selected_allowed:
            self._set_cartesian_error("Aucune configuration autorisee avec la politique selectionnee.")
            return []

        reference_joints = [float(v) for v in self.joint_target_widget.get_all_joints()[:6]]
        while len(reference_joints) < 6:
            reference_joints.append(0.0)
        reference_joints_rad = [math.radians(v) for v in reference_joints]

        weights = [float(v) for v in self.robot_model.get_joint_weights()[:6]]
        while len(weights) < 6:
            weights.append(1.0)

        best_solution = mgi_result.get_best_solution_from_current(
            reference_joints_rad,
            weights,
            allowed_configs=selected_allowed,
        )
        if best_solution is None:
            self._set_cartesian_error("Aucune solution valide dans les configurations autorisees.")
            return []
        _, selected_item = best_solution
        joints = [float(v) for v in selected_item.joints[:6]]
        while len(joints) < 6:
            joints.append(0.0)
        self._set_cartesian_error("")
        return joints

    def _sync_joint_target_from_cartesian_target(self) -> None:
        joints = self._compute_joint_values_from_cartesian_target()
        if len(joints) >= 6:
            self.joint_target_widget.set_all_joints(joints)

    def _sync_cartesian_target_from_joint_target(self) -> None:
        fk_result = self.robot_model.compute_fk_joints(
            self.joint_target_widget.get_all_joints(),
            tool=self.tool_model.get_tool(),
        )
        if fk_result is None:
            return
        _, _, pose, _, _ = fk_result
        if len(pose) >= 6:
            self._set_cartesian_target_from_base_pose([float(v) for v in pose[:6]])
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

        fk_result = self.robot_model.compute_fk_joints(joints, tool=self.tool_model.get_tool())
        if fk_result is None:
            return payload
        _, corrected_matrices, pose, _, _ = fk_result
        if len(pose) >= 6:
            self._set_cartesian_target_from_base_pose([float(v) for v in pose[:6]])
            self._set_cartesian_error("")
        payload["corrected_matrices"] = corrected_matrices
        return payload

    def _set_cartesian_error(self, message: str) -> None:
        self.cartesian_error_label.setText(message)

    def _current_target_type(self) -> KeypointTargetType:
        return KeypointTargetType(self.target_type_combo.currentData())

    def _current_mode(self) -> KeypointMotionMode:
        raw = self.mode_combo.currentData()
        if raw is None:
            raw = self.mode_combo.currentText()
        return KeypointMotionMode(str(raw))

    def _current_configuration_policy(self) -> ConfigurationPolicy:
        raw = str(self.config_policy_combo.currentData())
        try:
            return ConfigurationPolicy(raw)
        except ValueError:
            return ConfigurationPolicy.AUTO

    def _selected_forced_config(self) -> MgiConfigKey | None:
        raw = self.forced_config_combo.currentData()
        if not isinstance(raw, str):
            return None
        try:
            return MgiConfigKey[raw]
        except KeyError:
            return None

    def _resolve_allowed_configs_for_preview(self) -> set[MgiConfigKey]:
        robot_allowed = set(self.robot_model.get_allowed_configurations())
        if not robot_allowed:
            return set()

        if self._current_target_type() == KeypointTargetType.JOINT:
            joint_target = self.joint_target_widget.get_all_joints()
            deduced = TrajectoryKeypoint.identify_config_from_joint_target(
                joint_target,
                self.robot_model.get_config_identifier(),
            )
            return ({deduced} & robot_allowed)

        policy = self._current_configuration_policy()
        if policy == ConfigurationPolicy.AUTO:
            return robot_allowed

        if policy == ConfigurationPolicy.CURRENT_BRANCH:
            reference_joints = [float(v) for v in self.joint_target_widget.get_all_joints()[:6]]
            while len(reference_joints) < 6:
                reference_joints.append(0.0)
            config_identifier = self.robot_model.get_config_identifier()
            current_branch = MgiConfigKey.identify_configuration_deg(reference_joints, config_identifier)
            return ({current_branch} & robot_allowed)

        forced = self._selected_forced_config()
        if forced is None:
            return set()
        return ({forced} & robot_allowed)

    @staticmethod
    def _solution_status_text(status: MgiResultStatus) -> str:
        return status.name.replace("_", " ").title()

    def _apply_cartesian_solution(self, config_key: MgiConfigKey, joints: list[float]) -> None:
        _ = config_key
        self.joint_target_widget.set_all_joints(joints)
        self._set_cartesian_error("")
        self._refresh_cartesian_solutions_table()
        self._emit_ghost_update()

    def _refresh_cartesian_solutions_table(self) -> None:
        self.cartesian_solutions_table_left.setRowCount(0)
        self.cartesian_solutions_table_right.setRowCount(0)
        if self._current_target_type() != KeypointTargetType.CARTESIAN:
            return

        target = self._get_cartesian_target_in_base_frame()
        if len(target) < 6:
            return

        mgi_result = self.robot_model.compute_ik_target(target, tool=self.tool_model.get_tool())
        allowed_configs = self._resolve_allowed_configs_for_preview()
        reference_joints = [float(v) for v in self.joint_target_widget.get_all_joints()[:6]]
        while len(reference_joints) < 6:
            reference_joints.append(0.0)
        reference_joints_rad = [math.radians(v) for v in reference_joints]
        weights = [float(v) for v in self.robot_model.get_joint_weights()[:6]]
        while len(weights) < 6:
            weights.append(1.0)

        for config_index, config_key in enumerate(self.CONFIG_ORDER):
            solution = mgi_result.get_solution_raw(config_key)
            if solution is None:
                continue

            target_table = (
                self.cartesian_solutions_table_left
                if config_index < 4
                else self.cartesian_solutions_table_right
            )
            row_index = target_table.rowCount()
            target_table.insertRow(row_index)
            target_table.setItem(row_index, 0, QTableWidgetItem(config_key.name))

            expanded_valid_for_config = mgi_result.get_solutions_expanded(config_key, only_valid=True)
            displayed_status = solution.status
            if expanded_valid_for_config:
                displayed_status = MgiResultStatus.VALID
            if displayed_status == MgiResultStatus.VALID and config_key not in allowed_configs:
                displayed_status = MgiResultStatus.FORBIDDEN_CONFIGURATION

            status_item = QTableWidgetItem(TrajectoryKeypointDialog._solution_status_text(displayed_status))
            target_table.setItem(row_index, 1, status_item)

            use_btn = QPushButton("Sélectionner")
            can_use = displayed_status == MgiResultStatus.VALID
            use_btn.setEnabled(can_use)
            if can_use:
                best_for_config = mgi_result.get_best_solution_from_current(
                    reference_joints_rad,
                    weights,
                    allowed_configs={config_key},
                )
                if best_for_config is not None:
                    _, best_item = best_for_config
                    joints = [float(v) for v in best_item.joints[:6]]
                else:
                    joints = [float(v) for v in solution.joints[:6]]
                while len(joints) < 6:
                    joints.append(0.0)
                use_btn.clicked.connect(
                    lambda _, cfg=config_key, j=list(joints): self._apply_cartesian_solution(cfg, j)
                )
            target_table.setCellWidget(row_index, 2, use_btn)

        self.cartesian_solutions_table_left.resizeRowsToContents()
        self.cartesian_solutions_table_right.resizeRowsToContents()

    @staticmethod
    def _set_status_label(label: QLabel, title: str, messages: list[str]) -> None:
        if not messages:
            label.setText("")
            label.hide()
            return
        label.setText(f"{title}: {join_issue_messages(messages)}")
        label.show()

    def _resolve_current_segment_index(self) -> int | None:
        row = self._context_edited_row_index
        if row is None or row < 0:
            return None
        return row

    def _update_context_status_labels(self) -> None:
        trajectory = self._context_trajectory_result
        if trajectory is None:
            TrajectoryKeypointDialog._set_status_label(self.current_segment_status_label, "Segment courant", [])
            TrajectoryKeypointDialog._set_status_label(self.trajectory_status_label, "Trajectoire", [])
            return

        global_messages = build_trajectory_issue_messages(trajectory)
        TrajectoryKeypointDialog._set_status_label(self.trajectory_status_label, "Trajectoire", global_messages)

        segment_index = self._resolve_current_segment_index()
        if segment_index is None or segment_index >= len(trajectory.segments):
            TrajectoryKeypointDialog._set_status_label(self.current_segment_status_label, "Segment courant", [])
            return

        segment_messages = build_segment_issue_messages(trajectory.segments[segment_index], segment_index)
        TrajectoryKeypointDialog._set_status_label(
            self.current_segment_status_label,
            "Segment courant",
            segment_messages,
        )

    def update_trajectory_context(self, trajectory_result: TrajectoryResult | None) -> None:
        self._context_trajectory_result = trajectory_result
        self._update_context_status_labels()

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
        self._update_context_status_labels()

    def should_auto_update_adjacent_cubic_tangents(self) -> bool:
        return self.cubic_auto_update_adjacent_checkbox.isEnabled() and self.cubic_auto_update_adjacent_checkbox.isChecked()

    def _resolve_current_target_xyz(self) -> list[float] | None:
        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            target = self._get_cartesian_target_in_base_frame()
            if len(target) < 3:
                return None
            return [float(target[0]), float(target[1]), float(target[2])]

        fk_result = self.robot_model.compute_fk_joints(
            self.joint_target_widget.get_all_joints(),
            tool=self.tool_model.get_tool(),
        )
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

    def _set_cubic_start_amplitude_mm(self, amplitude_mm: float, emit_preview: bool = True) -> None:
        self.cubic_amplitude_1.blockSignals(True)
        self.cubic_amplitude_1.setValue(max(0.0, float(amplitude_mm)))
        self.cubic_amplitude_1.blockSignals(False)
        if emit_preview:
            self._emit_live_preview()

    def _set_cubic_end_amplitude_mm(self, amplitude_mm: float, emit_preview: bool = True) -> None:
        self.cubic_amplitude_2.blockSignals(True)
        self.cubic_amplitude_2.setValue(max(0.0, float(amplitude_mm)))
        self.cubic_amplitude_2.blockSignals(False)
        if emit_preview:
            self._emit_live_preview()

    @staticmethod
    def _ratio_to_percent(ratio: float) -> float:
        return max(0.0, float(ratio) * 100.0)

    @staticmethod
    def _percent_to_ratio(percent: float) -> float:
        return max(0.0, float(percent) / 100.0)

    def _set_linear_tangent_ratios(
        self,
        start_ratio: float,
        end_ratio: float,
        emit_preview: bool = True,
    ) -> None:
        self.linear_tangent_ratio_1.blockSignals(True)
        self.linear_tangent_ratio_1.setValue(self._ratio_to_percent(start_ratio))
        self.linear_tangent_ratio_1.blockSignals(False)

        self.linear_tangent_ratio_2.blockSignals(True)
        self.linear_tangent_ratio_2.setValue(self._ratio_to_percent(end_ratio))
        self.linear_tangent_ratio_2.blockSignals(False)

        if emit_preview:
            self._emit_live_preview()

    def _sync_linked_linear_ratio_from_start(self) -> None:
        if not self.linear_tangent_link_checkbox.isChecked():
            return
        target_value = float(self.linear_tangent_ratio_1.value())
        if abs(float(self.linear_tangent_ratio_2.value()) - target_value) <= 1e-12:
            return
        self.linear_tangent_ratio_2.blockSignals(True)
        self.linear_tangent_ratio_2.setValue(target_value)
        self.linear_tangent_ratio_2.blockSignals(False)

    def _sync_linked_linear_ratio_from_end(self) -> None:
        if not self.linear_tangent_link_checkbox.isChecked():
            return
        target_value = float(self.linear_tangent_ratio_2.value())
        if abs(float(self.linear_tangent_ratio_1.value()) - target_value) <= 1e-12:
            return
        self.linear_tangent_ratio_1.blockSignals(True)
        self.linear_tangent_ratio_1.setValue(target_value)
        self.linear_tangent_ratio_1.blockSignals(False)

    def _compute_end_tangent_from_first_tangent_look_at(
        self,
        start_direction_xyz: list[float],
    ) -> list[float] | None:
        row = self._context_edited_row_index
        if row is None or row < 1 or (row - 1) >= len(self._context_keypoints):
            return None

        previous_point_xyz = resolve_keypoint_xyz(
            self.robot_model,
            self._context_keypoints[row - 1],
            tool=self.tool_model.get_tool(),
            robot_base_pose_world=self._robot_base_pose_world(),
        )
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
        next_point_xyz = resolve_keypoint_xyz(
            self.robot_model,
            self._context_keypoints[row + 1],
            tool=self.tool_model.get_tool(),
            robot_base_pose_world=self._robot_base_pose_world(),
        )
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
        start_amplitude_mm = math_utils.vector_norm3(direction)
        start_direction = [-float(d) for d in direction[:3]]
        self._apply_cubic_start_tangent_direction(start_direction, emit_preview=False)
        self._set_cubic_start_amplitude_mm(start_amplitude_mm, emit_preview=False)

        current_end_direction = [float(spin.value()) for spin in self.cubic_vector_2]
        if math_utils.is_near_zero_vector_xyz(current_end_direction):
            end_direction = self._compute_end_tangent_from_first_tangent_look_at(start_direction)
            if end_direction is not None:
                self._apply_cubic_end_tangent_direction(end_direction, emit_preview=False)
                self._set_cubic_end_amplitude_mm(math_utils.vector_norm3(end_direction), emit_preview=False)
        self._emit_live_preview()

    def _on_auto_end_tangent_clicked(self) -> None:
        direction = self._get_next_segment_start_tangent_for_auto()
        if direction is None:
            return
        end_amplitude_mm = math_utils.vector_norm3(direction)
        end_direction = [-float(d) for d in direction[:3]]
        self._apply_cubic_end_tangent_direction(end_direction, emit_preview=False)
        self._set_cubic_end_amplitude_mm(end_amplitude_mm, emit_preview=False)

        current_start_direction = [float(spin.value()) for spin in self.cubic_vector_1]
        if math_utils.is_near_zero_vector_xyz(current_start_direction):
            start_direction = self._compute_start_tangent_from_end_tangent_look_at(end_direction)
            if start_direction is not None:
                self._apply_cubic_start_tangent_direction(start_direction, emit_preview=False)
                self._set_cubic_start_amplitude_mm(math_utils.vector_norm3(start_direction), emit_preview=False)
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
        self._refresh_cartesian_solutions_table()
        self._try_minimize_window_size()
        self._emit_ghost_update()

    def _on_joint_target_changed(self, *_args) -> None:
        if self._current_target_type() == KeypointTargetType.JOINT:
            self._sync_joint_mode_policy()
        self._emit_ghost_update()

    def _on_cartesian_target_changed(self, *_args) -> None:
        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            self._refresh_cartesian_solutions_table()
            self._emit_ghost_update()

    def _on_cartesian_reference_frame_changed(self, _frame: str) -> None:
        if self._current_target_type() == KeypointTargetType.CARTESIAN:
            self._refresh_cartesian_solutions_table()
            self._emit_ghost_update()

    def _on_configuration_policy_changed(self, _idx: int) -> None:
        self._update_config_policy_editor_state()
        if self._current_target_type() != KeypointTargetType.CARTESIAN:
            return

        self._refresh_cartesian_solutions_table()
        self._emit_ghost_update()

    def _on_speed_changed(self, *_args) -> None:
        self._emit_live_preview()

    def _on_cubic_vectors_changed(self, *_args) -> None:
        self._emit_live_preview()

    def _on_linear_tangent_start_ratio_changed(self, *_args) -> None:
        self._sync_linked_linear_ratio_from_start()
        self._emit_live_preview()

    def _on_linear_tangent_end_ratio_changed(self, *_args) -> None:
        self._sync_linked_linear_ratio_from_end()
        self._emit_live_preview()

    def _on_linear_tangent_link_toggled(self, linked: bool) -> None:
        if linked:
            self._sync_linked_linear_ratio_from_start()
        self._emit_live_preview()

    def _sync_joint_mode_policy(self) -> None:
        joint_target = self.joint_target_widget.get_all_joints()
        deduced = TrajectoryKeypoint.identify_config_from_joint_target(
            joint_target,
            self.robot_model.get_config_identifier(),
        )
        forced_idx = self.forced_config_combo.findData(deduced.name)
        if forced_idx >= 0:
            self.forced_config_combo.blockSignals(True)
            self.forced_config_combo.setCurrentIndex(forced_idx)
            self.forced_config_combo.blockSignals(False)
        policy_idx = self.config_policy_combo.findData(ConfigurationPolicy.FORCED.value)
        if policy_idx >= 0:
            self.config_policy_combo.blockSignals(True)
            self.config_policy_combo.setCurrentIndex(policy_idx)
            self.config_policy_combo.blockSignals(False)

    def _update_config_policy_editor_state(self) -> None:
        is_joint = self._current_target_type() == KeypointTargetType.JOINT
        policy = self._current_configuration_policy()

        self.config_policy_combo.setEnabled(not is_joint)
        self.forced_config_combo.setEnabled((not is_joint) and policy == ConfigurationPolicy.FORCED)

    def _update_target_editors(self) -> None:
        is_joint = self._current_target_type() == KeypointTargetType.JOINT
        self.target_stack.setCurrentIndex(1 if is_joint else 0)

        self.config_hint_label.setText(
            "En mode articulaire, la configuration est deduite automatiquement depuis J1..J6."
            if is_joint
            else "Auto: pas de contrainte locale. Current branch: branche issue du sample precedent. Forced: configuration explicite."
        )
        self._update_config_policy_editor_state()

        if is_joint:
            self._set_cartesian_error("")
            self._sync_joint_mode_policy()
            self.cartesian_solutions_table_left.setRowCount(0)
            self.cartesian_solutions_table_right.setRowCount(0)
        else:
            self._refresh_cartesian_solutions_table()
        self._last_target_type = KeypointTargetType.JOINT if is_joint else KeypointTargetType.CARTESIAN

    def _update_cubic_visibility(self) -> None:
        mode = self._current_mode()
        is_cubic = mode == KeypointMotionMode.CUBIC
        is_linear = mode == KeypointMotionMode.LINEAR
        show_interp_tab = is_cubic or is_linear
        cubic_tab_index = self.main_tabs.indexOf(self.cubic_tab)
        if cubic_tab_index >= 0:
            self.main_tabs.setTabEnabled(cubic_tab_index, show_interp_tab)
            if hasattr(self.main_tabs, "setTabVisible"):
                self.main_tabs.setTabVisible(cubic_tab_index, show_interp_tab)
            if show_interp_tab:
                self.main_tabs.setCurrentWidget(self.cubic_tab)
            elif self.main_tabs.currentIndex() == cubic_tab_index:
                self.main_tabs.setCurrentWidget(self.target_tab)
        self.cubic_group.setVisible(is_cubic)
        self.linear_group.setVisible(is_linear)
        self.cubic_auto_update_adjacent_checkbox.setVisible(is_cubic)
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
        self._initial_keypoint = keypoint.clone()
        target_type_idx = self.target_type_combo.findData(keypoint.target_type.value)
        if target_type_idx >= 0:
            self.target_type_combo.blockSignals(True)
            self.target_type_combo.setCurrentIndex(target_type_idx)
            self.target_type_combo.blockSignals(False)

        self.cartesian_target_widget.set_reference_frame(keypoint.cartesian_frame.value)
        self.cartesian_target_widget.set_all_cartesian(keypoint.cartesian_target)
        self.joint_target_widget.set_all_joints(keypoint.joint_target)

        self._ptp_speed_percent = keypoint.ptp_speed_percent
        self._linear_speed_mps = keypoint.linear_speed_mps

        self.mode_combo.blockSignals(True)
        mode_idx = self.mode_combo.findData(keypoint.mode.value)
        if mode_idx >= 0:
            self.mode_combo.setCurrentIndex(mode_idx)
        self.mode_combo.blockSignals(False)
        self._last_mode = keypoint.mode

        for i, spin in enumerate(self.cubic_vector_1):
            spin.setValue(keypoint.cubic_vectors[0][i])
        for i, spin in enumerate(self.cubic_vector_2):
            spin.setValue(keypoint.cubic_vectors[1][i])
        self.cubic_amplitude_1.setValue(float(keypoint.cubic_amplitudes_mm[0]))
        self.cubic_amplitude_2.setValue(float(keypoint.cubic_amplitudes_mm[1]))
        self._set_linear_tangent_ratios(
            float(keypoint.linear_tangent_ratios[0]),
            float(keypoint.linear_tangent_ratios[1]),
            emit_preview=False,
        )
        self.linear_tangent_link_checkbox.blockSignals(True)
        self.linear_tangent_link_checkbox.setChecked(bool(keypoint.linear_tangent_ratios_linked))
        self.linear_tangent_link_checkbox.blockSignals(False)

        policy_idx = self.config_policy_combo.findData(keypoint.configuration_policy.value)
        if policy_idx >= 0:
            self.config_policy_combo.blockSignals(True)
            self.config_policy_combo.setCurrentIndex(policy_idx)
            self.config_policy_combo.blockSignals(False)

        forced_name = keypoint.forced_config.name if keypoint.forced_config is not None else MgiConfigKey.FUN.name
        forced_idx = self.forced_config_combo.findData(forced_name)
        if forced_idx >= 0:
            self.forced_config_combo.blockSignals(True)
            self.forced_config_combo.setCurrentIndex(forced_idx)
            self.forced_config_combo.blockSignals(False)

        self._update_target_editors()
        self._update_cubic_visibility()
        self._update_speed_editor()
        self._refresh_cartesian_solutions_table()
        self._try_minimize_window_size()
        self._suspend_preview_emission = False
        self._emit_ghost_update()
        self._update_context_status_labels()

    def get_keypoint(self) -> TrajectoryKeypoint:
        self._store_current_speed()
        target_type = self._current_target_type()
        mode = self._current_mode()
        configuration_policy = self._current_configuration_policy()
        forced_config = self._selected_forced_config()

        if target_type == KeypointTargetType.JOINT:
            deduced = TrajectoryKeypoint.identify_config_from_joint_target(
                self.joint_target_widget.get_all_joints(),
                self.robot_model.get_config_identifier(),
            )
            configuration_policy = ConfigurationPolicy.FORCED
            forced_config = deduced
        elif configuration_policy != ConfigurationPolicy.FORCED:
            forced_config = None

        return TrajectoryKeypoint(
            target_type=target_type,
            cartesian_target=self.cartesian_target_widget.get_cartesian_values(),
            cartesian_frame=ReferenceFrame.from_value(self.cartesian_target_widget.get_reference_frame()),
            joint_target=self.joint_target_widget.get_all_joints(),
            mode=mode,
            cubic_vectors=[
                [spin.value() for spin in self.cubic_vector_1],
                [spin.value() for spin in self.cubic_vector_2],
            ],
            cubic_amplitudes_mm=[
                self.cubic_amplitude_1.value(),
                self.cubic_amplitude_2.value(),
            ],
            linear_tangent_ratios=[
                self._percent_to_ratio(self.linear_tangent_ratio_1.value()),
                self._percent_to_ratio(self.linear_tangent_ratio_2.value()),
            ],
            linear_tangent_ratios_linked=self.linear_tangent_link_checkbox.isChecked(),
            configuration_policy=configuration_policy,
            forced_config=forced_config,
            ptp_speed_percent=self._ptp_speed_percent,
            linear_speed_mps=self._linear_speed_mps,
        )
