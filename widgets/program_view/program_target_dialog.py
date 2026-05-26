from __future__ import annotations

import numpy as np

import utils.math_utils as math_utils
from PyQt6.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from models.reference_frame import ReferenceFrame
from models.robot_model import RobotModel
from models.robot_program import RobotProgramMotion, RobotProgramMotionMode, RobotProgramTarget, RobotProgramTargetType
from models.types import JointAngles6, Pose6
from utils.reference_frame_utils import matrix_to_pose, pose_to_matrix
from widgets.cartesian_control_view.cartesian_control_widget import CartesianControlWidget
from widgets.joint_control_view.joints_control_widget import JointsControlWidget


class ProgramTargetDialog(QDialog):
    def __init__(
        self,
        robot_model: RobotModel,
        motion: RobotProgramMotion,
        target: RobotProgramTarget,
        is_via_target: bool,
        allow_motion_type_editing: bool = False,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Parametrage du point de programme")
        self.setMinimumWidth(620)
        self._robot_model = robot_model
        self._motion = motion
        self._target = target
        self._is_via_target = bool(is_via_target)
        self._allow_motion_type_editing = bool(allow_motion_type_editing)
        self._current_frame = ReferenceFrame.PROGRAM

        self.target_type_label = QLabel()
        self.target_type_combo = QComboBox()
        self.motion_mode_label = QLabel()
        self.motion_mode_combo = QComboBox()
        self.editor_mode_label = QLabel()
        self.frame_row_widget = QWidget()
        self.frame_combo = QComboBox()
        self.cartesian_widget = CartesianControlWidget(compact=True)
        self.joint_widget = JointsControlWidget(compact=True)
        self.apply_current_position_button = QPushButton("Appliquer la position courante")
        self.target_stack = QStackedWidget()
        self.button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)

        self.cartesian_widget.set_spinbox_keyboard_tracking(False)
        self.joint_widget.set_spinbox_keyboard_tracking(False)
        self.joint_widget.update_axis_limits(self._robot_model.get_axis_limits())
        self._setup_ui()
        self._setup_connections()
        self._load_target()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        info_group = QGroupBox("Parametrage")
        info_layout = QFormLayout(info_group)
        if self._allow_motion_type_editing:
            self.motion_mode_combo.addItem("PTP", RobotProgramMotionMode.PTP.value)
            self.motion_mode_combo.addItem("LIN", RobotProgramMotionMode.LINEAR.value)
            self.target_type_combo.addItem("CARTESIAN", RobotProgramTargetType.CARTESIAN.value)
            self.target_type_combo.addItem("JOINT", RobotProgramTargetType.JOINT.value)
            info_layout.addRow("Mouvement", self.motion_mode_combo)
            info_layout.addRow("Type cible", self.target_type_combo)
        else:
            info_layout.addRow("Type cible", self.target_type_label)
            info_layout.addRow("Mouvement", self.motion_mode_label)
        layout.addWidget(info_group)

        frame_row = QHBoxLayout(self.frame_row_widget)
        frame_row.setContentsMargins(0, 0, 0, 0)
        frame_row.addWidget(self.editor_mode_label)
        self.frame_combo.addItem("Base programme", ReferenceFrame.PROGRAM.value)
        self.frame_combo.addItem("Base robot", ReferenceFrame.ROBOT.value)
        frame_row.addWidget(self.frame_combo)
        frame_row.addStretch()
        layout.addWidget(self.frame_row_widget)

        self.target_stack.addWidget(self.cartesian_widget)
        self.target_stack.addWidget(self.joint_widget)
        layout.addWidget(self.target_stack)
        actions_layout = QHBoxLayout()
        actions_layout.setContentsMargins(0, 0, 0, 0)
        actions_layout.addWidget(self.apply_current_position_button)
        actions_layout.addStretch()
        layout.addLayout(actions_layout)
        layout.addWidget(self.button_box)

    def _setup_connections(self) -> None:
        ok_button = self.button_box.button(QDialogButtonBox.StandardButton.Ok)
        if ok_button is not None:
            ok_button.setAutoDefault(False)
            ok_button.setDefault(False)
        cancel_button = self.button_box.button(QDialogButtonBox.StandardButton.Cancel)
        if cancel_button is not None:
            cancel_button.setAutoDefault(False)
            cancel_button.setDefault(False)
        self.motion_mode_combo.currentIndexChanged.connect(self._on_motion_mode_changed)
        self.target_type_combo.currentIndexChanged.connect(self._on_target_type_changed)
        self.frame_combo.currentIndexChanged.connect(self._on_frame_changed)
        self.apply_current_position_button.clicked.connect(self._on_apply_current_position_clicked)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)

    def _load_target(self) -> None:
        target_kind = "Intermediaire CIRC" if self._is_via_target else "Finale"
        if self._allow_motion_type_editing:
            index = self.motion_mode_combo.findData(self._motion.mode.value)
            if index >= 0:
                self.motion_mode_combo.blockSignals(True)
                self.motion_mode_combo.setCurrentIndex(index)
                self.motion_mode_combo.blockSignals(False)
            target_index = self.target_type_combo.findData(self._target.target_type.value)
            if target_index >= 0:
                self.target_type_combo.blockSignals(True)
                self.target_type_combo.setCurrentIndex(target_index)
                self.target_type_combo.blockSignals(False)
            self._sync_target_type_options_with_motion()
        else:
            self.motion_mode_label.setText("PTP" if self._motion.mode == RobotProgramMotionMode.PTP else self._motion.mode.value)
        self._refresh_target_editor(target_kind)

    def _refresh_target_editor(self, target_kind: str) -> None:
        self.target_type_label.setText(f"{target_kind} / {self._target.target_type.value}")
        is_joint_target = self._target.target_type == RobotProgramTargetType.JOINT
        self.editor_mode_label.setText("Edition articulaire" if is_joint_target else "Edition cartesienne")
        self.frame_row_widget.setVisible(not is_joint_target)
        if is_joint_target:
            self.target_stack.setCurrentWidget(self.joint_widget)
            self.joint_widget.set_all_joints(self._target.joint_angles.to_list())
            return

        self.target_stack.setCurrentWidget(self.cartesian_widget)
        self._set_cartesian_editor_pose(self._target.cartesian_pose.copy(), ReferenceFrame.PROGRAM)

    def _on_motion_mode_changed(self, _index: int) -> None:
        if not self._allow_motion_type_editing:
            return
        self._sync_target_type_options_with_motion()
        self._target = self._build_target_for_type(self.get_target_type())
        self._refresh_target_editor("Finale")

    def _on_target_type_changed(self, _index: int) -> None:
        if not self._allow_motion_type_editing:
            return
        self._target = self._build_target_for_type(self.get_target_type())
        self._refresh_target_editor("Finale")

    def _sync_target_type_options_with_motion(self) -> None:
        if not self._allow_motion_type_editing:
            return
        index_joint = self.target_type_combo.findData(RobotProgramTargetType.JOINT.value)
        index_cartesian = self.target_type_combo.findData(RobotProgramTargetType.CARTESIAN.value)
        if index_joint >= 0:
            self.target_type_combo.model().item(index_joint).setEnabled(True)
        if index_cartesian >= 0:
            self.target_type_combo.model().item(index_cartesian).setEnabled(True)

    def _build_target_for_type(self, target_type: RobotProgramTargetType) -> RobotProgramTarget:
        if target_type == RobotProgramTargetType.JOINT:
            return RobotProgramTarget(
                target_type=RobotProgramTargetType.JOINT,
                joint_angles=JointAngles6.from_values(self._robot_model.get_joints()),
            )
        return RobotProgramTarget(
            target_type=RobotProgramTargetType.CARTESIAN,
            cartesian_pose=self._robot_base_pose_to_program_pose(self._robot_model.get_tcp_pose()),
        )

    def _on_frame_changed(self, _index: int) -> None:
        if self._target.target_type != RobotProgramTargetType.CARTESIAN:
            return
        next_frame = ReferenceFrame.from_value(self.frame_combo.currentData(), ReferenceFrame.PROGRAM)
        if next_frame == self._current_frame:
            return
        current_pose = self.cartesian_widget.get_cartesian_values()
        pose_program = (
            current_pose.copy()
            if self._current_frame == ReferenceFrame.PROGRAM
            else self._robot_base_pose_to_program_pose(current_pose)
        )
        next_pose = (
            pose_program.copy()
            if next_frame == ReferenceFrame.PROGRAM
            else self._program_pose_to_robot_base_pose(pose_program)
        )
        self._set_cartesian_editor_pose(next_pose, next_frame)

    def _on_apply_current_position_clicked(self) -> None:
        if self._target.target_type == RobotProgramTargetType.JOINT:
            current_joint_angles = JointAngles6.from_values(self._robot_model.get_joints())
            self.joint_widget.set_all_joints(current_joint_angles.to_list())
        else:
            current_pose_program = self._robot_base_pose_to_program_pose(self._robot_model.get_tcp_pose())
            pose_to_apply = (
                current_pose_program
                if self._current_frame == ReferenceFrame.PROGRAM
                else self._program_pose_to_robot_base_pose(current_pose_program)
            )
            self._set_cartesian_editor_pose(pose_to_apply, self._current_frame)

    def get_target(self) -> RobotProgramTarget:
        if self._target.target_type == RobotProgramTargetType.JOINT:
            return RobotProgramTarget(
                target_type=RobotProgramTargetType.JOINT,
                joint_angles=JointAngles6.from_values(self.joint_widget.get_all_joints()),
            )

        pose_value = self.cartesian_widget.get_cartesian_values()
        pose_program = pose_value.copy() if self._current_frame == ReferenceFrame.PROGRAM else self._robot_base_pose_to_program_pose(pose_value)
        return RobotProgramTarget(
            target_type=RobotProgramTargetType.CARTESIAN,
            cartesian_pose=pose_program,
        )

    def get_motion_mode(self) -> RobotProgramMotionMode:
        if self._allow_motion_type_editing:
            return RobotProgramMotionMode(self.motion_mode_combo.currentData())
        return self._motion.mode

    def get_target_type(self) -> RobotProgramTargetType:
        if self._allow_motion_type_editing:
            return RobotProgramTargetType(self.target_type_combo.currentData())
        return self._target.target_type

    def _display_cartesian_slider_limits_xyz(
        self,
        frame: ReferenceFrame | None = None,
    ) -> list[tuple[float, float]]:
        xyz_limits = [
            (float(min_val), float(max_val))
            for min_val, max_val in self._robot_model.get_cartesian_slider_limits_xyz()
        ]
        resolved_frame = self._current_frame if frame is None else frame
        if resolved_frame == ReferenceFrame.PROGRAM:
            program_from_robot_pose = matrix_to_pose(np.linalg.inv(pose_to_matrix(self._motion.base_pose)))
            return math_utils.transform_xyz_limits_yaw_only(xyz_limits, program_from_robot_pose)
        return xyz_limits

    def _apply_cartesian_target_limits(self, frame: ReferenceFrame | None = None) -> None:
        xyz_limits = self._display_cartesian_slider_limits_xyz(frame)
        self.cartesian_widget.update_axis_limits(list(xyz_limits[:3]) + [(-180.0, 180.0)] * 3)

    def _set_cartesian_editor_pose(self, pose: Pose6, frame: ReferenceFrame) -> None:
        self._apply_cartesian_target_limits(frame)
        self._current_frame = frame
        frame_index = self.frame_combo.findData(frame.value)
        if frame_index >= 0:
            self.frame_combo.blockSignals(True)
            self.frame_combo.setCurrentIndex(frame_index)
            self.frame_combo.blockSignals(False)
        self.cartesian_widget.set_reference_frame(ReferenceFrame.ROBOT.value)
        self.cartesian_widget.set_all_cartesian(pose)

    def _program_pose_to_robot_base_pose(self, pose_program: Pose6) -> Pose6:
        return matrix_to_pose(pose_to_matrix(self._motion.base_pose) @ pose_to_matrix(pose_program))

    def _robot_base_pose_to_program_pose(self, pose_robot_base: Pose6) -> Pose6:
        return matrix_to_pose(np.linalg.inv(pose_to_matrix(self._motion.base_pose)) @ pose_to_matrix(pose_robot_base))
