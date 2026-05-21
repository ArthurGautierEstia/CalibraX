from __future__ import annotations

import numpy as np

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
from models.robot_program import RobotProgramMotion, RobotProgramTarget, RobotProgramTargetType
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
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Editer cible programme")
        self.setMinimumWidth(620)
        self._robot_model = robot_model
        self._motion = motion
        self._target = target
        self._is_via_target = bool(is_via_target)
        self._current_frame = ReferenceFrame.PROGRAM

        self.target_type_label = QLabel()
        self.motion_mode_label = QLabel()
        self.line_label = QLabel()
        self.base_label = QLabel()
        self.tool_label = QLabel()
        self.frame_combo = QComboBox()
        self.cartesian_widget = CartesianControlWidget(compact=True)
        self.joint_widget = JointsControlWidget(compact=True)
        self.apply_current_position_button = QPushButton("Appliquer la position courante")
        self.target_stack = QStackedWidget()
        self.button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)

        self.cartesian_widget.set_spinbox_keyboard_tracking(False)
        self.joint_widget.set_spinbox_keyboard_tracking(False)
        self._setup_ui()
        self._setup_connections()
        self._load_target()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        info_group = QGroupBox("Informations programme")
        info_layout = QFormLayout(info_group)
        info_layout.addRow("Type cible", self.target_type_label)
        info_layout.addRow("Mouvement", self.motion_mode_label)
        info_layout.addRow("Ligne", self.line_label)
        info_layout.addRow("Base", self.base_label)
        info_layout.addRow("Tool", self.tool_label)
        layout.addWidget(info_group)

        frame_row = QHBoxLayout()
        frame_row.addWidget(QLabel("Edition articulaire"))
        self.frame_combo.addItem("Base programme", ReferenceFrame.PROGRAM.value)
        self.frame_combo.addItem("Repere robot", ReferenceFrame.ROBOT.value)
        frame_row.addWidget(self.frame_combo)
        frame_row.addStretch()
        layout.addLayout(frame_row)

        self.target_stack.addWidget(self.cartesian_widget)
        self.target_stack.addWidget(self.joint_widget)
        layout.addWidget(self.target_stack)
        joint_actions_layout = QHBoxLayout()
        joint_actions_layout.addWidget(self.apply_current_position_button)
        joint_actions_layout.addStretch()
        layout.addLayout(joint_actions_layout)
        layout.addWidget(self.button_box)

    def _setup_connections(self) -> None:
        self.frame_combo.currentIndexChanged.connect(self._on_frame_changed)
        self.apply_current_position_button.clicked.connect(self._on_apply_current_position_clicked)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)

    def _load_target(self) -> None:
        target_kind = "Intermediaire CIRC" if self._is_via_target else "Finale"
        self.target_type_label.setText(f"{target_kind} / {self._target.target_type.value}")
        self.motion_mode_label.setText(self._motion.mode.value)
        self.line_label.setText(str(self._motion.line_number))
        self.base_label.setText(self._pose_text(self._motion.base_pose))
        self.tool_label.setText(self._pose_text(self._motion.tool_pose))

        is_joint_target = self._target.target_type == RobotProgramTargetType.JOINT
        self.frame_combo.setVisible(not is_joint_target)
        self.apply_current_position_button.setVisible(is_joint_target)
        if is_joint_target:
            self.target_stack.setCurrentWidget(self.joint_widget)
            self.joint_widget.set_all_joints(self._target.joint_angles.to_list())
            return

        self.target_stack.setCurrentWidget(self.cartesian_widget)
        self._current_frame = ReferenceFrame.PROGRAM
        self.frame_combo.blockSignals(True)
        self.frame_combo.setCurrentIndex(0)
        self.frame_combo.blockSignals(False)
        self.cartesian_widget.set_reference_frame(ReferenceFrame.ROBOT.value)
        self.cartesian_widget.set_all_cartesian(self._target.cartesian_pose.copy())

    def _on_frame_changed(self, _index: int) -> None:
        if self._target.target_type != RobotProgramTargetType.CARTESIAN:
            return
        next_frame = ReferenceFrame.from_value(self.frame_combo.currentData(), ReferenceFrame.PROGRAM)
        if next_frame == self._current_frame:
            return
        current_pose = self.cartesian_widget.get_cartesian_values()
        pose_program = current_pose.copy() if self._current_frame == ReferenceFrame.PROGRAM else self._robot_base_pose_to_program_pose(current_pose)
        next_pose = pose_program.copy() if next_frame == ReferenceFrame.PROGRAM else self._program_pose_to_robot_base_pose(pose_program)
        self._current_frame = next_frame
        self.cartesian_widget.set_all_cartesian(next_pose)

    def _on_apply_current_position_clicked(self) -> None:
        current_joint_angles = JointAngles6.from_values(self._robot_model.get_joints())
        self.joint_widget.set_all_joints(current_joint_angles.to_list())

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

    def _program_pose_to_robot_base_pose(self, pose_program: Pose6) -> Pose6:
        return matrix_to_pose(pose_to_matrix(self._motion.base_pose) @ pose_to_matrix(pose_program))

    def _robot_base_pose_to_program_pose(self, pose_robot_base: Pose6) -> Pose6:
        return matrix_to_pose(np.linalg.inv(pose_to_matrix(self._motion.base_pose)) @ pose_to_matrix(pose_robot_base))

    @staticmethod
    def _pose_text(pose: Pose6) -> str:
        return (
            f"X={pose.x:.3f}, Y={pose.y:.3f}, Z={pose.z:.3f}, "
            f"A={pose.a:.3f}, B={pose.b:.3f}, C={pose.c:.3f}"
        )
