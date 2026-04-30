from __future__ import annotations

import json
from pathlib import Path
from typing import Optional
import os

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QDialog,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
    QHeaderView,
    QComboBox
)

import utils.math_utils as math_utils
from utils.trajectory_keypoint_utils import resolve_keypoint_xyz
from models.trajectory_keypoint import (
    ConfigurationPolicy,
    KeypointMotionMode,
    KeypointTargetType,
    TrajectoryKeypoint,
)
from models.trajectory_options import TrajectoryBezierDegree
from models.reference_frame import ReferenceFrame
from models.types import XYZ3
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from models.trajectory_result import TrajectoryResult
from widgets.trajectory_view.trajectory_keypoint_dialog import TrajectoryKeypointDialog
from utils.reference_frame_utils import convert_pose_from_base_frame


class TrajectoryConfigWidget(QWidget):
    """Widget to configure a trajectory with a list of keypoints."""

    add_requested = pyqtSignal()
    edit_requested = pyqtSignal()
    delete_requested = pyqtSignal()
    keypoints_changed = pyqtSignal(list)
    keypointSelectionChanged = pyqtSignal(object)
    editingSessionStarted = pyqtSignal(int)
    editingSessionFinished = pyqtSignal()
    trajectoryPreviewRequested = pyqtSignal(list)
    trajectoryPreviewFinished = pyqtSignal()
    showRobotGhostRequested = pyqtSignal()
    hideRobotGhostRequested = pyqtSignal()
    updateRobotGhostRequested = pyqtSignal(object)
    goToRequested = pyqtSignal(int)
    timeSmoothingChanged = pyqtSignal(bool)
    bezierDegreeChanged = pyqtSignal(str)
    cartesianDisplayFrameChanged = pyqtSignal(str)
    jerkCheckChanged = pyqtSignal(bool)

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        parent: QWidget = None,
    ) -> None:
        super().__init__(parent)
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model

        self.keypoints_table = QTableWidget(0, 11)
        self.btn_add = QPushButton("Ajouter")
        self.btn_edit = QPushButton("Editer")
        self.btn_go_to = QPushButton("Aller à")
        self.btn_delete = QPushButton("Supprimer")
        self.btn_move_up = QPushButton("Monter")
        self.btn_move_down = QPushButton("Descendre")
        self.btn_import = QPushButton("Importer")
        self.btn_export = QPushButton("Exporter")
        self.btn_delete_all = QPushButton("Tout supprimer")
        self.cb_smooth_time = QCheckBox("Lisser le temps")
        self.cb_check_jerk = QCheckBox("Vérif. jerk")
        self.bezier_degree_combo = QComboBox()
        self.cartesian_display_frame_combo = QComboBox()
        self.cb_smooth_time.setChecked(True)
        self.cb_smooth_time.setToolTip(
            "Active : transition quintique. "
            "Désactive : transition linéaire"
        )
        self.cb_check_jerk.setChecked(True)
        self.cb_check_jerk.setToolTip(
            "Active : signale les dépassements de jerk. "
            "Désactivé : conserve les controles vitesse et accéleration."
        )

        self._keypoints: list[TrajectoryKeypoint] = []
        self._active_dialog: TrajectoryKeypointDialog | None = None
        self._active_dialog_mode: str | None = None
        self._active_dialog_row: int | None = None
        self._is_editing_active = False
        self._trajectory_context: TrajectoryResult | None = None

        self._setup_ui()
        self._setup_connections()
        self._update_buttons_state()

    def _setup_ui(self) -> None:
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)

        layout = QVBoxLayout(self)

        title = QLabel("Configuration de la trajectoire")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(title)

        options_row = QHBoxLayout()
        options_row.addWidget(self.cb_smooth_time)
        options_row.addWidget(self.cb_check_jerk)
        options_row.addSpacing(12)
        options_row.addWidget(QLabel("Interpolation"))
        self.bezier_degree_combo.addItem("Bézier 5", TrajectoryBezierDegree.BEZIER5.value)
        self.bezier_degree_combo.addItem("Bézier 3", TrajectoryBezierDegree.BEZIER3.value)
        options_row.addWidget(self.bezier_degree_combo)
        options_row.addSpacing(12)
        options_row.addWidget(QLabel("Repère cartésien"))
        self.cartesian_display_frame_combo.addItem("Robot", ReferenceFrame.BASE.value)
        self.cartesian_display_frame_combo.addItem("World", ReferenceFrame.WORLD.value)
        options_row.addWidget(self.cartesian_display_frame_combo)
        options_row.addStretch()
        layout.addLayout(options_row)

        self.keypoints_table.setHorizontalHeaderLabels([
            "#", "Cible", "Mode", "Vitesse", "J1 / X", "J2 / Y", "J3 / Z", "J4 / A", "J5 / B", "J6 / C", "Configs"
        ])

        header = self.keypoints_table.horizontalHeader()
        header.setMinimumSectionSize(60)

        for col in range(0, 10):
            header.setSectionResizeMode(col, QHeaderView.ResizeMode.ResizeToContents)

        header.setSectionResizeMode(10, QHeaderView.ResizeMode.Stretch)
                
        self.keypoints_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.keypoints_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.keypoints_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.keypoints_table.setMinimumHeight(220)
        layout.addWidget(self.keypoints_table)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self.btn_add)
        btn_row.addWidget(self.btn_edit)
        btn_row.addWidget(self.btn_go_to)
        btn_row.addWidget(self.btn_delete)
        btn_row.addWidget(self.btn_move_up)
        btn_row.addWidget(self.btn_move_down)
        btn_row.addWidget(self.btn_import)
        btn_row.addWidget(self.btn_export)
        btn_row.addWidget(self.btn_delete_all)
        btn_row.addStretch()
        layout.addLayout(btn_row)

    def _setup_connections(self) -> None:
        self.btn_add.clicked.connect(self._on_add_clicked)
        self.btn_edit.clicked.connect(self._on_edit_clicked)
        self.btn_delete.clicked.connect(self._on_delete_clicked)
        self.btn_delete_all.clicked.connect(self._on_delete_all_clicked)
        self.btn_go_to.clicked.connect(self._on_go_to_clicked)
        self.btn_move_up.clicked.connect(self._on_move_up_clicked)
        self.btn_move_down.clicked.connect(self._on_move_down_clicked)
        self.btn_import.clicked.connect(self._on_import_clicked)
        self.btn_export.clicked.connect(self._on_export_clicked)
        self.cb_smooth_time.toggled.connect(self._on_time_smoothing_toggled)
        self.cb_check_jerk.toggled.connect(self._on_jerk_check_toggled)
        self.bezier_degree_combo.currentIndexChanged.connect(self._on_bezier_degree_changed)
        self.cartesian_display_frame_combo.currentIndexChanged.connect(self._on_cartesian_display_frame_changed)
        self.keypoints_table.itemSelectionChanged.connect(self._on_table_selection_changed)

    def _emit_keypoints_changed(self) -> None:
        self.keypoints_changed.emit(self.get_keypoints())

    def _preferred_trajectories_dir(self) -> str:
        current_dir = os.getcwd()
        traj_dir = os.path.join(current_dir, "trajectories")
        return traj_dir if os.path.exists(traj_dir) else current_dir

    def _emit_selection_changed(self) -> None:
        self.keypointSelectionChanged.emit(self._selected_row())

    def _set_editing_active(self, active: bool) -> None:
        self._is_editing_active = active
        self.keypoints_table.setEnabled(not active)
        self.cb_smooth_time.setEnabled(not active)
        self.cb_check_jerk.setEnabled(not active)
        self.bezier_degree_combo.setEnabled(not active)
        self._update_buttons_state()

    def _on_time_smoothing_toggled(self, checked: bool) -> None:
        self.timeSmoothingChanged.emit(bool(checked))

    def _on_jerk_check_toggled(self, checked: bool) -> None:
        self.jerkCheckChanged.emit(bool(checked))

    def _on_bezier_degree_changed(self, _index: int) -> None:
        self.bezierDegreeChanged.emit(self.get_bezier_degree().value)

    def _on_cartesian_display_frame_changed(self, _index: int) -> None:
        self.cartesianDisplayFrameChanged.emit(self.get_cartesian_display_frame())

    def is_time_smoothing_enabled(self) -> bool:
        return self.cb_smooth_time.isChecked()

    def is_jerk_check_enabled(self) -> bool:
        return self.cb_check_jerk.isChecked()

    def get_bezier_degree(self) -> TrajectoryBezierDegree:
        return TrajectoryBezierDegree.from_value(self.bezier_degree_combo.currentData())

    def get_cartesian_display_frame(self) -> str:
        return ReferenceFrame.from_value(self.cartesian_display_frame_combo.currentData()).value

    def set_bezier_degree(self, degree: TrajectoryBezierDegree | str, emit_signal: bool = False) -> None:
        normalized = TrajectoryBezierDegree.from_value(degree)
        index = self.bezier_degree_combo.findData(normalized.value)
        if index < 0:
            return
        self.bezier_degree_combo.blockSignals(True)
        self.bezier_degree_combo.setCurrentIndex(index)
        self.bezier_degree_combo.blockSignals(False)
        if emit_signal:
            self.bezierDegreeChanged.emit(normalized.value)

    def set_cartesian_display_frame(self, display_frame: str, emit_signal: bool = False) -> None:
        normalized = ReferenceFrame.from_value(display_frame)
        index = self.cartesian_display_frame_combo.findData(normalized.value)
        if index < 0:
            return
        self.cartesian_display_frame_combo.blockSignals(True)
        self.cartesian_display_frame_combo.setCurrentIndex(index)
        self.cartesian_display_frame_combo.blockSignals(False)
        if emit_signal:
            self.cartesianDisplayFrameChanged.emit(normalized.value)

    def set_time_smoothing_enabled(self, enabled: bool, emit_signal: bool = False) -> None:
        self.cb_smooth_time.blockSignals(True)
        self.cb_smooth_time.setChecked(bool(enabled))
        self.cb_smooth_time.blockSignals(False)
        if emit_signal:
            self.timeSmoothingChanged.emit(self.cb_smooth_time.isChecked())

    def set_jerk_check_enabled(self, enabled: bool, emit_signal: bool = False) -> None:
        self.cb_check_jerk.blockSignals(True)
        self.cb_check_jerk.setChecked(bool(enabled))
        self.cb_check_jerk.blockSignals(False)
        if emit_signal:
            self.jerkCheckChanged.emit(self.cb_check_jerk.isChecked())

    def _update_buttons_state(self) -> None:
        if self._is_editing_active:
            self.btn_add.setEnabled(False)
            self.btn_edit.setEnabled(False)
            self.btn_delete.setEnabled(False)
            self.btn_delete_all.setEnabled(False)
            self.btn_go_to.setEnabled(False)
            self.btn_move_up.setEnabled(False)
            self.btn_move_down.setEnabled(False)
            self.btn_import.setEnabled(False)
            self.btn_export.setEnabled(False)
            return

        row = self._selected_row()
        has_selection = row is not None
        has_keypoints = len(self._keypoints) > 0
        self.btn_add.setEnabled(True)
        self.btn_edit.setEnabled(has_selection)
        self.btn_delete.setEnabled(has_selection)
        self.btn_delete_all.setEnabled(has_keypoints)
        self.btn_go_to.setEnabled(has_selection)
        self.btn_move_up.setEnabled(has_selection and row is not None and row > 0)
        self.btn_move_down.setEnabled(has_selection and row is not None and row < (len(self._keypoints) - 1))
        self.btn_import.setEnabled(True)
        self.btn_export.setEnabled(has_keypoints)

    def _emit_trajectory_preview(self, keypoints: list[TrajectoryKeypoint]) -> None:
        self.trajectoryPreviewRequested.emit([keypoint.clone() for keypoint in keypoints])

    def _resolve_segment_tangents_for_keypoint(self, keypoints: list[TrajectoryKeypoint], row: int) -> tuple[XYZ3, XYZ3] | None:
        if row < 0 or row >= len(keypoints):
            return None

        end_xyz = resolve_keypoint_xyz(
            self.robot_model,
            keypoints[row],
            tool=self.tool_model.get_tool(),
            robot_base_pose_world=self.workspace_model.get_robot_base_transform_world(),
        )
        if end_xyz is None:
            return None

        if row > 0:
            start_xyz = resolve_keypoint_xyz(
                self.robot_model,
                keypoints[row - 1],
                tool=self.tool_model.get_tool(),
                robot_base_pose_world=self.workspace_model.get_robot_base_transform_world(),
            )
        else:
            tcp_pose = self.robot_model.get_tcp_pose()
            start_xyz = XYZ3(tcp_pose.x, tcp_pose.y, tcp_pose.z)
        if start_xyz is None:
            return None

        current_keypoint = keypoints[row]
        if current_keypoint.mode == KeypointMotionMode.CUBIC:
            segment_length_mm = math_utils.norm3(
                end_xyz.x - start_xyz.x,
                end_xyz.y - start_xyz.y,
                end_xyz.z - start_xyz.z,
            )
            start_tangent, end_tangent = current_keypoint.resolve_cubic_tangent_vectors(segment_length_mm)
            return start_tangent, end_tangent

        return current_keypoint.resolve_linear_tangent_vectors(start_xyz, end_xyz)

    def _auto_update_adjacent_cubic_tangents(self, keypoints: list[TrajectoryKeypoint], row: int) -> None:
        tangents = self._resolve_segment_tangents_for_keypoint(keypoints, row)
        if tangents is None:
            return

        start_tangent, end_tangent = tangents
        previous_row = row - 1
        if previous_row >= 0:
            previous_keypoint = keypoints[previous_row]
            if previous_keypoint.mode == KeypointMotionMode.CUBIC:
                previous_keypoint.cubic_vectors[1] = (-start_tangent).normalized()
                previous_keypoint.cubic_amplitudes_mm[1] = start_tangent.norm()

        next_row = row + 1
        if next_row < len(keypoints):
            next_keypoint = keypoints[next_row]
            if next_keypoint.mode == KeypointMotionMode.CUBIC:
                next_keypoint.cubic_vectors[0] = (-end_tangent).normalized()
                next_keypoint.cubic_amplitudes_mm[0] = end_tangent.norm()

    def _on_active_dialog_preview_keypoint_changed(self, preview_keypoint: TrajectoryKeypoint) -> None:
        if self._active_dialog_mode == "add":
            self._emit_trajectory_preview([*self._keypoints, preview_keypoint])
            return

        row = self._active_dialog_row
        dialog = self._active_dialog
        if self._active_dialog_mode != "edit" or row is None or row < 0 or row >= len(self._keypoints):
            return
        preview_keypoints = [keypoint.clone() for keypoint in self._keypoints]
        preview_keypoints[row] = preview_keypoint.clone()
        if dialog is not None and dialog.should_auto_update_adjacent_cubic_tangents():
            self._auto_update_adjacent_cubic_tangents(preview_keypoints, row)
        self._emit_trajectory_preview(preview_keypoints)

    @staticmethod
    def _default_linear_tangent_ratio_for_bezier_degree(degree: TrajectoryBezierDegree) -> float:
        if degree == TrajectoryBezierDegree.BEZIER5:
            return 0.15
        return 0.30

    def _default_linear_tangent_ratios(self) -> list[float]:
        ratio = TrajectoryConfigWidget._default_linear_tangent_ratio_for_bezier_degree(self.get_bezier_degree())
        return [ratio, ratio]

    def _focus_active_dialog(self) -> bool:
        if self._active_dialog is None:
            return False
        self._active_dialog.showNormal()
        self._active_dialog.raise_()
        self._active_dialog.activateWindow()
        return True

    def _open_keypoint_dialog(self, mode: str, row: int | None, keypoint: TrajectoryKeypoint) -> None:
        if self._focus_active_dialog():
            return

        dialog = TrajectoryKeypointDialog(self.robot_model, self.tool_model, self.workspace_model, self)
        dialog.setModal(False)
        dialog.setWindowModality(Qt.WindowModality.NonModal)
        dialog.setWindowFlag(Qt.WindowType.Window, True)

        self._active_dialog = dialog
        self._active_dialog_mode = mode
        self._active_dialog_row = row

        preview_index = len(self._keypoints) if mode == "add" else (row if row is not None else -1)
        self._set_editing_active(True)
        self.editingSessionStarted.emit(preview_index)
        self.showRobotGhostRequested.emit()

        dialog.updateRobotGhostRequested.connect(self.updateRobotGhostRequested.emit)
        dialog.previewKeypointChanged.connect(self._on_active_dialog_preview_keypoint_changed)
        dialog.finished.connect(self._on_active_dialog_finished)
        dialog.set_keypoint_context(self.get_keypoints(), preview_index, self._trajectory_context)
        dialog.load_keypoint(keypoint)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()

    def _on_add_clicked(self) -> None:
        if self._keypoints:
            last_keypoint = self._keypoints[-1]
            initial_keypoint = TrajectoryKeypoint(
                target_type=last_keypoint.target_type,
                cartesian_target=last_keypoint.cartesian_target.copy(),
                cartesian_frame=last_keypoint.cartesian_frame,
                joint_target=list(last_keypoint.joint_target),
                linear_tangent_ratios=self._default_linear_tangent_ratios(),
            )
        else:
            initial_keypoint = TrajectoryKeypoint(
                cartesian_target=convert_pose_from_base_frame(
                    self.robot_model.get_tcp_pose(),
                    ReferenceFrame.from_value(self.get_cartesian_display_frame()),
                    self.workspace_model.get_robot_base_transform_world(),
                ),
                cartesian_frame=ReferenceFrame.from_value(self.get_cartesian_display_frame()),
                joint_target=list(self.robot_model.get_joints()),
                linear_tangent_ratios=self._default_linear_tangent_ratios(),
            )

        self._open_keypoint_dialog(
            "add",
            None,
            initial_keypoint,
        )

    def _on_edit_clicked(self) -> None:
        row = self._selected_row()
        if row is None:
            return
        self._open_keypoint_dialog("edit", row, self._keypoints[row].clone())

    def _on_active_dialog_finished(self, result: int) -> None:
        dialog = self._active_dialog
        mode = self._active_dialog_mode
        row = self._active_dialog_row

        accepted = result == QDialog.DialogCode.Accepted
        new_selection_row: int | None = None
        if dialog is not None and accepted:
            keypoint = dialog.get_keypoint()
            if mode == "add":
                self._keypoints.append(keypoint)
                new_selection_row = len(self._keypoints) - 1
                self.add_requested.emit()
            elif mode == "edit" and row is not None and 0 <= row < len(self._keypoints):
                self._keypoints[row] = keypoint
                if dialog.should_auto_update_adjacent_cubic_tangents():
                    self._auto_update_adjacent_cubic_tangents(self._keypoints, row)
                new_selection_row = row
                self.edit_requested.emit()
            self._refresh_table()
            self._emit_keypoints_changed()

        if dialog is not None:
            dialog.deleteLater()

        self._active_dialog = None
        self._active_dialog_mode = None
        self._active_dialog_row = None
        self._set_editing_active(False)
        self.editingSessionFinished.emit()
        self.hideRobotGhostRequested.emit()
        if accepted and new_selection_row is not None and 0 <= new_selection_row < self.keypoints_table.rowCount():
            self.keypoints_table.selectRow(new_selection_row)
        self.trajectoryPreviewFinished.emit()
        self._emit_selection_changed()

    def _on_delete_clicked(self) -> None:
        row = self._selected_row()
        if row is None:
            return
        self._keypoints.pop(row)
        self._refresh_table()
        self.delete_requested.emit()
        self._emit_keypoints_changed()
        self._emit_selection_changed()

    def _on_delete_all_clicked(self) -> None:
        if not self._keypoints:
            return

        answer = QMessageBox.question(
            self,
            "Supprimer tous les points cles",
            "Voulez-vous supprimer tous les points cles de la trajectoire ?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return

        self._keypoints.clear()
        self._refresh_table()
        self.keypoints_table.clearSelection()
        self._emit_keypoints_changed()
        self._emit_selection_changed()

    def _on_go_to_clicked(self) -> None:
        row = self._selected_row()
        if row is None:
            return
        self.goToRequested.emit(row)

    def _move_selected_keypoint(self, offset: int) -> None:
        row = self._selected_row()
        if row is None:
            return
        new_row = row + offset
        if new_row < 0 or new_row >= len(self._keypoints):
            return

        self._keypoints[row], self._keypoints[new_row] = self._keypoints[new_row], self._keypoints[row]
        self._refresh_table()
        self.keypoints_table.selectRow(new_row)
        self._emit_keypoints_changed()
        self._emit_selection_changed()

    def _on_move_up_clicked(self) -> None:
        self._move_selected_keypoint(-1)

    def _on_move_down_clicked(self) -> None:
        self._move_selected_keypoint(1)

    def _on_import_clicked(self) -> None:
        start_dir = self._preferred_trajectories_dir()
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Importer une trajectoire",
            start_dir,
            "Fichiers JSON (*.json);;Tous les fichiers (*.*)",
        )
        if not path:
            return

        try:
            with open(path, "r", encoding="utf-8") as f:
                payload = json.load(f)
            if not isinstance(payload, dict):
                raise ValueError("Format invalide: le fichier doit contenir un objet de trajectoire.")

            raw_keypoints = payload.get("keypoints")
            smooth_time_enabled = bool(payload.get("smooth_time_enabled", True))
            jerk_check_enabled = bool(payload.get("jerk_check_enabled", True))
            bezier_degree = TrajectoryBezierDegree.from_value(payload.get("bezier_degree", TrajectoryBezierDegree.BEZIER5.value))
            if not isinstance(raw_keypoints, list):
                raise ValueError("Format invalide: liste de keypoints introuvable.")
            parsed_keypoints = [TrajectoryKeypoint.from_dict(item) for item in raw_keypoints]
        except Exception as exc:
            QMessageBox.warning(self, "Import trajectoire", f"Impossible d'importer le fichier.\n{exc}")
            return

        self._keypoints = parsed_keypoints
        self.set_time_smoothing_enabled(smooth_time_enabled, emit_signal=False)
        self.set_jerk_check_enabled(jerk_check_enabled, emit_signal=False)
        self.set_bezier_degree(bezier_degree, emit_signal=False)
        self._refresh_table()
        self.keypoints_table.clearSelection()
        self._emit_selection_changed()
        self._emit_keypoints_changed()

    def _on_export_clicked(self) -> None:
        if not self._keypoints:
            return
        start_dir = self._preferred_trajectories_dir()
        default_path = str(Path(start_dir) / "trajectory_keypoints.json") if start_dir else "trajectory_keypoints.json"
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Exporter une trajectoire",
            default_path,
            "Fichiers JSON (*.json);;Tous les fichiers (*.*)",
        )
        if not path:
            return

        payload = {
            "smooth_time_enabled": self.is_time_smoothing_enabled(),
            "jerk_check_enabled": self.is_jerk_check_enabled(),
            "bezier_degree": self.get_bezier_degree().value,
            "keypoints": [keypoint.to_dict() for keypoint in self._keypoints],
        }
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2, ensure_ascii=False)
        except Exception as exc:
            QMessageBox.warning(self, "Export trajectoire", f"Impossible d'exporter le fichier.\n{exc}")

    def _on_table_selection_changed(self) -> None:
        self._update_buttons_state()
        self._emit_selection_changed()

    def _selected_row(self) -> Optional[int]:
        model = self.keypoints_table.selectionModel()
        if model is None:
            return None
        indexes = model.selectedRows()
        if not indexes:
            return None
        return indexes[0].row()

    @staticmethod
    def _keypoint_target_values(keypoint: TrajectoryKeypoint) -> list[float]:
        if keypoint.target_type == KeypointTargetType.CARTESIAN:
            return keypoint.cartesian_target.to_list()
        return keypoint.joint_target

    @staticmethod
    def _speed_text(keypoint: TrajectoryKeypoint) -> str:
        if keypoint.mode == KeypointMotionMode.PTP:
            return f"{keypoint.speed:.1f} %"
        return f"{keypoint.speed:.3f} m/s"

    @staticmethod
    def _mode_text(keypoint: TrajectoryKeypoint) -> str:
        if keypoint.mode == KeypointMotionMode.CUBIC:
            return "BEZIER"
        return keypoint.mode.value

    @staticmethod
    def _configuration_text(keypoint: TrajectoryKeypoint) -> str:
        if keypoint.target_type == KeypointTargetType.JOINT:
            forced = keypoint.forced_config.name if keypoint.forced_config is not None else "?"
            return f"JOINT({forced})"
        if keypoint.configuration_policy == ConfigurationPolicy.AUTO:
            return "AUTO"
        if keypoint.configuration_policy == ConfigurationPolicy.CURRENT_BRANCH:
            return "CURRENT_BRANCH"
        if keypoint.configuration_policy == ConfigurationPolicy.FORCED:
            forced = keypoint.forced_config.name if keypoint.forced_config is not None else "?"
            return f"FORCED({forced})"
        return "AUTO"

    def _refresh_table(self) -> None:
        self.keypoints_table.setRowCount(0)
        for idx, keypoint in enumerate(self._keypoints):
            self.keypoints_table.insertRow(idx)
            target_values = self._keypoint_target_values(keypoint)
            configs_txt = self._configuration_text(keypoint)

            values = [
                str(idx + 1),
                (
                    f"CART({keypoint.cartesian_frame.value})"
                    if keypoint.target_type == KeypointTargetType.CARTESIAN
                    else "JOINT"
                ),
                self._mode_text(keypoint),
                self._speed_text(keypoint),
            ]
            values.extend(f"{v:.3f}" for v in target_values[:6])
            values.append(configs_txt)

            for col, text in enumerate(values):
                self.keypoints_table.setItem(idx, col, QTableWidgetItem(text))
        self._update_buttons_state()

    def set_keypoints(self, keypoints: list[TrajectoryKeypoint]) -> None:
        self._keypoints = [keypoint.clone() for keypoint in keypoints]
        self._refresh_table()
        self._emit_selection_changed()

    def set_trajectory_context(self, trajectory_result: TrajectoryResult | None) -> None:
        self._trajectory_context = trajectory_result
        if self._active_dialog is not None:
            self._active_dialog.update_trajectory_context(trajectory_result)

    def get_keypoints(self) -> list[TrajectoryKeypoint]:
        return [keypoint.clone() for keypoint in self._keypoints]
