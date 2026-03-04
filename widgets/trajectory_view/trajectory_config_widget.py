from __future__ import annotations

import json
from pathlib import Path
from typing import Optional
import os

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QAbstractItemView,
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
    QHeaderView
)

import utils.math_utils as math_utils
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.robot_model import RobotModel
from models.trajectory_result import TrajectoryResult
from widgets.trajectory_view.trajectory_keypoint_dialog import TrajectoryKeypointDialog


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
    updateRobotGhostRequested = pyqtSignal(list)

    def __init__(self, robot_model: RobotModel, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.robot_model = robot_model

        self.keypoints_table = QTableWidget(0, 11)
        self.btn_add = QPushButton("Ajouter")
        self.btn_edit = QPushButton("Editer")
        self.btn_delete = QPushButton("Supprimer")
        self.btn_move_up = QPushButton("Monter")
        self.btn_move_down = QPushButton("Descendre")
        self.btn_import = QPushButton("Importer")
        self.btn_export = QPushButton("Exporter")

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
        btn_row.addWidget(self.btn_delete)
        btn_row.addWidget(self.btn_move_up)
        btn_row.addWidget(self.btn_move_down)
        btn_row.addWidget(self.btn_import)
        btn_row.addWidget(self.btn_export)
        btn_row.addStretch()
        layout.addLayout(btn_row)

    def _setup_connections(self) -> None:
        self.btn_add.clicked.connect(self._on_add_clicked)
        self.btn_edit.clicked.connect(self._on_edit_clicked)
        self.btn_delete.clicked.connect(self._on_delete_clicked)
        self.btn_move_up.clicked.connect(self._on_move_up_clicked)
        self.btn_move_down.clicked.connect(self._on_move_down_clicked)
        self.btn_import.clicked.connect(self._on_import_clicked)
        self.btn_export.clicked.connect(self._on_export_clicked)
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
        self._update_buttons_state()

    def _update_buttons_state(self) -> None:
        if self._is_editing_active:
            self.btn_add.setEnabled(False)
            self.btn_edit.setEnabled(False)
            self.btn_delete.setEnabled(False)
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
        self.btn_move_up.setEnabled(has_selection and row is not None and row > 0)
        self.btn_move_down.setEnabled(has_selection and row is not None and row < (len(self._keypoints) - 1))
        self.btn_import.setEnabled(True)
        self.btn_export.setEnabled(has_keypoints)

    def _emit_trajectory_preview(self, keypoints: list[TrajectoryKeypoint]) -> None:
        self.trajectoryPreviewRequested.emit([keypoint.clone() for keypoint in keypoints])

    def _resolve_keypoint_xyz(self, keypoint: TrajectoryKeypoint) -> list[float] | None:
        if keypoint.target_type == KeypointTargetType.CARTESIAN:
            if len(keypoint.cartesian_target) < 3:
                return None
            return [float(keypoint.cartesian_target[0]), float(keypoint.cartesian_target[1]), float(keypoint.cartesian_target[2])]

        fk_result = self.robot_model.compute_fk_joints(keypoint.joint_target)
        if fk_result is None:
            return None
        _, _, pose, _, _ = fk_result
        if len(pose) < 3:
            return None
        return [float(pose[0]), float(pose[1]), float(pose[2])]

    def _resolve_segment_tangents_for_keypoint(self, keypoints: list[TrajectoryKeypoint], row: int) -> tuple[list[float], list[float]] | None:
        if row < 0 or row >= len(keypoints):
            return None

        end_xyz = self._resolve_keypoint_xyz(keypoints[row])
        if end_xyz is None:
            return None

        if row > 0:
            start_xyz = self._resolve_keypoint_xyz(keypoints[row - 1])
        else:
            tcp_pose = self.robot_model.get_tcp_pose()
            start_xyz = [float(v) for v in tcp_pose[:3]] if len(tcp_pose) >= 3 else None
        if start_xyz is None:
            return None

        current_keypoint = keypoints[row]
        if current_keypoint.mode == KeypointMotionMode.CUBIC:
            segment_length_mm = math_utils.norm3(
                end_xyz[0] - start_xyz[0],
                end_xyz[1] - start_xyz[1],
                end_xyz[2] - start_xyz[2],
            )
            return current_keypoint.resolve_cubic_tangent_vectors(segment_length_mm)

        dx = (end_xyz[0] - start_xyz[0]) * 0.3
        dy = (end_xyz[1] - start_xyz[1]) * 0.3
        dz = (end_xyz[2] - start_xyz[2]) * 0.3
        return [dx, dy, dz], [-dx, -dy, -dz]

    def _auto_update_adjacent_cubic_tangents(self, keypoints: list[TrajectoryKeypoint], row: int) -> None:
        tangents = self._resolve_segment_tangents_for_keypoint(keypoints, row)
        if tangents is None:
            return

        start_tangent, end_tangent = tangents
        previous_row = row - 1
        if previous_row >= 0:
            previous_keypoint = keypoints[previous_row]
            if previous_keypoint.mode == KeypointMotionMode.CUBIC:
                previous_keypoint.cubic_vectors[1] = math_utils.normalize3(
                    [-start_tangent[0], -start_tangent[1], -start_tangent[2]]
                )

        next_row = row + 1
        if next_row < len(keypoints):
            next_keypoint = keypoints[next_row]
            if next_keypoint.mode == KeypointMotionMode.CUBIC:
                next_keypoint.cubic_vectors[0] = math_utils.normalize3(
                    [-end_tangent[0], -end_tangent[1], -end_tangent[2]]
                )

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

        dialog = TrajectoryKeypointDialog(self.robot_model, self)
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
        self._open_keypoint_dialog(
            "add",
            None,
            TrajectoryKeypoint(
                cartesian_target=list(self.robot_model.get_tcp_pose()),
                joint_target=list(self.robot_model.get_joints()),
            ),
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
            raw_keypoints = payload.get("keypoints") if isinstance(payload, dict) else payload
            if not isinstance(raw_keypoints, list):
                raise ValueError("Format invalide: liste de keypoints introuvable.")
            self._keypoints = [TrajectoryKeypoint.from_dict(item) for item in raw_keypoints]
        except Exception as exc:
            QMessageBox.warning(self, "Import trajectoire", f"Impossible d'importer le fichier.\n{exc}")
            return

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
            "format": "calibrax_trajectory_keypoints",
            "version": 1,
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
        return keypoint.cartesian_target if keypoint.target_type == KeypointTargetType.CARTESIAN else keypoint.joint_target

    @staticmethod
    def _speed_text(keypoint: TrajectoryKeypoint) -> str:
        if keypoint.mode == KeypointMotionMode.PTP:
            return f"{keypoint.speed:.1f} %"
        return f"{keypoint.speed:.3f} m/s"

    def _refresh_table(self) -> None:
        self.keypoints_table.setRowCount(0)
        for idx, keypoint in enumerate(self._keypoints):
            self.keypoints_table.insertRow(idx)
            target_values = self._keypoint_target_values(keypoint)
            configs_txt = ", ".join(cfg.name for cfg in keypoint.allowed_configs)

            values = [
                str(idx + 1),
                "CART" if keypoint.target_type == KeypointTargetType.CARTESIAN else "JOINT",
                keypoint.mode.value,
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

    def get_keypoints(self) -> list[TrajectoryKeypoint]:
        return [keypoint.clone() for keypoint in self._keypoints]
