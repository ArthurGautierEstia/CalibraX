from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QDialog,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.robot_model import RobotModel
from widgets.trajectory_view.trajectory_keypoint_dialog import TrajectoryKeypointDialog


class TrajectoryConfigWidget(QWidget):
    """Widget to configure a trajectory with a list of keypoints."""

    add_requested = pyqtSignal()
    edit_requested = pyqtSignal()
    delete_requested = pyqtSignal()
    keypoints_changed = pyqtSignal(list)

    def __init__(self, robot_model: RobotModel, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.robot_model = robot_model

        self.keypoints_table = QTableWidget(0, 11)
        self.btn_add = QPushButton("Ajouter")
        self.btn_edit = QPushButton("Editer")
        self.btn_delete = QPushButton("Supprimer")

        self._keypoints: list[TrajectoryKeypoint] = []

        self._setup_ui()
        self._setup_connections()

    def _setup_ui(self) -> None:
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)

        layout = QVBoxLayout(self)

        title = QLabel("Configuration de la trajectoire")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(title)

        self.keypoints_table.setHorizontalHeaderLabels([
            "#", "Cible", "Mode", "Vitesse", "1", "2", "3", "4", "5", "6", "Configs"
        ])
        self.keypoints_table.horizontalHeader().setDefaultSectionSize(90)
        self.keypoints_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.keypoints_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.keypoints_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.keypoints_table.setMinimumHeight(220)
        layout.addWidget(self.keypoints_table)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self.btn_add)
        btn_row.addWidget(self.btn_edit)
        btn_row.addWidget(self.btn_delete)
        btn_row.addStretch()
        layout.addLayout(btn_row)

    def _setup_connections(self) -> None:
        self.btn_add.clicked.connect(self._on_add_clicked)
        self.btn_edit.clicked.connect(self._on_edit_clicked)
        self.btn_delete.clicked.connect(self._on_delete_clicked)

    def _emit_keypoints_changed(self) -> None:
        self.keypoints_changed.emit(self.get_keypoints())

    def _on_add_clicked(self) -> None:
        dialog = TrajectoryKeypointDialog(self.robot_model, self)
        dialog.load_keypoint(TrajectoryKeypoint())
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self._keypoints.append(dialog.get_keypoint())
            self._refresh_table()
            self.add_requested.emit()
            self._emit_keypoints_changed()

    def _on_edit_clicked(self) -> None:
        row = self._selected_row()
        if row is None:
            return
        dialog = TrajectoryKeypointDialog(self.robot_model, self)
        dialog.load_keypoint(self._keypoints[row])
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self._keypoints[row] = dialog.get_keypoint()
            self._refresh_table()
            self.edit_requested.emit()
            self._emit_keypoints_changed()

    def _on_delete_clicked(self) -> None:
        row = self._selected_row()
        if row is None:
            return
        self._keypoints.pop(row)
        self._refresh_table()
        self.delete_requested.emit()
        self._emit_keypoints_changed()

    def _selected_row(self) -> Optional[int]:
        indexes = self.keypoints_table.selectionModel().selectedRows()
        if not indexes:
            return None
        return indexes[0].row()

    @staticmethod
    def _keypoint_target_values(keypoint: TrajectoryKeypoint) -> list[float]:
        if keypoint.target_type == KeypointTargetType.CARTESIAN:
            return keypoint.cartesian_target
        return keypoint.joint_target

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

    def set_keypoints(self, keypoints: list[TrajectoryKeypoint]) -> None:
        self._keypoints = [keypoint.clone() for keypoint in keypoints]
        self._refresh_table()

    def get_keypoints(self) -> list[TrajectoryKeypoint]:
        return [keypoint.clone() for keypoint in self._keypoints]
