from __future__ import annotations

import os

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from utils.math_utils import safe_float
from models.types import Pose6
from models.workspace_cad_element import WorkspaceCadElement
from models.workspace_primitive_zone_models import WorkspacePrimitiveZoneData
from widgets.workspace_view.workspace_primitive_zones_editor_widget import WorkspacePrimitiveZonesEditorWidget


class WorkspaceConfigurationWidget(QWidget):
    scene_name_changed = pyqtSignal(str)
    robot_base_pose_world_changed = pyqtSignal(list)
    workspace_save_requested = pyqtSignal()
    workspace_load_requested = pyqtSignal()
    workspace_clear_requested = pyqtSignal()
    workspace_cad_elements_changed = pyqtSignal(list)
    workspace_tcp_zones_changed = pyqtSignal(list)
    workspace_tcp_zone_changed = pyqtSignal(int, object)
    workspace_tcp_zone_added = pyqtSignal(int, object)
    workspace_tcp_zone_removed = pyqtSignal(int)
    workspace_collision_zones_changed = pyqtSignal(list)
    workspace_collision_zone_changed = pyqtSignal(int, object)
    workspace_collision_zone_added = pyqtSignal(int, object)
    workspace_collision_zone_removed = pyqtSignal(int)

    COL_ELEM_NAME = 0
    COL_ELEM_STL = 1
    COL_ELEM_X = 2
    COL_ELEM_Y = 3
    COL_ELEM_Z = 4
    COL_ELEM_A = 5
    COL_ELEM_B = 6
    COL_ELEM_C = 7
    COL_ELEM_STATUS = 8

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.scene_name_line_edit: QLineEdit | None = None
        self._workspace_directory: str = ""
        self._workspace_file_path: str = ""
        self.robot_base_pose_spinboxes: list[QDoubleSpinBox] = []
        self.table_elements: QTableWidget | None = None
        self.tcp_zones_editor: WorkspacePrimitiveZonesEditorWidget | None = None
        self.collision_zones_editor: WorkspacePrimitiveZonesEditorWidget | None = None
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.addWidget(self._build_scene_group())
        layout.addWidget(self._build_elements_group())

        self.tcp_zones_editor = WorkspacePrimitiveZonesEditorWidget(
            title="Zones de travail TCP (repère world)",
            add_button_text="Ajouter zone TCP",
            remove_button_text="Supprimer zone TCP",
            default_name_prefix="Zone TCP",
            parent=self,
        )
        self.tcp_zones_editor.zones_changed.connect(
            lambda _zones: self.workspace_tcp_zones_changed.emit(self.get_workspace_tcp_zones())
        )
        self.tcp_zones_editor.zone_changed.connect(self.workspace_tcp_zone_changed.emit)
        self.tcp_zones_editor.zone_added.connect(self.workspace_tcp_zone_added.emit)
        self.tcp_zones_editor.zone_removed.connect(self.workspace_tcp_zone_removed.emit)
        layout.addWidget(self.tcp_zones_editor)

        self.collision_zones_editor = WorkspacePrimitiveZonesEditorWidget(
            title="Zones de collision (repère world)",
            add_button_text="Ajouter zone collision",
            remove_button_text="Supprimer zone collision",
            default_name_prefix="Zone collision",
            parent=self,
        )
        self.collision_zones_editor.zones_changed.connect(
            lambda _zones: self.workspace_collision_zones_changed.emit(self.get_workspace_collision_zones())
        )
        self.collision_zones_editor.zone_changed.connect(self.workspace_collision_zone_changed.emit)
        self.collision_zones_editor.zone_added.connect(self.workspace_collision_zone_added.emit)
        self.collision_zones_editor.zone_removed.connect(self.workspace_collision_zone_removed.emit)
        layout.addWidget(self.collision_zones_editor)
        layout.addStretch()

    def _build_scene_group(self) -> QGroupBox:
        group = QGroupBox("Scène")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("Nom scène"), 0, 0)
        self.scene_name_line_edit = QLineEdit()
        self.scene_name_line_edit.setPlaceholderText("Nom de scène workspace")
        self.scene_name_line_edit.textChanged.connect(self.scene_name_changed.emit)
        layout.addWidget(self.scene_name_line_edit, 0, 1)

        save_btn = QPushButton("Sauvegarder scène")
        save_btn.clicked.connect(self.workspace_save_requested.emit)
        layout.addWidget(save_btn, 0, 2)

        load_btn = QPushButton("Charger scène")
        load_btn.clicked.connect(self.workspace_load_requested.emit)
        layout.addWidget(load_btn, 0, 3)

        clear_btn = QPushButton("Vider scène")
        clear_btn.clicked.connect(self.workspace_clear_requested.emit)
        layout.addWidget(clear_btn, 0, 4)

        layout.addWidget(QLabel("Base robot dans world"), 1, 0)
        pose_layout = QVBoxLayout()
        label_width = 16
        spinbox_width = 101
        for row_idx, axis_labels in enumerate((["X", "Y", "Z"], ["A", "B", "C"])):
            pose_row = QHBoxLayout()
            for col_idx, label_text in enumerate(axis_labels):
                idx = row_idx * 3 + col_idx
                label = QLabel(label_text)
                label.setFixedWidth(label_width)
                pose_row.addWidget(label)
                spinbox = QDoubleSpinBox()
                spinbox.setFixedWidth(spinbox_width)
                if idx < 3:
                    spinbox.setRange(-100000.0, 100000.0)
                    spinbox.setDecimals(3)
                    spinbox.setSingleStep(1.0)
                else:
                    spinbox.setRange(-360.0, 360.0)
                    spinbox.setDecimals(3)
                    spinbox.setSingleStep(1.0)
                spinbox.valueChanged.connect(self._on_robot_base_pose_world_value_changed)
                self.robot_base_pose_spinboxes.append(spinbox)
                pose_row.addWidget(spinbox)
            pose_row.addStretch()
            pose_layout.addLayout(pose_row)
        layout.addLayout(pose_layout, 1, 1, 1, 4)

        return group

    def _build_elements_group(self) -> QGroupBox:
        group = QGroupBox("Eléments STL (repère world)")
        layout = QVBoxLayout(group)

        self.table_elements = QTableWidget(0, 9)
        self.table_elements.setHorizontalHeaderLabels(
            ["Nom", "Fichier STL", "X", "Y", "Z", "A", "B", "C", "Etat"]
        )
        self.table_elements.horizontalHeader().setDefaultSectionSize(110)
        self.table_elements.itemChanged.connect(self._on_elements_table_item_changed)
        layout.addWidget(self.table_elements)

        btn_row = QHBoxLayout()
        add_btn = QPushButton("Ajouter élément")
        add_btn.clicked.connect(self._on_add_element_clicked)
        btn_row.addWidget(add_btn)

        remove_btn = QPushButton("Supprimer élément")
        remove_btn.clicked.connect(self._on_remove_element_clicked)
        btn_row.addWidget(remove_btn)

        browse_btn = QPushButton("Parcourir STL")
        browse_btn.clicked.connect(self._on_browse_element_stl_clicked)
        btn_row.addWidget(browse_btn)
        btn_row.addStretch()

        layout.addLayout(btn_row)
        return group

    def _on_elements_table_item_changed(self, item: QTableWidgetItem) -> None:
        if item.column() == WorkspaceConfigurationWidget.COL_ELEM_STATUS:
            return
        self._update_element_status_row(item.row())
        self.workspace_cad_elements_changed.emit(self.get_workspace_cad_elements())

    def _on_add_element_clicked(self) -> None:
        if self.table_elements is None:
            return
        row = self.table_elements.rowCount()
        self.table_elements.blockSignals(True)
        try:
            self.table_elements.insertRow(row)
            self.table_elements.setItem(row, self.COL_ELEM_NAME, QTableWidgetItem(f"Elément {row + 1}"))
            self.table_elements.setItem(row, self.COL_ELEM_STL, QTableWidgetItem(""))
            for col in range(self.COL_ELEM_X, self.COL_ELEM_C + 1):
                self.table_elements.setItem(row, col, QTableWidgetItem("0.0"))
            status_item = QTableWidgetItem("")
            status_item.setFlags(status_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table_elements.setItem(row, self.COL_ELEM_STATUS, status_item)
            self._update_element_status_row(row)
        finally:
            self.table_elements.blockSignals(False)
        self.workspace_cad_elements_changed.emit(self.get_workspace_cad_elements())

    def _on_remove_element_clicked(self) -> None:
        if self.table_elements is None:
            return
        row = self.table_elements.currentRow()
        if row < 0:
            row = self.table_elements.rowCount() - 1
        if row < 0:
            return
        self.table_elements.removeRow(row)
        self.workspace_cad_elements_changed.emit(self.get_workspace_cad_elements())

    def _on_browse_element_stl_clicked(self) -> None:
        if self.table_elements is None:
            return

        row = self.table_elements.currentRow()
        if row < 0:
            return

        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Selectionner un STL",
            self._get_stl_start_directory(),
            "STL files (*.stl);;All files (*)",
        )
        if not file_path:
            return

        stl_item = self.table_elements.item(row, self.COL_ELEM_STL)
        if stl_item is None:
            stl_item = QTableWidgetItem("")
            self.table_elements.setItem(row, self.COL_ELEM_STL, stl_item)
        stl_item.setText(self._normalize_project_path(file_path))
        self._update_element_status_row(row)
        self.workspace_cad_elements_changed.emit(self.get_workspace_cad_elements())

    def _update_element_status_row(self, row: int) -> None:
        if self.table_elements is None or not (0 <= row < self.table_elements.rowCount()):
            return

        stl_item = self.table_elements.item(row, self.COL_ELEM_STL)
        stl_path = stl_item.text().strip() if stl_item is not None else ""
        status_item = self.table_elements.item(row, self.COL_ELEM_STATUS)
        if status_item is None:
            status_item = QTableWidgetItem("")
            status_item.setFlags(status_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table_elements.setItem(row, self.COL_ELEM_STATUS, status_item)

        if stl_path == "":
            status_item.setText("Aucun STL")
            status_item.setForeground(QColor("#f2c94c"))
            return

        if os.path.exists(self._resolve_filesystem_path(stl_path)):
            status_item.setText("OK")
            status_item.setForeground(QColor("#6fcf97"))
            return

        status_item.setText("STL introuvable")
        status_item.setForeground(QColor("#eb5757"))

    def _cell_to_float(self, table: QTableWidget, row: int, col: int, default: float = 0.0) -> float:
        item = table.item(row, col)
        return safe_float(item.text() if item is not None else "", default)

    def _on_robot_base_pose_world_value_changed(self, _value: float) -> None:
        self.robot_base_pose_world_changed.emit(self.get_robot_base_pose_world())

    def set_workspace_directory(self, directory: str) -> None:
        self._workspace_directory = str(directory or "").strip()
        self._refresh_scene_name_tooltip()

    def set_workspace_scene_name(self, scene_name: str) -> None:
        if self.scene_name_line_edit is None:
            return
        self.scene_name_line_edit.blockSignals(True)
        self.scene_name_line_edit.setText(scene_name)
        self.scene_name_line_edit.blockSignals(False)

    def get_workspace_scene_name(self) -> str:
        if self.scene_name_line_edit is None:
            return ""
        return self.scene_name_line_edit.text().strip()

    def set_workspace_file_path(self, file_path: str) -> None:
        self._workspace_file_path = str(file_path or "").strip()
        self._refresh_scene_name_tooltip()

    def set_robot_base_pose_world(self, pose: Pose6) -> None:
        if not isinstance(pose, Pose6):
            raise TypeError("pose must be a Pose6")
        values = pose.to_list()
        for idx, spinbox in enumerate(self.robot_base_pose_spinboxes):
            spinbox.blockSignals(True)
            spinbox.setValue(values[idx])
            spinbox.blockSignals(False)

    def get_robot_base_pose_world(self) -> Pose6:
        values = [float(spinbox.value()) for spinbox in self.robot_base_pose_spinboxes[:6]]
        return Pose6(*values)

    def _refresh_scene_name_tooltip(self) -> None:
        if self.scene_name_line_edit is None:
            return
        tooltip_path = self._workspace_file_path if self._workspace_file_path else self._workspace_directory
        self.scene_name_line_edit.setToolTip(tooltip_path)

    def set_workspace_cad_elements(self, values: list[WorkspaceCadElement]) -> None:
        if self.table_elements is None:
            return
        if not all(isinstance(value, WorkspaceCadElement) for value in values):
            raise TypeError("values must contain WorkspaceCadElement")
        normalized = [value.copy() for value in values]
        self.table_elements.blockSignals(True)
        try:
            self.table_elements.setRowCount(0)
            for row, value in enumerate(normalized):
                self.table_elements.insertRow(row)
                pose = value.pose
                self.table_elements.setItem(row, self.COL_ELEM_NAME, QTableWidgetItem(value.name))
                self.table_elements.setItem(row, self.COL_ELEM_STL, QTableWidgetItem(value.cad_model))
                self.table_elements.setItem(row, self.COL_ELEM_X, QTableWidgetItem(str(float(pose.x))))
                self.table_elements.setItem(row, self.COL_ELEM_Y, QTableWidgetItem(str(float(pose.y))))
                self.table_elements.setItem(row, self.COL_ELEM_Z, QTableWidgetItem(str(float(pose.z))))
                self.table_elements.setItem(row, self.COL_ELEM_A, QTableWidgetItem(str(float(pose.a))))
                self.table_elements.setItem(row, self.COL_ELEM_B, QTableWidgetItem(str(float(pose.b))))
                self.table_elements.setItem(row, self.COL_ELEM_C, QTableWidgetItem(str(float(pose.c))))
                status_item = QTableWidgetItem("")
                status_item.setFlags(status_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                self.table_elements.setItem(row, self.COL_ELEM_STATUS, status_item)
                self._update_element_status_row(row)
        finally:
            self.table_elements.blockSignals(False)

    def get_workspace_cad_elements(self) -> list[WorkspaceCadElement]:
        if self.table_elements is None:
            return []

        values: list[WorkspaceCadElement] = []
        for row in range(self.table_elements.rowCount()):
            name_item = self.table_elements.item(row, self.COL_ELEM_NAME)
            stl_item = self.table_elements.item(row, self.COL_ELEM_STL)
            values.append(
                WorkspaceCadElement(
                    name=name_item.text().strip() if name_item is not None else f"Element {row + 1}",
                    cad_model=stl_item.text().strip() if stl_item is not None else "",
                    pose=Pose6(
                        self._cell_to_float(self.table_elements, row, self.COL_ELEM_X, 0.0),
                        self._cell_to_float(self.table_elements, row, self.COL_ELEM_Y, 0.0),
                        self._cell_to_float(self.table_elements, row, self.COL_ELEM_Z, 0.0),
                        self._cell_to_float(self.table_elements, row, self.COL_ELEM_A, 0.0),
                        self._cell_to_float(self.table_elements, row, self.COL_ELEM_B, 0.0),
                        self._cell_to_float(self.table_elements, row, self.COL_ELEM_C, 0.0),
                    ),
                )
            )
        return values

    def set_workspace_tcp_zones(self, values: list[WorkspacePrimitiveZoneData]) -> None:
        if self.tcp_zones_editor is None:
            return
        self.tcp_zones_editor.set_zones(values)

    def get_workspace_tcp_zones(self) -> list[WorkspacePrimitiveZoneData]:
        if self.tcp_zones_editor is None:
            return []
        return self.tcp_zones_editor.get_zones()

    def set_workspace_collision_zones(self, values: list[WorkspacePrimitiveZoneData]) -> None:
        if self.collision_zones_editor is None:
            return
        self.collision_zones_editor.set_zones(values)

    def get_workspace_collision_zones(self) -> list[WorkspacePrimitiveZoneData]:
        if self.collision_zones_editor is None:
            return []
        return self.collision_zones_editor.get_zones()

    @staticmethod
    def _resolve_filesystem_path(path: str) -> str:
        if not path:
            return ""
        return os.path.abspath(path)

    @staticmethod
    def _normalize_project_path(path: str) -> str:
        absolute_path = os.path.abspath(path)
        project_root = os.path.abspath(os.getcwd())
        try:
            common_path = os.path.commonpath([project_root, absolute_path])
        except ValueError:
            return absolute_path

        if common_path != project_root:
            return absolute_path

        try:
            relative_path = os.path.relpath(absolute_path, project_root)
        except ValueError:
            return absolute_path

        relative_path = relative_path.replace("\\", "/")
        if relative_path == ".":
            return "./"
        return f"./{relative_path}" if not relative_path.startswith(".") else relative_path

    @staticmethod
    def _get_stl_start_directory() -> str:
        current_dir = os.getcwd()
        robots_stl_dir = os.path.join(current_dir, "default", "robots_stl")
        if os.path.isdir(robots_stl_dir):
            return robots_stl_dir
        return current_dir
