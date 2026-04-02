from __future__ import annotations

import os
from typing import Any

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
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

from models.collider_models import parse_primitive_colliders
from models.workspace_file import parse_workspace_cad_elements


class WorkspaceConfigurationWidget(QWidget):
    scene_name_changed = pyqtSignal(str)
    workspace_save_requested = pyqtSignal()
    workspace_load_requested = pyqtSignal()
    workspace_cad_elements_changed = pyqtSignal(list)
    workspace_tcp_zones_changed = pyqtSignal(list)
    workspace_collision_zones_changed = pyqtSignal(list)

    COL_ELEM_NAME = 0
    COL_ELEM_STL = 1
    COL_ELEM_X = 2
    COL_ELEM_Y = 3
    COL_ELEM_Z = 4
    COL_ELEM_A = 5
    COL_ELEM_B = 6
    COL_ELEM_C = 7
    COL_ELEM_STATUS = 8

    COL_PRIM_ENABLED = 0
    COL_PRIM_NAME = 1
    COL_PRIM_TYPE = 2
    COL_PRIM_X = 3
    COL_PRIM_Y = 4
    COL_PRIM_Z = 5
    COL_PRIM_A = 6
    COL_PRIM_B = 7
    COL_PRIM_C = 8
    COL_PRIM_SIZE_X = 9
    COL_PRIM_SIZE_Y = 10
    COL_PRIM_SIZE_Z = 11
    COL_PRIM_RADIUS = 12
    COL_PRIM_HEIGHT = 13

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.scene_name_line_edit: QLineEdit | None = None
        self._workspace_directory: str = ""
        self._workspace_file_path: str = ""

        self.table_elements: QTableWidget | None = None
        self.table_tcp_zones: QTableWidget | None = None
        self.table_collision_zones: QTableWidget | None = None

        self._tcp_enabled_checkboxes: list[QCheckBox] = []
        self._tcp_type_combos: list[QComboBox] = []
        self._collision_enabled_checkboxes: list[QCheckBox] = []
        self._collision_type_combos: list[QComboBox] = []

        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.addWidget(self._build_scene_group())
        layout.addWidget(self._build_elements_group())
        layout.addWidget(self._build_tcp_zones_group())
        layout.addWidget(self._build_collision_zones_group())
        layout.addStretch()

    def _build_scene_group(self) -> QGroupBox:
        group = QGroupBox("Scene")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("Nom scene"), 0, 0)
        self.scene_name_line_edit = QLineEdit()
        self.scene_name_line_edit.setPlaceholderText("Nom de scene workspace")
        self.scene_name_line_edit.textChanged.connect(self.scene_name_changed.emit)
        layout.addWidget(self.scene_name_line_edit, 0, 1)

        save_btn = QPushButton("Sauvegarder scene")
        save_btn.clicked.connect(self.workspace_save_requested.emit)
        layout.addWidget(save_btn, 0, 2)

        load_btn = QPushButton("Charger scene")
        load_btn.clicked.connect(self.workspace_load_requested.emit)
        layout.addWidget(load_btn, 0, 3)


        return group

    def _build_elements_group(self) -> QGroupBox:
        group = QGroupBox("Elements STL (repere world)")
        layout = QVBoxLayout(group)

        self.table_elements = QTableWidget(0, 9)
        self.table_elements.setHorizontalHeaderLabels(
            ["Nom", "Fichier STL", "X", "Y", "Z", "A", "B", "C", "Etat"]
        )
        self.table_elements.horizontalHeader().setDefaultSectionSize(110)
        self.table_elements.itemChanged.connect(self._on_elements_table_item_changed)
        layout.addWidget(self.table_elements)

        btn_row = QHBoxLayout()
        add_btn = QPushButton("Ajouter element")
        add_btn.clicked.connect(self._on_add_element_clicked)
        btn_row.addWidget(add_btn)

        remove_btn = QPushButton("Supprimer element")
        remove_btn.clicked.connect(self._on_remove_element_clicked)
        btn_row.addWidget(remove_btn)

        browse_btn = QPushButton("Parcourir STL")
        browse_btn.clicked.connect(self._on_browse_element_stl_clicked)
        btn_row.addWidget(browse_btn)
        btn_row.addStretch()

        layout.addLayout(btn_row)
        return group

    def _build_tcp_zones_group(self) -> QGroupBox:
        group = QGroupBox("Zones de travail TCP (repere world)")
        layout = QVBoxLayout(group)
        self.table_tcp_zones = self._create_primitive_table(self._on_tcp_table_item_changed)
        layout.addWidget(self.table_tcp_zones)

        btn_row = QHBoxLayout()
        add_btn = QPushButton("Ajouter zone TCP")
        add_btn.clicked.connect(self._on_add_tcp_zone_clicked)
        btn_row.addWidget(add_btn)

        remove_btn = QPushButton("Supprimer zone TCP")
        remove_btn.clicked.connect(self._on_remove_tcp_zone_clicked)
        btn_row.addWidget(remove_btn)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        return group

    def _build_collision_zones_group(self) -> QGroupBox:
        group = QGroupBox("Zones de collision (repere world)")
        layout = QVBoxLayout(group)
        self.table_collision_zones = self._create_primitive_table(self._on_collision_table_item_changed)
        layout.addWidget(self.table_collision_zones)

        btn_row = QHBoxLayout()
        add_btn = QPushButton("Ajouter zone collision")
        add_btn.clicked.connect(self._on_add_collision_zone_clicked)
        btn_row.addWidget(add_btn)

        remove_btn = QPushButton("Supprimer zone collision")
        remove_btn.clicked.connect(self._on_remove_collision_zone_clicked)
        btn_row.addWidget(remove_btn)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        return group

    @staticmethod
    def _create_primitive_table(item_changed_handler) -> QTableWidget:
        table = QTableWidget(0, 14)
        table.setHorizontalHeaderLabels(
            [
                "Actif",
                "Nom",
                "Type",
                "X",
                "Y",
                "Z",
                "A",
                "B",
                "C",
                "Size X",
                "Size Y",
                "Size Z",
                "Rayon",
                "Hauteur",
            ]
        )
        table.horizontalHeader().setDefaultSectionSize(90)
        table.itemChanged.connect(item_changed_handler)
        return table

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
            self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_NAME, QTableWidgetItem(f"Element {row + 1}"))
            self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_STL, QTableWidgetItem(""))
            for col in range(WorkspaceConfigurationWidget.COL_ELEM_X, WorkspaceConfigurationWidget.COL_ELEM_C + 1):
                self.table_elements.setItem(row, col, QTableWidgetItem("0.0"))
            status_item = QTableWidgetItem("")
            status_item.setFlags(status_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_STATUS, status_item)
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

        stl_item = self.table_elements.item(row, WorkspaceConfigurationWidget.COL_ELEM_STL)
        if stl_item is None:
            stl_item = QTableWidgetItem("")
            self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_STL, stl_item)
        stl_item.setText(self._normalize_project_path(file_path))
        self._update_element_status_row(row)
        self.workspace_cad_elements_changed.emit(self.get_workspace_cad_elements())

    def _on_tcp_table_item_changed(self, _item: QTableWidgetItem) -> None:
        self.workspace_tcp_zones_changed.emit(self.get_workspace_tcp_zones())

    def _on_collision_table_item_changed(self, _item: QTableWidgetItem) -> None:
        self.workspace_collision_zones_changed.emit(self.get_workspace_collision_zones())

    def _on_add_tcp_zone_clicked(self) -> None:
        self._insert_primitive_row(
            self.table_tcp_zones,
            self._tcp_enabled_checkboxes,
            self._tcp_type_combos,
            {
                "name": f"Zone TCP {self.table_tcp_zones.rowCount() + 1 if self.table_tcp_zones is not None else 1}",
                "enabled": True,
                "shape": "box",
                "pose": [0.0] * 6,
                "size_x": 200.0,
                "size_y": 200.0,
                "size_z": 200.0,
                "radius": 100.0,
                "height": 200.0,
            },
            lambda: self.workspace_tcp_zones_changed.emit(self.get_workspace_tcp_zones()),
        )
        self.workspace_tcp_zones_changed.emit(self.get_workspace_tcp_zones())

    def _on_remove_tcp_zone_clicked(self) -> None:
        self._remove_primitive_row(
            self.table_tcp_zones,
            self._tcp_enabled_checkboxes,
            self._tcp_type_combos,
        )
        self.workspace_tcp_zones_changed.emit(self.get_workspace_tcp_zones())

    def _on_add_collision_zone_clicked(self) -> None:
        self._insert_primitive_row(
            self.table_collision_zones,
            self._collision_enabled_checkboxes,
            self._collision_type_combos,
            {
                "name": f"Zone collision {self.table_collision_zones.rowCount() + 1 if self.table_collision_zones is not None else 1}",
                "enabled": True,
                "shape": "box",
                "pose": [0.0] * 6,
                "size_x": 200.0,
                "size_y": 200.0,
                "size_z": 200.0,
                "radius": 100.0,
                "height": 200.0,
            },
            lambda: self.workspace_collision_zones_changed.emit(self.get_workspace_collision_zones()),
        )
        self.workspace_collision_zones_changed.emit(self.get_workspace_collision_zones())

    def _on_remove_collision_zone_clicked(self) -> None:
        self._remove_primitive_row(
            self.table_collision_zones,
            self._collision_enabled_checkboxes,
            self._collision_type_combos,
        )
        self.workspace_collision_zones_changed.emit(self.get_workspace_collision_zones())

    def _insert_primitive_row(
        self,
        table: QTableWidget | None,
        enabled_checkboxes: list[QCheckBox],
        type_combos: list[QComboBox],
        value: dict[str, Any],
        on_value_changed,
    ) -> None:
        if table is None:
            return

        row = table.rowCount()
        table.insertRow(row)

        enabled_checkbox = QCheckBox()
        enabled_checkbox.setChecked(bool(value.get("enabled", True)))
        enabled_checkbox.stateChanged.connect(lambda _state: on_value_changed())
        table.setCellWidget(row, WorkspaceConfigurationWidget.COL_PRIM_ENABLED, enabled_checkbox)
        enabled_checkboxes.append(enabled_checkbox)

        type_combo = QComboBox()
        type_combo.addItems(["box", "cylinder", "sphere"])
        shape_value = str(value.get("shape", "box")).strip().lower()
        type_combo.setCurrentText(shape_value if shape_value in {"box", "cylinder", "sphere"} else "box")
        type_combo.currentIndexChanged.connect(lambda _idx: on_value_changed())
        table.setCellWidget(row, WorkspaceConfigurationWidget.COL_PRIM_TYPE, type_combo)
        type_combos.append(type_combo)

        pose = value.get("pose", [0.0] * 6)
        cells = {
            WorkspaceConfigurationWidget.COL_PRIM_NAME: str(value.get("name", f"Zone {row + 1}")),
            WorkspaceConfigurationWidget.COL_PRIM_X: str(float(pose[0] if len(pose) > 0 else 0.0)),
            WorkspaceConfigurationWidget.COL_PRIM_Y: str(float(pose[1] if len(pose) > 1 else 0.0)),
            WorkspaceConfigurationWidget.COL_PRIM_Z: str(float(pose[2] if len(pose) > 2 else 0.0)),
            WorkspaceConfigurationWidget.COL_PRIM_A: str(float(pose[3] if len(pose) > 3 else 0.0)),
            WorkspaceConfigurationWidget.COL_PRIM_B: str(float(pose[4] if len(pose) > 4 else 0.0)),
            WorkspaceConfigurationWidget.COL_PRIM_C: str(float(pose[5] if len(pose) > 5 else 0.0)),
            WorkspaceConfigurationWidget.COL_PRIM_SIZE_X: str(float(value.get("size_x", 200.0))),
            WorkspaceConfigurationWidget.COL_PRIM_SIZE_Y: str(float(value.get("size_y", 200.0))),
            WorkspaceConfigurationWidget.COL_PRIM_SIZE_Z: str(float(value.get("size_z", 200.0))),
            WorkspaceConfigurationWidget.COL_PRIM_RADIUS: str(float(value.get("radius", 100.0))),
            WorkspaceConfigurationWidget.COL_PRIM_HEIGHT: str(float(value.get("height", 200.0))),
        }
        for col, text in cells.items():
            table.setItem(row, col, QTableWidgetItem(text))

    @staticmethod
    def _remove_primitive_row(
        table: QTableWidget | None,
        enabled_checkboxes: list[QCheckBox],
        type_combos: list[QComboBox],
    ) -> None:
        if table is None:
            return
        row = table.currentRow()
        if row < 0:
            row = table.rowCount() - 1
        if row < 0:
            return
        table.removeRow(row)
        if 0 <= row < len(enabled_checkboxes):
            enabled_checkboxes.pop(row)
        if 0 <= row < len(type_combos):
            type_combos.pop(row)

    def _update_element_status_row(self, row: int) -> None:
        if self.table_elements is None or not (0 <= row < self.table_elements.rowCount()):
            return

        stl_item = self.table_elements.item(row, WorkspaceConfigurationWidget.COL_ELEM_STL)
        stl_path = stl_item.text().strip() if stl_item is not None else ""
        status_item = self.table_elements.item(row, WorkspaceConfigurationWidget.COL_ELEM_STATUS)
        if status_item is None:
            status_item = QTableWidgetItem("")
            status_item.setFlags(status_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_STATUS, status_item)

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

    @staticmethod
    def _safe_float(value: str, default: float = 0.0) -> float:
        try:
            stripped = str(value).strip()
            return default if stripped == "" else float(stripped)
        except (TypeError, ValueError):
            return default

    def _cell_to_float(self, table: QTableWidget, row: int, col: int, default: float = 0.0) -> float:
        item = table.item(row, col)
        return self._safe_float(item.text() if item else "", default)

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

    def _refresh_scene_name_tooltip(self) -> None:
        if self.scene_name_line_edit is None:
            return
        tooltip_path = self._workspace_file_path if self._workspace_file_path else self._workspace_directory
        self.scene_name_line_edit.setToolTip(tooltip_path)

    def set_workspace_cad_elements(self, values: list[dict[str, Any]]) -> None:
        if self.table_elements is None:
            return
        normalized = parse_workspace_cad_elements(values)
        self.table_elements.blockSignals(True)
        try:
            self.table_elements.setRowCount(0)
            for row, value in enumerate(normalized):
                self.table_elements.insertRow(row)
                pose = value.get("pose", [0.0] * 6)
                self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_NAME, QTableWidgetItem(str(value.get("name", f"Element {row + 1}"))))
                self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_STL, QTableWidgetItem(str(value.get("cad_model", ""))))
                self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_X, QTableWidgetItem(str(float(pose[0] if len(pose) > 0 else 0.0))))
                self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_Y, QTableWidgetItem(str(float(pose[1] if len(pose) > 1 else 0.0))))
                self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_Z, QTableWidgetItem(str(float(pose[2] if len(pose) > 2 else 0.0))))
                self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_A, QTableWidgetItem(str(float(pose[3] if len(pose) > 3 else 0.0))))
                self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_B, QTableWidgetItem(str(float(pose[4] if len(pose) > 4 else 0.0))))
                self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_C, QTableWidgetItem(str(float(pose[5] if len(pose) > 5 else 0.0))))
                status_item = QTableWidgetItem("")
                status_item.setFlags(status_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                self.table_elements.setItem(row, WorkspaceConfigurationWidget.COL_ELEM_STATUS, status_item)
                self._update_element_status_row(row)
        finally:
            self.table_elements.blockSignals(False)

    def get_workspace_cad_elements(self) -> list[dict[str, Any]]:
        if self.table_elements is None:
            return []

        values: list[dict[str, Any]] = []
        for row in range(self.table_elements.rowCount()):
            name_item = self.table_elements.item(row, WorkspaceConfigurationWidget.COL_ELEM_NAME)
            stl_item = self.table_elements.item(row, WorkspaceConfigurationWidget.COL_ELEM_STL)
            values.append(
                {
                    "name": name_item.text().strip() if name_item is not None else f"Element {row + 1}",
                    "cad_model": stl_item.text().strip() if stl_item is not None else "",
                    "pose": [
                        self._cell_to_float(self.table_elements, row, WorkspaceConfigurationWidget.COL_ELEM_X, 0.0),
                        self._cell_to_float(self.table_elements, row, WorkspaceConfigurationWidget.COL_ELEM_Y, 0.0),
                        self._cell_to_float(self.table_elements, row, WorkspaceConfigurationWidget.COL_ELEM_Z, 0.0),
                        self._cell_to_float(self.table_elements, row, WorkspaceConfigurationWidget.COL_ELEM_A, 0.0),
                        self._cell_to_float(self.table_elements, row, WorkspaceConfigurationWidget.COL_ELEM_B, 0.0),
                        self._cell_to_float(self.table_elements, row, WorkspaceConfigurationWidget.COL_ELEM_C, 0.0),
                    ],
                }
            )
        return parse_workspace_cad_elements(values)

    def _set_primitives_table(
        self,
        table: QTableWidget | None,
        enabled_checkboxes: list[QCheckBox],
        type_combos: list[QComboBox],
        values: list[dict[str, Any]],
        on_value_changed,
    ) -> None:
        if table is None:
            return
        normalized = parse_primitive_colliders(values, default_shape="box")
        table.blockSignals(True)
        try:
            table.setRowCount(0)
            enabled_checkboxes.clear()
            type_combos.clear()
            for value in normalized:
                self._insert_primitive_row(table, enabled_checkboxes, type_combos, value, on_value_changed)
        finally:
            table.blockSignals(False)

    def _get_primitives_from_table(self, table: QTableWidget | None) -> list[dict[str, Any]]:
        if table is None:
            return []

        values: list[dict[str, Any]] = []
        for row in range(table.rowCount()):
            enabled_widget = table.cellWidget(row, WorkspaceConfigurationWidget.COL_PRIM_ENABLED)
            enabled = bool(enabled_widget.isChecked()) if isinstance(enabled_widget, QCheckBox) else True

            shape_widget = table.cellWidget(row, WorkspaceConfigurationWidget.COL_PRIM_TYPE)
            shape = str(shape_widget.currentText()).strip().lower() if isinstance(shape_widget, QComboBox) else "box"

            name_item = table.item(row, WorkspaceConfigurationWidget.COL_PRIM_NAME)
            name = name_item.text().strip() if name_item is not None else f"Zone {row + 1}"
            if name == "":
                name = f"Zone {row + 1}"

            values.append(
                {
                    "name": name,
                    "enabled": enabled,
                    "shape": shape,
                    "pose": [
                        self._cell_to_float(table, row, WorkspaceConfigurationWidget.COL_PRIM_X, 0.0),
                        self._cell_to_float(table, row, WorkspaceConfigurationWidget.COL_PRIM_Y, 0.0),
                        self._cell_to_float(table, row, WorkspaceConfigurationWidget.COL_PRIM_Z, 0.0),
                        self._cell_to_float(table, row, WorkspaceConfigurationWidget.COL_PRIM_A, 0.0),
                        self._cell_to_float(table, row, WorkspaceConfigurationWidget.COL_PRIM_B, 0.0),
                        self._cell_to_float(table, row, WorkspaceConfigurationWidget.COL_PRIM_C, 0.0),
                    ],
                    "size_x": max(0.0, self._cell_to_float(table, row, WorkspaceConfigurationWidget.COL_PRIM_SIZE_X, 200.0)),
                    "size_y": max(0.0, self._cell_to_float(table, row, WorkspaceConfigurationWidget.COL_PRIM_SIZE_Y, 200.0)),
                    "size_z": max(0.0, self._cell_to_float(table, row, WorkspaceConfigurationWidget.COL_PRIM_SIZE_Z, 200.0)),
                    "radius": max(0.0, self._cell_to_float(table, row, WorkspaceConfigurationWidget.COL_PRIM_RADIUS, 100.0)),
                    "height": max(0.0, self._cell_to_float(table, row, WorkspaceConfigurationWidget.COL_PRIM_HEIGHT, 200.0)),
                }
            )
        return parse_primitive_colliders(values, default_shape="box")

    def set_workspace_tcp_zones(self, values: list[dict[str, Any]]) -> None:
        self._set_primitives_table(
            self.table_tcp_zones,
            self._tcp_enabled_checkboxes,
            self._tcp_type_combos,
            values,
            lambda: self.workspace_tcp_zones_changed.emit(self.get_workspace_tcp_zones()),
        )

    def get_workspace_tcp_zones(self) -> list[dict[str, Any]]:
        return self._get_primitives_from_table(self.table_tcp_zones)

    def set_workspace_collision_zones(self, values: list[dict[str, Any]]) -> None:
        self._set_primitives_table(
            self.table_collision_zones,
            self._collision_enabled_checkboxes,
            self._collision_type_combos,
            values,
            lambda: self.workspace_collision_zones_changed.emit(self.get_workspace_collision_zones()),
        )

    def get_workspace_collision_zones(self) -> list[dict[str, Any]]:
        return self._get_primitives_from_table(self.table_collision_zones)

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
        robot_stl_dir = os.path.join(current_dir, "robot_stl")
        if os.path.isdir(robot_stl_dir):
            return robot_stl_dir
        return current_dir

