from __future__ import annotations

import os

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor, QPalette
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
from utils.config_action_icons import (
    CONFIG_ACTION_BUTTON_SIZE,
    CONFIG_ACTION_ICON_SIZE,
    build_new_icon,
    build_save_icon,
)
from widgets.workspace_view.workspace_primitive_zones_editor_widget import WorkspacePrimitiveZonesEditorWidget


class WorkspaceConfigurationWidget(QWidget):
    scene_name_changed = pyqtSignal(str)
    robot_base_pose_world_changed = pyqtSignal(object)
    workspace_save_requested = pyqtSignal()
    workspace_save_as_requested = pyqtSignal()
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
        self.current_config_label: QLabel | None = None
        self.status_label: QLabel | None = None
        self._workspace_directory: str = ""
        self._workspace_file_path: str = ""
        self.robot_base_pose_spinboxes: list[QDoubleSpinBox] = []
        self.table_elements: QTableWidget | None = None
        self.tcp_zones_editor: WorkspacePrimitiveZonesEditorWidget | None = None
        self.collision_zones_editor: WorkspacePrimitiveZonesEditorWidget | None = None
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(6)
        layout.addLayout(self._build_header())
        layout.addWidget(self._build_base_offset_group())
        layout.addWidget(self._build_elements_group())

        self.tcp_zones_editor = WorkspacePrimitiveZonesEditorWidget(
            title="Zones de travail TCP (repère world)",
            add_button_text="Ajouter",
            remove_button_text="Supprimer",
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
            add_button_text="Ajouter",
            remove_button_text="Supprimer",
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

    def _build_header(self) -> QVBoxLayout:
        header_layout = QVBoxLayout()
        header_layout.setSpacing(6)

        title_row = QHBoxLayout()
        title_label = QLabel("Configuration scene")
        title_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        title_row.addWidget(title_label)
        title_row.addStretch()

        self.status_label = QLabel("Configuration non chargée")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.status_label.setStyleSheet("color: #808080; font-size: 13px; font-weight: 400;")
        title_row.addWidget(self.status_label)
        header_layout.addLayout(title_row)

        fields_layout = QGridLayout()
        fields_layout.setHorizontalSpacing(8)
        fields_layout.setVerticalSpacing(6)

        current_config_title_label = QLabel("Configuration courante :")
        current_config_title_label.setMinimumWidth(150)
        fields_layout.addWidget(current_config_title_label, 0, 0)

        self.current_config_label = QLabel("Aucune configuration")
        self.current_config_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self.current_config_label.setMinimumWidth(220)
        self._apply_current_config_label_style()
        self.current_config_label.setFixedHeight(self.current_config_label.sizeHint().height())
        fields_layout.addWidget(self.current_config_label, 0, 1, Qt.AlignmentFlag.AlignVCenter)

        scene_name_title_label = QLabel("Nom scène :")
        scene_name_title_label.setMinimumWidth(150)
        fields_layout.addWidget(scene_name_title_label, 1, 0)

        self.scene_name_line_edit = QLineEdit()
        self.scene_name_line_edit.setPlaceholderText("Nom de scène workspace")
        self.scene_name_line_edit.textChanged.connect(self.scene_name_changed.emit)
        self.scene_name_line_edit.setMinimumWidth(220)
        fields_layout.addWidget(self.scene_name_line_edit, 1, 1)

        fields_layout.setColumnStretch(0, 0)
        fields_layout.setColumnStretch(1, 1)

        action_row = QHBoxLayout()
        action_row.addStretch()
        load_button = QPushButton("...")
        load_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        load_button.setToolTip("Charger une configuration scene")
        load_button.clicked.connect(self.workspace_load_requested.emit)
        action_row.addWidget(load_button)

        new_button = QPushButton()
        new_button.setIcon(build_new_icon(self.palette()))
        new_button.setIconSize(CONFIG_ACTION_ICON_SIZE)
        new_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        new_button.setToolTip("Créer une nouvelle configuration scene")
        new_button.clicked.connect(self.workspace_clear_requested.emit)
        action_row.addWidget(new_button)

        save_button = QPushButton()
        save_button.setIcon(build_save_icon(self.palette()))
        save_button.setIconSize(CONFIG_ACTION_ICON_SIZE)
        save_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        save_button.setToolTip("Enregistrer la configuration scene courante")
        save_button.clicked.connect(self.workspace_save_requested.emit)
        action_row.addWidget(save_button)

        save_as_button = QPushButton()
        save_as_button.setIcon(build_save_icon(self.palette(), include_pencil=True))
        save_as_button.setIconSize(CONFIG_ACTION_ICON_SIZE)
        save_as_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        save_as_button.setToolTip("Enregistrer la configuration scene dans un nouveau fichier JSON")
        save_as_button.clicked.connect(self.workspace_save_as_requested.emit)
        action_row.addWidget(save_as_button)

        fields_layout.addLayout(action_row, 2, 0, 1, 2)
        header_layout.addLayout(fields_layout)
        return header_layout

    def _build_base_offset_group(self) -> QGroupBox:
        group = QGroupBox("Décalage de base")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("Base robot dans world"), 0, 0)
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
                spinbox.setKeyboardTracking(False)
                spinbox.valueChanged.connect(self._on_robot_base_pose_world_value_changed)
                self.robot_base_pose_spinboxes.append(spinbox)
                pose_row.addWidget(spinbox)
            pose_row.addStretch()
            pose_layout.addLayout(pose_row)
        layout.addLayout(pose_layout, 0, 1)
        layout.setColumnStretch(1, 1)

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
        add_btn = QPushButton("Ajouter")
        add_btn.clicked.connect(self._on_add_element_clicked)
        btn_row.addWidget(add_btn)

        remove_btn = QPushButton("Supprimer")
        remove_btn.clicked.connect(self._on_remove_element_clicked)
        btn_row.addWidget(remove_btn)

        browse_btn = QPushButton("Parcourir")
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

    def set_configuration_status(self, text: str, color: str = "#808080") -> None:
        if self.status_label is None:
            return
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {color}; font-size: 13px; font-weight: 400;")

    def set_current_configuration_name(self, configuration_name: str) -> None:
        if self.current_config_label is None:
            return
        name = str(configuration_name or "").strip()
        self.current_config_label.setText(name or "Aucune configuration")

    def changeEvent(self, event) -> None:  # type: ignore[override]
        super().changeEvent(event)
        if event.type() == event.Type.PaletteChange:
            self._apply_current_config_label_style()

    def _apply_current_config_label_style(self) -> None:
        if self.current_config_label is None:
            return
        accent = self.palette().color(QPalette.ColorRole.Highlight).name()
        self.current_config_label.setStyleSheet(
            f"border: 1px solid #555; padding: 2px; background-color: #2a2a2a; color: {accent};"
        )

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
