from __future__ import annotations

import math
import os
from typing import Any

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QFileDialog,
    QGroupBox,
    QGridLayout,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QDoubleSpinBox,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
    QCheckBox,
)
from models.collider_models import (
    axis_colliders_to_dict,
    default_axis_colliders,
    parse_primitive_colliders,
)
from widgets.robot_view.tool_widget import ToolWidget
from utils.mgi import RobotTool
from models.tool_config_file import ToolConfigFile


class RobotConfigurationWidget(QWidget):
    """Widget principal de configuration robot (DH + axes + positions + CAD)."""

    load_config_requested = pyqtSignal()
    text_changed_requested = pyqtSignal()
    export_config_requested = pyqtSignal()

    dh_value_changed = pyqtSignal(int, int, str)
    tool_changed = pyqtSignal(RobotTool)
    axis_colliders_config_changed = pyqtSignal(list)

    axis_config_changed = pyqtSignal(list, list, list, list, list)
    positions_config_changed = pyqtSignal(list, list, list)
    position_zero_requested = pyqtSignal()
    position_transport_requested = pyqtSignal()
    home_position_requested = pyqtSignal()

    robot_cad_models_changed = pyqtSignal(list)
    tool_cad_model_changed = pyqtSignal(str)
    tool_cad_offset_rz_changed = pyqtSignal(float)
    tool_colliders_changed = pyqtSignal(list)
    tool_profiles_directory_changed = pyqtSignal(str)
    selected_tool_profile_changed = pyqtSignal(str)

    COL_AXIS_MIN = 0
    COL_AXIS_MAX = 1
    COL_AXIS_SPEED = 2
    COL_AXIS_ACCEL_EST = 3
    COL_AXIS_JERK = 4
    COL_AXIS_REVERSED = 5

    COL_POS_ZERO = 0
    COL_POS_TRANSPORT = 1
    COL_POS_HOME = 2

    COL_AXIS_COLLIDER_ENABLED = 0
    COL_AXIS_COLLIDER_DIRECTION = 1
    COL_AXIS_COLLIDER_RADIUS = 2
    COL_AXIS_COLLIDER_HEIGHT = 3
    COL_AXIS_COLLIDER_OFFSET_AXIS = 4
    COL_AXIS_COLLIDER_OFFSET_VALUE = 5

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

    AXIS_COLLIDER_COUNT = 6
    ROBOT_CAD_COUNT = 7

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.axis_reversed_checkboxes: list[QCheckBox] = []
        self.axis_collider_enabled_checkboxes: list[QCheckBox] = []
        self.axis_collider_direction_combos: list[QComboBox] = []
        self.robot_cad_line_edits: list[QLineEdit] = []
        self.tool_cad_line_edit: QLineEdit | None = None
        self.tool_cad_offset_rz_spin: QDoubleSpinBox | None = None
        self.table_axis_colliders: QTableWidget | None = None
        self.table_cartesian_slider_limits: QTableWidget | None = None
        self.table_tool_colliders: QTableWidget | None = None
        self._tool_collider_type_combos: list[QComboBox] = []
        self._tool_collider_enabled_checkboxes: list[QCheckBox] = []
        self.tool_profiles_dir_line_edit: QLineEdit | None = None
        self.tool_profiles_combo: QComboBox | None = None
        self.tool_name_line_edit: QLineEdit | None = None
        self._tool_profile_loading = False
        self._tool_profile_files: dict[str, str] = {}
        self.setup_ui()

    def setup_ui(self) -> None:
        main_layout = QVBoxLayout(self)

        top_layout = QVBoxLayout()

        title = QLabel("Configuration robot")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        top_layout.addWidget(title)

        header_layout = QGridLayout()

        self.line_edit_robot_name = QLineEdit()
        self.line_edit_robot_name.setPlaceholderText("Nom du robot")
        self.line_edit_robot_name.textChanged.connect(self.text_changed_requested.emit)
        header_layout.addWidget(self.line_edit_robot_name, 0, 0)

        self.btn_load = QPushButton("Charger")
        self.btn_load.clicked.connect(self.load_config_requested.emit)
        header_layout.addWidget(self.btn_load, 0, 1)

        self.btn_export = QPushButton("Exporter")
        self.btn_export.clicked.connect(self.export_config_requested.emit)
        header_layout.addWidget(self.btn_export, 0, 2)

        top_layout.addLayout(header_layout)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_dh_tab(), "DH Table")
        self.tabs.addTab(self._build_axis_tab(), "Parametrage des axes")
        self.tabs.addTab(self._build_axis_colliders_tab(), "Colliders axes")
        self.tabs.addTab(self._build_positions_tab(), "Parametrage des positions")
        self.tabs.addTab(self._build_cad_tab(), "CAO")
        top_layout.addWidget(self.tabs)

        main_layout.addLayout(top_layout, 3)
        main_layout.addWidget(self._build_tool_section(), 2)

        self.tool_widget = ToolWidget()
        self.tool_widget.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred)
        self.tool_widget.tool_changed.connect(self._on_tool_changed)
        self.tool_widget_container_layout.addWidget(self.tool_widget)
        self.set_tool_profiles_directory(self._default_tools_directory(), emit_change=False)
        self.set_tool_colliders([])
        self.set_axis_colliders(default_axis_colliders(RobotConfigurationWidget.AXIS_COLLIDER_COUNT))

    def _build_dh_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        title = QLabel("Table de Denavit-Hartenberg")
        layout.addWidget(title)

        self.table_dh = QTableWidget(6, 4)
        self.table_dh.setHorizontalHeaderLabels(["alpha (deg)", "d (mm)", "theta (deg)", "r (mm)"])
        self.table_dh.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_dh.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh.horizontalHeader().setDefaultSectionSize(90)
        self.table_dh.cellChanged.connect(self._on_dh_cell_changed)
        layout.addWidget(self.table_dh)

        return tab

    def _build_axis_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        self.table_axis = QTableWidget(6, 6)
        self.table_axis.setHorizontalHeaderLabels(
            [
                "Min (deg)",
                "Max (deg)",
                "Vitesse max (deg/s)",
                "Accel estimee (deg/s^2)",
                "Jerk max (deg/s^3)",
                "Inverse",
            ]
        )
        self.table_axis.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_axis.horizontalHeader().setDefaultSectionSize(135)

        self.axis_reversed_checkboxes.clear()
        for row in range(6):
            accel_item = QTableWidgetItem("")
            accel_item.setFlags(accel_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table_axis.setItem(row, RobotConfigurationWidget.COL_AXIS_ACCEL_EST, accel_item)

            checkbox = QCheckBox()
            checkbox.stateChanged.connect(self._emit_axis_config_changed)
            self.table_axis.setCellWidget(row, RobotConfigurationWidget.COL_AXIS_REVERSED, checkbox)
            self.axis_reversed_checkboxes.append(checkbox)

        self.table_axis.itemChanged.connect(self._on_axis_item_changed)
        layout.addWidget(self.table_axis)

        cartesian_group = QGroupBox("Plages des sliders cartesiens")
        cartesian_layout = QVBoxLayout(cartesian_group)
        cartesian_layout.addWidget(QLabel("Bornes X/Y/Z min/max du controle cartesien."))
        self.table_cartesian_slider_limits = QTableWidget(3, 2)
        self.table_cartesian_slider_limits.setHorizontalHeaderLabels(["Min (mm)", "Max (mm)"])
        self.table_cartesian_slider_limits.setVerticalHeaderLabels(["X", "Y", "Z"])
        self.table_cartesian_slider_limits.horizontalHeader().setDefaultSectionSize(120)
        self.table_cartesian_slider_limits.itemChanged.connect(self._on_cartesian_slider_limits_item_changed)
        cartesian_layout.addWidget(self.table_cartesian_slider_limits)
        layout.addWidget(cartesian_group)

        return tab

    def _build_axis_colliders_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        self.table_axis_colliders = QTableWidget(RobotConfigurationWidget.AXIS_COLLIDER_COUNT, 6)
        self.table_axis_colliders.setHorizontalHeaderLabels(
            [
                "Actif",
                "Axe cylindre",
                "Rayon (mm)",
                "Hauteur (mm)",
                "Axe decalage",
                "Decalage (mm)",
            ]
        )
        vertical_labels = [f"q{i + 1}" for i in range(6)]
        self.table_axis_colliders.setVerticalHeaderLabels(vertical_labels)
        self.table_axis_colliders.horizontalHeader().setDefaultSectionSize(120)

        self.axis_collider_enabled_checkboxes.clear()
        self.axis_collider_direction_combos.clear()
        for row in range(RobotConfigurationWidget.AXIS_COLLIDER_COUNT):
            checkbox = QCheckBox()
            checkbox.stateChanged.connect(self._emit_axis_colliders_config_changed)
            self.table_axis_colliders.setCellWidget(row, RobotConfigurationWidget.COL_AXIS_COLLIDER_ENABLED, checkbox)
            self.axis_collider_enabled_checkboxes.append(checkbox)

            direction_combo = QComboBox()
            direction_combo.addItems(["X", "Y", "Z"])
            direction_combo.setCurrentText("Z")
            direction_combo.currentIndexChanged.connect(self._emit_axis_colliders_config_changed)
            self.table_axis_colliders.setCellWidget(
                row,
                RobotConfigurationWidget.COL_AXIS_COLLIDER_DIRECTION,
                direction_combo,
            )
            self.axis_collider_direction_combos.append(direction_combo)

            offset_axis_combo = QComboBox()
            offset_axis_combo.addItem("Aucun", "")
            offset_axis_combo.addItem("X", "x")
            offset_axis_combo.addItem("Y", "y")
            offset_axis_combo.addItem("Z", "z")
            offset_axis_combo.currentIndexChanged.connect(self._emit_axis_colliders_config_changed)
            self.table_axis_colliders.setCellWidget(row, RobotConfigurationWidget.COL_AXIS_COLLIDER_OFFSET_AXIS, offset_axis_combo)

        self.table_axis_colliders.itemChanged.connect(self._on_axis_colliders_item_changed)
        layout.addWidget(self.table_axis_colliders)
        return tab

    def _build_positions_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        self.table_positions = QTableWidget(6, 3)
        self.table_positions.setHorizontalHeaderLabels(
            [
                "Position 0 (deg)",
                "Position transport (deg)",
                "Position home (deg)",
            ]
        )
        self.table_positions.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_positions.horizontalHeader().setDefaultSectionSize(180)
        self.table_positions.itemChanged.connect(self._on_positions_item_changed)
        layout.addWidget(self.table_positions)

        positions_btn_layout = QHBoxLayout()
        self.btn_go_position_zero = QPushButton("Aller Position 0")
        self.btn_go_position_zero.clicked.connect(self.position_zero_requested.emit)
        positions_btn_layout.addWidget(self.btn_go_position_zero)

        self.btn_go_position_transport = QPushButton("Aller Position transport")
        self.btn_go_position_transport.clicked.connect(self.position_transport_requested.emit)
        positions_btn_layout.addWidget(self.btn_go_position_transport)

        self.btn_go_home_position = QPushButton("Aller Position home")
        self.btn_go_home_position.clicked.connect(self.home_position_requested.emit)
        positions_btn_layout.addWidget(self.btn_go_home_position)

        layout.addLayout(positions_btn_layout)

        return tab

    def _build_cad_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        description = QLabel("Selection des fichiers STL pour chaque lien robot.")
        description.setWordWrap(True)
        layout.addWidget(description)

        multi_select_layout = QHBoxLayout()
        multi_select_layout.addStretch()
        multi_select_button = QPushButton("Parcourir plusieurs CAO robot")
        multi_select_button.clicked.connect(self._on_pick_multiple_robot_cad)
        multi_select_layout.addWidget(multi_select_button)
        layout.addLayout(multi_select_layout)

        grid = QGridLayout()
        self.robot_cad_line_edits.clear()

        for index in range(RobotConfigurationWidget.ROBOT_CAD_COUNT):
            row = index
            label = QLabel(f"Lien {index}")
            path_line = QLineEdit()
            path_line.setReadOnly(True)

            browse_button = QPushButton("Parcourir")
            browse_button.clicked.connect(lambda _, i=index: self._on_pick_robot_cad(i))

            clear_button = QPushButton("Vider")
            clear_button.clicked.connect(lambda _, i=index: self._on_clear_robot_cad(i))

            self.robot_cad_line_edits.append(path_line)

            grid.addWidget(label, row, 0)
            grid.addWidget(path_line, row, 1)
            grid.addWidget(browse_button, row, 2)
            grid.addWidget(clear_button, row, 3)

        layout.addLayout(grid)
        layout.addStretch()

        return tab

    def _build_tool_section(self) -> QGroupBox:
        group = QGroupBox("Configuration tool")
        layout = QVBoxLayout(group)

        description = QLabel("Definition du tool actif: Nom, XYZABC, CAO et offset visuel Rz.")
        description.setWordWrap(True)
        layout.addWidget(description)

        profiles_grid = QGridLayout()

        profiles_grid.addWidget(QLabel("Dossier tools"), 0, 0)
        self.tool_profiles_dir_line_edit = QLineEdit()
        self.tool_profiles_dir_line_edit.setReadOnly(True)
        profiles_grid.addWidget(self.tool_profiles_dir_line_edit, 0, 1)

        pick_tools_dir_btn = QPushButton("Sélectionner dossier")
        pick_tools_dir_btn.clicked.connect(self._on_pick_tool_profiles_directory)
        profiles_grid.addWidget(pick_tools_dir_btn, 0, 2)

        refresh_tools_btn = QPushButton("Rafraichir")
        refresh_tools_btn.clicked.connect(self._refresh_tool_profiles)
        profiles_grid.addWidget(refresh_tools_btn, 0, 3)

        profiles_grid.addWidget(QLabel("Tool"), 1, 0)
        self.tool_profiles_combo = QComboBox()
        self.tool_profiles_combo.currentIndexChanged.connect(self._on_selected_tool_profile_changed)
        profiles_grid.addWidget(self.tool_profiles_combo, 1, 1)

        save_tool_btn = QPushButton("Enregistrer tool")
        save_tool_btn.clicked.connect(self._on_save_tool_profile)
        profiles_grid.addWidget(save_tool_btn, 1, 2)

        self.tool_name_line_edit = QLineEdit()
        self.tool_name_line_edit.setPlaceholderText("Nom du tool")
        profiles_grid.addWidget(self.tool_name_line_edit, 1, 3)

        layout.addLayout(profiles_grid)

        self.tool_widget_container_layout = QVBoxLayout()
        layout.addLayout(self.tool_widget_container_layout)

        tool_cad_grid = QGridLayout()
        tool_cad_grid.addWidget(QLabel("CAO tool"), 0, 0)
        self.tool_cad_line_edit = QLineEdit()
        self.tool_cad_line_edit.setReadOnly(True)
        tool_cad_grid.addWidget(self.tool_cad_line_edit, 0, 1)

        tool_browse_button = QPushButton("Parcourir")
        tool_browse_button.clicked.connect(self._on_pick_tool_cad)
        tool_cad_grid.addWidget(tool_browse_button, 0, 2)

        tool_clear_button = QPushButton("Vider")
        tool_clear_button.clicked.connect(self._on_clear_tool_cad)
        tool_cad_grid.addWidget(tool_clear_button, 0, 3)

        tool_cad_grid.addWidget(QLabel("Offset Rz tool (deg)"), 1, 0)
        self.tool_cad_offset_rz_spin = QDoubleSpinBox()
        self.tool_cad_offset_rz_spin.setRange(-360.0, 360.0)
        self.tool_cad_offset_rz_spin.setDecimals(2)
        self.tool_cad_offset_rz_spin.setSingleStep(1.0)
        self.tool_cad_offset_rz_spin.valueChanged.connect(self.tool_cad_offset_rz_changed.emit)
        tool_cad_grid.addWidget(self.tool_cad_offset_rz_spin, 1, 1)

        layout.addLayout(tool_cad_grid)

        colliders_title = QLabel("Colliders tool (repere flange / axe 6)")
        colliders_title.setStyleSheet("font-weight: bold;")
        layout.addWidget(colliders_title)

        self.table_tool_colliders = QTableWidget(0, 14)
        self.table_tool_colliders.setHorizontalHeaderLabels(
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
        self.table_tool_colliders.horizontalHeader().setDefaultSectionSize(90)
        self.table_tool_colliders.itemChanged.connect(self._on_tool_colliders_item_changed)
        layout.addWidget(self.table_tool_colliders)

        colliders_btn_layout = QHBoxLayout()
        add_tool_collider_btn = QPushButton("Ajouter collider tool")
        add_tool_collider_btn.clicked.connect(self._on_add_tool_collider_clicked)
        colliders_btn_layout.addWidget(add_tool_collider_btn)

        remove_tool_collider_btn = QPushButton("Supprimer collider tool")
        remove_tool_collider_btn.clicked.connect(self._on_remove_tool_collider_clicked)
        colliders_btn_layout.addWidget(remove_tool_collider_btn)
        colliders_btn_layout.addStretch()
        layout.addLayout(colliders_btn_layout)

        return group

    def _on_dh_cell_changed(self, row: int, col: int) -> None:
        item = self.table_dh.item(row, col)
        if item is not None:
            self.dh_value_changed.emit(row, col, item.text())

    def _on_axis_item_changed(self, item: QTableWidgetItem) -> None:
        if item.column() in (RobotConfigurationWidget.COL_AXIS_SPEED, RobotConfigurationWidget.COL_AXIS_JERK):
            self._refresh_estimated_accel_for_row(item.row())
        self._emit_axis_config_changed()

    def _on_cartesian_slider_limits_item_changed(self, _item: QTableWidgetItem) -> None:
        self._emit_axis_config_changed()

    def _on_axis_colliders_item_changed(self, _item: QTableWidgetItem) -> None:
        self._emit_axis_colliders_config_changed()

    def _on_tool_colliders_item_changed(self, _item: QTableWidgetItem) -> None:
        self.tool_colliders_changed.emit(self.get_tool_colliders())

    def _on_add_tool_collider_clicked(self) -> None:
        self._insert_tool_collider_row(
            {
                "name": f"Tool collider {self.table_tool_colliders.rowCount() + 1 if self.table_tool_colliders is not None else 1}",
                "enabled": True,
                "shape": "cylinder",
                "pose": [0.0] * 6,
                "size_x": 100.0,
                "size_y": 100.0,
                "size_z": 100.0,
                "radius": 40.0,
                "height": 120.0,
            }
        )
        self.tool_colliders_changed.emit(self.get_tool_colliders())

    def _on_remove_tool_collider_clicked(self) -> None:
        if self.table_tool_colliders is None:
            return
        row = self.table_tool_colliders.currentRow()
        if row < 0:
            row = self.table_tool_colliders.rowCount() - 1
        if row < 0:
            return

        self.table_tool_colliders.blockSignals(True)
        try:
            self.table_tool_colliders.removeRow(row)
            if 0 <= row < len(self._tool_collider_enabled_checkboxes):
                self._tool_collider_enabled_checkboxes.pop(row)
            if 0 <= row < len(self._tool_collider_type_combos):
                self._tool_collider_type_combos.pop(row)
        finally:
            self.table_tool_colliders.blockSignals(False)

        self.tool_colliders_changed.emit(self.get_tool_colliders())

    def _insert_tool_collider_row(self, collider: dict[str, Any]) -> None:
        if self.table_tool_colliders is None:
            return

        row = self.table_tool_colliders.rowCount()
        self.table_tool_colliders.insertRow(row)

        enabled_checkbox = QCheckBox()
        enabled_checkbox.setChecked(bool(collider.get("enabled", True)))
        enabled_checkbox.stateChanged.connect(lambda _state: self.tool_colliders_changed.emit(self.get_tool_colliders()))
        self.table_tool_colliders.setCellWidget(row, RobotConfigurationWidget.COL_PRIM_ENABLED, enabled_checkbox)
        self._tool_collider_enabled_checkboxes.append(enabled_checkbox)

        type_combo = QComboBox()
        type_combo.addItems(["box", "cylinder", "sphere"])
        shape_value = str(collider.get("shape", "cylinder")).strip().lower()
        type_combo.setCurrentText(shape_value if shape_value in {"box", "cylinder", "sphere"} else "cylinder")
        type_combo.currentIndexChanged.connect(lambda _idx: self.tool_colliders_changed.emit(self.get_tool_colliders()))
        self.table_tool_colliders.setCellWidget(row, RobotConfigurationWidget.COL_PRIM_TYPE, type_combo)
        self._tool_collider_type_combos.append(type_combo)

        pose = collider.get("pose", [0.0] * 6)
        cells = {
            RobotConfigurationWidget.COL_PRIM_NAME: str(collider.get("name", f"Tool collider {row + 1}")),
            RobotConfigurationWidget.COL_PRIM_X: str(float(pose[0] if len(pose) > 0 else 0.0)),
            RobotConfigurationWidget.COL_PRIM_Y: str(float(pose[1] if len(pose) > 1 else 0.0)),
            RobotConfigurationWidget.COL_PRIM_Z: str(float(pose[2] if len(pose) > 2 else 0.0)),
            RobotConfigurationWidget.COL_PRIM_A: str(float(pose[3] if len(pose) > 3 else 0.0)),
            RobotConfigurationWidget.COL_PRIM_B: str(float(pose[4] if len(pose) > 4 else 0.0)),
            RobotConfigurationWidget.COL_PRIM_C: str(float(pose[5] if len(pose) > 5 else 0.0)),
            RobotConfigurationWidget.COL_PRIM_SIZE_X: str(float(collider.get("size_x", 100.0))),
            RobotConfigurationWidget.COL_PRIM_SIZE_Y: str(float(collider.get("size_y", 100.0))),
            RobotConfigurationWidget.COL_PRIM_SIZE_Z: str(float(collider.get("size_z", 100.0))),
            RobotConfigurationWidget.COL_PRIM_RADIUS: str(float(collider.get("radius", 40.0))),
            RobotConfigurationWidget.COL_PRIM_HEIGHT: str(float(collider.get("height", 120.0))),
        }
        for column, value in cells.items():
            self.table_tool_colliders.setItem(row, column, QTableWidgetItem(value))

    def _on_positions_item_changed(self, _item: QTableWidgetItem) -> None:
        self.positions_config_changed.emit(
            self.get_home_position(),
            self.get_position_zero(),
            self.get_position_transport(),
        )

    def _on_pick_robot_cad(self, index: int) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Sélectionner une CAO",
            self._get_cad_start_directory(),
            "STL files (*.stl);;All files (*)",
        )
        if not file_path:
            return

        self.robot_cad_line_edits[index].setText(self._normalize_cad_path(file_path))
        self.robot_cad_models_changed.emit(self.get_robot_cad_models())

    def _on_pick_multiple_robot_cad(self) -> None:
        file_paths, _ = QFileDialog.getOpenFileNames(
            self,
            "Sélectionner plusieurs CAO robot",
            self._get_cad_start_directory(),
            "STL files (*.stl);;All files (*)",
        )
        if not file_paths:
            return

        cad_paths = [self._normalize_cad_path(path) for path in file_paths]
        max_count = RobotConfigurationWidget.ROBOT_CAD_COUNT
        if len(cad_paths) > max_count:
            cad_paths = cad_paths[:max_count]
            QMessageBox.information(
                self,
                "Selection limitee",
                f"Seulement {max_count} fichiers sont utilises (indices 0 a {max_count - 1}).",
            )

        start_index = 0
        if len(cad_paths) < max_count:
            max_start = max_count - len(cad_paths)
            start_index, ok = QInputDialog.getInt(
                self,
                "Index de depart",
                (
                    f"{len(cad_paths)} fichiers selectionnes.\n"
                    f"Choisissez l'index de depart pour l'affectation ({0} a {max_start})."
                ),
                0,
                0,
                max_start,
                1,
            )
            if not ok:
                return

        for offset, cad_path in enumerate(cad_paths):
            target_index = start_index + offset
            if 0 <= target_index < max_count:
                self.robot_cad_line_edits[target_index].setText(cad_path)

        self.robot_cad_models_changed.emit(self.get_robot_cad_models())

    def _on_clear_robot_cad(self, index: int) -> None:
        self.robot_cad_line_edits[index].setText("")
        self.robot_cad_models_changed.emit(self.get_robot_cad_models())

    def _on_pick_tool_cad(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Sélectionner une CAO de tool",
            self._get_cad_start_directory(),
            "STL files (*.stl);;All files (*)",
        )
        if not file_path:
            return

        self.tool_cad_line_edit.setText(self._normalize_cad_path(file_path))
        self.tool_cad_model_changed.emit(self.get_tool_cad_model())

    def _on_clear_tool_cad(self) -> None:
        self.tool_cad_line_edit.setText("")
        self.tool_cad_model_changed.emit("")

    def _on_tool_changed(self, tool: RobotTool) -> None:
        self.tool_changed.emit(tool)

    def _on_pick_tool_profiles_directory(self) -> None:
        current_directory = self.get_tool_profiles_directory()
        start_directory = self._resolve_filesystem_path(current_directory) if current_directory else self._get_tools_start_directory()
        selected_dir = QFileDialog.getExistingDirectory(
            self,
            "Sélectionner le dossier des tools",
            start_directory,
        )
        if not selected_dir:
            return
        self.set_tool_profiles_directory(selected_dir, emit_change=True)

    def _on_selected_tool_profile_changed(self, _index: int) -> None:
        if self._tool_profile_loading or self.tool_profiles_combo is None:
            return
        file_path = self.tool_profiles_combo.currentData()
        if not file_path:
            self.selected_tool_profile_changed.emit("")
            return
        self.selected_tool_profile_changed.emit(self._normalize_project_path(str(file_path)))

    def _refresh_tool_profiles(self) -> None:
        if self.tool_profiles_combo is None or self.tool_profiles_dir_line_edit is None:
            return

        selected_data = self.tool_profiles_combo.currentData()
        tools_dir = self._resolve_filesystem_path(self.tool_profiles_dir_line_edit.text().strip())

        self._tool_profile_loading = True
        self._tool_profile_files.clear()
        self.tool_profiles_combo.clear()
        self.tool_profiles_combo.addItem("Aucun outil", "")

        if os.path.isdir(tools_dir):
            for file_name in sorted(os.listdir(tools_dir)):
                if not file_name.lower().endswith(".json"):
                    continue
                profile_name = os.path.splitext(file_name)[0]
                profile_path = os.path.join(tools_dir, file_name)
                self._tool_profile_files[profile_name] = profile_path
                self.tool_profiles_combo.addItem(profile_name, profile_path)

        if selected_data:
            found_index = self.tool_profiles_combo.findData(selected_data)
            self.tool_profiles_combo.setCurrentIndex(found_index if found_index >= 0 else 0)
        else:
            self.tool_profiles_combo.setCurrentIndex(0)

        self._tool_profile_loading = False

    def _load_tool_profile(self, file_path: str) -> bool:
        try:
            profile = ToolConfigFile.load(file_path)
        except (OSError, ValueError, TypeError) as exc:
            QMessageBox.warning(self, "Tool invalide", f"Impossible de charger {file_path}.\n{exc}")
            return False

        profile_name = profile.name if profile.name else os.path.splitext(os.path.basename(file_path))[0]
        if self.tool_name_line_edit is not None:
            self.tool_name_line_edit.setText(profile_name)

        loaded_tool = profile.to_robot_tool()
        self.tool_widget.set_tool(loaded_tool)
        self.tool_changed.emit(loaded_tool)

        self.set_tool_cad_model(profile.tool_cad_model)
        self.tool_cad_model_changed.emit(self.get_tool_cad_model())

        self.set_tool_cad_offset_rz(profile.tool_cad_offset_rz)
        self.tool_cad_offset_rz_changed.emit(profile.tool_cad_offset_rz)
        self.set_tool_colliders(profile.tool_colliders)
        self.tool_colliders_changed.emit(self.get_tool_colliders())
        return True

    def _on_save_tool_profile(self) -> None:
        if self.tool_profiles_dir_line_edit is None:
            return

        tools_dir = self._resolve_filesystem_path(self.tool_profiles_dir_line_edit.text().strip())
        if not tools_dir:
            QMessageBox.information(self, "Dossier manquant", "Selectionnez d'abord un dossier de tools.")
            return

        if not os.path.isdir(tools_dir):
            try:
                os.makedirs(tools_dir, exist_ok=True)
            except OSError as exc:
                QMessageBox.warning(self, "Dossier invalide", f"Impossible de creer le dossier:\n{tools_dir}\n{exc}")
                return

        raw_name = self.tool_name_line_edit.text().strip() if self.tool_name_line_edit is not None else ""
        if not raw_name:
            QMessageBox.information(self, "Nom manquant", "Saisissez un nom de tool avant d'enregistrer.")
            return

        if self._has_forbidden_filename_chars(raw_name):
            QMessageBox.warning(
                self,
                "Nom invalide",
                "Le nom du tool contient des caracteres interdits pour un nom de fichier.",
            )
            return

        safe_name = self._sanitize_tool_file_name(raw_name)
        if not safe_name:
            QMessageBox.warning(self, "Nom invalide", "Le nom du tool ne peut pas etre utilise comme nom de fichier.")
            return

        output_path = os.path.join(tools_dir, f"{safe_name}.json")
        existing_profile_path = ""
        if self.tool_profiles_combo is not None and self.tool_profiles_combo.currentData():
            existing_profile_path = str(self.tool_profiles_combo.currentData())

        if os.path.exists(output_path):
            same_path = (
                existing_profile_path
                and os.path.normcase(os.path.abspath(existing_profile_path))
                == os.path.normcase(os.path.abspath(output_path))
            )
            if not same_path:
                QMessageBox.warning(
                    self,
                    "Nom deja utilise",
                    f"Un tool existe deja avec ce nom dans le dossier:\n{output_path}",
                )
                return

        profile = ToolConfigFile.from_robot_tool(
            raw_name,
            self.get_tool(),
            self.get_tool_cad_model(),
            self.get_tool_cad_offset_rz(),
            self.get_tool_colliders(),
        )
        try:
            profile.save(output_path)
        except (OSError, ValueError, TypeError) as exc:
            QMessageBox.warning(self, "Erreur sauvegarde", f"Impossible d'enregistrer {output_path}.\n{exc}")
            return

        self._refresh_tool_profiles()
        if self.tool_profiles_combo is not None:
            match_index = self.tool_profiles_combo.findData(output_path)
            if match_index >= 0:
                self.tool_profiles_combo.setCurrentIndex(match_index)

    def _apply_no_tool_profile(self, emit_signals: bool = True) -> None:
        if self.tool_name_line_edit is not None:
            self.tool_name_line_edit.setText("Aucun outil")

        no_tool = RobotTool()
        self.tool_widget.set_tool(no_tool)
        if emit_signals:
            self.tool_changed.emit(no_tool)
        self.set_tool_cad_model("")
        if emit_signals:
            self.tool_cad_model_changed.emit("")
        self.set_tool_cad_offset_rz(0.0)
        if emit_signals:
            self.tool_cad_offset_rz_changed.emit(0.0)
        self.set_tool_colliders([])
        if emit_signals:
            self.tool_colliders_changed.emit([])

    @staticmethod
    def _safe_float(value: str, default: float = 0.0) -> float:
        try:
            if value is None:
                return default
            stripped = str(value).strip()
            if stripped == "":
                return default
            return float(stripped)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _set_combo_by_data(combo: QComboBox | None, target_data: str, fallback_data: str = "") -> None:
        if combo is None:
            return
        index = combo.findData(target_data)
        if index < 0:
            index = combo.findData(fallback_data)
        if index < 0:
            index = 0
        combo.setCurrentIndex(index)

    def _cell_to_float(self, table: QTableWidget, row: int, column: int, default: float = 0.0) -> float:
        item = table.item(row, column)
        return self._safe_float(item.text() if item else "", default)

    def _refresh_estimated_accel_for_row(self, row: int) -> None:
        speed = max(0.0, self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_SPEED, 0.0))
        jerk = max(0.0, self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_JERK, 0.0))
        accel = math.sqrt(speed * jerk)

        accel_item = self.table_axis.item(row, RobotConfigurationWidget.COL_AXIS_ACCEL_EST)
        if accel_item is None:
            accel_item = QTableWidgetItem("")
            accel_item.setFlags(accel_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table_axis.setItem(row, RobotConfigurationWidget.COL_AXIS_ACCEL_EST, accel_item)
        accel_item.setText(f"{accel:.3f}")

    def _refresh_estimated_accel_column(self) -> None:
        self.table_axis.blockSignals(True)
        try:
            for row in range(6):
                self._refresh_estimated_accel_for_row(row)
        finally:
            self.table_axis.blockSignals(False)

    @staticmethod
    def _get_cad_start_directory() -> str:
        current_dir = os.getcwd()
        robots_stl_dir = os.path.join(current_dir, "default", "robots_stl")
        if os.path.isdir(robots_stl_dir):
            return robots_stl_dir
        return current_dir

    @staticmethod
    def _get_tools_start_directory() -> str:
        current_dir = os.getcwd()
        tools_dir = os.path.join(current_dir, "configurations", "tools")
        os.makedirs(tools_dir, exist_ok=True)
        if os.path.isdir(tools_dir):
            return tools_dir
        tools_dir = os.path.join(current_dir, "tools")
        if os.path.isdir(tools_dir):
            return tools_dir
        return current_dir

    @staticmethod
    def _sanitize_tool_file_name(name: str) -> str:
        forbidden = '<>:"/\\|?*'
        safe = name.replace(" ", "_")
        safe = "".join("_" if char in forbidden else char for char in safe).strip().strip(".")
        return safe

    @staticmethod
    def _has_forbidden_filename_chars(name: str) -> bool:
        forbidden = '<>:"/\\|?*'
        return any(char in forbidden for char in str(name))

    @staticmethod
    def _resolve_filesystem_path(path: str) -> str:
        if not path:
            return ""
        return os.path.abspath(path)

    @staticmethod
    def _default_tools_directory() -> str:
        return "./configurations/tools"

    @staticmethod
    def _normalize_cad_path(file_path: str) -> str:
        return RobotConfigurationWidget._normalize_project_path(file_path)

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

    def _emit_axis_config_changed(self) -> None:
        self.axis_config_changed.emit(
            self.get_axis_limits(),
            self.get_cartesian_slider_limits_xyz(),
            self.get_axis_speed_limits(),
            self.get_axis_jerk_limits(),
            self.get_axis_reversed(),
        )

    def _emit_axis_colliders_config_changed(self) -> None:
        self.axis_colliders_config_changed.emit(self.get_axis_colliders())

    def set_robot_name(self, name: str) -> None:
        self.line_edit_robot_name.setText(name)

    def get_robot_name(self) -> str:
        return self.line_edit_robot_name.text()

    def set_dh_params(self, params: list[list[float]]) -> None:
        self.table_dh.blockSignals(True)
        try:
            for row in range(6):
                values = params[row] if row < len(params) else []
                for col in range(4):
                    value = str(values[col]) if col < len(values) else ""
                    self.table_dh.setItem(row, col, QTableWidgetItem(value))
        finally:
            self.table_dh.blockSignals(False)

    def get_dh_params(self) -> list[list[str]]:
        params: list[list[str]] = []
        for row in range(6):
            row_values: list[str] = []
            for col in range(4):
                item = self.table_dh.item(row, col)
                row_values.append(item.text() if item else "")
            params.append(row_values)
        return params

    def set_axis_config(
        self,
        axis_limits: list[tuple[float, float]],
        cartesian_slider_limits_xyz: list[tuple[float, float]],
        axis_speed_limits: list[float],
        axis_jerk_limits: list[float],
        axis_reversed: list[int],
    ) -> None:
        self.table_axis.blockSignals(True)
        try:
            for row in range(6):
                min_val = axis_limits[row][0] if row < len(axis_limits) else -180.0
                max_val = axis_limits[row][1] if row < len(axis_limits) else 180.0
                speed = axis_speed_limits[row] if row < len(axis_speed_limits) else 0.0
                jerk = axis_jerk_limits[row] if row < len(axis_jerk_limits) else 0.0
                reversed_axis = axis_reversed[row] if row < len(axis_reversed) else 1

                self.table_axis.setItem(row, RobotConfigurationWidget.COL_AXIS_MIN, QTableWidgetItem(str(min_val)))
                self.table_axis.setItem(row, RobotConfigurationWidget.COL_AXIS_MAX, QTableWidgetItem(str(max_val)))
                self.table_axis.setItem(row, RobotConfigurationWidget.COL_AXIS_SPEED, QTableWidgetItem(str(speed)))
                self.table_axis.setItem(row, RobotConfigurationWidget.COL_AXIS_JERK, QTableWidgetItem(str(jerk)))

                checkbox = self.axis_reversed_checkboxes[row]
                checkbox.blockSignals(True)
                checkbox.setChecked(reversed_axis == -1)
                checkbox.blockSignals(False)

            self._refresh_estimated_accel_column()
        finally:
            self.table_axis.blockSignals(False)
        self.set_cartesian_slider_limits_xyz(cartesian_slider_limits_xyz)

    def get_axis_limits(self) -> list[tuple[float, float]]:
        limits: list[tuple[float, float]] = []
        for row in range(6):
            min_val = self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_MIN, -180.0)
            max_val = self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_MAX, 180.0)
            limits.append((min_val, max_val))
        return limits

    def get_axis_speed_limits(self) -> list[float]:
        return [self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_SPEED, 0.0) for row in range(6)]

    def set_cartesian_slider_limits_xyz(self, limits: list[tuple[float, float]]) -> None:
        if self.table_cartesian_slider_limits is None:
            return

        defaults = [(-1000.0, 1000.0)] * 3
        self.table_cartesian_slider_limits.blockSignals(True)
        try:
            for row in range(3):
                min_val, max_val = defaults[row]
                if row < len(limits):
                    min_val = float(limits[row][0])
                    max_val = float(limits[row][1])
                self.table_cartesian_slider_limits.setItem(row, 0, QTableWidgetItem(str(min_val)))
                self.table_cartesian_slider_limits.setItem(row, 1, QTableWidgetItem(str(max_val)))
        finally:
            self.table_cartesian_slider_limits.blockSignals(False)

    def get_cartesian_slider_limits_xyz(self) -> list[tuple[float, float]]:
        if self.table_cartesian_slider_limits is None:
            return [(-1000.0, 1000.0)] * 3

        defaults = [(-1000.0, 1000.0)] * 3
        limits: list[tuple[float, float]] = []
        for row in range(3):
            default_min, default_max = defaults[row]
            min_val = self._cell_to_float(self.table_cartesian_slider_limits, row, 0, default_min)
            max_val = self._cell_to_float(self.table_cartesian_slider_limits, row, 1, default_max)
            limits.append((min_val, max_val))
        return limits

    def get_axis_jerk_limits(self) -> list[float]:
        return [self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_JERK, 0.0) for row in range(6)]

    def get_axis_reversed(self) -> list[int]:
        reversed_values: list[int] = []
        for row in range(6):
            checkbox = self.axis_reversed_checkboxes[row]
            reversed_values.append(-1 if checkbox.isChecked() else 1)
        return reversed_values

    def set_axis_colliders(self, axis_colliders: list[dict[str, Any]]) -> None:
        if self.table_axis_colliders is None:
            return

        normalized = axis_colliders_to_dict(
            axis_colliders if axis_colliders else default_axis_colliders(RobotConfigurationWidget.AXIS_COLLIDER_COUNT),
            RobotConfigurationWidget.AXIS_COLLIDER_COUNT,
        )
        self.table_axis_colliders.blockSignals(True)
        try:
            for row in range(RobotConfigurationWidget.AXIS_COLLIDER_COUNT):
                collider = normalized[row] if row < len(normalized) else {
                    "enabled": True,
                    "direction_axis": "z",
                    "radius": 40.0,
                    "height": 200.0,
                    "offset_axis": "",
                    "offset_value": 0.0,
                }
                enabled = bool(collider.get("enabled", True))
                direction_axis = str(collider.get("direction_axis", "z")).strip().lower()
                radius = float(collider.get("radius", 40.0))
                height = float(collider.get("height", 200.0))
                offset_axis = str(collider.get("offset_axis", "")).strip().lower()
                offset_value = float(collider.get("offset_value", 0.0))

                checkbox = self.axis_collider_enabled_checkboxes[row]
                checkbox.blockSignals(True)
                checkbox.setChecked(enabled)
                checkbox.blockSignals(False)

                direction_combo = self.axis_collider_direction_combos[row]
                direction_combo.blockSignals(True)
                direction_combo.setCurrentText(direction_axis.upper() if direction_axis in {"x", "y", "z"} else "Z")
                direction_combo.blockSignals(False)

                offset_axis_combo = self.table_axis_colliders.cellWidget(
                    row,
                    RobotConfigurationWidget.COL_AXIS_COLLIDER_OFFSET_AXIS,
                )
                if isinstance(offset_axis_combo, QComboBox):
                    offset_axis_combo.blockSignals(True)
                    self._set_combo_by_data(offset_axis_combo, offset_axis, "")
                    offset_axis_combo.blockSignals(False)

                self.table_axis_colliders.setItem(
                    row,
                    RobotConfigurationWidget.COL_AXIS_COLLIDER_RADIUS,
                    QTableWidgetItem(str(radius)),
                )
                self.table_axis_colliders.setItem(
                    row,
                    RobotConfigurationWidget.COL_AXIS_COLLIDER_HEIGHT,
                    QTableWidgetItem(str(height)),
                )
                self.table_axis_colliders.setItem(
                    row,
                    RobotConfigurationWidget.COL_AXIS_COLLIDER_OFFSET_VALUE,
                    QTableWidgetItem(str(offset_value)),
                )
        finally:
            self.table_axis_colliders.blockSignals(False)

    def get_axis_colliders(self) -> list[dict[str, Any]]:
        if self.table_axis_colliders is None:
            return axis_colliders_to_dict(
                default_axis_colliders(RobotConfigurationWidget.AXIS_COLLIDER_COUNT),
                RobotConfigurationWidget.AXIS_COLLIDER_COUNT,
            )

        values: list[dict[str, Any]] = []
        for row in range(RobotConfigurationWidget.AXIS_COLLIDER_COUNT):
            enabled = self.axis_collider_enabled_checkboxes[row].isChecked()
            direction_axis = self.axis_collider_direction_combos[row].currentText().strip().lower()

            offset_axis_widget = self.table_axis_colliders.cellWidget(row, RobotConfigurationWidget.COL_AXIS_COLLIDER_OFFSET_AXIS)
            offset_axis = str(offset_axis_widget.currentData()) if isinstance(offset_axis_widget, QComboBox) else ""

            radius = self._cell_to_float(
                self.table_axis_colliders,
                row,
                RobotConfigurationWidget.COL_AXIS_COLLIDER_RADIUS,
                40.0,
            )
            height = self._cell_to_float(
                self.table_axis_colliders,
                row,
                RobotConfigurationWidget.COL_AXIS_COLLIDER_HEIGHT,
                200.0,
            )
            offset_value = self._cell_to_float(
                self.table_axis_colliders,
                row,
                RobotConfigurationWidget.COL_AXIS_COLLIDER_OFFSET_VALUE,
                0.0,
            )
            values.append(
                {
                    "axis": row,
                    "enabled": enabled,
                    "direction_axis": direction_axis if direction_axis in {"x", "y", "z"} else "z",
                    "radius": max(0.0, radius),
                    "height": float(height),
                    "offset_axis": offset_axis if offset_axis in {"x", "y", "z"} else "",
                    "offset_value": float(offset_value),
                }
            )
        return values

    def set_positions_config(
        self,
        home_position: list[float],
        position_zero: list[float],
        position_transport: list[float],
    ) -> None:
        self.table_positions.blockSignals(True)
        try:
            for row in range(6):
                zero_value = position_zero[row] if row < len(position_zero) else 0.0
                transport_value = position_transport[row] if row < len(position_transport) else 0.0
                home_value = home_position[row] if row < len(home_position) else 0.0

                self.table_positions.setItem(row, RobotConfigurationWidget.COL_POS_ZERO, QTableWidgetItem(str(zero_value)))
                self.table_positions.setItem(row, RobotConfigurationWidget.COL_POS_TRANSPORT, QTableWidgetItem(str(transport_value)))
                self.table_positions.setItem(row, RobotConfigurationWidget.COL_POS_HOME, QTableWidgetItem(str(home_value)))
        finally:
            self.table_positions.blockSignals(False)

    def get_position_zero(self) -> list[float]:
        return [self._cell_to_float(self.table_positions, row, RobotConfigurationWidget.COL_POS_ZERO, 0.0) for row in range(6)]

    def get_position_transport(self) -> list[float]:
        return [self._cell_to_float(self.table_positions, row, RobotConfigurationWidget.COL_POS_TRANSPORT, 0.0) for row in range(6)]

    def get_home_position(self) -> list[float]:
        return [self._cell_to_float(self.table_positions, row, RobotConfigurationWidget.COL_POS_HOME, 0.0) for row in range(6)]

    def set_robot_cad_models(self, cad_models: list[str]) -> None:
        for index in range(RobotConfigurationWidget.ROBOT_CAD_COUNT):
            value = cad_models[index] if index < len(cad_models) else ""
            self.robot_cad_line_edits[index].setText(str(value))

    def get_robot_cad_models(self) -> list[str]:
        return [line_edit.text().strip() for line_edit in self.robot_cad_line_edits]

    def set_tool_cad_model(self, tool_cad_model: str | None) -> None:
        if self.tool_cad_line_edit is None:
            return
        self.tool_cad_line_edit.setText("" if tool_cad_model is None else str(tool_cad_model))

    def get_tool_cad_model(self) -> str:
        if self.tool_cad_line_edit is None:
            return ""
        return self.tool_cad_line_edit.text().strip()

    def set_tool_cad_offset_rz(self, offset_deg: float) -> None:
        if self.tool_cad_offset_rz_spin is None:
            return
        self.tool_cad_offset_rz_spin.blockSignals(True)
        self.tool_cad_offset_rz_spin.setValue(float(offset_deg))
        self.tool_cad_offset_rz_spin.blockSignals(False)

    def get_tool_cad_offset_rz(self) -> float:
        if self.tool_cad_offset_rz_spin is None:
            return 0.0
        return float(self.tool_cad_offset_rz_spin.value())

    def set_tool_colliders(self, tool_colliders: list[dict[str, Any]]) -> None:
        if self.table_tool_colliders is None:
            return
        normalized = parse_primitive_colliders(tool_colliders, default_shape="cylinder")
        self.table_tool_colliders.blockSignals(True)
        try:
            self.table_tool_colliders.setRowCount(0)
            self._tool_collider_enabled_checkboxes.clear()
            self._tool_collider_type_combos.clear()
            for collider in normalized:
                self._insert_tool_collider_row(collider)
        finally:
            self.table_tool_colliders.blockSignals(False)

    def get_tool_colliders(self) -> list[dict[str, Any]]:
        if self.table_tool_colliders is None:
            return []
        values: list[dict[str, Any]] = []
        for row in range(self.table_tool_colliders.rowCount()):
            enabled_widget = self.table_tool_colliders.cellWidget(row, RobotConfigurationWidget.COL_PRIM_ENABLED)
            enabled = bool(enabled_widget.isChecked()) if isinstance(enabled_widget, QCheckBox) else True

            shape_widget = self.table_tool_colliders.cellWidget(row, RobotConfigurationWidget.COL_PRIM_TYPE)
            shape = str(shape_widget.currentText()).strip().lower() if isinstance(shape_widget, QComboBox) else "cylinder"

            pose = [
                self._cell_to_float(self.table_tool_colliders, row, RobotConfigurationWidget.COL_PRIM_X, 0.0),
                self._cell_to_float(self.table_tool_colliders, row, RobotConfigurationWidget.COL_PRIM_Y, 0.0),
                self._cell_to_float(self.table_tool_colliders, row, RobotConfigurationWidget.COL_PRIM_Z, 0.0),
                self._cell_to_float(self.table_tool_colliders, row, RobotConfigurationWidget.COL_PRIM_A, 0.0),
                self._cell_to_float(self.table_tool_colliders, row, RobotConfigurationWidget.COL_PRIM_B, 0.0),
                self._cell_to_float(self.table_tool_colliders, row, RobotConfigurationWidget.COL_PRIM_C, 0.0),
            ]

            name_item = self.table_tool_colliders.item(row, RobotConfigurationWidget.COL_PRIM_NAME)
            name = name_item.text().strip() if name_item is not None else f"Tool collider {row + 1}"
            if name == "":
                name = f"Tool collider {row + 1}"

            values.append(
                {
                    "name": name,
                    "enabled": enabled,
                    "shape": shape,
                    "pose": pose,
                    "size_x": max(0.0, self._cell_to_float(self.table_tool_colliders, row, RobotConfigurationWidget.COL_PRIM_SIZE_X, 100.0)),
                    "size_y": max(0.0, self._cell_to_float(self.table_tool_colliders, row, RobotConfigurationWidget.COL_PRIM_SIZE_Y, 100.0)),
                    "size_z": max(0.0, self._cell_to_float(self.table_tool_colliders, row, RobotConfigurationWidget.COL_PRIM_SIZE_Z, 100.0)),
                    "radius": max(0.0, self._cell_to_float(self.table_tool_colliders, row, RobotConfigurationWidget.COL_PRIM_RADIUS, 40.0)),
                    "height": max(0.0, self._cell_to_float(self.table_tool_colliders, row, RobotConfigurationWidget.COL_PRIM_HEIGHT, 120.0)),
                }
            )

        return parse_primitive_colliders(values, default_shape="cylinder")

    def set_tool_profiles_directory(self, directory: str | None, emit_change: bool = False) -> None:
        if self.tool_profiles_dir_line_edit is None:
            return
        normalized = "" if directory is None else str(directory).strip()
        if not normalized:
            normalized = self._default_tools_directory()
        normalized = self._normalize_project_path(normalized)
        resolved_directory = self._resolve_filesystem_path(normalized)
        if resolved_directory:
            os.makedirs(resolved_directory, exist_ok=True)
        self.tool_profiles_dir_line_edit.setText(normalized)
        self._refresh_tool_profiles()
        if emit_change:
            self.tool_profiles_directory_changed.emit(normalized)

    def get_tool_profiles_directory(self) -> str:
        if self.tool_profiles_dir_line_edit is None:
            return self._default_tools_directory()
        current = self.tool_profiles_dir_line_edit.text().strip()
        return current if current else self._default_tools_directory()

    def set_selected_tool_profile(self, profile_path: str | None) -> None:
        if self.tool_profiles_combo is None:
            return
        target = "" if profile_path is None else str(profile_path).strip()
        self._tool_profile_loading = True
        try:
            if not target:
                self.tool_profiles_combo.setCurrentIndex(0)
                return

            target_abs = os.path.normcase(os.path.abspath(self._resolve_filesystem_path(target)))
            target_index = -1
            for idx in range(self.tool_profiles_combo.count()):
                item_data = self.tool_profiles_combo.itemData(idx)
                if not item_data:
                    continue
                item_abs = os.path.normcase(os.path.abspath(str(item_data)))
                if item_abs == target_abs:
                    target_index = idx
                    break

            self.tool_profiles_combo.setCurrentIndex(target_index if target_index >= 0 else 0)
        finally:
            self._tool_profile_loading = False

    def get_selected_tool_profile(self) -> str:
        if self.tool_profiles_combo is None:
            return ""
        current_data = self.tool_profiles_combo.currentData()
        if not current_data:
            return ""
        return self._normalize_project_path(str(current_data))

    def set_tool(self, tool: RobotTool) -> None:
        self.tool_widget.set_tool(tool)

    def get_tool(self) -> RobotTool:
        return self.tool_widget.get_tool()
