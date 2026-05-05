from __future__ import annotations

import math
import os

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QColorDialog,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from models.collider_models import default_axis_colliders
from models.primitive_collider_models import AxisDirection, RobotAxisColliderData
from models.types import XYZ3
from utils.math_utils import safe_float
from widgets.toggle_switch_widget import ToggleSwitchWidget


class RobotConfigurationWidget(QWidget):
    new_config_requested = pyqtSignal()
    load_config_requested = pyqtSignal()
    save_as_config_requested = pyqtSignal()
    text_changed_requested = pyqtSignal()
    export_config_requested = pyqtSignal()
    measured_dh_enabled_changed = pyqtSignal(bool)

    dh_value_changed = pyqtSignal(int, int, float)
    axis_colliders_config_changed = pyqtSignal(list)
    axis_config_changed = pyqtSignal(list, list, list, list, list, list)
    positions_config_changed = pyqtSignal(list, list, list)
    go_to_position_requested = pyqtSignal(list)
    robot_cad_models_changed = pyqtSignal(list)
    robot_cad_colors_changed = pyqtSignal(list)
    default_tool_profile_changed = pyqtSignal(str)
    default_tool_profile_selected = pyqtSignal(str)

    COL_AXIS_MIN = 0
    COL_AXIS_MAX = 1
    COL_AXIS_SPEED = 2
    COL_AXIS_ACCEL = 3
    COL_AXIS_JERK = 4
    COL_AXIS_REVERSED = 5

    COL_POS_ZERO = 0
    COL_POS_HOME = 1
    COL_POS_CALIBRATION = 2
    POSITION_JOINT_COUNT = 6
    POSITION_ACTION_ROW = POSITION_JOINT_COUNT

    COL_AXIS_COLLIDER_ENABLED = 0
    COL_AXIS_COLLIDER_DIRECTION = 1
    COL_AXIS_COLLIDER_RADIUS = 2
    COL_AXIS_COLLIDER_HEIGHT = 3
    COL_AXIS_COLLIDER_OFFSET_X = 4
    COL_AXIS_COLLIDER_OFFSET_Y = 5
    COL_AXIS_COLLIDER_OFFSET_Z = 6

    AXIS_COLLIDER_COUNT = 6
    ROBOT_CAD_COUNT = 7
    UNIT_DEG = "°"
    UNIT_MM = "mm"
    UNIT_DEG_PER_S = "°/s"
    UNIT_DEG_PER_S2 = "°/s^2"
    UNIT_DEG_PER_S3 = "°/s^3"
    NUMERIC_VALUE_ROLE = Qt.ItemDataRole.UserRole

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.axis_reversed_checkboxes: list[QCheckBox] = []
        self.axis_collider_enabled_checkboxes: list[QCheckBox] = []
        self.axis_collider_direction_combos: list[QComboBox] = []
        self.robot_cad_line_edits: list[QLineEdit] = []
        self.robot_cad_color_buttons: list[QPushButton] = []
        self.table_axis_colliders: QTableWidget | None = None
        self.table_cartesian_slider_limits: QTableWidget | None = None
        self._extra_tab_indexes: dict[str, int] = {}
        self._default_tool_profile_path: str = ""
        self.setup_ui()

    def setup_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        top_layout = QVBoxLayout()

        title_row = QHBoxLayout()
        title_label = QLabel("Configuration robot")
        title_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        title_row.addWidget(title_label)
        title_row.addStretch()
        self.status_label = QLabel("Configuration non enregistrée")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.status_label.setStyleSheet("color: #808080; font-size: 13px; font-weight: 400;")
        title_row.addWidget(self.status_label)
        top_layout.addLayout(title_row)

        header_layout = QVBoxLayout()

        fields_layout = QGridLayout()
        current_config_title_label = QLabel("Configuration courante :")
        current_config_title_label.setMinimumWidth(150)
        fields_layout.addWidget(current_config_title_label, 0, 0)

        self.current_config_name_label = QLabel("Aucune configuration")
        self.current_config_name_label.setStyleSheet(
            "border: 1px solid #555; padding: 2px; background-color: #2a2a2a; color: #d8d8d8;"
        )
        self.current_config_name_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self.current_config_name_label.setMinimumWidth(220)
        fields_layout.addWidget(self.current_config_name_label, 0, 1)

        robot_name_title_label = QLabel("Nom du robot :")
        robot_name_title_label.setMinimumWidth(150)
        fields_layout.addWidget(robot_name_title_label, 1, 0)

        self.line_edit_robot_name = QLineEdit()
        self.line_edit_robot_name.setPlaceholderText("Nom du robot")
        self.line_edit_robot_name.textChanged.connect(self.text_changed_requested.emit)
        self.line_edit_robot_name.setMinimumWidth(220)
        fields_layout.addWidget(self.line_edit_robot_name, 1, 1)

        self.default_tool_profile_label = QLabel("Aucun tool")
        self.default_tool_profile_label.setStyleSheet(
            "border: 1px solid #555; padding: 2px; background-color: #2a2a2a; color: #d8d8d8;"
        )
        self.default_tool_profile_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self.default_tool_profile_label.setMinimumWidth(220)

        fields_layout.setColumnStretch(0, 0)
        fields_layout.setColumnStretch(1, 1)
        header_layout.addLayout(fields_layout)

        actions_layout = QHBoxLayout()
        actions_layout.addStretch()

        self.btn_load = QPushButton("Charger")
        self.btn_load.clicked.connect(self.load_config_requested.emit)
        self.btn_load.setFixedWidth(120)
        actions_layout.addWidget(self.btn_load)

        self.btn_new = QPushButton("Nouveau")
        self.btn_new.clicked.connect(self.new_config_requested.emit)
        self.btn_new.setFixedWidth(120)
        actions_layout.addWidget(self.btn_new)

        self.btn_export = QPushButton("Enregistrer")
        self.btn_export.clicked.connect(self.export_config_requested.emit)
        self.btn_export.setFixedWidth(120)
        actions_layout.addWidget(self.btn_export)

        self.btn_save_as = QPushButton("Enregistrer sous")
        self.btn_save_as.clicked.connect(self.save_as_config_requested.emit)
        self.btn_save_as.setFixedWidth(120)
        actions_layout.addWidget(self.btn_save_as)

        header_layout.addLayout(actions_layout)
        header_layout.addSpacing(8)

        default_tool_row = QHBoxLayout()
        default_tool_title_label = QLabel("Tool par défaut :")
        default_tool_title_label.setMinimumWidth(150)
        default_tool_row.addWidget(default_tool_title_label)
        default_tool_row.addWidget(self.default_tool_profile_label, 1)
        header_layout.addLayout(default_tool_row)

        default_tool_browse_row = QHBoxLayout()
        default_tool_browse_row.addStretch()
        self.btn_browse_default_tool_profile = QPushButton("Parcourir")
        self.btn_browse_default_tool_profile.clicked.connect(self._on_pick_default_tool_profile)
        self.btn_browse_default_tool_profile.setFixedWidth(120)
        default_tool_browse_row.addWidget(self.btn_browse_default_tool_profile)

        header_layout.addLayout(default_tool_browse_row)
        top_layout.addLayout(header_layout)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_dh_tab(), "DHM")
        self.tabs.addTab(self._build_axis_tab(), "Axes")
        self.tabs.addTab(self._build_axis_colliders_tab(), "Colliders")
        self.tabs.addTab(self._build_positions_tab(), "Positions")
        self.tabs.addTab(self._build_cad_tab(), "CAD Files")
        top_layout.addWidget(self.tabs)
        main_layout.addLayout(top_layout, 1)

        self.set_axis_colliders(default_axis_colliders(RobotConfigurationWidget.AXIS_COLLIDER_COUNT))

    def set_configuration_status(self, text: str, color: str) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {color}; font-size: 13px; font-weight: 400;")

    def set_current_configuration_name(self, file_name: str) -> None:
        self.current_config_name_label.setText(file_name)

    def add_tab(self, widget: QWidget, title: str) -> None:
        existing_index = self._extra_tab_indexes.get(title, -1)
        if existing_index >= 0:
            self.tabs.removeTab(existing_index)
        tab_index = self.tabs.addTab(widget, title)
        self._extra_tab_indexes[title] = tab_index

    def set_tab_enabled(self, title: str, enabled: bool) -> None:
        tab_index = self._extra_tab_indexes.get(title, -1)
        if tab_index >= 0:
            self.tabs.setTabEnabled(tab_index, enabled)

    def _build_dh_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        self.measured_dh_toggle = ToggleSwitchWidget(off_label="Robot d'usine", on_label="Robot mesuré")
        self.measured_dh_toggle.setChecked(False)
        self.measured_dh_toggle.setEnabled(False)
        self.measured_dh_toggle.toggled.connect(self._on_measured_dh_toggle_changed)
        layout.addWidget(self.measured_dh_toggle)

        tables_layout = QVBoxLayout()

        nominal_group = QGroupBox("Valeurs nominales")
        nominal_layout = QVBoxLayout(nominal_group)
        self.table_dh = QTableWidget(6, 4)
        self.table_dh.setHorizontalHeaderLabels(["alpha", "d", "theta", "r"])
        self.table_dh.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_dh.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh.horizontalHeader().setDefaultSectionSize(90)
        self.table_dh.cellChanged.connect(self._on_dh_cell_changed)
        nominal_layout.addWidget(self.table_dh)
        tables_layout.addWidget(nominal_group, 1)

        measured_group = QGroupBox("Valeurs mesurées")
        measured_layout = QVBoxLayout(measured_group)
        self.table_dh_measured = QTableWidget(6, 4)
        self.table_dh_measured.setHorizontalHeaderLabels(["alpha", "d", "theta", "r"])
        self.table_dh_measured.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_dh_measured.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh_measured.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh_measured.horizontalHeader().setDefaultSectionSize(90)
        self.table_dh_measured.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table_dh_measured.setEnabled(False)
        measured_layout.addWidget(self.table_dh_measured)
        tables_layout.addWidget(measured_group, 1)

        layout.addLayout(tables_layout)
        return tab

    def _build_axis_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        articular_group = QGroupBox("Limites articulaires")
        articular_layout = QVBoxLayout(articular_group)

        self.table_axis = QTableWidget(6, 6)
        self.table_axis.setHorizontalHeaderLabels(["Min", "Max", "Vitesse max", "Accel max", "Jerk max", "Inverse"])
        self.table_axis.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_axis.horizontalHeader().setDefaultSectionSize(135)

        self.axis_reversed_checkboxes.clear()
        for row in range(6):
            self.table_axis.setItem(row, RobotConfigurationWidget.COL_AXIS_ACCEL, QTableWidgetItem(""))
            checkbox = QCheckBox()
            checkbox.stateChanged.connect(self._emit_axis_config_changed)
            self.table_axis.setCellWidget(row, RobotConfigurationWidget.COL_AXIS_REVERSED, checkbox)
            self.axis_reversed_checkboxes.append(checkbox)

        self.table_axis.itemChanged.connect(self._on_axis_item_changed)
        articular_layout.addWidget(self.table_axis)

        axis_button_layout = QHBoxLayout()
        reset_accel_button = QPushButton("Recalculer accélérations par defaut")
        reset_accel_button.setToolTip("Acc = sqrt(Vmax * Jerkmax)")
        reset_accel_button.clicked.connect(self._on_reset_axis_accel_limits_clicked)
        axis_button_layout.addWidget(reset_accel_button)
        axis_button_layout.addStretch()
        articular_layout.addLayout(axis_button_layout)
        layout.addWidget(articular_group, 1)

        cartesian_group = QGroupBox("Limites cartésiennes")
        cartesian_layout = QVBoxLayout(cartesian_group)
        cartesian_layout.addWidget(QLabel("Bornes X/Y/Z min/max du contrôle cartésien."))
        self.table_cartesian_slider_limits = QTableWidget(3, 2)
        self.table_cartesian_slider_limits.setHorizontalHeaderLabels(["Min", "Max"])
        self.table_cartesian_slider_limits.setVerticalHeaderLabels(["X", "Y", "Z"])
        self.table_cartesian_slider_limits.horizontalHeader().setDefaultSectionSize(120)
        self.table_cartesian_slider_limits.itemChanged.connect(self._on_cartesian_slider_limits_item_changed)
        cartesian_layout.addWidget(self.table_cartesian_slider_limits)
        layout.addWidget(cartesian_group, 1)
        return tab

    def _build_axis_colliders_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        robot_group = QGroupBox("Colliders des axes robot")
        robot_layout = QVBoxLayout(robot_group)
        self.table_axis_colliders = QTableWidget(RobotConfigurationWidget.AXIS_COLLIDER_COUNT, 7)
        self.table_axis_colliders.setHorizontalHeaderLabels(["Actif", "Axe cylindre", "Rayon", "Hauteur", "X", "Y", "Z"])
        self.table_axis_colliders.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
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
            self.table_axis_colliders.setCellWidget(row, RobotConfigurationWidget.COL_AXIS_COLLIDER_DIRECTION, direction_combo)
            self.axis_collider_direction_combos.append(direction_combo)

        self.table_axis_colliders.itemChanged.connect(self._on_axis_colliders_item_changed)
        robot_layout.addWidget(self.table_axis_colliders)
        layout.addWidget(robot_group, 1)
        layout.addStretch()
        return tab

    def _build_positions_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        positions_group = QGroupBox("Paramétrage des positions")
        positions_layout = QVBoxLayout(positions_group)
        self.table_positions = QTableWidget(RobotConfigurationWidget.POSITION_JOINT_COUNT + 1, 3)
        self.table_positions.setHorizontalHeaderLabels(["Position 0", "Position home", "Position de calibration"])
        self.table_positions.setVerticalHeaderLabels(
            [f"q{i + 1}" for i in range(RobotConfigurationWidget.POSITION_JOINT_COUNT)] + ["Aller à"]
        )
        self.table_positions.horizontalHeader().setDefaultSectionSize(180)
        self.table_positions.itemChanged.connect(self._on_positions_item_changed)

        for column in (
            RobotConfigurationWidget.COL_POS_ZERO,
            RobotConfigurationWidget.COL_POS_HOME,
            RobotConfigurationWidget.COL_POS_CALIBRATION,
        ):
            go_to_button = QPushButton("Aller à")
            go_to_button.clicked.connect(lambda _, c=column: self._on_go_to_position_clicked(c))
            self.table_positions.setCellWidget(RobotConfigurationWidget.POSITION_ACTION_ROW, column, go_to_button)

        positions_layout.addWidget(self.table_positions)
        layout.addWidget(positions_group, 1)

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
        self.robot_cad_color_buttons.clear()
        for index in range(RobotConfigurationWidget.ROBOT_CAD_COUNT):
            label = QLabel(f"Lien {index}")
            path_line = QLineEdit()
            path_line.setReadOnly(True)
            color_button = QPushButton()
            color_button.setFixedSize(24, 24)
            color_button.clicked.connect(lambda _, i=index: self._on_pick_robot_cad_color(i))
            browse_button = QPushButton("Parcourir")
            browse_button.clicked.connect(lambda _, i=index: self._on_pick_robot_cad(i))
            clear_button = QPushButton("Vider")
            clear_button.clicked.connect(lambda _, i=index: self._on_clear_robot_cad(i))
            self.robot_cad_line_edits.append(path_line)
            self.robot_cad_color_buttons.append(color_button)
            grid.addWidget(label, index, 0)
            grid.addWidget(path_line, index, 1)
            grid.addWidget(color_button, index, 2, alignment=Qt.AlignmentFlag.AlignLeft)
            grid.addWidget(browse_button, index, 3)
            grid.addWidget(clear_button, index, 4)

        layout.addLayout(grid)
        layout.addStretch()
        return tab

    def _on_dh_cell_changed(self, row: int, col: int) -> None:
        item = self.table_dh.item(row, col)
        if item is not None:
            unit = RobotConfigurationWidget.UNIT_DEG if col in (0, 2) else RobotConfigurationWidget.UNIT_MM
            numeric_value = self._format_table_item_with_unit(self.table_dh, item, unit, True)
            self.dh_value_changed.emit(row, col, numeric_value)

    def _on_axis_item_changed(self, item: QTableWidgetItem) -> None:
        if item.column() == RobotConfigurationWidget.COL_AXIS_ACCEL:
            self._format_axis_accel_item(item)
        elif item.column() in (RobotConfigurationWidget.COL_AXIS_MIN, RobotConfigurationWidget.COL_AXIS_MAX):
            self._format_table_item_with_unit(self.table_axis, item, RobotConfigurationWidget.UNIT_DEG)
        elif item.column() == RobotConfigurationWidget.COL_AXIS_SPEED:
            self._format_table_item_with_unit(self.table_axis, item, RobotConfigurationWidget.UNIT_DEG_PER_S)
        elif item.column() == RobotConfigurationWidget.COL_AXIS_JERK:
            self._format_table_item_with_unit(self.table_axis, item, RobotConfigurationWidget.UNIT_DEG_PER_S3)
        self._emit_axis_config_changed()

    def _on_cartesian_slider_limits_item_changed(self, item: QTableWidgetItem) -> None:
        self._format_table_item_with_unit(self.table_cartesian_slider_limits, item, RobotConfigurationWidget.UNIT_MM)
        self._emit_axis_config_changed()

    def _on_reset_axis_accel_limits_clicked(self) -> None:
        self._reset_axis_accel_limits_to_calculated_defaults()
        self._emit_axis_config_changed()

    def _on_axis_colliders_item_changed(self, item: QTableWidgetItem) -> None:
        self._format_table_item_with_unit(self.table_axis_colliders, item, RobotConfigurationWidget.UNIT_MM, True)
        self._emit_axis_colliders_config_changed()

    def _on_positions_item_changed(self, item: QTableWidgetItem) -> None:
        self._format_table_item_with_unit(self.table_positions, item, RobotConfigurationWidget.UNIT_DEG, True)
        self.positions_config_changed.emit(self.get_home_position(), self.get_position_zero(), self.get_position_calibration())

    def _on_go_to_position_clicked(self, column: int) -> None:
        self.go_to_position_requested.emit(self._get_position_column_values(column))

    def _on_pick_robot_cad(self, index: int) -> None:
        file_path, _ = QFileDialog.getOpenFileName(self, "Sélectionner une CAO", self._get_cad_start_directory(), "STL files (*.stl);;All files (*)")
        if not file_path:
            return
        self.robot_cad_line_edits[index].setText(self._normalize_cad_path(file_path))
        self.robot_cad_models_changed.emit(self.get_robot_cad_models())

    def _on_pick_multiple_robot_cad(self) -> None:
        file_paths, _ = QFileDialog.getOpenFileNames(self, "Sélectionner plusieurs CAD Files", self._get_cad_start_directory(), "STL files (*.stl);;All files (*)")
        if not file_paths:
            return
        cad_paths = [self._normalize_cad_path(path) for path in file_paths]
        max_count = RobotConfigurationWidget.ROBOT_CAD_COUNT
        if len(cad_paths) > max_count:
            cad_paths = cad_paths[:max_count]
            QMessageBox.information(self, "Sélection limitée", f"Seulement {max_count} fichiers sont utilisés (indices 0 à {max_count - 1}).")

        start_index = 0
        if len(cad_paths) < max_count:
            max_start = max_count - len(cad_paths)
            start_index, ok = QInputDialog.getInt(
                self,
                "Index de départ",
                f"{len(cad_paths)} fichiers sélectionnés.\nChoisissez l'index de départ pour l'affectation ({0}   {max_start}).",
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

    def _on_pick_default_tool_profile(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Sélectionner un tool par défaut",
            self._get_tool_start_directory(),
            "JSON files (*.json);;All files (*)",
        )
        if not file_path:
            return
        normalized_path = self._normalize_project_path(file_path)
        self.set_default_tool_profile(normalized_path)
        self.default_tool_profile_changed.emit(normalized_path)
        self._emit_default_tool_profile_selected()

    def _on_pick_robot_cad_color(self, index: int) -> None:
        current_hex = self.get_robot_cad_colors()[index]
        current_color = QColor(current_hex) if current_hex else QColor("#808080")
        selected_color = QColorDialog.getColor(current_color, self, f"Choisir une couleur STL pour le lien {index}")
        if not selected_color.isValid():
            return
        self._set_robot_cad_color_label(index, selected_color.name(QColor.NameFormat.HexRgb).upper())
        self.robot_cad_colors_changed.emit(self.get_robot_cad_colors())

    def _cell_to_float(self, table: QTableWidget, row: int, column: int, default: float = 0.0) -> float:
        item = table.item(row, column)
        if item is None:
            return default
        numeric_data = item.data(RobotConfigurationWidget.NUMERIC_VALUE_ROLE)
        if numeric_data is not None:
            return safe_float(numeric_data, default)
        return RobotConfigurationWidget._parse_table_numeric_text(item.text(), default)

    @staticmethod
    def _format_value_with_unit(value: float | int | str, unit: str) -> str:
        return f"{value} {unit}"

    @staticmethod
    def _numeric_unit_suffixes() -> tuple[str, ...]:
        return (
            RobotConfigurationWidget.UNIT_DEG_PER_S3,
            RobotConfigurationWidget.UNIT_DEG_PER_S2,
            RobotConfigurationWidget.UNIT_DEG_PER_S,
            RobotConfigurationWidget.UNIT_DEG,
            RobotConfigurationWidget.UNIT_MM,
        )

    @staticmethod
    def _parse_table_numeric_text(text: str, default: float = 0.0) -> float:
        numeric_text = str(text).strip()
        if numeric_text == "":
            return default
        for unit in RobotConfigurationWidget._numeric_unit_suffixes():
            if numeric_text.endswith(unit):
                numeric_text = numeric_text[: -len(unit)].strip()
                break
        return safe_float(numeric_text, default)

    @staticmethod
    def _set_item_numeric_value(item: QTableWidgetItem, value: float) -> None:
        item.setData(RobotConfigurationWidget.NUMERIC_VALUE_ROLE, float(value))

    @staticmethod
    def _clear_item_numeric_value(item: QTableWidgetItem) -> None:
        item.setData(RobotConfigurationWidget.NUMERIC_VALUE_ROLE, None)

    @staticmethod
    def _make_table_item_with_unit(value: float | int | str, unit: str) -> QTableWidgetItem:
        item = QTableWidgetItem(RobotConfigurationWidget._format_value_with_unit(value, unit))
        raw_value = str(value).strip()
        if raw_value != "":
            numeric_value = RobotConfigurationWidget._parse_table_numeric_text(raw_value, 0.0)
            RobotConfigurationWidget._set_item_numeric_value(item, numeric_value)
        return item

    @staticmethod
    def _set_table_item_with_unit(table: QTableWidget, row: int, column: int, value: float | int | str, unit: str) -> None:
        table.setItem(row, column, RobotConfigurationWidget._make_table_item_with_unit(value, unit))

    @staticmethod
    def _format_table_item_with_unit(
        table: QTableWidget,
        item: QTableWidgetItem,
        unit: str,
        restore_current_on_empty: bool = False,
    ) -> float:
        raw_text = item.text().strip()
        if raw_text == "":
            if restore_current_on_empty:
                current_value = safe_float(item.data(RobotConfigurationWidget.NUMERIC_VALUE_ROLE), 0.0)
                table.blockSignals(True)
                try:
                    item.setText(RobotConfigurationWidget._format_value_with_unit(current_value, unit))
                    RobotConfigurationWidget._set_item_numeric_value(item, current_value)
                finally:
                    table.blockSignals(False)
                return current_value
            RobotConfigurationWidget._clear_item_numeric_value(item)
            return 0.0
        numeric_value = RobotConfigurationWidget._parse_table_numeric_text(raw_text, 0.0)
        table.blockSignals(True)
        try:
            item.setText(RobotConfigurationWidget._format_value_with_unit(numeric_value, unit))
            RobotConfigurationWidget._set_item_numeric_value(item, numeric_value)
        finally:
            table.blockSignals(False)
        return numeric_value

    def _calculate_default_axis_accel_for_row(self, row: int) -> float:
        speed = max(0.0, self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_SPEED, 0.0))
        jerk = max(0.0, self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_JERK, 0.0))
        return math.sqrt(speed * jerk)

    def _set_axis_accel_cell(self, row: int, accel: float) -> None:
        accel_item = self.table_axis.item(row, RobotConfigurationWidget.COL_AXIS_ACCEL)
        if accel_item is None:
            accel_item = QTableWidgetItem("")
            self.table_axis.setItem(row, RobotConfigurationWidget.COL_AXIS_ACCEL, accel_item)
        self._set_item_numeric_value(accel_item, accel)
        accel_item.setText(self._format_value_with_unit(f"{accel:.3f}", RobotConfigurationWidget.UNIT_DEG_PER_S2))

    def _format_axis_accel_item(self, item: QTableWidgetItem) -> float:
        if item.text().strip() == "":
            self._clear_item_numeric_value(item)
            return 0.0
        accel = max(0.0, self._parse_table_numeric_text(item.text(), 0.0))
        self.table_axis.blockSignals(True)
        try:
            item.setText(self._format_value_with_unit(f"{accel:.3f}", RobotConfigurationWidget.UNIT_DEG_PER_S2))
            self._set_item_numeric_value(item, accel)
        finally:
            self.table_axis.blockSignals(False)
        return accel

    def _reset_axis_accel_limits_to_calculated_defaults(self) -> None:
        self.table_axis.blockSignals(True)
        try:
            for row in range(6):
                self._set_axis_accel_cell(row, self._calculate_default_axis_accel_for_row(row))
        finally:
            self.table_axis.blockSignals(False)

    @staticmethod
    def _get_cad_start_directory() -> str:
        current_dir = os.getcwd()
        robots_stl_dir = os.path.join(current_dir, "default_data", "robots_stl")
        if os.path.isdir(robots_stl_dir):
            return robots_stl_dir
        return current_dir

    @staticmethod
    def _get_tool_start_directory() -> str:
        current_dir = os.getcwd()
        tools_dir = os.path.join(current_dir, "default_data", "tools")
        if os.path.isdir(tools_dir):
            return tools_dir
        return current_dir

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
            self.get_axis_accel_limits(),
            self.get_axis_jerk_limits(),
            self.get_axis_reversed(),
        )

    def _emit_axis_colliders_config_changed(self) -> None:
        self.axis_colliders_config_changed.emit(self.get_axis_colliders())

    def set_robot_name(self, name: str) -> None:
        self.line_edit_robot_name.setText(name)

    def get_robot_name(self) -> str:
        return self.line_edit_robot_name.text()

    def set_default_tool_profile(self, profile_path: str) -> None:
        normalized_path = str(profile_path).strip()
        self._default_tool_profile_path = normalized_path
        display_name = os.path.basename(normalized_path) if normalized_path else ""
        self.default_tool_profile_label.setText(display_name if display_name else "Aucun tool")

    def get_default_tool_profile(self) -> str:
        return self._default_tool_profile_path

    def _emit_default_tool_profile_selected(self) -> None:
        self.default_tool_profile_selected.emit(self.get_default_tool_profile())

    def set_dh_params(self, params: list[list[float]]) -> None:
        self.table_dh.blockSignals(True)
        try:
            for row in range(6):
                values = params[row] if row < len(params) else []
                for col in range(4):
                    value = str(values[col]) if col < len(values) else ""
                    unit = RobotConfigurationWidget.UNIT_DEG if col in (0, 2) else RobotConfigurationWidget.UNIT_MM
                    self._set_table_item_with_unit(self.table_dh, row, col, value, unit)
        finally:
            self.table_dh.blockSignals(False)

    def set_measured_dh_params(self, params: list[list[float]]) -> None:
        if self.table_dh_measured is None:
            return
        self.table_dh_measured.blockSignals(True)
        try:
            for row in range(6):
                values = params[row] if row < len(params) else []
                for col in range(4):
                    value = str(values[col]) if col < len(values) else ""
                    unit = RobotConfigurationWidget.UNIT_DEG if col in (0, 2) else RobotConfigurationWidget.UNIT_MM
                    item = QTableWidgetItem(self._format_value_with_unit(value, unit))
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                    self.table_dh_measured.setItem(row, col, item)
        finally:
            self.table_dh_measured.blockSignals(False)

    def set_measured_dh_table_enabled(self, enabled: bool) -> None:
        if self.measured_dh_toggle is not None:
            self.measured_dh_toggle.setChecked(enabled)
        if self.table_dh_measured is not None:
            self.table_dh_measured.setEnabled(enabled)
        if self.table_dh is not None:
            self.table_dh.setEnabled(not enabled)

    def _on_measured_dh_toggle_changed(self, checked: bool) -> None:
        if self.table_dh_measured is not None:
            self.table_dh_measured.setEnabled(checked)
        if self.table_dh is not None:
            self.table_dh.setEnabled(not checked)
        self.measured_dh_enabled_changed.emit(checked)

    def get_dh_params(self) -> list[list[float]]:
        params: list[list[float]] = []
        for row in range(6):
            row_values: list[float] = []
            for col in range(4):
                row_values.append(self._cell_to_float(self.table_dh, row, col, 0.0))
            params.append(row_values)
        return params

    def set_axis_config(
        self,
        axis_limits: list[tuple[float, float]],
        cartesian_slider_limits_xyz: list[tuple[float, float]],
        axis_speed_limits: list[float],
        axis_accel_limits: list[float],
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
                accel = axis_accel_limits[row] if row < len(axis_accel_limits) else math.sqrt(max(0.0, float(speed)) * max(0.0, float(jerk)))
                reversed_axis = axis_reversed[row] if row < len(axis_reversed) else 1
                self._set_table_item_with_unit(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_MIN, min_val, RobotConfigurationWidget.UNIT_DEG)
                self._set_table_item_with_unit(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_MAX, max_val, RobotConfigurationWidget.UNIT_DEG)
                self._set_table_item_with_unit(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_SPEED, speed, RobotConfigurationWidget.UNIT_DEG_PER_S)
                self._set_axis_accel_cell(row, float(accel))
                self._set_table_item_with_unit(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_JERK, jerk, RobotConfigurationWidget.UNIT_DEG_PER_S3)
                checkbox = self.axis_reversed_checkboxes[row]
                checkbox.blockSignals(True)
                checkbox.setChecked(reversed_axis == -1)
                checkbox.blockSignals(False)
        finally:
            self.table_axis.blockSignals(False)
        self.set_cartesian_slider_limits_xyz(cartesian_slider_limits_xyz)

    def get_axis_limits(self) -> list[tuple[float, float]]:
        return [
            (
                self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_MIN, -180.0),
                self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_MAX, 180.0),
            )
            for row in range(6)
        ]

    def get_axis_speed_limits(self) -> list[float]:
        return [self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_SPEED, 0.0) for row in range(6)]

    def get_axis_accel_limits(self) -> list[float]:
        return [self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_ACCEL, 0.0) for row in range(6)]

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
                self._set_table_item_with_unit(self.table_cartesian_slider_limits, row, 0, min_val, RobotConfigurationWidget.UNIT_MM)
                self._set_table_item_with_unit(self.table_cartesian_slider_limits, row, 1, max_val, RobotConfigurationWidget.UNIT_MM)
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
        return [-1 if checkbox.isChecked() else 1 for checkbox in self.axis_reversed_checkboxes]

    def set_axis_colliders(self, axis_colliders: list[RobotAxisColliderData]) -> None:
        if self.table_axis_colliders is None:
            return
        if not all(isinstance(collider, RobotAxisColliderData) for collider in axis_colliders):
            raise TypeError("axis_colliders must contain RobotAxisColliderData")
        if axis_colliders and len(axis_colliders) != RobotConfigurationWidget.AXIS_COLLIDER_COUNT:
            raise ValueError("axis_colliders must contain 6 values")

        normalized = axis_colliders if axis_colliders else default_axis_colliders(RobotConfigurationWidget.AXIS_COLLIDER_COUNT)
        self.table_axis_colliders.blockSignals(True)
        try:
            for row in range(RobotConfigurationWidget.AXIS_COLLIDER_COUNT):
                collider = normalized[row]
                checkbox = self.axis_collider_enabled_checkboxes[row]
                checkbox.blockSignals(True)
                checkbox.setChecked(collider.enabled)
                checkbox.blockSignals(False)

                direction_combo = self.axis_collider_direction_combos[row]
                direction_combo.blockSignals(True)
                direction_combo.setCurrentText(collider.direction_axis.value.upper())
                direction_combo.blockSignals(False)

                self._set_table_item_with_unit(self.table_axis_colliders, row, RobotConfigurationWidget.COL_AXIS_COLLIDER_RADIUS, collider.radius, RobotConfigurationWidget.UNIT_MM)
                self._set_table_item_with_unit(self.table_axis_colliders, row, RobotConfigurationWidget.COL_AXIS_COLLIDER_HEIGHT, collider.height, RobotConfigurationWidget.UNIT_MM)
                self._set_table_item_with_unit(self.table_axis_colliders, row, RobotConfigurationWidget.COL_AXIS_COLLIDER_OFFSET_X, float(collider.offset_xyz.x), RobotConfigurationWidget.UNIT_MM)
                self._set_table_item_with_unit(self.table_axis_colliders, row, RobotConfigurationWidget.COL_AXIS_COLLIDER_OFFSET_Y, float(collider.offset_xyz.y), RobotConfigurationWidget.UNIT_MM)
                self._set_table_item_with_unit(self.table_axis_colliders, row, RobotConfigurationWidget.COL_AXIS_COLLIDER_OFFSET_Z, float(collider.offset_xyz.z), RobotConfigurationWidget.UNIT_MM)
        finally:
            self.table_axis_colliders.blockSignals(False)

    def get_axis_colliders(self) -> list[RobotAxisColliderData]:
        if self.table_axis_colliders is None:
            return default_axis_colliders(RobotConfigurationWidget.AXIS_COLLIDER_COUNT)
        values: list[RobotAxisColliderData] = []
        for row in range(RobotConfigurationWidget.AXIS_COLLIDER_COUNT):
            values.append(
                RobotAxisColliderData(
                    axis_index=row,
                    enabled=self.axis_collider_enabled_checkboxes[row].isChecked(),
                    direction_axis=AxisDirection(self.axis_collider_direction_combos[row].currentText().strip().lower()),
                    radius=max(0.0, self._cell_to_float(self.table_axis_colliders, row, RobotConfigurationWidget.COL_AXIS_COLLIDER_RADIUS, 40.0)),
                    height=float(self._cell_to_float(self.table_axis_colliders, row, RobotConfigurationWidget.COL_AXIS_COLLIDER_HEIGHT, 200.0)),
                    offset_xyz=XYZ3(
                        float(self._cell_to_float(self.table_axis_colliders, row, RobotConfigurationWidget.COL_AXIS_COLLIDER_OFFSET_X, 0.0)),
                        float(self._cell_to_float(self.table_axis_colliders, row, RobotConfigurationWidget.COL_AXIS_COLLIDER_OFFSET_Y, 0.0)),
                        float(self._cell_to_float(self.table_axis_colliders, row, RobotConfigurationWidget.COL_AXIS_COLLIDER_OFFSET_Z, 0.0)),
                    ),
                )
            )
        return values

    def set_positions_config(self, home_position: list[float], position_zero: list[float], position_calibration: list[float]) -> None:
        self.table_positions.blockSignals(True)
        try:
            for row in range(RobotConfigurationWidget.POSITION_JOINT_COUNT):
                zero_value = position_zero[row] if row < len(position_zero) else 0.0
                calibration_value = position_calibration[row] if row < len(position_calibration) else 0.0
                home_value = home_position[row] if row < len(home_position) else 0.0
                self._set_table_item_with_unit(self.table_positions, row, RobotConfigurationWidget.COL_POS_ZERO, zero_value, RobotConfigurationWidget.UNIT_DEG)
                self._set_table_item_with_unit(self.table_positions, row, RobotConfigurationWidget.COL_POS_CALIBRATION, calibration_value, RobotConfigurationWidget.UNIT_DEG)
                self._set_table_item_with_unit(self.table_positions, row, RobotConfigurationWidget.COL_POS_HOME, home_value, RobotConfigurationWidget.UNIT_DEG)
        finally:
            self.table_positions.blockSignals(False)

    def get_position_zero(self) -> list[float]:
        return self._get_position_column_values(RobotConfigurationWidget.COL_POS_ZERO)

    def get_position_calibration(self) -> list[float]:
        return self._get_position_column_values(RobotConfigurationWidget.COL_POS_CALIBRATION)

    def get_home_position(self) -> list[float]:
        return self._get_position_column_values(RobotConfigurationWidget.COL_POS_HOME)

    def _get_position_column_values(self, column: int) -> list[float]:
        return [
            self._cell_to_float(self.table_positions, row, column, 0.0)
            for row in range(RobotConfigurationWidget.POSITION_JOINT_COUNT)
        ]

    def set_robot_cad_models(self, cad_models: list[str]) -> None:
        for index in range(RobotConfigurationWidget.ROBOT_CAD_COUNT):
            value = cad_models[index] if index < len(cad_models) else ""
            self.robot_cad_line_edits[index].setText(str(value))

    def get_robot_cad_models(self) -> list[str]:
        return [line_edit.text().strip() for line_edit in self.robot_cad_line_edits]

    def set_robot_cad_colors(self, cad_colors: list) -> None:
        for index in range(RobotConfigurationWidget.ROBOT_CAD_COUNT):
            if index < len(cad_colors):
                raw_color = cad_colors[index]
                color_hex = raw_color.to_hex() if hasattr(raw_color, "to_hex") else str(raw_color)
            else:
                color_hex = ""
            self._set_robot_cad_color_label(index, color_hex)

    def get_robot_cad_colors(self) -> list[str]:
        values: list[str] = []
        for button in self.robot_cad_color_buttons:
            values.append(str(button.property("cad_color_hex") or "").strip().upper())
        return values

    def _set_robot_cad_color_label(self, index: int, color_hex: str) -> None:
        if not (0 <= index < len(self.robot_cad_color_buttons)):
            return
        normalized_hex = str(color_hex).strip().upper()
        button = self.robot_cad_color_buttons[index]
        button.setText("")
        if normalized_hex:
            button.setStyleSheet(
                f"min-width: 24px; max-width: 24px; min-height: 24px; max-height: 24px; "
                f"border: 1px solid #555; background-color: {normalized_hex};"
            )
            button.setProperty("cad_color_hex", normalized_hex)
            return
        button.setStyleSheet(
            "min-width: 24px; max-width: 24px; min-height: 24px; max-height: 24px; "
            "border: 1px solid #777; background-color: transparent;"
        )
        button.setProperty("cad_color_hex", "")
