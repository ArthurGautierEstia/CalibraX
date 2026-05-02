from __future__ import annotations

import math
import os

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
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
    load_config_requested = pyqtSignal()
    text_changed_requested = pyqtSignal()
    export_config_requested = pyqtSignal()
    measured_dh_enabled_changed = pyqtSignal(bool)

    dh_value_changed = pyqtSignal(int, int, str)
    axis_colliders_config_changed = pyqtSignal(list)
    axis_config_changed = pyqtSignal(list, list, list, list, list, list)
    positions_config_changed = pyqtSignal(list, list, list)
    robot_cad_models_changed = pyqtSignal(list)

    COL_AXIS_MIN = 0
    COL_AXIS_MAX = 1
    COL_AXIS_SPEED = 2
    COL_AXIS_ACCEL = 3
    COL_AXIS_JERK = 4
    COL_AXIS_REVERSED = 5

    COL_POS_ZERO = 0
    COL_POS_CALIBRATION = 1
    COL_POS_HOME = 2

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

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.axis_reversed_checkboxes: list[QCheckBox] = []
        self.axis_collider_enabled_checkboxes: list[QCheckBox] = []
        self.axis_collider_direction_combos: list[QComboBox] = []
        self.robot_cad_line_edits: list[QLineEdit] = []
        self.table_axis_colliders: QTableWidget | None = None
        self.table_cartesian_slider_limits: QTableWidget | None = None
        self._extra_tab_indexes: dict[str, int] = {}
        self.setup_ui()

    def setup_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        top_layout = QVBoxLayout()

        title_label = QLabel("Configuration robot")
        title_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        top_layout.addWidget(title_label)

        header_layout = QGridLayout()
        self.line_edit_robot_name = QLineEdit()
        self.line_edit_robot_name.setPlaceholderText("Nom du robot")
        self.line_edit_robot_name.textChanged.connect(self.text_changed_requested.emit)
        header_layout.addWidget(self.line_edit_robot_name, 0, 0)

        self.btn_load = QPushButton("Charger")
        self.btn_load.clicked.connect(self.load_config_requested.emit)
        header_layout.addWidget(self.btn_load, 0, 1)

        self.btn_export = QPushButton("Enregistrer")
        self.btn_export.clicked.connect(self.export_config_requested.emit)
        header_layout.addWidget(self.btn_export, 0, 2)
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
        layout.addWidget(self.table_axis)

        axis_button_layout = QHBoxLayout()
        reset_accel_button = QPushButton("Recalculer accels par defaut")
        reset_accel_button.clicked.connect(self._on_reset_axis_accel_limits_clicked)
        axis_button_layout.addWidget(reset_accel_button)
        axis_button_layout.addStretch()
        layout.addLayout(axis_button_layout)
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
        self.table_positions = QTableWidget(6, 3)
        self.table_positions.setHorizontalHeaderLabels(["Position 0", "Position calibration", "Position home"])
        self.table_positions.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_positions.horizontalHeader().setDefaultSectionSize(180)
        self.table_positions.itemChanged.connect(self._on_positions_item_changed)
        positions_layout.addWidget(self.table_positions)
        layout.addWidget(positions_group, 1)

        cartesian_group = QGroupBox("Plages des sliders cartésiens")
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
            label = QLabel(f"Lien {index}")
            path_line = QLineEdit()
            path_line.setReadOnly(True)
            browse_button = QPushButton("Parcourir")
            browse_button.clicked.connect(lambda _, i=index: self._on_pick_robot_cad(i))
            clear_button = QPushButton("Vider")
            clear_button.clicked.connect(lambda _, i=index: self._on_clear_robot_cad(i))
            self.robot_cad_line_edits.append(path_line)
            grid.addWidget(label, index, 0)
            grid.addWidget(path_line, index, 1)
            grid.addWidget(browse_button, index, 2)
            grid.addWidget(clear_button, index, 3)

        layout.addLayout(grid)
        layout.addStretch()
        return tab

    def _on_dh_cell_changed(self, row: int, col: int) -> None:
        item = self.table_dh.item(row, col)
        if item is not None:
            unit = RobotConfigurationWidget.UNIT_DEG if col in (0, 2) else RobotConfigurationWidget.UNIT_MM
            self._format_table_item_with_unit(self.table_dh, item, unit)
            self.dh_value_changed.emit(row, col, item.text())

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
        self._format_table_item_with_unit(self.table_axis_colliders, item, RobotConfigurationWidget.UNIT_MM)
        self._emit_axis_colliders_config_changed()

    def _on_positions_item_changed(self, item: QTableWidgetItem) -> None:
        self._format_table_item_with_unit(self.table_positions, item, RobotConfigurationWidget.UNIT_DEG)
        self.positions_config_changed.emit(self.get_home_position(), self.get_position_zero(), self.get_position_calibration())

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
                f"{len(cad_paths)} fichiers sélectionnés.\nChoisissez l'index de départ pour l'affectation ({0} à {max_start}).",
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

    def _cell_to_float(self, table: QTableWidget, row: int, column: int, default: float = 0.0) -> float:
        item = table.item(row, column)
        return safe_float(item.text() if item else "", default)

    @staticmethod
    def _format_value_with_unit(value: float | int | str, unit: str) -> str:
        return f"{value} {unit}"

    @staticmethod
    def _set_table_item_with_unit(table: QTableWidget, row: int, column: int, value: float | int | str, unit: str) -> None:
        table.setItem(row, column, QTableWidgetItem(RobotConfigurationWidget._format_value_with_unit(value, unit)))

    @staticmethod
    def _format_table_item_with_unit(table: QTableWidget, item: QTableWidgetItem, unit: str) -> None:
        raw_text = item.text().strip()
        if raw_text == "":
            return
        numeric_value = safe_float(raw_text, 0.0)
        table.blockSignals(True)
        try:
            item.setText(RobotConfigurationWidget._format_value_with_unit(numeric_value, unit))
        finally:
            table.blockSignals(False)

    def _calculate_default_axis_accel_for_row(self, row: int) -> float:
        speed = max(0.0, self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_SPEED, 0.0))
        jerk = max(0.0, self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_JERK, 0.0))
        return math.sqrt(speed * jerk)

    def _set_axis_accel_cell(self, row: int, accel: float) -> None:
        accel_item = self.table_axis.item(row, RobotConfigurationWidget.COL_AXIS_ACCEL)
        if accel_item is None:
            accel_item = QTableWidgetItem("")
            self.table_axis.setItem(row, RobotConfigurationWidget.COL_AXIS_ACCEL, accel_item)
        accel_item.setText(self._format_value_with_unit(f"{accel:.3f}", RobotConfigurationWidget.UNIT_DEG_PER_S2))

    def _format_axis_accel_item(self, item: QTableWidgetItem) -> None:
        if item.text().strip() == "":
            return
        accel = max(0.0, safe_float(item.text(), 0.0))
        self.table_axis.blockSignals(True)
        try:
            item.setText(self._format_value_with_unit(f"{accel:.3f}", RobotConfigurationWidget.UNIT_DEG_PER_S2))
        finally:
            self.table_axis.blockSignals(False)

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
        robots_stl_dir = os.path.join(current_dir, "default", "robots_stl")
        if os.path.isdir(robots_stl_dir):
            return robots_stl_dir
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
            for row in range(6):
                zero_value = position_zero[row] if row < len(position_zero) else 0.0
                calibration_value = position_calibration[row] if row < len(position_calibration) else 0.0
                home_value = home_position[row] if row < len(home_position) else 0.0
                self._set_table_item_with_unit(self.table_positions, row, RobotConfigurationWidget.COL_POS_ZERO, zero_value, RobotConfigurationWidget.UNIT_DEG)
                self._set_table_item_with_unit(self.table_positions, row, RobotConfigurationWidget.COL_POS_CALIBRATION, calibration_value, RobotConfigurationWidget.UNIT_DEG)
                self._set_table_item_with_unit(self.table_positions, row, RobotConfigurationWidget.COL_POS_HOME, home_value, RobotConfigurationWidget.UNIT_DEG)
        finally:
            self.table_positions.blockSignals(False)

    def get_position_zero(self) -> list[float]:
        return [self._cell_to_float(self.table_positions, row, RobotConfigurationWidget.COL_POS_ZERO, 0.0) for row in range(6)]

    def get_position_calibration(self) -> list[float]:
        return [self._cell_to_float(self.table_positions, row, RobotConfigurationWidget.COL_POS_CALIBRATION, 0.0) for row in range(6)]

    def get_home_position(self) -> list[float]:
        return [self._cell_to_float(self.table_positions, row, RobotConfigurationWidget.COL_POS_HOME, 0.0) for row in range(6)]

    def set_robot_cad_models(self, cad_models: list[str]) -> None:
        for index in range(RobotConfigurationWidget.ROBOT_CAD_COUNT):
            value = cad_models[index] if index < len(cad_models) else ""
            self.robot_cad_line_edits[index].setText(str(value))

    def get_robot_cad_models(self) -> list[str]:
        return [line_edit.text().strip() for line_edit in self.robot_cad_line_edits]
