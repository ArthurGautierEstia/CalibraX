from __future__ import annotations

import math
import os

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QDoubleSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
    QCheckBox,
)

from widgets.robot_view.tool_widget import ToolWidget
from utils.mgi import RobotTool


class RobotConfigurationWidget(QWidget):
    """Widget principal de configuration robot (DH + axes + positions + CAD)."""

    load_config_requested = pyqtSignal()
    text_changed_requested = pyqtSignal()
    export_config_requested = pyqtSignal()

    dh_value_changed = pyqtSignal(int, int, str)
    tool_changed = pyqtSignal(RobotTool)

    axis_config_changed = pyqtSignal(list, list, list, list)
    positions_config_changed = pyqtSignal(list, list, list)
    position_zero_requested = pyqtSignal()
    position_transport_requested = pyqtSignal()
    home_position_requested = pyqtSignal()

    robot_cad_models_changed = pyqtSignal(list)
    tool_cad_model_changed = pyqtSignal(str)
    tool_cad_offset_rz_changed = pyqtSignal(float)

    COL_AXIS_MIN = 0
    COL_AXIS_MAX = 1
    COL_AXIS_SPEED = 2
    COL_AXIS_ACCEL_EST = 3
    COL_AXIS_JERK = 4
    COL_AXIS_REVERSED = 5

    COL_POS_ZERO = 0
    COL_POS_TRANSPORT = 1
    COL_POS_HOME = 2

    ROBOT_CAD_COUNT = 7

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.axis_reversed_checkboxes: list[QCheckBox] = []
        self.robot_cad_line_edits: list[QLineEdit] = []
        self.tool_cad_line_edit: QLineEdit | None = None
        self.tool_cad_offset_rz_spin: QDoubleSpinBox | None = None
        self.setup_ui()

    def setup_ui(self) -> None:
        main_layout = QHBoxLayout(self)

        left_layout = QVBoxLayout()

        title = QLabel("Configuration robot")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        left_layout.addWidget(title)

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

        left_layout.addLayout(header_layout)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_dh_tab(), "DH Table")
        self.tabs.addTab(self._build_axis_tab(), "Parametrage des axes")
        self.tabs.addTab(self._build_positions_tab(), "Parametrage des positions")
        self.tabs.addTab(self._build_cad_tab(), "CAO")
        left_layout.addWidget(self.tabs)

        main_layout.addLayout(left_layout, 3)

        self.tool_widget = ToolWidget()
        self.tool_widget.tool_changed.connect(self.tool_changed.emit)
        main_layout.addWidget(self.tool_widget, 1)

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

        description = QLabel("Selection des fichiers STL pour chaque lien robot et le tool optionnel.")
        description.setWordWrap(True)
        layout.addWidget(description)

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

        tool_row = RobotConfigurationWidget.ROBOT_CAD_COUNT
        tool_label = QLabel("Tool (optionnel)")
        self.tool_cad_line_edit = QLineEdit()
        self.tool_cad_line_edit.setReadOnly(True)

        tool_browse_button = QPushButton("Parcourir")
        tool_browse_button.clicked.connect(self._on_pick_tool_cad)

        tool_clear_button = QPushButton("Vider")
        tool_clear_button.clicked.connect(self._on_clear_tool_cad)

        grid.addWidget(tool_label, tool_row, 0)
        grid.addWidget(self.tool_cad_line_edit, tool_row, 1)
        grid.addWidget(tool_browse_button, tool_row, 2)
        grid.addWidget(tool_clear_button, tool_row, 3)

        offset_row = tool_row + 1
        offset_label = QLabel("Offset Rz tool (deg)")
        self.tool_cad_offset_rz_spin = QDoubleSpinBox()
        self.tool_cad_offset_rz_spin.setRange(-360.0, 360.0)
        self.tool_cad_offset_rz_spin.setDecimals(2)
        self.tool_cad_offset_rz_spin.setSingleStep(1.0)
        self.tool_cad_offset_rz_spin.valueChanged.connect(self.tool_cad_offset_rz_changed.emit)

        grid.addWidget(offset_label, offset_row, 0)
        grid.addWidget(self.tool_cad_offset_rz_spin, offset_row, 1)

        layout.addLayout(grid)
        layout.addStretch()

        return tab

    def _on_dh_cell_changed(self, row: int, col: int) -> None:
        item = self.table_dh.item(row, col)
        if item is not None:
            self.dh_value_changed.emit(row, col, item.text())

    def _on_axis_item_changed(self, item: QTableWidgetItem) -> None:
        if item.column() in (RobotConfigurationWidget.COL_AXIS_SPEED, RobotConfigurationWidget.COL_AXIS_JERK):
            self._refresh_estimated_accel_for_row(item.row())
        self._emit_axis_config_changed()

    def _on_positions_item_changed(self, _item: QTableWidgetItem) -> None:
        self.positions_config_changed.emit(
            self.get_home_position(),
            self.get_position_zero(),
            self.get_position_transport(),
        )

    def _on_pick_robot_cad(self, index: int) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Selectionner une CAO",
            self._get_cad_start_directory(),
            "STL files (*.stl);;All files (*)",
        )
        if not file_path:
            return

        self.robot_cad_line_edits[index].setText(file_path)
        self.robot_cad_models_changed.emit(self.get_robot_cad_models())

    def _on_clear_robot_cad(self, index: int) -> None:
        self.robot_cad_line_edits[index].setText("")
        self.robot_cad_models_changed.emit(self.get_robot_cad_models())

    def _on_pick_tool_cad(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Selectionner une CAO de tool",
            self._get_cad_start_directory(),
            "STL files (*.stl);;All files (*)",
        )
        if not file_path:
            return

        self.tool_cad_line_edit.setText(file_path)
        self.tool_cad_model_changed.emit(self.get_tool_cad_model())

    def _on_clear_tool_cad(self) -> None:
        self.tool_cad_line_edit.setText("")
        self.tool_cad_model_changed.emit("")

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
        robot_stl_dir = os.path.join(current_dir, "robot_stl")
        if os.path.isdir(robot_stl_dir):
            return robot_stl_dir
        return current_dir

    def _emit_axis_config_changed(self) -> None:
        self.axis_config_changed.emit(
            self.get_axis_limits(),
            self.get_axis_speed_limits(),
            self.get_axis_jerk_limits(),
            self.get_axis_reversed(),
        )

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

    def get_axis_limits(self) -> list[tuple[float, float]]:
        limits: list[tuple[float, float]] = []
        for row in range(6):
            min_val = self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_MIN, -180.0)
            max_val = self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_MAX, 180.0)
            limits.append((min_val, max_val))
        return limits

    def get_axis_speed_limits(self) -> list[float]:
        return [self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_SPEED, 0.0) for row in range(6)]

    def get_axis_jerk_limits(self) -> list[float]:
        return [self._cell_to_float(self.table_axis, row, RobotConfigurationWidget.COL_AXIS_JERK, 0.0) for row in range(6)]

    def get_axis_reversed(self) -> list[int]:
        reversed_values: list[int] = []
        for row in range(6):
            checkbox = self.axis_reversed_checkboxes[row]
            reversed_values.append(-1 if checkbox.isChecked() else 1)
        return reversed_values

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

    def set_tool(self, tool: RobotTool) -> None:
        self.tool_widget.set_tool(tool)

    def get_tool(self) -> RobotTool:
        return self.tool_widget.get_tool()
