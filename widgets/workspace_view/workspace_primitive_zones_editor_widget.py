from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from models.workspace_primitive_zone_models import (
    WorkspacePrimitiveZoneData,
    parse_workspace_primitive_zones,
)
from utils.math_utils import safe_float


class WorkspacePrimitiveZonesEditorWidget(QWidget):
    zones_changed = pyqtSignal(list)
    zone_changed = pyqtSignal(int, object)
    zone_added = pyqtSignal(int, object)
    zone_removed = pyqtSignal(int)

    COL_ENABLED = 0
    COL_NAME = 1
    COL_TYPE = 2
    COL_X = 3
    COL_Y = 4
    COL_Z = 5
    COL_A = 6
    COL_B = 7
    COL_C = 8
    COL_SIZE_X = 9
    COL_SIZE_Y = 10
    COL_SIZE_Z = 11
    COL_RADIUS = 12
    COL_HEIGHT = 13

    def __init__(
        self,
        title: str,
        add_button_text: str,
        remove_button_text: str,
        default_name_prefix: str,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._default_name_prefix = str(default_name_prefix).strip() or "Zone"
        self._rows: list[WorkspacePrimitiveZoneData] = []
        self._enabled_checkboxes: list[QCheckBox] = []
        self._type_combos: list[QComboBox] = []
        self.table: QTableWidget | None = None
        self._build_ui(title, add_button_text, remove_button_text)

    def _build_ui(self, title: str, add_button_text: str, remove_button_text: str) -> None:
        layout = QVBoxLayout(self)
        group = QGroupBox(title)
        group_layout = QVBoxLayout(group)

        self.table = QTableWidget(0, 14)
        self.table.setHorizontalHeaderLabels(
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
        self.table.horizontalHeader().setDefaultSectionSize(90)
        self.table.itemChanged.connect(self._on_table_item_changed)
        group_layout.addWidget(self.table)

        btn_row = QHBoxLayout()
        add_btn = QPushButton(add_button_text)
        add_btn.clicked.connect(self._on_add_clicked)
        btn_row.addWidget(add_btn)

        remove_btn = QPushButton(remove_button_text)
        remove_btn.clicked.connect(self._on_remove_clicked)
        btn_row.addWidget(remove_btn)
        btn_row.addStretch()
        group_layout.addLayout(btn_row)

        layout.addWidget(group)

    def set_zones(self, values: list[WorkspacePrimitiveZoneData] | list[dict]) -> None:
        if self.table is None:
            return
        normalized = parse_workspace_primitive_zones(values)
        self.table.blockSignals(True)
        try:
            self.table.setRowCount(0)
            self._enabled_checkboxes.clear()
            self._type_combos.clear()
            self._rows.clear()
            for value in normalized:
                self._insert_row(value)
        finally:
            self.table.blockSignals(False)

    def get_zones(self) -> list[WorkspacePrimitiveZoneData]:
        return [value.copy() for value in self._rows]

    def _on_table_item_changed(self, item: QTableWidgetItem) -> None:
        row = item.row()
        self._sync_row(row)
        if 0 <= row < len(self._rows):
            self.zone_changed.emit(row, self._rows[row].copy())
        self.zones_changed.emit(self.get_zones())

    def _on_add_clicked(self) -> None:
        row = self.table.rowCount() if self.table is not None else len(self._rows)
        value = WorkspacePrimitiveZoneData(
            name=f"{self._default_name_prefix} {row + 1}",
            enabled=True,
            shape="box",
            pose=[0.0] * 6,
            size_x=200.0,
            size_y=200.0,
            size_z=200.0,
            radius=100.0,
            height=200.0,
        )
        self._insert_row(value)
        self.zone_added.emit(len(self._rows) - 1, value.copy())
        self.zones_changed.emit(self.get_zones())

    def _on_remove_clicked(self) -> None:
        if self.table is None:
            return
        row = self.table.currentRow()
        if row < 0:
            row = self.table.rowCount() - 1
        if row < 0:
            return
        self.table.removeRow(row)
        if 0 <= row < len(self._enabled_checkboxes):
            self._enabled_checkboxes.pop(row)
        if 0 <= row < len(self._type_combos):
            self._type_combos.pop(row)
        if 0 <= row < len(self._rows):
            self._rows.pop(row)
        self.zone_removed.emit(row)
        self.zones_changed.emit(self.get_zones())

    def _insert_row(self, value: WorkspacePrimitiveZoneData) -> None:
        if self.table is None:
            return

        previous_block_state = self.table.blockSignals(True)
        row = self.table.rowCount()
        self.table.insertRow(row)

        enabled_checkbox = QCheckBox()
        enabled_checkbox.setChecked(value.enabled)
        enabled_checkbox.stateChanged.connect(
            lambda _state, checkbox=enabled_checkbox: self._on_enabled_changed(checkbox)
        )
        self.table.setCellWidget(row, self.COL_ENABLED, enabled_checkbox)
        self._enabled_checkboxes.append(enabled_checkbox)

        type_combo = QComboBox()
        type_combo.addItems(["box", "cylinder", "sphere"])
        type_combo.setCurrentText(value.shape)
        type_combo.currentIndexChanged.connect(
            lambda _idx, combo=type_combo: self._on_shape_changed(combo)
        )
        self.table.setCellWidget(row, self.COL_TYPE, type_combo)
        self._type_combos.append(type_combo)

        self.table.setItem(row, self.COL_NAME, QTableWidgetItem(value.name))
        self.table.setItem(row, self.COL_X, QTableWidgetItem(str(value.pose[0])))
        self.table.setItem(row, self.COL_Y, QTableWidgetItem(str(value.pose[1])))
        self.table.setItem(row, self.COL_Z, QTableWidgetItem(str(value.pose[2])))
        self.table.setItem(row, self.COL_A, QTableWidgetItem(str(value.pose[3])))
        self.table.setItem(row, self.COL_B, QTableWidgetItem(str(value.pose[4])))
        self.table.setItem(row, self.COL_C, QTableWidgetItem(str(value.pose[5])))
        self.table.setItem(row, self.COL_SIZE_X, QTableWidgetItem(str(value.size_x)))
        self.table.setItem(row, self.COL_SIZE_Y, QTableWidgetItem(str(value.size_y)))
        self.table.setItem(row, self.COL_SIZE_Z, QTableWidgetItem(str(value.size_z)))
        self.table.setItem(row, self.COL_RADIUS, QTableWidgetItem(str(value.radius)))
        self.table.setItem(row, self.COL_HEIGHT, QTableWidgetItem(str(value.height)))
        self.table.blockSignals(previous_block_state)
        self._rows.append(value.copy())

    def _on_enabled_changed(self, checkbox: QCheckBox) -> None:
        if checkbox not in self._enabled_checkboxes:
            return
        row = self._enabled_checkboxes.index(checkbox)
        self._sync_row(row)
        if 0 <= row < len(self._rows):
            self.zone_changed.emit(row, self._rows[row].copy())
        self.zones_changed.emit(self.get_zones())

    def _on_shape_changed(self, combo: QComboBox) -> None:
        if combo not in self._type_combos:
            return
        row = self._type_combos.index(combo)
        self._sync_row(row)
        if 0 <= row < len(self._rows):
            self.zone_changed.emit(row, self._rows[row].copy())
        self.zones_changed.emit(self.get_zones())

    def _sync_row(self, row: int) -> None:
        if self.table is None or not (0 <= row < self.table.rowCount()):
            return
        value = self._build_zone_from_row(row)
        if row < len(self._rows):
            self._rows[row] = value
        elif row == len(self._rows):
            self._rows.append(value)

    def _build_zone_from_row(self, row: int) -> WorkspacePrimitiveZoneData:
        if self.table is None:
            return WorkspacePrimitiveZoneData(name=f"{self._default_name_prefix} {row + 1}")

        enabled_widget = self.table.cellWidget(row, self.COL_ENABLED)
        enabled = bool(enabled_widget.isChecked()) if isinstance(enabled_widget, QCheckBox) else True

        shape_widget = self.table.cellWidget(row, self.COL_TYPE)
        shape = str(shape_widget.currentText()).strip().lower() if isinstance(shape_widget, QComboBox) else "box"

        name_item = self.table.item(row, self.COL_NAME)
        name = name_item.text().strip() if name_item is not None else f"{self._default_name_prefix} {row + 1}"
        if name == "":
            name = f"{self._default_name_prefix} {row + 1}"

        return WorkspacePrimitiveZoneData(
            name=name,
            enabled=enabled,
            shape=shape,
            pose=[
                self._cell_to_float(row, self.COL_X, 0.0),
                self._cell_to_float(row, self.COL_Y, 0.0),
                self._cell_to_float(row, self.COL_Z, 0.0),
                self._cell_to_float(row, self.COL_A, 0.0),
                self._cell_to_float(row, self.COL_B, 0.0),
                self._cell_to_float(row, self.COL_C, 0.0),
            ],
            size_x=max(0.0, self._cell_to_float(row, self.COL_SIZE_X, 200.0)),
            size_y=max(0.0, self._cell_to_float(row, self.COL_SIZE_Y, 200.0)),
            size_z=max(0.0, self._cell_to_float(row, self.COL_SIZE_Z, 200.0)),
            radius=max(0.0, self._cell_to_float(row, self.COL_RADIUS, 100.0)),
            height=max(0.0, self._cell_to_float(row, self.COL_HEIGHT, 200.0)),
        )

    def _cell_to_float(self, row: int, col: int, default: float = 0.0) -> float:
        if self.table is None:
            return default
        item = self.table.item(row, col)
        return safe_float(item.text() if item is not None else "", default)
