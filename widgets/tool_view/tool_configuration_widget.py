from __future__ import annotations

import os

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from models.primitive_collider_models import PrimitiveColliderData, PrimitiveColliderShape
from models.tool_config_file import ToolConfigFile
from models.types import Pose6
from utils.math_utils import safe_float
from utils.mgi import RobotTool


class ToolConfigurationWidget(QWidget):
    tool_changed = pyqtSignal(RobotTool)
    tool_cad_model_changed = pyqtSignal(str)
    tool_cad_offset_rz_changed = pyqtSignal(float)
    tool_colliders_changed = pyqtSignal(list)
    tool_evaluated_robot_axis_colliders_changed = pyqtSignal(list)
    tool_profiles_directory_changed = pyqtSignal(str)
    selected_tool_profile_changed = pyqtSignal(str)

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
    UNIT_DEG = "°"
    UNIT_MM = "mm"
    NUMERIC_VALUE_ROLE = Qt.ItemDataRole.UserRole

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.tool_cad_line_edit: QLineEdit | None = None
        self.tool_cad_offset_rz_spin: QDoubleSpinBox | None = None
        self.table_tool_colliders: QTableWidget | None = None
        self._tool_collider_type_combos: list[QComboBox] = []
        self._tool_collider_enabled_checkboxes: list[QCheckBox] = []
        self._tool_evaluated_robot_axis_colliders_checkboxes: list[QCheckBox] = []
        self.tool_profiles_dir_line_edit: QLineEdit | None = None
        self.tool_profiles_combo: QComboBox | None = None
        self.tool_name_line_edit: QLineEdit | None = None
        self._tool_profile_loading = False
        self._tool_profile_files: dict[str, str] = {}
        self._tool = RobotTool()
        self._spin_boxes: dict[str, QDoubleSpinBox] = {}
        self._setup_ui()
        self.set_tool_profiles_directory(self._default_tools_directory(), emit_change=False)
        self.set_tool_colliders([])
        self.set_tool_evaluated_robot_axis_colliders([True] * ToolConfigurationWidget.AXIS_COLLIDER_COUNT)

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        profiles_grid = QGridLayout()
        profiles_grid.addWidget(QLabel("Dossier tools"), 0, 0)
        self.tool_profiles_dir_line_edit = QLineEdit()
        self.tool_profiles_dir_line_edit.setReadOnly(True)
        profiles_grid.addWidget(self.tool_profiles_dir_line_edit, 0, 1)

        pick_tools_dir_btn = QPushButton("Sélectionner dossier")
        pick_tools_dir_btn.clicked.connect(self._on_pick_tool_profiles_directory)
        profiles_grid.addWidget(pick_tools_dir_btn, 0, 2)

        refresh_tools_btn = QPushButton("Rafraîchir")
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

        layout.addWidget(self._build_tool_pose_group())

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

        tool_cad_grid.addWidget(QLabel("Offset Rz tool"), 1, 0)
        self.tool_cad_offset_rz_spin = QDoubleSpinBox()
        self.tool_cad_offset_rz_spin.setRange(-360.0, 360.0)
        self.tool_cad_offset_rz_spin.setDecimals(2)
        self.tool_cad_offset_rz_spin.setSingleStep(1.0)
        self.tool_cad_offset_rz_spin.setSuffix(f" {ToolConfigurationWidget.UNIT_DEG}")
        self.tool_cad_offset_rz_spin.valueChanged.connect(self.tool_cad_offset_rz_changed.emit)
        tool_cad_grid.addWidget(self.tool_cad_offset_rz_spin, 1, 1)
        layout.addLayout(tool_cad_grid)

        layout.addWidget(self._build_tool_colliders_group())
        layout.addStretch()

    def _build_tool_pose_group(self) -> QGroupBox:
        group = QGroupBox("Configuration de l'outil")
        group_layout = QVBoxLayout(group)
        description = QLabel("Paramètres de transformation de l'outil par rapport au flange du robot")
        description.setWordWrap(True)
        group_layout.addWidget(description)

        inputs_layout = QHBoxLayout()
        trans_layout = QVBoxLayout()
        trans_layout.addWidget(QLabel("Translation :"))
        for axis in ["x", "y", "z"]:
            axis_layout = QHBoxLayout()
            axis_layout.addWidget(QLabel(f"{axis.upper()}:"))
            spin_box = QDoubleSpinBox()
            spin_box.setRange(-1000.0, 1000.0)
            spin_box.setSingleStep(0.1)
            spin_box.setDecimals(2)
            spin_box.setSuffix(f" {ToolConfigurationWidget.UNIT_MM}")
            spin_box.valueChanged.connect(lambda value, ax=axis: self._on_tool_param_changed(ax, value))
            self._spin_boxes[axis] = spin_box
            axis_layout.addWidget(spin_box)
            trans_layout.addLayout(axis_layout)
        inputs_layout.addLayout(trans_layout)

        rot_layout = QVBoxLayout()
        rot_layout.addWidget(QLabel("Rotation :"))
        for axis in ["a", "b", "c"]:
            axis_layout = QHBoxLayout()
            axis_layout.addWidget(QLabel(f"{axis.upper()}:"))
            spin_box = QDoubleSpinBox()
            spin_box.setRange(-180.0, 180.0)
            spin_box.setSingleStep(0.1)
            spin_box.setDecimals(2)
            spin_box.setSuffix(f" {ToolConfigurationWidget.UNIT_DEG}")
            spin_box.valueChanged.connect(lambda value, ax=axis: self._on_tool_param_changed(ax, value))
            self._spin_boxes[axis] = spin_box
            axis_layout.addWidget(spin_box)
            rot_layout.addLayout(axis_layout)
        inputs_layout.addLayout(rot_layout)

        reset_btn = QPushButton("Remettre à zéro")
        reset_btn.clicked.connect(self._reset_tool_to_identity)
        inputs_layout.addWidget(reset_btn)
        group_layout.addLayout(inputs_layout)
        return group

    def _build_tool_colliders_group(self) -> QGroupBox:
        tool_group = QGroupBox("Colliders de l'outil")
        tool_layout = QVBoxLayout(tool_group)

        evaluated_colliders_layout = QHBoxLayout()
        evaluated_colliders_layout.addWidget(QLabel("Colliders robot à évaluer pour ce tool"))
        self._tool_evaluated_robot_axis_colliders_checkboxes.clear()
        for axis in range(ToolConfigurationWidget.AXIS_COLLIDER_COUNT):
            checkbox = QCheckBox(f"J{axis + 1}")
            checkbox.setChecked(True)
            checkbox.stateChanged.connect(self._emit_tool_evaluated_robot_axis_colliders_changed)
            self._tool_evaluated_robot_axis_colliders_checkboxes.append(checkbox)
            evaluated_colliders_layout.addWidget(checkbox)
        evaluated_colliders_layout.addStretch()
        tool_layout.addLayout(evaluated_colliders_layout)

        self.table_tool_colliders = QTableWidget(0, 14)
        self.table_tool_colliders.setHorizontalHeaderLabels(
            ["Actif", "Nom", "Type", "X", "Y", "Z", "A", "B", "C", "Size X", "Size Y", "Size Z", "Rayon", "Hauteur"]
        )
        self.table_tool_colliders.horizontalHeader().setDefaultSectionSize(90)
        self.table_tool_colliders.itemChanged.connect(self._on_tool_colliders_item_changed)
        tool_layout.addWidget(self.table_tool_colliders)

        colliders_btn_layout = QHBoxLayout()
        add_tool_collider_btn = QPushButton("Ajouter")
        add_tool_collider_btn.clicked.connect(self._on_add_tool_collider_clicked)
        colliders_btn_layout.addWidget(add_tool_collider_btn)
        remove_tool_collider_btn = QPushButton("Supprimer")
        remove_tool_collider_btn.clicked.connect(self._on_remove_tool_collider_clicked)
        colliders_btn_layout.addWidget(remove_tool_collider_btn)
        colliders_btn_layout.addStretch()
        tool_layout.addLayout(colliders_btn_layout)
        return tool_group

    def _on_tool_param_changed(self, param: str, value: float) -> None:
        setattr(self._tool, param, value)
        self.tool_changed.emit(self._copy_tool(self._tool))

    def _reset_tool_to_identity(self) -> None:
        self._tool = RobotTool()
        for param in ["x", "y", "z", "a", "b", "c"]:
            self._spin_boxes[param].blockSignals(True)
            self._spin_boxes[param].setValue(0.0)
            self._spin_boxes[param].blockSignals(False)
        self.tool_changed.emit(self._copy_tool(self._tool))

    def _on_tool_colliders_item_changed(self, item: QTableWidgetItem) -> None:
        if item.column() in (
            ToolConfigurationWidget.COL_PRIM_X,
            ToolConfigurationWidget.COL_PRIM_Y,
            ToolConfigurationWidget.COL_PRIM_Z,
            ToolConfigurationWidget.COL_PRIM_SIZE_X,
            ToolConfigurationWidget.COL_PRIM_SIZE_Y,
            ToolConfigurationWidget.COL_PRIM_SIZE_Z,
            ToolConfigurationWidget.COL_PRIM_RADIUS,
            ToolConfigurationWidget.COL_PRIM_HEIGHT,
        ):
            self._format_table_item_with_unit(self.table_tool_colliders, item, ToolConfigurationWidget.UNIT_MM)
        elif item.column() in (
            ToolConfigurationWidget.COL_PRIM_A,
            ToolConfigurationWidget.COL_PRIM_B,
            ToolConfigurationWidget.COL_PRIM_C,
        ):
            self._format_table_item_with_unit(self.table_tool_colliders, item, ToolConfigurationWidget.UNIT_DEG)
        self.tool_colliders_changed.emit(self.get_tool_colliders())

    def _emit_tool_evaluated_robot_axis_colliders_changed(self) -> None:
        self.tool_evaluated_robot_axis_colliders_changed.emit(self.get_tool_evaluated_robot_axis_colliders())

    def _on_add_tool_collider_clicked(self) -> None:
        self._insert_tool_collider_row(
            PrimitiveColliderData(
                name=f"Tool collider {self.table_tool_colliders.rowCount() + 1 if self.table_tool_colliders is not None else 1}",
                enabled=True,
                shape=PrimitiveColliderShape.CYLINDER,
                pose=Pose6.zeros(),
                size_x=100.0,
                size_y=100.0,
                size_z=100.0,
                radius=40.0,
                height=120.0,
            )
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

    def _insert_tool_collider_row(self, collider: PrimitiveColliderData) -> None:
        if self.table_tool_colliders is None:
            return
        row = self.table_tool_colliders.rowCount()
        self.table_tool_colliders.insertRow(row)

        enabled_checkbox = QCheckBox()
        enabled_checkbox.setChecked(collider.enabled)
        enabled_checkbox.stateChanged.connect(lambda _state: self.tool_colliders_changed.emit(self.get_tool_colliders()))
        self.table_tool_colliders.setCellWidget(row, ToolConfigurationWidget.COL_PRIM_ENABLED, enabled_checkbox)
        self._tool_collider_enabled_checkboxes.append(enabled_checkbox)

        type_combo = QComboBox()
        type_combo.addItems(["box", "cylinder", "sphere"])
        type_combo.setCurrentText(collider.shape.value)
        type_combo.currentIndexChanged.connect(lambda _idx: self.tool_colliders_changed.emit(self.get_tool_colliders()))
        self.table_tool_colliders.setCellWidget(row, ToolConfigurationWidget.COL_PRIM_TYPE, type_combo)
        self._tool_collider_type_combos.append(type_combo)

        pose = collider.pose
        self.table_tool_colliders.setItem(row, ToolConfigurationWidget.COL_PRIM_NAME, QTableWidgetItem(collider.name))
        self._set_table_item_with_unit(
            self.table_tool_colliders,
            row,
            ToolConfigurationWidget.COL_PRIM_X,
            float(pose.x),
            ToolConfigurationWidget.UNIT_MM,
        )
        self._set_table_item_with_unit(
            self.table_tool_colliders,
            row,
            ToolConfigurationWidget.COL_PRIM_Y,
            float(pose.y),
            ToolConfigurationWidget.UNIT_MM,
        )
        self._set_table_item_with_unit(
            self.table_tool_colliders,
            row,
            ToolConfigurationWidget.COL_PRIM_Z,
            float(pose.z),
            ToolConfigurationWidget.UNIT_MM,
        )
        self._set_table_item_with_unit(
            self.table_tool_colliders,
            row,
            ToolConfigurationWidget.COL_PRIM_A,
            float(pose.a),
            ToolConfigurationWidget.UNIT_DEG,
        )
        self._set_table_item_with_unit(
            self.table_tool_colliders,
            row,
            ToolConfigurationWidget.COL_PRIM_B,
            float(pose.b),
            ToolConfigurationWidget.UNIT_DEG,
        )
        self._set_table_item_with_unit(
            self.table_tool_colliders,
            row,
            ToolConfigurationWidget.COL_PRIM_C,
            float(pose.c),
            ToolConfigurationWidget.UNIT_DEG,
        )
        self._set_table_item_with_unit(
            self.table_tool_colliders,
            row,
            ToolConfigurationWidget.COL_PRIM_SIZE_X,
            float(collider.size_x),
            ToolConfigurationWidget.UNIT_MM,
        )
        self._set_table_item_with_unit(
            self.table_tool_colliders,
            row,
            ToolConfigurationWidget.COL_PRIM_SIZE_Y,
            float(collider.size_y),
            ToolConfigurationWidget.UNIT_MM,
        )
        self._set_table_item_with_unit(
            self.table_tool_colliders,
            row,
            ToolConfigurationWidget.COL_PRIM_SIZE_Z,
            float(collider.size_z),
            ToolConfigurationWidget.UNIT_MM,
        )
        self._set_table_item_with_unit(
            self.table_tool_colliders,
            row,
            ToolConfigurationWidget.COL_PRIM_RADIUS,
            float(collider.radius),
            ToolConfigurationWidget.UNIT_MM,
        )
        self._set_table_item_with_unit(
            self.table_tool_colliders,
            row,
            ToolConfigurationWidget.COL_PRIM_HEIGHT,
            float(collider.height),
            ToolConfigurationWidget.UNIT_MM,
        )

    def _on_pick_tool_cad(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Sélectionner une CAD File pour le tool",
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

    def _on_pick_tool_profiles_directory(self) -> None:
        current_directory = self.get_tool_profiles_directory()
        start_directory = self._resolve_filesystem_path(current_directory) if current_directory else self._get_tools_start_directory()
        selected_dir = QFileDialog.getExistingDirectory(self, "Sélectionner le dossier des tools", start_directory)
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

    def _on_save_tool_profile(self) -> None:
        if self.tool_profiles_dir_line_edit is None:
            return
        tools_dir = self._resolve_filesystem_path(self.tool_profiles_dir_line_edit.text().strip())
        if not tools_dir:
            QMessageBox.information(self, "Dossier manquant", "Sélectionnez d'abord un dossier de tools.")
            return
        os.makedirs(tools_dir, exist_ok=True)
        raw_name = self.tool_name_line_edit.text().strip() if self.tool_name_line_edit is not None else ""
        if not raw_name:
            QMessageBox.information(self, "Nom manquant", "Saisissez un nom de tool avant d'enregistrer.")
            return
        if self._has_forbidden_filename_chars(raw_name):
            QMessageBox.warning(self, "Nom invalide", "Le nom du tool contient des caractères interdits pour un nom de fichier.")
            return
        safe_name = self._sanitize_tool_file_name(raw_name)
        if not safe_name:
            QMessageBox.warning(self, "Nom invalide", "Le nom du tool ne peut pas être utilisé comme nom de fichier.")
            return
        output_path = os.path.join(tools_dir, f"{safe_name}.json")
        profile = ToolConfigFile.from_robot_tool(
            raw_name,
            self.get_tool(),
            self.get_tool_cad_model(),
            self.get_tool_cad_offset_rz(),
            self.get_tool_colliders(),
            self.get_tool_evaluated_robot_axis_colliders(),
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

    @staticmethod
    def _get_cad_start_directory() -> str:
        current_dir = os.getcwd()
        return current_dir

    @staticmethod
    def _get_tools_start_directory() -> str:
        current_dir = os.getcwd()
        tools_dir = os.path.join(current_dir, "tools")
        if os.path.isdir(tools_dir):
            return tools_dir
        return current_dir

    @staticmethod
    def _sanitize_tool_file_name(name: str) -> str:
        forbidden = '<>:"/\\|?*'
        safe = name.replace(" ", "_")
        return "".join("_" if char in forbidden else char for char in safe).strip().strip(".")

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
        return "./user_data/tools"

    @staticmethod
    def _normalize_cad_path(file_path: str) -> str:
        return ToolConfigurationWidget._normalize_project_path(file_path)

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
    def _copy_tool(tool: RobotTool | None = None) -> RobotTool:
        source = tool if tool is not None else RobotTool()
        return RobotTool(float(source.x), float(source.y), float(source.z), float(source.a), float(source.b), float(source.c))

    @staticmethod
    def _copy_tool_evaluated_robot_axis_colliders(values: list[bool]) -> list[bool]:
        if len(values) != ToolConfigurationWidget.AXIS_COLLIDER_COUNT:
            raise ValueError("tool evaluated robot axis colliders must contain 6 values")
        return [bool(value) for value in values]

    @staticmethod
    def _format_value_with_unit(value: float | int | str, unit: str) -> str:
        return f"{value} {unit}"

    @staticmethod
    def _numeric_unit_suffixes() -> tuple[str, ...]:
        return (
            ToolConfigurationWidget.UNIT_DEG,
            ToolConfigurationWidget.UNIT_MM,
        )

    @staticmethod
    def _parse_table_numeric_text(text: str, default: float = 0.0) -> float:
        numeric_text = str(text).strip()
        if numeric_text == "":
            return default
        for unit in ToolConfigurationWidget._numeric_unit_suffixes():
            if numeric_text.endswith(unit):
                numeric_text = numeric_text[: -len(unit)].strip()
                break
        return safe_float(numeric_text, default)

    @staticmethod
    def _set_item_numeric_value(item: QTableWidgetItem, value: float) -> None:
        item.setData(ToolConfigurationWidget.NUMERIC_VALUE_ROLE, float(value))

    @staticmethod
    def _clear_item_numeric_value(item: QTableWidgetItem) -> None:
        item.setData(ToolConfigurationWidget.NUMERIC_VALUE_ROLE, None)

    @staticmethod
    def _make_table_item_with_unit(value: float | int | str, unit: str) -> QTableWidgetItem:
        item = QTableWidgetItem(ToolConfigurationWidget._format_value_with_unit(value, unit))
        raw_value = str(value).strip()
        if raw_value != "":
            numeric_value = ToolConfigurationWidget._parse_table_numeric_text(raw_value, 0.0)
            ToolConfigurationWidget._set_item_numeric_value(item, numeric_value)
        return item

    @staticmethod
    def _set_table_item_with_unit(
        table: QTableWidget,
        row: int,
        column: int,
        value: float | int | str,
        unit: str,
    ) -> None:
        table.setItem(row, column, ToolConfigurationWidget._make_table_item_with_unit(value, unit))

    @staticmethod
    def _format_table_item_with_unit(table: QTableWidget, item: QTableWidgetItem, unit: str) -> float:
        raw_text = item.text().strip()
        if raw_text == "":
            ToolConfigurationWidget._clear_item_numeric_value(item)
            return 0.0
        numeric_value = ToolConfigurationWidget._parse_table_numeric_text(raw_text, 0.0)
        table.blockSignals(True)
        try:
            item.setText(ToolConfigurationWidget._format_value_with_unit(numeric_value, unit))
            ToolConfigurationWidget._set_item_numeric_value(item, numeric_value)
        finally:
            table.blockSignals(False)
        return numeric_value

    @staticmethod
    def _cell_to_float(table: QTableWidget, row: int, column: int, default: float = 0.0) -> float:
        item = table.item(row, column)
        if item is None:
            return default
        numeric_data = item.data(ToolConfigurationWidget.NUMERIC_VALUE_ROLE)
        if numeric_data is not None:
            return safe_float(numeric_data, default)
        return ToolConfigurationWidget._parse_table_numeric_text(item.text(), default)

    def set_tool(self, tool: RobotTool) -> None:
        self._tool = self._copy_tool(tool)
        for param in ["x", "y", "z", "a", "b", "c"]:
            self._spin_boxes[param].blockSignals(True)
            self._spin_boxes[param].setValue(getattr(self._tool, param))
            self._spin_boxes[param].blockSignals(False)

    def get_tool(self) -> RobotTool:
        return self._copy_tool(self._tool)

    def set_tool_cad_model(self, tool_cad_model: str | None) -> None:
        if self.tool_cad_line_edit is not None:
            self.tool_cad_line_edit.setText("" if tool_cad_model is None else str(tool_cad_model))

    def get_tool_cad_model(self) -> str:
        return "" if self.tool_cad_line_edit is None else self.tool_cad_line_edit.text().strip()

    def set_tool_cad_offset_rz(self, offset_deg: float) -> None:
        if self.tool_cad_offset_rz_spin is not None:
            self.tool_cad_offset_rz_spin.blockSignals(True)
            self.tool_cad_offset_rz_spin.setValue(float(offset_deg))
            self.tool_cad_offset_rz_spin.blockSignals(False)

    def get_tool_cad_offset_rz(self) -> float:
        return 0.0 if self.tool_cad_offset_rz_spin is None else float(self.tool_cad_offset_rz_spin.value())

    def set_tool_colliders(self, tool_colliders: list[PrimitiveColliderData]) -> None:
        if self.table_tool_colliders is None:
            return
        normalized = [collider.copy() for collider in tool_colliders]
        self.table_tool_colliders.blockSignals(True)
        try:
            self.table_tool_colliders.setRowCount(0)
            self._tool_collider_enabled_checkboxes.clear()
            self._tool_collider_type_combos.clear()
            for collider in normalized:
                self._insert_tool_collider_row(collider)
        finally:
            self.table_tool_colliders.blockSignals(False)

    def get_tool_colliders(self) -> list[PrimitiveColliderData]:
        if self.table_tool_colliders is None:
            return []
        values: list[PrimitiveColliderData] = []
        for row in range(self.table_tool_colliders.rowCount()):
            enabled_widget = self.table_tool_colliders.cellWidget(row, ToolConfigurationWidget.COL_PRIM_ENABLED)
            enabled = bool(enabled_widget.isChecked()) if isinstance(enabled_widget, QCheckBox) else True
            shape_widget = self.table_tool_colliders.cellWidget(row, ToolConfigurationWidget.COL_PRIM_TYPE)
            shape = PrimitiveColliderShape(str(shape_widget.currentText()).strip().lower()) if isinstance(shape_widget, QComboBox) else PrimitiveColliderShape.CYLINDER
            pose = Pose6(
                self._cell_to_float(self.table_tool_colliders, row, ToolConfigurationWidget.COL_PRIM_X, 0.0),
                self._cell_to_float(self.table_tool_colliders, row, ToolConfigurationWidget.COL_PRIM_Y, 0.0),
                self._cell_to_float(self.table_tool_colliders, row, ToolConfigurationWidget.COL_PRIM_Z, 0.0),
                self._cell_to_float(self.table_tool_colliders, row, ToolConfigurationWidget.COL_PRIM_A, 0.0),
                self._cell_to_float(self.table_tool_colliders, row, ToolConfigurationWidget.COL_PRIM_B, 0.0),
                self._cell_to_float(self.table_tool_colliders, row, ToolConfigurationWidget.COL_PRIM_C, 0.0),
            )
            name_item = self.table_tool_colliders.item(row, ToolConfigurationWidget.COL_PRIM_NAME)
            name = name_item.text().strip() if name_item is not None else f"Tool collider {row + 1}"
            if name == "":
                name = f"Tool collider {row + 1}"
            values.append(
                PrimitiveColliderData(
                    name=name,
                    enabled=enabled,
                    shape=shape,
                    pose=pose,
                    size_x=max(0.0, self._cell_to_float(self.table_tool_colliders, row, ToolConfigurationWidget.COL_PRIM_SIZE_X, 100.0)),
                    size_y=max(0.0, self._cell_to_float(self.table_tool_colliders, row, ToolConfigurationWidget.COL_PRIM_SIZE_Y, 100.0)),
                    size_z=max(0.0, self._cell_to_float(self.table_tool_colliders, row, ToolConfigurationWidget.COL_PRIM_SIZE_Z, 100.0)),
                    radius=max(0.0, self._cell_to_float(self.table_tool_colliders, row, ToolConfigurationWidget.COL_PRIM_RADIUS, 40.0)),
                    height=max(0.0, self._cell_to_float(self.table_tool_colliders, row, ToolConfigurationWidget.COL_PRIM_HEIGHT, 120.0)),
                )
            )
        return values

    def set_tool_evaluated_robot_axis_colliders(self, values: list[bool]) -> None:
        normalized = self._copy_tool_evaluated_robot_axis_colliders(values)
        for axis, checkbox in enumerate(self._tool_evaluated_robot_axis_colliders_checkboxes):
            checkbox.blockSignals(True)
            checkbox.setChecked(normalized[axis])
            checkbox.blockSignals(False)

    def get_tool_evaluated_robot_axis_colliders(self) -> list[bool]:
        if not self._tool_evaluated_robot_axis_colliders_checkboxes:
            return [True] * ToolConfigurationWidget.AXIS_COLLIDER_COUNT
        return [checkbox.isChecked() for checkbox in self._tool_evaluated_robot_axis_colliders_checkboxes[:6]]

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
