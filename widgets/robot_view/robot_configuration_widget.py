from __future__ import annotations

import math
import os

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
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
    QCheckBox,
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

    axis_config_changed = pyqtSignal(list, list, list, list)
    positions_config_changed = pyqtSignal(list, list, list)
    position_zero_requested = pyqtSignal()
    position_transport_requested = pyqtSignal()
    home_position_requested = pyqtSignal()

    robot_cad_models_changed = pyqtSignal(list)
    tool_cad_model_changed = pyqtSignal(str)
    tool_cad_offset_rz_changed = pyqtSignal(float)
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

    ROBOT_CAD_COUNT = 7

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.axis_reversed_checkboxes: list[QCheckBox] = []
        self.robot_cad_line_edits: list[QLineEdit] = []
        self.tool_cad_line_edit: QLineEdit | None = None
        self.tool_cad_offset_rz_spin: QDoubleSpinBox | None = None
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
        self.tabs.addTab(self._build_positions_tab(), "Parametrage des positions")
        self.tabs.addTab(self._build_cad_tab(), "CAO")
        top_layout.addWidget(self.tabs)

        main_layout.addLayout(top_layout, 3)
        main_layout.addWidget(self._build_tool_section(), 2)

        self.tool_widget = ToolWidget()
        self.tool_widget.tool_changed.connect(self._on_tool_changed)
        self.tool_widget_container_layout.addWidget(self.tool_widget)
        self.set_tool_profiles_directory(self._default_tools_directory(), emit_change=False)

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

        pick_tools_dir_btn = QPushButton("Selectionner dossier")
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
        return group

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

        self.robot_cad_line_edits[index].setText(self._normalize_cad_path(file_path))
        self.robot_cad_models_changed.emit(self.get_robot_cad_models())

    def _on_pick_multiple_robot_cad(self) -> None:
        file_paths, _ = QFileDialog.getOpenFileNames(
            self,
            "Selectionner plusieurs CAO robot",
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
            "Selectionner une CAO de tool",
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
            "Selectionner le dossier des tools",
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
            self._apply_no_tool_profile(emit_signals=True)
            self.selected_tool_profile_changed.emit("")
            return
        if self._load_tool_profile(str(file_path)):
            self.selected_tool_profile_changed.emit(self._normalize_project_path(str(file_path)))
            return

        self._apply_no_tool_profile(emit_signals=True)
        self.selected_tool_profile_changed.emit("")

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

    @staticmethod
    def _get_tools_start_directory() -> str:
        current_dir = os.getcwd()
        tools_dir = os.path.join(current_dir, "configurations", "tools")
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

    def set_tool_profiles_directory(self, directory: str | None, emit_change: bool = False) -> None:
        if self.tool_profiles_dir_line_edit is None:
            return
        normalized = "" if directory is None else str(directory).strip()
        if not normalized:
            normalized = self._default_tools_directory()
        normalized = self._normalize_project_path(normalized)
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
        if not target:
            self._tool_profile_loading = True
            self.tool_profiles_combo.setCurrentIndex(0)
            self._tool_profile_loading = False
            self._apply_no_tool_profile(emit_signals=True)
            self.selected_tool_profile_changed.emit("")
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

        if target_index < 0:
            self._tool_profile_loading = True
            self.tool_profiles_combo.setCurrentIndex(0)
            self._tool_profile_loading = False
            self._apply_no_tool_profile(emit_signals=True)
            self.selected_tool_profile_changed.emit("")
            return

        self._tool_profile_loading = True
        self.tool_profiles_combo.setCurrentIndex(target_index)
        self._tool_profile_loading = False
        item_data = self.tool_profiles_combo.itemData(target_index)
        if item_data and self._load_tool_profile(str(item_data)):
            self.selected_tool_profile_changed.emit(self._normalize_project_path(str(item_data)))
            return

        self._tool_profile_loading = True
        self.tool_profiles_combo.setCurrentIndex(0)
        self._tool_profile_loading = False
        self._apply_no_tool_profile(emit_signals=True)
        self.selected_tool_profile_changed.emit("")

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
