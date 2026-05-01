from typing import Dict, List, Optional, Any
from PyQt6.QtWidgets import (
    QLayout, QWidget, QVBoxLayout, QGridLayout, QLabel,
    QPushButton, QLineEdit, QTreeWidget, QTreeWidgetItem,
    QTableWidget, QTableWidgetItem, QAbstractItemView, QComboBox, QSizePolicy, QHBoxLayout, QCheckBox, QHeaderView,
    QGroupBox
)
from PyQt6.QtCore import pyqtSignal, Qt, QEvent
from PyQt6.QtGui import QFont
import utils.math_utils as math_utils
import numpy as np


class DHCellWidget(QWidget):
    """Widget personnalise pour une cellule DH : checkbox + valeur en disposition horizontale"""

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        self._raw_value = ""
        self._unit_suffix = ""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(4)

        self.checkbox = QCheckBox()
        layout.addWidget(self.checkbox)

        self.label = QLabel("")
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.label)

        self.checkbox.toggled.connect(self._update_label_style)
        self.label.installEventFilter(self)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.label.setCursor(Qt.CursorShape.PointingHandCursor)
        self._update_label_style(self.checkbox.isChecked())

        self.setLayout(layout)

    def set_value(self, value: str) -> None:
        self._raw_value = value
        self._refresh_label_text()

    def get_value(self) -> str:
        return self._raw_value

    def set_unit_suffix(self, unit_suffix: str) -> None:
        self._unit_suffix = unit_suffix
        self._refresh_label_text()

    def set_checked(self, checked: bool) -> None:
        self.checkbox.setChecked(checked)

    def is_checked(self) -> bool:
        return self.checkbox.isChecked()

    def set_enabled_state(self, enabled: bool) -> None:
        self.setEnabled(enabled)

    def _update_label_style(self, checked: bool) -> None:
        if checked:
            self.label.setStyleSheet("color: #ff8c00; font-weight: 700;")
        else:
            self.label.setStyleSheet("")

    def _refresh_label_text(self) -> None:
        if self._raw_value and self._unit_suffix:
            self.label.setText(f"{self._raw_value} {self._unit_suffix}")
            return
        self.label.setText(self._raw_value)

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton and self.isEnabled():
            self.checkbox.setChecked(not self.checkbox.isChecked())
            event.accept()
            return
        super().mousePressEvent(event)

    def eventFilter(self, obj, event):
        if obj is self.label and event.type() == QEvent.Type.MouseButtonPress:
            if self.isEnabled() and event.button() == Qt.MouseButton.LeftButton:
                self.checkbox.setChecked(not self.checkbox.isChecked())
                return True
        return super().eventFilter(obj, event)


class MeasurementWidget(QWidget):
    """Widget pour l'importation et la gestion des mesures"""

    import_measurements_requested = pyqtSignal()
    clear_measurements_requested = pyqtSignal()
    set_as_reference_requested = pyqtSignal()
    apply_measured_dh_requested = pyqtSignal()
    repere_selected = pyqtSignal(str)
    rotation_type_changed = pyqtSignal(str)
    dh_checkboxes_changed = pyqtSignal()
    corrections_changed = pyqtSignal(list)

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.measurements: List[Dict[str, Any]] = []
        self.setup_ui()

    def setup_ui(self) -> None:
        """Initialise l'interface du widget"""
        main_layout = QVBoxLayout(self)

        import_group = QGroupBox("Import des mesures")
        import_layout = QVBoxLayout(import_group)

        top_layout = QGridLayout()

        self.lineEdit_measure_filename = QLineEdit()
        self.lineEdit_measure_filename.setReadOnly(False)
        self.lineEdit_measure_filename.setPlaceholderText("Fichier de mesure")
        top_layout.addWidget(self.lineEdit_measure_filename, 0, 0)

        label_2 = QLabel("Convention d'angles : ")
        label_2.setAlignment(Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter)
        self.rotation_type = QComboBox()
        self.rotation_type.addItems(["XYZ Fixed Angles", "XYZ Euler Angles", "ZYX Fixed Angles", "ZYX Euler Angles"])
        self.rotation_type.currentTextChanged.connect(self.rotation_type_changed)
        top_layout.addWidget(label_2, 0, 3)
        top_layout.addWidget(self.rotation_type, 0, 4)

        top_layout.setColumnStretch(0, 2)
        top_layout.setColumnStretch(1, 1)
        top_layout.setColumnStretch(2, 1)
        top_layout.setColumnStretch(3, 1)
        top_layout.setColumnStretch(4, 1)

        import_layout.addLayout(top_layout)

        middle_layout = QHBoxLayout()

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Repères"])
        self.tree.itemClicked.connect(self._on_item_clicked)
        middle_layout.addWidget(self.tree, 1)
        middle_layout.setStretch(0, 1)

        self.table_me = QTableWidget(5, 3)
        self.table_me.setHorizontalHeaderLabels(["X", "Y", "Z"])
        self.table_me.setVerticalHeaderLabels(["Translation (mm)", "Rotation (°)", "X axis", "Y axis", "Z axis"])
        self.table_me.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_me.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_me.horizontalHeader().setDefaultAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_me.verticalHeader().setDefaultAlignment(Qt.AlignmentFlag.AlignCenter)
        middle_layout.addWidget(self.table_me, 1)
        middle_layout.setStretch(1, 2)

        import_layout.addLayout(middle_layout)

        buttons_layout = QHBoxLayout()

        self.btn_import_me = QPushButton("Importer")
        self.btn_import_me.clicked.connect(self.import_measurements_requested.emit)
        buttons_layout.addWidget(self.btn_import_me)

        self.btn_set_as_ref = QPushButton("Définir en Référence")
        self.btn_set_as_ref.clicked.connect(self.set_as_reference_requested.emit)
        self.btn_set_as_ref.setEnabled(False)
        buttons_layout.addWidget(self.btn_set_as_ref)

        self.btn_clear = QPushButton("Effacer")
        self.btn_clear.clicked.connect(self.clear_measurements_requested.emit)
        self.btn_clear.setEnabled(False)
        buttons_layout.addWidget(self.btn_clear)

        import_layout.addLayout(buttons_layout)
        main_layout.addWidget(import_group)

        dh_group = QGroupBox("Paramètres DHM mesurés")
        dh_group_layout = QVBoxLayout(dh_group)

        self.table_dh_measured = QTableWidget(6, 4)
        self.table_dh_measured.setHorizontalHeaderLabels(["alpha", "d", "theta", "r"])
        self.table_dh_measured.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_dh_measured.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh_measured.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh_measured.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.table_dh_measured.horizontalHeader().setDefaultAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_dh_measured.verticalHeader().setDefaultAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_dh_measured.horizontalHeader().setDefaultSectionSize(120)
        self.table_dh_measured.setEnabled(False)
        self.table_dh_measured.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        self.table_tcp_offsets = QTableWidget(4, 1)
        self.table_tcp_offsets.setHorizontalHeaderLabels(["Impact sur TCP"])
        self.table_tcp_offsets.setVerticalHeaderLabels(["X", "Y", "Z", "3D"])
        self.table_tcp_offsets.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_tcp_offsets.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_tcp_offsets.horizontalHeader().setDefaultAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_tcp_offsets.verticalHeader().setDefaultAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_tcp_offsets.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.table_tcp_offsets.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table_tcp_offsets.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.table_tcp_offsets.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.table_tcp_offsets.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

        self.checkbox_segment_lengths: QCheckBox | None = None
        self.checkbox_axis_origins: QCheckBox | None = None
        self.checkbox_parallelism: QCheckBox | None = None
        self._updating_corrections_from_model = False

        self._initialize_dh_cells()
        self._freeze_dh_table_height()

        dh_tables_layout = QHBoxLayout()
        dh_tables_layout.addWidget(self.table_dh_measured, 1)
        tcp_side_layout = QVBoxLayout()
        tcp_side_layout.setSizeConstraint(QLayout.SizeConstraint.SetFixedSize)
        tcp_side_layout.addWidget(self.table_tcp_offsets, 0, Qt.AlignmentFlag.AlignLeft)

        self.checkbox_segment_lengths = QCheckBox("Longueurs de segments")
        self.checkbox_segment_lengths.toggled.connect(
            lambda checked: self._set_dh_group_checked([1, 3], checked)
        )
        self.checkbox_segment_lengths.setEnabled(False)
        tcp_side_layout.addWidget(self.checkbox_segment_lengths, 0, Qt.AlignmentFlag.AlignLeft)

        self.checkbox_axis_origins = QCheckBox("Origines d'axes")
        self.checkbox_axis_origins.toggled.connect(
            lambda checked: self._set_dh_group_checked([2], checked)
        )
        self.checkbox_axis_origins.setEnabled(False)
        tcp_side_layout.addWidget(self.checkbox_axis_origins, 0, Qt.AlignmentFlag.AlignLeft)

        self.checkbox_parallelism = QCheckBox("Parallélisme")
        self.checkbox_parallelism.toggled.connect(
            lambda checked: self._set_dh_group_checked([0], checked)
        )
        self.checkbox_parallelism.setEnabled(False)
        tcp_side_layout.addWidget(self.checkbox_parallelism, 0, Qt.AlignmentFlag.AlignLeft)
        tcp_side_layout.addStretch(1)

        dh_tables_layout.addSpacing(12)
        dh_tables_layout.addLayout(tcp_side_layout, 0)
        dh_tables_layout.setAlignment(tcp_side_layout, Qt.AlignmentFlag.AlignLeft)
        dh_group_layout.addLayout(dh_tables_layout)

        self._initialize_tcp_offsets_table()
        main_layout.addWidget(dh_group)

        correction_group = QGroupBox("Correction 6D")
        correction_layout = QVBoxLayout(correction_group)
        self.table_corr = QTableWidget(6, 6)
        self.table_corr.setEditTriggers(QAbstractItemView.EditTrigger.AllEditTriggers)
        self.table_corr.setHorizontalHeaderLabels(["Tx(mm)", "Ty(mm)", "Tz(mm)", "Rx(°)", "Ry(°)", "Rz(°)"])
        self.table_corr.horizontalHeader().setDefaultSectionSize(80)
        self.table_corr.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_corr.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_corr.itemChanged.connect(self._on_correction_item_changed)
        buttons2_layout = QHBoxLayout()
        self.btn_toggle_check = QPushButton("Tout sélectionner")
        self.btn_toggle_check.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.btn_toggle_check.clicked.connect(self.toggle_dh_checkboxes)
        self.btn_toggle_check.setEnabled(False)
        buttons2_layout.addWidget(self.btn_toggle_check, 1)

        self.btn_apply_measured = QPushButton("Appliquer à la configuration robot")
        self.btn_apply_measured.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.btn_apply_measured.clicked.connect(self.apply_measured_dh_requested.emit)
        self.btn_apply_measured.setEnabled(False)
        buttons2_layout.addWidget(self.btn_apply_measured, 1)

        dh_group_layout.addLayout(buttons2_layout)
        correction_layout.addWidget(self.table_corr)
        main_layout.addWidget(correction_group)
        self.setLayout(main_layout)

    def _is_dh_cell_disabled(self, row: int, col: int) -> bool:
        # Existing locked cells
        if row == 0 or (row == 1 and col == 3):
            return True

        # Requested locked cells:
        # q5 -> d, r (row 4 -> col 1, 3)
        # q6 -> d, theta, r (row 5 -> col 1, 2, 3)
        if row == 4 and col in (1, 3):
            return True
        if row == 5 and col in (1, 2, 3):
            return True

        return False

    def _set_dh_group_checked(self, columns: List[int], checked: bool) -> None:
        for row in range(self.table_dh_measured.rowCount()):
            for col in columns:
                cell_widget = self.table_dh_measured.cellWidget(row, col)
                if isinstance(cell_widget, DHCellWidget) and cell_widget.isEnabled():
                    cell_widget.set_checked(checked)
        self._sync_group_checkboxes()
        self._update_toggle_check_button_text()
        self._update_apply_button_state()
        self.dh_checkboxes_changed.emit()

    def _is_dh_group_fully_checked(self, columns: List[int]) -> bool:
        has_enabled_cell = False
        for row in range(self.table_dh_measured.rowCount()):
            for col in columns:
                cell_widget = self.table_dh_measured.cellWidget(row, col)
                if isinstance(cell_widget, DHCellWidget) and cell_widget.isEnabled():
                    has_enabled_cell = True
                    if not cell_widget.is_checked():
                        return False
        return has_enabled_cell

    def _set_group_checkbox_state(self, checkbox: QCheckBox, checked: bool) -> None:
        checkbox.blockSignals(True)
        checkbox.setChecked(checked)
        checkbox.blockSignals(False)

    def _sync_group_checkboxes(self) -> None:
        if (
            self.checkbox_segment_lengths is None
            or self.checkbox_axis_origins is None
            or self.checkbox_parallelism is None
        ):
            return
        self._set_group_checkbox_state(
            self.checkbox_segment_lengths,
            self._is_dh_group_fully_checked([1, 3]),
        )
        self._set_group_checkbox_state(
            self.checkbox_axis_origins,
            self._is_dh_group_fully_checked([2]),
        )
        self._set_group_checkbox_state(
            self.checkbox_parallelism,
            self._is_dh_group_fully_checked([0]),
        )

    def _get_dh_column_unit_suffix(self, col: int) -> str:
        if col in (0, 2):
            return "°"
        if col in (1, 3):
            return "mm"
        return ""

    def _on_item_clicked(self, item: QTreeWidgetItem, column: int) -> None:
        self.repere_selected.emit(item.text(0))

    def _make_centered_item(self, value: str) -> QTableWidgetItem:
        item = QTableWidgetItem(value)
        item.setTextAlignment(int(Qt.AlignmentFlag.AlignCenter))
        return item

    def _rotation_matrix_to_display_angles(self, R: np.ndarray) -> np.ndarray:
        rotation_type = self.rotation_type.currentText()

        if rotation_type == "XYZ Fixed Angles":
            return np.asarray(math_utils.rotation_matrix_to_fixed_xyz(R), dtype=float)
        if rotation_type == "XYZ Euler Angles":
            return np.asarray(math_utils.rotation_matrix_to_euler_xyz(R), dtype=float)
        if rotation_type == "ZYX Fixed Angles":
            return np.asarray(math_utils.rotation_matrix_to_fixed_zyx(R), dtype=float)
        return np.asarray(math_utils.rotation_matrix_to_euler_zyx(R), dtype=float)

    def _format_value(self, value: float, decimals: int = 4) -> str:
        formatted = f"{value:.{decimals}f}"
        return f"0.{'0' * decimals}" if formatted == f"-0.{'0' * decimals}" else formatted

    def _format_value_with_unit(self, value: float, unit_suffix: str, decimals: int = 4) -> str:
        formatted_value = self._format_value(value, decimals)
        if not unit_suffix:
            return formatted_value
        return f"{formatted_value} {unit_suffix}"

    def populate_tree(self, repere_names: List[str]) -> None:
        self.tree.clear()
        for name in repere_names:
            item = QTreeWidgetItem([name])
            self.tree.addTopLevelItem(item)

    def set_measurements_data(self, measurements: List[Dict[str, Any]]) -> None:
        self.measurements = measurements

    def set_reference_bold(self, ref_name: str) -> None:
        for i in range(self.tree.topLevelItemCount()):
            item = self.tree.topLevelItem(i)
            font = QFont()
            font.setBold(item.text(0) == ref_name)
            item.setFont(0, font)

    def display_repere_data(self, delta_T: np.ndarray) -> None:
        """
        Affiche les écarts X, Y, Z, RX, RY, RZ calculés à partir de delta_T dans table_me.
        delta_T : matrice homogène 4x4 (numpy array)
        """
        self.table_me.blockSignals(True)

        X = delta_T[0, 3]
        Y = delta_T[1, 3]
        Z = delta_T[2, 3]

        R = delta_T[:3, :3]
        angles = self._rotation_matrix_to_display_angles(R)
        RX, RY, RZ = float(angles[0]), float(angles[1]), float(angles[2])

        self.table_me.setItem(0, 0, self._make_centered_item(self._format_value(X, 4)))
        self.table_me.setItem(0, 1, self._make_centered_item(self._format_value(Y, 4)))
        self.table_me.setItem(0, 2, self._make_centered_item(self._format_value(Z, 4)))

        self.table_me.setItem(1, 0, self._make_centered_item(self._format_value(RX, 4)))
        self.table_me.setItem(1, 1, self._make_centered_item(self._format_value(RY, 4)))
        self.table_me.setItem(1, 2, self._make_centered_item(self._format_value(RZ, 4)))

        for i in range(3):
            for j in range(3):
                self.table_me.setItem(2 + i, j, self._make_centered_item(self._format_value(delta_T[i, j], 6)))

        self.table_me.blockSignals(False)

    def display_measurement(self, measurement: Dict[str, Any]) -> None:
        """
        Affiche les données d'une mesure dans la table.
        La matrice homogène 4x4 'T' est utilisée en priorité.
        """
        self.table_me.blockSignals(True)

        T = measurement.get("T")
        if isinstance(T, np.ndarray) and T.shape == (4, 4):
            X = float(T[0, 3])
            Y = float(T[1, 3])
            Z = float(T[2, 3])
            R = T[:3, :3]
        else:
            X = measurement.get("X", 0)
            Y = measurement.get("Y", 0)
            Z = measurement.get("Z", 0)
            A = measurement.get("A", 0)
            B = measurement.get("B", 0)
            C = measurement.get("C", 0)

            if "R" in measurement and isinstance(measurement["R"], np.ndarray):
                R = measurement["R"]
            else:
                R = math_utils.euler_to_rotation_matrix(A, B, C, degrees=True)

        angles = self._rotation_matrix_to_display_angles(R)
        A = float(angles[0])
        B = float(angles[1])
        C = float(angles[2])

        self.table_me.setItem(0, 0, self._make_centered_item(self._format_value(X, 4)))
        self.table_me.setItem(0, 1, self._make_centered_item(self._format_value(Y, 4)))
        self.table_me.setItem(0, 2, self._make_centered_item(self._format_value(Z, 4)))

        self.table_me.setItem(1, 0, self._make_centered_item(self._format_value(A, 4)))
        self.table_me.setItem(1, 1, self._make_centered_item(self._format_value(B, 4)))
        self.table_me.setItem(1, 2, self._make_centered_item(self._format_value(C, 4)))

        x_axis = R[:, 0]
        y_axis = R[:, 1]
        z_axis = R[:, 2]

        self.table_me.setItem(2, 0, self._make_centered_item(self._format_value(x_axis[0], 6)))
        self.table_me.setItem(2, 1, self._make_centered_item(self._format_value(x_axis[1], 6)))
        self.table_me.setItem(2, 2, self._make_centered_item(self._format_value(x_axis[2], 6)))

        self.table_me.setItem(3, 0, self._make_centered_item(self._format_value(y_axis[0], 6)))
        self.table_me.setItem(3, 1, self._make_centered_item(self._format_value(y_axis[1], 6)))
        self.table_me.setItem(3, 2, self._make_centered_item(self._format_value(y_axis[2], 6)))

        self.table_me.setItem(4, 0, self._make_centered_item(self._format_value(z_axis[0], 6)))
        self.table_me.setItem(4, 1, self._make_centered_item(self._format_value(z_axis[1], 6)))
        self.table_me.setItem(4, 2, self._make_centered_item(self._format_value(z_axis[2], 6)))

        self.table_me.blockSignals(False)

    def clear_measurements(self) -> None:
        self.lineEdit_measure_filename.clear()
        self.tree.clear()
        self.table_me.clearContents()
        self.table_dh_measured.clearContents()
        self._initialize_dh_cells()
        self._initialize_tcp_offsets_table()
        self._sync_group_checkboxes()

    def set_measure_filename(self, filename) -> None:
        self.lineEdit_measure_filename.setText(filename)

    def get_current_repere_name(self) -> Optional[str]:
        current_item = self.tree.currentItem()
        return current_item.text(0) if current_item else None

    def display_dh_measured(self, dh_matrix: np.ndarray) -> None:
        """
        Affiche une matrice 4x4 homogène dans la table DH mesurée.
        Affiche uniquement les 3x4 premiers éléments.
        """
        self.table_dh_measured.blockSignals(True)

        for i in range(3):
            for j in range(4):
                cell_widget = self.table_dh_measured.cellWidget(i, j)
                if isinstance(cell_widget, DHCellWidget):
                    cell_widget.set_unit_suffix(self._get_dh_column_unit_suffix(j))
                    cell_widget.set_value(self._format_value(dh_matrix[i, j], 4))

        for i in range(3, 6):
            for j in range(4):
                cell_widget = self.table_dh_measured.cellWidget(i, j)
                if isinstance(cell_widget, DHCellWidget):
                    cell_widget.set_unit_suffix(self._get_dh_column_unit_suffix(j))
                    cell_widget.set_value("")

        self.table_dh_measured.blockSignals(False)

    def populate_dh_measured_deviations(self, dh_deviations: List[Dict[str, float]]) -> None:
        """
        Remplit la table DH Mesuree avec les parametres DH mesures.
        Les valeurs sont affichees comme : alpha, d, theta, r
        """
        self.table_dh_measured.blockSignals(True)
        self.table_dh_measured.clearContents()

        for row in range(6):
            if row < len(dh_deviations):
                deviation = dh_deviations[row]

                for col, param_key in enumerate(["alpha", "d", "theta", "r"]):
                    value = float(deviation.get(param_key, 0))
                    formatted_value = self._format_value(value, 4)

                    cell_widget = DHCellWidget()
                    if self._is_dh_cell_disabled(row, col):
                        cell_widget.set_enabled_state(False)
                    cell_widget.checkbox.stateChanged.connect(self._emit_dh_checkboxes_changed)
                    cell_widget.set_unit_suffix(self._get_dh_column_unit_suffix(col))
                    cell_widget.set_value(formatted_value)
                    self.table_dh_measured.setCellWidget(row, col, cell_widget)
            else:
                for col in range(4):
                    cell_widget = DHCellWidget()
                    if self._is_dh_cell_disabled(row, col):
                        cell_widget.set_enabled_state(False)
                    cell_widget.checkbox.stateChanged.connect(self._emit_dh_checkboxes_changed)
                    cell_widget.set_unit_suffix(self._get_dh_column_unit_suffix(col))
                    cell_widget.set_value("")
                    self.table_dh_measured.setCellWidget(row, col, cell_widget)

        for row in range(6):
            self.table_dh_measured.setRowHeight(row, 35)

        self.table_dh_measured.update()
        self.table_dh_measured.blockSignals(False)
        self._sync_group_checkboxes()
        self.dh_checkboxes_changed.emit()

        print(f"Table DH Mesuree remplie avec {len([d for d in dh_deviations if d])} articulations")

    def _set_table_read_only(self) -> None:
        for row in range(self.table_me.rowCount()):
            for col in range(self.table_me.columnCount()):
                item = self.table_me.item(row, col)
                if item:
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)

        for row in range(self.table_dh_measured.rowCount()):
            for col in range(self.table_dh_measured.columnCount()):
                item = self.table_dh_measured.item(row, col)
                if item:
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)

    def _initialize_dh_cells(self) -> None:
        for row in range(self.table_dh_measured.rowCount()):
            self.table_dh_measured.setRowHeight(row, 35)
            for col in range(self.table_dh_measured.columnCount()):
                cell_widget = DHCellWidget()
                if self._is_dh_cell_disabled(row, col):
                    cell_widget.set_enabled_state(False)
                cell_widget.checkbox.stateChanged.connect(self._emit_dh_checkboxes_changed)
                cell_widget.set_unit_suffix(self._get_dh_column_unit_suffix(col))
                self.table_dh_measured.setCellWidget(row, col, cell_widget)
        self._sync_group_checkboxes()

    def _freeze_dh_table_height(self) -> None:
        header_height = self.table_dh_measured.horizontalHeader().height()
        rows_height = sum(self.table_dh_measured.rowHeight(row) for row in range(self.table_dh_measured.rowCount()))
        frame_height = 2 * self.table_dh_measured.frameWidth()
        self.table_dh_measured.setFixedHeight(header_height + rows_height + frame_height)

    def _initialize_tcp_offsets_table(self) -> None:
        self.table_tcp_offsets.blockSignals(True)
        self.table_tcp_offsets.clearContents()
        for row in range(self.table_tcp_offsets.rowCount()):
            self.table_tcp_offsets.setItem(
                row,
                0,
                self._make_centered_item(self._format_value_with_unit(0.0, "mm", 2)),
            )
        self.table_tcp_offsets.blockSignals(False)
        self._update_tcp_offsets_table_geometry()

    def set_tcp_offsets_values(
        self,
        offsets_xyz: List[float],
        offset_3d_mm: float,
    ) -> None:
        offset_values_mm = [float(v) for v in offsets_xyz[:3]]
        while len(offset_values_mm) < 3:
            offset_values_mm.append(0.0)
        offset_values_mm.append(float(offset_3d_mm))

        self.table_tcp_offsets.blockSignals(True)
        for row in range(4):
            value = float(offset_values_mm[row]) if row < len(offset_values_mm) else 0.0
            display_value = self._format_value_with_unit(value, "mm", 2)
            self.table_tcp_offsets.setItem(row, 0, self._make_centered_item(display_value))
        self.table_tcp_offsets.blockSignals(False)
        self._update_tcp_offsets_table_geometry()

    def _update_tcp_offsets_table_geometry(self) -> None:
        self.table_tcp_offsets.resizeRowsToContents()
        header_width = self.table_tcp_offsets.verticalHeader().width()
        column_width = max(
            self.table_tcp_offsets.sizeHintForColumn(0),
            self.checkbox_segment_lengths.sizeHint().width() if self.checkbox_segment_lengths is not None else 0,
            self.checkbox_axis_origins.sizeHint().width() if self.checkbox_axis_origins is not None else 0,
            self.checkbox_parallelism.sizeHint().width() if self.checkbox_parallelism is not None else 0,
        )
        frame_width = 2 * self.table_tcp_offsets.frameWidth()
        extra_padding = 8
        total_width = header_width + column_width + frame_width + extra_padding
        header_height = self.table_tcp_offsets.horizontalHeader().height()
        rows_height = sum(
            self.table_tcp_offsets.rowHeight(row)
            for row in range(self.table_tcp_offsets.rowCount())
        )
        total_height = header_height + rows_height + frame_width
        self.table_tcp_offsets.setFixedWidth(total_width)
        self.table_tcp_offsets.setFixedHeight(total_height)

    def get_dh_checkboxes_state(self) -> Dict[str, bool]:
        states = {}
        col_names = ["alpha", "d", "theta", "r"]

        for row in range(self.table_dh_measured.rowCount()):
            joint_num = row + 1
            for col in range(self.table_dh_measured.columnCount()):
                param_name = f"{col_names[col]}{joint_num}"
                cell_widget = self.table_dh_measured.cellWidget(row, col)
                if isinstance(cell_widget, DHCellWidget):
                    states[param_name] = cell_widget.is_checked()

        return states

    def set_dh_checkboxes_state(self, states: Dict[str, bool]) -> None:
        col_names = ["alpha", "d", "theta", "r"]

        for row in range(self.table_dh_measured.rowCount()):
            joint_num = row + 1
            for col in range(self.table_dh_measured.columnCount()):
                param_name = f"{col_names[col]}{joint_num}"
                cell_widget = self.table_dh_measured.cellWidget(row, col)
                if isinstance(cell_widget, DHCellWidget) and param_name in states:
                    cell_widget.set_checked(states[param_name])

    def toggle_dh_checkboxes(self) -> None:
        self._set_all_dh_checkboxes(not self._are_all_dh_checkboxes_checked())

    def _are_all_dh_checkboxes_checked(self) -> bool:
        for row in range(self.table_dh_measured.rowCount()):
            for col in range(self.table_dh_measured.columnCount()):
                cell_widget = self.table_dh_measured.cellWidget(row, col)
                if isinstance(cell_widget, DHCellWidget) and cell_widget.isEnabled() and not cell_widget.is_checked():
                    return False
        return True

    def _are_any_dh_checkboxes_checked(self) -> bool:
        for row in range(self.table_dh_measured.rowCount()):
            for col in range(self.table_dh_measured.columnCount()):
                cell_widget = self.table_dh_measured.cellWidget(row, col)
                if isinstance(cell_widget, DHCellWidget) and cell_widget.isEnabled() and cell_widget.is_checked():
                    return True
        return False

    def _update_toggle_check_button_text(self) -> None:
        if self._are_all_dh_checkboxes_checked():
            self.btn_toggle_check.setText("Tout désélectionner")
        else:
            self.btn_toggle_check.setText("Tout sélectionner")

    def _update_apply_button_state(self) -> None:
        self.btn_apply_measured.setEnabled(self._are_any_dh_checkboxes_checked())

    def _set_all_dh_checkboxes(self, checked: bool) -> None:
        for row in range(self.table_dh_measured.rowCount()):
            for col in range(self.table_dh_measured.columnCount()):
                cell_widget = self.table_dh_measured.cellWidget(row, col)
                if isinstance(cell_widget, DHCellWidget) and cell_widget.isEnabled():
                    cell_widget.set_checked(checked)
        self._update_toggle_check_button_text()
        self._update_apply_button_state()
        self.dh_checkboxes_changed.emit()

    def _emit_dh_checkboxes_changed(self, *_args) -> None:
        self._sync_group_checkboxes()
        self._update_toggle_check_button_text()
        self._update_apply_button_state()
        self.dh_checkboxes_changed.emit()

    def get_measured_dh_params(self) -> List[List[float]]:
        measured: List[List[float]] = []
        for row in range(self.table_dh_measured.rowCount()):
            row_values: List[float] = []
            for col in range(self.table_dh_measured.columnCount()):
                value = 0.0
                cell_widget = self.table_dh_measured.cellWidget(row, col)
                if isinstance(cell_widget, DHCellWidget):
                    text = cell_widget.get_value().strip().replace(",", ".")
                    if text:
                        try:
                            value = float(text)
                        except ValueError:
                            value = 0.0
                row_values.append(value)
            measured.append(row_values)
        return measured

    def set_corrections(self, corrections: List[List[float]]) -> None:
        self._updating_corrections_from_model = True
        try:
            for row in range(6):
                correction_row = corrections[row] if row < len(corrections) else []
                for col in range(6):
                    value = str(correction_row[col]) if col < len(correction_row) else "0"
                    item = self.table_corr.item(row, col)
                    if item is None:
                        self.table_corr.setItem(row, col, QTableWidgetItem(value))
                    else:
                        item.setText(value)
        finally:
            self._updating_corrections_from_model = False

    def get_corrections(self) -> List[List[float]]:
        corrections: List[List[float]] = []
        for row in range(6):
            correction_row: List[float] = []
            for col in range(6):
                item = self.table_corr.item(row, col)
                try:
                    correction_value = float(item.text()) if item and item.text().strip() != "" else 0.0
                except ValueError:
                    correction_value = 0.0
                correction_row.append(correction_value)
            corrections.append(correction_row)
        return corrections

    def _on_correction_item_changed(self, *_args) -> None:
        if self._updating_corrections_from_model:
            return
        self.corrections_changed.emit(self.get_corrections())

    def set_measured_controls_enabled(self, enabled: bool) -> None:
        """Active ou désactive les contrôles des valeurs mesurées"""
        self.table_dh_measured.setEnabled(enabled)
        self.btn_toggle_check.setEnabled(enabled)
        self.btn_set_as_ref.setEnabled(enabled)
        self.btn_clear.setEnabled(enabled)
        self.checkbox_segment_lengths.setEnabled(enabled)
        self.checkbox_axis_origins.setEnabled(enabled)
        self.checkbox_parallelism.setEnabled(enabled)
        # Désactiver le bouton appliquer lors du clear
        if not enabled:
            self.checkbox_segment_lengths.setChecked(False)
            self.checkbox_axis_origins.setChecked(False)
            self.checkbox_parallelism.setChecked(False)
            self.btn_apply_measured.setEnabled(False)
        # Désactiver le bouton importer si des mesures sont présentes
        self.btn_import_me.setEnabled(not enabled)
