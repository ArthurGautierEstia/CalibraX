from typing import List, Optional

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPainter, QPixmap
from PyQt6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)


class ExternalAxisWidget(QWidget):
    """Widget pour l'axe externe : import de mesures et visualisation des ecarts."""

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.measurements: Optional[np.ndarray] = None
        self.theoretical_forward: Optional[np.ndarray] = None
        self.theoretical_return: Optional[np.ndarray] = None
        self.current_slope: Optional[float] = None
        self.current_backlash_delta: Optional[float] = None
        self.current_precision_peak: Optional[float] = None
        self.scale_limit_overridden = False
        self.tolerance_min_line: Optional[pg.InfiniteLine] = None
        self.tolerance_max_line: Optional[pg.InfiniteLine] = None
        self.point_names: List[str] = []
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(5)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)

        content_widget = QWidget()
        content_layout = QVBoxLayout(content_widget)
        content_layout.setSpacing(10)
        content_widget.setMinimumWidth(620)

        import_group = QGroupBox("Import des mesures")
        import_group_layout = QVBoxLayout(import_group)
        import_group_layout.setSpacing(8)

        axis_type_layout = QHBoxLayout()
        axis_type_layout.addWidget(QLabel("Type d'axe externe :"))
        self.linear_radio = QRadioButton("Lineaire")
        self.rotary_radio = QRadioButton("Rotatif")
        self.linear_radio.setChecked(True)
        self.axis_group = QButtonGroup()
        self.axis_group.addButton(self.linear_radio)
        self.axis_group.addButton(self.rotary_radio)
        self.linear_radio.toggled.connect(self._update_unit_context)
        self.rotary_radio.toggled.connect(self._update_unit_context)
        axis_type_layout.addWidget(self.linear_radio)
        axis_type_layout.addWidget(self.rotary_radio)
        axis_type_layout.addStretch()
        self.lever_arm_container = QWidget()
        lever_arm_layout = QHBoxLayout(self.lever_arm_container)
        lever_arm_layout.setContentsMargins(0, 0, 0, 0)
        lever_arm_layout.setSpacing(6)
        self.lever_arm_label = QLabel("Bras de levier :")
        self.lever_arm_input = self._make_coordinate_spinbox(1000.0, minimum=0.0)
        self.lever_arm_input.setSuffix(" mm")
        self.lever_arm_input.valueChanged.connect(self._update_lever_arm_error_display)
        lever_arm_layout.addWidget(self.lever_arm_label)
        lever_arm_layout.addWidget(self.lever_arm_input)
        axis_type_layout.addWidget(self.lever_arm_container)
        import_layout = QHBoxLayout()
        self.file_label = QLabel("Aucun fichier chargé")
        self.file_label.setStyleSheet("border: 1px solid #555; padding: 2px; background-color: #2a2a2a; color: #d8d8d8;")
        self.file_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        import_layout.addWidget(self.file_label, 1)
        self.btn_import = QPushButton("Importer (.txt)")
        self.btn_import.clicked.connect(self._import_measurements)
        import_layout.addWidget(self.btn_import, 1)
        self.btn_clear = QPushButton("Vider")
        self.btn_clear.clicked.connect(self._clear_measurements)
        import_layout.addWidget(self.btn_clear, 1)
        import_group_layout.addLayout(import_layout)

        import_group_layout.addLayout(axis_type_layout)

        theoretical_layout = QHBoxLayout()
        theoretical_layout.addWidget(QLabel("Plage de mesures :"))
        self.min_input = self._make_coordinate_spinbox(0.0)
        self.max_input = self._make_coordinate_spinbox(9750.0)
        self.step_input = self._make_coordinate_spinbox(250.0, minimum=1.0)
        self.min_input.valueChanged.connect(self._update_plot)
        self.max_input.valueChanged.connect(self._update_plot)
        self.step_input.valueChanged.connect(self._update_plot)
        theoretical_layout.addWidget(QLabel("Min :"))
        theoretical_layout.addWidget(self.min_input)
        theoretical_layout.addWidget(QLabel("Max :"))
        theoretical_layout.addWidget(self.max_input)
        theoretical_layout.addWidget(QLabel("Pas :"))
        theoretical_layout.addWidget(self.step_input)
        theoretical_layout.addStretch()
        import_group_layout.addLayout(theoretical_layout)
        import_group.setMinimumHeight(150)
        import_group.setMinimumWidth(590)

        content_layout.addWidget(import_group)

        tolerances_section_layout = QHBoxLayout()
        tolerances_section_layout.setSpacing(10)

        tolerances_group = QGroupBox("Tolérances")
        tolerances_group_layout = QVBoxLayout(tolerances_group)
        tolerances_group_layout.setSpacing(8)

        reduction_tolerance_layout = QHBoxLayout()
        reduction_tolerance_label = QLabel("Tolérance de réduction :")
        reduction_tolerance_layout.addWidget(reduction_tolerance_label, 1)
        self.reduction_tolerance_input = self._make_tolerance_spinbox(0.001)
        self.reduction_tolerance_input.valueChanged.connect(self._update_conformity_status)
        reduction_tolerance_layout.addWidget(self.reduction_tolerance_input, 1)
        tolerances_group_layout.addLayout(reduction_tolerance_layout)

        backlash_tolerance_layout = QHBoxLayout()
        backlash_tolerance_label = QLabel("Tolérance de jeu :")
        backlash_tolerance_layout.addWidget(backlash_tolerance_label, 1)
        self.backlash_tolerance_input = self._make_tolerance_spinbox(0.005)
        self.backlash_tolerance_input.valueChanged.connect(self._update_conformity_status)
        backlash_tolerance_layout.addWidget(self.backlash_tolerance_input, 1)
        tolerances_group_layout.addLayout(backlash_tolerance_layout)

        precision_tolerance_layout = QHBoxLayout()
        precision_tolerance_label = QLabel("Tolérance de précision :")
        precision_tolerance_layout.addWidget(precision_tolerance_label, 1)
        self.precision_tolerance_input = self._make_tolerance_spinbox(0.001)
        self.precision_tolerance_input.valueChanged.connect(self._update_conformity_status)
        precision_tolerance_layout.addWidget(self.precision_tolerance_input, 1)
        tolerances_group_layout.addLayout(precision_tolerance_layout)
        tolerances_group.setMinimumWidth(380)

        conformity_group = QGroupBox("Conformité")
        conformity_group_layout = QVBoxLayout(conformity_group)
        conformity_group_layout.setSpacing(8)
        conformity_group.setMinimumWidth(190)

        reduction_status_layout = QHBoxLayout()
        reduction_status_layout.addWidget(QLabel("Réduction :"), 1)
        self.reduction_status_label = QLabel("-")
        reduction_status_layout.addWidget(self.reduction_status_label, 1)
        conformity_group_layout.addLayout(reduction_status_layout)

        backlash_status_layout = QHBoxLayout()
        backlash_status_layout.addWidget(QLabel("Jeu :"), 1)
        self.backlash_status_label = QLabel("-")
        backlash_status_layout.addWidget(self.backlash_status_label, 1)
        conformity_group_layout.addLayout(backlash_status_layout)

        precision_status_layout = QHBoxLayout()
        precision_status_layout.addWidget(QLabel("Précision :"), 1)
        self.precision_status_label = QLabel("-")
        precision_status_layout.addWidget(self.precision_status_label, 1)
        conformity_group_layout.addLayout(precision_status_layout)

        tolerances_section_layout.addWidget(tolerances_group, 2)
        tolerances_section_layout.addWidget(conformity_group, 1)
        content_layout.addLayout(tolerances_section_layout)

        deviation_group = QGroupBox("Visualisation des écarts")
        deviation_group_layout = QVBoxLayout(deviation_group)
        deviation_group_layout.setSpacing(8)

        tolerance_layout = QHBoxLayout()
        tolerance_layout.addWidget(QLabel("Echelle des écarts :"))
        self.scale_limit_input = self._make_tolerance_spinbox(0.05)
        self.scale_limit_input.valueChanged.connect(self._on_scale_limit_changed)
        tolerance_layout.addWidget(self.scale_limit_input)
        self.lever_arm_error_label = QLabel("-")
        self.lever_arm_error_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        tolerance_layout.addWidget(self.lever_arm_error_label, 1)
        tolerance_layout.addStretch()
        deviation_group_layout.addLayout(tolerance_layout)

        coord_layout = QHBoxLayout()
        coord_layout.addWidget(QLabel("Composante à afficher :"))
        self.x_checkbox = QCheckBox("X")
        self.y_checkbox = QCheckBox("Y")
        self.z_checkbox = QCheckBox("Z")
        self.x_checkbox.setChecked(True)
        self.x_checkbox.stateChanged.connect(self._update_plot)
        self.y_checkbox.stateChanged.connect(self._update_plot)
        self.z_checkbox.stateChanged.connect(self._update_plot)
        coord_layout.addWidget(self.x_checkbox)
        coord_layout.addWidget(self.y_checkbox)
        coord_layout.addWidget(self.z_checkbox)
        coord_layout.addStretch()
        deviation_group_layout.addLayout(coord_layout)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setMinimumHeight(260)
        self.plot_widget.setMaximumHeight(360)
        self.plot_widget.setBackground("#151515")
        self.plot_widget.setLabel("left", "Ecart (mm)", color="#d8d8d8")
        self.plot_widget.setLabel("bottom", "Valeurs theoriques (mm)", color="#d8d8d8")
        self.plot_widget.getAxis("left").enableAutoSIPrefix(False)
        self.plot_widget.getAxis("bottom").enableAutoSIPrefix(False)
        self.plot_widget.getAxis("left").setPen(pg.mkPen("#a8a8a8"))
        self.plot_widget.getAxis("bottom").setPen(pg.mkPen("#a8a8a8"))
        self.plot_widget.getAxis("left").setTextPen(pg.mkPen("#d8d8d8"))
        self.plot_widget.getAxis("bottom").setTextPen(pg.mkPen("#d8d8d8"))
        self.plot_widget.showGrid(x=True, y=True, alpha=0.22)
        legend = self.plot_widget.addLegend(offset=(-85, 10))
        legend.anchor((1, 0), (1, 0))
        deviation_group_layout.addWidget(self.plot_widget, 0)
        deviation_group_layout.addLayout(self._build_statistics_layout())
        deviation_group.setMinimumWidth(590)
        content_layout.addWidget(deviation_group)

        calibration_group = QGroupBox("Calibration")
        calibration_group_layout = QVBoxLayout(calibration_group)
        calibration_group_layout.setSpacing(8)

        ratio_group = QGroupBox("Rapport de réduction")
        ratio_group_layout = QVBoxLayout(ratio_group)
        ratio_group_layout.setSpacing(5)

        ratio_stats_layout = QHBoxLayout()
        ratio_stats_layout.setSpacing(20)
        self.slope_label = QLabel("Pente : -")
        self.intercept_label = QLabel("Ordonnée : -")
        self.slope_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.intercept_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ratio_stats_layout.addWidget(self.slope_label, 1)
        ratio_stats_layout.addWidget(self.intercept_label, 1)
        ratio_group_layout.addLayout(ratio_stats_layout)

        ratio_fraction_layout = QHBoxLayout()
        ratio_fraction_layout.addWidget(QLabel("Numérateur :"))
        self.ratio_numerator_input = self._make_ratio_spinbox()
        self.ratio_numerator_input.setDecimals(0)
        self.ratio_numerator_input.setSingleStep(1.0)
        self.ratio_numerator_input.setValue(1.0)
        self.ratio_numerator_input.valueChanged.connect(self._update_current_ratio_from_fraction)
        ratio_fraction_layout.addWidget(self.ratio_numerator_input)
        ratio_fraction_layout.addWidget(QLabel("Dénominateur :"))
        self.ratio_denominator_input = self._make_ratio_spinbox()
        self.ratio_denominator_input.setDecimals(0)
        self.ratio_denominator_input.setSingleStep(1.0)
        self.ratio_denominator_input.setValue(1.0)
        self.ratio_denominator_input.valueChanged.connect(self._update_current_ratio_from_fraction)
        ratio_fraction_layout.addWidget(self.ratio_denominator_input)
        ratio_fraction_layout.addStretch()
        ratio_group_layout.addLayout(ratio_fraction_layout)

        ratio_layout = QHBoxLayout()
        ratio_layout.addWidget(QLabel("Coeff. actuel :"))
        self.current_ratio_input = self._make_ratio_spinbox()
        self.current_ratio_input.setReadOnly(True)
        self.current_ratio_input.setEnabled(False)
        self.current_ratio_input.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
        ratio_layout.addWidget(self.current_ratio_input)
        ratio_layout.addWidget(QLabel("Coeff. corrigé :"))
        self.corrected_ratio_input = self._make_ratio_spinbox()
        self.corrected_ratio_input.setReadOnly(True)
        self.corrected_ratio_input.setEnabled(False)
        self.corrected_ratio_input.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
        ratio_layout.addWidget(self.corrected_ratio_input)
        ratio_layout.addStretch()
        ratio_group_layout.addLayout(ratio_layout)

        new_numerator_layout = QHBoxLayout()
        new_numerator_layout.addWidget(QLabel("Nouveau numérateur :"))
        self.new_ratio_numerator_input = self._make_ratio_spinbox()
        self.new_ratio_numerator_input.setDecimals(0)
        self.new_ratio_numerator_input.setSingleStep(1.0)
        self.new_ratio_numerator_input.setReadOnly(True)
        self.new_ratio_numerator_input.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
        new_numerator_layout.addWidget(self.new_ratio_numerator_input)
        new_numerator_layout.addStretch()
        ratio_group_layout.addLayout(new_numerator_layout)
        calibration_group_layout.addWidget(ratio_group)

        backlash_group = QGroupBox("Jeu à l'inversion")
        backlash_group_layout = QVBoxLayout(backlash_group)
        backlash_group_layout.setSpacing(5)

        backlash_layout = QHBoxLayout()
        backlash_layout.addWidget(QLabel("Jeu actuel :"))
        self.current_backlash_input = self._make_ratio_spinbox()
        self.current_backlash_input.valueChanged.connect(self._update_corrected_backlash)
        backlash_layout.addWidget(self.current_backlash_input)
        backlash_layout.addWidget(QLabel("Jeu corrigé :"))
        self.corrected_backlash_input = self._make_ratio_spinbox()
        self.corrected_backlash_input.setReadOnly(True)
        self.corrected_backlash_input.setEnabled(False)
        self.corrected_backlash_input.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
        backlash_layout.addWidget(self.corrected_backlash_input)
        backlash_layout.addStretch()
        backlash_group_layout.addLayout(backlash_layout)
        calibration_group_layout.addWidget(backlash_group)

        self._set_statistics_values()
        self._update_corrected_backlash()
        self._update_current_ratio_from_fraction()
        self._update_conformity_status()

        compensation_group = QGroupBox("Table de compensation bi-directionnelle (Siemens CEC)")
        compensation_group_layout = QVBoxLayout(compensation_group)
        compensation_group_layout.setSpacing(6)

        compensation_name_layout = QHBoxLayout()
        compensation_name_layout.addWidget(QLabel("Nom de l'axe :"))
        self.axis_name_input = QLineEdit("A1")
        self.axis_name_input.textChanged.connect(self._update_compensation_filename_from_axis_name)
        compensation_name_layout.addWidget(self.axis_name_input)
        compensation_name_layout.addStretch()
        compensation_group_layout.addLayout(compensation_name_layout)

        compensation_index_layout = QHBoxLayout()
        compensation_index_layout.addWidget(QLabel("Index table sens positif :"))
        self.forward_table_index_input = self._make_table_index_spinbox(3)
        compensation_index_layout.addWidget(self.forward_table_index_input)
        self.forward_modulo_checkbox = QCheckBox("Modulo")
        compensation_index_layout.addWidget(self.forward_modulo_checkbox)
        compensation_index_layout.addStretch()
        compensation_group_layout.addLayout(compensation_index_layout)

        compensation_negative_index_layout = QHBoxLayout()
        compensation_negative_index_layout.addWidget(QLabel("Index table sens negatif :"))
        self.return_table_index_input = self._make_table_index_spinbox(4)
        compensation_negative_index_layout.addWidget(self.return_table_index_input)
        self.return_modulo_checkbox = QCheckBox("Modulo")
        compensation_negative_index_layout.addWidget(self.return_modulo_checkbox)
        compensation_negative_index_layout.addStretch()
        compensation_group_layout.addLayout(compensation_negative_index_layout)

        compensation_reference_layout = QHBoxLayout()
        compensation_reference_layout.addWidget(QLabel("Position zéro :"))
        self.calibration_position_input = self._make_coordinate_spinbox(0.0)
        compensation_reference_layout.addWidget(self.calibration_position_input)
        compensation_reference_layout.addWidget(QLabel("Direction :"))
        self.calibration_reference_combo = QComboBox()
        self.calibration_reference_combo.addItems(["Sens positif", "Sens négatif"])
        compensation_reference_layout.addWidget(self.calibration_reference_combo)
        compensation_reference_layout.addStretch()
        compensation_group_layout.addLayout(compensation_reference_layout)

        compensation_filename_layout = QHBoxLayout()
        compensation_filename_field_layout = QHBoxLayout()
        compensation_filename_field_layout.addWidget(QLabel("Nom du fichier :"))
        self.compensation_filename_input = QLineEdit("COMP_A1-A1_CEC")
        compensation_filename_field_layout.addWidget(self.compensation_filename_input)
        compensation_filename_layout.addLayout(compensation_filename_field_layout, 1)
        self.btn_generate_compensation_table = QPushButton("Générer la table .SPF")
        self.btn_generate_compensation_table.clicked.connect(self._generate_compensation_table)
        compensation_filename_layout.addWidget(self.btn_generate_compensation_table, 1)
        compensation_group_layout.addLayout(compensation_filename_layout)
        self._update_compensation_filename_from_axis_name(self.axis_name_input.text())
        compensation_group.setMinimumWidth(590)
        calibration_group_layout.addWidget(compensation_group)
        calibration_group.setMinimumWidth(590)

        content_layout.addWidget(calibration_group)
        content_layout.addStretch()

        self._update_unit_context()
        scroll_area.setWidget(content_widget)
        layout.addWidget(scroll_area)

    @staticmethod
    def _make_coordinate_spinbox(
        value: float,
        minimum: float = -1000000.0,
        maximum: float = 1000000.0,
    ) -> QDoubleSpinBox:
        spinbox = QDoubleSpinBox()
        spinbox.setRange(minimum, maximum)
        spinbox.setDecimals(0)
        spinbox.setSingleStep(1.0)
        spinbox.setValue(value)
        return spinbox

    @staticmethod
    def _make_ratio_spinbox() -> QDoubleSpinBox:
        spinbox = QDoubleSpinBox()
        spinbox.setRange(-1000000000.0, 1000000000.0)
        spinbox.setDecimals(6)
        spinbox.setSingleStep(0.000001)
        spinbox.setValue(0.0)
        return spinbox

    @staticmethod
    def _make_tolerance_spinbox(value: float) -> QDoubleSpinBox:
        spinbox = QDoubleSpinBox()
        spinbox.setRange(-1000000.0, 1000000.0)
        spinbox.setDecimals(3)
        spinbox.setSingleStep(0.01)
        spinbox.setValue(value)
        return spinbox

    def _current_axis_unit(self) -> str:
        return "°" if self.rotary_radio.isChecked() else "mm"

    def _update_unit_context(self) -> None:
        axis_unit = self._current_axis_unit()
        suffix = f" {axis_unit}"
        backlash_decimals = 6 if self.rotary_radio.isChecked() else 3
        is_rotary_mode = self.rotary_radio.isChecked()

        axis_spinboxes = [
            self.min_input,
            self.max_input,
            self.step_input,
            self.reduction_tolerance_input,
            self.backlash_tolerance_input,
            self.precision_tolerance_input,
            self.scale_limit_input,
            self.calibration_position_input,
            self.current_backlash_input,
            self.corrected_backlash_input,
        ]
        for spinbox in axis_spinboxes:
            spinbox.setSuffix(suffix)
        self.reduction_tolerance_input.setSuffix("")
        self.current_backlash_input.setDecimals(backlash_decimals)
        self.corrected_backlash_input.setDecimals(backlash_decimals)
        self.lever_arm_label.setHidden(not is_rotary_mode)
        self.lever_arm_input.setHidden(not is_rotary_mode)
        self.lever_arm_input.setSuffix(" mm")
        self._apply_default_tolerance_values()
        self._update_lever_arm_error_display()

        self.plot_widget.setLabel("left", f"Ecart ({axis_unit})", color="#d8d8d8")
        self.plot_widget.setLabel("bottom", f"Valeurs theoriques ({axis_unit})", color="#d8d8d8")

        if self.measurements is None:
            self._set_statistics_values()
        else:
            self._update_plot()

    def _apply_default_tolerance_values(self) -> None:
        if self.rotary_radio.isChecked():
            reduction_value = 0.001
            backlash_value = 0.001
            precision_value = 0.003
        else:
            reduction_value = 0.01
            backlash_value = 0.05
            precision_value = 0.1

        self.reduction_tolerance_input.blockSignals(True)
        self.backlash_tolerance_input.blockSignals(True)
        self.precision_tolerance_input.blockSignals(True)
        self.reduction_tolerance_input.setValue(reduction_value)
        self.backlash_tolerance_input.setValue(backlash_value)
        self.precision_tolerance_input.setValue(precision_value)
        self.reduction_tolerance_input.blockSignals(False)
        self.backlash_tolerance_input.blockSignals(False)
        self.precision_tolerance_input.blockSignals(False)

    @staticmethod
    def _make_table_index_spinbox(value: int) -> QSpinBox:
        spinbox = QSpinBox()
        spinbox.setRange(0, 999)
        spinbox.setValue(value)
        return spinbox

    def _build_statistics_layout(self) -> QVBoxLayout:
        stats_layout = QVBoxLayout()
        stats_layout.setSpacing(6)

        primary_stats_layout = QHBoxLayout()
        primary_stats_layout.setSpacing(20)

        self.min_label = QLabel("Min : -")
        self.max_label = QLabel("Max : -")
        self.mean_label = QLabel("Moy : -")
        self.amplitude_label = QLabel("Ampl : -")

        self.min_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.max_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.mean_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.amplitude_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        primary_stats_layout.addWidget(self.min_label, 1)
        primary_stats_layout.addWidget(self.max_label, 1)
        primary_stats_layout.addWidget(self.mean_label, 1)
        primary_stats_layout.addWidget(self.amplitude_label, 1)

        stats_layout.addLayout(primary_stats_layout)
        return stats_layout

    def _build_plot_legend_layout(self) -> QHBoxLayout:
        legend_layout = QHBoxLayout()
        legend_layout.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        legend_layout.setSpacing(18)
        legend_layout.addWidget(self._make_legend_sample("Aller", "#27d6a1", Qt.PenStyle.SolidLine))
        legend_layout.addWidget(self._make_legend_sample("Retour", "#196079", Qt.PenStyle.DashLine))
        return legend_layout

    @staticmethod
    def _make_legend_sample(label: str, color: str, style: Qt.PenStyle) -> QWidget:
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)

        sample = QLabel()
        pixmap = QPixmap(34, 10)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(pg.mkPen(color=color, width=2, style=style))
        painter.drawLine(2, 5, 32, 5)
        painter.end()
        sample.setPixmap(pixmap)
        sample.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

        layout.addWidget(sample)
        layout.addWidget(QLabel(label))
        return widget

    def _import_measurements(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Selectionner un fichier de mesures",
            "",
            "Fichiers texte (*.txt);;Tous les fichiers (*.*)",
        )

        if not file_path:
            return

        try:
            import os

            self.file_label.setText(os.path.basename(file_path))
            self.scale_limit_overridden = False

            data = self._read_measurements_file(file_path)
            if not data:
                QMessageBox.warning(self, "Erreur", "Aucun point valide trouve dans le fichier.")
                return
            if len(data) % 2 != 0:
                QMessageBox.warning(
                    self,
                    "Erreur",
                    "Le fichier doit contenir un nombre pair de lignes : premiere moitie Aller, deuxieme moitie Retour.",
                )
                return

            self.measurements = np.array(data, dtype=float)
            self._apply_detected_measurement_context()
            self._update_theoretical_coordinates()
            expected_count = len(self.theoretical_forward) if self.theoretical_forward is not None else 0
            if expected_count != len(data) // 2:
                QMessageBox.warning(
                    self,
                    "Coordonnees theoriques",
                    "La plage Min/Max/Step ne correspond pas au nombre de points Aller du fichier.",
                )
            self._update_plot()
            QMessageBox.information(self, "Succes", f"{len(data)} points importes avec succes.")
        except Exception as error:
            QMessageBox.critical(self, "Erreur", f"Erreur lors de l'import : {error}")

    def _clear_measurements(self) -> None:
        self.measurements = None
        self.theoretical_forward = None
        self.theoretical_return = None
        self.current_slope = None
        self.current_backlash_delta = None
        self.current_precision_peak = None
        self.point_names = []

        self.file_label.setText("Aucun fichier chargé")
        self.plot_widget.clear()
        self.tolerance_min_line = None
        self.tolerance_max_line = None

        self.linear_radio.setChecked(True)
        self.lever_arm_input.setValue(1000.0)
        self.min_input.setValue(0.0)
        self.max_input.setValue(9750.0)
        self.step_input.setValue(250.0)
        self.scale_limit_overridden = False
        self.scale_limit_input.setValue(0.05)
        self.x_checkbox.setChecked(True)
        self.y_checkbox.setChecked(False)
        self.z_checkbox.setChecked(False)

        self.current_ratio_input.setValue(0.0)
        self.corrected_ratio_input.setValue(0.0)
        self.new_ratio_numerator_input.setValue(0.0)
        self.ratio_numerator_input.setValue(1.0)
        self.ratio_denominator_input.setValue(1.0)
        self.current_backlash_input.setValue(0.0)
        self.corrected_backlash_input.setValue(0.0)

        self.axis_name_input.setText("A_1")
        self.forward_table_index_input.setValue(3)
        self.return_table_index_input.setValue(4)
        self.forward_modulo_checkbox.setChecked(False)
        self.return_modulo_checkbox.setChecked(False)
        self.calibration_position_input.setValue(0.0)
        self.calibration_reference_combo.setCurrentText("Aller")
        self.compensation_filename_input.setText("COMP_A_1-A_1_CEC")

        self._apply_default_tolerance_values()
        self._set_stats_empty()
        self._update_current_ratio_from_fraction()
        self._update_corrected_backlash()
        self._update_conformity_status()
        self._update_lever_arm_error_display()
        self._update_unit_context()

    def _read_measurements_file(self, file_path: str) -> list[list[float]]:
        data: list[list[float]] = []
        self.point_names = []
        with open(file_path, "r", encoding="utf-8-sig") as file:
            for line_index, line in enumerate(file):
                line = line.strip()
                if not line:
                    continue

                if "\t" in line:
                    parts = [part.strip() for part in line.split("\t")]
                else:
                    parts = [part.strip() for part in line.split(",")]

                if len(parts) < 3:
                    continue
                try:
                    values = [float(part) for part in parts[:3]]
                except ValueError:
                    continue
                data.append(values)
                self.point_names.append(f"Point {line_index + 1}")
        return data

    def _apply_detected_measurement_context(self) -> None:
        if self.measurements is None or len(self.measurements) == 0:
            return

        varying_axis_index = self._detect_varying_axis_index()
        self._apply_detected_axis_type(varying_axis_index)
        self._set_displayed_component(varying_axis_index)
        self._apply_inferred_measurement_range(varying_axis_index)

    def _detect_varying_axis_index(self) -> int:
        if self.measurements is None or len(self.measurements) == 0:
            return 0

        axis_ranges = np.ptp(self.measurements, axis=0)
        return int(np.argmax(axis_ranges))

    def _set_displayed_component(self, axis_index: int) -> None:
        self.x_checkbox.blockSignals(True)
        self.y_checkbox.blockSignals(True)
        self.z_checkbox.blockSignals(True)

        self.x_checkbox.setChecked(axis_index == 0)
        self.y_checkbox.setChecked(axis_index == 1)
        self.z_checkbox.setChecked(axis_index == 2)

        self.x_checkbox.blockSignals(False)
        self.y_checkbox.blockSignals(False)
        self.z_checkbox.blockSignals(False)

    def _apply_detected_axis_type(self, axis_index: int) -> None:
        if self.measurements is None or len(self.measurements) == 0:
            return

        axis_values = self.measurements[:, axis_index]
        axis_span = float(np.max(axis_values) - np.min(axis_values))
        inferred_step = self._infer_measurement_step(self.measurements[: len(self.measurements) // 2, axis_index])

        is_rotary = (
            axis_span <= 360.0 + 1e-6
            and inferred_step > 0.0
            and inferred_step <= 10.0
            and abs(round(inferred_step) - inferred_step) <= 0.05
        )

        if is_rotary:
            self.rotary_radio.setChecked(True)
        else:
            self.linear_radio.setChecked(True)

    def _apply_inferred_measurement_range(self, axis_index: int) -> None:
        if self.measurements is None or len(self.measurements) < 2:
            return

        forward_count = len(self.measurements) // 2
        if forward_count <= 1:
            return

        forward_values = self.measurements[:forward_count, axis_index]
        inferred_step = self._infer_measurement_step(forward_values)
        if inferred_step <= 0.0:
            return

        inferred_min = round(float(np.min(forward_values)) / inferred_step) * inferred_step
        inferred_max = round(float(np.max(forward_values)) / inferred_step) * inferred_step

        self.min_input.blockSignals(True)
        self.max_input.blockSignals(True)
        self.step_input.blockSignals(True)
        self.min_input.setValue(inferred_min)
        self.max_input.setValue(inferred_max)
        self.step_input.setValue(inferred_step)
        self.min_input.blockSignals(False)
        self.max_input.blockSignals(False)
        self.step_input.blockSignals(False)

    @staticmethod
    def _infer_measurement_step(values: np.ndarray) -> float:
        if len(values) < 2:
            return 0.0

        deltas = np.abs(np.diff(values))
        significant_deltas = deltas[deltas > 1e-6]
        if len(significant_deltas) == 0:
            return 0.0

        raw_step = float(np.median(significant_deltas))
        rounded_candidates = [
            round(raw_step, 0),
            round(raw_step, 1),
            round(raw_step, 2),
            round(raw_step, 3),
        ]

        best_step = raw_step
        best_error = float("inf")
        for candidate in rounded_candidates:
            if candidate <= 0.0:
                continue
            error = abs(candidate - raw_step)
            if error < best_error:
                best_error = error
                best_step = float(candidate)

        if best_step >= 1.0 and abs(best_step - raw_step) <= 0.05:
            return float(round(best_step))
        return best_step

    def _update_theoretical_coordinates(self) -> None:
        if self.measurements is None:
            self.theoretical_forward = None
            self.theoretical_return = None
            return

        forward_count = len(self.measurements) // 2
        values = self._build_theoretical_values(forward_count)
        self.theoretical_forward = values
        self.theoretical_return = values[::-1]

    def _build_theoretical_values(self, count: int) -> np.ndarray:
        minimum = self.min_input.value()
        maximum = self.max_input.value()
        step = self.step_input.value()
        if count <= 0 or step <= 0:
            return np.array([])

        direction = 1.0 if maximum >= minimum else -1.0
        span = abs(maximum - minimum)
        expected_count = int(round(span / step)) + 1
        if np.isclose((expected_count - 1) * step, span, atol=1e-6):
            values = minimum + direction * step * np.arange(expected_count, dtype=float)
        else:
            values = minimum + direction * step * np.arange(count, dtype=float)

        if len(values) > 0 and np.isclose(values[-1], maximum, atol=1e-6):
            values[-1] = maximum
        return values

    def _update_plot(self) -> None:
        if self.measurements is None:
            return

        self._update_theoretical_coordinates()
        self.plot_widget.clear()

        forward_count = len(self.measurements) // 2
        return_count = len(self.measurements) - forward_count
        if forward_count == 0 or return_count == 0:
            return

        if len(self.theoretical_forward) != forward_count or len(self.theoretical_return) != return_count:
            self._set_stats_empty()
            return

        forward_x = self.theoretical_forward
        return_x = self.theoretical_return
        forward_data = self.measurements[:forward_count]
        return_data = self.measurements[forward_count:forward_count + return_count]

        forward_colors = ["#27d6a1", "#2ca02c", "#1f77b4"]
        return_colors = ["#196079", "#9467bd", "#17becf"]
        labels = ["X", "Y", "Z"]
        checkboxes = [self.x_checkbox, self.y_checkbox, self.z_checkbox]
        all_values = []
        regression_theoretical = []
        regression_measured = []

        first_forward = True
        first_return = True

        for index, (checkbox, label, forward_color, return_color) in enumerate(
            zip(checkboxes, labels, forward_colors, return_colors)
        ):
            if not checkbox.isChecked():
                continue

            forward_error = forward_data[:, index] - forward_x
            return_error = return_data[:, index] - return_x
            all_values.extend(forward_error)
            all_values.extend(return_error)
            regression_theoretical.extend(forward_x)
            regression_theoretical.extend(return_x)
            regression_measured.extend(forward_data[:, index])
            regression_measured.extend(return_data[:, index])

            self.plot_widget.plot(
                forward_x,
                forward_error,
                pen=pg.mkPen(color=forward_color, width=2),
                symbol="o",
                symbolBrush=forward_color,
                name="Sens positif" if first_forward else None,
            )
            first_forward = False

            self.plot_widget.plot(
                return_x,
                return_error,
                pen=pg.mkPen(color=return_color, width=2),
                symbol="t",
                symbolBrush=return_color,
                name="Sens négatif" if first_return else None,
            )
            first_return = False

            forward_regression = self._calculate_linear_regression(forward_x, forward_data[:, index])
            if forward_regression is not None:
                slope, intercept = forward_regression
                x_range = np.linspace(np.min(forward_x), np.max(forward_x), 100)
                y_regression = slope * x_range + intercept - x_range
                self.plot_widget.plot(
                    x_range,
                    y_regression,
                    pen=pg.mkPen(color=forward_color, width=1, style=Qt.PenStyle.DotLine),
                )

            return_regression = self._calculate_linear_regression(return_x, return_data[:, index])
            if return_regression is not None:
                slope, intercept = return_regression
                x_range = np.linspace(np.min(return_x), np.max(return_x), 100)
                y_regression = slope * x_range + intercept - x_range
                self.plot_widget.plot(
                    x_range,
                    y_regression,
                    pen=pg.mkPen(color=return_color, width=1, style=Qt.PenStyle.DotLine),
                )

        if all_values:
            values_array = np.array(all_values, dtype=float)
            min_val = np.min(values_array)
            max_val = np.max(values_array)
            mean_val = np.mean(values_array)
            amplitude = max_val - min_val

            regression = self._calculate_linear_regression(regression_theoretical, regression_measured)
            if regression is None:
                self.current_slope = None
                slope = None
                intercept = None
            else:
                slope, intercept = regression
                self.current_slope = slope

            coordinate_index = self._selected_coordinate_index()
            aligned_return_data = return_data[::-1, coordinate_index]
            forward_measurements = forward_data[:, coordinate_index]
            backlash_delta = float(np.mean(aligned_return_data - forward_measurements))
            self.current_backlash_delta = backlash_delta
            self.current_precision_peak = float(max(abs(min_val), abs(max_val)))
            self._apply_suggested_y_scale(min_val, max_val)
            self._set_statistics_values(min_val, max_val, mean_val, amplitude, slope, intercept)
            self._update_corrected_ratio()
            self._update_corrected_backlash()
            self._update_conformity_status()
            self._update_lever_arm_error_display()
        else:
            self._set_stats_empty()

    @staticmethod
    def _calculate_linear_regression(x_values: list[float], y_values: list[float]) -> Optional[tuple[float, float]]:
        if len(x_values) < 2 or len(y_values) < 2:
            return None

        x_data = np.array(x_values, dtype=float)
        y_data = np.array(y_values, dtype=float)
        x_mean = np.mean(x_data)
        denominator = np.sum((x_data - x_mean) ** 2)
        if np.isclose(denominator, 0.0):
            return None

        y_mean = np.mean(y_data)
        numerator = np.sum((x_data - x_mean) * (y_data - y_mean))
        slope = float(numerator / denominator)
        intercept = float(y_mean - slope * x_mean)
        return slope, intercept

    def _generate_compensation_table(self) -> None:
        if self.measurements is None:
            QMessageBox.warning(self, "Table de compensation", "Importer un fichier de mesures avant d'exporter la table.")
            return

        self._update_theoretical_coordinates()
        forward_count = len(self.measurements) // 2
        return_count = len(self.measurements) - forward_count
        if forward_count == 0 or return_count == 0:
            QMessageBox.warning(self, "Table de compensation", "Les mesures Aller/Retour sont incompletes.")
            return

        if len(self.theoretical_forward) != forward_count or len(self.theoretical_return) != return_count:
            QMessageBox.warning(
                self,
                "Table de compensation",
                "La plage Min/Max/Step ne correspond pas au nombre de points importes.",
            )
            return

        axis_name = self.axis_name_input.text().strip()
        if not axis_name:
            QMessageBox.warning(self, "Table de compensation", "Renseigner le nom de l'axe a compenser.")
            return

        coordinate_index = self._selected_coordinate_index()
        forward_data = self.measurements[:forward_count, coordinate_index]
        return_data = self.measurements[forward_count:forward_count + return_count, coordinate_index]

        forward_compensation = forward_data - self.theoretical_forward
        return_compensation = return_data - self.theoretical_return
        reference_compensation = self._compensation_at_calibration_position(
            self.theoretical_return if self.calibration_reference_combo.currentText() == "Retour" else self.theoretical_forward,
            return_compensation if self.calibration_reference_combo.currentText() == "Retour" else forward_compensation,
        )
        forward_compensation = forward_compensation - reference_compensation
        return_compensation = return_compensation - reference_compensation

        table = self._format_compensation_table(
            axis_name=axis_name,
            forward_index=self.forward_table_index_input.value(),
            return_index=self.return_table_index_input.value(),
            forward_compensation=forward_compensation,
            return_compensation=return_compensation[::-1],
            forward_modulo_enabled=self.forward_modulo_checkbox.isChecked(),
            return_modulo_enabled=self.return_modulo_checkbox.isChecked(),
        )
        file_name = self.compensation_filename_input.text().strip()
        if not file_name:
            file_name = f"COMP_{axis_name}-{axis_name}_CEC"

        default_name = file_name if file_name.lower().endswith(".spf") else f"{file_name}.SPF"
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Exporter la table de compensation",
            default_name,
            "Fichiers SPF (*.SPF);;Tous les fichiers (*.*)",
        )
        if not file_path:
            return
        if not file_path.lower().endswith(".spf"):
            file_path = f"{file_path}.SPF"

        try:
            with open(file_path, "w", encoding="utf-8", newline="\n") as file:
                file.write(table)
                file.write("\n")
        except Exception as error:
            QMessageBox.critical(self, "Table de compensation", f"Erreur lors de l'export : {error}")
            return

        QMessageBox.information(self, "Table de compensation", f"Fichier exporte : {file_path}")

    def _selected_coordinate_index(self) -> int:
        if self.x_checkbox.isChecked():
            return 0
        if self.y_checkbox.isChecked():
            return 1
        if self.z_checkbox.isChecked():
            return 2
        return 0

    def _compensation_at_calibration_position(
        self,
        theoretical_values: np.ndarray,
        compensation_values: np.ndarray,
    ) -> float:
        calibration_position = self.calibration_position_input.value()
        if len(theoretical_values) == 0 or len(compensation_values) == 0:
            return 0.0

        order = np.argsort(theoretical_values)
        sorted_theoretical = theoretical_values[order]
        sorted_compensation = compensation_values[order]
        return float(np.interp(
            calibration_position,
            sorted_theoretical,
            sorted_compensation,
        ))

    def _update_compensation_filename_from_axis_name(self, axis_name: str) -> None:
        normalized_axis_name = axis_name.strip()
        if not normalized_axis_name:
            return
        self.compensation_filename_input.setText(
            f"COMP_{normalized_axis_name}-{normalized_axis_name}_CEC"
        )

    def _format_compensation_table(
        self,
        axis_name: str,
        forward_index: int,
        return_index: int,
        forward_compensation: np.ndarray,
        return_compensation: np.ndarray,
        forward_modulo_enabled: bool,
        return_modulo_enabled: bool,
    ) -> str:
        blocks = [
            self._format_compensation_block(
                axis_name,
                forward_index,
                forward_compensation,
                direction=1,
                modulo_value=1 if forward_modulo_enabled else -1,
            ),
            self._format_compensation_block(
                axis_name,
                return_index,
                return_compensation,
                direction=-1,
                modulo_value=1 if return_modulo_enabled else -1,
            ),
            "M0",
            "M17",
        ]
        return "\n\n".join(blocks)

    def _format_compensation_block(
        self,
        axis_name: str,
        table_index: int,
        compensation_values: np.ndarray,
        direction: int,
        modulo_value: int,
    ) -> str:
        lines = [
            f"$AN_CEC_INPUT_AXIS[{table_index}]=({axis_name})",
            f"$AN_CEC_OUTPUT_AXIS[{table_index}]=({axis_name})",
            f"$AN_CEC_STEP[{table_index}]={self.step_input.value():.0f}",
            f"$AN_CEC_MIN[{table_index}]={self.min_input.value():.0f}",
            f"$AN_CEC_MAX[{table_index}]={self.max_input.value():.0f}",
            f"$AN_CEC_DIRECTION[{table_index}]={direction}",
            f"$AN_CEC_IS_MODULO[{table_index}]={modulo_value}",
            "",
        ]
        for value_index, value in enumerate(compensation_values):
            lines.append(f"$AN_CEC[{table_index},{value_index}]={value:.6f}")
        return "\n".join(lines)

    def _set_stats_empty(self) -> None:
        self.current_slope = None
        self.current_backlash_delta = None
        self.current_precision_peak = None
        self._set_statistics_values()
        self._update_corrected_ratio()
        self._update_corrected_backlash()
        self._update_conformity_status()
        self._update_lever_arm_error_display()

    def _set_statistics_values(
        self,
        min_val: Optional[float] = None,
        max_val: Optional[float] = None,
        mean_val: Optional[float] = None,
        amplitude: Optional[float] = None,
        slope: Optional[float] = None,
        intercept: Optional[float] = None,
    ) -> None:
        axis_unit = self._current_axis_unit()
        value_format = ".6f" if self.rotary_radio.isChecked() else ".3f"
        if min_val is None:
            self.min_label.setText("Min : -")
            self.max_label.setText("Max : -")
            self.mean_label.setText("Moy : -")
            self.amplitude_label.setText("Ampl : -")
            self.slope_label.setText("Pente : -")
            self.intercept_label.setText("Ordonnée : -")
            return

        slope_text = "-" if slope is None else f"{slope:.12f}"
        intercept_text = "-" if intercept is None else f"{intercept:.6f} {axis_unit}"
        self.min_label.setText(f"Min : {format(min_val, value_format)} {axis_unit}")
        self.max_label.setText(f"Max : {format(max_val, value_format)} {axis_unit}")
        self.mean_label.setText(f"Moy : {format(mean_val, value_format)} {axis_unit}")
        self.amplitude_label.setText(f"Ampl : {format(amplitude, value_format)} {axis_unit}")
        self.slope_label.setText(f"Pente : {slope_text}")
        self.intercept_label.setText(f"Ordonnée : {intercept_text}")

    def _update_lever_arm_error_display(self) -> None:
        if not self.rotary_radio.isChecked() or self.current_precision_peak is None:
            self.lever_arm_error_label.setText("-")
            return

        lever_arm_mm = self.lever_arm_input.value()
        max_error_deg = abs(self.current_precision_peak)
        error_mm = lever_arm_mm * np.tan(np.radians(max_error_deg))
        self.lever_arm_error_label.setText(
            f"Erreur sur {lever_arm_mm:.0f} mm = {error_mm:.3f} mm"
        )

    def _on_scale_limit_changed(self) -> None:
        self.scale_limit_overridden = True
        if self.measurements is not None:
            self._update_plot()

    def _apply_suggested_y_scale(self, min_val: float, max_val: float) -> None:
        max_absolute_value = max(abs(min_val), abs(max_val))
        suggested_limit = max_absolute_value * 1.2
        if suggested_limit <= 0.0:
            suggested_limit = 0.1

        if not self.scale_limit_overridden:
            self.scale_limit_input.blockSignals(True)
            self.scale_limit_input.setValue(suggested_limit)
            self.scale_limit_input.blockSignals(False)

        applied_limit = max(self.scale_limit_input.value(), 1e-6)
        self.plot_widget.setYRange(-applied_limit, applied_limit, padding=0.0)

    def _set_conformity_label(self, label: QLabel, is_ok: bool) -> None:
        label.setText("OK" if is_ok else "NOK")
        color = "#2e7d32" if is_ok else "#c62828"
        label.setStyleSheet(f"font-weight: bold; color: {color};")

    def _set_conformity_label_pending(self, label: QLabel) -> None:
        label.setText("-")
        label.setStyleSheet("font-weight: bold; color: #d8d8d8;")

    def _update_conformity_status(self) -> None:
        if self.measurements is None:
            self._set_conformity_label_pending(self.reduction_status_label)
            self._set_conformity_label_pending(self.backlash_status_label)
            self._set_conformity_label_pending(self.precision_status_label)
            return

        reduction_delta = abs(self.corrected_ratio_input.value() - self.current_ratio_input.value())
        backlash_delta = abs(self.current_backlash_delta) if self.current_backlash_delta is not None else None
        precision_peak = self.current_precision_peak

        reduction_ok = reduction_delta <= self.reduction_tolerance_input.value()
        backlash_ok = backlash_delta is not None and backlash_delta <= self.backlash_tolerance_input.value()
        precision_ok = precision_peak is not None and precision_peak <= self.precision_tolerance_input.value()

        self._set_conformity_label(self.reduction_status_label, reduction_ok)
        self._set_conformity_label(self.backlash_status_label, backlash_ok)
        self._set_conformity_label(self.precision_status_label, precision_ok)

    def _update_corrected_ratio(self) -> None:
        if self.current_slope is None:
            self.corrected_ratio_input.setEnabled(False)
            self.corrected_ratio_input.setValue(0.0)
            self.new_ratio_numerator_input.setValue(0.0)
            self._update_conformity_status()
            return

        self.corrected_ratio_input.setEnabled(True)
        corrected_ratio = self.current_ratio_input.value() * self.current_slope
        self.corrected_ratio_input.setValue(corrected_ratio)
        rounded_new_numerator = round(self.ratio_denominator_input.value() * corrected_ratio)
        self.new_ratio_numerator_input.setValue(rounded_new_numerator)
        self._update_conformity_status()

    def _update_current_ratio_from_fraction(self) -> None:
        numerator = self.ratio_numerator_input.value()
        denominator = self.ratio_denominator_input.value()

        if np.isclose(denominator, 0.0):
            self.current_ratio_input.setValue(0.0)
            self._update_corrected_ratio()
            return

        current_ratio = numerator / denominator
        self.current_ratio_input.setValue(current_ratio)
        self._update_corrected_ratio()

    def _update_corrected_backlash(self) -> None:
        if self.current_backlash_delta is None:
            self.corrected_backlash_input.setEnabled(False)
            self.corrected_backlash_input.setValue(0.0)
            self._update_conformity_status()
            return

        self.corrected_backlash_input.setEnabled(True)
        corrected_backlash = self.current_backlash_input.value() + self.current_backlash_delta
        self.corrected_backlash_input.setValue(corrected_backlash)
        self._update_conformity_status()
