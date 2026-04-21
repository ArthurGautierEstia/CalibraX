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
        self.tolerance_min_line: Optional[pg.InfiniteLine] = None
        self.tolerance_max_line: Optional[pg.InfiniteLine] = None
        self.point_names: List[str] = []
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(5)

        title = QLabel("Axe Externe")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(title)

        import_group = QGroupBox("Import")
        import_group_layout = QVBoxLayout(import_group)
        import_group_layout.setSpacing(5)

        import_layout = QHBoxLayout()
        self.file_label = QLabel("Aucun fichier chargé")
        self.file_label.setStyleSheet("border: 1px solid #555; padding: 2px; background-color: #2a2a2a; color: #d8d8d8;")
        import_layout.addWidget(self.file_label)
        self.btn_import = QPushButton("Importer .txt")
        self.btn_import.clicked.connect(self._import_measurements)
        import_layout.addWidget(self.btn_import)
        import_layout.addStretch()
        import_group_layout.addLayout(import_layout)

        theoretical_layout = QHBoxLayout()
        theoretical_layout.addWidget(QLabel("Plage de mesures :"))
        self.min_input = self._make_coordinate_spinbox(0.0)
        self.max_input = self._make_coordinate_spinbox(9750.0)
        self.step_input = self._make_coordinate_spinbox(250.0, minimum=1.0)
        self.min_input.valueChanged.connect(self._update_plot)
        self.max_input.valueChanged.connect(self._update_plot)
        self.step_input.valueChanged.connect(self._update_plot)
        theoretical_layout.addWidget(QLabel("Min"))
        theoretical_layout.addWidget(self.min_input)
        theoretical_layout.addWidget(QLabel("Max"))
        theoretical_layout.addWidget(self.max_input)
        theoretical_layout.addWidget(QLabel("Step"))
        theoretical_layout.addWidget(self.step_input)
        theoretical_layout.addStretch()
        import_group_layout.addLayout(theoretical_layout)

        tolerance_layout = QHBoxLayout()
        tolerance_layout.addWidget(QLabel("Tolerances :"))
        self.tolerance_min_input = self._make_tolerance_spinbox(-0.05)
        self.tolerance_max_input = self._make_tolerance_spinbox(0.05)
        self.tolerance_min_input.valueChanged.connect(self._update_plot)
        self.tolerance_max_input.valueChanged.connect(self._update_plot)
        tolerance_layout.addWidget(QLabel("Min"))
        tolerance_layout.addWidget(self.tolerance_min_input)
        tolerance_layout.addWidget(QLabel("Max"))
        tolerance_layout.addWidget(self.tolerance_max_input)
        tolerance_layout.addStretch()
        import_group_layout.addLayout(tolerance_layout)

        axis_type_layout = QHBoxLayout()
        axis_type_layout.addWidget(QLabel("Type d'axe externe :"))
        self.linear_radio = QRadioButton("Lineaire")
        self.rotary_radio = QRadioButton("Rotatif")
        self.linear_radio.setChecked(True)
        self.axis_group = QButtonGroup()
        self.axis_group.addButton(self.linear_radio)
        self.axis_group.addButton(self.rotary_radio)
        axis_type_layout.addWidget(self.linear_radio)
        axis_type_layout.addWidget(self.rotary_radio)
        axis_type_layout.addStretch()
        import_group_layout.addLayout(axis_type_layout)

        import_group_layout.addWidget(QLabel("Visualisation des ecarts :"))

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
        import_group_layout.addLayout(coord_layout)

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setMinimumHeight(260)
        self.plot_widget.setMaximumHeight(360)
        self.plot_widget.setBackground("#151515")
        self.plot_widget.setLabel("left", "Ecart (mm)", color="#d8d8d8")
        self.plot_widget.setLabel("bottom", "Valeurs théoriques (mm)")
        self.plot_widget.getAxis("left").enableAutoSIPrefix(False)
        self.plot_widget.getAxis("bottom").enableAutoSIPrefix(False)
        self.plot_widget.getAxis("bottom").setLabel("Valeurs theoriques (mm)", color="#d8d8d8")
        self.plot_widget.getAxis("left").setPen(pg.mkPen("#a8a8a8"))
        self.plot_widget.getAxis("bottom").setPen(pg.mkPen("#a8a8a8"))
        self.plot_widget.getAxis("left").setTextPen(pg.mkPen("#d8d8d8"))
        self.plot_widget.getAxis("bottom").setTextPen(pg.mkPen("#d8d8d8"))
        self.plot_widget.showGrid(x=True, y=True, alpha=0.22)
        legend = self.plot_widget.addLegend(offset=(-85, 10))
        legend.anchor((1, 0), (1, 0))
        import_group_layout.addWidget(self.plot_widget, 0)
        self._build_statistics_group(import_group_layout)
        self._set_statistics_values()

        layout.addWidget(import_group)

        calibration_group = QGroupBox("Calibration")
        calibration_group_layout = QVBoxLayout(calibration_group)
        calibration_group_layout.setSpacing(5)

        ratio_group = QGroupBox("Rapport de reduction")
        ratio_group_layout = QVBoxLayout(ratio_group)
        ratio_group_layout.setSpacing(5)

        ratio_layout = QHBoxLayout()
        ratio_layout.addWidget(QLabel("Actuel"))
        self.current_ratio_input = self._make_ratio_spinbox()
        self.current_ratio_input.valueChanged.connect(self._update_corrected_ratio)
        ratio_layout.addWidget(self.current_ratio_input)
        ratio_layout.addWidget(QLabel("Corrigé"))
        self.corrected_ratio_input = self._make_ratio_spinbox()
        self.corrected_ratio_input.setReadOnly(True)
        self.corrected_ratio_input.setEnabled(False)
        self.corrected_ratio_input.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
        ratio_layout.addWidget(self.corrected_ratio_input)
        ratio_layout.addStretch()
        ratio_group_layout.addLayout(ratio_layout)
        calibration_group_layout.addWidget(ratio_group)

        compensation_group = QGroupBox("Table de compensation")
        compensation_group_layout = QVBoxLayout(compensation_group)
        compensation_group_layout.setSpacing(5)

        compensation_params_layout = QHBoxLayout()
        compensation_params_layout.addWidget(QLabel("Nom de l'axe"))
        self.axis_name_input = QLineEdit("A_1")
        compensation_params_layout.addWidget(self.axis_name_input)
        compensation_params_layout.addWidget(QLabel("Index table aller"))
        self.forward_table_index_input = self._make_table_index_spinbox(3)
        compensation_params_layout.addWidget(self.forward_table_index_input)
        compensation_params_layout.addWidget(QLabel("Index table retour"))
        self.return_table_index_input = self._make_table_index_spinbox(4)
        compensation_params_layout.addWidget(self.return_table_index_input)
        compensation_params_layout.addWidget(QLabel("Position zéro"))
        self.calibration_position_input = self._make_coordinate_spinbox(0.0)
        compensation_params_layout.addWidget(self.calibration_position_input)
        compensation_params_layout.addWidget(QLabel("Direction"))
        self.calibration_reference_combo = QComboBox()
        self.calibration_reference_combo.addItems(["Aller", "Retour"])
        compensation_params_layout.addWidget(self.calibration_reference_combo)
        compensation_params_layout.addStretch()
        compensation_group_layout.addLayout(compensation_params_layout)

        generate_layout = QHBoxLayout()
        self.btn_generate_compensation_table = QPushButton("Exporter la table de compensation bidirectionnelle .SPF")
        self.btn_generate_compensation_table.clicked.connect(self._generate_compensation_table)
        generate_layout.addWidget(self.btn_generate_compensation_table)
        generate_layout.addStretch()
        compensation_group_layout.addLayout(generate_layout)
        calibration_group_layout.addWidget(compensation_group)

        layout.addWidget(calibration_group)
        layout.addStretch()

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

    @staticmethod
    def _make_table_index_spinbox(value: int) -> QSpinBox:
        spinbox = QSpinBox()
        spinbox.setRange(0, 999)
        spinbox.setValue(value)
        return spinbox

    def _build_statistics_group(self, parent_layout: QVBoxLayout) -> None:
        stats_group = QGroupBox("Donnees statistiques")
        stats_layout = QHBoxLayout(stats_group)
        stats_layout.setSpacing(12)

        self.min_label = QLabel("Min: -")
        self.max_label = QLabel("Max: -")
        self.mean_label = QLabel("Moyenne: -")
        self.amplitude_label = QLabel("Amplitude: -")
        self.slope_label = QLabel("Pente regression: -")
        self.intercept_label = QLabel("Ordonnee origine: -")

        stats_layout.addWidget(self.min_label)
        stats_layout.addWidget(self.max_label)
        stats_layout.addWidget(self.mean_label)
        stats_layout.addWidget(self.amplitude_label)
        stats_layout.addWidget(self.slope_label)
        stats_layout.addWidget(self.intercept_label)
        stats_layout.addStretch()
        parent_layout.addWidget(stats_group)

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
            # Mettre à jour le label avec le nom du fichier
            import os
            self.file_label.setText(os.path.basename(file_path))
            
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

    def _read_measurements_file(self, file_path: str) -> list[list[float]]:
        data: list[list[float]] = []
        self.point_names = []
        with open(file_path, "r", encoding="utf-8-sig") as file:
            for line_index, line in enumerate(file):
                line = line.strip()
                if not line:
                    continue
                parts = [part.strip() for part in line.split("\t")]
                if len(parts) < 3:
                    continue
                try:
                    values = [float(part.replace(",", ".")) for part in parts[:3]]
                except ValueError:
                    continue
                data.append(values)
                self.point_names.append(f"Point {line_index + 1}")
        return data

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
        self.tolerance_min_line = None
        self.tolerance_max_line = None

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
        self._add_tolerance_lines()

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
                name="Aller" if first_forward else None,
            )
            first_forward = False
            
            self.plot_widget.plot(
                return_x,
                return_error,
                pen=pg.mkPen(color=return_color, width=2),
                symbol="t",
                symbolBrush=return_color,
                name="Retour" if first_return else None,
            )
            first_return = False
            
            # Ajouter les lignes de régression linéaire pour Aller et Retour
            forward_regression = self._calculate_linear_regression(forward_x, forward_data[:, index])
            if forward_regression is not None:
                slope, intercept = forward_regression
                x_range = np.linspace(np.min(forward_x), np.max(forward_x), 100)
                y_regression = slope * x_range + intercept - x_range  # Soustraire x_range car on affiche les erreurs
                self.plot_widget.plot(
                    x_range,
                    y_regression,
                    pen=pg.mkPen(color=forward_color, width=1, style=Qt.PenStyle.DotLine),
                )
            
            return_regression = self._calculate_linear_regression(return_x, return_data[:, index])
            if return_regression is not None:
                slope, intercept = return_regression
                x_range = np.linspace(np.min(return_x), np.max(return_x), 100)
                y_regression = slope * x_range + intercept - x_range  # Soustraire x_range car on affiche les erreurs
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
            self._set_statistics_values(min_val, max_val, mean_val, amplitude, slope, intercept)
            self._update_corrected_ratio()
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

        forward_compensation = self.theoretical_forward - forward_data
        return_compensation = self.theoretical_return - return_data
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
        )
        default_name = f"COMP_{axis_name}_CEC.SPF"
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

    def _format_compensation_table(
        self,
        axis_name: str,
        forward_index: int,
        return_index: int,
        forward_compensation: np.ndarray,
        return_compensation: np.ndarray,
    ) -> str:
        blocks = [
            self._format_compensation_block(axis_name, forward_index, forward_compensation, direction=1),
            self._format_compensation_block(axis_name, return_index, return_compensation, direction=-1),
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
    ) -> str:
        lines = [
            f"$AN_CEC_INPUT_AXIS[{table_index}]=({axis_name})",
            f"$AN_CEC_OUTPUT_AXIS[{table_index}]=({axis_name})",
            f"$AN_CEC_STEP[{table_index}]={self.step_input.value():.0f}",
            f"$AN_CEC_MIN[{table_index}]={self.min_input.value():.0f}",
            f"$AN_CEC_MAX[{table_index}]={self.max_input.value():.0f}",
            f"$AN_CEC_DIRECTION[{table_index}]={direction}",
            f"$AN_CEC_IS_MODULO[{table_index}]=0",
            "",
        ]
        for value_index, value in enumerate(compensation_values):
            lines.append(f"$AN_CEC[{table_index},{value_index}]={value:.6f}")
        return "\n".join(lines)

    def _set_stats_empty(self) -> None:
        self.current_slope = None
        self._set_statistics_values()
        self._update_corrected_ratio()

    def _set_statistics_values(
        self,
        min_val: Optional[float] = None,
        max_val: Optional[float] = None,
        mean_val: Optional[float] = None,
        amplitude: Optional[float] = None,
        slope: Optional[float] = None,
        intercept: Optional[float] = None,
    ) -> None:
        if min_val is None:
            self.min_label.setText("Min: -")
            self.max_label.setText("Max: -")
            self.mean_label.setText("Moyenne: -")
            self.amplitude_label.setText("Amplitude: -")
            self.slope_label.setText("Pente regression: -")
            self.intercept_label.setText("Ordonnee origine: -")
            return

        slope_text = "-" if slope is None else f"{slope:.12f}"
        intercept_text = "-" if intercept is None else f"{intercept:.6f}"
        self.min_label.setText(f"Min: {min_val:.3f}")
        self.max_label.setText(f"Max: {max_val:.3f}")
        self.mean_label.setText(f"Moyenne: {mean_val:.3f}")
        self.amplitude_label.setText(f"Amplitude: {amplitude:.3f}")
        self.slope_label.setText(f"Pente regression: {slope_text}")
        self.intercept_label.setText(f"Ordonnee origine: {intercept_text}")

    def _add_tolerance_lines(self) -> None:
        min_tolerance = self.tolerance_min_input.value()
        max_tolerance = self.tolerance_max_input.value()
        tolerance_pen = pg.mkPen(color="#c62828", width=1.5, style=Qt.PenStyle.DashLine)
        self.tolerance_min_line = pg.InfiniteLine(
            pos=min_tolerance,
            angle=0,
            pen=tolerance_pen,
            label=f"Tol min {min_tolerance:.3f}",
            labelOpts={"position": 0.02, "color": "#c62828"},
        )
        self.tolerance_max_line = pg.InfiniteLine(
            pos=max_tolerance,
            angle=0,
            pen=tolerance_pen,
            label=f"Tol max {max_tolerance:.3f}",
            labelOpts={"position": 0.02, "color": "#c62828"},
        )
        self.plot_widget.addItem(self.tolerance_min_line, ignoreBounds=True)
        self.plot_widget.addItem(self.tolerance_max_line, ignoreBounds=True)

    def _update_corrected_ratio(self) -> None:
        if self.current_slope is None:
            self.corrected_ratio_input.setEnabled(False)
            self.corrected_ratio_input.setValue(0.0)
            return

        self.corrected_ratio_input.setEnabled(True)
        corrected_ratio = self.current_ratio_input.value() * self.current_slope
        self.corrected_ratio_input.setValue(corrected_ratio)
