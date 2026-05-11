from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QCheckBox, QVBoxLayout, QWidget
import pyqtgraph as pg


class ProgramGraphsWidget(QWidget):
    error_graph_visibility_changed = pyqtSignal(bool)

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.show_error_graph_checkbox = QCheckBox("Afficher la courbe d'erreur")
        self.error_plot = pg.PlotWidget()
        self._measured_curve = None
        self._compensated_curve = None
        self._setup_ui()
        self._setup_connections()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        self.show_error_graph_checkbox.setChecked(False)
        layout.addWidget(self.show_error_graph_checkbox)
        self.error_plot.showGrid(x=True, y=True, alpha=0.3)
        self.error_plot.setTitle("Erreur Y le long de la trajectoire")
        self.error_plot.setLabel("bottom", "Abscisse curviligne (mm)")
        self.error_plot.setLabel("left", "Erreur Y (mm)")
        self.error_plot.addLegend()
        self._measured_curve = self.error_plot.plot([], [], pen=pg.mkPen("#007aff", width=2), name="Reelle")
        self._compensated_curve = self.error_plot.plot([], [], pen=pg.mkPen("#34c759", width=2), name="Compensee")
        layout.addWidget(self.error_plot)
        self.error_plot.setVisible(False)

    def _setup_connections(self) -> None:
        self.show_error_graph_checkbox.toggled.connect(self._on_show_error_graph_toggled)

    def _on_show_error_graph_toggled(self, checked: bool) -> None:
        self.error_plot.setVisible(bool(checked))
        self.error_graph_visibility_changed.emit(bool(checked))

    def is_error_graph_visible(self) -> bool:
        return self.show_error_graph_checkbox.isChecked()

    def set_error_curves(
        self,
        abscissa_mm: list[float],
        measured_error_y_mm: list[float],
        compensated_error_y_mm: list[float],
    ) -> None:
        if not self.is_error_graph_visible():
            return
        self._measured_curve.setData(abscissa_mm, measured_error_y_mm)
        self._compensated_curve.setData(abscissa_mm, compensated_error_y_mm)

    def clear(self) -> None:
        self._measured_curve.setData([], [])
        self._compensated_curve.setData([], [])
