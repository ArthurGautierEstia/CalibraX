from PyQt6.QtWidgets import QWidget, QVBoxLayout, QTabWidget
from widgets.calibration_view.measurement_widget import MeasurementWidget
from widgets.calibration_view.correction_table_widget import CorrectionTableWidget
from widgets.calibration_view.optimized_widget import OptimizedWidget
from widgets.calibration_view.external_axis_widget import ExternalAxisWidget

class CalibrationView(QWidget):
    """Vue de calibration du robot"""

    def __init__(self, parent: QWidget = None):
            super().__init__(parent)
            
            # ====================================================================
            # RÉGION: Initialisation des widgets
            # ====================================================================
            self.measurement_widget = MeasurementWidget()
            self.correction_widget = CorrectionTableWidget()
            self.optimized_widget = OptimizedWidget()
            self.external_axis_widget = ExternalAxisWidget()
            self.tab_widget = QTabWidget()

            self._setup_ui()

    def _setup_ui(self) -> None:
        """Configure l'interface utilisateur pour la vue du robot"""
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        
        # Onglet Géométrique
        geometrique_tab = QWidget()
        geometrique_layout = QVBoxLayout(geometrique_tab)
        geometrique_layout.setSpacing(5)
        geometrique_layout.addWidget(self.measurement_widget)
        geometrique_layout.addWidget(self.correction_widget)
        self.tab_widget.addTab(geometrique_tab, "Géométrique")
        
        # Onglet Optimisée
        optimise_tab = QWidget()
        optimise_layout = QVBoxLayout(optimise_tab)
        optimise_layout.setSpacing(5)
        optimise_layout.addWidget(self.optimized_widget)
        self.tab_widget.addTab(optimise_tab, "Optimisée")
        
        # Onglet Axe externe
        external_axis_tab = QWidget()
        external_axis_layout = QVBoxLayout(external_axis_tab)
        external_axis_layout.setSpacing(5)
        external_axis_layout.addWidget(self.external_axis_widget)
        self.tab_widget.addTab(external_axis_tab, "Axe externe")
        
        layout.addWidget(self.tab_widget)


    def get_measurement_widget(self) -> MeasurementWidget:
        """Retourne le widget d'import des mesures"""
        return self.measurement_widget

    def get_correction_widget(self) -> CorrectionTableWidget:
        """Retourne le widget de tableau des corrections"""
        return self.correction_widget

    def get_external_axis_widget(self) -> ExternalAxisWidget:
        """Retourne le widget d'axe externe"""
        return self.external_axis_widget

    def get_optimized_widget(self) -> OptimizedWidget:
        """Retourne le widget d'optimisation DH"""
        return self.optimized_widget