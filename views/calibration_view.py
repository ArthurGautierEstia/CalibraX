from PyQt6.QtWidgets import QWidget, QVBoxLayout
from widgets.calibration_view.measurement_widget import MeasurementWidget
from widgets.calibration_view.correction_table_widget import CorrectionTableWidget

class CalibrationView(QWidget):
    """Vue de calibration du robot"""

    def __init__(self, parent: QWidget = None):
            super().__init__(parent)
            
            # ====================================================================
            # RÉGION: Initialisation des widgets
            # ====================================================================
            self.measurement_widget = MeasurementWidget()
            self.correction_widget = CorrectionTableWidget()

            self._setup_ui()

    def _setup_ui(self) -> None:
        """Configure l'interface utilisateur pour la vue du robot"""
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        layout.addWidget(self.measurement_widget)
        layout.addWidget(self.correction_widget)


    def get_measurement_widget(self) -> MeasurementWidget:
        """Retourne le widget d'import des mesures"""
        return self.measurement_widget
    
    def get_correction_widget(self) -> CorrectionTableWidget:
        """Retourne le widget de tableau des corrections"""
        return self.correction_widget