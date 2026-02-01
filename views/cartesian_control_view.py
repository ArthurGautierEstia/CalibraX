from PyQt5.QtWidgets import QWidget, QVBoxLayout

from widgets.cartesian_control_view.cartesian_control_widget import CartesianControlWidget
from widgets.correction_table_widget import CorrectionTableWidget
from widgets.cartesian_control_view.mgi_solutions_widget import MgiSolutionsWidget
from widgets.cartesian_control_view.mgi_configuration_selector_widget import MgiConfigurationSelectorWidget


class CartesianControlView(QWidget):
    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        self.cartesian_control_widget = CartesianControlWidget()
        self.mgi_solutions_widget = MgiSolutionsWidget()
        self.correction_widget = CorrectionTableWidget()

        self._setup_ui()
    
    def _setup_ui(self) -> None:
        """Configure l'interface utilisateur pour la vue du robot"""
        layout = QVBoxLayout(self)
        layout.addWidget(self.cartesian_control_widget)
        layout.addWidget(self.mgi_solutions_widget)
        layout.addWidget(self.correction_widget)

    def get_cartesian_control_widget(self) -> CartesianControlWidget:
        return self.cartesian_control_widget

    def get_mgi_solutions_widget(self) -> MgiSolutionsWidget:
        """Retourne le widget de contrôle des axes"""
        return self.mgi_solutions_widget
    
    def get_configuration_selector_widget(self) -> MgiConfigurationSelectorWidget:
        """Retourne le widget de tableau des résultats"""
        return self.mgi_solutions_widget.get_config_selector()
    
    def get_correction_widget(self) -> CorrectionTableWidget:
        """Retourne le widget de tableau des corrections"""
        return self.correction_widget
