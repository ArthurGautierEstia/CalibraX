from PyQt6.QtWidgets import QWidget, QVBoxLayout, QScrollArea

from widgets.robot_view.robot_configuration_widget import RobotConfigurationWidget
from widgets.robot_view.robot_mgi_configuration_widget import RobotMgiConfigurationWidget

class RobotView(QWidget):

    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        
        # ====================================================================
        # RÉGION: Initialisation des widgets
        # ====================================================================
        self.configuration_widget = RobotConfigurationWidget()
        self.mgi_configuration_widget = RobotMgiConfigurationWidget()

        self._setup_ui()
    
    def _setup_ui(self) -> None:
        """Configure l'interface utilisateur pour la vue du robot"""
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        self.configuration_widget.add_tab(self.mgi_configuration_widget, "MGI")
        layout.addWidget(self.configuration_widget)
    
    def get_configuration_widget(self) -> RobotConfigurationWidget:
        return self.configuration_widget

    def get_mgi_configuration_widget(self) -> RobotMgiConfigurationWidget:
        return self.mgi_configuration_widget
