from PyQt6.QtWidgets import QWidget, QVBoxLayout, QScrollArea

from widgets.robot_view.robot_configuration_widget import RobotConfigurationWidget

class RobotView(QWidget):

    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        
        # ====================================================================
        # RÉGION: Initialisation des widgets
        # ====================================================================
        self.configuration_widget = RobotConfigurationWidget()

        self._setup_ui()
    
    def _setup_ui(self) -> None:
        """Configure l'interface utilisateur pour la vue du robot"""
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        layout.addWidget(self.configuration_widget)
    
    def get_configuration_widget(self) -> RobotConfigurationWidget:
        return self.configuration_widget
