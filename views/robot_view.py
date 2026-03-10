from PyQt6.QtWidgets import QWidget, QVBoxLayout

from widgets.robot_view.dh_table_widget import DHTableWidget

class RobotView(QWidget):

    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        
        # ====================================================================
        # RÉGION: Initialisation des widgets
        # ====================================================================
        self.dh_widget = DHTableWidget()

        self._setup_ui()
    
    def _setup_ui(self) -> None:
        """Configure l'interface utilisateur pour la vue du robot"""
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        layout.addWidget(self.dh_widget)
    
    def get_dh_widget(self) -> DHTableWidget:
        return self.dh_widget
