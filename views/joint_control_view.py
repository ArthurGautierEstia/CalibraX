from PyQt6.QtWidgets import QWidget, QVBoxLayout

from widgets.joint_control_view.joints_control_widget import JointsControlWidget
from widgets.joint_control_view.joints_result_table_widget import JointsResultTableWidget


class JointControlView(QWidget):
    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        self.joints_widget = JointsControlWidget()
        self.joints_result_widget = JointsResultTableWidget()
        self._setup_ui()
    
    def _setup_ui(self) -> None:
        """Configure l'interface utilisateur pour la vue du robot"""
        layout = QVBoxLayout(self)
        layout.addWidget(self.joints_widget)
        layout.addWidget(self.joints_result_widget)

    def get_joints_widget(self) -> JointsControlWidget:
        """Retourne le widget de contrôle des articulations"""
        return self.joints_widget
    
    def get_joints_result_widget(self) -> JointsResultTableWidget:
        """Retourne le widget de tableau des résultats des articulations"""
        return self.joints_result_widget