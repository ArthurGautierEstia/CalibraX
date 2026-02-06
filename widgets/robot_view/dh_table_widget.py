from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QTableWidget,
    QTableWidgetItem, QAbstractItemView
)
from PyQt6.QtCore import pyqtSignal

from widgets.robot_view.tool_widget import ToolWidget
from mgi import RobotTool

class DHTableWidget(QWidget):
    """Widget pour la configuration du robot (table DH)"""
    
    # Signaux
    load_config_requested = pyqtSignal()
    text_changed_requested = pyqtSignal()
    export_config_requested = pyqtSignal()
    dh_value_changed = pyqtSignal(int, int, str)  # row, col, value
    tool_changed = pyqtSignal(RobotTool)  # tool changed
    
    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        self.setup_ui()
    
    def setup_ui(self):
        """Initialise l'interface du widget"""
        main_layout = QHBoxLayout(self)
        
        # Partie gauche : Configuration et table DH
        left_layout = QVBoxLayout()
        
        # En-tête
        th_layout = QGridLayout()
        titre1 = QLabel("Configuration robot")
        titre1.setStyleSheet("font-size: 14px; font-weight: bold;")
        left_layout.addWidget(titre1)
        
        self.lineEdit_robot_name_th = QLineEdit()
        self.lineEdit_robot_name_th.setReadOnly(False)
        self.lineEdit_robot_name_th.setPlaceholderText("Nom du robot")
        self.lineEdit_robot_name_th.textChanged.connect(self.text_changed_requested.emit)
        th_layout.addWidget(self.lineEdit_robot_name_th, 0, 0)
        
        self.btn_load_th = QPushButton("Charger")
        self.btn_load_th.clicked.connect(self.load_config_requested.emit)
        th_layout.addWidget(self.btn_load_th, 0, 2)
        
        self.btn_export_th = QPushButton("Exporter")
        self.btn_export_th.clicked.connect(self.export_config_requested.emit)
        th_layout.addWidget(self.btn_export_th, 0, 3)
        
        left_layout.addLayout(th_layout)
        
        # Table DH
        table_dh_titre = QLabel("Table de Denavit-Hartenberg")
        left_layout.addWidget(table_dh_titre)
        
        self.table_dh = QTableWidget(6, 4)
        self.table_dh.setHorizontalHeaderLabels(["alpha (°)", "d (mm)", "theta (°)", "r (mm)"])
        self.table_dh.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh.horizontalHeader().setDefaultSectionSize(90)
        self.table_dh.cellChanged.connect(self._on_cell_changed)
        left_layout.addWidget(self.table_dh)
        
        main_layout.addLayout(left_layout)
        
        # Partie droite : Widget outil
        self.tool_widget = ToolWidget()
        self.tool_widget.tool_changed.connect(self.tool_changed.emit)
        main_layout.addWidget(self.tool_widget)
    
    def _on_cell_changed(self, row, col):
        """Callback interne quand une cellule change"""
        item = self.table_dh.item(row, col)
        if item:
            self.dh_value_changed.emit(row, col, item.text())
    
    def set_robot_name(self, name):
        """Définit le nom du robot"""
        self.lineEdit_robot_name_th.setText(name)
    
    def get_robot_name(self):
        """Récupère le nom du robot"""
        return self.lineEdit_robot_name_th.text()
    
    def set_dh_params(self, params: list[list[float]]):
        """Charge les paramètres DH dans la table"""
        self.table_dh.blockSignals(True)  # Éviter les signaux pendant le chargement
        for i in range(min(7, len(params))):
            for j in range(4):
                value = str(params[i][j]) if i < len(params) and j < len(params[i]) else ""
                self.table_dh.setItem(i, j, QTableWidgetItem(value))
        self.table_dh.blockSignals(False)
    
    def get_dh_params(self):
        """Récupère les paramètres DH depuis la table"""
        params = []
        for i in range(7):
            row = []
            for j in range(4):
                item = self.table_dh.item(i, j)
                row.append(item.text() if item else "")
            params.append(row)
        return params
    
    def set_tool(self, tool: RobotTool):
        """Définit l'outil dans le widget"""
        self.tool_widget.set_tool(tool)
    
    def get_tool(self) -> RobotTool:
        """Récupère l'outil depuis le widget"""
        return self.tool_widget.get_tool()

