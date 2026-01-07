from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QCheckBox, QTableWidget,
    QTableWidgetItem, QAbstractItemView
)
from PyQt5.QtCore import pyqtSignal

class DHTableWidget(QWidget):
    """Widget pour la configuration du robot (table DH)"""
    
    # Signaux
    load_config_requested = pyqtSignal()
    save_config_requested = pyqtSignal()
    dh_value_changed = pyqtSignal(int, int, str)  # row, col, value
    cad_toggled = pyqtSignal(bool)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
    
    def setup_ui(self):
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        
        # En-tête
        th_layout = QGridLayout()
        titre1 = QLabel("Configuration robot")
        titre1.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(titre1)
        
        self.label_robot_name_th = QLineEdit()
        self.label_robot_name_th.setReadOnly(False)
        th_layout.addWidget(self.label_robot_name_th, 0, 0)
        
        self.cad_cb = QCheckBox("CAD")
        self.cad_cb.stateChanged.connect(self.cad_toggled.emit)
        th_layout.addWidget(self.cad_cb, 0, 1)
        
        self.btn_load_th = QPushButton("Importer une configuration")
        self.btn_load_th.clicked.connect(self.load_config_requested.emit)
        th_layout.addWidget(self.btn_load_th, 0, 2)
        
        self.btn_save_th = QPushButton("Exporter")
        self.btn_save_th.clicked.connect(self.save_config_requested.emit)
        th_layout.addWidget(self.btn_save_th, 0, 3)
        
        layout.addLayout(th_layout)
        
        # Table DH
        table_dh_titre = QLabel("Table de Denavit-Hartenberg")
        layout.addWidget(table_dh_titre)
        
        self.table_dh = QTableWidget(7, 4)
        self.table_dh.setHorizontalHeaderLabels(["alpha (°)", "d (mm)", "theta (°)", "r (mm)"])
        self.table_dh.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.table_dh.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.table_dh.horizontalHeader().setDefaultSectionSize(90)
        self.table_dh.cellChanged.connect(self._on_cell_changed)
        layout.addWidget(self.table_dh)
        
        self.setLayout(layout)
    
    def _on_cell_changed(self, row, col):
        """Callback interne quand une cellule change"""
        item = self.table_dh.item(row, col)
        if item:
            self.dh_value_changed.emit(row, col, item.text())
    
    def set_robot_name(self, name):
        """Définit le nom du robot"""
        self.label_robot_name_th.setText(name)
    
    def get_robot_name(self):
        """Récupère le nom du robot"""
        return self.label_robot_name_th.text()
    
    def set_dh_params(self, params):
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
    
    def set_cad_visible(self, visible):
        """Définit l'état de la checkbox CAD"""
        self.cad_cb.setChecked(visible)
