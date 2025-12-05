from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QTableWidget, 
    QTableWidgetItem, QPushButton, QApplication
)

class AxisLimitsDialog(QDialog):
    """Dialog pour configurer les limites des axes et la position home"""
    
    def __init__(self, parent, current_limits, home_position=None):
        super().__init__(parent)
        self.setWindowTitle("Paramètrage des axes")
        self.setGeometry(100, 100, 500, 400)
        self.current_limits = current_limits
        self.home_position = home_position if home_position else [0, -90, 90, 0, 90, 0]
        
        self.setup_ui()
        self.center_on_screen()
    
    def setup_ui(self):
        """Initialise l'interface du dialog"""
        limits_layout = QVBoxLayout()
        
        # Créer une table pour les limites avec colonne Home Position
        self.table_limits = QTableWidget(6, 3)
        self.table_limits.setHorizontalHeaderLabels(["Min (°)", "Max (°)", "Home (°)"])
        self.table_limits.setVerticalHeaderLabels([f"q{i+1}" for i in range(6)])
        self.table_limits.horizontalHeader().setDefaultSectionSize(100)
        limits_layout.addWidget(self.table_limits)
        
        # Initialiser la table avec les valeurs actuelles
        for i in range(6):
            # Min value
            min_item = QTableWidgetItem(str(self.current_limits[i][0]))
            self.table_limits.setItem(i, 0, min_item)
            
            # Max value
            max_item = QTableWidgetItem(str(self.current_limits[i][1]))
            self.table_limits.setItem(i, 1, max_item)
            
            # Home Position
            home_item = QTableWidgetItem(str(self.home_position[i]))
            self.table_limits.setItem(i, 2, home_item)
        
        # Boutons
        btn_layout = QHBoxLayout()
        btn_ok = QPushButton("Valider")
        btn_cancel = QPushButton("Annuler")
        btn_ok.clicked.connect(self.accept)
        btn_cancel.clicked.connect(self.reject)
        btn_layout.addWidget(btn_ok)
        btn_layout.addWidget(btn_cancel)
        limits_layout.addLayout(btn_layout)
        
        self.setLayout(limits_layout)
    
    def center_on_screen(self):
        """Centre la fenêtre de dialogue sur l'écran"""
        screen_geometry = QApplication.primaryScreen().geometry()
        dialog_width = self.width()
        dialog_height = self.height()
        x = (screen_geometry.width() - dialog_width) // 2
        y = (screen_geometry.height() - dialog_height) // 2
        self.move(x, y)
    
    def get_limits(self):
        """Retourne les nouvelles limites configurées"""
        limits = []
        for i in range(6):
            min_val = int(self.table_limits.item(i, 0).text())
            max_val = int(self.table_limits.item(i, 1).text())
            limits.append((min_val, max_val))
        return limits
    
    def get_home_position(self):
        """Retourne la position home configurée"""
        home_pos = []
        for i in range(6):
            home_val = int(self.table_limits.item(i, 2).text())
            home_pos.append(home_val)
        return home_pos
