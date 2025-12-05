from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QTableWidget, QTableWidgetItem, QPushButton, QAbstractItemView
)
from PyQt5.QtCore import pyqtSignal

class ResultTableWidget(QWidget):
    """Widget pour afficher les positions cartésiennes (TCP)"""
    
    # Signaux
    jog_increment_requested = pyqtSignal(int, int)  # row, delta (+1 ou -1)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
    
    def setup_ui(self):
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        
        # Titre
        titre4 = QLabel("Positions cartésiennes")
        titre4.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(titre4)
        
        # Table des résultats
        self.result_table = QTableWidget(6, 4)
        self.result_table.setHorizontalHeaderLabels(["TCP", "TCP Corr", "Ecarts", "Jog"])
        self.result_table.setVerticalHeaderLabels(["X (mm)", "Y (mm)", "Z (mm)", "A (°)", "B (°)", "C (°)"])
        self.result_table.horizontalHeader().setDefaultSectionSize(110)
        self.result_table.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.result_table.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        
        # Ajouter les boutons + et - dans la colonne Jog (colonne 3)
        for row in range(6):
            # Créer les boutons
            btn_plus = QPushButton("+")
            btn_minus = QPushButton("-")
            
            # Connecter les signaux avec le numéro de ligne
            btn_plus.clicked.connect(lambda checked, r=row: self.jog_increment_requested.emit(r, +1))
            btn_minus.clicked.connect(lambda checked, r=row: self.jog_increment_requested.emit(r, -1))
            
            # Créer un layout horizontal pour les deux boutons
            btn_layout = QHBoxLayout()
            btn_layout.addWidget(btn_minus)
            btn_layout.addWidget(btn_plus)
            btn_layout.setContentsMargins(0, 0, 0, 0)
            
            # Créer un widget conteneur pour le layout
            cell_widget = QWidget()
            cell_widget.setLayout(btn_layout)
            
            # Insérer le widget dans la cellule (colonne 3)
            self.result_table.setCellWidget(row, 3, cell_widget)
        
        layout.addWidget(self.result_table)
        self.setLayout(layout)
    
    def update_results(self, tcp_pose, corrected_tcp_pose, deviations):
        """
        Met à jour les résultats affichés
        
        Args:
            tcp_pose: array [X, Y, Z, A, B, C] du TCP standard
            corrected_tcp_pose: array [X, Y, Z, A, B, C] du TCP corrigé
            deviations: array [dX, dY, dZ, dA, dB, dC] des écarts
        """
        self.result_table.blockSignals(True)
        
        for row in range(6):
            # Colonne 0: TCP standard
            if row < 3:  # Position
                self.result_table.setItem(row, 0, QTableWidgetItem(f"{tcp_pose[row]:.2f}"))
            else:  # Orientation
                self.result_table.setItem(row, 0, QTableWidgetItem(f"{tcp_pose[row]:.4f}"))
            
            # Colonne 1: TCP corrigé
            if row < 3:  # Position
                self.result_table.setItem(row, 1, QTableWidgetItem(f"{corrected_tcp_pose[row]:.2f}"))
            else:  # Orientation
                self.result_table.setItem(row, 1, QTableWidgetItem(f"{corrected_tcp_pose[row]:.4f}"))
            
            # Colonne 2: Écarts
            if row < 3:  # Position
                self.result_table.setItem(row, 2, QTableWidgetItem(f"{deviations[row]:.2f}"))
            else:  # Orientation
                self.result_table.setItem(row, 2, QTableWidgetItem(f"{deviations[row]:.4f}"))
        
        self.result_table.blockSignals(False)
    
    def get_value(self, row, col):
        """Récupère la valeur d'une cellule"""
        item = self.result_table.item(row, col)
        if item:
            try:
                return float(item.text())
            except ValueError:
                return 0.0
        return 0.0
    
    def set_value(self, row, col, value):
        """Définit la valeur d'une cellule"""
        if row < 3:  # Position
            self.result_table.setItem(row, col, QTableWidgetItem(f"{value:.2f}"))
        else:  # Orientation
            self.result_table.setItem(row, col, QTableWidgetItem(f"{value:.4f}"))
