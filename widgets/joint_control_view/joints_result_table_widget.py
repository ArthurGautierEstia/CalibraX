from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel,
    QTableWidget, QTableWidgetItem, QAbstractItemView
)

from utils.str_utils import str_to_float

class JointsResultTableWidget(QWidget):
    """Widget pour afficher les positions cartésiennes (TCP)"""
    
    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        self.setMinimumHeight(280)
        self.setup_ui()
    
    def setup_ui(self):
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        
        # Titre
        titre4 = QLabel("Positions cartésiennes")
        titre4.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(titre4)
        
        # Table des résultats
        self.result_table = QTableWidget(6, 3)
        self.result_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.result_table.setHorizontalHeaderLabels(["TCP", "TCP Corr", "Ecarts"])
        self.result_table.setVerticalHeaderLabels(["X (mm)", "Y (mm)", "Z (mm)", "A (°)", "B (°)", "C (°)"])
        self.result_table.horizontalHeader().setDefaultSectionSize(110)
        self.result_table.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.result_table.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        
        layout.addWidget(self.result_table)
        self.setLayout(layout)
    
    def update_results(self, tcp_pose: list[float], corrected_tcp_pose: list[float], deviations: list[float]):
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
    
    def get_value(self, row: int, col: int):
        """Récupère la valeur d'une cellule"""
        item = self.result_table.item(row, col)
        return str_to_float(item.text()) if item else 0.0
    
    def set_value(self, row: int, col: int, value: float):
        """Définit la valeur d'une cellule"""
        if row < 3:  # Position
            self.result_table.setItem(row, col, QTableWidgetItem(f"{value:.2f}"))
        else:  # Orientation
            self.result_table.setItem(row, col, QTableWidgetItem(f"{value:.4f}"))
