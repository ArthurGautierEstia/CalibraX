from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem, QWidget, QVBoxLayout
from PyQt5.QtCore import pyqtSignal, Qt
import numpy as np


class Matrix3x3Widget(QWidget):
    # Signal émis quand une cellule change : (row, col, nouvelle_valeur)
    cellChanged = pyqtSignal(int, int, float)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()
        self._editable = True
        self._matrix = np.zeros((3, 3))
        
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Créer le QTableWidget 3x3
        self.table = QTableWidget(3, 3)
        self.table.setHorizontalHeaderLabels(['Col 0', 'Col 1', 'Col 2'])
        self.table.setVerticalHeaderLabels(['Row 0', 'Row 1', 'Row 2'])
        
        # Connecter le signal de modification
        self.table.itemChanged.connect(self._on_item_changed)
        
        layout.addWidget(self.table)
        self.setLayout(layout)
    
    def setHorizontalHeaderLabels(self, labels: list[str]):
        self.table.setHorizontalHeaderLabels(labels[:3])
    
    def setVerticalHeaderLabels(self, labels: list[str]):
        self.table.setVerticalHeaderLabels(labels[:3])
        
    def set_matrix(self, matrix: np.ndarray):
        """Définit la matrice à afficher"""
        if matrix.shape != (3, 3):
            raise ValueError("La matrice doit être de dimension 3x3")
        
        self._matrix = matrix.copy()
        
        # Bloquer temporairement les signaux pour éviter les événements parasites
        self.table.blockSignals(True)
        
        for i in range(3):
            for j in range(3):
                item = QTableWidgetItem(str(round(matrix[i, j], 3)))
                item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(i, j, item)
                
                if not self._editable:
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
        
        self.table.blockSignals(False)
        
    def get_matrix(self) -> np.ndarray:
        """Récupère la matrice actuelle"""
        return self._matrix.copy()
    
    def set_editable(self, editable: bool):
        """Définit si la table peut être éditée"""
        self._editable = editable
        
        for i in range(3):
            for j in range(3):
                item = self.table.item(i, j)
                if item:
                    if editable:
                        item.setFlags(item.flags() | Qt.ItemIsEditable)
                    else:
                        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
    
    def _on_item_changed(self, item: QTableWidgetItem):
        """Gestionnaire appelé quand une cellule est modifiée"""
        row = item.row()
        col = item.column()
        
        try:
            value = float(item.text())
            self._matrix[row, col] = value
            self.cellChanged.emit(row, col, value)
        except ValueError:
            # Si la valeur n'est pas un nombre, restaurer l'ancienne valeur
            self.table.blockSignals(True)
            item.setText(str(self._matrix[row, col]))
            self.table.blockSignals(False)
