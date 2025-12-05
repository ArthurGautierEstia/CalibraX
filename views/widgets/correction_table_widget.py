from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QTableWidget,
    QTableWidgetItem, QAbstractItemView
)
from PyQt5.QtCore import pyqtSignal

class CorrectionTableWidget(QWidget):
    """Widget pour afficher et éditer les corrections 6D"""
    
    # Signaux
    correction_value_changed = pyqtSignal(int, int, str)  # row, col, value
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
    
    def setup_ui(self):
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        
        # Titre
        titre5 = QLabel("Corrections 6D")
        titre5.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(titre5)
        
        # Table des corrections
        self.table_corr = QTableWidget(6, 6)
        self.table_corr.setHorizontalHeaderLabels(["Tx(mm)", "Ty(mm)", "Tz(mm)", "Rx(°)", "Ry(°)", "Rz(°)"])
        self.table_corr.horizontalHeader().setDefaultSectionSize(80)
        self.table_corr.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.table_corr.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.table_corr.cellChanged.connect(self._on_cell_changed)
        
        layout.addWidget(self.table_corr)
        self.setLayout(layout)
    
    def _on_cell_changed(self, row, col):
        """Callback interne quand une cellule change"""
        item = self.table_corr.item(row, col)
        if item:
            self.correction_value_changed.emit(row, col, item.text())
    
    def set_corrections(self, corrections):
        """Charge les corrections dans la table"""
        self.table_corr.blockSignals(True)
        for i in range(min(6, len(corrections))):
            for j in range(6):
                value = str(corrections[i][j]) if j < len(corrections[i]) else ""
                self.table_corr.setItem(i, j, QTableWidgetItem(value))
        self.table_corr.blockSignals(False)
    
    def get_corrections(self):
        """Récupère les corrections depuis la table"""
        corrections = []
        for i in range(6):
            row = []
            for j in range(6):
                item = self.table_corr.item(i, j)
                row.append(item.text() if item else "")
            corrections.append(row)
        return corrections
