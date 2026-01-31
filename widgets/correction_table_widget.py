from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QTableWidget,
    QTableWidgetItem, QAbstractItemView
)

class CorrectionTableWidget(QWidget):
    """Widget pour afficher et éditer les corrections 6D"""
    
    def __init__(self, parent: QWidget=None):
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
        self.table_corr.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table_corr.setHorizontalHeaderLabels(["Tx(mm)", "Ty(mm)", "Tz(mm)", "Rx(°)", "Ry(°)", "Rz(°)"])
        self.table_corr.horizontalHeader().setDefaultSectionSize(80)
        self.table_corr.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.table_corr.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        
        layout.addWidget(self.table_corr)
        self.setLayout(layout)

    def set_corrections(self, corrections: list[list[str]]):
        """Charge les corrections dans la table"""
        for i in range(min(6, len(corrections))):
            for j in range(6):
                value = str(corrections[i][j]) if j < len(corrections[i]) else ""
                self.table_corr.setItem(i, j, QTableWidgetItem(value))
    
    def get_corrections(self) -> list[list[str]]:
        """Récupère les corrections depuis la table"""
        corrections: list[list[str]] = []
        for i in range(6):
            row: list[str] = []
            for j in range(6):
                item = self.table_corr.item(i, j)
                row.append(item.text() if item else "")
            corrections.append(row)
        return corrections
