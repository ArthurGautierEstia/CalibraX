from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QLabel,
    QPushButton, QLineEdit, QTreeWidget, QTreeWidgetItem,
    QTableWidget, QTableWidgetItem, QAbstractItemView
)
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFont

class MeasurementWidget(QWidget):
    """Widget pour l'importation et la gestion des mesures"""
    
    # Signaux
    import_measurements_requested = pyqtSignal()
    set_as_reference_requested = pyqtSignal()
    calculate_corrections_requested = pyqtSignal()
    repere_selected = pyqtSignal(str)  # nom du repère
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
    
    def setup_ui(self):
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        
        # En-tête
        me_layout = QGridLayout()
        titre2 = QLabel("Mesures robot")
        titre2.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(titre2)
        
        self.label_robot_name_me = QLineEdit()
        self.label_robot_name_me.setReadOnly(False)
        me_layout.addWidget(self.label_robot_name_me, 0, 0)
        
        self.btn_import_me = QPushButton("Importer")
        self.btn_import_me.clicked.connect(self.import_measurements_requested.emit)
        me_layout.addWidget(self.btn_import_me, 0, 1)
        
        # Arbre des repères
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Repères"])
        self.tree.itemClicked.connect(self._on_item_clicked)
        me_layout.addWidget(self.tree, 1, 0)
        
        # Boutons d'action
        tables_btn_layout = QVBoxLayout()
        self.btn_set_as_ref = QPushButton("Définir en Référence")
        self.btn_set_as_ref.clicked.connect(self.set_as_reference_requested.emit)
        self.btn_calculate_corr = QPushButton("Calculer les corrections")
        self.btn_calculate_corr.clicked.connect(self.calculate_corrections_requested.emit)
        tables_btn_layout.addWidget(self.btn_set_as_ref)
        tables_btn_layout.addWidget(self.btn_calculate_corr)
        tables_btn_layout.addStretch()
        me_layout.addLayout(tables_btn_layout, 1, 1)
        
        layout.addLayout(me_layout)
        
        # Table des mesures
        self.table_me = QTableWidget(5, 3)
        self.table_me.setHorizontalHeaderLabels(["X", "Y", "Z"])
        self.table_me.setVerticalHeaderLabels(["Translation (mm)", "Rotation (°)", "X axis", "Y axis", "Z axis"])
        self.table_me.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.table_me.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        layout.addWidget(self.table_me)
        
        self.setLayout(layout)
    
    def _on_item_clicked(self, item, column):
        """Callback interne quand un item est cliqué"""
        self.repere_selected.emit(item.text(0))
    
    def populate_tree(self, repere_names):
        """Remplit l'arbre avec les noms de repères"""
        self.tree.clear()
        for name in repere_names:
            item = QTreeWidgetItem([name])
            self.tree.addTopLevelItem(item)
    
    def set_reference_bold(self, ref_name):
        """Met en gras le repère de référence"""
        for i in range(self.tree.topLevelItemCount()):
            item = self.tree.topLevelItem(i)
            font = QFont()
            font.setBold(item.text(0) == ref_name)
            item.setFont(0, font)
    
    def display_repere_data(self, repere, rotation_matrix):
        """Affiche les données d'un repère dans la table"""
        self.table_me.blockSignals(True)
        
        # Ligne 0 : Translation
        self.table_me.setItem(0, 0, QTableWidgetItem(f"{repere['X']:.2f}"))
        self.table_me.setItem(0, 1, QTableWidgetItem(f"{repere['Y']:.2f}"))
        self.table_me.setItem(0, 2, QTableWidgetItem(f"{repere['Z']:.2f}"))
        
        # Ligne 1 : Rotation
        self.table_me.setItem(1, 0, QTableWidgetItem(f"{repere['A']:.2f}"))
        self.table_me.setItem(1, 1, QTableWidgetItem(f"{repere['B']:.2f}"))
        self.table_me.setItem(1, 2, QTableWidgetItem(f"{repere['C']:.2f}"))
        
        # Lignes 2-4 : Axes X, Y, Z de la matrice de rotation
        for i in range(3):  # Pour chaque axe
            for j in range(3):  # Pour chaque composante
                self.table_me.setItem(2 + i, j, QTableWidgetItem(f"{rotation_matrix[j, i]:.6f}"))
        
        self.table_me.blockSignals(False)
    
    def clear_measurements(self):
        """Efface les mesures"""
        self.label_robot_name_me.setText("")
        self.tree.clear()
        self.table_me.clearContents()
    
    def get_current_repere_name(self):
        """Retourne le nom du repère actuellement sélectionné"""
        current_item = self.tree.currentItem()
        return current_item.text(0) if current_item else None
