from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGridLayout, QLabel,
    QPushButton, QLineEdit, QTreeWidget, QTreeWidgetItem,
    QTableWidget, QTableWidgetItem, QAbstractItemView, QComboBox
)
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFont
from math import atan2, sqrt, degrees, sin, cos, radians
import numpy as np

class MeasurementWidget(QWidget):
    """Widget pour l'importation et la gestion des mesures"""
    
    # Signaux
    import_measurements_requested = pyqtSignal()
    set_as_reference_requested = pyqtSignal()
    calculate_corrections_requested = pyqtSignal()
    repere_selected = pyqtSignal(str)  # nom du repère
    display_mode_changed = pyqtSignal(str)  # display_mode
    rotation_type_changed = pyqtSignal(str)  # rotation_type
    clear_measurements_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.measurements = []  # Stocker les mesures importées
        self.setup_ui()
    
    def setup_ui(self):
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        
        # En-tête
        me_layout = QGridLayout()
        titre2 = QLabel("Mesures robot")
        titre2.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(titre2)
        
        self.label_measure_filename = QLineEdit()
        self.label_measure_filename.setReadOnly(False)
        self.label_measure_filename.setPlaceholderText("Nom du fichier de mesure")
        me_layout.addWidget(self.label_measure_filename, 0, 0)
        
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
        self.btn_clear = QPushButton("Effacer")
        self.btn_clear.clicked.connect(self.clear_measurements)
        tables_btn_layout.addWidget(self.btn_set_as_ref)
        tables_btn_layout.addWidget(self.btn_calculate_corr)
        tables_btn_layout.addWidget(self.btn_clear) 
        tables_btn_layout.addStretch()
        me_layout.addLayout(tables_btn_layout, 1, 1)
        
        layout.addLayout(me_layout)


        tables_display_me = QVBoxLayout()
        choice_table = QGridLayout()
        label_1 = QLabel("Afficher : ")
        display_mode = QComboBox()
        display_mode.addItems(["Repères", "Ecarts"])
        display_mode.currentTextChanged.connect(self.display_mode_changed)
        label_2 = QLabel("Rotation : ")
        rotation_type = QComboBox()
        rotation_type.addItems(["EulerXYZ", "Fixed EulerXYZ"])
        rotation_type.currentTextChanged.connect(self.rotation_type_changed)

        choice_table.addWidget(label_1, 0, 0)
        choice_table.addWidget(display_mode, 0, 1)
        choice_table.addWidget(label_2, 0, 2)
        choice_table.addWidget(rotation_type, 0, 3)

        tables_display_me.addLayout(choice_table)

        # Table des mesures
        self.table_me = QTableWidget(5, 3)
        self.table_me.setHorizontalHeaderLabels(["X", "Y", "Z"])
        self.table_me.setVerticalHeaderLabels(["Translation (mm)", "Rotation (°)", "X axis", "Y axis", "Z axis"])
        self.table_me.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        self.table_me.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        tables_display_me.addWidget(self.table_me)
        layout.addLayout(tables_display_me)
        
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
    
    def set_measurements_data(self, measurements):
        """Stocke les données des mesures"""
        self.measurements = measurements
    
    def set_reference_bold(self, ref_name):
        """Met en gras le repère de référence"""
        for i in range(self.tree.topLevelItemCount()):
            item = self.tree.topLevelItem(i)
            font = QFont()
            font.setBold(item.text(0) == ref_name)
            item.setFont(0, font)
    
    def display_repere_data(self, delta_T):
        """
        Affiche les écarts X, Y, Z, RX, RY, RZ calculés à partir de delta_T dans table_me.
        delta_T : matrice homogène 4x4 (numpy array)
        """
        self.table_me.blockSignals(True)

        # --- 1. Extraire la translation (en mm) ---
        X = delta_T[0, 3]
        Y = delta_T[1, 3]
        Z = delta_T[2, 3]

        # --- 2. Extraire la rotation (angles d'Euler ZYX en degrés) ---
        r11, r12, r13 = delta_T[0, 0], delta_T[0, 1], delta_T[0, 2]
        r21, r22, r23 = delta_T[1, 0], delta_T[1, 1], delta_T[1, 2]
        r31, r32, r33 = delta_T[2, 0], delta_T[2, 1], delta_T[2, 2]

        # Calcul des angles (en radians)
        ry = atan2(-r31, sqrt(r11**2 + r21**2))  # rotation autour Y
        rx = atan2(r32, r33)                     # rotation autour X
        rz = atan2(r21, r11)                     # rotation autour Z

        # Conversion en degrés
        RX, RY, RZ = degrees(rx), degrees(ry), degrees(rz)

        # --- 3. Afficher dans la table ---
        # Ligne 0 : Translation
        self.table_me.setItem(0, 0, QTableWidgetItem(f"{X:.2f}"))
        self.table_me.setItem(0, 1, QTableWidgetItem(f"{Y:.2f}"))
        self.table_me.setItem(0, 2, QTableWidgetItem(f"{Z:.2f}"))

        # Ligne 1 : Rotation
        self.table_me.setItem(1, 0, QTableWidgetItem(f"{RX:.2f}"))
        self.table_me.setItem(1, 1, QTableWidgetItem(f"{RY:.2f}"))
        self.table_me.setItem(1, 2, QTableWidgetItem(f"{RZ:.2f}"))

        # --- 4. Afficher la matrice delta_T (optionnel) ---
        for i in range(3):  # lignes
            for j in range(3):  # colonnes
                self.table_me.setItem(2 + i, j, QTableWidgetItem(f"{delta_T[i, j]:.6f}"))

        self.table_me.blockSignals(False)
    
    def display_measurement(self, measurement):
        """
        Affiche les données d'une mesure (X, Y, Z, A, B, C) dans la table.
        measurement : dictionnaire avec clés 'X', 'Y', 'Z', 'A', 'B', 'C'
        """
        self.table_me.blockSignals(True)
        
        # Extraire les valeurs de la mesure
        X = measurement.get("X", 0)
        Y = measurement.get("Y", 0)
        Z = measurement.get("Z", 0)
        A = measurement.get("A", 0)  # Rotation autour X
        B = measurement.get("B", 0)  # Rotation autour Y
        C = measurement.get("C", 0)  # Rotation autour Z
        
        # Afficher la translation (ligne 0)
        self.table_me.setItem(0, 0, QTableWidgetItem(f"{X:.2f}"))
        self.table_me.setItem(0, 1, QTableWidgetItem(f"{Y:.2f}"))
        self.table_me.setItem(0, 2, QTableWidgetItem(f"{Z:.2f}"))
        
        # Afficher la rotation (ligne 1)
        self.table_me.setItem(1, 0, QTableWidgetItem(f"{A:.2f}"))
        self.table_me.setItem(1, 1, QTableWidgetItem(f"{B:.2f}"))
        self.table_me.setItem(1, 2, QTableWidgetItem(f"{C:.2f}"))
        
        # Calculer la matrice de rotation à partir des angles A, B, C (en degrés)
        A_rad = radians(A)
        B_rad = radians(B)
        C_rad = radians(C)
        
        # Matrice de rotation autour X (A)
        Rx = np.array([
            [1, 0, 0],
            [0, cos(A_rad), -sin(A_rad)],
            [0, sin(A_rad), cos(A_rad)]
        ])
        
        # Matrice de rotation autour Y (B)
        Ry = np.array([
            [cos(B_rad), 0, sin(B_rad)],
            [0, 1, 0],
            [-sin(B_rad), 0, cos(B_rad)]
        ])
        
        # Matrice de rotation autour Z (C)
        Rz = np.array([
            [cos(C_rad), -sin(C_rad), 0],
            [sin(C_rad), cos(C_rad), 0],
            [0, 0, 1]
        ])
        
        # Combinaison : Rz * Ry * Rx (ordre ZYX)
        R = Rz @ Ry @ Rx
        
        # Extraire les composantes vectorielles (colonnes de la matrice)
        # X axis = première colonne
        x_axis = R[:, 0]
        # Y axis = deuxième colonne
        y_axis = R[:, 1]
        # Z axis = troisième colonne
        z_axis = R[:, 2]
        
        # Afficher les composantes (lignes 2, 3, 4)
        # Ligne 2 : X axis
        self.table_me.setItem(2, 0, QTableWidgetItem(f"{x_axis[0]:.6f}"))
        self.table_me.setItem(2, 1, QTableWidgetItem(f"{x_axis[1]:.6f}"))
        self.table_me.setItem(2, 2, QTableWidgetItem(f"{x_axis[2]:.6f}"))
        
        # Ligne 3 : Y axis
        self.table_me.setItem(3, 0, QTableWidgetItem(f"{y_axis[0]:.6f}"))
        self.table_me.setItem(3, 1, QTableWidgetItem(f"{y_axis[1]:.6f}"))
        self.table_me.setItem(3, 2, QTableWidgetItem(f"{y_axis[2]:.6f}"))
        
        # Ligne 4 : Z axis
        self.table_me.setItem(4, 0, QTableWidgetItem(f"{z_axis[0]:.6f}"))
        self.table_me.setItem(4, 1, QTableWidgetItem(f"{z_axis[1]:.6f}"))
        self.table_me.setItem(4, 2, QTableWidgetItem(f"{z_axis[2]:.6f}"))
        
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
