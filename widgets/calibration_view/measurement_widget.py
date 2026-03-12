from typing import Dict, List, Optional, Any
from PyQt6.QtWidgets import (
    QLayout, QWidget, QVBoxLayout, QGridLayout, QLabel,
    QPushButton, QLineEdit, QTreeWidget, QTreeWidgetItem,
    QTableWidget, QTableWidgetItem, QAbstractItemView, QComboBox, QHBoxLayout, QCheckBox
)
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtGui import QFont
import math
import numpy as np


class DHCellWidget(QWidget):
    """Widget personnalisé pour une cellule DH : checkbox + valeur en disposition horizontale"""
    
    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(4)
        
        # Checkbox
        self.checkbox = QCheckBox()
        layout.addWidget(self.checkbox)
        
        # Label pour la valeur
        self.label = QLabel("")
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.label)
        
        self.setLayout(layout)
    
    def set_value(self, value: str) -> None:
        """Définit la valeur affichée"""
        self.label.setText(value)
    
    def get_value(self) -> str:
        """Récupère la valeur affichée"""
        return self.label.text()
    
    def set_checked(self, checked: bool) -> None:
        """Définit l'état du checkbox"""
        self.checkbox.setChecked(checked)
    
    def is_checked(self) -> bool:
        """Récupère l'état du checkbox"""
        return self.checkbox.isChecked()

class MeasurementWidget(QWidget):
    """Widget pour l'importation et la gestion des mesures"""
    
    # Signaux
    import_measurements_requested = pyqtSignal()
    clear_measurements_requested = pyqtSignal()
    apply_parameters_requested = pyqtSignal()
    set_as_reference_requested = pyqtSignal()
    repere_selected = pyqtSignal(str)  # nom du repère
    display_mode_changed = pyqtSignal(str)  # display_mode
    rotation_type_changed = pyqtSignal(str)  # rotation_type

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.measurements: List[Dict[str, Any]] = []  # Stocker les mesures importées
        self.setup_ui()
    
    def setup_ui(self) -> None:
        """Initialise l'interface du widget"""
        main_layout = QVBoxLayout(self)
        
        # Titre
        titre = QLabel("Mesures robot")
        titre.setStyleSheet("font-size: 14px; font-weight: bold;")
        main_layout.addWidget(titre)
        
        # === HAUT : Nom du fichier + Sélecteurs ===
        top_layout = QGridLayout()
        
        self.lineEdit_measure_filename = QLineEdit()
        self.lineEdit_measure_filename.setReadOnly(False)
        self.lineEdit_measure_filename.setPlaceholderText("Fichier de mesure")
        top_layout.addWidget(self.lineEdit_measure_filename, 0, 0)
        
        label_1 = QLabel("Afficher : ")
        label_1.setAlignment(Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter)
        self.display_mode = QComboBox()
        self.display_mode.addItems(["Repères", "Ecarts"])
        self.display_mode.currentTextChanged.connect(self.display_mode_changed)
        top_layout.addWidget(label_1, 0, 1)
        top_layout.addWidget(self.display_mode, 0, 2)
        
        label_2 = QLabel("Convention d'angles : ")
        label_2.setAlignment(Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignVCenter)
        self.rotation_type = QComboBox()
        self.rotation_type.addItems(["Euler XYZ", "Fixed XYZ", "Euler ZYX", "Fixed ZYX"])
        self.rotation_type.currentTextChanged.connect(self.rotation_type_changed)
        top_layout.addWidget(label_2, 0, 3)
        top_layout.addWidget(self.rotation_type, 0, 4)

        top_layout.setColumnStretch(0, 2)
        top_layout.setColumnStretch(1, 1)
        top_layout.setColumnStretch(2, 1)
        top_layout.setColumnStretch(3, 1)
        top_layout.setColumnStretch(4, 1)

        main_layout.addLayout(top_layout)
        
        # === MILIEU : Arbre des repères + Table des mesures ===
        middle_layout = QHBoxLayout()
        
        # Arbre des repères
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Repères"])
        self.tree.itemClicked.connect(self._on_item_clicked)
        middle_layout.addWidget(self.tree, 1)
        middle_layout.setStretch(0, 1)  # Arbre prend 1 part
        
        # Table des mesures (du repère sélectionné)
        self.table_me = QTableWidget(5, 3)
        self.table_me.setHorizontalHeaderLabels(["X", "Y", "Z"])
        self.table_me.setVerticalHeaderLabels(["Translation (mm)", "Rotation (°)", "X axis", "Y axis", "Z axis"])
        self.table_me.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_me.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        middle_layout.addWidget(self.table_me, 1)
        middle_layout.setStretch(1, 2)  # Table prend 2 parts
        
        main_layout.addLayout(middle_layout)

        # === BAS : Boutons alignés horizontalement ===
        buttons_layout = QHBoxLayout()
        
        self.btn_import_me = QPushButton("Importer")
        self.btn_import_me.clicked.connect(self.import_measurements_requested.emit)
        buttons_layout.addWidget(self.btn_import_me)
        
        self.btn_set_as_ref = QPushButton("Définir en Référence")
        self.btn_set_as_ref.clicked.connect(self.set_as_reference_requested.emit)
        buttons_layout.addWidget(self.btn_set_as_ref)

        self.btn_clear = QPushButton("Effacer")
        self.btn_clear.clicked.connect(self.clear_measurements)
        buttons_layout.addWidget(self.btn_clear)

        main_layout.addLayout(buttons_layout)
        
        # === Table DH Mesuré (combinée avec checkboxes) ===
        dh_title = QLabel("Table DH Mesurée")
        dh_title.setStyleSheet("font-size: 14px; font-weight: bold;")
        main_layout.addWidget(dh_title)
        
        # Table DH Mesuré unique avec checkboxes intégrés
        self.table_dh_measured = QTableWidget(6, 4)
        self.table_dh_measured.setHorizontalHeaderLabels(["alpha (deg)", "d (mm)", "theta (deg)", "r (mm)"])
        self.table_dh_measured.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_dh_measured.setHorizontalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh_measured.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.table_dh_measured.horizontalHeader().setDefaultSectionSize(120)
        
        # Initialiser les cellules avec DHCellWidget
        self._initialize_dh_cells()
        
        main_layout.addWidget(self.table_dh_measured)
        
        self.setLayout(main_layout)

        # === BAS : Boutons alignés horizontalement ===
        buttons2_layout = QHBoxLayout()
        self.btn_calculate = QPushButton(" Appliquer les paramètres ")
        self.btn_calculate.clicked.connect(self.apply_parameters_requested.emit)
        buttons2_layout.addStretch(2)
        buttons2_layout.addWidget(self.btn_calculate, 1)

        main_layout.addLayout(buttons2_layout)

    
    def _on_item_clicked(self, item: QTreeWidgetItem, column: int) -> None:
        """Callback interne quand un item est cliqué"""
        self.repere_selected.emit(item.text(0))
    
    def _format_value(self, value: float, decimals: int = 2) -> str:
        """Formate une valeur numérique en évitant l'affichage de -0"""
        formatted = f"{value:.{decimals}f}"
        # Remplacer -0 par 0
        return f"0.{'0' * decimals}" if formatted == f"-0.{'0' * decimals}" else formatted
    
    def populate_tree(self, repere_names: List[str]) -> None:
        """Remplit l'arbre avec les noms de repères"""
        self.tree.clear()
        for name in repere_names:
            item = QTreeWidgetItem([name])
            self.tree.addTopLevelItem(item)
    
    def set_measurements_data(self, measurements: List[Dict[str, Any]]) -> None:
        """Stocke les données des mesures"""
        self.measurements = measurements
    
    def set_reference_bold(self, ref_name: str) -> None:
        """Met en gras le repère de référence"""
        for i in range(self.tree.topLevelItemCount()):
            item = self.tree.topLevelItem(i)
            font = QFont()
            font.setBold(item.text(0) == ref_name)
            item.setFont(0, font)
    
    def display_repere_data(self, delta_T: np.ndarray) -> None:
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
        ry = math.atan2(-r31, math.sqrt(r11**2 + r21**2))  # rotation autour Y
        rx = math.atan2(r32, r33)                     # rotation autour X
        rz = math.atan2(r21, r11)                     # rotation autour Z

        # Conversion en degrés
        RX, RY, RZ = math.degrees(rx), math.degrees(ry), math.degrees(rz)

        # --- 3. Afficher dans la table ---
        # Ligne 0 : Translation
        self.table_me.setItem(0, 0, QTableWidgetItem(self._format_value(X, 2)))
        self.table_me.setItem(0, 1, QTableWidgetItem(self._format_value(Y, 2)))
        self.table_me.setItem(0, 2, QTableWidgetItem(self._format_value(Z, 2)))

        # Ligne 1 : Rotation
        self.table_me.setItem(1, 0, QTableWidgetItem(self._format_value(RX, 2)))
        self.table_me.setItem(1, 1, QTableWidgetItem(self._format_value(RY, 2)))
        self.table_me.setItem(1, 2, QTableWidgetItem(self._format_value(RZ, 2)))

        # --- 4. Afficher la matrice delta_T (optionnel) ---
        for i in range(3):  # lignes
            for j in range(3):  # colonnes
                self.table_me.setItem(2 + i, j, QTableWidgetItem(self._format_value(delta_T[i, j], 6)))

        self.table_me.blockSignals(False)
    
    def display_measurement(self, measurement: Dict[str, float]) -> None:
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
        self.table_me.setItem(0, 0, QTableWidgetItem(self._format_value(X, 2)))
        self.table_me.setItem(0, 1, QTableWidgetItem(self._format_value(Y, 2)))
        self.table_me.setItem(0, 2, QTableWidgetItem(self._format_value(Z, 2)))
        
        # Afficher la rotation (ligne 1)
        self.table_me.setItem(1, 0, QTableWidgetItem(self._format_value(A, 2)))
        self.table_me.setItem(1, 1, QTableWidgetItem(self._format_value(B, 2)))
        self.table_me.setItem(1, 2, QTableWidgetItem(self._format_value(C, 2)))
        
        # Calculer la matrice de rotation à partir des angles A, B, C (en degrés)
        A_rad = math.radians(A)
        B_rad = math.radians(B)
        C_rad = math.radians(C)

        ca = math.cos(A_rad)
        sa = math.sin(A_rad)
        cb = math.cos(B_rad)
        sb = math.sin(B_rad)
        cc = math.cos(C_rad)
        sc = math.sin(C_rad)
        
        # Matrice de rotation autour X (A)
        Rx = np.array([
            [1, 0, 0],
            [0, ca, -sa],
            [0, sa,  ca]
        ])
        
        # Matrice de rotation autour Y (B)
        Ry = np.array([
            [cb, 0, sb],
            [0, 1, 0],
            [-sb, 0, cb]
        ])
        
        # Matrice de rotation autour Z (C)
        Rz = np.array([
            [cc, -sc, 0],
            [sc, cc, 0],
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
        self.table_me.setItem(2, 0, QTableWidgetItem(self._format_value(x_axis[0], 6)))
        self.table_me.setItem(2, 1, QTableWidgetItem(self._format_value(x_axis[1], 6)))
        self.table_me.setItem(2, 2, QTableWidgetItem(self._format_value(x_axis[2], 6)))
        
        # Ligne 3 : Y axis
        self.table_me.setItem(3, 0, QTableWidgetItem(self._format_value(y_axis[0], 6)))
        self.table_me.setItem(3, 1, QTableWidgetItem(self._format_value(y_axis[1], 6)))
        self.table_me.setItem(3, 2, QTableWidgetItem(self._format_value(y_axis[2], 6)))
        
        # Ligne 4 : Z axis
        self.table_me.setItem(4, 0, QTableWidgetItem(self._format_value(z_axis[0], 6)))
        self.table_me.setItem(4, 1, QTableWidgetItem(self._format_value(z_axis[1], 6)))
        self.table_me.setItem(4, 2, QTableWidgetItem(self._format_value(z_axis[2], 6)))
        
        self.table_me.blockSignals(False)
    
    def clear_measurements(self) -> None:
        """Efface les mesures"""
        self.lineEdit_measure_filename.clear()
        self.tree.clear()
        self.table_me.clearContents()
        self.table_dh_measured.clearContents()

    def set_measure_filename(self, filename) -> None:
        """Défini le l"""
        self.lineEdit_measure_filename.setText(filename)

    def get_current_repere_name(self) -> Optional[str]:
        """Retourne le nom du repère actuellement sélectionné"""
        current_item = self.tree.currentItem()
        return current_item.text(0) if current_item else None
    
    def display_dh_measured(self, dh_matrix: np.ndarray) -> None:
        """
        Affiche une matrice 4x4 homogène dans la table DH mesurée.
        Affiche uniquement les 3x4 premiers éléments (position et orientation).
        dh_matrix : matrice homogène 4x4 (numpy array)
        """
        self.table_dh_measured.blockSignals(True)
        
        # La table a 6 lignes et 4 colonnes
        # Pour une seule matrice, on l'affiche sur les 3 premières lignes de la matrice de rotation
        for i in range(3):
            for j in range(4):
                cell_widget = self.table_dh_measured.cellWidget(i, j)
                if isinstance(cell_widget, DHCellWidget):
                    cell_widget.set_value(self._format_value(dh_matrix[i, j], 4))
        
        # Vider le reste
        for i in range(3, 6):
            for j in range(4):
                cell_widget = self.table_dh_measured.cellWidget(i, j)
                if isinstance(cell_widget, DHCellWidget):
                    cell_widget.set_value("")
        
        self.table_dh_measured.blockSignals(False)
    
    def _set_table_read_only(self) -> None:
        """Rend toutes les cellules des tableaux en read-only"""
        # Table des mesures
        for row in range(self.table_me.rowCount()):
            for col in range(self.table_me.columnCount()):
                item = self.table_me.item(row, col)
                if item:
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
        
        # Table DH Mesurée
        for row in range(self.table_dh_measured.rowCount()):
            for col in range(self.table_dh_measured.columnCount()):
                item = self.table_dh_measured.item(row, col)
                if item:
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
    
    def _initialize_dh_cells(self) -> None:
        """Initialise la table avec des DHCellWidget pour chaque cellule"""
        for row in range(self.table_dh_measured.rowCount()):
            for col in range(self.table_dh_measured.columnCount()):
                cell_widget = DHCellWidget()
                self.table_dh_measured.setCellWidget(row, col, cell_widget)
    
    def get_dh_checkboxes_state(self) -> Dict[str, bool]:
        """
        Récupère l'état des checkboxes pour les paramètres DH.
        Retourne un dictionnaire avec les clés au format: 'd1', 'd2', 'alpha1', etc.
        """
        states = {}
        col_names = ['alpha', 'd', 'theta', 'r']
        
        for row in range(self.table_dh_measured.rowCount()):
            joint_num = row + 1  # q1 à q6
            for col in range(self.table_dh_measured.columnCount()):
                param_name = f"{col_names[col]}{joint_num}"
                
                # Récupérer le widget DHCellWidget
                cell_widget = self.table_dh_measured.cellWidget(row, col)
                if isinstance(cell_widget, DHCellWidget):
                    states[param_name] = cell_widget.is_checked()
        
        return states
    
    def set_dh_checkboxes_state(self, states: Dict[str, bool]) -> None:
        """
        Définit l'état des checkboxes pour les paramètres DH.
        states : dictionnaire avec les clés au format: 'd1', 'd2', 'alpha1', etc.
        """
        col_names = ['alpha', 'd', 'theta', 'r']
        
        for row in range(self.table_dh_measured.rowCount()):
            joint_num = row + 1  # q1 à q6
            for col in range(self.table_dh_measured.columnCount()):
                param_name = f"{col_names[col]}{joint_num}"
                
                # Récupérer le widget DHCellWidget
                cell_widget = self.table_dh_measured.cellWidget(row, col)
                if isinstance(cell_widget, DHCellWidget) and param_name in states:
                    cell_widget.set_checked(states[param_name])
