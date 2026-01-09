from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSlider, QDoubleSpinBox, QComboBox
)
from PyQt5.QtCore import Qt, pyqtSignal

class CartesianControlWidget(QWidget):
    """Widget pour le contrôle des coordonnées cartésiennes"""
    
    # ============================================================================
    # RÉGION: Signaux
    # ============================================================================
    cartesian_value_changed = pyqtSignal(int, float)  # index (0-5), value
    convention_changed = pyqtSignal(str)  # convention name
    home_position_requested = pyqtSignal()
    axis_limits_config_requested = pyqtSignal()
    
    # ============================================================================
    # RÉGION: Conventions constructeurs
    # ============================================================================
    CONVENTIONS = {
        "Kuka": {
            "labels": ["X (mm)", "Y (mm)", "Z (mm)", "A (°)", "B (°)", "C (°)"],
            "rotation_axes": ["Z", "Y", "X"],  # A autour de Z, B autour de Y, C autour de X
            "description": "X, Y, Z, A(rot Z), B(rot Y), C(rot X)"
        },
        "Fanuc": {
            "labels": ["X (mm)", "Y (mm)", "Z (mm)", "W (°)", "P (°)", "R (°)"],
            "rotation_axes": ["Z", "Y", "X"],  # W=Rz, P=Ry, R=Rx
            "description": "X, Y, Z, W(rot Z), P(rot Y), R(rot X)"
        },
        "ABB": {
            "labels": ["X (mm)", "Y (mm)", "Z (mm)", "Rx (°)", "Ry (°)", "Rz (°)"],
            "rotation_axes": ["X", "Y", "Z"],
            "description": "X, Y, Z, Rx, Ry, Rz"
        },
        "Universal Robots": {
            "labels": ["X (mm)", "Y (mm)", "Z (mm)", "Rx (°)", "Ry (°)", "Rz (°)"],
            "rotation_axes": ["X", "Y", "Z"],
            "description": "X, Y, Z, Rx, Ry, Rz (axis-angle)"
        },
        "Standard": {
            "labels": ["X (mm)", "Y (mm)", "Z (mm)", "Rx (°)", "Ry (°)", "Rz (°)"],
            "rotation_axes": ["X", "Y", "Z"],
            "description": "X, Y, Z, Rx, Ry, Rz"
        }
    }
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # ========================================================================
        # RÉGION: Attributs
        # ========================================================================
        self.sliders_cart = []
        self.spinboxes_cart = []
        self.labels_cart = []
        self.scale_position = 100  # Facteur d'échelle pour X, Y, Z (2 décimales)
        self.scale_rotation = 100  # Facteur d'échelle pour rotations (2 décimales)
        self.current_convention = "Kuka"
        
        # ========================================================================
        # RÉGION: Initialisation UI
        # ========================================================================
        self.setup_ui()
        
    def setup_ui(self):
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        
        # ========================================================================
        # RÉGION: En-tête
        # ========================================================================
        titre = QLabel("Coordonnées cartésiennes")
        titre.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(titre)
        
        # Convention constructeur
        convention_layout = QHBoxLayout()
        convention_label = QLabel("Convention:")
        self.convention_combo = QComboBox()
        self.convention_combo.addItems(list(self.CONVENTIONS.keys()))
        self.convention_combo.setCurrentText(self.current_convention)
        self.convention_combo.currentTextChanged.connect(self._on_convention_changed)
        
        convention_layout.addWidget(convention_label)
        convention_layout.addWidget(self.convention_combo)
        convention_layout.addStretch()
        layout.addLayout(convention_layout)
        
        # Description de la convention
        self.convention_description = QLabel(self.CONVENTIONS[self.current_convention]["description"])
        self.convention_description.setStyleSheet("font-size: 10px; font-style: italic; color: gray;")
        layout.addWidget(self.convention_description)
        
        # ========================================================================
        # RÉGION: Sliders et spinboxes pour les 6 coordonnées cartésiennes
        # ========================================================================
        for i in range(6):
            row_layout = QHBoxLayout()
            
            # Label
            label_text = self.CONVENTIONS[self.current_convention]["labels"][i]
            label = QLabel(label_text)
            label.setMinimumWidth(80)
            self.labels_cart.append(label)
            
            # Déterminer si c'est une position ou une rotation
            is_position = i < 3
            scale = self.scale_position if is_position else self.scale_rotation
            
            # Slider (int)
            slider = QSlider(Qt.Horizontal)
            if is_position:
                slider.setRange(-2000 * scale, 2000 * scale)  # -2000mm à +2000mm
            else:
                slider.setRange(-180 * scale, 180 * scale)  # -180° à +180°

            # SpinBox (float)
            spinbox = QDoubleSpinBox()
            if is_position:
                spinbox.setRange(-2000.00, 2000.00)
                spinbox.setSingleStep(1.00)
            else:
                spinbox.setRange(-180.00, 180.00)
                spinbox.setSingleStep(0.10)
            spinbox.setDecimals(2)

            # Connexions internes
            slider.valueChanged.connect(
                lambda value, s=spinbox, sc=scale: self._update_spinbox(s, value, sc)
            )
            spinbox.valueChanged.connect(
                lambda value, s=slider, sc=scale: self._update_slider(s, value, sc)
            )
            
            # Signal vers le contrôleur (depuis spinbox pour avoir la vraie valeur float)
            spinbox.valueChanged.connect(
                lambda value, idx=i: self.cartesian_value_changed.emit(idx, value)
            )
            
            row_layout.addWidget(label)
            row_layout.addWidget(slider)
            row_layout.addWidget(spinbox)
            layout.addLayout(row_layout)
            
            self.sliders_cart.append(slider)
            self.spinboxes_cart.append(spinbox)

    
    # ============================================================================
    # RÉGION: Méthodes privées
    # ============================================================================
    
    def _update_spinbox(self, spinbox, value, scale):
        """Convertit la valeur du slider (int) en float avec 2 décimales"""
        spinbox.setValue(value / scale)

    def _update_slider(self, slider, value, scale):
        """Convertit la valeur float en int pour le slider"""
        slider.setValue(int(round(value * scale)))
    
    def _on_convention_changed(self, convention_name):
        """Gère le changement de convention constructeur"""
        self.current_convention = convention_name
        
        # Mise à jour des labels
        labels = self.CONVENTIONS[convention_name]["labels"]
        for i, label in enumerate(self.labels_cart):
            label.setText(labels[i])
        
        # Mise à jour de la description
        self.convention_description.setText(self.CONVENTIONS[convention_name]["description"])
        
        # Émettre le signal
        self.convention_changed.emit(convention_name)
    
    # ============================================================================
    # RÉGION: Méthodes publiques
    # ============================================================================
    
    def set_cartesian_value(self, index, value):
        """Définit la valeur d'une coordonnée cartésienne"""
        if 0 <= index < 6:
            is_position = index < 3
            scale = self.scale_position if is_position else self.scale_rotation
            
            self.spinboxes_cart[index].blockSignals(True)
            self.sliders_cart[index].blockSignals(True)
            
            # Le spinbox reçoit la vraie valeur (float)
            self.spinboxes_cart[index].setValue(float(value))
            # Le slider reçoit la valeur multipliée par scale (int)
            self.sliders_cart[index].setValue(int(round(float(value) * scale)))
            
            self.spinboxes_cart[index].blockSignals(False)
            self.sliders_cart[index].blockSignals(False)
    
    def set_all_cartesian(self, values):
        """Définit toutes les valeurs cartésiennes"""
        for i, val in enumerate(values[:6]):
            self.set_cartesian_value(i, val)
    
    def get_cartesian_values(self):
        """Retourne les valeurs actuelles des coordonnées cartésiennes"""
        return [spinbox.value() for spinbox in self.spinboxes_cart]
    
    def get_current_convention(self):
        """Retourne la convention actuellement sélectionnée"""
        return self.current_convention
    
    def update_axis_limits(self, limits):
        """Met à jour les limites des axes
        
        Args:
            limits: Liste de 6 tuples (min, max) pour chaque axe
        """
        for i in range(6):
            min_val, max_val = limits[i]
            is_position = i < 3
            scale = self.scale_position if is_position else self.scale_rotation
            
            current_value = self.sliders_cart[i].value()
            
            # Mettre à jour le slider
            self.sliders_cart[i].setRange(int(min_val * scale), int(max_val * scale))
            if current_value < min_val * scale:
                self.sliders_cart[i].setValue(int(min_val * scale))
            elif current_value > max_val * scale:
                self.sliders_cart[i].setValue(int(max_val * scale))

            # Mettre à jour le spinbox
            current_spinbox_value = self.spinboxes_cart[i].value()
            self.spinboxes_cart[i].setRange(min_val, max_val)
            if current_spinbox_value < min_val:
                self.spinboxes_cart[i].setValue(min_val)
            elif current_spinbox_value > max_val:
                self.spinboxes_cart[i].setValue(max_val)
