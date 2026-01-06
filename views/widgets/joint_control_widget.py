from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSlider, QDoubleSpinBox
)
from PyQt5.QtCore import Qt, pyqtSignal

class JointControlWidget(QWidget):
    """Widget pour le contrôle des coordonnées articulaires"""
    
    # Signaux
    joint_value_changed = pyqtSignal(int, float)  # index, value
    home_position_requested = pyqtSignal()
    axis_limits_config_requested = pyqtSignal()
    step_by_step_requested = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.sliders_q = []
        self.spinboxes_q = []
        self.scale = 100  # Facteur d'échelle pour les sliders (2 décimales)
        self.setup_ui()
        
    def setup_ui(self):
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        
        # Titre
        titre3 = QLabel("Coordonnées articulaires")
        titre3.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(titre3)
        
        # Sliders et spinboxes pour les 6 joints
        for i in range(6):
            row_layout = QHBoxLayout()
            label = QLabel(f"q{i+1} (°)")
            # Facteur d'échelle (pour 2 décimales)
              # 1 unité slider = 0.01 spinbox

            # Slider (int)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-180 * self.scale, 180 * self.scale)

            # SpinBox (float)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(-180.00, 180.00)
            spinbox.setDecimals(2)
            spinbox.setSingleStep(0.10)

            # Connexions
            slider.valueChanged.connect(lambda  value, s=spinbox : self.update_spinbox(s,value))
            spinbox.valueChanged.connect(lambda value, s=slider: self.update_slider(s,value))

            
            # Signal vers le contrôleur (depuis spinbox pour avoir la vraie valeur float)
            spinbox.valueChanged.connect(lambda value, idx=i: self.joint_value_changed.emit(idx, value))
            
            row_layout.addWidget(label)
            row_layout.addWidget(slider)
            row_layout.addWidget(spinbox)
            layout.addLayout(row_layout)
            
            self.sliders_q.append(slider)
            self.spinboxes_q.append(spinbox)
        
        # Boutons de configuration
        btn_layout = QVBoxLayout()
        btn_grid = QGridLayout()
        
        self.btn_limits = QPushButton("Paramètrage des axes")
        self.btn_limits.clicked.connect(self.axis_limits_config_requested.emit)
        btn_grid.addWidget(self.btn_limits, 0, 0)
        
        self.btn_home_position = QPushButton("Position home")
        self.btn_home_position.clicked.connect(self.home_position_requested.emit)
        btn_grid.addWidget(self.btn_home_position, 0, 1)
        
        btn_layout.addLayout(btn_grid)
        layout.addLayout(btn_layout)
        
        self.btn_step = QPushButton("Affichage pas à pas")
        self.btn_step.clicked.connect(self.step_by_step_requested.emit)
        layout.addWidget(self.btn_step)
        
        self.setLayout(layout)
    
    def update_spinbox(self,spinbox, value):
        # Convertit le slider (int) en float avec 2 décimales
        spinbox.blockSignals(True)
        spinbox.setValue(value / self.scale)
        spinbox.blockSignals(False)

    def update_slider(self, slider,value):
        # Convertit le float en int pour le slider
        slider.blockSignals(True)
        slider.setValue(int(round(value * self.scale)))
        slider.blockSignals(False)
    
    def set_joint_value(self, index, value):
        """Définit la valeur d'un joint"""
        if 0 <= index < 6:
            self.spinboxes_q[index].blockSignals(True)
            self.sliders_q[index].blockSignals(True)
            
            # Le spinbox reçoit la vraie valeur (float)
            self.spinboxes_q[index].setValue(float(value))
            # Le slider reçoit la valeur multipliée par scale (int)
            self.sliders_q[index].setValue(int(round(value * self.scale)))
            
            self.spinboxes_q[index].blockSignals(False)
            self.sliders_q[index].blockSignals(False)
    
    def set_all_joints(self, values):
        """Définit toutes les valeurs de joints"""
        for i, val in enumerate(values[:6]):
            self.set_joint_value(i, val)
    
    def update_axis_limits(self, limits):
        """Met à jour les limites des axes"""
        for i in range(6):
            min_val, max_val = limits[i]
            current_value = self.sliders_q[i].value()
            
            # Mettre à jour le slider
            self.sliders_q[i].setRange(min_val*self.scale, max_val*self.scale)
            if current_value < min_val:
                self.sliders_q[i].setValue(min_val)
            elif current_value > max_val:
                self.sliders_q[i].setValue(max_val)
            
            # Mettre à jour le spinbox
            current_spinbox_value = self.spinboxes_q[i].value()
            self.spinboxes_q[i].setRange(min_val, max_val)
            if current_spinbox_value < min_val:
                self.spinboxes_q[i].setValue(min_val)
            elif current_spinbox_value > max_val:
                self.spinboxes_q[i].setValue(max_val)
