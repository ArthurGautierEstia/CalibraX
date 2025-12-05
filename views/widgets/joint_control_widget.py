from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSlider, QSpinBox
)
from PyQt5.QtCore import Qt, pyqtSignal

class JointControlWidget(QWidget):
    """Widget pour le contrôle des coordonnées articulaires"""
    
    # Signaux
    joint_value_changed = pyqtSignal(int, int)  # index, value
    home_position_requested = pyqtSignal()
    axis_limits_config_requested = pyqtSignal()
    step_by_step_requested = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.sliders_q = []
        self.spinboxes_q = []
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
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            spinbox = QSpinBox()
            spinbox.setRange(-180, 180)
            spinbox.setValue(0)
            
            # Synchronisation slider <-> spinbox
            slider.valueChanged.connect(spinbox.setValue)
            spinbox.valueChanged.connect(slider.setValue)
            
            # Signal vers le contrôleur
            slider.valueChanged.connect(lambda val, idx=i: self.joint_value_changed.emit(idx, val))
            
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
    
    def set_joint_value(self, index, value):
        """Définit la valeur d'un joint"""
        if 0 <= index < 6:
            self.spinboxes_q[index].blockSignals(True)
            self.sliders_q[index].blockSignals(True)
            self.spinboxes_q[index].setValue(value)
            self.sliders_q[index].setValue(value)
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
            self.sliders_q[i].setRange(min_val, max_val)
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
