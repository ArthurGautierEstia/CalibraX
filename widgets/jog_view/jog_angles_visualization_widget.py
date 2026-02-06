from typing import List
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QSlider, QDoubleSpinBox
)
from PyQt5.QtCore import Qt


class JogAnglesVisualizationWidget(QWidget):
    """Widget pour la visualisation des angles articulaires (lecture seule)"""
    
    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        
        # Données internes
        self._joint_values: List[float] = [0.0] * 6
        self._axis_limits: List[tuple[float, float]] = [(-180.0, 180.0) for _ in range(6)]
        
        # UI
        self.sliders_q: List[QSlider] = []
        self.spinboxes_q: List[QDoubleSpinBox] = []
        self._SLIDER_MAX: int = 1000
        
        self.setup_ui()
        
    def setup_ui(self) -> None:
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        
        groupbox = QGroupBox("Angles articulaires")
        groupbox_layout = QVBoxLayout()

        # Sliders et spinboxes pour les 6 joints (lecture seule)
        for i in range(6):
            row_layout = QHBoxLayout()
            label = QLabel(f"q{i+1} (°)")
            
            # Slider (0-100 représente min-max) - Désactivé
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, self._SLIDER_MAX)
            slider.setValue(int(self._SLIDER_MAX / 2))
            slider.setEnabled(False)  # Désactivé
            
            # SpinBox (valeur réelle) - Désactivé
            spinbox = QDoubleSpinBox()
            spinbox.setRange(self._axis_limits[i][0], self._axis_limits[i][1])
            spinbox.setDecimals(2)
            spinbox.setValue(0.0)
            spinbox.setEnabled(False)  # Désactivé
            
            row_layout.addWidget(label)
            row_layout.addWidget(slider)
            row_layout.addWidget(spinbox)
            groupbox_layout.addLayout(row_layout)
            
            self.sliders_q.append(slider)
            self.spinboxes_q.append(spinbox)
        
        groupbox.setLayout(groupbox_layout)
        layout.addWidget(groupbox)
        self.setLayout(layout)
    
    def set_joint_values(self, joint_values: List[float]) -> None:
        """Met à jour l'affichage des valeurs des joints"""
        self._joint_values = joint_values.copy()
        
        for i in range(min(6, len(joint_values))):
            value = joint_values[i]
            self._joint_values[i] = value
            
            # Mettre à jour le slider
            min_val, max_val = self._axis_limits[i]
            slider_pos = int((value - min_val) / (max_val - min_val) * self._SLIDER_MAX)
            slider_pos = max(0, min(self._SLIDER_MAX, slider_pos))
            
            # Bloquer les signaux pour éviter les boucles
            self.sliders_q[i].blockSignals(True)
            self.spinboxes_q[i].blockSignals(True)
            
            self.sliders_q[i].setValue(slider_pos)
            self.spinboxes_q[i].setValue(value)
            
            self.sliders_q[i].blockSignals(False)
            self.spinboxes_q[i].blockSignals(False)
    
    def set_axis_limits(self, axis_limits: List[tuple[float, float]]) -> None:
        """Met à jour les limites des axes"""
        self._axis_limits = axis_limits.copy()
        
        for i in range(min(6, len(axis_limits))):
            min_val, max_val = axis_limits[i]
            self.spinboxes_q[i].setRange(min_val, max_val)
    
    def get_joint_values(self) -> List[float]:
        """Retourne les valeurs actuelles des joints"""
        return self._joint_values.copy()
