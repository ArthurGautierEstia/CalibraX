from typing import List
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, 
    QLabel, QDoubleSpinBox
)


class JogTCPVisualizationWidget(QWidget):
    """Widget pour la visualisation des coordonnées TCP (lecture seule)"""
    
    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        
        # Données internes
        self._tcp_position: List[float] = [0.0] * 3  # X, Y, Z
        self._tcp_orientation: List[float] = [0.0] * 3  # A, B, C
        
        # UI
        self.spinbox_x = QDoubleSpinBox()
        self.spinbox_y = QDoubleSpinBox()
        self.spinbox_z = QDoubleSpinBox()
        self.spinbox_a = QDoubleSpinBox()
        self.spinbox_b = QDoubleSpinBox()
        self.spinbox_c = QDoubleSpinBox()
        
        self.setup_ui()
        
    def setup_ui(self) -> None:
        """Initialise l'interface du widget"""

        layout = QHBoxLayout(self)

        groupbox = QGroupBox("Coordonnées TCP")
        groupbox_layout = QVBoxLayout(groupbox)
        
        # Grid pour les coordonnées

        labels = ["X:", "Y:", "Z:", "A:", "B:", "C:"]
        spinboxes = [
            self.spinbox_x,
            self.spinbox_y,
            self.spinbox_z,
            self.spinbox_a,
            self.spinbox_b,
            self.spinbox_c
        ]

        for i, (label_text, spinbox) in enumerate(zip(labels, spinboxes)):
            hbox = QHBoxLayout()

            label = QLabel(label_text)
            label.setFixedWidth(80)
            label.setAlignment(Qt.AlignCenter)
            hbox.addWidget(label)

            spinbox.setDecimals(2)
            if i < 3:
                spinbox.setRange(-5000, 5000)
            else:
                spinbox.setRange(-180, 180)
            spinbox.setEnabled(False)

            hbox.addWidget(spinbox)
            groupbox_layout.addLayout(hbox)

        groupbox_layout.addStretch()

        layout.addWidget(groupbox)
        
    
    def set_tcp_pose(self, tcp_pose: List[float]) -> None:
        """Met à jour l'affichage de la pose TCP (position et orientation)
        
        Args:
            position: [X, Y, Z] en mm
            orientation: [Rx, Ry, Rz] en degrés
        """
        # Bloquer les signaux pour éviter les boucles
        self.spinbox_x.blockSignals(True)
        self.spinbox_y.blockSignals(True)
        self.spinbox_z.blockSignals(True)
        self.spinbox_a.blockSignals(True)
        self.spinbox_b.blockSignals(True)
        self.spinbox_c.blockSignals(True)

        self._tcp_pose = list(tcp_pose[:6])
        
        # Position
        self.spinbox_x.setValue(tcp_pose[0])
        self.spinbox_y.setValue(tcp_pose[1])
        self.spinbox_z.setValue(tcp_pose[2])
        
        # Orientation
        self.spinbox_a.setValue(tcp_pose[3])
        self.spinbox_b.setValue(tcp_pose[4])
        self.spinbox_c.setValue(tcp_pose[5])
        
        # Débloquer les signaux
        self.spinbox_x.blockSignals(False)
        self.spinbox_y.blockSignals(False)
        self.spinbox_z.blockSignals(False)
        self.spinbox_a.blockSignals(False)
        self.spinbox_b.blockSignals(False)
        self.spinbox_c.blockSignals(False)
    
    def get_tcp_pose(self) -> List[float]:
        """Retourne la pose TCP actuelle (position, orientation)"""
        return self._tcp_pose.copy()
