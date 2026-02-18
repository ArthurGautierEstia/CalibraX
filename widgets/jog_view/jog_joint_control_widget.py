from typing import List
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QDoubleSpinBox, QSizePolicy
)
from PyQt6.QtCore import Qt, pyqtSignal


class JogJointControlWidget(QWidget):
    """Widget pour le contrôle Jog (Articulaire et Cartésien)"""
    
    # Signaux
    jog_joint_minus_pressed = pyqtSignal(int)  # index du joint
    jog_joint_minus_released = pyqtSignal(int)  # index du joint
    jog_joint_plus_pressed = pyqtSignal(int)   # index du joint
    jog_joint_plus_released = pyqtSignal(int)   # index du joint
    delta_changed = pyqtSignal(float)
    
    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        
        # Composants
        self.delta_input = QDoubleSpinBox()
        self.jog_joint_buttons_minus: List[QPushButton] = []
        self.jog_joint_buttons_plus: List[QPushButton] = []
        
        self.setup_ui()
        
    def setup_ui(self) -> None:
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
              
        groupbox = QGroupBox("Jog Articulaire")
        groupbox_layout = QVBoxLayout()

        groupbox_layout.addWidget(QLabel()) # label vide pour etre aligné avec articulaire widget

        # Delta
        row_layout = QHBoxLayout()
        label = QLabel(f"Delta")
        label.setFixedWidth(80)
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.delta_input.setValue(0.0)
        self.delta_input.setRange(0, 10)
        self.delta_input.setDecimals(2)
        self.delta_input.setSingleStep(0.1)
        self.delta_input.setFixedWidth(101)
        
        self.delta_input.valueChanged.connect(self.delta_changed.emit)

        row_layout.addWidget(label)
        row_layout.addWidget(self.delta_input)
        row_layout.addStretch()
        groupbox_layout.addLayout(row_layout)

        # Joints
        for i in range(6):
            row_layout = QHBoxLayout()
            label = QLabel(f"q{i+1}")
            label.setFixedWidth(80)
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            
            btn_minus = QPushButton("-")
            btn_minus.pressed.connect(lambda idx=i: self.jog_joint_minus_pressed.emit(idx))
            btn_minus.released.connect(lambda idx=i: self.jog_joint_minus_released.emit(idx))
            btn_minus.setFixedWidth(48)

            btn_plus = QPushButton("+")
            btn_plus.pressed.connect(lambda idx=i: self.jog_joint_plus_pressed.emit(idx))
            btn_plus.released.connect(lambda idx=i: self.jog_joint_plus_released.emit(idx))
            btn_plus.setFixedWidth(48)
            
            row_layout.addWidget(label)
            row_layout.addWidget(btn_minus)
            row_layout.addWidget(btn_plus)
            row_layout.addStretch()
            
            groupbox_layout.addLayout(row_layout)
            
            self.jog_joint_buttons_minus.append(btn_minus)
            self.jog_joint_buttons_plus.append(btn_plus)
        
        groupbox.setLayout(groupbox_layout)
        layout.addWidget(groupbox)
        self.setLayout(layout)

    def set_delta(self, value: float):
        self.delta_input.blockSignals(True)
        self.delta_input.setValue(value)
        self.delta_input.blockSignals(False)
    