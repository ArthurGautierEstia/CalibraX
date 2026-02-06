from typing import List
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QLabel, QPushButton, QComboBox, QDoubleSpinBox
)
from PyQt5.QtCore import Qt, pyqtSignal


class JogCartesianControlWidget(QWidget):
    """Widget pour le contrôle Jog (Articulaire et Cartésien)"""
    
    # Signaux
    jog_cartesian_minus_pressed = pyqtSignal(int)  # index de l'axe cartésien
    jog_cartesian_minus_released = pyqtSignal(int)  # index de l'axe cartésien
    jog_cartesian_plus_pressed = pyqtSignal(int)   # index de l'axe cartésien
    jog_cartesian_plus_released = pyqtSignal(int)  # index de l'axe cartésien
    delta_changed = pyqtSignal(float)
    jog_base_tool_changed = pyqtSignal(str)  # "Base" ou "Tool"
    
    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        
        # Composants
        self.delta_input = QDoubleSpinBox()
        self.jog_cartesian_buttons_minus: List[QPushButton] = []
        self.jog_cartesian_buttons_plus: List[QPushButton] = []
        self.base_tool_combobox: QComboBox = None
        
        self.setup_ui()
        
    def setup_ui(self) -> None:
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
              
        groupbox = QGroupBox("Jog Cartésien")
        groupbox_layout = QVBoxLayout()
        
        # Combobox Base/Tool
        base_tool_h_layout = QHBoxLayout()
        base_tool_label = QLabel("Référentiel")
        base_tool_label.setFixedWidth(80)

        self.base_tool_combobox = QComboBox()
        self.base_tool_combobox.addItem("Base")
        self.base_tool_combobox.addItem("Tool")
        self.base_tool_combobox.setFixedWidth(86)
        
        self.base_tool_combobox.currentTextChanged.connect(self.jog_base_tool_changed.emit)
        
        base_tool_h_layout.addWidget(base_tool_label)
        base_tool_h_layout.addWidget(self.base_tool_combobox)
        base_tool_h_layout.addStretch()

        groupbox_layout.addLayout(base_tool_h_layout)
        
        # Delta
        row_layout = QHBoxLayout()
        label = QLabel(f"Delta")
        label.setFixedWidth(80)
        label.setAlignment(Qt.AlignCenter)

        self.delta_input.setValue(0.0)
        self.delta_input.setRange(0, 10)
        self.delta_input.setDecimals(2)
        self.delta_input.setSingleStep(0.1)
        self.delta_input.setFixedWidth(86)
        self.delta_input.valueChanged.connect(self.delta_changed.emit)

        row_layout.addWidget(label)
        row_layout.addWidget(self.delta_input)
        row_layout.addStretch()
        groupbox_layout.addLayout(row_layout)

        # Boutons jog cartésien (X, Y, Z, A, B, C)
        cartesian_axes = ["X", "Y", "Z", "A", "B", "C"]
        for i, axis in enumerate(cartesian_axes):
            row_layout = QHBoxLayout()
            label = QLabel(axis)
            label.setFixedWidth(80)
            label.setAlignment(Qt.AlignCenter)
            
            btn_minus = QPushButton("-")
            btn_minus.setMaximumWidth(50)
            btn_minus.pressed.connect(lambda idx=i: self.jog_cartesian_minus_pressed.emit(idx))
            btn_minus.released.connect(lambda idx=i: self.jog_cartesian_minus_released.emit(idx))
            
            btn_plus = QPushButton("+")
            btn_plus.setMaximumWidth(50)
            btn_plus.pressed.connect(lambda idx=i: self.jog_cartesian_plus_pressed.emit(idx))
            btn_plus.released.connect(lambda idx=i: self.jog_cartesian_plus_released.emit(idx))
            
            row_layout.addWidget(label)
            row_layout.addWidget(btn_minus)
            row_layout.addWidget(btn_plus)
            row_layout.addStretch()
            
            groupbox_layout.addLayout(row_layout)
            
            self.jog_cartesian_buttons_minus.append(btn_minus)
            self.jog_cartesian_buttons_plus.append(btn_plus)
        
        groupbox.setLayout(groupbox_layout)
        layout.addWidget(groupbox)
        self.setLayout(layout)
    
    def get_base_tool_reference(self) -> str:
        """Retourne le référentiel actuellement sélectionné (Base ou Tool)"""
        return self.base_tool_combobox.currentText()

    def set_delta(self, value: float):
        self.delta_input.blockSignals(True)
        self.delta_input.setValue(value)
        self.delta_input.blockSignals(False)
    