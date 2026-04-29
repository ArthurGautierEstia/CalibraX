from typing import List
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSlider, QDoubleSpinBox, QSizePolicy
)
from PyQt6.QtCore import Qt, pyqtSignal
from utils.mgi import MgiConfigKey


class JointsControlWidget(QWidget):
    """Widget pour le contrÃ´le des coordonnÃ©es articulaires"""
    
    # Signaux
    joint_value_changed = pyqtSignal(int, float)  # index, value
    home_position_requested = pyqtSignal()
    position_zero_requested = pyqtSignal()
    position_calibration_requested = pyqtSignal()
    
    def __init__(self, parent: QWidget = None, compact: bool = False) -> None:
        super().__init__(parent)
        
        # DonnÃ©es internes
        self._joint_values: List[float] = [0.0] * 6  # Valeurs rÃ©elles (prÃ©cision complÃ¨te)
        self._axis_limits: List[tuple[float, float]] = [(-180.0, 180.0) for _ in range(6)]
        self._current_axis_config: MgiConfigKey = MgiConfigKey.FUN
        self._compact = bool(compact)
        
        # UI
        self.configuration_label = QLabel("Configuration courante : ")
        self.sliders_q: List[QSlider] = []
        self.spinboxes_q: List[QDoubleSpinBox] = []

        self._SLIDER_MAX: int = 1000
        
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        self.setup_ui()
        
    def setup_ui(self) -> None:
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        if self._compact:
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(4)
        
        # Titre
        if not self._compact:
            titre = QLabel("Coordonnees articulaires")
            titre.setStyleSheet("font-size: 14px; font-weight: bold;")
            layout.addWidget(titre)

        # Config
        if not self._compact:
            layout.addWidget(self.configuration_label)
        self.set_configuration(self._current_axis_config)
        
        # Sliders et spinboxes pour les 6 joints
        spinbox_width: int | None = None
        for i in range(6):
            row_layout = QHBoxLayout()
            label = QLabel(f"q{i+1}")
            label.setMinimumWidth(58 if self._compact else 72)

            # Slider (0-100 reprÃ©sente min-max)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, self._SLIDER_MAX)
            slider.setValue(int(self._SLIDER_MAX / 2))  # Milieu par dÃ©faut

            # SpinBox (valeur rÃ©elle)
            spinbox = QDoubleSpinBox()
            spinbox.setRange(self._axis_limits[i][0], self._axis_limits[i][1])
            spinbox.setDecimals(3)
            spinbox.setSingleStep(0.10)
            spinbox.setSuffix(" °")
            spinbox.setValue(0.0)
            if spinbox_width is None:
                spinbox_width = spinbox.sizeHint().width()
            spinbox.setFixedWidth(spinbox_width)

            # Connexions
            slider.valueChanged.connect(lambda value, idx=i: self._on_slider_changed(idx, value))
            spinbox.valueChanged.connect(lambda value, idx=i: self._on_spinbox_changed(idx, value))
            
            row_layout.addWidget(label)
            row_layout.addWidget(slider)
            row_layout.addWidget(spinbox)
            layout.addLayout(row_layout)
            
            self.sliders_q.append(slider)
            self.spinboxes_q.append(spinbox)
        
        # Boutons de configuration
        btn_layout = QVBoxLayout()
        btn_grid = QGridLayout()
        
        self.btn_position_zero = QPushButton("Position 0")
        self.btn_position_zero.clicked.connect(self.position_zero_requested.emit)
        btn_grid.addWidget(self.btn_position_zero, 0, 0)

        self.btn_position_calibration = QPushButton("Position calibration")
        self.btn_position_calibration.clicked.connect(self.position_calibration_requested.emit)
        btn_grid.addWidget(self.btn_position_calibration, 0, 1)
        
        self.btn_home_position = QPushButton("Position home")
        self.btn_home_position.clicked.connect(self.home_position_requested.emit)
        btn_grid.addWidget(self.btn_home_position, 0, 2)
        btn_layout.addLayout(btn_grid)
        layout.addLayout(btn_layout)
        if self._compact:
            self.btn_position_zero.hide()
            self.btn_position_calibration.hide()
            self.btn_home_position.hide()

        self.setLayout(layout)
    
    def _slider_to_value(self, index: int, slider_pos: int) -> float:
        """Convertit une position de slider (0-100) en valeur rÃ©elle"""
        min_val, max_val = self._axis_limits[index]
        return min_val + (slider_pos / float(self._SLIDER_MAX)) * (max_val - min_val)
    
    def _value_to_slider(self, index: int, value: float) -> int:
        """Convertit une valeur rÃ©elle en position de slider (0-100)"""
        min_val, max_val = self._axis_limits[index]
        if max_val == min_val:
            return int(self._SLIDER_MAX / 2)
        ratio = (value - min_val) / (max_val - min_val)
        return int(round(ratio * self._SLIDER_MAX))
    
    def _on_slider_changed(self, index: int, slider_pos: int) -> None:
        """Callback quand le slider change"""
        # Calculer la valeur rÃ©elle depuis le slider
        value = self._slider_to_value(index, slider_pos)
        
        # Mettre Ã  jour la valeur interne
        self._joint_values[index] = value
        
        # Mettre Ã  jour le spinbox sans dÃ©clencher son signal
        self.spinboxes_q[index].blockSignals(True)
        self.spinboxes_q[index].setValue(value)
        self.spinboxes_q[index].blockSignals(False)
        
        # Ã‰mettre le signal de changement
        self.joint_value_changed.emit(index, value)
    
    def _on_spinbox_changed(self, index: int, value: float) -> None:
        """Callback quand le spinbox change"""
        # Mettre Ã  jour la valeur interne
        self._joint_values[index] = value
        
        # Mettre Ã  jour le slider sans dÃ©clencher son signal
        slider_pos = self._value_to_slider(index, value)
        self.sliders_q[index].blockSignals(True)
        self.sliders_q[index].setValue(slider_pos)
        self.sliders_q[index].blockSignals(False)
        
        # Ã‰mettre le signal de changement
        self.joint_value_changed.emit(index, value)
    
    def set_joint_value(self, index: int, value: float) -> None:
        """DÃ©finit la valeur d'un joint (mise Ã  jour externe)"""
        if not (0 <= index < 6):
            return
        
        # Clamper la valeur dans les limites
        min_val, max_val = self._axis_limits[index]
        value = max(min_val, min(max_val, value))
        
        # Mettre Ã  jour la valeur interne
        self._joint_values[index] = value
        
        # Mettre Ã  jour les widgets sans dÃ©clencher les signaux
        self.spinboxes_q[index].blockSignals(True)
        self.sliders_q[index].blockSignals(True)
        
        self.spinboxes_q[index].setValue(value)
        self.sliders_q[index].setValue(self._value_to_slider(index, value))
        
        self.spinboxes_q[index].blockSignals(False)
        self.sliders_q[index].blockSignals(False)
    
    def set_all_joints(self, values: List[float]) -> None:
        """DÃ©finit toutes les valeurs de joints"""
        for i, val in enumerate(values[:6]):
            self.set_joint_value(i, val)
    
    def get_joint_value(self, index: int) -> float:
        """RÃ©cupÃ¨re la valeur rÃ©elle d'un joint"""
        if 0 <= index < 6:
            return self._joint_values[index]
        return 0.0
    
    def get_all_joints(self) -> List[float]:
        """RÃ©cupÃ¨re toutes les valeurs de joints"""
        return self._joint_values.copy()
    
    def update_axis_limits(self, limits: List[tuple[float, float]]) -> None:
        """Met Ã  jour les limites des axes"""
        for i in range(min(6, len(limits))):
            min_val, max_val = limits[i]
            self._axis_limits[i] = (min_val, max_val)
            
            # Mettre Ã  jour la range du spinbox
            self.spinboxes_q[i].setRange(min_val, max_val)
            
            # Clamper la valeur actuelle si nÃ©cessaire
            current_value = self._joint_values[i]
            if current_value < min_val or current_value > max_val:
                clamped_value = max(min_val, min(max_val, current_value))
                self.set_joint_value(i, clamped_value)
            else:
                # Juste mettre Ã  jour le slider (les limites ont changÃ©)
                self.sliders_q[i].blockSignals(True)
                self.sliders_q[i].setValue(self._value_to_slider(i, current_value))
                self.sliders_q[i].blockSignals(False)
    
    def get_axis_limits(self) -> List[tuple[float, float]]:
        """RÃ©cupÃ¨re les limites des axes"""
        return self._axis_limits.copy()
    
    def set_configuration(self, config: MgiConfigKey) -> None:
        """Met Ã  jour le texte de configuration"""
        self.configuration_label.setText(f"Configuration courante : {config.name}")


