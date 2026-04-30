from typing import List, Tuple
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QDoubleSpinBox, QComboBox, QSizePolicy
)
from PyQt6.QtCore import Qt, pyqtSignal

from models.reference_frame import ReferenceFrame
from models.types import Pose6
from widgets.jog_spin_box import JogSpinBox


class CartesianControlWidget(QWidget):
    """Widget pour le contrôle des coordonnées cartésiennes"""

    _COMPACT_ROW_HEIGHT = 28
    _POSE_AXIS_NAMES = ("x", "y", "z", "a", "b", "c")
    
    # ============================================================================
    # RÉ‰GION: Signaux
    # ============================================================================
    cartesian_value_changed = pyqtSignal(int, float)  # index (0-5), value
    convention_changed = pyqtSignal(str)  # convention name
    reference_frame_changed = pyqtSignal(str)
    spinbox_jog_pressed = pyqtSignal(int, int)
    spinbox_jog_released = pyqtSignal(int, int)
    
    # ============================================================================
    # RÉ‰GION: Conventions constructeurs
    # ============================================================================
    CONVENTIONS = {
        "Kuka": {
            "labels": ["X ", "Y ", "Z ", "A ", "B ", "C "],
            "rotation_axes": ["Z", "Y", "X"],  # A autour de Z, B autour de Y, C autour de X
            "description": "X, Y, Z, A(rot Z), B(rot Y), C(rot X)"
        },
        "Fanuc": {
            "labels": ["X ", "Y ", "Z ", "W ", "P ", "R "],
            "rotation_axes": ["Z", "Y", "X"],  # W=Rz, P=Ry, R=Rx
            "description": "X, Y, Z, W(rot Z), P(rot Y), R(rot X)"
        },
        "ABB": {
            "labels": ["X ", "Y ", "Z ", "Rx ", "Ry ", "Rz "],
            "rotation_axes": ["X", "Y", "Z"],
            "description": "X, Y, Z, Rx, Ry, Rz"
        },
        "Universal Robots": {
            "labels": ["X ", "Y ", "Z ", "Rx ", "Ry ", "Rz "],
            "rotation_axes": ["X", "Y", "Z"],
            "description": "X, Y, Z, Rx, Ry, Rz (axis-angle)"
        },
        "Standard": {
            "labels": ["X ", "Y ", "Z ", "Rx ", "Ry ", "Rz "],
            "rotation_axes": ["X", "Y", "Z"],
            "description": "X, Y, Z, Rx, Ry, Rz"
        }
    }
    
    def __init__(self, parent: QWidget = None, compact: bool = False, enable_jog_spin_buttons: bool = False):
        super().__init__(parent)
        
        # ========================================================================
        # RÉ‰GION: Attributs
        # ========================================================================
        # Données internes
        self._cartesian_values: Pose6 = Pose6.zeros()  # Valeurs réelles (précision complète)
        self._axis_limits: List[Tuple[float, float]] = [
            (-2000.0, 2000.0),  # X
            (-2000.0, 2000.0),  # Y
            (-2000.0, 2000.0),  # Z
            (-180.0, 180.0),    # Rotation 1
            (-180.0, 180.0),    # Rotation 2
            (-180.0, 180.0)     # Rotation 3
        ]

        self._SLIDER_MAX: int = 1000
        
        # UI
        self.sliders_cart: List[QSlider] = []
        self.spinboxes_cart: List[QDoubleSpinBox] = []
        self.labels_cart: List[QLabel] = []
        self.reference_label: QLabel | None = None
        self.reference_frame_combo: QComboBox | None = None
        self.current_convention = "Kuka"
        self.current_reference_frame = ReferenceFrame.BASE.value
        self._compact = bool(compact)
        self._enable_jog_spin_buttons = bool(enable_jog_spin_buttons)
        
        # ========================================================================
        # RÉ‰GION: Initialisation UI
        # ========================================================================
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        self.setup_ui()
        self._apply_reference_frame_interaction_mode()
        
    def setup_ui(self) -> None:
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        if self._compact:
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(4)
        
        # ========================================================================
        # RÉ‰GION: En-tÉªte
        # ========================================================================
        if not self._compact:
            titre = QLabel("Coordonnées cartésiennes")
            titre.setStyleSheet("font-size: 14px; font-weight: bold;")
            layout.addWidget(titre)
        
        # Convention constructeur
        self.convention_combo = QComboBox()
        self.convention_combo.addItems(list(self.CONVENTIONS.keys()))
        self.convention_combo.setCurrentText(self.current_convention)
        self.convention_combo.currentTextChanged.connect(self._on_convention_changed)
        self.convention_combo.setEnabled(False)
        self.reference_label = QLabel("Référentiel :")
        self.reference_frame_combo = QComboBox()
        self.reference_frame_combo.addItem("Robot", ReferenceFrame.BASE.value)
        self.reference_frame_combo.addItem("World", ReferenceFrame.WORLD.value)
        self.reference_frame_combo.addItem("Tool", ReferenceFrame.TOOL.value)
        self.reference_frame_combo.currentIndexChanged.connect(self._on_reference_frame_changed)
        self.convention_description = QLabel(self.CONVENTIONS[self.current_convention]["description"])
        self.convention_description.setStyleSheet("font-size: 10px; font-style: italic; color: gray;")
        convention_layout = QHBoxLayout()
        if not self._compact:
            convention_label = QLabel("Convention:")
            convention_layout.addWidget(convention_label)
            convention_layout.addWidget(self.convention_combo)
        if not self._compact:
            convention_layout.addWidget(self.reference_label)
            convention_layout.addWidget(self.reference_frame_combo)
        convention_layout.addStretch()
        layout.addLayout(convention_layout)
        if not self._compact:
            layout.addWidget(self.convention_description)
        
        # ========================================================================
        # RÉ‰GION: Sliders et spinboxes pour les 6 coordonnées cartésiennes
        # ========================================================================
        spinbox_width: int | None = None
        
        for i in range(6):
            row_layout = QHBoxLayout()
            if self._compact:
                row_layout.setContentsMargins(0, 0, 0, 0)
                row_layout.setSpacing(6)
            
            # Label
            label_text = self.CONVENTIONS[self.current_convention]["labels"][i]
            label = QLabel(label_text)
            label.setMinimumWidth(62 if self._compact else 80)
            if self._compact:
                label.setFixedHeight(self._COMPACT_ROW_HEIGHT)
            self.labels_cart.append(label)
            
            # Déterminer les limites
            min_val, max_val = self._axis_limits[i]
            
            # Slider (0-100 représente min-max)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, self._SLIDER_MAX)
            slider.setValue(int(self._SLIDER_MAX / 2))  # Milieu par défaut
            if self._compact:
                slider.setFixedHeight(self._COMPACT_ROW_HEIGHT)
            
            # SpinBox (valeur réelle)
            spinbox = JogSpinBox() if self._enable_jog_spin_buttons else QDoubleSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setDecimals(3)
            spinbox.setSingleStep(0.10)
            spinbox.setSuffix(" mm" if i < 3 else " °")
            spinbox.setValue(0.0)
            if self._compact:
                spinbox.setFixedHeight(self._COMPACT_ROW_HEIGHT)
            
            # Assurer une largeur uniforme pour tous les spinboxes
            if spinbox_width is None or spinbox.sizeHint().width() > spinbox_width:
                spinbox_width = spinbox.sizeHint().width()
            spinbox.setFixedWidth(spinbox_width)
            if isinstance(spinbox, JogSpinBox):
                spinbox.jog_button_pressed.connect(
                    lambda direction, idx=i: self.spinbox_jog_pressed.emit(idx, direction)
                )
                spinbox.jog_button_released.connect(
                    lambda direction, idx=i: self.spinbox_jog_released.emit(idx, direction)
                )
            
            # Connexions
            slider.valueChanged.connect(lambda value, idx=i: self._on_slider_changed(idx, value))
            spinbox.valueChanged.connect(lambda value, idx=i: self._on_spinbox_changed(idx, value))
            
            row_layout.addWidget(label)
            row_layout.addWidget(slider)
            row_layout.addWidget(spinbox)
            layout.addLayout(row_layout)
            
            self.sliders_cart.append(slider)
            self.spinboxes_cart.append(spinbox)
    
    # ============================================================================
    # RÉ‰GION: Méthodes privées - Conversion slider <-> valeur
    # ============================================================================
    
    def _slider_to_value(self, index: int, slider_pos: int) -> float:
        """Convertit une position de slider (0-100) en valeur réelle"""
        min_val, max_val = self._axis_limits[index]
        return min_val + (slider_pos / self._SLIDER_MAX) * (max_val - min_val)
    
    def _value_to_slider(self, index: int, value: float) -> int:
        """Convertit une valeur réelle en position de slider (0-100)"""
        min_val, max_val = self._axis_limits[index]
        if max_val == min_val:
            return int(self._SLIDER_MAX / 2)
        ratio = (value - min_val) / (max_val - min_val)
        return int(round(ratio * self._SLIDER_MAX))
    
    def _on_slider_changed(self, index: int, slider_pos: int) -> None:
        """Callback quand le slider change"""
        # Calculer la valeur réelle depuis le slider
        value = self._slider_to_value(index, slider_pos)
        
        # Mettre à jour la valeur interne
        self._set_internal_cartesian_value(index, value)
        
        # Mettre à jour le spinbox sans déclencher son signal
        self.spinboxes_cart[index].blockSignals(True)
        self.spinboxes_cart[index].setValue(value)
        self.spinboxes_cart[index].blockSignals(False)
        
        # É‰mettre le signal de changement
        self.cartesian_value_changed.emit(index, value)
    
    def _on_spinbox_changed(self, index: int, value: float) -> None:
        """Callback quand le spinbox change"""
        # Mettre à jour la valeur interne
        self._set_internal_cartesian_value(index, value)
        
        # Mettre à jour le slider sans déclencher son signal
        slider_pos = self._value_to_slider(index, value)
        self.sliders_cart[index].blockSignals(True)
        self.sliders_cart[index].setValue(slider_pos)
        self.sliders_cart[index].blockSignals(False)
        
        # É‰mettre le signal de changement
        self.cartesian_value_changed.emit(index, value)
    
    def _on_convention_changed(self, convention_name: str) -> None:
        """Gère le changement de convention constructeur"""
        self.current_convention = convention_name
        
        # Mise à jour des labels
        labels = self.CONVENTIONS[convention_name]["labels"]
        for i, label in enumerate(self.labels_cart):
            label.setText(labels[i])
        
        # Mise à jour de la description
        self.convention_description.setText(self.CONVENTIONS[convention_name]["description"])
        
        # É‰mettre le signal
        self.convention_changed.emit(convention_name)

    def _on_reference_frame_changed(self, _index: int) -> None:
        raw = self.reference_frame_combo.currentData()
        self.current_reference_frame = ReferenceFrame.from_value(raw).value
        self._apply_reference_frame_interaction_mode()
        self.reference_frame_changed.emit(self.current_reference_frame)

    def _set_internal_cartesian_value(self, index: int, value: float) -> None:
        setattr(self._cartesian_values, self._POSE_AXIS_NAMES[index], float(value))

    def _get_internal_cartesian_value(self, index: int) -> float:
        return float(getattr(self._cartesian_values, self._POSE_AXIS_NAMES[index]))
    
    # ============================================================================
    # RÉ‰GION: Méthodes publiques
    # ============================================================================
    
    def set_cartesian_value(self, index: int, value: float) -> None:
        """Définit la valeur d'une coordonnée cartésienne (mise à jour externe)"""
        if not (0 <= index < 6):
            return
        
        # Clamper la valeur dans les limites
        min_val, max_val = self._axis_limits[index]
        value = max(min_val, min(max_val, value))
        
        # Mettre à jour la valeur interne
        self._set_internal_cartesian_value(index, value)
        
        # Mettre à jour les widgets sans déclencher les signaux
        self.spinboxes_cart[index].blockSignals(True)
        self.sliders_cart[index].blockSignals(True)
        
        self.spinboxes_cart[index].setValue(value)
        self.sliders_cart[index].setValue(self._value_to_slider(index, value))
        
        self.spinboxes_cart[index].blockSignals(False)
        self.sliders_cart[index].blockSignals(False)
    
    def set_all_cartesian(self, values: Pose6 | List[float]) -> None:
        """Définit toutes les valeurs cartésiennes"""
        if isinstance(values, Pose6):
            pose = values
        else:
            raw_values = [float(value) for value in list(values)[:6]]
            while len(raw_values) < 6:
                raw_values.append(0.0)
            pose = Pose6(*raw_values)
        for i, val in enumerate(pose.to_list()):
            self.set_cartesian_value(i, float(val))
    
    def get_cartesian_value(self, index: int) -> float:
        """Récupère la valeur réelle d'une coordonnée"""
        if 0 <= index < 6:
            return self._get_internal_cartesian_value(index)
        return 0.0

    def get_cartesian_values(self) -> Pose6:
        """Retourne les valeurs actuelles des coordonnées cartésiennes."""
        return self._cartesian_values.copy()
    
    def get_current_convention(self) -> str:
        """Retourne la convention actuellement sélectionnée"""
        return self.current_convention

    def get_reference_frame(self) -> str:
        return self.current_reference_frame

    def set_reference_frame(self, reference_frame: str, emit_signal: bool = False) -> None:
        normalized = ReferenceFrame.from_value(reference_frame)
        index = self.reference_frame_combo.findData(normalized.value)
        if index < 0:
            return
        self.reference_frame_combo.blockSignals(True)
        self.reference_frame_combo.setCurrentIndex(index)
        self.reference_frame_combo.blockSignals(False)
        self.current_reference_frame = normalized.value
        self._apply_reference_frame_interaction_mode()
        if emit_signal:
            self.reference_frame_changed.emit(self.current_reference_frame)
    
    def update_axis_limits(self, limits: List[Tuple[float, float]]) -> None:
        """Met à jour les limites des axes
        
        Args:
            limits: Liste de 6 tuples (min, max) pour chaque axe
        """
        for i in range(min(6, len(limits))):
            min_val, max_val = limits[i]
            self._axis_limits[i] = (min_val, max_val)
            
            # Mettre à jour la range du spinbox
            self.spinboxes_cart[i].setRange(min_val, max_val)
            
            # Clamper la valeur actuelle si nécessaire
            current_value = self._get_internal_cartesian_value(i)
            if current_value < min_val or current_value > max_val:
                clamped_value = max(min_val, min(max_val, current_value))
                self.set_cartesian_value(i, clamped_value)
            else:
                # Juste mettre à jour le slider (les limites ont changé)
                self.sliders_cart[i].blockSignals(True)
                self.sliders_cart[i].setValue(self._value_to_slider(i, current_value))
                self.sliders_cart[i].blockSignals(False)
    
    def get_axis_limits(self) -> List[Tuple[float, float]]:
        """Récupère les limites des axes"""
        return self._axis_limits.copy()

    def set_spinbox_single_step(self, step: float) -> None:
        normalized_step = max(0.001, float(step))
        for spinbox in self.spinboxes_cart:
            spinbox.setSingleStep(normalized_step)

    def set_jog_increment(self, value: float) -> None:
        normalized_value = max(0.001, float(value))
        for index, spinbox in enumerate(self.spinboxes_cart):
            spinbox.setSingleStep(normalized_value if index < 3 else normalized_value * 0.1)

    def _apply_reference_frame_interaction_mode(self) -> None:
        jog_only_mode = self.current_reference_frame == ReferenceFrame.TOOL.value
        for slider in self.sliders_cart:
            slider.setEnabled(not jog_only_mode)
        for spinbox in self.spinboxes_cart:
            spinbox.setReadOnly(jog_only_mode)
            if isinstance(spinbox, JogSpinBox):
                spinbox.set_allow_jog_while_read_only(jog_only_mode)


