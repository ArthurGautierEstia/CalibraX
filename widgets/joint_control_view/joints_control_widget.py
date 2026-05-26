from typing import List
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QSlider, QDoubleSpinBox, QSizePolicy, QFrame
)
from PyQt6.QtCore import Qt, pyqtSignal, QEvent
from PyQt6.QtGui import QColor, QPalette
from utils.mgi import MgiConfigKey
from widgets.jog_spin_box import JogSpinBox


class JointsControlWidget(QWidget):
    """Widget pour le contrôle des coordonnées articulaires"""

    _COMPACT_ROW_HEIGHT = 28

    # Signaux robot
    joint_value_changed = pyqtSignal(int, float)  # index, value
    configuration_changed = pyqtSignal(str)
    home_position_requested = pyqtSignal()
    position_zero_requested = pyqtSignal()
    position_calibration_requested = pyqtSignal()
    spinbox_jog_pressed = pyqtSignal(int, int)
    spinbox_jog_released = pyqtSignal(int, int)

    # Signaux axes externes : (axis_id, joint_index, direction +1/-1)
    external_joint_value_changed = pyqtSignal(str, int, float)
    spinbox_ext_jog_pressed = pyqtSignal(str, int, int)
    spinbox_ext_jog_released = pyqtSignal(str, int, int)

    def __init__(
        self,
        parent: QWidget = None,
        compact: bool = False,
        show_configuration_in_compact: bool = False,
        enable_jog_spin_buttons: bool = False,
    ) -> None:
        super().__init__(parent)

        # Données internes robot
        self._joint_values: List[float] = [0.0] * 6
        self._axis_limits: List[tuple[float, float]] = [(-180.0, 180.0) for _ in range(6)]
        self._current_axis_config: MgiConfigKey = MgiConfigKey.FUN
        self._compact = bool(compact)
        self._show_configuration_in_compact = bool(show_configuration_in_compact)
        self._enable_jog_spin_buttons = bool(enable_jog_spin_buttons)

        # Données internes axes externes
        self._ext_sliders: List[QSlider] = []
        self._ext_spinboxes: List[QDoubleSpinBox] = []
        self._ext_axes_info: List[tuple[str, int]] = []  # (axis_id, joint_index)
        self._ext_limits: List[tuple[float, float]] = []
        self._ext_container: QWidget | None = None
        self._main_layout: QVBoxLayout | None = None
        self._ext_section_start: int = 0
        self._spinbox_width: int = 120

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
            titre = QLabel("Coordonnées articulaires")
            titre.setStyleSheet("font-size: 14px; font-weight: bold;")
            layout.addWidget(titre)

        # Config
        if not self._compact or self._show_configuration_in_compact:
            layout.addWidget(self.configuration_label)
        self.set_configuration(self._current_axis_config)

        # Sliders et spinboxes pour les 6 joints
        translation_reference_spinbox = QDoubleSpinBox()
        translation_reference_spinbox.setRange(-2000.0, 2000.0)
        translation_reference_spinbox.setDecimals(3)
        translation_reference_spinbox.setSingleStep(0.10)
        translation_reference_spinbox.setSuffix(" mm")
        translation_reference_spinbox.setValue(0.0)
        translation_reference_spinbox.setKeyboardTracking(False)

        rotation_reference_spinbox = QDoubleSpinBox()
        rotation_reference_spinbox.setRange(-180.0, 180.0)
        rotation_reference_spinbox.setDecimals(3)
        rotation_reference_spinbox.setSingleStep(0.10)
        rotation_reference_spinbox.setSuffix(" °")
        rotation_reference_spinbox.setValue(0.0)
        rotation_reference_spinbox.setKeyboardTracking(False)

        spinbox_width = max(
            translation_reference_spinbox.sizeHint().width(),
            rotation_reference_spinbox.sizeHint().width(),
        )
        self._spinbox_width = spinbox_width

        for i in range(6):
            row_layout = QHBoxLayout()
            if self._compact:
                row_layout.setContentsMargins(0, 0, 0, 0)
                row_layout.setSpacing(6)
            label = QLabel(f"q{i+1}")
            label.setMinimumWidth(58 if self._compact else 72)
            if self._compact:
                label.setFixedHeight(self._COMPACT_ROW_HEIGHT)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, self._SLIDER_MAX)
            slider.setValue(int(self._SLIDER_MAX / 2))
            if self._compact:
                slider.setFixedHeight(self._COMPACT_ROW_HEIGHT)

            spinbox = JogSpinBox() if self._enable_jog_spin_buttons else QDoubleSpinBox()
            spinbox.setRange(self._axis_limits[i][0], self._axis_limits[i][1])
            spinbox.setDecimals(3)
            spinbox.setSingleStep(0.10)
            spinbox.setSuffix(" °")
            spinbox.setValue(0.0)
            spinbox.setKeyboardTracking(False)
            if self._compact:
                spinbox.setFixedHeight(self._COMPACT_ROW_HEIGHT)
            spinbox.setFixedWidth(spinbox_width)
            if isinstance(spinbox, JogSpinBox):
                spinbox.jog_button_pressed.connect(
                    lambda direction, idx=i: self.spinbox_jog_pressed.emit(idx, direction)
                )
                spinbox.jog_button_released.connect(
                    lambda direction, idx=i: self.spinbox_jog_released.emit(idx, direction)
                )

            slider.valueChanged.connect(lambda value, idx=i: self._on_slider_changed(idx, value))
            spinbox.valueChanged.connect(lambda value, idx=i: self._on_spinbox_changed(idx, value))

            row_layout.addWidget(label)
            row_layout.addWidget(slider)
            row_layout.addWidget(spinbox)
            layout.addLayout(row_layout)

            self.sliders_q.append(slider)
            self.spinboxes_q.append(spinbox)

        # Point d'insertion pour les axes externes (avant les boutons)
        self._ext_section_start = layout.count()
        self._main_layout = layout

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
        """Convertit une position de slider (0-100) en valeur réelle"""
        min_val, max_val = self._axis_limits[index]
        return min_val + (slider_pos / float(self._SLIDER_MAX)) * (max_val - min_val)
    
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
        self._joint_values[index] = value
        
        # Mettre à jour le spinbox sans déclencher son signal
        self.spinboxes_q[index].blockSignals(True)
        self.spinboxes_q[index].setValue(value)
        self.spinboxes_q[index].blockSignals(False)
        
        # É‰mettre le signal de changement
        self.joint_value_changed.emit(index, value)
    
    def _on_spinbox_changed(self, index: int, value: float) -> None:
        """Callback quand le spinbox change"""
        # Mettre à jour la valeur interne
        self._joint_values[index] = value
        
        # Mettre à jour le slider sans déclencher son signal
        slider_pos = self._value_to_slider(index, value)
        self.sliders_q[index].blockSignals(True)
        self.sliders_q[index].setValue(slider_pos)
        self.sliders_q[index].blockSignals(False)
        
        # É‰mettre le signal de changement
        self.joint_value_changed.emit(index, value)
    
    def set_joint_value(self, index: int, value: float) -> None:
        """Définit la valeur d'un joint (mise à jour externe)"""
        if not (0 <= index < 6):
            return
        
        # Clamper la valeur dans les limites
        min_val, max_val = self._axis_limits[index]
        value = max(min_val, min(max_val, value))
        
        # Mettre à jour la valeur interne
        self._joint_values[index] = value
        
        # Mettre à jour les widgets sans déclencher les signaux
        self.spinboxes_q[index].blockSignals(True)
        self.sliders_q[index].blockSignals(True)
        
        self.spinboxes_q[index].setValue(value)
        self.sliders_q[index].setValue(self._value_to_slider(index, value))
        
        self.spinboxes_q[index].blockSignals(False)
        self.sliders_q[index].blockSignals(False)
    
    def set_all_joints(self, values: List[float]) -> None:
        """Définit toutes les valeurs de joints"""
        for i, val in enumerate(values[:6]):
            self.set_joint_value(i, val)
    
    def get_joint_value(self, index: int) -> float:
        """Récupère la valeur réelle d'un joint"""
        if 0 <= index < 6:
            return self._joint_values[index]
        return 0.0
    
    def get_all_joints(self) -> List[float]:
        """Récupère toutes les valeurs de joints"""
        return self._joint_values.copy()
    
    def update_axis_limits(self, limits: List[tuple[float, float]]) -> None:
        """Met à jour les limites des axes"""
        for i in range(min(6, len(limits))):
            min_val, max_val = limits[i]
            self._axis_limits[i] = (min_val, max_val)
            
            # Mettre à jour la range du spinbox
            self.spinboxes_q[i].setRange(min_val, max_val)
            
            # Clamper la valeur actuelle si nécessaire
            current_value = self._joint_values[i]
            if current_value < min_val or current_value > max_val:
                clamped_value = max(min_val, min(max_val, current_value))
                self.set_joint_value(i, clamped_value)
            else:
                # Juste mettre à jour le slider (les limites ont changé)
                self.sliders_q[i].blockSignals(True)
                self.sliders_q[i].setValue(self._value_to_slider(i, current_value))
                self.sliders_q[i].blockSignals(False)
    
    def get_axis_limits(self) -> List[tuple[float, float]]:
        """Récupère les limites des axes"""
        return self._axis_limits.copy()
    
    def set_configuration(self, config: MgiConfigKey) -> None:
        """Met à jour le texte de configuration"""
        self._current_axis_config = config
        accent_hex = self.palette().color(QPalette.ColorRole.Highlight).name()
        self.configuration_label.setText(
            f'Configuration courante : <span style="color: {accent_hex};">{config.name}</span>'
        )
        self.configuration_changed.emit(config.name)

    def changeEvent(self, event) -> None:
        super().changeEvent(event)
        if event.type() == QEvent.Type.PaletteChange:
            self.set_configuration(self._current_axis_config)

    def apply_text_color(self, text_color: QColor) -> None:
        text_hex = text_color.name(QColor.NameFormat.HexRgb)
        control_style = f"color: {text_hex};"
        for spinbox in self.spinboxes_q:
            spinbox.setStyleSheet(control_style)
        for spinbox in self._ext_spinboxes:
            spinbox.setStyleSheet(control_style)

    def set_spinbox_single_step(self, step: float) -> None:
        normalized_step = max(0.001, float(step))
        for spinbox in self.spinboxes_q:
            spinbox.setSingleStep(normalized_step)
        for spinbox in self._ext_spinboxes:
            spinbox.setSingleStep(normalized_step)

    def set_spinbox_keyboard_tracking(self, enabled: bool) -> None:
        for spinbox in self.spinboxes_q:
            spinbox.setKeyboardTracking(bool(enabled))

    def set_jog_increment(self, value: float) -> None:
        self.set_spinbox_single_step(max(0.001, float(value)) * 0.1)

    # ------------------------------------------------------------------
    # Axes externes dynamiques
    # ------------------------------------------------------------------

    def set_external_axes(self, axes_info: list[tuple[str, int, float, float, float, str]]) -> None:
        """Reconstruit les lignes slider/spinbox pour les axes externes.

        axes_info: liste de (axis_id, joint_index, q_min, q_max, current_value, unit)
        """
        if self._main_layout is None:
            return

        # Supprimer l'ancien container axes externes
        if self._ext_container is not None:
            self._ext_container.setParent(None)
            self._ext_container.deleteLater()
            self._ext_container = None

        self._ext_sliders.clear()
        self._ext_spinboxes.clear()
        self._ext_axes_info.clear()
        self._ext_limits.clear()

        if not axes_info:
            return

        container = QWidget()
        container_layout = QVBoxLayout(container)
        container_layout.setContentsMargins(0, 0, 0, 0)
        container_layout.setSpacing(4 if self._compact else 6)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFrameShadow(QFrame.Shadow.Sunken)
        container_layout.addWidget(sep)

        for ei, (axis_id, joint_index, q_min, q_max, current_value, unit) in enumerate(axes_info):
            row_layout = QHBoxLayout()
            if self._compact:
                row_layout.setContentsMargins(0, 0, 0, 0)
                row_layout.setSpacing(6)

            label = QLabel(f"e{ei + 1}")
            label.setMinimumWidth(58 if self._compact else 72)
            if self._compact:
                label.setFixedHeight(self._COMPACT_ROW_HEIGHT)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, self._SLIDER_MAX)
            if q_max > q_min:
                slider_pos = int(round(((current_value - q_min) / (q_max - q_min)) * self._SLIDER_MAX))
            else:
                slider_pos = self._SLIDER_MAX // 2
            slider.setValue(max(0, min(self._SLIDER_MAX, slider_pos)))
            if self._compact:
                slider.setFixedHeight(self._COMPACT_ROW_HEIGHT)

            ext_idx = len(self._ext_sliders)

            spinbox = JogSpinBox() if self._enable_jog_spin_buttons else QDoubleSpinBox()
            spinbox.setRange(q_min, q_max)
            spinbox.setDecimals(3)
            spinbox.setSingleStep(0.10)
            spinbox.setSuffix(f" {unit}")
            spinbox.setValue(current_value)
            spinbox.setKeyboardTracking(False)
            if self._compact:
                spinbox.setFixedHeight(self._COMPACT_ROW_HEIGHT)
            spinbox.setFixedWidth(self._spinbox_width)

            if isinstance(spinbox, JogSpinBox):
                spinbox.jog_button_pressed.connect(
                    lambda direction, aid=axis_id, ji=joint_index:
                        self.spinbox_ext_jog_pressed.emit(aid, ji, direction)
                )
                spinbox.jog_button_released.connect(
                    lambda direction, aid=axis_id, ji=joint_index:
                        self.spinbox_ext_jog_released.emit(aid, ji, direction)
                )

            slider.valueChanged.connect(
                lambda val, idx=ext_idx, qmin=q_min, qmax=q_max, aid=axis_id, ji=joint_index:
                    self._on_ext_slider_changed(idx, qmin, qmax, aid, ji, val)
            )
            spinbox.valueChanged.connect(
                lambda val, idx=ext_idx, qmin=q_min, qmax=q_max, aid=axis_id, ji=joint_index:
                    self._on_ext_spinbox_changed(idx, qmin, qmax, aid, ji, val)
            )

            row_layout.addWidget(label)
            row_layout.addWidget(slider)
            row_layout.addWidget(spinbox)
            container_layout.addLayout(row_layout)

            self._ext_sliders.append(slider)
            self._ext_spinboxes.append(spinbox)
            self._ext_axes_info.append((axis_id, joint_index))
            self._ext_limits.append((q_min, q_max))

        self._main_layout.insertWidget(self._ext_section_start, container)
        self._ext_container = container

    def set_external_joint_value(self, axis_id: str, joint_index: int, value: float) -> None:
        """Met à jour la valeur d'un joint d'axe externe sans émettre de signal."""
        for i, (aid, ji) in enumerate(self._ext_axes_info):
            if aid == axis_id and ji == joint_index:
                q_min, q_max = self._ext_limits[i]
                value = max(q_min, min(q_max, value))
                self._ext_spinboxes[i].blockSignals(True)
                self._ext_sliders[i].blockSignals(True)
                self._ext_spinboxes[i].setValue(value)
                if q_max > q_min:
                    slider_pos = int(round(((value - q_min) / (q_max - q_min)) * self._SLIDER_MAX))
                else:
                    slider_pos = self._SLIDER_MAX // 2
                self._ext_sliders[i].setValue(max(0, min(self._SLIDER_MAX, slider_pos)))
                self._ext_spinboxes[i].blockSignals(False)
                self._ext_sliders[i].blockSignals(False)
                return

    def _on_ext_slider_changed(self, idx: int, q_min: float, q_max: float, axis_id: str, joint_index: int, slider_pos: int) -> None:
        value = q_min + (slider_pos / self._SLIDER_MAX) * (q_max - q_min) if q_max > q_min else q_min
        self._ext_spinboxes[idx].blockSignals(True)
        self._ext_spinboxes[idx].setValue(value)
        self._ext_spinboxes[idx].blockSignals(False)
        self.external_joint_value_changed.emit(axis_id, joint_index, value)

    def _on_ext_spinbox_changed(self, idx: int, q_min: float, q_max: float, axis_id: str, joint_index: int, value: float) -> None:
        if q_max > q_min:
            slider_pos = int(round(((value - q_min) / (q_max - q_min)) * self._SLIDER_MAX))
        else:
            slider_pos = self._SLIDER_MAX // 2
        self._ext_sliders[idx].blockSignals(True)
        self._ext_sliders[idx].setValue(max(0, min(self._SLIDER_MAX, slider_pos)))
        self._ext_sliders[idx].blockSignals(False)
        self.external_joint_value_changed.emit(axis_id, joint_index, value)


