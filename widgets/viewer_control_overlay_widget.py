from PyQt6.QtWidgets import (
    QButtonGroup,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QRadioButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)
from PyQt6.QtCore import pyqtSignal

from widgets.cartesian_control_view.cartesian_control_widget import CartesianControlWidget
from widgets.joint_control_view.joints_control_widget import JointsControlWidget


class ViewerControlOverlayWidget(QWidget):
    jog_delta_changed = pyqtSignal(float)
    _CURRENT_CONFIG_COLOR = "orange"

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("viewerControlOverlay")

        self.mode_articular_radio = QRadioButton("Articulaire")
        self.mode_cartesian_radio = QRadioButton("Cartésien")
        self.mode_group = QButtonGroup(self)
        self.mode_group.addButton(self.mode_articular_radio)
        self.mode_group.addButton(self.mode_cartesian_radio)
        self.mode_selector_frame = QWidget(self)
        self.jog_delta_label = QLabel("Incrément :")
        self.jog_delta_spinbox = QDoubleSpinBox(self)

        self.joints_widget = JointsControlWidget(compact=True, enable_jog_spin_buttons=True)
        self.cartesian_widget = CartesianControlWidget(compact=True, enable_jog_spin_buttons=True)
        self.configuration_label = QLabel()
        self.reference_label = self.cartesian_widget.reference_label
        self.reference_frame_combo = self.cartesian_widget.reference_frame_combo

        self.mode_stack = QStackedWidget()
        self.mode_stack.addWidget(self.joints_widget)
        self.mode_stack.addWidget(self.cartesian_widget)

        self._setup_ui()
        self.mode_articular_radio.setChecked(True)
        self.mode_stack.setCurrentWidget(self.joints_widget)

    def _setup_ui(self) -> None:
        self.setStyleSheet("""
            QWidget#viewerControlOverlay {
                background-color: rgba(25, 25, 28, 130);
                border: 1px solid rgba(255, 255, 255, 35);
                border-radius: 6px;
            }
            QWidget#viewerControlOverlay QLabel,
            QWidget#viewerControlOverlay QRadioButton {
                color: lightgray;
            }
            QWidget#viewerControlOverlay QWidget#viewerModeSelector {
                background-color: rgba(255, 255, 255, 10);
                border: 1px solid rgba(255, 255, 255, 32);
                border-radius: 6px;
            }
            QWidget#viewerControlOverlay QComboBox,
            QWidget#viewerControlOverlay QPushButton {
                color: white;
                background-color: rgba(255, 255, 255, 18);
                border: 1px solid rgba(255, 255, 255, 28);
                border-radius: 6px;
                padding: 4px 6px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 12)
        layout.setSpacing(8)

        mode_layout = QHBoxLayout()
        mode_layout.setContentsMargins(0, 0, 0, 0)
        mode_layout.setSpacing(12)

        self.mode_selector_frame.setObjectName("viewerModeSelector")
        selector_layout = QHBoxLayout(self.mode_selector_frame)
        selector_layout.setContentsMargins(10, 6, 10, 6)
        selector_layout.setSpacing(12)
        selector_layout.addWidget(self.mode_articular_radio)
        selector_layout.addWidget(self.mode_cartesian_radio)
        self.jog_delta_spinbox.setDecimals(2)
        self.jog_delta_spinbox.setRange(1, 10.0)
        self.jog_delta_spinbox.setSingleStep(1)
        self.jog_delta_spinbox.setValue(5.0)
        self.jog_delta_spinbox.setFixedWidth(88)
        self.jog_delta_spinbox.setToolTip("Unités : mm/s pour X,Y,Z et 1/10°/s pour les angles.")
        self.jog_delta_spinbox.valueChanged.connect(self._on_jog_delta_changed)
        mode_layout.addWidget(self.mode_selector_frame, 0)
        mode_layout.addWidget(self.jog_delta_label)
        mode_layout.addWidget(self.jog_delta_spinbox)
        if self.reference_label is not None:
            mode_layout.addWidget(self.reference_label)
            self.reference_label.hide()
        if self.reference_frame_combo is not None:
            mode_layout.addWidget(self.reference_frame_combo)
            self.reference_frame_combo.hide()
        mode_layout.addStretch()
        mode_layout.addWidget(self.configuration_label, 0)
        layout.addLayout(mode_layout)
        layout.addWidget(self.mode_stack)

        self.mode_articular_radio.toggled.connect(self._on_mode_changed)
        self.mode_cartesian_radio.toggled.connect(self._on_mode_changed)
        self.joints_widget.configuration_changed.connect(self._on_configuration_changed)
        self._on_configuration_changed("FUN")
        self._on_jog_delta_changed(self.jog_delta_spinbox.value())

    def _on_mode_changed(self) -> None:
        show_cartesian_controls = self.mode_cartesian_radio.isChecked()
        if self.reference_label is not None:
            self.reference_label.setVisible(show_cartesian_controls)
        if self.reference_frame_combo is not None:
            self.reference_frame_combo.setVisible(show_cartesian_controls)
        if self.mode_cartesian_radio.isChecked():
            self.mode_stack.setCurrentWidget(self.cartesian_widget)
            return
        self.mode_stack.setCurrentWidget(self.joints_widget)

    def _on_configuration_changed(self, config_name: str) -> None:
        self.configuration_label.setText(
            f'Configuration courante : <span style="color: {self._CURRENT_CONFIG_COLOR};">{config_name}</span>'
        )

    def get_joints_widget(self) -> JointsControlWidget:
        return self.joints_widget

    def get_cartesian_widget(self) -> CartesianControlWidget:
        return self.cartesian_widget

    def get_jog_delta(self) -> float:
        return float(self.jog_delta_spinbox.value())

    def set_jog_delta(self, value: float) -> None:
        normalized_value = max(0.01, float(value))
        self.jog_delta_spinbox.blockSignals(True)
        self.jog_delta_spinbox.setValue(normalized_value)
        self.jog_delta_spinbox.blockSignals(False)
        self._on_jog_delta_changed(normalized_value)

    def _on_jog_delta_changed(self, value: float) -> None:
        normalized_value = max(0.01, float(value))
        self.joints_widget.set_jog_increment(normalized_value)
        self.cartesian_widget.set_jog_increment(normalized_value)
        self.jog_delta_changed.emit(normalized_value)
