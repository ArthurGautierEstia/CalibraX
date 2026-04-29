from PyQt6.QtWidgets import (
    QButtonGroup,
    QHBoxLayout,
    QLabel,
    QRadioButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from widgets.cartesian_control_view.cartesian_control_widget import CartesianControlWidget
from widgets.joint_control_view.joints_control_widget import JointsControlWidget


class ViewerControlOverlayWidget(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("viewerControlOverlay")

        self.mode_articular_radio = QRadioButton("Articulaire")
        self.mode_cartesian_radio = QRadioButton("Cartésien")
        self.mode_group = QButtonGroup(self)
        self.mode_group.addButton(self.mode_articular_radio)
        self.mode_group.addButton(self.mode_cartesian_radio)

        self.joints_widget = JointsControlWidget(compact=True)
        self.cartesian_widget = CartesianControlWidget(compact=True)
        self.configuration_label = QLabel("Configuration courante : FUN")

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
            QWidget#viewerControlOverlay QRadioButton::indicator {
                width: 14px;
                height: 14px;
            }
            QWidget#viewerControlOverlay QRadioButton::indicator:unchecked {
                border: 1px solid rgba(255, 255, 255, 80);
                border-radius: 7px;
                background-color: rgba(255, 255, 255, 18);
            }
            QWidget#viewerControlOverlay QRadioButton::indicator:checked {
                border: 1px solid #ff8c00;
                border-radius: 7px;
                background-color: #ff8c00;
            }
            QWidget#viewerControlOverlay QSlider::groove:horizontal {
                height: 6px;
                border-radius: 3px;
                background-color: rgba(255, 255, 255, 36);
            }
            QWidget#viewerControlOverlay QSlider::handle:horizontal {
                width: 14px;
                margin: -5px 0;
                border-radius: 7px;
                background-color: #ff8c00;
            }
            QWidget#viewerControlOverlay QDoubleSpinBox,
            QWidget#viewerControlOverlay QComboBox,
            QWidget#viewerControlOverlay QPushButton {
                color: white;
                background-color: rgba(255, 255, 255, 18);
                border: 1px solid rgba(255, 255, 255, 28);
                border-radius: 6px;
                padding: 4px 6px;
            }
            QWidget#viewerControlOverlay QDoubleSpinBox {
                padding-right: 20px;
            }
            QWidget#viewerControlOverlay QDoubleSpinBox::up-button,
            QWidget#viewerControlOverlay QDoubleSpinBox::down-button {
                width: 18px;
                subcontrol-origin: border;
                background-color: rgba(255, 255, 255, 10);
                border-left: 1px solid rgba(255, 255, 255, 24);
            }
            QWidget#viewerControlOverlay QDoubleSpinBox::up-button {
                subcontrol-position: top right;
                border-top-right-radius: 6px;
            }
            QWidget#viewerControlOverlay QDoubleSpinBox::down-button {
                subcontrol-position: bottom right;
                border-bottom-right-radius: 6px;
                border-top: 1px solid rgba(255, 255, 255, 18);
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 12)
        layout.setSpacing(8)

        mode_layout = QHBoxLayout()
        mode_layout.setContentsMargins(0, 0, 0, 0)
        mode_layout.setSpacing(12)
        mode_layout.addWidget(self.mode_articular_radio)
        mode_layout.addWidget(self.mode_cartesian_radio)
        mode_layout.addStretch()
        mode_layout.addWidget(self.configuration_label, 0)
        layout.addLayout(mode_layout)
        layout.addWidget(self.mode_stack)

        self.mode_articular_radio.toggled.connect(self._on_mode_changed)
        self.mode_cartesian_radio.toggled.connect(self._on_mode_changed)
        self.joints_widget.configuration_changed.connect(self._on_configuration_changed)

    def _on_mode_changed(self) -> None:
        if self.mode_cartesian_radio.isChecked():
            self.mode_stack.setCurrentWidget(self.cartesian_widget)
            return
        self.mode_stack.setCurrentWidget(self.joints_widget)

    def _on_configuration_changed(self, config_name: str) -> None:
        self.configuration_label.setText(f"Configuration courante : {config_name}")

    def get_joints_widget(self) -> JointsControlWidget:
        return self.joints_widget

    def get_cartesian_widget(self) -> CartesianControlWidget:
        return self.cartesian_widget
