from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QDoubleSpinBox, QHBoxLayout, QGroupBox
)
from PyQt6.QtCore import pyqtSignal

from utils.mgi import RobotTool


class ToolWidget(QWidget):
    """
    Widget pour la configuration de l'outil du robot
    """

    # Émis à chaque modification
    tool_changed = pyqtSignal(RobotTool)

    def __init__(self, tool: RobotTool = None, parent=None):
        super().__init__(parent)

        if tool is None:
            tool = RobotTool()

        self._tool = tool
        self._spin_boxes: dict[str, QDoubleSpinBox] = {}

        self._init_ui()

    # ---------------------------------------------------------
    # UI
    # ---------------------------------------------------------

    def _init_ui(self):
        layout = QVBoxLayout(self)

        # Groupe pour l'outil
        group = QGroupBox("Configuration de l'outil")
        group_layout = QVBoxLayout(group)

        # Description
        description = QLabel("Paramètres de transformation de l'outil\n"
                           "par rapport au flange du robot")
        description.setWordWrap(True)
        group_layout.addWidget(description)

        inputs_layout = QHBoxLayout()

        # Paramètres de translation
        trans_layout = QVBoxLayout()
        trans_layout.addWidget(QLabel("Translation (mm):"))

        for axis in ['x', 'y', 'z']:
            axis_layout = QHBoxLayout()
            axis_layout.addWidget(QLabel(f"{axis.upper()}:"))

            spin_box = QDoubleSpinBox()
            spin_box.setRange(-1000.0, 1000.0)
            spin_box.setValue(getattr(self._tool, axis))
            spin_box.setSingleStep(0.1)
            spin_box.setDecimals(2)
            spin_box.valueChanged.connect(
                lambda value, ax=axis: self._on_param_changed(ax, value)
            )

            self._spin_boxes[axis] = spin_box
            axis_layout.addWidget(spin_box)
            trans_layout.addLayout(axis_layout)

        inputs_layout.addLayout(trans_layout)

        # Paramètres de rotation
        rot_layout = QVBoxLayout()
        rot_layout.addWidget(QLabel("Rotation (°):"))

        for axis in ['a', 'b', 'c']:
            axis_layout = QHBoxLayout()
            axis_layout.addWidget(QLabel(f"{axis.upper()}:"))

            spin_box = QDoubleSpinBox()
            spin_box.setRange(-180.0, 180.0)
            spin_box.setValue(getattr(self._tool, axis))
            spin_box.setSingleStep(0.1)
            spin_box.setDecimals(2)
            spin_box.valueChanged.connect(
                lambda value, ax=axis: self._on_param_changed(ax, value)
            )

            self._spin_boxes[axis] = spin_box
            axis_layout.addWidget(spin_box)
            rot_layout.addLayout(axis_layout)

        inputs_layout.addLayout(rot_layout)

        # Bouton pour remettre à zéro
        from PyQt6.QtWidgets import QPushButton
        reset_btn = QPushButton("Remettre à zéro")
        reset_btn.clicked.connect(self._reset_to_identity)
        inputs_layout.addWidget(reset_btn)

        group_layout.addLayout(inputs_layout)

        layout.addWidget(group)
        layout.addStretch()

    # ---------------------------------------------------------
    # Internals
    # ---------------------------------------------------------

    def _on_param_changed(self, param: str, value: float):
        setattr(self._tool, param, value)
        self.tool_changed.emit(self._tool)

    def _reset_to_identity(self):
        """Remet l'outil à l'identité (pas de transformation)"""
        self._tool = RobotTool()
        for param in ['x', 'y', 'z', 'a', 'b', 'c']:
            setattr(self._tool, param, 0.0)
            self._spin_boxes[param].blockSignals(True)
            self._spin_boxes[param].setValue(0.0)
            self._spin_boxes[param].blockSignals(False)
        self.tool_changed.emit(self._tool)

    # ---------------------------------------------------------
    # Public API
    # ---------------------------------------------------------

    def set_tool(self, tool: RobotTool):
        """Met à jour l'outil affiché"""
        self._tool = tool
        for param in ['x', 'y', 'z', 'a', 'b', 'c']:
            value = getattr(tool, param)
            self._spin_boxes[param].blockSignals(True)
            self._spin_boxes[param].setValue(value)
            self._spin_boxes[param].blockSignals(False)

    def get_tool(self) -> RobotTool:
        """Retourne l'outil actuel"""
        return self._tool
