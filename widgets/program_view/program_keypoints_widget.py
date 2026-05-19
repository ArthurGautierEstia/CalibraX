from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
    QHeaderView,
)

from models.reference_frame import ReferenceFrame
from models.trajectory_keypoint import (
    KeypointMotionMode,
    KeypointTargetType,
    TrajectoryKeypoint,
)
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel


class ProgramKeypointsWidget(QWidget):
    """Widget pour afficher et gérer les points clés d'un programme robot.
    
    Ce widget est spécifique à l'onglet Programme et permet :
    - L'affichage des points clés du programme
    - La sélection et l'édition des cibles
    - Le choix du repère cartésien d'affichage
    """

    goToRequested = pyqtSignal(int)
    keypointSelectionChanged = pyqtSignal(object)
    keypoints_changed = pyqtSignal(list)
    cartesianDisplayFrameChanged = pyqtSignal(str)
    edit_requested = pyqtSignal(int)
    toolSourceChanged = pyqtSignal(str)
    targetModeChanged = pyqtSignal(str)
    motionModeChanged = pyqtSignal(str)
    editProgramBaseRequested = pyqtSignal()

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        parent: QWidget = None,
    ) -> None:
        super().__init__(parent)
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model

        self.keypoints_table = QTableWidget(0, 10)
        self.btn_edit = QPushButton("Editer")
        self.btn_go_to = QPushButton("Aller à")
        self.cartesian_display_frame_combo = QComboBox()
        self.tool_source_combo = QComboBox()
        self.target_mode_combo = QComboBox()
        self.motion_mode_combo = QComboBox()
        self.btn_edit_program_base = QPushButton("Editer la base programme")

        self._keypoints: list[TrajectoryKeypoint] = []
        self._current_tool_source = "CURRENT"  # CURRENT or PROGRAM

        self._setup_ui()
        self._setup_connections()
        self._update_buttons_state()

    def _setup_ui(self) -> None:
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)

        layout = QVBoxLayout(self)

        title = QLabel("Points clés du programme")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(title)

        # Ligne unique avec les 4 sélecteurs : Base, Tool, Cibles, Mode
        selectors_row = QHBoxLayout()
        
        # Sélecteur Base
        selectors_row.addWidget(QLabel("Base : "))
        self.cartesian_display_frame_combo.addItem("Programme", ReferenceFrame.PROGRAM.value)
        self.cartesian_display_frame_combo.addItem("Robot", ReferenceFrame.ROBOT.value)
        selectors_row.addWidget(self.cartesian_display_frame_combo)
        selectors_row.addSpacing(12)
        
        # Sélecteur Tool
        selectors_row.addWidget(QLabel("Tool : "))
        self.tool_source_combo.addItem("Courant", "CURRENT")
        self.tool_source_combo.addItem("Programme", "PROGRAM")
        selectors_row.addWidget(self.tool_source_combo)
        selectors_row.addSpacing(12)
        
        # Sélecteur Cibles (Theorique/Compense)
        selectors_row.addWidget(QLabel("Cibles : "))
        self.target_mode_combo.addItem("Theorique", "THEORETICAL")
        self.target_mode_combo.addItem("Compense", "COMPENSATED")
        selectors_row.addWidget(self.target_mode_combo)
        selectors_row.addSpacing(12)
        
        # Sélecteur Mode (Cartesien/Articulaire)
        selectors_row.addWidget(QLabel("Mode : "))
        self.motion_mode_combo.addItem("Cartesien", "CARTESIAN")
        self.motion_mode_combo.addItem("Articulaire", "ARTICULAR")
        selectors_row.addWidget(self.motion_mode_combo)
        selectors_row.addStretch()
        
        layout.addLayout(selectors_row)

        base_actions_row = QHBoxLayout()
        base_actions_row.addWidget(self.btn_edit_program_base)
        base_actions_row.addStretch()
        layout.addLayout(base_actions_row)

        self.keypoints_table.setHorizontalHeaderLabels([
            "Cible", "Mode", "Vitesse", "J1 / X", "J2 / Y", "J3 / Z", "J4 / A", "J5 / B", "J6 / C", "Configs"
        ])

        header = self.keypoints_table.horizontalHeader()
        header.setMinimumSectionSize(60)

        for col in range(0, 9):
            header.setSectionResizeMode(col, QHeaderView.ResizeMode.ResizeToContents)

        header.setSectionResizeMode(9, QHeaderView.ResizeMode.Stretch)
        
        self.keypoints_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.keypoints_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.keypoints_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.keypoints_table.setMinimumHeight(220)
        layout.addWidget(self.keypoints_table)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self.btn_edit)
        btn_row.addWidget(self.btn_go_to)
        btn_row.addStretch()
        layout.addLayout(btn_row)

    def _setup_connections(self) -> None:
        self.btn_edit.clicked.connect(self._on_edit_clicked)
        self.btn_go_to.clicked.connect(self._on_go_to_clicked)
        self.cartesian_display_frame_combo.currentIndexChanged.connect(self._on_cartesian_display_frame_changed)
        self.tool_source_combo.currentIndexChanged.connect(self._on_tool_source_changed)
        self.target_mode_combo.currentIndexChanged.connect(self._on_target_mode_changed)
        self.motion_mode_combo.currentIndexChanged.connect(self._on_motion_mode_changed)
        self.keypoints_table.itemSelectionChanged.connect(self._on_table_selection_changed)
        self.keypoints_table.itemDoubleClicked.connect(self._on_table_item_double_clicked)
        self.btn_edit_program_base.clicked.connect(self.editProgramBaseRequested.emit)

    def _emit_keypoints_changed(self) -> None:
        self.keypoints_changed.emit(self.get_keypoints())

    def _emit_selection_changed(self) -> None:
        self.keypointSelectionChanged.emit(self._selected_row())

    def _update_buttons_state(self) -> None:
        row = self._selected_row()
        has_selection = row is not None
        
        self.btn_edit.setEnabled(has_selection)
        self.btn_go_to.setEnabled(has_selection)

    def _on_cartesian_display_frame_changed(self, _index: int) -> None:
        self.cartesianDisplayFrameChanged.emit(self.get_cartesian_display_frame())

    def _on_tool_source_changed(self, _index: int) -> None:
        self._current_tool_source = self.tool_source_combo.currentData()
        self.toolSourceChanged.emit(self._current_tool_source)

    def _on_target_mode_changed(self, _index: int) -> None:
        self.targetModeChanged.emit(self.get_target_mode())

    def _on_motion_mode_changed(self, _index: int) -> None:
        self.motionModeChanged.emit(self.get_motion_mode())

    def get_cartesian_display_frame(self) -> str:
        return ReferenceFrame.from_value(self.cartesian_display_frame_combo.currentData()).value

    def set_cartesian_display_frame(self, display_frame: str, emit_signal: bool = False) -> None:
        normalized = ReferenceFrame.from_value(display_frame)
        index = self.cartesian_display_frame_combo.findData(normalized.value)
        if index < 0:
            return
        self.cartesian_display_frame_combo.blockSignals(True)
        self.cartesian_display_frame_combo.setCurrentIndex(index)
        self.cartesian_display_frame_combo.blockSignals(False)
        if emit_signal:
            self.cartesianDisplayFrameChanged.emit(normalized.value)

    def get_tool_source(self) -> str:
        """Retourne la source du tool sélectionnée (CURRENT ou PROGRAM)."""
        return self._current_tool_source

    def set_tool_source(self, tool_source: str, emit_signal: bool = False) -> None:
        """Définir la source du tool (CURRENT ou PROGRAM)."""
        index = self.tool_source_combo.findData(tool_source)
        if index < 0:
            return
        self.tool_source_combo.blockSignals(True)
        self.tool_source_combo.setCurrentIndex(index)
        self.tool_source_combo.blockSignals(False)
        self._current_tool_source = tool_source
        if emit_signal:
            self.toolSourceChanged.emit(tool_source)

    def get_target_mode(self) -> str:
        """Retourne le mode de cibles sélectionné (THEORETICAL ou COMPENSATED)."""
        return self.target_mode_combo.currentData()

    def set_target_mode(self, target_mode: str, emit_signal: bool = False) -> None:
        """Définir le mode de cibles (THEORETICAL ou COMPENSATED)."""
        index = self.target_mode_combo.findData(target_mode)
        if index < 0:
            return
        self.target_mode_combo.blockSignals(True)
        self.target_mode_combo.setCurrentIndex(index)
        self.target_mode_combo.blockSignals(False)
        if emit_signal:
            self.targetModeChanged.emit(target_mode)

    def get_motion_mode(self) -> str:
        """Retourne le mode de mouvement sélectionné (CARTESIAN ou ARTICULAR)."""
        return self.motion_mode_combo.currentData()

    def set_motion_mode(self, motion_mode: str, emit_signal: bool = False) -> None:
        """Définir le mode de mouvement (CARTESIAN ou ARTICULAR)."""
        index = self.motion_mode_combo.findData(motion_mode)
        if index < 0:
            return
        self.motion_mode_combo.blockSignals(True)
        self.motion_mode_combo.setCurrentIndex(index)
        self.motion_mode_combo.blockSignals(False)
        if emit_signal:
            self.motionModeChanged.emit(motion_mode)

    def _on_edit_clicked(self) -> None:
        row = self._selected_row()
        if row is not None:
            self.edit_requested.emit(row)

    def _on_go_to_clicked(self) -> None:
        row = self._selected_row()
        if row is not None:
            self.goToRequested.emit(row)

    def _on_table_selection_changed(self) -> None:
        self._update_buttons_state()
        self._emit_selection_changed()

    def _on_table_item_double_clicked(self, item: QTableWidgetItem) -> None:
        row = item.row()
        if row < 0 or row >= len(self._keypoints):
            return
        self.edit_requested.emit(row)
        self.keypointSelectionChanged.emit(row)

    def _selected_row(self) -> Optional[int]:
        model = self.keypoints_table.selectionModel()
        if model is None:
            return None
        indexes = model.selectedRows()
        if not indexes:
            return None
        return indexes[0].row()

    @staticmethod
    def _keypoint_target_values(keypoint: TrajectoryKeypoint) -> list[float]:
        if keypoint.target_type == KeypointTargetType.CARTESIAN:
            return keypoint.cartesian_target.to_list()
        return keypoint.joint_target

    @staticmethod
    def _speed_text(keypoint: TrajectoryKeypoint) -> str:
        if keypoint.mode == KeypointMotionMode.PTP:
            return f"{keypoint.speed:.1f} %"
        return f"{keypoint.speed:.3f} m/s"

    @staticmethod
    def _mode_text(keypoint: TrajectoryKeypoint) -> str:
        if keypoint.mode == KeypointMotionMode.CUBIC:
            return "BEZIER"
        return keypoint.mode.value

    @staticmethod
    def _configuration_text(keypoint: TrajectoryKeypoint) -> str:
        if keypoint.target_type == KeypointTargetType.JOINT:
            forced = keypoint.forced_config.name if keypoint.forced_config is not None else "?"
            return f"JOINT({forced})"
        if keypoint.configuration_policy.name == "AUTO":
            return "AUTO"
        if keypoint.configuration_policy.name == "CURRENT_BRANCH":
            return "CURRENT_BRANCH"
        if keypoint.configuration_policy.name == "FORCED":
            forced = keypoint.forced_config.name if keypoint.forced_config is not None else "?"
            return f"FORCED({forced})"
        return "AUTO"

    def _refresh_table(self) -> None:
        self.keypoints_table.setRowCount(0)
        for idx, keypoint in enumerate(self._keypoints):
            self.keypoints_table.insertRow(idx)
            target_values = self._keypoint_target_values(keypoint)
            configs_txt = self._configuration_text(keypoint)

            values = [
                (
                    f"CART({keypoint.cartesian_frame.value})"
                    if keypoint.target_type == KeypointTargetType.CARTESIAN
                    else "JOINT"
                ),
                self._mode_text(keypoint),
                self._speed_text(keypoint),
            ]
            values.extend(f"{v:.3f}" for v in target_values[:6])
            values.append(configs_txt)

            for col, text in enumerate(values):
                self.keypoints_table.setItem(idx, col, QTableWidgetItem(text))
        self._update_buttons_state()

    def set_keypoints(self, keypoints: list[TrajectoryKeypoint]) -> None:
        self._keypoints = [keypoint.clone() for keypoint in keypoints]
        self._refresh_table()
        self._emit_selection_changed()

    def get_keypoints(self) -> list[TrajectoryKeypoint]:
        return [keypoint.clone() for keypoint in self._keypoints]

    def clear(self) -> None:
        self._keypoints = []
        self._refresh_table()

    def keypoints_table(self) -> QTableWidget:
        """Retourne la table des keypoints pour compatibilité."""
        return self.keypoints_table

    def cartesian_display_frame_combo(self) -> QComboBox:
        """Retourne le combo box du frame pour compatibilité."""
        return self.cartesian_display_frame_combo

    def tool_source_combo(self) -> QComboBox:
        """Retourne le combo box du tool source pour compatibilité."""
        return self.tool_source_combo

    def select_row(self, row: int) -> None:
        """Sélectionne une ligne dans la table des keypoints."""
        if 0 <= row < self.keypoints_table.rowCount():
            self.keypoints_table.selectRow(row)

    def set_target_mode_enabled(self, enabled: bool) -> None:
        """Active/desactive l option COMPENSE dans target_mode_combo."""
        index_compensated = self.target_mode_combo.findData("COMPENSATED")
        if index_compensated >= 0:
            self.target_mode_combo.model().item(index_compensated).setEnabled(enabled)
        # Si on desactive COMPENSE et qu il est selectionne, revenir a THEORETICAL
        if not enabled and self.get_target_mode() == "COMPENSATED":
            self.set_target_mode("THEORETICAL", emit_signal=True)

    def set_program_base_edit_enabled(self, enabled: bool) -> None:
        self.btn_edit_program_base.setEnabled(bool(enabled))
