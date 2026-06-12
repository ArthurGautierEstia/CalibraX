from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QBrush, QColor
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from models.reference_frame import ReferenceFrame
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.trajectory_keypoint import (
    KeypointMotionMode,
    KeypointTargetType,
    TrajectoryKeypoint,
)
from models.workspace_model import WorkspaceModel


class ProgramKeypointsWidget(QWidget):
    """Widget pour afficher et gerer les points cles d'un programme robot."""

    goToRequested = pyqtSignal(int)
    keypointSelectionChanged = pyqtSignal(object)
    keypoints_changed = pyqtSignal(list)
    cartesianDisplayFrameChanged = pyqtSignal(str)
    add_requested = pyqtSignal()
    edit_requested = pyqtSignal(int)
    delete_requested = pyqtSignal()
    targetModeChanged = pyqtSignal(str)
    motionModeChanged = pyqtSignal(str)
    programSettingsRequested = pyqtSignal()

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
        self.btn_add = QPushButton("Ajouter")
        self.btn_edit = QPushButton("Editer")
        self.btn_go_to = QPushButton("Aller a")
        self.btn_delete = QPushButton("Supprimer")
        self.cartesian_display_frame_combo = QComboBox()
        self.target_mode_combo = QComboBox()
        self.motion_mode_combo = QComboBox()
        self.btn_program_settings = QPushButton("Paramètres")

        self._keypoints: list[TrajectoryKeypoint] = []
        self._has_program = False
        self._row_roles: list[str] = []
        self._row_approx_texts: list[str] = []
        self._row_cible_overrides: list[str | None] = []
        self._ext_axis_col_headers: list[str] = []
        self._ext_axis_values_per_row: list[tuple[float, ...]] = []

        self._setup_ui()
        self._setup_connections()
        self._update_buttons_state()

    def _setup_ui(self) -> None:
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)

        layout = QVBoxLayout(self)

        title = QLabel("Points cles du programme")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(title)

        selectors_row = QGridLayout()
        selectors_row.setContentsMargins(0, 0, 0, 0)
        selectors_row.setHorizontalSpacing(12)
        selectors_row.setVerticalSpacing(0)

        # Repère d'EXPRESSION des coordonnées de la table (n'altère pas la trajectoire) ;
        # il définit aussi le $BASE utilisé à la génération KRL.
        base_layout = QHBoxLayout()
        base_layout.setContentsMargins(0, 0, 0, 0)
        base_layout.setSpacing(6)
        base_layout.addWidget(QLabel("Repère : "))
        self.cartesian_display_frame_combo.addItem("Programme", ReferenceFrame.PROGRAM.value)
        self.cartesian_display_frame_combo.addItem("Robot", ReferenceFrame.ROBOT.value)
        self.cartesian_display_frame_combo.addItem("Monde", ReferenceFrame.WORLD.value)
        base_layout.addWidget(self.cartesian_display_frame_combo, 1)

        target_layout = QHBoxLayout()
        target_layout.setContentsMargins(0, 0, 0, 0)
        target_layout.setSpacing(6)
        target_layout.addWidget(QLabel("Cibles : "))
        self.target_mode_combo.addItem("Theorique", "THEORETICAL")
        self.target_mode_combo.addItem("Compense", "COMPENSATED")
        target_layout.addWidget(self.target_mode_combo, 1)

        mode_layout = QHBoxLayout()
        mode_layout.setContentsMargins(0, 0, 0, 0)
        mode_layout.setSpacing(6)
        mode_layout.addWidget(QLabel("Mode : "))
        self.motion_mode_combo.addItem("Cartesien", "CARTESIAN")
        self.motion_mode_combo.addItem("Articulaire", "ARTICULAR")
        mode_layout.addWidget(self.motion_mode_combo, 1)

        selectors_row.addLayout(base_layout, 0, 0)
        selectors_row.addLayout(target_layout, 0, 1)
        selectors_row.addLayout(mode_layout, 0, 2)
        for column in range(3):
            selectors_row.setColumnStretch(column, 1)

        layout.addLayout(selectors_row)

        base_actions_row = QHBoxLayout()
        base_actions_row.addWidget(self.btn_program_settings, 1)
        base_actions_row.addStretch(3)
        layout.addLayout(base_actions_row)

        self.keypoints_table.setHorizontalHeaderLabels(
            [
                "Cible",
                "Mode",
                "Vitesse",
                "J1 / X",
                "J2 / Y",
                "J3 / Z",
                "J4 / A",
                "J5 / B",
                "J6 / C",
                "Approx",
            ]
        )

        header = self.keypoints_table.horizontalHeader()
        header.setMinimumSectionSize(60)

        for col in range(0, 10):
            header.setSectionResizeMode(col, QHeaderView.ResizeMode.ResizeToContents)

        self.keypoints_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.keypoints_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.keypoints_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.keypoints_table.setMinimumHeight(220)
        layout.addWidget(self.keypoints_table)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self.btn_add)
        btn_row.addWidget(self.btn_edit)
        btn_row.addWidget(self.btn_go_to)
        btn_row.addWidget(self.btn_delete)
        btn_row.addStretch()
        layout.addLayout(btn_row)

    def _setup_connections(self) -> None:
        self.btn_add.clicked.connect(self._on_add_clicked)
        self.btn_edit.clicked.connect(self._on_edit_clicked)
        self.btn_go_to.clicked.connect(self._on_go_to_clicked)
        self.btn_delete.clicked.connect(self._on_delete_clicked)
        self.cartesian_display_frame_combo.currentIndexChanged.connect(self._on_cartesian_display_frame_changed)
        self.target_mode_combo.currentIndexChanged.connect(self._on_target_mode_changed)
        self.motion_mode_combo.currentIndexChanged.connect(self._on_motion_mode_changed)
        self.keypoints_table.itemSelectionChanged.connect(self._on_table_selection_changed)
        self.keypoints_table.itemDoubleClicked.connect(self._on_table_item_double_clicked)
        self.btn_program_settings.clicked.connect(self.programSettingsRequested.emit)

    def _emit_selection_changed(self) -> None:
        self.keypointSelectionChanged.emit(self._selected_row())

    def _update_buttons_state(self) -> None:
        row = self._selected_row()
        has_selection = row is not None
        is_locked = has_selection and self._is_row_locked(row)

        self.btn_add.setEnabled(self._has_program)
        self.btn_edit.setEnabled(has_selection and not is_locked)
        self.btn_go_to.setEnabled(has_selection)
        self.btn_delete.setEnabled(has_selection and not is_locked)
        self.btn_program_settings.setEnabled(self._has_program)

    _LOCKED_ROLES = frozenset({"HOME_START", "HOME_END", "APPROACH", "RETRACT", "EXTERNAL_SETUP"})

    def _is_row_locked(self, row: int) -> bool:
        if row < 0 or row >= len(self._row_roles):
            return False
        return self._row_roles[row] in self._LOCKED_ROLES

    def _on_cartesian_display_frame_changed(self, _index: int) -> None:
        self.cartesianDisplayFrameChanged.emit(self.get_cartesian_display_frame())

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

    def get_target_mode(self) -> str:
        return self.target_mode_combo.currentData()

    def set_target_mode(self, target_mode: str, emit_signal: bool = False) -> None:
        index = self.target_mode_combo.findData(target_mode)
        if index < 0:
            return
        self.target_mode_combo.blockSignals(True)
        self.target_mode_combo.setCurrentIndex(index)
        self.target_mode_combo.blockSignals(False)
        if emit_signal:
            self.targetModeChanged.emit(target_mode)

    def get_motion_mode(self) -> str:
        return self.motion_mode_combo.currentData()

    def set_motion_mode(self, motion_mode: str, emit_signal: bool = False) -> None:
        index = self.motion_mode_combo.findData(motion_mode)
        if index < 0:
            return
        self.motion_mode_combo.blockSignals(True)
        self.motion_mode_combo.setCurrentIndex(index)
        self.motion_mode_combo.blockSignals(False)
        if emit_signal:
            self.motionModeChanged.emit(motion_mode)

    def _on_add_clicked(self) -> None:
        self.add_requested.emit()

    def _on_edit_clicked(self) -> None:
        row = self._selected_row()
        if row is not None:
            self.edit_requested.emit(row)

    def _on_go_to_clicked(self) -> None:
        row = self._selected_row()
        if row is not None:
            self.goToRequested.emit(row)

    def _on_delete_clicked(self) -> None:
        row = self._selected_row()
        if row is not None:
            self.delete_requested.emit()

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
        return keypoint.joint_target.to_list()

    @staticmethod
    def _speed_text(keypoint: TrajectoryKeypoint) -> str:
        if keypoint.mode == KeypointMotionMode.PTP:
            return f"{keypoint.speed:.1f} %"
        return f"{keypoint.speed:.3f} m/s"

    @staticmethod
    def _mode_text(keypoint: TrajectoryKeypoint) -> str:
        if keypoint.mode == KeypointMotionMode.BEZIER:
            return "Bézier"
        if keypoint.mode == KeypointMotionMode.LINEAR:
            return "LIN"
        return keypoint.mode.value

    def _refresh_table(self) -> None:
        self.keypoints_table.setRowCount(0)
        for idx, keypoint in enumerate(self._keypoints):
            self.keypoints_table.insertRow(idx)
            target_values = self._keypoint_target_values(keypoint)

            cible_override = self._row_cible_overrides[idx] if idx < len(self._row_cible_overrides) else None
            cible_text = cible_override if cible_override is not None else (
                "CARTESIAN" if keypoint.target_type == KeypointTargetType.CARTESIAN else "JOINT"
            )
            approx_text = self._row_approx_texts[idx] if idx < len(self._row_approx_texts) else ""

            values = [
                cible_text,
                self._mode_text(keypoint),
                self._speed_text(keypoint),
            ]
            values.extend(f"{v:.3f}" for v in target_values[:6])
            values.append(approx_text)

            ext_vals = self._ext_axis_values_per_row[idx] if idx < len(self._ext_axis_values_per_row) else ()
            for col_idx in range(len(self._ext_axis_col_headers)):
                v = ext_vals[col_idx] if col_idx < len(ext_vals) else float("nan")
                values.append("" if v != v else f"{v:.3f}")  # nan check

            for col, text in enumerate(values):
                self.keypoints_table.setItem(idx, col, QTableWidgetItem(text))

            if self._is_row_locked(idx):
                gray = QBrush(QColor(180, 180, 180))
                for col in range(self.keypoints_table.columnCount()):
                    item = self.keypoints_table.item(idx, col)
                    if item is not None:
                        item.setForeground(gray)

        self._update_buttons_state()

    def set_keypoints(self, keypoints: list[TrajectoryKeypoint]) -> None:
        self._keypoints = [keypoint.clone() for keypoint in keypoints]
        n = len(self._keypoints)
        if len(self._row_roles) != n:
            self._row_roles = ["NORMAL"] * n
        if len(self._row_approx_texts) != n:
            self._row_approx_texts = [""] * n
        if len(self._row_cible_overrides) != n:
            self._row_cible_overrides = [None] * n
        self._refresh_table()
        self._emit_selection_changed()

    def setup_external_axes_columns(self, col_headers: list[str]) -> None:
        self._ext_axis_col_headers = list(col_headers)
        total = 10 + len(col_headers)
        self.keypoints_table.setColumnCount(total)
        base_labels = [
            "Cible", "Mode", "Vitesse",
            "J1 / X", "J2 / Y", "J3 / Z", "J4 / A", "J5 / B", "J6 / C",
            "Approx",
        ]
        self.keypoints_table.setHorizontalHeaderLabels(base_labels + col_headers)
        header = self.keypoints_table.horizontalHeader()
        for col in range(total):
            header.setSectionResizeMode(col, QHeaderView.ResizeMode.ResizeToContents)

    def set_row_metadata(
        self,
        roles: list[str],
        approx_texts: list[str],
        cible_overrides: list[str | None],
        ext_axis_values_per_row: list[tuple[float, ...]] | None = None,
    ) -> None:
        self._row_roles = list(roles)
        self._row_approx_texts = list(approx_texts)
        self._row_cible_overrides = list(cible_overrides)
        self._ext_axis_values_per_row = list(ext_axis_values_per_row) if ext_axis_values_per_row is not None else []
        self._apply_row_metadata()
        self._update_buttons_state()

    def _apply_row_metadata(self) -> None:
        gray = QBrush(QColor(180, 180, 180))
        default_fg = QBrush(QColor(0, 0, 0, 0))
        for row in range(self.keypoints_table.rowCount()):
            cible_override = self._row_cible_overrides[row] if row < len(self._row_cible_overrides) else None
            approx_text = self._row_approx_texts[row] if row < len(self._row_approx_texts) else ""
            locked = self._is_row_locked(row)

            if cible_override is not None:
                item = self.keypoints_table.item(row, 0)
                if item is not None:
                    item.setText(cible_override)

            approx_item = self.keypoints_table.item(row, 9)
            if approx_item is not None:
                approx_item.setText(approx_text)
            else:
                self.keypoints_table.setItem(row, 9, QTableWidgetItem(approx_text))

            ext_vals = self._ext_axis_values_per_row[row] if row < len(self._ext_axis_values_per_row) else ()
            for col_idx in range(len(self._ext_axis_col_headers)):
                col = 10 + col_idx
                v = ext_vals[col_idx] if col_idx < len(ext_vals) else float("nan")
                text = "" if v != v else f"{v:.3f}"
                item = self.keypoints_table.item(row, col)
                if item is not None:
                    item.setText(text)
                else:
                    self.keypoints_table.setItem(row, col, QTableWidgetItem(text))

            brush = gray if locked else default_fg
            for col in range(self.keypoints_table.columnCount()):
                item = self.keypoints_table.item(row, col)
                if item is not None:
                    item.setForeground(brush)

    def set_program_loaded(self, loaded: bool) -> None:
        self._has_program = bool(loaded)
        self._update_buttons_state()

    def get_keypoints(self) -> list[TrajectoryKeypoint]:
        return [keypoint.clone() for keypoint in self._keypoints]

    def clear(self) -> None:
        self._keypoints = []
        self._refresh_table()

    def select_row(self, row: int) -> None:
        if 0 <= row < self.keypoints_table.rowCount():
            self.keypoints_table.selectRow(row)

    def set_target_mode_enabled(self, enabled: bool) -> None:
        index_compensated = self.target_mode_combo.findData("COMPENSATED")
        if index_compensated >= 0:
            self.target_mode_combo.model().item(index_compensated).setEnabled(enabled)
        if not enabled and self.get_target_mode() == "COMPENSATED":
            self.set_target_mode("THEORETICAL", emit_signal=True)
