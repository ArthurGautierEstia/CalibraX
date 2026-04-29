import os
from dataclasses import dataclass
from typing import Any

from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QListWidgetItem,
)
from PyQt6.QtCore import Qt, QSize, pyqtSignal
from PyQt6.QtGui import QBrush, QColor, QFont, QIcon, QPainter, QPen, QPixmap
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui
import numpy as np
from stl import mesh

import utils.math_utils as math_utils
from models.app_session_file import ViewerDisplayState
from models.collision_scene_model import CollisionSceneModel
from models.primitive_collider_models import PrimitiveCollider
from models.robot_model import RobotModel
from models.reference_frame import ReferenceFrame
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from widgets.frame_visibility_overlay_widget import FrameVisibilityOverlayWidget
from widgets.viewer_control_overlay_widget import ViewerControlOverlayWidget
from utils.reference_frame_utils import (
    FrameTransform,
    normalize_pose6,
    transform_matrix_base_to_world,
    transform_points_base_to_world,
)


@dataclass(frozen=True)
class WorkspaceElementState:
    name: str
    cad_model: str
    pose: tuple[float, float, float, float, float, float]
    world_transform: np.ndarray
    revision: int

    @classmethod
    def from_dict(cls, value: dict[str, Any], index: int) -> "WorkspaceElementState":
        name = str(value.get("name", f"Element {index + 1}")).strip()
        if name == "":
            name = f"Element {index + 1}"
        cad_model = str(value.get("cad_model", "")).strip()
        pose = tuple(normalize_pose6(value.get("pose", [0.0] * 6)))
        transform = math_utils.pose_zyx_to_matrix(pose)
        transform.setflags(write=False)
        revision = hash((name, cad_model, pose))
        return cls(name, cad_model, pose, transform, revision)


@dataclass(frozen=True)
class PrimitiveColliderState:
    name: str
    enabled: bool
    shape: str
    pose: tuple[float, float, float, float, float, float]
    size_x: float
    size_y: float
    size_z: float
    radius: float
    height: float
    local_transform: np.ndarray
    shape_key: tuple[str, float, float, float, float, float]
    revision: int

    @classmethod
    def from_dict(cls, value: dict[str, Any], index: int) -> "PrimitiveColliderState":
        name = str(value.get("name", f"Zone {index + 1}")).strip()
        if name == "":
            name = f"Zone {index + 1}"
        shape = str(value.get("shape", "box")).strip().lower()
        shape = shape if shape in {"box", "cylinder", "sphere"} else "box"
        pose = tuple(normalize_pose6(value.get("pose", [0.0] * 6)))
        size_x = max(0.0, float(value.get("size_x", 100.0)))
        size_y = max(0.0, float(value.get("size_y", 100.0)))
        size_z = max(0.0, float(value.get("size_z", 100.0)))
        radius = max(0.0, float(value.get("radius", 50.0)))
        height = max(0.0, float(value.get("height", 100.0)))
        local_transform = math_utils.pose_zyx_to_matrix(pose)
        local_transform.setflags(write=False)
        shape_key = (shape, size_x, size_y, size_z, radius, height)
        revision = hash((name, bool(value.get("enabled", True)), shape_key, pose))
        return cls(
            name=name,
            enabled=bool(value.get("enabled", True)),
            shape=shape,
            pose=pose,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            radius=radius,
            height=height,
            local_transform=local_transform,
            shape_key=shape_key,
            revision=revision,
        )

class Viewer3DWidget(QWidget):
    """Widget pour la visualisation 3D avec PyQtGraph"""
    display_state_changed = pyqtSignal(object)

    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        self.robot_links: list[gl.GLMeshItem] = []
        self.robot_ghost_links: list[gl.GLMeshItem] = []
        self._trajectory_path_items: list[gl.GLLinePlotItem] = []
        self._trajectory_keypoints_item: gl.GLScatterPlotItem | None = None
        self._trajectory_keypoint_selected_item: gl.GLScatterPlotItem | None = None
        self._trajectory_keypoint_editing_item: gl.GLScatterPlotItem | None = None
        self._trajectory_tangent_out_items: list[gl.GLLinePlotItem] = []
        self._trajectory_tangent_in_items: list[gl.GLLinePlotItem] = []
        self._trajectory_path_segments: list[tuple[np.ndarray, tuple[float, float, float, float]]] | None = None
        self._trajectory_keypoint_points: np.ndarray | None = None
        self._trajectory_keypoint_selected_index: int | None = None
        self._trajectory_keypoint_editing_index: int | None = None
        self._trajectory_tangent_out_segments: list[np.ndarray] | None = None
        self._trajectory_tangent_in_segments: list[np.ndarray] | None = None
        self.last_dh_matrices = []
        self.last_corrected_matrices = []
        self.last_ghost_corrected_matrices = []
        self._robot_link_matrix_indices: list[int] = []
        self._robot_ghost_link_matrix_indices: list[int] = []
        self._robot_link_roles: list[str] = []
        self._robot_ghost_link_roles: list[str] = []
        self._robot_frame_items: list[gl.GLLinePlotItem] = []
        self._workspace_frame_items: list[gl.GLLinePlotItem] = []
        self.last_invert_table = []
        self.frames_visibility: list[bool] = []
        self.workspace_frames_visibility: list[bool] = []
        self.show_axes = True
        self._cad_loaded = False
        self._cad_showed = True
        self._ghost_visible = False
        self._robot_model: RobotModel | None = None
        self._tool_model: ToolModel | None = None
        self._workspace_model: WorkspaceModel | None = None
        self._robot_base_transform_world = FrameTransform.from_pose([0.0] * 6)
        self._workspace_structure_revision: int | None = None
        self._mesh_data_cache: dict[str, gl.MeshData] = {}
        self._missing_mesh_paths: set[str] = set()
        self._primitive_mesh_cache: dict[str, gl.MeshData] = {}
        self._workspace_elements: list[WorkspaceElementState] = []
        self._workspace_frame_matrices: list[np.ndarray] = []
        self._workspace_frame_labels: list[str] = []
        self._workspace_tcp_zones: list[PrimitiveCollider] = []
        self._workspace_collision_zones: list[PrimitiveCollider] = []
        self._robot_colliders: list[PrimitiveCollider] = []
        self._tool_colliders: list[PrimitiveCollider] = []
        self._workspace_element_items: list[gl.GLMeshItem] = []
        self._workspace_tcp_zone_items: list[gl.GLMeshItem] = []
        self._workspace_collision_zone_items: list[gl.GLMeshItem] = []
        self._robot_collider_items: list[gl.GLMeshItem] = []
        self._tool_collider_items: list[gl.GLMeshItem] = []
        self._workspace_tcp_zones_visible = True
        self._workspace_collision_zones_visible = True
        self._robot_colliders_visible = True
        self._tool_colliders_visible = True
        self.transparency_enabled = False
        self._loading_feedback_depth = 0
        self._position_match_tolerance_deg = 0.05
        self._robot_controls_collapsed = False
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Viewer 3D
        self.viewer = gl.GLViewWidget()
        self.viewer.opts['glOptions'] = 'translucent'
        self.viewer.opts['depth'] = True
        self.viewer.setCameraPosition(distance=2000, elevation=40, azimuth=45)
        #self.viewer.setMinimumSize(900, 400)
        self.viewer.setBackgroundColor(45, 45, 48, 255)
        layout.addWidget(self.viewer)

        # --- LISTE DES REPERES (Overlay en haut a droite) ---
        self.frame_overlay = FrameVisibilityOverlayWidget(self.viewer)
        self.frame_overlay.set_title("Frames robot")
        self.frame_list = self.frame_overlay.list_widget
        self.workspace_frame_overlay = FrameVisibilityOverlayWidget(self.viewer)
        self.workspace_frame_overlay.set_title("Frames scene")
        self.viewer_control_overlay = ViewerControlOverlayWidget(self.viewer)
        self.robot_controls_toggle_button = QPushButton(self.viewer)
        self.robot_controls_toggle_button.setStyleSheet(
            """
            QPushButton {
                background-color: rgba(25, 25, 28, 160);
                color: lightgray;
                border: 1px solid rgba(255, 255, 255, 35);
                border-radius: 6px;
                padding: 6px 10px;
                text-align: left;
            }
            QPushButton:hover {
                background-color: rgba(40, 40, 45, 185);
            }
            """
        )
        self.position_buttons_overlay = QWidget(self.viewer)
        self.position_buttons_overlay.setObjectName("viewerPositionOverlay")
        self.position_buttons_overlay.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.position_buttons_overlay.setStyleSheet("""
            QWidget#viewerPositionOverlay {
                background-color: rgba(0, 0, 0, 18);
                border: 1px solid rgba(255, 255, 255, 20);
                border-radius: 10px;
            }
        """)

        # --- LABEL EN HAUT A GAUCHE ---
        self.msg_label = QLabel("", self.viewer)  # Parent = viewer pour l'overlay
        self.msg_label.setStyleSheet("""
            QLabel {
                color: white;
                background-color: transparent;
                padding: 5px;
                border-radius: 3px;
            }
        """)
        self.msg_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        # Position initiale (sera ajustée dans resizeEvent)
        self.msg_label.adjustSize()
        self._position_overlays()
        self.toolbar_overlay = QWidget(self.viewer)
        self.toolbar_overlay.setObjectName("viewerToolbarOverlay")
        self.toolbar_overlay.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.toolbar_overlay.setStyleSheet("""
            QWidget#viewerToolbarOverlay {
                background-color: rgba(0, 0, 0, 18);
                border: 1px solid rgba(255, 255, 255, 20);
                border-radius: 10px;
            }
        """)
        toolbar_layout = QHBoxLayout(self.toolbar_overlay)
        toolbar_layout.setContentsMargins(8, 8, 8, 8)
        toolbar_layout.setSpacing(6)

        self.btn_toggle_cad = self._create_overlay_button("Affichage CAD", "cad")
        self.btn_toggle_transparency = self._create_overlay_button("Transparence", "transparency")
        self.btn_toggle_axes = self._create_overlay_button("Afficher / Masquer tous les repères", "axes")
        self.btn_toggle_axes_base_tool = self._create_overlay_button("Repères Base & Tool", "base_tool")
        self.btn_toggle_workspace_tcp_zones = self._create_overlay_button("Zones TCP", "tcp_zones")
        self.btn_toggle_workspace_collision_zones = self._create_overlay_button("Zones collisions", "collision_zones")
        self.btn_toggle_robot_colliders = self._create_overlay_button("Colliders robot", "robot_colliders")
        self.btn_toggle_tool_colliders = self._create_overlay_button("Colliders tool", "tool_colliders")

        for button in (
            self.btn_toggle_cad,
            self.btn_toggle_transparency,
            self.btn_toggle_axes,
            self.btn_toggle_axes_base_tool,
            self.btn_toggle_workspace_tcp_zones,
            self.btn_toggle_workspace_collision_zones,
            self.btn_toggle_robot_colliders,
            self.btn_toggle_tool_colliders,
        ):
            toolbar_layout.addWidget(button)
        self.toolbar_overlay.adjustSize()

        position_layout = QVBoxLayout(self.position_buttons_overlay)
        position_layout.setContentsMargins(6, 6, 6, 6)
        position_layout.setSpacing(6)
        self.btn_go_position_calibration_overlay = self._create_overlay_button(
            "Position calibration",
            "calibration_pose",
            checkable=True,
            parent=self.position_buttons_overlay,
        )
        self.btn_go_position_zero_overlay = self._create_overlay_button(
            "Position zéro",
            "zero_pose",
            checkable=True,
            parent=self.position_buttons_overlay,
        )
        self.btn_go_home_position_overlay = self._create_overlay_button(
            "Position home",
            "home_pose",
            checkable=True,
            parent=self.position_buttons_overlay,
        )
        for button in (
            self.btn_go_position_calibration_overlay,
            self.btn_go_position_zero_overlay,
            self.btn_go_home_position_overlay,
        ):
            position_layout.addWidget(button)
        self.position_buttons_overlay.adjustSize()
        
        self.setLayout(layout)
        self.add_grid()

        self.frame_overlay.frame_clicked.connect(self.on_frame_clicked)
        self.frame_overlay.geometry_changed.connect(self._position_overlays)
        self.workspace_frame_overlay.frame_clicked.connect(self.on_workspace_frame_clicked)
        self.workspace_frame_overlay.geometry_changed.connect(self._position_overlays)
        self.robot_controls_toggle_button.clicked.connect(self._toggle_robot_controls_overlay)
        self.btn_toggle_cad.clicked.connect(self._on_cad_button_clicked)
        self.btn_toggle_transparency.clicked.connect(self._on_transparency_button_clicked)
        self.btn_toggle_axes.clicked.connect(self._on_axes_button_clicked)
        self.btn_toggle_axes_base_tool.clicked.connect(self.toogle_base_axis_frames)
        self.btn_toggle_workspace_tcp_zones.clicked.connect(self._on_workspace_tcp_zones_button_clicked)
        self.btn_toggle_workspace_collision_zones.clicked.connect(self._on_workspace_collision_zones_button_clicked)
        self.btn_toggle_robot_colliders.clicked.connect(self._on_robot_colliders_button_clicked)
        self.btn_toggle_tool_colliders.clicked.connect(self._on_tool_colliders_button_clicked)
        self.btn_go_position_zero_overlay.clicked.connect(self.get_overlay_joints_widget().position_zero_requested.emit)
        self.btn_go_position_calibration_overlay.clicked.connect(
            self.get_overlay_joints_widget().position_calibration_requested.emit
        )
        self.btn_go_home_position_overlay.clicked.connect(self.get_overlay_joints_widget().home_position_requested.emit)
        self._refresh_toolbar_buttons()
        self._refresh_position_buttons()
        self._refresh_robot_controls_overlay()

    def _position_overlays(self):
        """Positionne la liste en haut a droite et le label en haut a gauche"""
        margin = 10
        if hasattr(self, "toolbar_overlay"):
            self.toolbar_overlay.adjustSize()
            self.toolbar_overlay.move(margin, margin)
        frame_overlay_x = max(margin, self.viewer.width() - self.frame_overlay.width() - margin)
        self.frame_overlay.move(frame_overlay_x, margin)
        workspace_overlay_y = margin + (self.frame_overlay.height() + margin if self.frame_overlay.isVisible() else 0)
        workspace_overlay_x = max(margin, self.viewer.width() - self.workspace_frame_overlay.width() - margin)
        self.workspace_frame_overlay.move(workspace_overlay_x, workspace_overlay_y)
        message_y = margin
        if hasattr(self, "toolbar_overlay"):
            message_y = self.toolbar_overlay.y() + self.toolbar_overlay.height() + 6
        self.msg_label.move(margin, message_y)
        if hasattr(self, "viewer_control_overlay"):
            self.viewer_control_overlay.adjustSize()
            overlay_width = max(0, self.viewer.width() - (2 * margin))
            self.viewer_control_overlay.resize(overlay_width, self.viewer_control_overlay.sizeHint().height())
            control_height = self.viewer_control_overlay.height() if self.viewer_control_overlay.isVisible() else 0
        else:
            control_height = 0
        if hasattr(self, "robot_controls_toggle_button"):
            self.robot_controls_toggle_button.adjustSize()
            toggle_y = max(
                margin,
                self.viewer.height() - self.robot_controls_toggle_button.height() - margin,
            )
            self.robot_controls_toggle_button.move(margin, toggle_y)
        else:
            toggle_y = self.viewer.height() - margin
        if hasattr(self, "viewer_control_overlay"):
            control_y = max(
                margin,
                toggle_y - control_height - 6,
            )
            self.viewer_control_overlay.move(margin, control_y)
        if hasattr(self, "position_buttons_overlay"):
            self.position_buttons_overlay.adjustSize()
            positions_x = max(margin, self.viewer.width() - self.position_buttons_overlay.width() - margin)
            positions_y = max(margin, (self.viewer.height() - self.position_buttons_overlay.height()) // 2)
            self.position_buttons_overlay.move(positions_x, positions_y)

    def resizeEvent(self, event):
        """Repositionne les overlays lors du redimensionnement"""
        super().resizeEvent(event)
        self._position_overlays()

    def _set_label_msg(self, txt: str):
        self.msg_label.setText(txt)
        self.msg_label.adjustSize()
        self._position_overlays()

    def _clear_label_msg(self):
        self.msg_label.clear()
        self.msg_label.adjustSize()
        self._position_overlays()

    def _toggle_robot_controls_overlay(self) -> None:
        self._robot_controls_collapsed = not self._robot_controls_collapsed
        self._refresh_robot_controls_overlay()
        self._position_overlays()

    def _refresh_robot_controls_overlay(self) -> None:
        controls_visible = not self._robot_controls_collapsed
        if hasattr(self, "viewer_control_overlay"):
            self.viewer_control_overlay.setVisible(controls_visible)
        if hasattr(self, "position_buttons_overlay"):
            self.position_buttons_overlay.setVisible(controls_visible)
        if hasattr(self, "robot_controls_toggle_button"):
            arrow = "▸" if self._robot_controls_collapsed else "▾"
            self.robot_controls_toggle_button.setText(f"Contrôles robot {arrow}")
            self.robot_controls_toggle_button.adjustSize()

    def begin_loading_feedback(self, message: str) -> None:
        self._loading_feedback_depth += 1
        self._set_label_msg(message)
        QApplication.setOverrideCursor(Qt.CursorShape.WaitCursor)
        QApplication.processEvents()

    def end_loading_feedback(self) -> None:
        if self._loading_feedback_depth > 0:
            self._loading_feedback_depth -= 1
        if self._loading_feedback_depth <= 0:
            self._loading_feedback_depth = 0
            self._clear_label_msg()
            try:
                QApplication.restoreOverrideCursor()
            except Exception:
                pass
        QApplication.processEvents()

    def get_overlay_joints_widget(self):
        return self.viewer_control_overlay.get_joints_widget()

    def get_overlay_cartesian_widget(self):
        return self.viewer_control_overlay.get_cartesian_widget()

    def get_display_state(self) -> ViewerDisplayState:
        return ViewerDisplayState(
            cad_visible=bool(self._cad_showed),
            transparency_enabled=bool(self.transparency_enabled),
            show_axes=bool(self.show_axes),
            frames_visibility=[bool(v) for v in self.frames_visibility],
            workspace_frames_visibility=[bool(v) for v in self.workspace_frames_visibility],
            workspace_tcp_zones_visible=bool(self._workspace_tcp_zones_visible),
            workspace_collision_zones_visible=bool(self._workspace_collision_zones_visible),
            robot_colliders_visible=bool(self._robot_colliders_visible),
            tool_colliders_visible=bool(self._tool_colliders_visible),
        )

    def apply_display_state(self, state: ViewerDisplayState, emit_signal: bool = False) -> None:
        self._cad_showed = bool(state.cad_visible)
        self.transparency_enabled = bool(state.transparency_enabled)
        self.show_axes = bool(state.show_axes)
        self.frames_visibility = [bool(v) for v in state.frames_visibility]
        self.workspace_frames_visibility = [bool(v) for v in state.workspace_frames_visibility]
        self._workspace_tcp_zones_visible = bool(state.workspace_tcp_zones_visible)
        self._workspace_collision_zones_visible = bool(state.workspace_collision_zones_visible)
        self._robot_colliders_visible = bool(state.robot_colliders_visible)
        self._tool_colliders_visible = bool(state.tool_colliders_visible)
        self._clear_and_refresh()
        if self.transparency_enabled:
            self.set_transparency(True, emit_signal=False)
        self._refresh_toolbar_buttons()
        if emit_signal:
            self._emit_display_state_changed()

    def _emit_display_state_changed(self) -> None:
        self.display_state_changed.emit(self.get_display_state())

    def _create_overlay_button(
        self,
        tooltip: str,
        icon_kind: str,
        checkable: bool = True,
        parent: QWidget | None = None,
    ) -> QPushButton:
        button = QPushButton("", parent if parent is not None else self.toolbar_overlay)
        button.setToolTip(tooltip)
        button.setProperty("icon_kind", icon_kind)
        button.setCheckable(checkable)
        button.setCursor(Qt.CursorShape.PointingHandCursor)
        button.setFixedSize(36, 36)
        button.setIconSize(QSize(20, 20))
        button.setIcon(self._build_toolbar_icon(icon_kind, False))
        button.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                border: 1px solid rgba(255, 255, 255, 28);
                border-radius: 8px;
                padding: 0px;
            }
            QPushButton:hover {
                background-color: rgba(255, 255, 255, 20);
                border-color: rgba(255, 255, 255, 55);
            }
            QPushButton:pressed {
                background-color: rgba(255, 255, 255, 28);
            }
            QPushButton:checked {
                background-color: rgba(255, 140, 0, 48);
                border-color: rgba(255, 140, 0, 120);
            }
        """)
        return button

    def _refresh_toolbar_buttons(self) -> None:
        self._set_overlay_button_state(self.btn_toggle_cad, self._cad_showed)
        self._set_overlay_button_state(self.btn_toggle_transparency, self.transparency_enabled)
        self._set_overlay_button_state(self.btn_toggle_axes, self.show_axes)
        self._set_overlay_button_state(self.btn_toggle_axes_base_tool, self._is_base_tool_only_mode())
        self._set_overlay_button_state(self.btn_toggle_workspace_tcp_zones, self._workspace_tcp_zones_visible)
        self._set_overlay_button_state(self.btn_toggle_workspace_collision_zones, self._workspace_collision_zones_visible)
        self._set_overlay_button_state(self.btn_toggle_robot_colliders, self._robot_colliders_visible)
        self._set_overlay_button_state(self.btn_toggle_tool_colliders, self._tool_colliders_visible)

    def _refresh_position_buttons(self) -> None:
        self._set_overlay_button_state(
            self.btn_go_position_calibration_overlay,
            self._is_robot_at_reference_position("calibration"),
        )
        self._set_overlay_button_state(
            self.btn_go_position_zero_overlay,
            self._is_robot_at_reference_position("zero"),
        )
        self._set_overlay_button_state(
            self.btn_go_home_position_overlay,
            self._is_robot_at_reference_position("home"),
        )

    def _set_overlay_button_state(self, button: QPushButton, active: bool) -> None:
        button.blockSignals(True)
        button.setChecked(bool(active))
        button.setIcon(self._build_toolbar_icon(str(button.property("icon_kind")), active))
        button.blockSignals(False)

    def _is_base_tool_only_mode(self) -> bool:
        size = len(self.frames_visibility)
        if size <= 1:
            return False
        last = size - 1
        return all(visible == (idx == 0 or idx == last) for idx, visible in enumerate(self.frames_visibility))

    def _is_robot_at_reference_position(self, position_kind: str) -> bool:
        if self._robot_model is None:
            return False

        current_joints = list(self._robot_model.get_joints())
        if position_kind == "calibration":
            reference_joints = list(self._robot_model.get_position_calibration())
        elif position_kind == "zero":
            reference_joints = list(self._robot_model.get_position_zero())
        elif position_kind == "home":
            reference_joints = list(self._robot_model.get_home_position())
        else:
            return False

        if len(current_joints) < 6 or len(reference_joints) < 6:
            return False

        return all(
            abs(float(current_joints[idx]) - float(reference_joints[idx])) <= self._position_match_tolerance_deg
            for idx in range(6)
        )

    def _build_toolbar_icon(self, icon_kind: str, active: bool) -> QIcon:
        size = 20
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        color = QColor(210, 220, 230) if not active else QColor("#ff8c00")
        pen = QPen(color, 1.8)
        pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
        painter.setPen(pen)
        painter.setBrush(QBrush(Qt.BrushStyle.NoBrush))

        if icon_kind == "cad":
            painter.drawRoundedRect(3, 5, 14, 10, 2, 2)
            painter.drawLine(6, 5, 10, 2)
            painter.drawLine(17, 5, 13, 2)
            painter.drawLine(10, 2, 13, 2)
        elif icon_kind == "transparency":
            painter.setBrush(QBrush(QColor(color.red(), color.green(), color.blue(), 70)))
            painter.drawEllipse(4, 6, 8, 8)
            painter.drawEllipse(8, 6, 8, 8)
        elif icon_kind == "axes":
            painter.drawLine(10, 3, 10, 17)
            painter.drawLine(3, 10, 17, 10)
            painter.drawLine(6, 14, 14, 6)
        elif icon_kind == "base_tool":
            painter.drawEllipse(2, 8, 4, 4)
            painter.drawEllipse(14, 8, 4, 4)
            painter.drawLine(6, 10, 14, 10)
        elif icon_kind == "tcp_zones":
            painter.drawRoundedRect(3, 3, 14, 14, 3, 3)
            painter.drawLine(10, 6, 10, 14)
            painter.drawLine(6, 10, 14, 10)
        elif icon_kind == "collision_zones":
            painter.drawRoundedRect(4, 4, 12, 12, 3, 3)
            painter.drawLine(6, 6, 14, 14)
            painter.drawLine(14, 6, 6, 14)
        elif icon_kind == "robot_colliders":
            painter.drawEllipse(2, 8, 4, 4)
            painter.drawEllipse(8, 4, 4, 4)
            painter.drawEllipse(14, 8, 4, 4)
            painter.drawLine(6, 10, 8, 6)
            painter.drawLine(12, 6, 14, 10)
        elif icon_kind == "tool_colliders":
            painter.drawLine(4, 15, 15, 4)
            painter.drawLine(9, 4, 15, 4)
            painter.drawLine(15, 4, 15, 10)
            painter.drawEllipse(2, 13, 4, 4)
        elif icon_kind == "calibration_pose":
            painter.drawEllipse(4, 4, 12, 12)
            painter.drawEllipse(8, 8, 4, 4)
        elif icon_kind == "zero_pose":
            font = QFont("Arial", 12)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(0, 0, size, size, int(Qt.AlignmentFlag.AlignCenter), "0")
        elif icon_kind == "home_pose":
            painter.drawLine(4, 10, 10, 4)
            painter.drawLine(10, 4, 16, 10)
            painter.drawLine(6, 9, 6, 16)
            painter.drawLine(14, 9, 14, 16)
            painter.drawLine(6, 16, 14, 16)
            painter.drawLine(9, 16, 9, 12)
            painter.drawLine(9, 12, 11, 12)
            painter.drawLine(11, 12, 11, 16)
        painter.end()
        return QIcon(pixmap)

    def on_frame_clicked(self, index: int):
        """Gère le clic sur un élément de la liste"""
        if index < 0 or index >= len(self.frames_visibility):
            return
        self.frames_visibility[index] = not self.frames_visibility[index]
        self._clear_and_refresh()
        self._refresh_toolbar_buttons()
        self._emit_display_state_changed()
    
    def on_workspace_frame_clicked(self, index: int):
        if index < 0 or index >= len(self.workspace_frames_visibility):
            return
        self.workspace_frames_visibility[index] = not self.workspace_frames_visibility[index]
        self._clear_and_refresh()
        self._refresh_toolbar_buttons()
        self._emit_display_state_changed()

    def _on_cad_button_clicked(self):
        self.set_robot_visibility(not self._cad_showed)

    def _on_transparency_button_clicked(self):
        self.set_transparency(not self.transparency_enabled)

    def _on_axes_button_clicked(self):
        self.show_axes = not self.show_axes
        for i in range(len(self.frames_visibility)):
            self.frames_visibility[i] = self.show_axes
        for i in range(len(self.workspace_frames_visibility)):
            self.workspace_frames_visibility[i] = self.show_axes
        self._clear_and_refresh()
        self._refresh_toolbar_buttons()
        self._emit_display_state_changed()

    def _on_workspace_tcp_zones_button_clicked(self):
        self._workspace_tcp_zones_visible = not self._workspace_tcp_zones_visible
        self._apply_items_visibility(self._workspace_tcp_zone_items, self._workspace_tcp_zones_visible)
        self._refresh_toolbar_buttons()
        self._emit_display_state_changed()

    def _on_workspace_collision_zones_button_clicked(self):
        self._workspace_collision_zones_visible = not self._workspace_collision_zones_visible
        self._apply_items_visibility(self._workspace_collision_zone_items, self._workspace_collision_zones_visible)
        self._refresh_toolbar_buttons()
        self._emit_display_state_changed()

    def _on_robot_colliders_button_clicked(self):
        self._robot_colliders_visible = not self._robot_colliders_visible
        self._apply_items_visibility(self._robot_collider_items, self._robot_colliders_visible)
        self._refresh_toolbar_buttons()
        self._emit_display_state_changed()

    def _on_tool_colliders_button_clicked(self):
        self._tool_colliders_visible = not self._tool_colliders_visible
        self._apply_items_visibility(self._tool_collider_items, self._tool_colliders_visible)
        self._refresh_toolbar_buttons()
        self._emit_display_state_changed()

    def update_frame_list_ui(self):
        """Met à jour l'apparence de la liste (Gras = Visible)"""
        count = len(self.frames_visibility)
        
        # Si le nombre de repères a changé, on recrée la liste
        if self.frame_list.count() != count:
            self.frame_list.clear()
            for i in range(count):
                item = QListWidgetItem(f"Frame {i}")
                item.setTextAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
                item.setSizeHint(QSize(0, 28))
                self.frame_list.addItem(item)
            self.frame_list.show()

        # Mise à jour du style (Gras vs Normal)
        font_bold = QFont()
        font_bold.setBold(True)
        
        font_normal = QFont()
        font_normal.setBold(False)

        for i in range(count):
            item = self.frame_list.item(i)
            
            # Appliquer le style (GRAS + GRIS CLAIR = Visible)
            if self.frames_visibility[i]:
                item.setFont(font_bold)
                item.setForeground(Qt.GlobalColor.lightGray)  # Gris clair en gras
            else:
                item.setFont(font_normal)
                item.setForeground(Qt.GlobalColor.darkGray)  # Gris foncé en normal

        self.frame_overlay.set_frames_visibility(self.frames_visibility)
        self.workspace_frame_overlay.set_frames_visibility(
            self.workspace_frames_visibility,
            self._workspace_frame_labels,
        )

    def add_grid(self):
        grid = gl.GLGridItem()
        grid.setSize(x=4000, y=4000, z=0)
        grid.setSpacing(x=200, y=200, z=200)
        grid.setColor((150, 150, 150, 100))
        self.viewer.addItem(grid)

    def clear_viewer(self):
        self.viewer.clear()
        self.add_grid()
        self._robot_frame_items = []
        self._workspace_frame_items = []
        self._trajectory_path_items = []
        self._trajectory_keypoints_item = None
        self._trajectory_keypoint_selected_item = None
        self._trajectory_keypoint_editing_item = None
        self._trajectory_tangent_out_items = []
        self._trajectory_tangent_in_items = []

    def set_trajectory_path_segments(
        self,
        segments: list[tuple[list[list[float]], tuple[float, float, float, float]]],
    ) -> None:
        parsed_segments: list[tuple[np.ndarray, tuple[float, float, float, float]]] = []
        for points_xyz, color in segments:
            if len(points_xyz) < 2:
                continue
            parsed_segments.append((np.array(points_xyz, dtype=float), color))
        self._trajectory_path_segments = parsed_segments if parsed_segments else None
        self._render_trajectory_overlay()

    def clear_trajectory_path(self) -> None:
        self._trajectory_path_segments = None
        self._render_trajectory_overlay()

    def set_trajectory_keypoints(
        self,
        points_xyz: list[list[float]],
        selected_index: int | None = None,
        editing_index: int | None = None,
    ) -> None:
        if not points_xyz:
            self._trajectory_keypoint_points = None
        else:
            self._trajectory_keypoint_points = np.array([p[:3] for p in points_xyz], dtype=float)
        self._trajectory_keypoint_selected_index = selected_index
        self._trajectory_keypoint_editing_index = editing_index
        self._render_trajectory_overlay()

    def clear_trajectory_keypoints(self) -> None:
        self._trajectory_keypoint_points = None
        self._trajectory_keypoint_selected_index = None
        self._trajectory_keypoint_editing_index = None
        self._render_trajectory_overlay()

    def set_trajectory_edit_tangents(
        self,
        tangent_out_segments: list[list[list[float]]] | None,
        tangent_in_segments: list[list[list[float]]] | None,
    ) -> None:
        if tangent_out_segments is None:
            self._trajectory_tangent_out_segments = None
        else:
            parsed_out: list[np.ndarray] = []
            for segment in tangent_out_segments:
                if len(segment) < 2:
                    continue
                parsed_out.append(np.array([p[:3] for p in segment], dtype=float))
            self._trajectory_tangent_out_segments = parsed_out if parsed_out else None

        if tangent_in_segments is None:
            self._trajectory_tangent_in_segments = None
        else:
            parsed_in: list[np.ndarray] = []
            for segment in tangent_in_segments:
                if len(segment) < 2:
                    continue
                parsed_in.append(np.array([p[:3] for p in segment], dtype=float))
            self._trajectory_tangent_in_segments = parsed_in if parsed_in else None

        self._render_trajectory_overlay()

    def clear_trajectory_edit_tangents(self) -> None:
        self._trajectory_tangent_out_segments = None
        self._trajectory_tangent_in_segments = None
        self._render_trajectory_overlay()

    def _render_trajectory_overlay(self) -> None:
        for item in self._trajectory_path_items:
            self.viewer.removeItem(item)
        self._trajectory_path_items = []
        if self._trajectory_keypoints_item is not None:
            self.viewer.removeItem(self._trajectory_keypoints_item)
            self._trajectory_keypoints_item = None
        if self._trajectory_keypoint_selected_item is not None:
            self.viewer.removeItem(self._trajectory_keypoint_selected_item)
            self._trajectory_keypoint_selected_item = None
        if self._trajectory_keypoint_editing_item is not None:
            self.viewer.removeItem(self._trajectory_keypoint_editing_item)
            self._trajectory_keypoint_editing_item = None
        for item in self._trajectory_tangent_out_items:
            self.viewer.removeItem(item)
        self._trajectory_tangent_out_items = []
        for item in self._trajectory_tangent_in_items:
            self.viewer.removeItem(item)
        self._trajectory_tangent_in_items = []

        if self._trajectory_path_segments is not None and len(self._trajectory_path_segments) > 0:
            for points_xyz, color in self._trajectory_path_segments:
                world_points = self._transform_robot_points_to_world(points_xyz)
                path_item = gl.GLLinePlotItem(
                    pos=world_points,
                    color=color,
                    width=2,
                    antialias=True,
                )
                self._trajectory_path_items.append(path_item)
                self.viewer.addItem(path_item)

        if self._trajectory_keypoint_points is not None and len(self._trajectory_keypoint_points) > 0:
            points = self._trajectory_keypoint_points
            selected_idx = self._trajectory_keypoint_selected_index
            editing_idx = self._trajectory_keypoint_editing_index
            mask = np.ones(len(points), dtype=bool)
            if selected_idx is not None and 0 <= selected_idx < len(points):
                mask[selected_idx] = False
            if editing_idx is not None and 0 <= editing_idx < len(points):
                mask[editing_idx] = False

            base_points = points[mask]
            if len(base_points) > 0:
                world_base_points = self._transform_robot_points_to_world(base_points)
                self._trajectory_keypoints_item = gl.GLScatterPlotItem(
                    pos=world_base_points,
                    color=(0.95, 0.95, 0.95, 0.9),
                    size=9,
                    pxMode=True,
                )
                self.viewer.addItem(self._trajectory_keypoints_item)

            if selected_idx is not None and 0 <= selected_idx < len(points):
                selected_points = self._transform_robot_points_to_world(np.array([points[selected_idx]], dtype=float))
                self._trajectory_keypoint_selected_item = gl.GLScatterPlotItem(
                    pos=selected_points,
                    color=(0.1, 0.85, 1.0, 1.0),
                    size=13,
                    pxMode=True,
                )
                self.viewer.addItem(self._trajectory_keypoint_selected_item)

            if editing_idx is not None and 0 <= editing_idx < len(points):
                editing_points = self._transform_robot_points_to_world(np.array([points[editing_idx]], dtype=float))
                self._trajectory_keypoint_editing_item = gl.GLScatterPlotItem(
                    pos=editing_points,
                    color=(1.0, 0.35, 0.1, 1.0),
                    size=15,
                    pxMode=True,
                )
                self.viewer.addItem(self._trajectory_keypoint_editing_item)

        if self._trajectory_tangent_out_segments is not None:
            for segment in self._trajectory_tangent_out_segments:
                if len(segment) < 2:
                    continue
                world_segment = self._transform_robot_points_to_world(segment)
                item = gl.GLLinePlotItem(
                    pos=world_segment,
                    color=(1.0, 0.5, 0.1, 0.95),
                    width=2,
                    antialias=True,
                )
                self._trajectory_tangent_out_items.append(item)
                self.viewer.addItem(item)

        if self._trajectory_tangent_in_segments is not None:
            for segment in self._trajectory_tangent_in_segments:
                if len(segment) < 2:
                    continue
                world_segment = self._transform_robot_points_to_world(segment)
                item = gl.GLLinePlotItem(
                    pos=world_segment,
                    color=(0.25, 1.0, 0.55, 0.95),
                    width=2,
                    antialias=True,
                )
                self._trajectory_tangent_in_items.append(item)
                self.viewer.addItem(item)

    def draw_frame(self, T, longueur=100, color: tuple[int, int, int]=None):
        """Dessine un repère unique"""
        origine = T[:3, 3]
        R = T[:3, :3]
        
        axes = [
            np.array([origine, origine + R[:, 0] * longueur]), # X
            np.array([origine, origine + R[:, 1] * longueur]), # Y
            np.array([origine, origine + R[:, 2] * longueur])  # Z
        ]

        if color is None:
            couleurs = [(255, 0, 0, 1), (0, 255, 0, 1), (0, 0, 255, 1)]
        else:
            couleurs = [color, color, color]

        items = []
        for i, axis in enumerate(axes):
            plt = gl.GLLinePlotItem(pos=axis, color=couleurs[i], width=3, antialias=True)
            self.viewer.addItem(plt)
            items.append(plt)
        return items

    def draw_all_frames(self, matrices):
        """Dessine les repères en fonction de leur visibilité individuelle"""
        self._clear_viewer_items(self._robot_frame_items)
        for i, T in enumerate(matrices):
            # On dessine seulement si l'index est marqué visible dans la liste
            if i < len(self.frames_visibility) and self.frames_visibility[i]:
                self._robot_frame_items.extend(self.draw_frame(self._transform_robot_matrix_to_world(T)))
    
    def draw_workspace_frames(self) -> None:
        self._clear_viewer_items(self._workspace_frame_items)
        for i, transform in enumerate(self._workspace_frame_matrices):
            if i < len(self.workspace_frames_visibility) and self.workspace_frames_visibility[i]:
                self._workspace_frame_items.extend(self.draw_frame(transform))

    @staticmethod
    def _normalize_visibility_list(
        values: list[bool],
        count: int,
        default_value: bool = True,
    ) -> list[bool]:
        normalized = [bool(v) for v in values[:count]]
        if len(normalized) < count:
            normalized.extend([bool(default_value)] * (count - len(normalized)))
        return normalized

    def _normalize_workspace_frames_visibility(self) -> None:
        target_count = len(self._workspace_frame_matrices)
        current_values = [bool(v) for v in self.workspace_frames_visibility]

        if len(current_values) == target_count:
            self.workspace_frames_visibility = current_values
            return

        # Compatibilite avec les anciennes sessions qui ne stockaient pas le frame World.
        if target_count > 0 and len(current_values) == (target_count - 1):
            self.workspace_frames_visibility = [True, *current_values]
            return

        self.workspace_frames_visibility = self._normalize_visibility_list(current_values, target_count)

    def load_cad(self, robot_model: RobotModel, tool_model: ToolModel | None = None):
        self._robot_model = robot_model
        self._tool_model = tool_model
        self.begin_loading_feedback("Chargement CAO robot...")
        try:
            matrices = self._resolve_cad_matrices(robot_model, tool_model)
            self.last_corrected_matrices = matrices
            self.add_robot_links(matrices)
            if self.transparency_enabled:
                self.set_transparency(True, emit_signal=False)
            self._cad_loaded = True
            self._clear_and_refresh()
        finally:
            self.end_loading_feedback()

    def reload_tool_cad(self, robot_model: RobotModel, tool_model: ToolModel | None = None):
        self._robot_model = robot_model
        self._tool_model = tool_model
        if not self._cad_loaded:
            self.load_cad(robot_model, tool_model)
            return

        self.begin_loading_feedback("Chargement CAO tool...")
        try:
            matrices = self._resolve_cad_matrices(robot_model, tool_model)
            self.last_corrected_matrices = matrices
            ghost_matrices = self.last_ghost_corrected_matrices if self.last_ghost_corrected_matrices else matrices
            tool_cad_model = self._resolve_tool_cad_model()

            self._replace_tool_link(matrices, tool_cad_model, ghost=False)
            self._replace_tool_link(ghost_matrices, tool_cad_model, ghost=True)

            if self.transparency_enabled:
                self.set_transparency(True, emit_signal=False)
            self._clear_and_refresh()
        finally:
            self.end_loading_feedback()

    def _resolve_cad_matrices(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel | None = None,
    ) -> list[np.ndarray]:
        matrices = robot_model.get_current_tcp_corrected_dh_matrices()
        if matrices:
            return matrices

        if self.last_corrected_matrices:
            return self.last_corrected_matrices

        active_tool = tool_model.get_tool() if tool_model is not None else None
        fk_result = robot_model.compute_fk_joints(robot_model.get_joints(), tool=active_tool)
        if fk_result is None:
            return []

        _, corrected_matrices, _, _, _ = fk_result
        return corrected_matrices

    def load_robot_mesh(self, stl_path: str, transform_matrix, color: tuple[int, int, int]):
        # (Copier le code original ici, pas de changement)
        try:
            if not stl_path:
                return None
            if stl_path in self._missing_mesh_paths:
                if os.path.exists(stl_path):
                    self._missing_mesh_paths.remove(stl_path)
                else:
                    return None
            mesh_data = self._mesh_data_cache.get(stl_path)
            if mesh_data is None:
                stl_mesh = mesh.Mesh.from_file(stl_path)
                verts = stl_mesh.vectors.reshape(-1, 3)
                faces = np.arange(len(verts)).reshape(-1, 3)
                mesh_data = gl.MeshData(vertexes=verts, faces=faces)
                self._mesh_data_cache[stl_path] = mesh_data

            mesh_item = gl.GLMeshItem(meshdata=mesh_data, smooth=True, color=color, shader='shaded')
            T = transform_matrix
            qmat = QtGui.QMatrix4x4(
                T[0,0], T[0,1], T[0,2], T[0,3],
                T[1,0], T[1,1], T[1,2], T[1,3],
                T[2,0], T[2,1], T[2,2], T[2,3],
                T[3,0], T[3,1], T[3,2], T[3,3]
            )
            mesh_item.setTransform(qmat)
            return mesh_item
        except Exception as e:
            self._missing_mesh_paths.add(stl_path)
            print(f"Erreur STL {stl_path}: {e}")
            return None

    def _resolve_robot_cad_models(self) -> list[str]:
        if self._robot_model is None:
            return [f"./default_data/robots_stl/rocky{i}.stl" for i in range(7)]
        cad_models = self._robot_model.get_robot_cad_models()
        if not cad_models:
            return [f"./default_data/robots_stl/rocky{i}.stl" for i in range(7)]
        return [str(path) for path in cad_models]

    def _resolve_tool_cad_model(self) -> str:
        if self._tool_model is None:
            return ""
        return str(self._tool_model.get_tool_cad_model())

    def _resolve_tool_cad_offset_rz(self) -> float:
        if self._tool_model is None:
            return 0.0
        return float(self._tool_model.get_tool_cad_offset_rz())

    @staticmethod
    def _resolve_tool_attachment_matrix_index(matrices) -> int | None:
        # Le dernier repere correspond au TCP (avec tool). La CAO tool doit
        # etre attachee au repere de l'axe 6, donc juste avant le TCP.
        if len(matrices) < 2:
            return None
        return len(matrices) - 2

    @staticmethod
    def _resolve_tool_link_color() -> tuple[float, float, float, float]:
        return (0.70, 0.70, 0.70, 0.5)

    @staticmethod
    def _apply_tool_visual_offset(transform_matrix: np.ndarray, offset_rz_deg: float) -> np.ndarray:
        if abs(offset_rz_deg) < 1e-12:
            return transform_matrix

        # Rotation locale autour de l'axe Z du repere outil.
        return transform_matrix @ math_utils.homogeneous_rotation_z(offset_rz_deg, degrees=True)

    @staticmethod
    def _resolve_link_color(matrix_index: int) -> tuple[float, float, float, float]:
        kuka_orange = (1.0, 0.4, 0.0, 0.5)
        kuka_black = (0.1, 0.1, 0.1, 0.5)
        kuka_grey = (0.5, 0.5, 0.5, 0.5)
        if matrix_index == 0:
            return kuka_black
        if matrix_index == 6:
            return kuka_grey
        return kuka_orange

    def _build_cad_specs(self, matrices) -> list[tuple[int, str, tuple[float, float, float, float], bool]]:
        specs: list[tuple[int, str, tuple[float, float, float, float], bool]] = []
        if not matrices:
            return specs

        robot_cad_models = self._resolve_robot_cad_models()
        robot_matrix_count = min(7, len(matrices))
        for matrix_index in range(robot_matrix_count):
            if matrix_index < len(robot_cad_models):
                stl_path = robot_cad_models[matrix_index]
            else:
                stl_path = f"./default_data/robots_stl/rocky{matrix_index}.stl"

            if not stl_path:
                continue
            specs.append((matrix_index, stl_path, self._resolve_link_color(matrix_index), False))

        tool_cad_model = self._resolve_tool_cad_model()
        tool_matrix_index = self._resolve_tool_attachment_matrix_index(matrices)
        if tool_cad_model and tool_matrix_index is not None:
            specs.append((tool_matrix_index, tool_cad_model, self._resolve_tool_link_color(), True))

        return specs

    def add_robot_links(self, matrices):
        self.clear_robot_links()
        self.clear_robot_ghost_links()

        ghost_color = (0.2, 0.75, 1.0, 0.22)
        tool_offset_rz = self._resolve_tool_cad_offset_rz()
        for matrix_index, stl_path, link_color, is_tool in self._build_cad_specs(matrices):
            T = matrices[matrix_index]
            if is_tool:
                T = self._apply_tool_visual_offset(T, tool_offset_rz)
            mesh_item = self.load_robot_mesh(stl_path, self._transform_robot_matrix_to_world(T), link_color)
            if mesh_item:
                self.robot_links.append(mesh_item)
                self._robot_link_matrix_indices.append(matrix_index)
                self._robot_link_roles.append("tool" if is_tool else "robot")
                self.viewer.addItem(mesh_item)
                if not self._cad_showed:
                    mesh_item.hide()

            ghost_item = self.load_robot_mesh(stl_path, self._transform_robot_matrix_to_world(T), ghost_color)
            if ghost_item:
                ghost_item.setGLOptions('translucent')
                self.robot_ghost_links.append(ghost_item)
                self._robot_ghost_link_matrix_indices.append(matrix_index)
                self._robot_ghost_link_roles.append("tool" if is_tool else "robot")
                self.viewer.addItem(ghost_item)
                if not self._ghost_visible:
                    ghost_item.hide()

    def _replace_tool_link(self, matrices, stl_path: str, ghost: bool) -> None:
        links = self.robot_ghost_links if ghost else self.robot_links
        indices = self._robot_ghost_link_matrix_indices if ghost else self._robot_link_matrix_indices
        roles = self._robot_ghost_link_roles if ghost else self._robot_link_roles

        tool_slot = -1
        for idx, role in enumerate(roles):
            if role == "tool":
                tool_slot = idx
                break

        if tool_slot >= 0:
            old_item = links.pop(tool_slot)
            indices.pop(tool_slot)
            roles.pop(tool_slot)
            self._safe_remove_viewer_item(old_item)

        matrix_index = self._resolve_tool_attachment_matrix_index(matrices)
        if not stl_path or matrix_index is None or matrix_index >= len(matrices):
            return

        color = (0.2, 0.75, 1.0, 0.22) if ghost else self._resolve_tool_link_color()
        base_transform = matrices[matrix_index]
        visual_transform = self._apply_tool_visual_offset(base_transform, self._resolve_tool_cad_offset_rz())
        mesh_item = self.load_robot_mesh(stl_path, self._transform_robot_matrix_to_world(visual_transform), color)
        if mesh_item is None:
            return

        if ghost:
            mesh_item.setGLOptions('translucent')
        elif self.transparency_enabled:
            mesh_item.setGLOptions('translucent')

        links.append(mesh_item)
        indices.append(matrix_index)
        roles.append("tool")
        self.viewer.addItem(mesh_item)

        if ghost and not self._ghost_visible:
            mesh_item.hide()
        if not ghost and not self._cad_showed:
            mesh_item.hide()

    def update_robot(self, robot_model: RobotModel, tool_model: ToolModel | None = None):
        """Met à jour la visualisation 3D avec repères et visibilité des frames"""
        self._robot_model = robot_model
        self._tool_model = tool_model

        dh_matrices = robot_model.get_current_tcp_dh_matrices()
        corrected_matrices = robot_model.get_current_tcp_corrected_dh_matrices()

        if not dh_matrices or not corrected_matrices:
            active_tool = tool_model.get_tool() if tool_model is not None else None
            fk_result = robot_model.compute_fk_joints(
                robot_model.get_joints(),
                tool=active_tool,
            )
            if fk_result is None:
                return
            dh_matrices, corrected_matrices, _, _, _ = fk_result

        self.last_dh_matrices = dh_matrices
        self.last_corrected_matrices = corrected_matrices

        self._refresh_robot_state_items()
        self._refresh_position_buttons()

    def update_workspace(self, workspace_model: WorkspaceModel | None) -> None:
        previous_model = self._workspace_model
        previous_base_revision = self._robot_base_transform_world.revision
        previous_structure_revision = self._workspace_structure_revision

        self._workspace_model = workspace_model
        self._robot_base_transform_world = (
            FrameTransform.from_pose([0.0] * 6)
            if workspace_model is None
            else workspace_model.get_robot_base_transform_world()
        )
        self._workspace_structure_revision = (
            None if workspace_model is None else workspace_model.get_workspace_structure_revision()
        )

        same_workspace = previous_model is workspace_model
        pose_changed = previous_base_revision != self._robot_base_transform_world.revision
        structure_changed = previous_structure_revision != self._workspace_structure_revision
        if same_workspace and pose_changed and not structure_changed:
            self._refresh_robot_state_items()
            return
        if same_workspace and not pose_changed and not structure_changed:
            return

        raw_elements = [] if workspace_model is None else workspace_model.get_workspace_cad_elements()
        self._workspace_elements = [
            WorkspaceElementState.from_dict(element, index)
            for index, element in enumerate(raw_elements)
        ]
        self.begin_loading_feedback("Chargement scene workspace...")
        try:
            self._clear_and_refresh()
        finally:
            self.end_loading_feedback()

    def update_collision_scene(self, collision_scene_model: CollisionSceneModel | None) -> None:
        if collision_scene_model is None:
            self._workspace_tcp_zones = []
            self._workspace_collision_zones = []
            self._robot_colliders = []
            self._tool_colliders = []
            self._render_workspace_zones()
            self._render_robot_axis_colliders()
            self._render_tool_colliders()
            return

        self._workspace_tcp_zones = collision_scene_model.get_workspace_tcp_colliders()
        self._workspace_collision_zones = collision_scene_model.get_workspace_collision_colliders()
        self._robot_colliders = collision_scene_model.get_robot_colliders()
        self._tool_colliders = collision_scene_model.get_tool_colliders()
        self._render_workspace_zones()
        self._render_robot_axis_colliders()
        self._render_tool_colliders()

    @staticmethod
    def _pose_to_matrix(pose: list[float]) -> np.ndarray:
        return math_utils.pose_zyx_to_matrix(pose)

    def _get_robot_base_pose_world(self) -> list[float]:
        return self._robot_base_transform_world.pose_list()

    def _get_robot_base_world_transform(self) -> np.ndarray:
        return self._robot_base_transform_world.matrix.copy()

    def _transform_robot_matrix_to_world(self, transform: np.ndarray) -> np.ndarray:
        return transform_matrix_base_to_world(transform, self._robot_base_transform_world)

    def _transform_robot_points_to_world(self, points_xyz: np.ndarray) -> np.ndarray:
        return transform_points_base_to_world(points_xyz, self._robot_base_transform_world)

    def _render_workspace_models(self) -> None:
        self._clear_viewer_items(self._workspace_element_items)
        self._workspace_element_items.clear()
        self._workspace_frame_matrices = [np.eye(4, dtype=float)]
        self._workspace_frame_labels = ["World"]

        for element in self._workspace_elements:
            transform = element.world_transform
            self._workspace_frame_matrices.append(transform)
            self._workspace_frame_labels.append(element.name)
            if element.cad_model == "":
                continue
            item = self.load_robot_mesh(element.cad_model, transform, (0.65, 0.70, 0.80, 0.45))
            if item is None:
                continue
            item.setGLOptions('translucent')
            self.viewer.addItem(item)
            self._workspace_element_items.append(item)

    def _render_workspace_zones(self) -> None:
        self._clear_viewer_items(self._workspace_tcp_zone_items)
        self._clear_viewer_items(self._workspace_collision_zone_items)

        for zone in self._workspace_tcp_zones:
            item = self._build_primitive_item(zone, (1.0, 0.93, 0.2, 0.22))
            if item is None:
                continue
            self.viewer.addItem(item)
            self._workspace_tcp_zone_items.append(item)
            if not self._workspace_tcp_zones_visible:
                item.hide()

        for zone in self._workspace_collision_zones:
            item = self._build_primitive_item(zone, (1.0, 0.2, 0.2, 0.22))
            if item is None:
                continue
            self.viewer.addItem(item)
            self._workspace_collision_zone_items.append(item)
            if not self._workspace_collision_zones_visible:
                item.hide()

    def _render_robot_axis_colliders(self) -> None:
        self._clear_viewer_items(self._robot_collider_items)
        for collider in self._robot_colliders:
            if not collider.enabled:
                continue
            item = self._build_primitive_item(collider, (0.2, 0.55, 1.0, 0.18))
            if item is None:
                continue
            self.viewer.addItem(item)
            self._robot_collider_items.append(item)
            if not self._robot_colliders_visible:
                item.hide()

    def _render_tool_colliders(self) -> None:
        self._clear_viewer_items(self._tool_collider_items)
        for collider in self._tool_colliders:
            if not collider.enabled:
                continue
            item = self._build_primitive_item(collider, (0.85, 0.35, 1.0, 0.24))
            if item is None:
                continue
            self.viewer.addItem(item)
            self._tool_collider_items.append(item)
            if not self._tool_colliders_visible:
                item.hide()

    def _refresh_robot_state_items(self) -> None:
        num_frames = len(self.last_dh_matrices)
        if len(self.frames_visibility) != num_frames:
            self.frames_visibility = [True] * num_frames

        if self.show_axes:
            self.draw_all_frames(self.last_dh_matrices)
        else:
            self._clear_viewer_items(self._robot_frame_items)

        if self._cad_loaded:
            self.update_robot_poses(self.last_corrected_matrices)

        if self.last_ghost_corrected_matrices:
            self._update_robot_ghost_poses(self.last_ghost_corrected_matrices)
            for mesh_item in self.robot_ghost_links:
                if self._ghost_visible:
                    mesh_item.show()
                else:
                    mesh_item.hide()

        self._render_robot_axis_colliders()
        self._render_tool_colliders()
        self._render_trajectory_overlay()
        self.update_frame_list_ui()

    def _clear_and_refresh(self):
        num_frames = len(self.last_dh_matrices)
        
        # Initialiser la liste de visibilité si nécessaire
        if len(self.frames_visibility) != num_frames:
            self.frames_visibility = [True] * num_frames
        
        # Mettre à jour l'interface de la liste (affichage gras/normal)
        
        # Effacer et redessiner la scène
        self.clear_viewer()
        self._workspace_frame_matrices = []
        self._workspace_frame_labels = []
        
        # Afficher les repères selon la visibilité
        if self.show_axes:
            self.draw_all_frames(self.last_dh_matrices)
        
        # Mettre à jour le CAD si chargé
        if self._cad_loaded:
            self.update_robot_poses(self.last_corrected_matrices)

        # Restaurer le fantome apres clear_viewer()
        if self.last_ghost_corrected_matrices:
            self._update_robot_ghost_poses(self.last_ghost_corrected_matrices)
            if self._ghost_visible:
                for mesh_item in self.robot_ghost_links:
                    mesh_item.show()
            else:
                for mesh_item in self.robot_ghost_links:
                    mesh_item.hide()

        self._render_workspace_models()
        self._normalize_workspace_frames_visibility()
        if self.show_axes:
            self.draw_workspace_frames()
        self._render_workspace_zones()
        self._render_robot_axis_colliders()
        self._render_tool_colliders()
        self._render_trajectory_overlay()
        self.update_frame_list_ui()

    def update_robot_poses(self, matrices):
        tool_offset_rz = self._resolve_tool_cad_offset_rz()
        for mesh_item, matrix_index, role in zip(self.robot_links, self._robot_link_matrix_indices, self._robot_link_roles):
            if matrix_index >= len(matrices):
                continue
            T = matrices[matrix_index]
            if role == "tool":
                T = self._apply_tool_visual_offset(T, tool_offset_rz)
            T = self._transform_robot_matrix_to_world(T)
            if mesh_item:
                mesh_item.resetTransform()
                qmat = QtGui.QMatrix4x4(
                    T[0,0], T[0,1], T[0,2], T[0,3],
                    T[1,0], T[1,1], T[1,2], T[1,3],
                    T[2,0], T[2,1], T[2,2], T[2,3],
                    T[3,0], T[3,1], T[3,2], T[3,3]
                )
                mesh_item.setTransform(qmat)
                self._ensure_viewer_item(mesh_item)

    def _build_primitive_item(
        self,
        primitive: PrimitiveCollider | PrimitiveColliderState | dict,
        color: tuple[float, float, float, float],
        base_transform: np.ndarray | None = None,
        skip_pose: bool = False,
    ) -> gl.GLMeshItem | None:
        if isinstance(primitive, PrimitiveCollider):
            shape = primitive.shape
            size_x = primitive.size_x
            size_y = primitive.size_y
            size_z = primitive.size_z
            radius = primitive.radius
            height = primitive.height
            transform = np.array(primitive.world_transform, dtype=float)
        elif isinstance(primitive, PrimitiveColliderState):
            shape = primitive.shape
            size_x = primitive.size_x
            size_y = primitive.size_y
            size_z = primitive.size_z
            radius = primitive.radius
            height = primitive.height
            primitive_transform = primitive.local_transform
        else:
            shape = str(primitive.get("shape", "box")).strip().lower()
            size_x = primitive.get("size_x", 100.0)
            size_y = primitive.get("size_y", 100.0)
            size_z = primitive.get("size_z", 100.0)
            radius = primitive.get("radius", 50.0)
            height = primitive.get("height", 100.0)
            primitive_transform = self._pose_to_matrix(primitive.get("pose", [0.0] * 6))

        mesh_data = self._build_primitive_mesh_data(shape, size_x, size_y, size_z, radius, height)
        if mesh_data is None:
            return None

        if not isinstance(primitive, PrimitiveCollider):
            transform = np.array(base_transform if base_transform is not None else np.eye(4), dtype=float)
            if not skip_pose:
                transform = transform @ primitive_transform

        item = gl.GLMeshItem(meshdata=mesh_data, smooth=True, color=color, shader='shaded')
        item.setTransform(
            QtGui.QMatrix4x4(
                transform[0, 0], transform[0, 1], transform[0, 2], transform[0, 3],
                transform[1, 0], transform[1, 1], transform[1, 2], transform[1, 3],
                transform[2, 0], transform[2, 1], transform[2, 2], transform[2, 3],
                transform[3, 0], transform[3, 1], transform[3, 2], transform[3, 3],
            )
        )
        item.setGLOptions('translucent')
        return item

    def _build_primitive_mesh_data(
        self,
        shape: str,
        size_x: float,
        size_y: float,
        size_z: float,
        radius: float,
        height: float,
    ) -> gl.MeshData | None:
        normalized_shape = shape if shape in {"box", "cylinder", "sphere"} else "box"
        if normalized_shape == "box":
            sx = max(1e-6, float(size_x))
            sy = max(1e-6, float(size_y))
            sz = max(1e-6, float(size_z))
            key = f"box:{sx:.4f}:{sy:.4f}:{sz:.4f}"
            mesh_data = self._primitive_mesh_cache.get(key)
            if mesh_data is not None:
                return mesh_data

            hx, hy = sx * 0.5, sy * 0.5
            vertices = np.array(
                [
                    [-hx, -hy, 0.0],
                    [hx, -hy, 0.0],
                    [hx, hy, 0.0],
                    [-hx, hy, 0.0],
                    [-hx, -hy, sz],
                    [hx, -hy, sz],
                    [hx, hy, sz],
                    [-hx, hy, sz],
                ],
                dtype=float,
            )
            faces = np.array(
                [
                    [0, 1, 2], [0, 2, 3],  # bottom
                    [4, 7, 6], [4, 6, 5],  # top
                    [0, 4, 5], [0, 5, 1],  # front
                    [1, 5, 6], [1, 6, 2],  # right
                    [2, 6, 7], [2, 7, 3],  # back
                    [3, 7, 4], [3, 4, 0],  # left
                ],
                dtype=int,
            )
            mesh_data = gl.MeshData(vertexes=vertices, faces=faces)
            self._primitive_mesh_cache[key] = mesh_data
            return mesh_data

        if normalized_shape == "cylinder":
            r = max(1e-6, float(radius))
            h = max(1e-6, float(height))
            segments = 24
            key = f"cylinder:{r:.4f}:{h:.4f}:{segments}"
            mesh_data = self._primitive_mesh_cache.get(key)
            if mesh_data is not None:
                return mesh_data

            vertices: list[list[float]] = []
            for idx in range(segments):
                angle = 2.0 * np.pi * float(idx) / float(segments)
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                vertices.append([x, y, 0.0])  # bottom ring
                vertices.append([x, y, h])    # top ring

            top_center_idx = len(vertices)
            vertices.append([0.0, 0.0, h])
            bottom_center_idx = len(vertices)
            vertices.append([0.0, 0.0, 0.0])

            faces: list[list[int]] = []
            for idx in range(segments):
                next_idx = (idx + 1) % segments
                b0 = idx * 2
                t0 = b0 + 1
                b1 = next_idx * 2
                t1 = b1 + 1
                faces.append([b0, b1, t1])
                faces.append([b0, t1, t0])

                faces.append([t0, t1, top_center_idx])
                faces.append([b1, b0, bottom_center_idx])

            mesh_data = gl.MeshData(vertexes=np.array(vertices, dtype=float), faces=np.array(faces, dtype=int))
            self._primitive_mesh_cache[key] = mesh_data
            return mesh_data

        r = max(1e-6, float(radius))
        rows = 12
        cols = 24
        key = f"sphere:{r:.4f}:{rows}:{cols}"
        mesh_data = self._primitive_mesh_cache.get(key)
        if mesh_data is not None:
            return mesh_data

        vertices: list[list[float]] = []
        top_index = 0
        vertices.append([0.0, 0.0, r])

        for row in range(1, rows):
            phi = np.pi * float(row) / float(rows)
            z = r * np.cos(phi)
            xy = r * np.sin(phi)
            for col in range(cols):
                theta = 2.0 * np.pi * float(col) / float(cols)
                x = xy * np.cos(theta)
                y = xy * np.sin(theta)
                vertices.append([x, y, z])

        bottom_index = len(vertices)
        vertices.append([0.0, 0.0, -r])

        faces: list[list[int]] = []
        first_ring_start = 1
        last_ring_start = 1 + (rows - 2) * cols

        for col in range(cols):
            next_col = (col + 1) % cols
            faces.append([top_index, first_ring_start + col, first_ring_start + next_col])

        for row in range(rows - 3):
            ring_start = 1 + row * cols
            next_ring_start = ring_start + cols
            for col in range(cols):
                next_col = (col + 1) % cols
                a = ring_start + col
                b = ring_start + next_col
                c = next_ring_start + col
                d = next_ring_start + next_col
                faces.append([a, c, b])
                faces.append([b, c, d])

        for col in range(cols):
            next_col = (col + 1) % cols
            faces.append([last_ring_start + next_col, last_ring_start + col, bottom_index])

        mesh_data = gl.MeshData(vertexes=np.array(vertices, dtype=float), faces=np.array(faces, dtype=int))
        self._primitive_mesh_cache[key] = mesh_data
        return mesh_data

    @staticmethod
    def _apply_items_visibility(items: list, visible: bool) -> None:
        for item in items:
            if visible:
                item.show()
            else:
                item.hide()

    def _ensure_viewer_item(self, item) -> None:
        if item is None:
            return
        try:
            if item not in self.viewer.items:
                self.viewer.addItem(item)
        except Exception:
            self.viewer.addItem(item)

    def _clear_viewer_items(self, items: list) -> None:
        for item in list(items):
            self._safe_remove_viewer_item(item)
        items.clear()

    def clear_robot_links(self):
        for mesh_item in self.robot_links:
            self._safe_remove_viewer_item(mesh_item)
        self.robot_links.clear()
        self._robot_link_matrix_indices.clear()
        self._robot_link_roles.clear()

    def clear_robot_ghost_links(self):
        for mesh_item in self.robot_ghost_links:
            self._safe_remove_viewer_item(mesh_item)
        self.robot_ghost_links.clear()
        self._robot_ghost_link_matrix_indices.clear()
        self._robot_ghost_link_roles.clear()

    def _safe_remove_viewer_item(self, item):
        if item is None:
            return
        try:
            self.viewer.removeItem(item)
        except ValueError:
            # L'item n'est déjà plus enregistré dans GLViewWidget.items
            pass
        except Exception:
            pass

    def set_robot_visibility(self, visible: bool, emit_signal: bool = True):
        self._cad_showed = visible
        for mesh_item in self.robot_links:
            if visible: mesh_item.show()
            else: mesh_item.hide()
        self._refresh_toolbar_buttons()
        if emit_signal:
            self._emit_display_state_changed()

    def set_transparency(self, enabled: bool, emit_signal: bool = True):
        self.transparency_enabled = enabled
        for mesh_item in self.robot_links:
            mesh_item.setGLOptions('translucent' if enabled else 'opaque')
        self._refresh_toolbar_buttons()
        if emit_signal:
            self._emit_display_state_changed()

    def toogle_base_axis_frames(self):
        self.show_axes = True
        size = len(self.frames_visibility)
        last = size - 1
        self.frames_visibility = [(i == 0 or i == last) for i in range(size)]
        self._clear_and_refresh()
        self._refresh_toolbar_buttons()
        self._emit_display_state_changed()

    def show_robot_ghost(self):
        self._ghost_visible = True

        if not self._cad_loaded:
            matrices = self.last_corrected_matrices
            if not matrices and self._robot_model is not None:
                matrices = self._resolve_cad_matrices(self._robot_model, self._tool_model)
                self.last_corrected_matrices = matrices
            if matrices:
                self.add_robot_links(matrices)
                self._cad_loaded = True

        if self.last_ghost_corrected_matrices:
            self._update_robot_ghost_poses(self.last_ghost_corrected_matrices)
        for mesh_item in self.robot_ghost_links:
            mesh_item.show()

    def hide_robot_ghost(self):
        self._ghost_visible = False
        for mesh_item in self.robot_ghost_links:
            mesh_item.hide()

    def update_robot_ghost(self, joints: list[float]):
        if self._robot_model is None or len(joints) < 6:
            self.hide_robot_ghost()
            return

        active_tool = self._tool_model.get_tool() if self._tool_model is not None else None
        fk_result = self._robot_model.compute_fk_joints(joints, tool=active_tool)
        if fk_result is None:
            self.hide_robot_ghost()
            return

        _, corrected_matrices, _, _, _ = fk_result
        self.update_robot_ghost_from_matrices(corrected_matrices)

    def update_robot_ghost_from_matrices(self, corrected_matrices: list):
        if not corrected_matrices:
            self.hide_robot_ghost()
            return

        self.last_ghost_corrected_matrices = corrected_matrices
        self._update_robot_ghost_poses(corrected_matrices)

        if self._ghost_visible:
            for mesh_item in self.robot_ghost_links:
                mesh_item.show()
        else:
            for mesh_item in self.robot_ghost_links:
                mesh_item.hide()

    def _ensure_robot_ghost_links(self, matrices):
        expected_count = len(self._build_cad_specs(matrices))
        if len(self.robot_links) == expected_count and len(self.robot_ghost_links) == expected_count:
            return

        self.add_robot_links(matrices)
        self._cad_loaded = True

    def _update_robot_ghost_poses(self, matrices):
        self._ensure_robot_ghost_links(matrices)

        tool_offset_rz = self._resolve_tool_cad_offset_rz()
        for mesh_item, matrix_index, role in zip(self.robot_ghost_links, self._robot_ghost_link_matrix_indices, self._robot_ghost_link_roles):
            if matrix_index >= len(matrices):
                continue
            T = matrices[matrix_index]
            if role == "tool":
                T = self._apply_tool_visual_offset(T, tool_offset_rz)
            T = self._transform_robot_matrix_to_world(T)
            if mesh_item:
                mesh_item.resetTransform()
                qmat = QtGui.QMatrix4x4(
                    T[0,0], T[0,1], T[0,2], T[0,3],
                    T[1,0], T[1,1], T[1,2], T[1,3],
                    T[2,0], T[2,1], T[2,2], T[2,3],
                    T[3,0], T[3,1], T[3,2], T[3,3]
                )
                mesh_item.setTransform(qmat)
                self._ensure_viewer_item(mesh_item)

