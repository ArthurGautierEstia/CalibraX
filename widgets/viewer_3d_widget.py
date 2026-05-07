import os
from dataclasses import dataclass
from OpenGL.GL import GL_DEPTH_COMPONENT, GL_FLOAT, glReadPixels
from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QListWidgetItem,
    QListWidget,
    QAbstractItemView,
    QColorDialog,
    QComboBox,
    QSpinBox,
    QInputDialog,
)
from PyQt6.QtCore import Qt, QSize, pyqtSignal, QPoint, QPointF
from PyQt6.QtGui import QBrush, QColor, QFont, QIcon, QPainter, QPen, QPixmap, QLinearGradient, QPolygonF, QRadialGradient, QPalette
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import shaders as gl_shaders
from pyqtgraph.Qt import QtGui
import numpy as np
from stl import mesh

import utils.math_utils as math_utils
from models.app_session_file import ViewerDisplayState, ViewerThemeState
from models.collision_scene_model import CollisionSceneModel
from models.primitive_collider_models import PrimitiveCollider, PrimitiveColliderData
from models.robot_model import RobotModel
from models.reference_frame import ReferenceFrame
from models.types import CadColor, Pose6, XYZ3
from models.tool_model import ToolModel
from models.workspace_cad_element import WorkspaceCadElement
from models.workspace_model import WorkspaceModel
from models.viewer_theme_store import ViewerThemeStore


TangentSegment = tuple[XYZ3, XYZ3]
from widgets.viewer_control_overlay_widget import ViewerControlOverlayWidget
from utils.reference_frame_utils import (
    FrameTransform,
    transform_matrix_base_to_world,
    transform_points_base_to_world,
)


class CalibraXGLViewWidget(gl.GLViewWidget):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._background_mode = "solid"
        self._background_primary_color = QColor(45, 45, 48, 255)
        self._background_secondary_color = QColor(15, 15, 18, 255)
        self._background_gradient_direction = "vertical"
        self._perspective_enabled = True
        self._orthographic_zoom_factor = 1.0
        self._grid_reference = None
        self.setBackgroundColor(self._background_primary_color)

    def mousePressEvent(self, ev) -> None:
        local_position = ev.position() if hasattr(ev, "position") else ev.localPos()
        if ev.button() == Qt.MouseButton.LeftButton:
            self._orbit_pivot_mode = "picked"
            self._orbit_pivot_point_world = self._pick_orbit_world_point(local_position)
            self._origin_orbit_anchor_world = None
            if self._orbit_pivot_point_world is None:
                self._orbit_pivot_point_world = np.array([0.0, 0.0, 0.0], dtype=float)
                self._orbit_pivot_mode = "origin"
                self._origin_orbit_anchor_world = self._project_cursor_to_floor(local_position)
                if self._origin_orbit_anchor_world is None:
                    self._origin_orbit_anchor_world = self._project_cursor_to_center_depth(local_position)
            self._orbit_pivot_screen_position = local_position
            self._orbit_pivot_camera_distance = (
                self._camera_distance_to_point(self._orbit_pivot_point_world)
                if self.is_perspective_enabled()
                else None
            )
        super().mousePressEvent(ev)

    def mouseReleaseEvent(self, ev) -> None:
        if ev.button() == Qt.MouseButton.LeftButton:
            self._orbit_pivot_point_world = None
            self._orbit_pivot_mode = None
            self._origin_orbit_anchor_world = None
            self._orbit_pivot_screen_position = None
            self._orbit_pivot_camera_distance = None
        super().mouseReleaseEvent(ev)

    def mouseMoveEvent(self, ev) -> None:
        local_position = ev.position() if hasattr(ev, "position") else ev.localPos()
        if ev.buttons() & Qt.MouseButton.MiddleButton:
            if not hasattr(self, "mousePos"):
                self.mousePos = local_position
            diff = local_position - self.mousePos
            self.mousePos = local_position
            pan_speed_factor = self._compute_middle_pan_speed_factor(local_position)
            self.pan(
                diff.x() * pan_speed_factor,
                0.0,
                diff.y() * pan_speed_factor,
                relative="view-upright",
            )
            return

        if (
            ev.buttons() & Qt.MouseButton.LeftButton
            and not (ev.modifiers() & Qt.KeyboardModifier.ControlModifier)
            and getattr(self, "_orbit_pivot_mode", None) == "origin"
        ):
            if not hasattr(self, "mousePos"):
                self.mousePos = local_position
            diff = local_position - self.mousePos
            self.mousePos = local_position
            rotation_factor = self._compute_origin_orbit_rotation_factor()
            self._orbit_around_fixed_pivot(
                np.array([0.0, 0.0, 0.0], dtype=float),
                -diff.x() * rotation_factor,
                -diff.y() * rotation_factor,
            )
            return

        super().mouseMoveEvent(ev)
        if not (ev.buttons() & Qt.MouseButton.LeftButton):
            return
        if ev.modifiers() & Qt.KeyboardModifier.ControlModifier:
            return

        orbit_pivot_point_world = getattr(self, "_orbit_pivot_point_world", None)
        orbit_pivot_screen_position = getattr(self, "_orbit_pivot_screen_position", None)
        orbit_pivot_camera_distance = getattr(self, "_orbit_pivot_camera_distance", None)
        orbit_pivot_mode = getattr(self, "_orbit_pivot_mode", None)
        if (
            orbit_pivot_point_world is None
            or orbit_pivot_screen_position is None
            or (orbit_pivot_camera_distance is None and self.is_perspective_enabled() and orbit_pivot_mode == "picked")
        ):
            return

        if orbit_pivot_mode != "origin":
            self._recenter_to_keep_point_under_cursor(
                orbit_pivot_screen_position,
                orbit_pivot_point_world,
                orbit_pivot_camera_distance,
            )

    def wheelEvent(self, ev) -> None:
        local_position = ev.position() if hasattr(ev, "position") else ev.localPos()
        target_point_world = self._pick_world_point(local_position)
        if target_point_world is None:
            target_point_world = self._project_cursor_to_center_depth(local_position)

        delta = ev.angleDelta().x()
        if delta == 0:
            delta = ev.angleDelta().y()

        if ev.modifiers() & Qt.KeyboardModifier.ControlModifier:
            self.opts["fov"] *= 0.999 ** delta
        else:
            if self.is_perspective_enabled():
                self._dolly_along_cursor_ray(local_position, target_point_world, delta)
            else:
                self._zoom_orthographic_toward_cursor(local_position, target_point_world, delta)

        self.update()

    def set_background_style(self, mode: str, primary_color: QColor, secondary_color: QColor, gradient_direction: str = "vertical") -> None:
        self._background_mode = "gradient" if mode == "gradient" else "solid"
        self._background_primary_color = QColor(primary_color)
        self._background_secondary_color = QColor(secondary_color)
        self._background_gradient_direction = gradient_direction if gradient_direction in {"vertical", "horizontal", "diagonal", "radial"} else "vertical"
        if self._background_mode == "solid":
            self.setBackgroundColor(self._background_primary_color)
        self.update()

    def set_perspective_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if self._perspective_enabled == enabled:
            self.update()
            return

        self._perspective_enabled = enabled
        self.update()

    def is_perspective_enabled(self) -> bool:
        return bool(self._perspective_enabled)

    def set_grid_reference(self, grid_item) -> None:
        self._grid_reference = grid_item

    def projectionMatrix(self, region, viewport):
        x0, y0, w, h = viewport
        dist = max(1e-6, float(self.opts["distance"]))
        fov = float(self.opts["fov"])
        near_clip = max(1e-3, dist * 0.001)
        far_clip = max(near_clip + 1.0, dist * 1000.0)

        r = near_clip * np.tan(0.5 * np.radians(fov))
        t = r * h / w

        left = r * ((region[0] - x0) * (2.0 / w) - 1.0)
        right = r * ((region[0] + region[2] - x0) * (2.0 / w) - 1.0)
        bottom = t * ((region[1] - y0) * (2.0 / h) - 1.0)
        top = t * ((region[1] + region[3] - y0) * (2.0 / h) - 1.0)

        tr = QtGui.QMatrix4x4()
        if self._perspective_enabled:
            tr.frustum(left, right, bottom, top, near_clip, far_clip)
        else:
            ortho_scale = (dist * self._orthographic_zoom_factor) / near_clip
            tr.ortho(
                left * ortho_scale,
                right * ortho_scale,
                bottom * ortho_scale,
                top * ortho_scale,
                -far_clip,
                far_clip,
            )
        return tr

    def paintGL(self) -> None:
        region = self.getViewport()
        if self._background_mode != "gradient":
            self.paint(region=region, viewport=region)
            return

        from OpenGL import GL

        GL.glClearColor(0.0, 0.0, 0.0, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

        painter = QPainter(self)
        width = float(max(1, self.width()))
        height = float(max(1, self.height()))
        if self._background_gradient_direction == "horizontal":
            gradient = QLinearGradient(0.0, 0.0, width, 0.0)
        elif self._background_gradient_direction == "diagonal":
            gradient = QLinearGradient(0.0, 0.0, width, height)
        elif self._background_gradient_direction == "radial":
            gradient = QRadialGradient(width * 0.5, height * 0.5, max(width, height) * 0.65)
        else:
            gradient = QLinearGradient(0.0, 0.0, 0.0, height)
        gradient.setColorAt(0.0, self._background_primary_color)
        gradient.setColorAt(1.0, self._background_secondary_color)
        painter.fillRect(self.rect(), gradient)
        painter.end()

        GL.glClear(GL.GL_DEPTH_BUFFER_BIT)
        self.setProjection(region, region)
        self.setModelview()
        self.drawItemTree(useItemNames=False)

    def _pick_world_point(self, local_position) -> np.ndarray | None:
        depth_value = self._read_depth_value(local_position)
        if depth_value is None or depth_value >= 1.0:
            return None
        return self._unproject_view_point(local_position, depth_value)

    def _pick_orbit_world_point(self, local_position) -> np.ndarray | None:
        picked_world_point = self._pick_world_point(local_position)
        if picked_world_point is None:
            return None

        if self._is_world_point_on_grid_plane(picked_world_point):
            return None
        return picked_world_point

    def _get_items_at(self, local_position) -> list:
        try:
            return list(self.itemsAt(region=(int(local_position.x()), int(local_position.y()), 1, 1)))
        except Exception:
            return []

    @staticmethod
    def _belongs_to_grid(item, grid_item) -> bool:
        current_item = item
        while current_item is not None:
            if current_item is grid_item:
                return True
            parent_getter = getattr(current_item, "parentItem", None)
            current_item = parent_getter() if callable(parent_getter) else None
        return False

    def _has_non_grid_item_in_items(self, items: list, grid_item) -> bool:
        for item in items:
            if not self._belongs_to_grid(item, grid_item):
                return True
        return False

    @staticmethod
    def _is_world_point_on_grid_plane(world_point: np.ndarray) -> bool:
        return abs(float(world_point[2])) <= 1.0

    def _pick_world_point_without_grid(self, local_position) -> np.ndarray | None:
        grid_item = self._grid_reference
        if grid_item is None:
            return None

        viewport_width, viewport_height = self._device_viewport_size()
        screen_x = float(local_position.x()) * self.devicePixelRatioF()
        screen_y = float(local_position.y()) * self.devicePixelRatioF()
        pixel_x = int(round(screen_x))
        pixel_y = int(round((viewport_height - 1) - screen_y))
        if pixel_x < 0 or pixel_x >= viewport_width or pixel_y < 0 or pixel_y >= viewport_height:
            return None

        grid_item.hide()
        depth_value: float | None = None
        try:
            self.makeCurrent()
            try:
                region = (0, 0, viewport_width, viewport_height)
                self.paint(region=region, viewport=region)
                depth_data = glReadPixels(pixel_x, pixel_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT)
            finally:
                self.doneCurrent()
            depth_array = np.asarray(depth_data, dtype=float).reshape(-1)
            if depth_array.size > 0:
                depth_value = float(depth_array[0])
        finally:
            grid_item.show()

        if depth_value is None or depth_value >= 1.0:
            return None
        return self._unproject_view_point(local_position, depth_value)

    def _recenter_to_keep_point_under_cursor(
        self,
        local_position,
        target_point_world: np.ndarray,
        camera_distance_to_target: float | None = None,
    ) -> None:
        ray = self._view_ray(local_position)
        if ray is None:
            return

        camera_origin_world, ray_direction_world = ray
        translation_world = self._compute_ray_translation(
            camera_origin_world,
            ray_direction_world,
            target_point_world,
            camera_distance_to_target,
        )
        self._translate_center(translation_world)

    def _read_depth_value(self, local_position) -> float | None:
        viewport_width, viewport_height = self._device_viewport_size()
        screen_x = float(local_position.x()) * self.devicePixelRatioF()
        screen_y = float(local_position.y()) * self.devicePixelRatioF()
        pixel_x = int(round(screen_x))
        pixel_y = int(round((viewport_height - 1) - screen_y))

        if pixel_x < 0 or pixel_x >= viewport_width or pixel_y < 0 or pixel_y >= viewport_height:
            return None

        self.makeCurrent()
        try:
            depth_data = glReadPixels(pixel_x, pixel_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT)
        except Exception:
            return None
        finally:
            self.doneCurrent()

        depth_array = np.asarray(depth_data, dtype=float).reshape(-1)
        if depth_array.size == 0:
            return None
        return float(depth_array[0])

    def _view_ray(self, local_position) -> tuple[np.ndarray, np.ndarray] | None:
        near_point_world = self._unproject_view_point(local_position, 0.0)
        far_point_world = self._unproject_view_point(local_position, 1.0)
        if near_point_world is None or far_point_world is None:
            return None

        if self.is_perspective_enabled():
            camera_position = self.cameraPosition()
            ray_origin_world = np.array(
                [
                    float(camera_position.x()),
                    float(camera_position.y()),
                    float(camera_position.z()),
                ],
                dtype=float,
            )
            ray_direction_world = far_point_world - ray_origin_world
        else:
            ray_origin_world = near_point_world
            ray_direction_world = far_point_world - near_point_world

        ray_length = float(np.linalg.norm(ray_direction_world))
        if ray_length <= 1e-9:
            return None

        return ray_origin_world, ray_direction_world / ray_length

    def _project_cursor_to_center_depth(self, local_position) -> np.ndarray | None:
        ray = self._view_ray(local_position)
        if ray is None:
            return None

        camera_origin_world, ray_direction_world = ray
        camera_position = self.cameraPosition()
        camera_position_world = np.array(
            [
                float(camera_position.x()),
                float(camera_position.y()),
                float(camera_position.z()),
            ],
            dtype=float,
        )
        current_center = self.opts["center"]
        center_world = np.array(
            [
                float(current_center.x()),
                float(current_center.y()),
                float(current_center.z()),
            ],
            dtype=float,
        )
        camera_forward_world = center_world - camera_position_world
        forward_norm = float(np.linalg.norm(camera_forward_world))
        if forward_norm <= 1e-9:
            return None

        plane_normal_world = camera_forward_world / forward_norm
        return self._intersect_ray_with_plane(
            camera_origin_world,
            ray_direction_world,
            center_world,
            plane_normal_world,
        )

    def _project_cursor_to_floor(self, local_position) -> np.ndarray | None:
        ray = self._view_ray(local_position)
        if ray is None:
            return None

        camera_origin_world, ray_direction_world = ray
        return self._intersect_ray_with_plane(
            camera_origin_world,
            ray_direction_world,
            np.array([0.0, 0.0, 0.0], dtype=float),
            np.array([0.0, 0.0, 1.0], dtype=float),
        )

    def _compute_origin_orbit_rotation_factor(self) -> float:
        anchor_world = getattr(self, "_origin_orbit_anchor_world", None)
        if anchor_world is None:
            return 0.45

        lever_arm_distance = float(np.linalg.norm(np.array(anchor_world, dtype=float)))
        if lever_arm_distance <= 1e-6:
            return 0.45

        reference_distance = max(150.0, float(self.opts["distance"]) * 0.5)
        distance_ratio = reference_distance / lever_arm_distance
        return float(np.clip(0.45 * distance_ratio, 0.05, 0.42))

    def _dolly_along_cursor_ray(self, local_position, target_point_world: np.ndarray | None, wheel_delta: int) -> None:
        if wheel_delta == 0:
            return

        ray = self._view_ray(local_position)
        if ray is None:
            return

        camera_origin_world, ray_direction_world = ray
        distance_to_target = self._camera_distance_to_point(target_point_world)
        translation_distance = self._compute_dolly_translation_distance(distance_to_target, wheel_delta)
        if abs(translation_distance) <= 1e-9:
            return

        translation_world = ray_direction_world * translation_distance
        self._translate_center(translation_world)

    def _zoom_orthographic_toward_cursor(
        self,
        local_position,
        target_point_world: np.ndarray | None,
        wheel_delta: int,
    ) -> None:
        if wheel_delta == 0:
            return

        zoom_factor = 0.999 ** wheel_delta
        self._orthographic_zoom_factor = max(1e-3, float(self._orthographic_zoom_factor) * zoom_factor)
        if target_point_world is not None:
            self._recenter_to_keep_point_under_cursor(local_position, target_point_world)

    def _compute_middle_pan_speed_factor(self, local_position) -> float:
        target_point_world = self._pick_world_point(local_position)
        if target_point_world is None:
            target_point_world = np.array([0.0, 0.0, 0.0], dtype=float)
        distance = max(1.0, float(self._camera_distance_to_point(target_point_world) or 1.0))
        if not self.is_perspective_enabled():
            return float(np.clip((distance / 2000.0) ** -0.15, 0.18, 1.15))
        return float(np.clip((distance / 2000.0) ** 0.85, 0.2, 6.0))

    def _orbit_around_fixed_pivot(
        self,
        pivot_point_world: np.ndarray,
        azimuth_delta_deg: float,
        elevation_delta_deg: float,
    ) -> None:
        camera_position = self.cameraPosition()
        camera_position_world = np.array(
            [
                float(camera_position.x()),
                float(camera_position.y()),
                float(camera_position.z()),
            ],
            dtype=float,
        )
        current_center = self.opts["center"]
        center_world = np.array(
            [
                float(current_center.x()),
                float(current_center.y()),
                float(current_center.z()),
            ],
            dtype=float,
        )

        rotated_camera_world = self._rotate_point_around_axis(
            camera_position_world,
            pivot_point_world,
            np.array([0.0, 0.0, 1.0], dtype=float),
            azimuth_delta_deg,
        )
        rotated_center_world = self._rotate_point_around_axis(
            center_world,
            pivot_point_world,
            np.array([0.0, 0.0, 1.0], dtype=float),
            azimuth_delta_deg,
        )

        view_direction_world = rotated_center_world - rotated_camera_world
        right_axis_world = np.cross(view_direction_world, np.array([0.0, 0.0, 1.0], dtype=float))
        right_axis_norm = float(np.linalg.norm(right_axis_world))
        if right_axis_norm > 1e-9:
            right_axis_world = right_axis_world / right_axis_norm
            rotated_camera_world = self._rotate_point_around_axis(
                rotated_camera_world,
                pivot_point_world,
                right_axis_world,
                elevation_delta_deg,
            )
            rotated_center_world = self._rotate_point_around_axis(
                rotated_center_world,
                pivot_point_world,
                right_axis_world,
                elevation_delta_deg,
            )

        camera_to_center_world = rotated_camera_world - rotated_center_world
        distance = float(np.linalg.norm(camera_to_center_world))
        if distance <= 1e-9:
            return

        azimuth_deg = float(np.degrees(np.arctan2(camera_to_center_world[1], camera_to_center_world[0])))
        elevation_deg = float(
            np.degrees(np.arcsin(np.clip(camera_to_center_world[2] / distance, -1.0, 1.0)))
        )
        self.setCameraPosition(
            pos=QtGui.QVector3D(
                float(rotated_center_world[0]),
                float(rotated_center_world[1]),
                float(rotated_center_world[2]),
            ),
            distance=distance,
            elevation=elevation_deg,
            azimuth=azimuth_deg,
        )

    def _unproject_view_point(self, local_position, depth_value: float) -> np.ndarray | None:
        viewport_width, viewport_height = self._device_viewport_size()
        if viewport_width <= 0 or viewport_height <= 0:
            return None

        screen_x = float(local_position.x()) * self.devicePixelRatioF()
        screen_y = float(local_position.y()) * self.devicePixelRatioF()
        x_ndc = (2.0 * screen_x / float(viewport_width)) - 1.0
        y_ndc = 1.0 - (2.0 * screen_y / float(viewport_height))
        z_ndc = (2.0 * float(depth_value)) - 1.0

        projection_matrix = self.projectionMatrix(
            region=(0, 0, viewport_width, viewport_height),
            viewport=(0, 0, viewport_width, viewport_height),
        )
        inverted_matrix, invertible = (projection_matrix * self.viewMatrix()).inverted()
        if not invertible:
            return None

        clip_position = QtGui.QVector4D(x_ndc, y_ndc, z_ndc, 1.0)
        world_position = inverted_matrix.map(clip_position)
        if abs(world_position.w()) <= 1e-9:
            return None

        w = float(world_position.w())
        return np.array(
            [
                float(world_position.x()) / w,
                float(world_position.y()) / w,
                float(world_position.z()) / w,
            ],
            dtype=float,
        )

    def _device_viewport_size(self) -> tuple[int, int]:
        pixel_ratio = self.devicePixelRatioF()
        viewport_width = max(1, int(round(float(self.width()) * pixel_ratio)))
        viewport_height = max(1, int(round(float(self.height()) * pixel_ratio)))
        return viewport_width, viewport_height

    def _translate_center(self, translation_world: np.ndarray) -> None:
        current_center = self.opts["center"]
        self.opts["center"] = QtGui.QVector3D(
            float(current_center.x()) + float(translation_world[0]),
            float(current_center.y()) + float(translation_world[1]),
            float(current_center.z()) + float(translation_world[2]),
        )

    @staticmethod
    def _compute_ray_translation(
        camera_origin_world: np.ndarray,
        ray_direction_world: np.ndarray,
        target_point_world: np.ndarray,
        camera_distance_to_target: float | None = None,
    ) -> np.ndarray:
        point_offset_world = target_point_world - camera_origin_world
        distance_along_ray = float(np.dot(point_offset_world, ray_direction_world))
        translation_world = point_offset_world - (distance_along_ray * ray_direction_world)
        if camera_distance_to_target is None:
            return translation_world

        desired_distance = max(1e-9, float(camera_distance_to_target))
        translation_world = translation_world + (
            ray_direction_world * (distance_along_ray - desired_distance)
        )
        return translation_world

    @staticmethod
    def _intersect_ray_with_plane(
        ray_origin_world: np.ndarray,
        ray_direction_world: np.ndarray,
        plane_point_world: np.ndarray,
        plane_normal_world: np.ndarray,
    ) -> np.ndarray | None:
        denominator = float(np.dot(ray_direction_world, plane_normal_world))
        if abs(denominator) <= 1e-9:
            return None

        distance_along_ray = float(
            np.dot(plane_point_world - ray_origin_world, plane_normal_world) / denominator
        )
        if distance_along_ray <= 1e-9:
            return None

        return ray_origin_world + (ray_direction_world * distance_along_ray)

    def _camera_distance_to_point(self, point_world: np.ndarray | None) -> float | None:
        if point_world is None:
            return None

        camera_position = self.cameraPosition()
        camera_position_world = np.array(
            [
                float(camera_position.x()),
                float(camera_position.y()),
                float(camera_position.z()),
            ],
            dtype=float,
        )
        return float(np.linalg.norm(np.array(point_world, dtype=float) - camera_position_world))

    @staticmethod
    def _rotate_point_around_axis(
        point_world: np.ndarray,
        pivot_point_world: np.ndarray,
        axis_world: np.ndarray,
        angle_deg: float,
    ) -> np.ndarray:
        axis_norm = float(np.linalg.norm(axis_world))
        if axis_norm <= 1e-9 or abs(angle_deg) <= 1e-9:
            return np.array(point_world, dtype=float)

        axis_unit_world = axis_world / axis_norm
        angle_rad = float(np.radians(angle_deg))
        relative_point_world = np.array(point_world, dtype=float) - np.array(pivot_point_world, dtype=float)
        cos_angle = float(np.cos(angle_rad))
        sin_angle = float(np.sin(angle_rad))
        rotated_relative_point_world = (
            (relative_point_world * cos_angle)
            + (np.cross(axis_unit_world, relative_point_world) * sin_angle)
            + (axis_unit_world * np.dot(axis_unit_world, relative_point_world) * (1.0 - cos_angle))
        )
        return np.array(pivot_point_world, dtype=float) + rotated_relative_point_world

    @staticmethod
    def _compute_dolly_translation_distance(
        distance_to_target: float | None,
        wheel_delta: int,
    ) -> float:
        if wheel_delta == 0:
            return 0.0

        zoom_direction = 1.0 if wheel_delta > 0 else -1.0
        if distance_to_target is None:
            return zoom_direction * max(1.0, abs(float(wheel_delta)) * 0.25)

        safe_distance = max(1e-6, float(distance_to_target))
        if zoom_direction > 0.0:
            proportional_step = safe_distance * (1.0 - (0.998 ** abs(int(wheel_delta))))
            step_distance = max(1.0, proportional_step)
            max_forward_step = max(0.0, safe_distance - 1e-4)
            return min(step_distance, max_forward_step)

        proportional_step = safe_distance * (1.0 - (0.998 ** abs(int(wheel_delta))))
        step_distance = max(1.0, proportional_step)
        return -step_distance

@dataclass(frozen=True)
class WorkspaceElementState:
    name: str
    cad_model: str
    pose: tuple[float, float, float, float, float, float]
    world_transform: np.ndarray
    revision: int

    @classmethod
    def from_element(cls, value: WorkspaceCadElement, index: int) -> "WorkspaceElementState":
        name = value.name if value.name != "" else f"Element {index + 1}"
        cad_model = value.cad_model
        pose = value.pose.to_tuple()
        transform = math_utils.pose_zyx_to_matrix(value.pose)
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
    def from_collider_data(cls, value: PrimitiveColliderData, index: int) -> "PrimitiveColliderState":
        name = value.name if value.name != "" else f"Zone {index + 1}"
        shape = value.shape.value
        pose = value.pose.to_tuple()
        size_x = max(0.0, float(value.size_x))
        size_y = max(0.0, float(value.size_y))
        size_z = max(0.0, float(value.size_z))
        radius = max(0.0, float(value.radius))
        height = max(0.0, float(value.height))
        local_transform = math_utils.pose_zyx_to_matrix(value.pose)
        local_transform.setflags(write=False)
        shape_key = (shape, size_x, size_y, size_z, radius, height)
        revision = hash((name, value.enabled, shape_key, pose))
        return cls(
            name=name,
            enabled=value.enabled,
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
    CAD_SHADER_NAME = "calibrax_bright_shaded"
    DEFAULT_BACKGROUND_MODE = "solid"
    DEFAULT_BACKGROUND_PRIMARY_COLOR = QColor(45, 45, 48, 255)
    DEFAULT_BACKGROUND_SECONDARY_COLOR = QColor(15, 15, 18, 255)
    DEFAULT_GRID_SIZE = 4000
    DEFAULT_GRID_SPACING = 200
    DEFAULT_GRID_COLOR = QColor(150, 150, 150, 100)
    DEFAULT_TEXT_COLOR = QColor(230, 230, 230, 255)
    ACTIVE_ICON_COLOR = QColor("#ff8c00")

    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        self._ensure_custom_shaders_registered()
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
        self._robot_base_transform_world = FrameTransform.from_pose(Pose6.zeros())
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
        self._viewer_background_mode = self.DEFAULT_BACKGROUND_MODE
        self._viewer_background_primary_color = QColor(self.DEFAULT_BACKGROUND_PRIMARY_COLOR)
        self._viewer_background_secondary_color = QColor(self.DEFAULT_BACKGROUND_SECONDARY_COLOR)
        self._viewer_background_gradient_direction = "vertical"
        self._viewer_text_color = QColor(self.DEFAULT_TEXT_COLOR)
        self._viewer_accent_color = QColor(self.ACTIVE_ICON_COLOR)
        self._grid_size = self.DEFAULT_GRID_SIZE
        self._grid_spacing = self.DEFAULT_GRID_SPACING
        self._grid_color = QColor(self.DEFAULT_GRID_COLOR)
        self._grid_item: gl.GLGridItem | None = None
        self._project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        self._viewer_theme_store = ViewerThemeStore(self._project_root)
        self._selected_viewer_theme_name = ""
        self._default_viewer_theme_name = self._viewer_theme_store.load_default_theme_name()
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Viewer 3D
        self.viewer = CalibraXGLViewWidget()
        self.viewer.opts['glOptions'] = 'translucent'
        self.viewer.opts['depth'] = True
        self.viewer.setCameraPosition(distance=2000, elevation=40, azimuth=45)
        #self.viewer.setMinimumSize(900, 400)
        self.viewer.set_background_style(
            self._viewer_background_mode,
            self._viewer_background_primary_color,
            self._viewer_background_secondary_color,
            self._viewer_background_gradient_direction,
        )
        layout.addWidget(self.viewer)

        # --- LISTE DES REPERES (Overlay ancré au bouton de liste) ---
        self.frame_lists_overlay = QWidget(self.viewer)
        self.frame_lists_overlay.setObjectName("viewerFrameListsOverlay")
        self.frame_lists_overlay.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.frame_lists_overlay.setStyleSheet("""
            QWidget#viewerFrameListsOverlay {
                background-color: rgba(0, 0, 0, 18);
                border: 1px solid rgba(255, 255, 255, 20);
                border-radius: 10px;
            }
            QWidget#viewerFrameListsOverlay QLabel {
                color: rgba(230, 230, 230, 210);
                font-size: 10px;
                font-weight: 600;
            }
            QWidget#viewerFrameListsOverlay QListWidget {
                background-color: rgba(25, 25, 28, 130);
                color: lightgray;
                border: 1px solid rgba(255, 255, 255, 35);
                border-radius: 6px;
                outline: 0;
                font-size: 10px;
            }
            QWidget#viewerFrameListsOverlay QListWidget::item {
                padding: 3px 4px;
            }
        """)
        frame_lists_layout = QHBoxLayout(self.frame_lists_overlay)
        frame_lists_layout.setContentsMargins(6, 6, 6, 6)
        frame_lists_layout.setSpacing(6)
        frame_lists_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        frame_column_width = 126
        self.robot_frame_list_label = QLabel("Robot", self.frame_lists_overlay)
        self.frame_list = QListWidget(self.frame_lists_overlay)
        self.scene_frame_list_label = QLabel("Scene", self.frame_lists_overlay)
        self.workspace_frame_list = QListWidget(self.frame_lists_overlay)
        for list_widget in (self.frame_list, self.workspace_frame_list):
            list_widget.setFixedWidth(frame_column_width)
            list_widget.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
            list_widget.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.robot_frame_column = QWidget(self.frame_lists_overlay)
        self.robot_frame_column.setFixedWidth(frame_column_width)
        robot_column = QVBoxLayout(self.robot_frame_column)
        robot_column.setContentsMargins(0, 0, 0, 0)
        robot_column.setSpacing(4)
        robot_column.setAlignment(Qt.AlignmentFlag.AlignTop)
        robot_column.addWidget(self.robot_frame_list_label)
        robot_column.addWidget(self.frame_list)
        self.scene_frame_column = QWidget(self.frame_lists_overlay)
        self.scene_frame_column.setFixedWidth(frame_column_width)
        scene_column = QVBoxLayout(self.scene_frame_column)
        scene_column.setContentsMargins(0, 0, 0, 0)
        scene_column.setSpacing(4)
        scene_column.setAlignment(Qt.AlignmentFlag.AlignTop)
        scene_column.addWidget(self.scene_frame_list_label)
        scene_column.addWidget(self.workspace_frame_list)
        frame_lists_layout.addWidget(self.robot_frame_column, 0, Qt.AlignmentFlag.AlignTop)
        frame_lists_layout.addWidget(self.scene_frame_column, 0, Qt.AlignmentFlag.AlignTop)
        self.frame_lists_overlay.hide()
        self.viewer_style_overlay = QWidget(self.viewer)
        self.viewer_style_overlay.setObjectName("viewerStyleOverlay")
        self.viewer_style_overlay.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.viewer_style_overlay.setStyleSheet("""
            QWidget#viewerStyleOverlay {
                background-color: rgba(0, 0, 0, 18);
                border: 1px solid rgba(255, 255, 255, 20);
                border-radius: 10px;
            }
            QWidget#viewerStyleOverlay QLabel {
                color: rgba(230, 230, 230, 210);
                font-size: 10px;
                font-weight: 600;
            }
            QWidget#viewerStyleOverlay QComboBox,
            QWidget#viewerStyleOverlay QSpinBox {
                background-color: rgba(25, 25, 28, 130);
                color: lightgray;
                border: 1px solid rgba(255, 255, 255, 35);
                border-radius: 6px;
                padding: 3px 6px;
                min-height: 24px;
            }
        """)
        viewer_style_layout = QVBoxLayout(self.viewer_style_overlay)
        viewer_style_layout.setContentsMargins(8, 8, 8, 8)
        viewer_style_layout.setSpacing(6)
        viewer_style_layout.addWidget(QLabel("Viewer", self.viewer_style_overlay))
        self.background_mode_combo = QComboBox(self.viewer_style_overlay)
        self.background_mode_combo.addItem("Solid", userData="solid")
        self.background_mode_combo.addItem("Gradient", userData="gradient")
        viewer_style_layout.addWidget(self._create_style_row("Fond", self.background_mode_combo))
        self.btn_background_primary_color = self._create_color_picker_button("Couleur fond principale")
        self.background_primary_color_row = self._create_style_row("Couleur 1", self.btn_background_primary_color)
        viewer_style_layout.addWidget(self.background_primary_color_row)
        self.btn_background_secondary_color = self._create_color_picker_button("Couleur fond secondaire")
        self.background_secondary_color_row = self._create_style_row("Couleur 2", self.btn_background_secondary_color)
        viewer_style_layout.addWidget(self.background_secondary_color_row)
        self.background_gradient_direction_combo = QComboBox(self.viewer_style_overlay)
        self.background_gradient_direction_combo.addItem("Vertical", userData="vertical")
        self.background_gradient_direction_combo.addItem("Horizontal", userData="horizontal")
        self.background_gradient_direction_combo.addItem("Diagonal", userData="diagonal")
        self.background_gradient_direction_combo.addItem("Radial", userData="radial")
        self.background_gradient_direction_row = self._create_style_row("Direction", self.background_gradient_direction_combo)
        viewer_style_layout.addWidget(self.background_gradient_direction_row)
        self.btn_swap_background_colors = QPushButton("Inverser", self.viewer_style_overlay)
        self.btn_swap_background_colors.setCursor(Qt.CursorShape.PointingHandCursor)
        self.background_swap_colors_row = self._create_style_row("", self.btn_swap_background_colors)
        viewer_style_layout.addWidget(self.background_swap_colors_row)
        self.btn_text_color = self._create_color_picker_button("Couleur du texte")
        self.text_color_row = self._create_style_row("Texte", self.btn_text_color)
        viewer_style_layout.addWidget(self.text_color_row)
        self.btn_accent_color = self._create_color_picker_button("Couleur d'accent")
        self.accent_color_row = self._create_style_row("Accent", self.btn_accent_color)
        viewer_style_layout.addWidget(self.accent_color_row)
        self.grid_size_spin = QSpinBox(self.viewer_style_overlay)
        self.grid_size_spin.setRange(200, 20000)
        self.grid_size_spin.setSingleStep(100)
        viewer_style_layout.addWidget(self._create_style_row("Taille grille", self.grid_size_spin))
        self.grid_spacing_spin = QSpinBox(self.viewer_style_overlay)
        self.grid_spacing_spin.setRange(10, 5000)
        self.grid_spacing_spin.setSingleStep(10)
        viewer_style_layout.addWidget(self._create_style_row("Pas grille", self.grid_spacing_spin))
        self.btn_grid_color = self._create_color_picker_button("Couleur grille")
        viewer_style_layout.addWidget(self._create_style_row("Grille couleur", self.btn_grid_color))
        self.btn_save_viewer_theme = QPushButton("Enregistrer", self.viewer_style_overlay)
        self.btn_select_viewer_theme = QPushButton("S\u00e9lectionner", self.viewer_style_overlay)
        self.btn_save_default_viewer_style = QPushButton("D\u00e9finir d\u00e9faut", self.viewer_style_overlay)
        self.btn_save_viewer_theme.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_select_viewer_theme.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_save_default_viewer_style.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_save_viewer_theme.setStyleSheet("""
            QPushButton {
                background-color: rgba(255, 255, 255, 14);
                color: rgba(235, 235, 235, 220);
                border: 1px solid rgba(255, 255, 255, 24);
                border-radius: 6px;
                padding: 5px 8px;
            }
            QPushButton:hover {
                background-color: rgba(255, 255, 255, 22);
            }
        """)
        self.btn_swap_background_colors.setStyleSheet(self.btn_save_viewer_theme.styleSheet())
        self.btn_select_viewer_theme.setStyleSheet(self.btn_save_viewer_theme.styleSheet())
        self.btn_save_default_viewer_style.setStyleSheet(self.btn_save_viewer_theme.styleSheet())
        viewer_style_layout.addWidget(self.btn_save_viewer_theme)
        viewer_style_layout.addWidget(self.btn_select_viewer_theme)
        viewer_style_layout.addWidget(self.btn_save_default_viewer_style)
        self.viewer_style_overlay.hide()
        self.viewer_presets_overlay = QWidget(self.viewer)
        self.viewer_presets_overlay.setObjectName("viewerPresetsOverlay")
        self.viewer_presets_overlay.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.viewer_presets_overlay.setStyleSheet("""
            QWidget#viewerPresetsOverlay {
                background-color: rgba(0, 0, 0, 18);
                border: 1px solid rgba(255, 255, 255, 20);
                border-radius: 10px;
            }
            QWidget#viewerPresetsOverlay QLabel {
                color: rgba(230, 230, 230, 210);
                font-size: 10px;
                font-weight: 600;
            }
        """)
        presets_layout = QVBoxLayout(self.viewer_presets_overlay)
        presets_layout.setContentsMargins(8, 8, 8, 8)
        presets_layout.setSpacing(6)
        presets_row = QHBoxLayout()
        presets_row.setContentsMargins(0, 0, 0, 0)
        presets_row.setSpacing(6)
        self.viewer_presets_overlay.hide()
        self.viewer_control_overlay = ViewerControlOverlayWidget(self.viewer)
        self.position_buttons_overlay = QWidget(self.viewer)
        self.position_buttons_overlay.setObjectName("viewerPositionOverlay")
        self.position_buttons_overlay.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.position_buttons_overlay.setStyleSheet("QWidget#viewerPositionOverlay { background-color: transparent; border: none; }")

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
        toolbar_layout.setSpacing(8)

        self.btn_toggle_cad = self._create_overlay_button("Affichage CAD", "cad")
        self.btn_toggle_transparency = self._create_overlay_button("Transparence", "transparency")
        self.btn_toggle_robot_controls = self._create_overlay_button("Contrôles robot", "robot_controls")
        self.btn_toggle_frame_lists = self._create_overlay_button("Liste de repères", "frame_list")
        self.btn_toggle_axes = self._create_overlay_button("Afficher / Masquer tous les repères", "axes")
        self.btn_toggle_viewer_style = self._create_overlay_button("Style viewer", "appearance")
        self.btn_toggle_view_presets = self._create_overlay_button("Vues prédéfinies", "view_cube")
        self.btn_toggle_perspective = self._create_overlay_button("Perspective", "perspective")
        self.btn_toggle_workspace_tcp_zones = self._create_overlay_button("Zone de travail", "tcp_zones")
        self.btn_toggle_workspace_collision_zones = self._create_overlay_button("Zone de collision", "collision_zones")
        self.btn_toggle_robot_colliders = self._create_overlay_button("Colliders robot", "robot_colliders")
        self.btn_toggle_tool_colliders = self._create_overlay_button("Colliders tool", "tool_colliders")

        for zone_widget in (
            self._create_toolbar_zone(
                "Robot",
                (self.btn_toggle_cad, self.btn_toggle_transparency, self.btn_toggle_robot_controls),
            ),
            self._create_toolbar_zone(
                "Repères",
                (self.btn_toggle_axes, self.btn_toggle_frame_lists),
            ),
            self._create_toolbar_zone(
                "Vue",
                (self.btn_toggle_viewer_style, self.btn_toggle_view_presets, self.btn_toggle_perspective),
            ),
            self._create_toolbar_zone(
                "Zones",
                (self.btn_toggle_workspace_tcp_zones, self.btn_toggle_workspace_collision_zones),
            ),
            self._create_toolbar_zone(
                "Colliders",
                (self.btn_toggle_robot_colliders, self.btn_toggle_tool_colliders),
            ),
        ):
            toolbar_layout.addWidget(zone_widget)
        toolbar_layout.addStretch(1)
        self.toolbar_overlay.adjustSize()

        position_layout = QVBoxLayout(self.position_buttons_overlay)
        position_layout.setContentsMargins(0, 0, 0, 0)
        position_layout.setSpacing(6)
        self.btn_go_position_calibration_overlay = self._create_overlay_button(
            "Position de calibration",
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
            position_layout.setAlignment(button, Qt.AlignmentFlag.AlignHCenter)
            position_layout.addWidget(button)
        self.position_buttons_overlay.adjustSize()
        
        self.setLayout(layout)
        self.add_grid()

        self.frame_list.itemClicked.connect(self._on_robot_frame_item_clicked)
        self.workspace_frame_list.itemClicked.connect(self._on_workspace_frame_item_clicked)
        self.frame_list.itemChanged.connect(self._on_robot_frame_item_changed)
        self.workspace_frame_list.itemChanged.connect(self._on_workspace_frame_item_changed)
        self.btn_toggle_robot_controls.clicked.connect(self._toggle_robot_controls_overlay)
        self.btn_toggle_cad.clicked.connect(self._on_cad_button_clicked)
        self.btn_toggle_transparency.clicked.connect(self._on_transparency_button_clicked)
        self.btn_toggle_axes.clicked.connect(self._on_axes_button_clicked)
        self.btn_toggle_frame_lists.clicked.connect(self._on_frame_lists_button_clicked)
        self.btn_toggle_viewer_style.clicked.connect(self._on_viewer_style_button_clicked)
        self.btn_toggle_view_presets.clicked.connect(self._on_view_presets_button_clicked)
        self.btn_toggle_perspective.clicked.connect(self._on_perspective_button_clicked)
        self.btn_toggle_workspace_tcp_zones.clicked.connect(self._on_workspace_tcp_zones_button_clicked)
        self.btn_toggle_workspace_collision_zones.clicked.connect(self._on_workspace_collision_zones_button_clicked)
        self.btn_toggle_robot_colliders.clicked.connect(self._on_robot_colliders_button_clicked)
        self.btn_toggle_tool_colliders.clicked.connect(self._on_tool_colliders_button_clicked)
        self.btn_view_right = self._create_overlay_button("Vue droite", "view_right", checkable=False, parent=self.viewer_presets_overlay)
        self.btn_view_left = self._create_overlay_button("Vue gauche", "view_left", checkable=False, parent=self.viewer_presets_overlay)
        self.btn_view_front = self._create_overlay_button("Vue devant", "view_front", checkable=False, parent=self.viewer_presets_overlay)
        self.btn_view_back = self._create_overlay_button("Vue derrière", "view_back", checkable=False, parent=self.viewer_presets_overlay)
        self.btn_view_top = self._create_overlay_button("Vue dessus", "view_top", checkable=False, parent=self.viewer_presets_overlay)
        self.btn_view_bottom = self._create_overlay_button("Vue dessous", "view_bottom", checkable=False, parent=self.viewer_presets_overlay)
        self.btn_view_isometric = self._create_overlay_button("Vue isométrique", "view_iso", checkable=False, parent=self.viewer_presets_overlay)
        for button in (
            self.btn_view_right,
            self.btn_view_left,
            self.btn_view_front,
            self.btn_view_back,
            self.btn_view_top,
            self.btn_view_bottom,
            self.btn_view_isometric,
        ):
            presets_row.addWidget(button)
        presets_layout.addLayout(presets_row)
        self.btn_view_right.clicked.connect(lambda: self._set_camera_preset("right"))
        self.btn_view_left.clicked.connect(lambda: self._set_camera_preset("left"))
        self.btn_view_front.clicked.connect(lambda: self._set_camera_preset("front"))
        self.btn_view_back.clicked.connect(lambda: self._set_camera_preset("back"))
        self.btn_view_top.clicked.connect(lambda: self._set_camera_preset("top"))
        self.btn_view_bottom.clicked.connect(lambda: self._set_camera_preset("bottom"))
        self.btn_view_isometric.clicked.connect(lambda: self._set_camera_preset("isometric"))
        self.background_mode_combo.currentIndexChanged.connect(self._on_background_mode_changed)
        self.btn_background_primary_color.clicked.connect(self._choose_background_primary_color)
        self.btn_background_secondary_color.clicked.connect(self._choose_background_secondary_color)
        self.background_gradient_direction_combo.currentIndexChanged.connect(self._on_background_gradient_direction_changed)
        self.btn_swap_background_colors.clicked.connect(self._swap_background_colors)
        self.btn_text_color.clicked.connect(self._choose_text_color)
        self.btn_accent_color.clicked.connect(self._choose_accent_color)
        self.grid_size_spin.valueChanged.connect(self._on_grid_size_changed)
        self.grid_spacing_spin.valueChanged.connect(self._on_grid_spacing_changed)
        self.btn_grid_color.clicked.connect(self._choose_grid_color)
        self.btn_save_viewer_theme.clicked.connect(self._save_current_viewer_theme)
        self.btn_select_viewer_theme.clicked.connect(self._select_viewer_theme)
        self.btn_save_default_viewer_style.clicked.connect(self._save_current_viewer_style_as_default)
        self.btn_go_position_zero_overlay.clicked.connect(self.get_overlay_joints_widget().position_zero_requested.emit)
        self.btn_go_position_calibration_overlay.clicked.connect(
            self.get_overlay_joints_widget().position_calibration_requested.emit
        )
        self.btn_go_home_position_overlay.clicked.connect(self.get_overlay_joints_widget().home_position_requested.emit)
        self._refresh_toolbar_buttons()
        self._refresh_position_buttons()
        self._refresh_robot_controls_overlay()
        self._sync_viewer_style_controls()

    def _position_overlays(self):
        """Positionne la liste en haut a droite et le label en haut a gauche"""
        margin = 10
        if hasattr(self, "toolbar_overlay"):
            self.toolbar_overlay.adjustSize()
            self.toolbar_overlay.move(margin, margin)
        if hasattr(self, "btn_toggle_frame_lists"):
            anchor_pos = self.btn_toggle_frame_lists.mapTo(self.viewer, QPoint(0, 0))
            overlay_anchor_x = anchor_pos.x() + (self.btn_toggle_frame_lists.width() // 2)
            overlay_y = anchor_pos.y() + self.btn_toggle_frame_lists.height() + 16
        else:
            overlay_anchor_x = self.viewer.width() - margin
            overlay_y = margin
        if hasattr(self, "frame_lists_overlay"):
            self.frame_lists_overlay.adjustSize()
            frame_overlay_x = max(
                margin,
                min(
                    self.viewer.width() - self.frame_lists_overlay.width() - margin,
                    overlay_anchor_x - (self.frame_lists_overlay.width() // 2),
                ),
            )
            self.frame_lists_overlay.move(frame_overlay_x, overlay_y)
        if hasattr(self, "btn_toggle_viewer_style") and hasattr(self, "viewer_style_overlay"):
            self.viewer_style_overlay.adjustSize()
            style_anchor_pos = self.btn_toggle_viewer_style.mapTo(self.viewer, QPoint(0, 0))
            style_anchor_center_x = style_anchor_pos.x() + (self.btn_toggle_viewer_style.width() // 2)
            style_overlay_x = max(
                margin,
                min(
                    self.viewer.width() - self.viewer_style_overlay.width() - margin,
                    style_anchor_center_x - (self.viewer_style_overlay.width() // 2),
                ),
            )
            style_overlay_y = style_anchor_pos.y() + self.btn_toggle_viewer_style.height() + 16
            self.viewer_style_overlay.move(style_overlay_x, style_overlay_y)
        if hasattr(self, "btn_toggle_view_presets") and hasattr(self, "viewer_presets_overlay"):
            self.viewer_presets_overlay.adjustSize()
            presets_anchor_pos = self.btn_toggle_view_presets.mapTo(self.viewer, QPoint(0, 0))
            presets_anchor_center_x = presets_anchor_pos.x() + (self.btn_toggle_view_presets.width() // 2)
            presets_overlay_x = max(
                margin,
                min(
                    self.viewer.width() - self.viewer_presets_overlay.width() - margin,
                    presets_anchor_center_x - (self.viewer_presets_overlay.width() // 2),
                ),
            )
            presets_overlay_y = presets_anchor_pos.y() + self.btn_toggle_view_presets.height() + 16
            self.viewer_presets_overlay.move(presets_overlay_x, presets_overlay_y)
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
            if hasattr(self, "viewer_control_overlay"):
                positions_y = max(
                    margin,
                    self.viewer_control_overlay.y() - self.position_buttons_overlay.height() - 10,
                )
            else:
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

    def set_trajectory_status_message(self, text: str) -> None:
        self._set_label_msg(text)

    def clear_trajectory_status_message(self) -> None:
        self._clear_label_msg()

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
        if hasattr(self, "btn_toggle_robot_controls"):
            self._set_overlay_button_state(self.btn_toggle_robot_controls, controls_visible)

    def _create_style_row(self, label_text: str, control: QWidget) -> QWidget:
        row = QWidget(self.viewer_style_overlay)
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        label = QLabel(label_text, row)
        label.setFixedWidth(78)
        layout.addWidget(label)
        layout.addWidget(control, 1)
        return row

    def _create_color_picker_button(self, tooltip: str) -> QPushButton:
        button = QPushButton("", self.viewer_style_overlay)
        button.setToolTip(tooltip)
        button.setCursor(Qt.CursorShape.PointingHandCursor)
        button.setFixedHeight(26)
        return button

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

    def _on_viewer_style_button_clicked(self) -> None:
        should_show = not self.viewer_style_overlay.isVisible()
        self.viewer_style_overlay.setVisible(should_show)
        self._position_overlays()
        self._refresh_toolbar_buttons()

    def _on_view_presets_button_clicked(self) -> None:
        should_show = not self.viewer_presets_overlay.isVisible()
        self.viewer_presets_overlay.setVisible(should_show)
        self._position_overlays()
        self._refresh_toolbar_buttons()

    def _on_perspective_button_clicked(self) -> None:
        self.viewer.set_perspective_enabled(not self.viewer.is_perspective_enabled())
        self._refresh_toolbar_buttons()

    def _set_camera_preset(self, preset_kind: str) -> None:
        side_center_world, top_center_world, side_distance, top_distance, isometric_distance = self._get_view_preset_camera_parameters()
        if preset_kind in {"front", "back", "right", "left"}:
            center_world = side_center_world
        else:
            center_world = top_center_world

        center_vector = QtGui.QVector3D(
            float(center_world[0]),
            float(center_world[1]),
            float(center_world[2]),
        )

        if preset_kind == "front":
            self.viewer.setCameraPosition(pos=center_vector, distance=side_distance, elevation=0.0, azimuth=0.0)
        elif preset_kind == "back":
            self.viewer.setCameraPosition(pos=center_vector, distance=side_distance, elevation=0.0, azimuth=180.0)
        elif preset_kind == "right":
            self.viewer.setCameraPosition(pos=center_vector, distance=side_distance, elevation=0.0, azimuth=90.0)
        elif preset_kind == "left":
            self.viewer.setCameraPosition(pos=center_vector, distance=side_distance, elevation=0.0, azimuth=270.0)
        elif preset_kind == "top":
            self.viewer.setCameraPosition(pos=center_vector, distance=top_distance, elevation=90.0, azimuth=0.0)
        elif preset_kind == "bottom":
            self.viewer.setCameraPosition(pos=center_vector, distance=top_distance, elevation=-90.0, azimuth=0.0)
        elif preset_kind == "isometric":
            self.viewer.setCameraPosition(pos=center_vector, distance=isometric_distance, elevation=35.26438968, azimuth=45.0)

        self.viewer_presets_overlay.hide()
        self._refresh_toolbar_buttons()

    def _get_view_preset_camera_parameters(self) -> tuple[np.ndarray, np.ndarray, float, float, float]:
        grid_size = max(100.0, float(self._grid_size))
        viewer_width_px = max(1.0, float(self.viewer.width()))
        viewer_height_px = max(1.0, float(self.viewer.height()))
        horizontal_margin_px = 32.0
        usable_width_px = max(1.0, viewer_width_px - (2.0 * horizontal_margin_px))
        horizontal_fill_ratio = np.clip(usable_width_px / viewer_width_px, 0.6, 0.98)
        fov_rad = np.radians(float(self.viewer.opts["fov"]))
        tan_half_fov = max(1e-6, np.tan(0.5 * fov_rad))
        aspect_ratio = max(1e-6, viewer_width_px / viewer_height_px)

        visible_width_world = grid_size / horizontal_fill_ratio
        side_distance = visible_width_world / (2.0 * tan_half_fov * aspect_ratio)
        top_distance = side_distance

        floor_target_y_px = self._get_side_view_floor_target_y()
        floor_ndc_y = 1.0 - (2.0 * floor_target_y_px / viewer_height_px)
        visible_height_world = 2.0 * side_distance * tan_half_fov
        side_center_z = max(0.0, -floor_ndc_y * (0.5 * visible_height_world))

        side_center_world = np.array([0.0, 0.0, side_center_z], dtype=float)
        top_center_world = np.array([0.0, 0.0, 0.0], dtype=float)
        isometric_distance = side_distance * 1.35
        return side_center_world, top_center_world, side_distance, top_distance, isometric_distance

    def _get_side_view_floor_target_y(self) -> float:
        viewer_height_px = max(1.0, float(self.viewer.height()))
        default_target_y = viewer_height_px * 0.76
        if not hasattr(self, "btn_go_home_position_overlay"):
            return default_target_y

        home_button_top_left = self.btn_go_home_position_overlay.mapTo(self.viewer, QPoint(0, 0))
        home_button_center_y = float(home_button_top_left.y() + (self.btn_go_home_position_overlay.height() * 0.5))
        return float(np.clip(home_button_center_y - 4.0, 0.0, viewer_height_px))

    def _on_background_mode_changed(self, _index: int) -> None:
        self._selected_viewer_theme_name = ""
        self._viewer_background_mode = str(self.background_mode_combo.currentData())
        self._apply_viewer_background_style()
        self._refresh_viewer_style_controls()

    def _choose_background_primary_color(self) -> None:
        color = QColorDialog.getColor(self._viewer_background_primary_color, self, "Choisir la couleur du fond")
        if not color.isValid():
            return
        self._selected_viewer_theme_name = ""
        self._viewer_background_primary_color = color
        self._apply_viewer_background_style()
        self._refresh_viewer_style_controls()

    def _choose_background_secondary_color(self) -> None:
        color = QColorDialog.getColor(self._viewer_background_secondary_color, self, "Choisir la seconde couleur du fond")
        if not color.isValid():
            return
        self._selected_viewer_theme_name = ""
        self._viewer_background_secondary_color = color
        self._apply_viewer_background_style()
        self._refresh_viewer_style_controls()

    def _on_background_gradient_direction_changed(self, _index: int) -> None:
        self._selected_viewer_theme_name = ""
        self._viewer_background_gradient_direction = str(self.background_gradient_direction_combo.currentData())
        self._apply_viewer_background_style()
        self._refresh_viewer_style_controls()

    def _swap_background_colors(self) -> None:
        self._selected_viewer_theme_name = ""
        primary_color = QColor(self._viewer_background_primary_color)
        self._viewer_background_primary_color = QColor(self._viewer_background_secondary_color)
        self._viewer_background_secondary_color = primary_color
        self._apply_viewer_background_style()
        self._refresh_viewer_style_controls()

    def _choose_text_color(self) -> None:
        color = QColorDialog.getColor(self._viewer_text_color, self, "Choisir la couleur du texte")
        if not color.isValid():
            return
        self._selected_viewer_theme_name = ""
        self._viewer_text_color = color
        self._apply_viewer_chrome_style()
        self._refresh_viewer_style_controls()

    def _choose_accent_color(self) -> None:
        color = QColorDialog.getColor(self._viewer_accent_color, self, "Choisir la couleur d'accent")
        if not color.isValid():
            return
        self._selected_viewer_theme_name = ""
        self._viewer_accent_color = color
        self._apply_viewer_chrome_style()
        self._refresh_toolbar_buttons()
        self._refresh_position_buttons()
        self._refresh_viewer_style_controls()

    def _on_grid_size_changed(self, value: int) -> None:
        self._selected_viewer_theme_name = ""
        self._grid_size = int(value)
        self._apply_grid_style()
        self._refresh_viewer_style_controls()

    def _on_grid_spacing_changed(self, value: int) -> None:
        self._selected_viewer_theme_name = ""
        self._grid_spacing = int(value)
        self._apply_grid_style()
        self._refresh_viewer_style_controls()

    def _choose_grid_color(self) -> None:
        color = QColorDialog.getColor(self._grid_color, self, "Choisir la couleur de la grille")
        if not color.isValid():
            return
        self._selected_viewer_theme_name = ""
        self._grid_color = color
        self._apply_grid_style()
        self._refresh_viewer_style_controls()

    def _save_current_viewer_style_as_default(self) -> None:
        default_theme_name = self._selected_viewer_theme_name.strip()
        if default_theme_name == "":
            default_theme_name = "Theme par defaut"
            self._viewer_theme_store.save_theme(default_theme_name, self._build_current_viewer_theme_state())
            self._selected_viewer_theme_name = default_theme_name
        self._viewer_theme_store.save_default_theme_name(default_theme_name)
        self._default_viewer_theme_name = default_theme_name
        self._refresh_viewer_style_controls()
        self._emit_display_state_changed()

    def _select_viewer_theme(self) -> None:
        selectable_theme_names = ["Original"] + [stored_theme.name for stored_theme in self._viewer_theme_store.list_themes()]
        current_theme_name = self._selected_viewer_theme_name if self._selected_viewer_theme_name != "" else "Original"
        selected_theme_name, accepted = QInputDialog.getItem(
            self,
            "Selectionner un theme",
            "Theme :",
            selectable_theme_names,
            max(0, selectable_theme_names.index(current_theme_name) if current_theme_name in selectable_theme_names else 0),
            False,
        )
        if not accepted:
            return

        normalized_theme_name = str(selected_theme_name).strip()
        if normalized_theme_name == "Original":
            self._apply_theme_state(self._build_original_viewer_theme_state(), selected_theme_name="")
            self._emit_display_state_changed()
            return

        from PyQt6.QtWidgets import QMessageBox

        action_dialog = QMessageBox(self)
        action_dialog.setWindowTitle("Theme viewer")
        action_dialog.setText(f'Theme selectionne : "{normalized_theme_name}"')
        apply_button = action_dialog.addButton("Appliquer", QMessageBox.ButtonRole.AcceptRole)
        delete_button = action_dialog.addButton("Supprimer", QMessageBox.ButtonRole.DestructiveRole)
        action_dialog.addButton("Annuler", QMessageBox.ButtonRole.RejectRole)
        action_dialog.exec()

        clicked_button = action_dialog.clickedButton()
        if clicked_button is delete_button:
            if self._viewer_theme_store.delete_theme(normalized_theme_name):
                if self._selected_viewer_theme_name == normalized_theme_name:
                    self._selected_viewer_theme_name = ""
                if self._default_viewer_theme_name == normalized_theme_name:
                    self._default_viewer_theme_name = ""
                self._refresh_viewer_style_controls()
                self._emit_display_state_changed()
            return
        if clicked_button is not apply_button:
            return

        selected_theme = self._viewer_theme_store.load_theme(normalized_theme_name)
        if selected_theme is None:
            return
        self._apply_theme_state(selected_theme, selected_theme_name=normalized_theme_name)
        self._emit_display_state_changed()

    def _save_current_viewer_theme(self) -> None:
        initial_name = self._selected_viewer_theme_name.strip()
        selected_name, accepted = QInputDialog.getText(
            self,
            "Enregistrer un theme",
            "Nom du theme :",
            text=initial_name,
        )
        if not accepted:
            return
        normalized_theme_name = str(selected_name).strip()
        if normalized_theme_name == "":
            return
        saved_theme_name = self._viewer_theme_store.save_theme(
            normalized_theme_name,
            self._build_current_viewer_theme_state(),
        )
        self._selected_viewer_theme_name = saved_theme_name
        self._refresh_viewer_style_controls()
        self._emit_display_state_changed()

    def _sync_viewer_style_controls(self) -> None:
        mode_index = self.background_mode_combo.findData(self._viewer_background_mode)
        self.background_mode_combo.blockSignals(True)
        self.background_mode_combo.setCurrentIndex(max(0, mode_index))
        self.background_mode_combo.blockSignals(False)
        direction_index = self.background_gradient_direction_combo.findData(self._viewer_background_gradient_direction)
        self.background_gradient_direction_combo.blockSignals(True)
        self.background_gradient_direction_combo.setCurrentIndex(max(0, direction_index))
        self.background_gradient_direction_combo.blockSignals(False)
        self.grid_size_spin.blockSignals(True)
        self.grid_size_spin.setValue(int(self._grid_size))
        self.grid_size_spin.blockSignals(False)
        self.grid_spacing_spin.blockSignals(True)
        self.grid_spacing_spin.setValue(int(self._grid_spacing))
        self.grid_spacing_spin.blockSignals(False)
        self._refresh_viewer_style_controls()
        self._apply_viewer_chrome_style()
        self._apply_viewer_background_style()
        self._apply_grid_style()

    def _refresh_viewer_style_controls(self) -> None:
        self._set_color_button_preview(self.btn_background_primary_color, self._viewer_background_primary_color)
        self._set_color_button_preview(self.btn_background_secondary_color, self._viewer_background_secondary_color)
        self._set_color_button_preview(self.btn_text_color, self._viewer_text_color)
        self._set_color_button_preview(self.btn_accent_color, self._viewer_accent_color)
        self._set_color_button_preview(self.btn_grid_color, self._grid_color)
        is_gradient = self._viewer_background_mode == "gradient"
        self.background_secondary_color_row.setVisible(is_gradient)
        self.background_gradient_direction_row.setVisible(is_gradient)
        self.background_swap_colors_row.setVisible(is_gradient)
        self.viewer_style_overlay.adjustSize()
        self._position_overlays()

    def _apply_viewer_chrome_style(self) -> None:
        self._apply_application_accent_palette()
        text_rgba = f"rgba({self._viewer_text_color.red()}, {self._viewer_text_color.green()}, {self._viewer_text_color.blue()}, {self._viewer_text_color.alpha()})"

        self.viewer_style_overlay.setStyleSheet(
            f"""
            QWidget#viewerStyleOverlay {{
                background-color: rgba(0, 0, 0, 18);
                border: 1px solid rgba(255, 255, 255, 20);
                border-radius: 10px;
            }}
            QWidget#viewerStyleOverlay QLabel {{
                color: {text_rgba};
                font-size: 10px;
                font-weight: 600;
            }}
            QWidget#viewerStyleOverlay QComboBox {{
                color: {text_rgba};
                min-height: 24px;
            }}
            """
        )
        self.toolbar_overlay.setStyleSheet(
            f"""
            QWidget#viewerToolbarOverlay {{
                background-color: rgba(0, 0, 0, 18);
                border: 1px solid rgba(255, 255, 255, 20);
                border-radius: 10px;
            }}
            QWidget#viewerToolbarZone {{
                background-color: rgba(255, 255, 255, 10);
                border: 1px solid rgba(255, 255, 255, 22);
                border-radius: 8px;
            }}
            QWidget#viewerToolbarZone QLabel {{
                color: {text_rgba};
                font-size: 11px;
                font-weight: 600;
                padding-left: 2px;
            }}
            """
        )
        self.msg_label.setStyleSheet(
            f"""
            QLabel {{
                color: {text_rgba};
                background-color: transparent;
                padding: 5px;
                border-radius: 3px;
            }}
            """
        )
        action_button_style = (
            f"""
            QPushButton {{
                background-color: rgba(255, 255, 255, 14);
                color: {text_rgba};
                border: 1px solid rgba(255, 255, 255, 24);
                border-radius: 6px;
                padding: 5px 8px;
            }}
            QPushButton:hover {{
                background-color: rgba(255, 255, 255, 22);
            }}
            """
        )
        self.btn_save_viewer_theme.setStyleSheet(action_button_style)
        self.btn_select_viewer_theme.setStyleSheet(action_button_style)
        self.btn_save_default_viewer_style.setStyleSheet(action_button_style)
        self.btn_swap_background_colors.setStyleSheet(action_button_style)
        if hasattr(self, "viewer_control_overlay"):
            self.viewer_control_overlay.apply_theme_colors(self._viewer_text_color, self._viewer_accent_color)
        for button in getattr(self, "_iter_overlay_buttons", lambda: [])():
            self._apply_overlay_button_style(button, button.isChecked())
            icon_kind = button.property("icon_kind")
            if icon_kind is not None:
                button.setIcon(self._build_toolbar_icon(str(icon_kind), button.isChecked()))

    def _apply_application_accent_palette(self) -> None:
        app = QApplication.instance()
        if app is None:
            return
        palette = app.palette()
        palette.setColor(QPalette.ColorRole.Highlight, self._viewer_accent_color)
        palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#FFFFFF"))
        palette.setColor(QPalette.ColorRole.Link, self._viewer_accent_color)
        palette.setColor(QPalette.ColorRole.LinkVisited, self._viewer_accent_color.darker(115))
        if hasattr(QPalette.ColorRole, "Accent"):
            palette.setColor(QPalette.ColorRole.Accent, self._viewer_accent_color)
        app.setPalette(palette)

    def _iter_overlay_buttons(self):
        yield self.btn_toggle_cad
        yield self.btn_toggle_transparency
        yield self.btn_toggle_robot_controls
        yield self.btn_toggle_frame_lists
        yield self.btn_toggle_axes
        yield self.btn_toggle_viewer_style
        yield self.btn_toggle_view_presets
        yield self.btn_toggle_perspective
        yield self.btn_toggle_workspace_tcp_zones
        yield self.btn_toggle_workspace_collision_zones
        yield self.btn_toggle_robot_colliders
        yield self.btn_toggle_tool_colliders
        yield self.btn_go_position_calibration_overlay
        yield self.btn_go_position_zero_overlay
        yield self.btn_go_home_position_overlay
        yield self.btn_view_right
        yield self.btn_view_left
        yield self.btn_view_front
        yield self.btn_view_back
        yield self.btn_view_top
        yield self.btn_view_bottom
        yield self.btn_view_isometric

    def _set_color_button_preview(self, button: QPushButton, color: QColor) -> None:
        text_color = "#101010" if color.lightness() > 128 else "#f0f0f0"
        button.setText(color.name().upper())
        button.setStyleSheet(
            f"""
            QPushButton {{
                background-color: rgba({color.red()}, {color.green()}, {color.blue()}, {color.alpha()});
                color: {text_color};
                border: 1px solid rgba(255, 255, 255, 24);
                border-radius: 6px;
                padding: 4px 8px;
                font-weight: 600;
            }}
            QPushButton:hover {{
                border-color: rgba(255, 255, 255, 60);
            }}
            """
        )

    def _apply_viewer_background_style(self) -> None:
        self.viewer.set_background_style(
            self._viewer_background_mode,
            self._viewer_background_primary_color,
            self._viewer_background_secondary_color,
            self._viewer_background_gradient_direction,
        )

    def _apply_grid_style(self) -> None:
        self._rebuild_grid_item()

    def _build_original_viewer_theme_state(self) -> ViewerThemeState:
        return ViewerThemeState(
            background_mode=self.DEFAULT_BACKGROUND_MODE,
            background_primary_color=self._color_to_hex_rgba(self.DEFAULT_BACKGROUND_PRIMARY_COLOR),
            background_secondary_color=self._color_to_hex_rgba(self.DEFAULT_BACKGROUND_SECONDARY_COLOR),
            background_gradient_direction="vertical",
            text_color=self._color_to_hex_rgba(self.DEFAULT_TEXT_COLOR),
            accent_color=self._color_to_hex_rgba(self.ACTIVE_ICON_COLOR),
            grid_size=int(self.DEFAULT_GRID_SIZE),
            grid_spacing=int(self.DEFAULT_GRID_SPACING),
            grid_color=self._color_to_hex_rgba(self.DEFAULT_GRID_COLOR),
        )

    def _build_current_viewer_theme_state(self) -> ViewerThemeState:
        return ViewerThemeState(
            background_mode=self._viewer_background_mode,
            background_primary_color=self._color_to_hex_rgba(self._viewer_background_primary_color),
            background_secondary_color=self._color_to_hex_rgba(self._viewer_background_secondary_color),
            background_gradient_direction=self._viewer_background_gradient_direction,
            text_color=self._color_to_hex_rgba(self._viewer_text_color),
            accent_color=self._color_to_hex_rgba(self._viewer_accent_color),
            grid_size=int(self._grid_size),
            grid_spacing=int(self._grid_spacing),
            grid_color=self._color_to_hex_rgba(self._grid_color),
        )

    def _apply_theme_state(self, theme_state: ViewerThemeState, selected_theme_name: str = "") -> None:
        self._selected_viewer_theme_name = str(selected_theme_name or "").strip()
        self._viewer_background_mode = theme_state.background_mode if theme_state.background_mode in {"solid", "gradient"} else "solid"
        self._viewer_background_primary_color = self._color_from_hex_rgba(theme_state.background_primary_color, self.DEFAULT_BACKGROUND_PRIMARY_COLOR)
        self._viewer_background_secondary_color = self._color_from_hex_rgba(theme_state.background_secondary_color, self.DEFAULT_BACKGROUND_SECONDARY_COLOR)
        self._viewer_background_gradient_direction = (
            theme_state.background_gradient_direction
            if theme_state.background_gradient_direction in {"vertical", "horizontal", "diagonal", "radial"}
            else "vertical"
        )
        self._viewer_text_color = self._color_from_hex_rgba(theme_state.text_color, self.DEFAULT_TEXT_COLOR)
        self._viewer_accent_color = self._color_from_hex_rgba(theme_state.accent_color, self.ACTIVE_ICON_COLOR)
        self._grid_size = max(1, int(theme_state.grid_size))
        self._grid_spacing = max(1, int(theme_state.grid_spacing))
        self._grid_color = self._color_from_hex_rgba(theme_state.grid_color, self.DEFAULT_GRID_COLOR)
        self._sync_viewer_style_controls()

    @staticmethod
    def _color_to_hex_rgba(color: QColor) -> str:
        return "#{:02X}{:02X}{:02X}{:02X}".format(color.red(), color.green(), color.blue(), color.alpha())

    @staticmethod
    def _color_from_hex_rgba(value: str, fallback: QColor) -> QColor:
        normalized = str(value or "").strip()
        if len(normalized) == 9 and normalized.startswith("#"):
            try:
                return QColor(
                    int(normalized[1:3], 16),
                    int(normalized[3:5], 16),
                    int(normalized[5:7], 16),
                    int(normalized[7:9], 16),
                )
            except ValueError:
                return QColor(fallback)
        parsed = QColor(normalized)
        return parsed if parsed.isValid() else QColor(fallback)

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
            theme=self._build_current_viewer_theme_state(),
            selected_theme_name=self._selected_viewer_theme_name,
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
        self._default_viewer_theme_name = self._viewer_theme_store.load_default_theme_name()
        selected_theme_name = str(state.selected_theme_name or "").strip()
        theme_to_apply = self._build_original_viewer_theme_state()
        applied_theme_name = ""
        if self._default_viewer_theme_name != "":
            default_theme = self._viewer_theme_store.load_theme(self._default_viewer_theme_name)
            if default_theme is not None:
                theme_to_apply = default_theme
                applied_theme_name = self._default_viewer_theme_name
        elif selected_theme_name != "":
            selected_theme = self._viewer_theme_store.load_theme(selected_theme_name)
            if selected_theme is not None:
                theme_to_apply = selected_theme
                applied_theme_name = selected_theme_name
        self._apply_theme_state(theme_to_apply, selected_theme_name=applied_theme_name)
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
        self._apply_overlay_button_style(button, False)
        return button

    def _apply_overlay_button_style(self, button: QPushButton, active: bool) -> None:
        accent_rgba_soft = f"rgba({self._viewer_accent_color.red()}, {self._viewer_accent_color.green()}, {self._viewer_accent_color.blue()}, 48)"
        accent_rgba_border = f"rgba({self._viewer_accent_color.red()}, {self._viewer_accent_color.green()}, {self._viewer_accent_color.blue()}, 120)"
        button.setStyleSheet(
            f"""
            QPushButton {{
                background-color: transparent;
                border: 1px solid rgba(255, 255, 255, 28);
                border-radius: 8px;
                padding: 0px;
            }}
            QPushButton:hover {{
                background-color: rgba(255, 255, 255, 20);
                border-color: rgba(255, 255, 255, 55);
            }}
            QPushButton:pressed {{
                background-color: rgba(255, 255, 255, 28);
            }}
            QPushButton:checked {{
                background-color: {accent_rgba_soft};
                border-color: {accent_rgba_border};
            }}
            """
        )

    def _create_toolbar_zone(self, title: str, buttons: tuple[QPushButton, ...]) -> QWidget:
        zone = QWidget(self.toolbar_overlay)
        zone.setObjectName("viewerToolbarZone")
        zone.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        layout = QVBoxLayout(zone)
        layout.setContentsMargins(8, 6, 8, 8)
        layout.setSpacing(6)

        title_label = QLabel(title, zone)
        title_label.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        buttons_layout = QHBoxLayout()
        buttons_layout.setContentsMargins(0, 0, 0, 0)
        buttons_layout.setSpacing(6)
        buttons_layout.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        for button in buttons:
            buttons_layout.addWidget(button)

        layout.addWidget(title_label)
        layout.addLayout(buttons_layout)
        return zone

    def _refresh_toolbar_buttons(self) -> None:
        self._set_overlay_button_state(self.btn_toggle_cad, self._cad_showed)
        self._set_overlay_button_state(self.btn_toggle_transparency, self.transparency_enabled)
        self._set_overlay_button_state(self.btn_toggle_axes, self.show_axes)
        self._set_overlay_button_state(self.btn_toggle_frame_lists, self._is_frame_lists_overlay_visible())
        self._set_overlay_button_state(self.btn_toggle_viewer_style, self.viewer_style_overlay.isVisible())
        self._set_overlay_button_state(self.btn_toggle_view_presets, self.viewer_presets_overlay.isVisible())
        self._set_overlay_button_state(self.btn_toggle_perspective, self.viewer.is_perspective_enabled())
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
        self._apply_overlay_button_style(button, active)
        button.setIcon(self._build_toolbar_icon(str(button.property("icon_kind")), active))
        button.blockSignals(False)

    def _is_base_tool_only_mode(self) -> bool:
        size = len(self.frames_visibility)
        if size <= 1:
            return False
        last = size - 1
        return all(visible == (idx == 0 or idx == last) for idx, visible in enumerate(self.frames_visibility))

    def _is_frame_lists_overlay_visible(self) -> bool:
        return hasattr(self, "frame_lists_overlay") and self.frame_lists_overlay.isVisible()

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

    def _draw_cube_view_icon(self, painter: QPainter, face_kind: str | None) -> None:
        view_pen = QPen(painter.pen())
        view_pen.setWidthF(1.15)
        painter.setPen(view_pen)

        front_face = QPolygonF([
            QPointF(2.8, 5.8),
            QPointF(13.2, 5.8),
            QPointF(13.2, 15.6),
            QPointF(2.8, 15.6),
        ])
        top_face = QPolygonF([
            QPointF(2.8, 5.8),
            QPointF(8.2, 1.8),
            QPointF(17.9, 1.8),
            QPointF(13.2, 5.8),
        ])
        right_face = QPolygonF([
            QPointF(13.2, 5.8),
            QPointF(17.9, 1.8),
            QPointF(17.9, 11.8),
            QPointF(13.2, 15.6),
        ])

        hidden_front_face = QPolygonF([
            QPointF(8.2, 1.8),
            QPointF(17.9, 1.8),
            QPointF(17.9, 11.8),
            QPointF(8.2, 11.8),
        ])
        hidden_right_face = QPolygonF([
            QPointF(2.8, 5.8),
            QPointF(8.2, 1.8),
            QPointF(8.2, 11.8),
            QPointF(2.8, 15.6),
        ])

        outline_brush = QBrush(Qt.BrushStyle.NoBrush)
        highlight_brush = QBrush(QColor(self.ACTIVE_ICON_COLOR.red(), self.ACTIVE_ICON_COLOR.green(), self.ACTIVE_ICON_COLOR.blue(), 95))
        highlight_brush_soft = QBrush(QColor(self.ACTIVE_ICON_COLOR.red(), self.ACTIVE_ICON_COLOR.green(), self.ACTIVE_ICON_COLOR.blue(), 70))

        painter.setBrush(outline_brush)
        painter.drawPolygon(top_face)
        painter.drawPolygon(right_face)
        painter.drawPolygon(front_face)

        if face_kind == "top":
            painter.setBrush(highlight_brush)
            painter.drawPolygon(top_face)
        elif face_kind == "right":
            painter.setBrush(highlight_brush)
            painter.drawPolygon(right_face)
        elif face_kind in {"front", "view_cube"}:
            painter.setBrush(highlight_brush)
            painter.drawPolygon(front_face)
        elif face_kind == "left":
            painter.setBrush(highlight_brush_soft)
            painter.drawPolygon(hidden_right_face)
        elif face_kind == "back":
            painter.setBrush(highlight_brush_soft)
            painter.drawPolygon(hidden_front_face)
        elif face_kind == "bottom":
            painter.setBrush(highlight_brush_soft)
            painter.drawRect(2, 13, 11, 2)
        elif face_kind == "iso":
            painter.setBrush(highlight_brush_soft)
            painter.drawPolygon(top_face)
            painter.setBrush(highlight_brush)
            painter.drawPolygon(right_face)
            painter.setBrush(highlight_brush_soft)
            painter.drawPolygon(front_face)

        painter.setBrush(outline_brush)
        painter.drawPolygon(top_face)
        painter.drawPolygon(right_face)
        painter.drawPolygon(front_face)

    def _build_toolbar_icon(self, icon_kind: str, active: bool) -> QIcon:
        size = 20
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)

        color = QColor(self._viewer_text_color) if not active else QColor(self._viewer_accent_color)
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
        elif icon_kind == "frame_list":
            painter.drawLine(4, 5, 16, 5)
            painter.drawLine(4, 10, 16, 10)
            painter.drawLine(4, 15, 16, 15)
            painter.drawEllipse(2, 4, 2, 2)
            painter.drawEllipse(2, 9, 2, 2)
            painter.drawEllipse(2, 14, 2, 2)
        elif icon_kind == "appearance":
            painter.drawRect(3, 4, 14, 12)
            painter.drawLine(3, 9, 17, 9)
            painter.drawLine(8, 4, 8, 16)
            painter.drawEllipse(5, 6, 2, 2)
            painter.drawEllipse(10, 11, 2, 2)
        elif icon_kind == "view_cube":
            self._draw_cube_view_icon(painter, "view_cube")
        elif icon_kind == "perspective":
            painter.drawRect(4, 5, 12, 10)
            painter.drawLine(4, 5, 7, 3)
            painter.drawLine(16, 5, 19, 3)
            painter.drawLine(7, 3, 19, 3)
            painter.drawLine(16, 15, 19, 13)
            painter.drawLine(19, 3, 19, 13)
            painter.drawLine(10, 8, 13, 8)
            painter.drawLine(9, 10, 14, 10)
            painter.drawLine(8, 12, 15, 12)
        elif icon_kind == "view_right":
            self._draw_cube_view_icon(painter, "right")
        elif icon_kind == "view_left":
            self._draw_cube_view_icon(painter, "left")
        elif icon_kind == "view_front":
            self._draw_cube_view_icon(painter, "front")
        elif icon_kind == "view_back":
            self._draw_cube_view_icon(painter, "back")
        elif icon_kind == "view_top":
            self._draw_cube_view_icon(painter, "top")
        elif icon_kind == "view_bottom":
            self._draw_cube_view_icon(painter, "bottom")
        elif icon_kind == "view_iso":
            self._draw_cube_view_icon(painter, "iso")
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
        elif icon_kind == "robot_controls":
            painter.drawEllipse(4, 3, 12, 6)
            painter.drawLine(10, 9, 10, 15)
            painter.drawLine(6, 16, 10, 12)
            painter.drawLine(14, 16, 10, 12)
            painter.drawLine(5, 6, 3, 10)
            painter.drawLine(15, 6, 17, 10)
            painter.drawEllipse(2, 10, 3, 3)
            painter.drawEllipse(15, 10, 3, 3)
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

    def _on_robot_frame_item_changed(self, item: QListWidgetItem) -> None:
        index = self.frame_list.row(item)
        if index < 0 or index >= len(self.frames_visibility):
            return
        is_visible = item.checkState() == Qt.CheckState.Checked
        if self.frames_visibility[index] == is_visible:
            return
        self.frames_visibility[index] = is_visible
        self._clear_and_refresh()
        self._refresh_toolbar_buttons()
        self._emit_display_state_changed()

    def _on_robot_frame_item_clicked(self, item: QListWidgetItem) -> None:
        self._toggle_list_item_check_state(self.frame_list, item)

    def _on_workspace_frame_item_changed(self, item: QListWidgetItem) -> None:
        index = self.workspace_frame_list.row(item)
        if index < 0 or index >= len(self.workspace_frames_visibility):
            return
        is_visible = item.checkState() == Qt.CheckState.Checked
        if self.workspace_frames_visibility[index] == is_visible:
            return
        self.workspace_frames_visibility[index] = is_visible
        self._clear_and_refresh()
        self._refresh_toolbar_buttons()
        self._emit_display_state_changed()

    def _on_workspace_frame_item_clicked(self, item: QListWidgetItem) -> None:
        self._toggle_list_item_check_state(self.workspace_frame_list, item)

    def _toggle_list_item_check_state(self, list_widget: QListWidget, item: QListWidgetItem) -> None:
        if item is None:
            return
        list_widget.blockSignals(True)
        item.setCheckState(
            Qt.CheckState.Unchecked if item.checkState() == Qt.CheckState.Checked else Qt.CheckState.Checked
        )
        list_widget.blockSignals(False)
        list_widget.itemChanged.emit(item)

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

    def _on_frame_lists_button_clicked(self):
        if not hasattr(self, "frame_lists_overlay"):
            return
        should_show = not self.frame_lists_overlay.isVisible()
        if should_show:
            self._refresh_frame_lists_overlay()
        self.frame_lists_overlay.setVisible(should_show)
        self._position_overlays()
        self._refresh_toolbar_buttons()

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
        self._sync_frame_list_widget(self.frame_list, self.frames_visibility)
        self._sync_frame_list_widget(
            self.workspace_frame_list,
            self.workspace_frames_visibility,
            self._workspace_frame_labels,
        )
        self._refresh_frame_lists_overlay()
        return
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

        self.frame_overlay.set_frames_visibility(
            self.frames_visibility,
            self._robot_frame_labels(),
        )
        self.workspace_frame_overlay.set_frames_visibility(
            self.workspace_frames_visibility,
            self._workspace_frame_labels,
        )

    def _sync_frame_list_widget(
        self,
        list_widget: QListWidget,
        frames_visibility: list[bool],
        labels: list[str] | None = None,
    ) -> None:
        count = len(frames_visibility)
        normalized_labels = [str(label) for label in labels] if isinstance(labels, list) else []
        list_widget.blockSignals(True)
        if list_widget.count() != count:
            list_widget.clear()
            for index in range(count):
                item = QListWidgetItem(self._frame_label_for_index(index, normalized_labels))
                item.setTextAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
                item.setSizeHint(QSize(0, 28))
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled)
                list_widget.addItem(item)
        else:
            for index in range(count):
                item = list_widget.item(index)
                if item is not None:
                    item.setText(self._frame_label_for_index(index, normalized_labels))
                    item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled)
        for index, is_visible in enumerate(frames_visibility):
            item = list_widget.item(index)
            if item is None:
                continue
            item.setCheckState(Qt.CheckState.Checked if is_visible else Qt.CheckState.Unchecked)
        self._set_frame_list_height(list_widget, count)
        list_widget.blockSignals(False)

    def _frame_label_for_index(self, index: int, labels: list[str]) -> str:
        if 0 <= index < len(labels):
            label = labels[index].strip()
            if label:
                return label
        robot_labels = self._robot_frame_labels()
        if 0 <= index < len(robot_labels):
            return robot_labels[index]
        return f"Frame {index}"

    def _robot_frame_labels(self) -> list[str]:
        count = len(self.frames_visibility)
        if count <= 0:
            return []
        labels = [f"Frame {index}" for index in range(count)]
        labels[0] = "Robot Frame"
        labels[-1] = "Tool Frame"
        return labels

    def _set_frame_list_height(self, list_widget: QListWidget, count: int) -> None:
        row_height = max(22, list_widget.sizeHintForRow(0) if count > 0 else 22)
        visible_rows = min(count, 10)
        height = (row_height * max(1, visible_rows)) + (2 * list_widget.frameWidth())
        list_widget.setFixedHeight(height)

    def _refresh_frame_lists_overlay(self) -> None:
        has_robot_frames = self.frame_list.count() > 0
        has_scene_frames = self.workspace_frame_list.count() > 0
        self.robot_frame_column.setVisible(has_robot_frames)
        self.robot_frame_list_label.setVisible(has_robot_frames)
        self.frame_list.setVisible(has_robot_frames)
        self.scene_frame_column.setVisible(has_scene_frames)
        self.scene_frame_list_label.setVisible(has_scene_frames)
        self.workspace_frame_list.setVisible(has_scene_frames)
        should_show_overlay = self.btn_toggle_frame_lists.isChecked() and (has_robot_frames or has_scene_frames)
        self.frame_lists_overlay.setVisible(should_show_overlay)
        self.frame_lists_overlay.adjustSize()
        self._position_overlays()

    def add_grid(self):
        self._rebuild_grid_item()

    def _rebuild_grid_item(self) -> None:
        if self._grid_item is not None:
            self._safe_remove_viewer_item(self._grid_item)
            self._grid_item = None
            self.viewer.set_grid_reference(None)

        grid = gl.GLGridItem()
        grid.setSize(x=float(self._grid_size), y=float(self._grid_size), z=0.0)
        grid.setSpacing(x=float(self._grid_spacing), y=float(self._grid_spacing), z=float(self._grid_spacing))
        grid.setColor((self._grid_color.red(), self._grid_color.green(), self._grid_color.blue(), self._grid_color.alpha()))
        self.viewer.addItem(grid)
        self._grid_item = grid
        self.viewer.set_grid_reference(grid)

    def clear_viewer(self):
        self.viewer.clear()
        self._grid_item = None
        self.viewer.set_grid_reference(None)
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
        tangent_out_segments: list[TangentSegment] | None,
        tangent_in_segments: list[TangentSegment] | None,
    ) -> None:
        if tangent_out_segments is None:
            self._trajectory_tangent_out_segments = None
        else:
            parsed_out: list[np.ndarray] = []
            for segment in tangent_out_segments:
                start_xyz, end_xyz = segment
                parsed_out.append(np.array([start_xyz.to_list(), end_xyz.to_list()], dtype=float))
            self._trajectory_tangent_out_segments = parsed_out if parsed_out else None

        if tangent_in_segments is None:
            self._trajectory_tangent_in_segments = None
        else:
            parsed_in: list[np.ndarray] = []
            for segment in tangent_in_segments:
                start_xyz, end_xyz = segment
                parsed_in.append(np.array([start_xyz.to_list(), end_xyz.to_list()], dtype=float))
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
            plt.setGLOptions('additive')
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
            dh_matrices, corrected_matrices = self._resolve_robot_matrices(robot_model, tool_model)
            self.last_dh_matrices = dh_matrices
            self.last_corrected_matrices = corrected_matrices
            self.add_robot_links(corrected_matrices)
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
            dh_matrices, corrected_matrices = self._resolve_robot_matrices(robot_model, tool_model)
            self.last_dh_matrices = dh_matrices
            self.last_corrected_matrices = corrected_matrices
            ghost_matrices = (
                self.last_ghost_corrected_matrices if self.last_ghost_corrected_matrices else corrected_matrices
            )
            tool_cad_model = self._resolve_tool_cad_model()

            self._replace_tool_link(corrected_matrices, tool_cad_model, ghost=False)
            self._replace_tool_link(ghost_matrices, tool_cad_model, ghost=True)

            if self.transparency_enabled:
                self.set_transparency(True, emit_signal=False)
            self._clear_and_refresh()
        finally:
            self.end_loading_feedback()

    def _resolve_robot_matrices(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel | None = None,
    ) -> tuple[list[np.ndarray], list[np.ndarray]]:
        dh_matrices = robot_model.get_current_tcp_dh_matrices()
        corrected_matrices = robot_model.get_current_tcp_corrected_dh_matrices()
        if dh_matrices and corrected_matrices:
            return dh_matrices, corrected_matrices

        if self.last_dh_matrices and self.last_corrected_matrices:
            return self.last_dh_matrices, self.last_corrected_matrices

        active_tool = tool_model.get_tool() if tool_model is not None else None
        fk_result = robot_model.compute_fk_joints(robot_model.get_joints(), tool=active_tool)
        if fk_result is None:
            return [], []

        return fk_result.dh_matrices, fk_result.corrected_matrices

    def load_robot_mesh(self, stl_path: str, transform_matrix, color: tuple[int, int, int]):
        try:
            resolved_stl_path = self._resolve_filesystem_path(stl_path)
            if not resolved_stl_path:
                return None
            if resolved_stl_path in self._missing_mesh_paths:
                if os.path.exists(resolved_stl_path):
                    self._missing_mesh_paths.remove(resolved_stl_path)
                else:
                    return None
            mesh_data = self._mesh_data_cache.get(resolved_stl_path)
            if mesh_data is None:
                stl_mesh = mesh.Mesh.from_file(resolved_stl_path)
                verts = stl_mesh.vectors.reshape(-1, 3)
                faces = np.arange(len(verts)).reshape(-1, 3)
                mesh_data = gl.MeshData(vertexes=verts, faces=faces)
                self._mesh_data_cache[resolved_stl_path] = mesh_data

            mesh_item = gl.GLMeshItem(
                meshdata=mesh_data,
                smooth=True,
                color=self._brighten_mesh_color(color),
                shader=Viewer3DWidget.CAD_SHADER_NAME,
            )
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
            resolved_stl_path = self._resolve_filesystem_path(stl_path)
            if resolved_stl_path:
                self._missing_mesh_paths.add(resolved_stl_path)
            print(f"Erreur STL {resolved_stl_path or stl_path}: {e}")
            return None

    @staticmethod
    def _resolve_filesystem_path(path: str) -> str:
        normalized = str(path or "").strip()
        if normalized == "":
            return ""
        if os.path.isabs(normalized):
            return os.path.abspath(normalized)
        return os.path.abspath(os.path.join(os.getcwd(), normalized))

    @staticmethod
    def _ensure_custom_shaders_registered() -> None:
        if Viewer3DWidget.CAD_SHADER_NAME in gl_shaders.ShaderProgram.names:
            return
        gl_shaders.ShaderProgram(
            Viewer3DWidget.CAD_SHADER_NAME,
            [
                gl_shaders.VertexShader(
                    """
                    uniform mat4 u_mvp;
                    uniform mat3 u_normal;
                    attribute vec4 a_position;
                    attribute vec3 a_normal;
                    attribute vec4 a_color;
                    varying vec4 v_color;
                    varying vec3 v_normal;
                    void main() {
                        v_normal = normalize(u_normal * a_normal);
                        v_color = a_color;
                        gl_Position = u_mvp * a_position;
                    }
                    """
                ),
                gl_shaders.FragmentShader(
                    """
                    #ifdef GL_ES
                    precision mediump float;
                    #endif
                    varying vec4 v_color;
                    varying vec3 v_normal;
                    void main() {
                        vec3 normal = normalize(v_normal);
                        vec3 key_light = normalize(vec3(1.0, -0.8, -0.6));
                        vec3 fill_light = normalize(vec3(-0.7, 0.45, -0.35));
                        vec3 rim_light = normalize(vec3(0.15, 0.25, 1.0));
                        float key = max(dot(normal, key_light), 0.0);
                        float fill = max(dot(normal, fill_light), 0.0);
                        float rim = pow(1.0 - max(dot(normal, rim_light), 0.0), 2.0);
                        float lighting = 0.48 + key * 0.62 + fill * 0.28 + rim * 0.12;
                        vec3 rgb = clamp(v_color.rgb * lighting, 0.0, 1.0);
                        gl_FragColor = vec4(rgb, v_color.a);
                    }
                    """
                ),
            ],
        )

    @staticmethod
    def _brighten_mesh_color(color: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        red, green, blue, alpha = color
        brighten = lambda component: min(1.0, component * 0.82 + 0.18)
        return (brighten(red), brighten(green), brighten(blue), alpha)

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

    def _resolve_robot_cad_colors(self) -> list[CadColor]:
        if self._robot_model is None:
            return [CadColor("") for _ in range(7)]
        return self._robot_model.get_robot_cad_colors()

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

    def _resolve_link_color(self, matrix_index: int) -> tuple[float, float, float, float]:
        robot_cad_colors = self._resolve_robot_cad_colors()
        if 0 <= matrix_index < len(robot_cad_colors):
            return robot_cad_colors[matrix_index].to_rgb_float_tuple(alpha=0.5)
        return CadColor("").to_rgb_float_tuple(alpha=0.5)

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
            dh_matrices = fk_result.dh_matrices
            corrected_matrices = fk_result.corrected_matrices

        self.last_dh_matrices = dh_matrices
        self.last_corrected_matrices = corrected_matrices

        expected_link_count = len(self._build_cad_specs(corrected_matrices))
        if self._cad_loaded and len(self.robot_links) != expected_link_count:
            self.add_robot_links(corrected_matrices)
            self._cad_loaded = True

        self._refresh_robot_state_items()
        self._refresh_position_buttons()

    def update_workspace(self, workspace_model: WorkspaceModel | None) -> None:
        previous_model = self._workspace_model
        previous_base_revision = self._robot_base_transform_world.revision
        previous_structure_revision = self._workspace_structure_revision

        self._workspace_model = workspace_model
        self._robot_base_transform_world = (
            FrameTransform.from_pose(Pose6.zeros())
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
            WorkspaceElementState.from_element(element, index)
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
    def _pose_to_matrix(pose) -> np.ndarray:
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
        self._workspace_frame_labels = ["World Frame"]

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

        self.draw_all_frames(self.last_dh_matrices)

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
        primitive: PrimitiveCollider | PrimitiveColliderState,
        color: tuple[float, float, float, float],
        base_transform: np.ndarray | None = None,
        skip_pose: bool = False,
    ) -> gl.GLMeshItem | None:
        if isinstance(primitive, PrimitiveCollider):
            shape = primitive.shape.value
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
            raise TypeError("primitive must be a PrimitiveCollider or PrimitiveColliderState")

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

        self.update_robot_ghost_from_matrices(fk_result.corrected_matrices)

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

