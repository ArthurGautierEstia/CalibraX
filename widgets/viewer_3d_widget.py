from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QListWidget, QListWidgetItem, QAbstractItemView, QLabel
from PyQt6.QtCore import Qt, QSize
from PyQt6.QtGui import QFont
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui
import numpy as np
from stl import mesh

from models.robot_model import RobotModel

class Viewer3DWidget(QWidget):
    """Widget pour la visualisation 3D avec PyQtGraph"""

    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        self.robot_links: list[gl.GLMeshItem] = []
        self.robot_ghost_links: list[gl.GLMeshItem] = []
        self._trajectory_path_items: list[gl.GLLinePlotItem] = []
        self._trajectory_keypoints_item: gl.GLScatterPlotItem | None = None
        self._trajectory_keypoint_selected_item: gl.GLScatterPlotItem | None = None
        self._trajectory_keypoint_editing_item: gl.GLScatterPlotItem | None = None
        self._trajectory_tangent_out_item: gl.GLLinePlotItem | None = None
        self._trajectory_tangent_in_item: gl.GLLinePlotItem | None = None
        self._trajectory_path_points: np.ndarray | None = None
        self._trajectory_path_segments: list[tuple[np.ndarray, tuple[float, float, float, float]]] | None = None
        self._trajectory_keypoint_points: np.ndarray | None = None
        self._trajectory_keypoint_selected_index: int | None = None
        self._trajectory_keypoint_editing_index: int | None = None
        self._trajectory_tangent_out_segment: np.ndarray | None = None
        self._trajectory_tangent_in_segment: np.ndarray | None = None
        self.last_dh_matrices = []
        self.last_corrected_matrices = []
        self.last_ghost_corrected_matrices = []
        self._robot_link_matrix_indices: list[int] = []
        self._robot_ghost_link_matrix_indices: list[int] = []
        self._robot_link_roles: list[str] = []
        self._robot_ghost_link_roles: list[str] = []
        self.last_invert_table = []
        self.frames_visibility: list[bool] = []
        self.show_axes = True
        self._cad_loaded = False
        self._cad_showed = True
        self._ghost_visible = False
        self._robot_model: RobotModel | None = None
        self._mesh_data_cache: dict[str, gl.MeshData] = {}
        self.transparency_enabled = False
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
        self.frame_list = QListWidget(self.viewer) # Parent = viewer pour l'overlay
        self.frame_list.setGeometry(10, 10, 150, 300) # Position et taille
        self.frame_list.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.frame_list.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.frame_list.setStyleSheet("""
            QListWidget {
                background-color: rgba(25, 25, 28, 130);
                color: lightgray;
                border: 1px solid rgba(255, 255, 255, 35);
                border-radius: 6px;
                outline: 0;
            }
            QListWidget::item {
                padding: 6px 8px;
            }
        """)
        
        self.frame_list.hide()

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

        # Boutons de contrôle (CAD / Transparence / Repères global)
        toggle_layout = QHBoxLayout()

        self.btn_toggle_cad = QPushButton("CAD")

        self.btn_toggle_transparency = QPushButton("Transparence")
        
        self.btn_toggle_axes = QPushButton("Afficher / Masquer tous les Repères")

        self.btn_toggle_axes_base_tool = QPushButton("Repères Base & Tool")
        
        toggle_layout.addWidget(self.btn_toggle_cad)
        toggle_layout.addWidget(self.btn_toggle_transparency)
        toggle_layout.addWidget(self.btn_toggle_axes)
        toggle_layout.addWidget(self.btn_toggle_axes_base_tool)
        layout.addLayout(toggle_layout)
        
        self.setLayout(layout)
        self.add_grid()

        self.frame_list.itemClicked.connect(self.on_frame_clicked)
        self.btn_toggle_cad.clicked.connect(self._on_cad_button_clicked)
        self.btn_toggle_transparency.clicked.connect(self._on_transparency_button_clicked)
        self.btn_toggle_axes.clicked.connect(self._on_axes_button_clicked)
        self.btn_toggle_axes_base_tool.clicked.connect(self.toogle_base_axis_frames)

    def _position_overlays(self):
        """Positionne la liste en haut a droite et le label en haut a gauche"""
        margin = 10
        frame_list_x = max(margin, self.viewer.width() - self.frame_list.width() - margin)
        self.frame_list.move(frame_list_x, margin)
        self.msg_label.move(margin, margin)

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

    def on_frame_clicked(self, item: QListWidgetItem):
        """Gère le clic sur un élément de la liste"""
        index = self.frame_list.row(item)
        self.frames_visibility[index] = not self.frames_visibility[index]
        self._clear_and_refresh()
    
    def _on_cad_button_clicked(self):
        self.set_robot_visibility(not self._cad_showed)

    def _on_transparency_button_clicked(self):
        self.set_transparency(not self.transparency_enabled)

    def _on_axes_button_clicked(self):
        self.show_axes = not self.show_axes
        for i in range(6):
            self.frames_visibility[i] = self.show_axes
        self._clear_and_refresh()

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

    def add_grid(self):
        grid = gl.GLGridItem()
        grid.setSize(x=4000, y=4000, z=0)
        grid.setSpacing(x=200, y=200, z=200)
        grid.setColor((150, 150, 150, 100))
        self.viewer.addItem(grid)

    def clear_viewer(self):
        self.viewer.clear()
        self.add_grid()
        self._trajectory_path_items = []
        self._trajectory_keypoints_item = None
        self._trajectory_keypoint_selected_item = None
        self._trajectory_keypoint_editing_item = None
        self._trajectory_tangent_out_item = None
        self._trajectory_tangent_in_item = None

    def set_trajectory_path(self, points_xyz: list[list[float]]) -> None:
        self._trajectory_path_segments = None
        if len(points_xyz) < 2:
            self._trajectory_path_points = None
        else:
            self._trajectory_path_points = np.array(points_xyz, dtype=float)
        self._render_trajectory_overlay()

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
        self._trajectory_path_points = None
        self._render_trajectory_overlay()

    def clear_trajectory_path(self) -> None:
        self._trajectory_path_points = None
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
        tangent_out_segment: list[list[float]] | None,
        tangent_in_segment: list[list[float]] | None,
    ) -> None:
        self._trajectory_tangent_out_segment = (
            None if tangent_out_segment is None else np.array([p[:3] for p in tangent_out_segment], dtype=float)
        )
        self._trajectory_tangent_in_segment = (
            None if tangent_in_segment is None else np.array([p[:3] for p in tangent_in_segment], dtype=float)
        )
        self._render_trajectory_overlay()

    def clear_trajectory_edit_tangents(self) -> None:
        self._trajectory_tangent_out_segment = None
        self._trajectory_tangent_in_segment = None
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
        if self._trajectory_tangent_out_item is not None:
            self.viewer.removeItem(self._trajectory_tangent_out_item)
            self._trajectory_tangent_out_item = None
        if self._trajectory_tangent_in_item is not None:
            self.viewer.removeItem(self._trajectory_tangent_in_item)
            self._trajectory_tangent_in_item = None

        if self._trajectory_path_segments is not None and len(self._trajectory_path_segments) > 0:
            for points_xyz, color in self._trajectory_path_segments:
                path_item = gl.GLLinePlotItem(
                    pos=points_xyz,
                    color=color,
                    width=2,
                    antialias=True,
                )
                self._trajectory_path_items.append(path_item)
                self.viewer.addItem(path_item)
        elif self._trajectory_path_points is not None and len(self._trajectory_path_points) >= 2:
            path_item = gl.GLLinePlotItem(
                pos=self._trajectory_path_points,
                color=(1.0, 0.84, 0.1, 0.85),
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
                self._trajectory_keypoints_item = gl.GLScatterPlotItem(
                    pos=base_points,
                    color=(0.95, 0.95, 0.95, 0.9),
                    size=9,
                    pxMode=True,
                )
                self.viewer.addItem(self._trajectory_keypoints_item)

            if selected_idx is not None and 0 <= selected_idx < len(points):
                self._trajectory_keypoint_selected_item = gl.GLScatterPlotItem(
                    pos=np.array([points[selected_idx]], dtype=float),
                    color=(0.1, 0.85, 1.0, 1.0),
                    size=13,
                    pxMode=True,
                )
                self.viewer.addItem(self._trajectory_keypoint_selected_item)

            if editing_idx is not None and 0 <= editing_idx < len(points):
                self._trajectory_keypoint_editing_item = gl.GLScatterPlotItem(
                    pos=np.array([points[editing_idx]], dtype=float),
                    color=(1.0, 0.35, 0.1, 1.0),
                    size=15,
                    pxMode=True,
                )
                self.viewer.addItem(self._trajectory_keypoint_editing_item)

        if self._trajectory_tangent_out_segment is not None and len(self._trajectory_tangent_out_segment) >= 2:
            self._trajectory_tangent_out_item = gl.GLLinePlotItem(
                pos=self._trajectory_tangent_out_segment,
                color=(1.0, 0.5, 0.1, 0.95),
                width=2,
                antialias=True,
            )
            self.viewer.addItem(self._trajectory_tangent_out_item)

        if self._trajectory_tangent_in_segment is not None and len(self._trajectory_tangent_in_segment) >= 2:
            self._trajectory_tangent_in_item = gl.GLLinePlotItem(
                pos=self._trajectory_tangent_in_segment,
                color=(0.25, 1.0, 0.55, 0.95),
                width=2,
                antialias=True,
            )
            self.viewer.addItem(self._trajectory_tangent_in_item)

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

        for i, axis in enumerate(axes):
            plt = gl.GLLinePlotItem(pos=axis, color=couleurs[i], width=3, antialias=True)
            self.viewer.addItem(plt)

    def draw_all_frames(self, matrices):
        """Dessine les repères en fonction de leur visibilité individuelle"""
        for i, T in enumerate(matrices):
            # On dessine seulement si l'index est marqué visible dans la liste
            if i < len(self.frames_visibility) and self.frames_visibility[i]:
                self.draw_frame(T)
    
    def load_cad(self, robot_model: RobotModel):
        self._robot_model = robot_model
        self._set_label_msg("CAD loading...")
        QApplication.processEvents()  # Force le traitement des événements pour afficher le curseur
        matrices = self._resolve_cad_matrices(robot_model)
        self.last_corrected_matrices = matrices
        self.add_robot_links(matrices)
        if self.transparency_enabled:
            self.set_transparency(True)
        self._clear_label_msg()
        self._cad_loaded = True

    def reload_tool_cad(self, robot_model: RobotModel):
        self._robot_model = robot_model
        if not self._cad_loaded:
            self.load_cad(robot_model)
            return

        matrices = self._resolve_cad_matrices(robot_model)
        self.last_corrected_matrices = matrices
        ghost_matrices = self.last_ghost_corrected_matrices if self.last_ghost_corrected_matrices else matrices
        tool_cad_model = self._resolve_tool_cad_model()

        self._replace_tool_link(matrices, tool_cad_model, ghost=False)
        self._replace_tool_link(ghost_matrices, tool_cad_model, ghost=True)

        if self.transparency_enabled:
            self.set_transparency(True)

    def _resolve_cad_matrices(self, robot_model: RobotModel) -> list[np.ndarray]:
        matrices = robot_model.get_current_tcp_corrected_dh_matrices()
        if matrices:
            return matrices

        if self.last_corrected_matrices:
            return self.last_corrected_matrices

        fk_result = robot_model.compute_fk_joints(robot_model.get_joints())
        if fk_result is None:
            return []

        _, corrected_matrices, _, _, _ = fk_result
        return corrected_matrices

    def load_robot_mesh(self, stl_path: str, transform_matrix, color: tuple[int, int, int]):
        # (Copier le code original ici, pas de changement)
        try:
            if not stl_path:
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
            print(f"Erreur STL {stl_path}: {e}")
            return None

    def _resolve_robot_cad_models(self) -> list[str]:
        if self._robot_model is None:
            return [f"./robot_stl/rocky{i}.stl" for i in range(7)]
        cad_models = self._robot_model.get_robot_cad_models()
        if not cad_models:
            return [f"./robot_stl/rocky{i}.stl" for i in range(7)]
        return [str(path) for path in cad_models]

    def _resolve_tool_cad_model(self) -> str:
        if self._robot_model is None:
            return ""
        return str(self._robot_model.get_tool_cad_model())

    def _resolve_tool_cad_offset_rz(self) -> float:
        if self._robot_model is None:
            return 0.0
        return float(self._robot_model.get_tool_cad_offset_rz())

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

        theta = np.radians(offset_rz_deg)
        rot_z = np.array(
            [
                [np.cos(theta), -np.sin(theta), 0.0, 0.0],
                [np.sin(theta), np.cos(theta), 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        # Rotation locale autour de l'axe Z du repere outil.
        return transform_matrix @ rot_z

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
                stl_path = f"./robot_stl/rocky{matrix_index}.stl"

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
            mesh_item = self.load_robot_mesh(stl_path, T, link_color)
            if mesh_item:
                self.robot_links.append(mesh_item)
                self._robot_link_matrix_indices.append(matrix_index)
                self._robot_link_roles.append("tool" if is_tool else "robot")
                self.viewer.addItem(mesh_item)
                if not self._cad_showed:
                    mesh_item.hide()

            ghost_item = self.load_robot_mesh(stl_path, T, ghost_color)
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
        mesh_item = self.load_robot_mesh(stl_path, visual_transform, color)
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

    def update_robot(self, robot_model: RobotModel):
        """Met à jour la visualisation 3D avec repères et visibilité des frames"""
        self._robot_model = robot_model

        #self.last_dh_matrices = robot_model.get_current_tcp_dh_matrices()
        #self.last_corrected_matrices = robot_model.get_current_tcp_corrected_dh_matrices()
        
        # ne pas prendre les matrices courantes mais recalculer à partir des joints actuels (inversé)
        # compute_fk_joints va renvoyer les valeurs correctes sans inversion

        dh_matrices, corrected_matrices, _, _, _ = robot_model.compute_fk_joints(robot_model.get_joints())

        self.last_dh_matrices = dh_matrices
        self.last_corrected_matrices = corrected_matrices

        self._clear_and_refresh()

    def _clear_and_refresh(self):
        num_frames = len(self.last_dh_matrices)
        
        # Initialiser la liste de visibilité si nécessaire
        if len(self.frames_visibility) != num_frames:
            self.frames_visibility = [True] * num_frames
        
        # Mettre à jour l'interface de la liste (affichage gras/normal)
        self.update_frame_list_ui()
        
        # Effacer et redessiner la scène
        self.clear_viewer()
        
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
        self._render_trajectory_overlay()

    def update_robot_poses(self, matrices):
        tool_offset_rz = self._resolve_tool_cad_offset_rz()
        for mesh_item, matrix_index, role in zip(self.robot_links, self._robot_link_matrix_indices, self._robot_link_roles):
            if matrix_index >= len(matrices):
                continue
            T = matrices[matrix_index]
            if role == "tool":
                T = self._apply_tool_visual_offset(T, tool_offset_rz)
            if mesh_item:
                mesh_item.resetTransform()
                qmat = QtGui.QMatrix4x4(
                    T[0,0], T[0,1], T[0,2], T[0,3],
                    T[1,0], T[1,1], T[1,2], T[1,3],
                    T[2,0], T[2,1], T[2,2], T[2,3],
                    T[3,0], T[3,1], T[3,2], T[3,3]
                )
                mesh_item.setTransform(qmat)
                self.viewer.addItem(mesh_item)

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

    def set_robot_visibility(self, visible: bool):
        self._cad_showed = visible
        for mesh_item in self.robot_links:
            if visible: mesh_item.show()
            else: mesh_item.hide()

    def set_transparency(self, enabled: bool):
        self.transparency_enabled = enabled
        for mesh_item in self.robot_links:
            mesh_item.setGLOptions('translucent' if enabled else 'opaque')

    def toogle_base_axis_frames(self):
        self.show_axes = True
        size = len(self.frames_visibility)
        last = size - 1
        self.frames_visibility = [(i == 0 or i == last) for i in range(size)]
        self._clear_and_refresh()

    def show_robot_ghost(self):
        self._ghost_visible = True

        if not self._cad_loaded:
            matrices = self.last_corrected_matrices
            if not matrices and self._robot_model is not None:
                matrices = self._resolve_cad_matrices(self._robot_model)
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

        fk_result = self._robot_model.compute_fk_joints(joints)
        if fk_result is None:
            self.hide_robot_ghost()
            return

        _, corrected_matrices, _, _, _ = fk_result
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
            if mesh_item:
                mesh_item.resetTransform()
                qmat = QtGui.QMatrix4x4(
                    T[0,0], T[0,1], T[0,2], T[0,3],
                    T[1,0], T[1,1], T[1,2], T[1,3],
                    T[2,0], T[2,1], T[2,2], T[2,3],
                    T[3,0], T[3,1], T[3,2], T[3,3]
                )
                mesh_item.setTransform(qmat)
                self.viewer.addItem(mesh_item)
