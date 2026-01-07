from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QListWidget, QListWidgetItem, QAbstractItemView
from PyQt5.QtCore import pyqtSignal, Qt
from PyQt5.QtGui import QFont
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui
import numpy as np
from stl import mesh

class Viewer3DWidget(QWidget):
    """Widget pour la visualisation 3D avec PyQtGraph"""

    # Signaux
    transparency_toggled = pyqtSignal()
    axes_toggled = pyqtSignal()
    frame_visibility_toggled = pyqtSignal(int) # Nouveau signal avec l'index du repère

    def __init__(self, parent=None):
        super().__init__(parent)
        self.show_axes = True
        self.transparency_enabled = False
        self.robot_links = [] 
        self.setup_ui()

    def setup_ui(self):
        layout = QVBoxLayout(self)

        # Viewer 3D
        self.viewer = gl.GLViewWidget()
        self.viewer.opts['glOptions'] = 'translucent'
        self.viewer.opts['depth'] = True
        self.viewer.setCameraPosition(distance=2000, elevation=40, azimuth=45)
        self.viewer.setMinimumSize(900, 400)
        self.viewer.setBackgroundColor(45, 45, 48, 255)
        layout.addWidget(self.viewer)

        # --- LISTE DES REPÈRES (Overlay en haut à gauche) ---
        self.frame_list = QListWidget(self.viewer) # Parent = viewer pour l'overlay
        self.frame_list.setGeometry(10, 10, 150, 300) # Position et taille
        self.frame_list.setSelectionMode(QAbstractItemView.NoSelection)
        self.frame_list.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)

        self.frame_list.itemClicked.connect(self.on_frame_clicked)
        self.frame_list.hide() # Caché par défaut, affiché quand on a des données

        # Boutons de contrôle (Transparence / Repères global)
        toggle_layout = QHBoxLayout()
        self.btn_toggle_transparency = QPushButton("Transparence")
        self.btn_toggle_transparency.clicked.connect(self.transparency_toggled.emit)
        
        self.btn_toggle_axes = QPushButton("Masquer tous les Repères") # Renommé pour clarté
        self.btn_toggle_axes.clicked.connect(self.axes_toggled.emit)
        
        toggle_layout.addWidget(self.btn_toggle_transparency)
        toggle_layout.addWidget(self.btn_toggle_axes)
        layout.addLayout(toggle_layout)
        
        self.setLayout(layout)
        self.add_grid()

    def on_frame_clicked(self, item):
        """Gère le clic sur un élément de la liste"""
        index = self.frame_list.row(item)
        self.frame_visibility_toggled.emit(index)

    def update_frame_list_ui(self, visibility_list):
        """Met à jour l'apparence de la liste (Gras = Visible)"""
        count = len(visibility_list)
        
        # Si le nombre de repères a changé, on recrée la liste
        if self.frame_list.count() != count:
            self.frame_list.clear()
            for i in range(count):
                item = QListWidgetItem(f"Frame {i}")
                item.setTextAlignment(Qt.AlignLeft)
                self.frame_list.addItem(item)
            self.frame_list.show()

        # Mise à jour du style (Gras vs Normal)
        font_bold = QFont()
        font_bold.setBold(True)
        
        font_normal = QFont()
        font_normal.setBold(False)

        for i in range(count):
            item = self.frame_list.item(i)
            is_visible = visibility_list[i]
            
            # Appliquer le style (GRAS + GRIS CLAIR = Visible)
            if is_visible:
                item.setFont(font_bold)
                item.setForeground(Qt.lightGray)  # Gris clair en gras
            else:
                item.setFont(font_normal)
                item.setForeground(Qt.darkGray)  # Gris foncé en normal

    def add_grid(self):
        grid = gl.GLGridItem()
        grid.setSize(x=4000, y=4000, z=0)
        grid.setSpacing(x=200, y=200, z=200)
        grid.setColor((150, 150, 150, 100))
        self.viewer.addItem(grid)

    def clear_viewer(self):
        self.viewer.clear()
        self.add_grid()

    def draw_frame(self, T, longueur=100, color=None):
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

    def draw_all_frames(self, matrices, visibility_list):
        """Dessine les repères en fonction de leur visibilité individuelle"""
        for i, T in enumerate(matrices):
            # On dessine seulement si l'index est marqué visible dans la liste
            if i < len(visibility_list) and visibility_list[i]:
                self.draw_frame(T)

    
    def load_robot_mesh(self, stl_path, transform_matrix, color):
        # (Copier le code original ici, pas de changement)
        try:
            stl_mesh = mesh.Mesh.from_file(stl_path)
            verts = stl_mesh.vectors.reshape(-1, 3)
            faces = np.arange(len(verts)).reshape(-1, 3)
            mesh_data = gl.MeshData(vertexes=verts, faces=faces)
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

    def add_robot_links(self, matrices):
        # (Copier le code original ici)
        self.clear_robot_links()
        kuka_orange = (1.0, 0.4, 0.0, 0.5)
        kuka_black = (0.1, 0.1, 0.1, 0.5)
        kuka_grey = (0.5, 0.5, 0.5, 0.5)
        for i in range(min(7, len(matrices))):
            chemin_stl = f"./robot_stl/rocky{i}.stl"
            T = matrices[i]
            if i == 0: kuka_color = kuka_black
            elif i == 6: kuka_color = kuka_grey
            else: kuka_color = kuka_orange
            mesh_item = self.load_robot_mesh(chemin_stl, T, kuka_color)
            if mesh_item:
                self.robot_links.append(mesh_item)
                self.viewer.addItem(mesh_item)

    def update_robot_poses(self, matrices):
        for i in range(min(len(self.robot_links), len(matrices))):
            mesh_item = self.robot_links[i]
            T = matrices[i]
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
            self.viewer.removeItem(mesh_item)
        self.robot_links.clear()

    def set_robot_visibility(self, visible):
        for mesh_item in self.robot_links:
            if visible: mesh_item.show()
            else: mesh_item.hide()

    def set_transparency(self, enabled):
        self.transparency_enabled = enabled
        for mesh_item in self.robot_links:
            if enabled: mesh_item.setGLOptions('translucent')
            else: mesh_item.setGLOptions('opaque')

