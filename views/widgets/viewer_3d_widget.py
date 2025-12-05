from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5.QtCore import pyqtSignal
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtGui
import numpy as np
from stl import mesh

class Viewer3DWidget(QWidget):
    """Widget pour la visualisation 3D avec PyQtGraph"""
    
    # Signaux
    transparency_toggled = pyqtSignal()
    axes_toggled = pyqtSignal()
    prev_frame_requested = pyqtSignal()
    next_frame_requested = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.show_axes = True
        self.transparency_enabled = False
        self.robot_links = []  # Liste des mesh items du robot
        self.setup_ui()
    
    def setup_ui(self):
        """Initialise l'interface du widget"""
        layout = QVBoxLayout(self)
        
        # Viewer 3D
        self.viewer = gl.GLViewWidget()
        self.viewer.opts['glOptions'] = 'translucent'
        self.viewer.opts['depth'] = True
        self.viewer.setCameraPosition(distance=2000, elevation=40, azimuth=45)
        self.viewer.setMinimumSize(900, 400)
        self.viewer.setBackgroundColor(45, 45, 48, 255)  # Gris foncé
        layout.addWidget(self.viewer)
        
        # Boutons de contrôle
        toggle_layout = QHBoxLayout()
        self.btn_toggle_transparency = QPushButton("Transparence")
        self.btn_toggle_transparency.clicked.connect(self.transparency_toggled.emit)
        self.btn_toggle_axes = QPushButton("Repères")
        self.btn_toggle_axes.clicked.connect(self.axes_toggled.emit)
        toggle_layout.addWidget(self.btn_toggle_transparency)
        toggle_layout.addWidget(self.btn_toggle_axes)
        layout.addLayout(toggle_layout)
        
        # Boutons de navigation (pour mode pas à pas)
        nav_layout = QHBoxLayout()
        self.btn_prev = QPushButton("Repère précédent")
        self.btn_prev.clicked.connect(self.prev_frame_requested.emit)
        self.btn_prev.setVisible(False)
        self.btn_next = QPushButton("Repère suivant")
        self.btn_next.clicked.connect(self.next_frame_requested.emit)
        self.btn_next.setVisible(False)
        nav_layout.addWidget(self.btn_prev)
        nav_layout.addWidget(self.btn_next)
        layout.addLayout(nav_layout)
        
        self.setLayout(layout)
        
        # Ajouter la grille au démarrage
        self.add_grid()
    
    def add_grid(self):
        """Ajoute une grille quadrillée au sol selon les axes X et Y"""
        grid = gl.GLGridItem()
        grid.setSize(x=4000, y=4000, z=0)  # Taille de la grille en mm
        grid.setSpacing(x=200, y=200, z=200)  # Espacement des lignes en mm
        grid.setColor((150, 150, 150, 100))  # Couleur grise semi-transparente
        self.viewer.addItem(grid)
    
    def clear_viewer(self):
        """Efface le viewer (sauf la grille qui sera rajoutée)"""
        self.viewer.clear()
        self.add_grid()
    
    def draw_frame(self, T, longueur=100, color=None):
        """
        Dessine un repère orienté selon la matrice homogène T
        
        Args:
            T: matrice 4x4 (rotation + translation)
            longueur: taille des axes
            color: tuple RGBA pour les 3 axes (si None, utilise RGB par défaut)
        """
        origine = T[:3, 3]  # Position
        R = T[:3, :3]  # Rotation (3x3)
        
        # Calcul des extrémités des axes en tenant compte de la rotation
        axes = [
            np.array([origine, origine + R[:, 0] * longueur]),  # Axe X
            np.array([origine, origine + R[:, 1] * longueur]),  # Axe Y
            np.array([origine, origine + R[:, 2] * longueur])   # Axe Z
        ]
        
        if color is None:
            # Couleurs par défaut: X=Rouge, Y=Vert, Z=Bleu
            couleurs = [(255, 0, 0, 1), (0, 255, 0, 1), (0, 0, 255, 1)]
        else:
            # Couleur unique pour tous les axes
            couleurs = [color, color, color]
        
        for i, axis in enumerate(axes):
            plt = gl.GLLinePlotItem(pos=axis, color=couleurs[i], width=3, antialias=True)
            self.viewer.addItem(plt)
    
    def draw_all_frames(self, matrices):
        """Dessine tous les repères à partir d'une liste de matrices"""
        for T in matrices:
            self.draw_frame(T)
    
    def draw_highlighted_frame(self, T, longueur=100):
        """Dessine un repère en jaune (pour le mode pas à pas)"""
        self.draw_frame(T, longueur, color=(1, 1, 0, 1))
    
    def load_robot_mesh(self, stl_path, transform_matrix, color):
        """
        Charge un segment du robot depuis un fichier STL
        
        Args:
            stl_path: chemin vers le fichier STL
            transform_matrix: matrice de transformation 4x4
            color: tuple RGBA de la couleur
        
        Returns:
            mesh_item ou None si erreur
        """
        try:
            # Charger le STL avec numpy-stl
            stl_mesh = mesh.Mesh.from_file(stl_path)
            
            # Extraire les sommets et faces
            verts = stl_mesh.vectors.reshape(-1, 3)
            faces = np.arange(len(verts)).reshape(-1, 3)
            
            # Créer MeshData
            mesh_data = gl.MeshData(vertexes=verts, faces=faces)
            
            # Créer l'objet 3D
            mesh_item = gl.GLMeshItem(meshdata=mesh_data, smooth=True, color=color, shader='shaded')
            
            # Convertir la matrice numpy (4x4) en QMatrix4x4
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
            print(f"Erreur lors de l'import du segment STL {stl_path}: {e}")
            return None
    
    def add_robot_links(self, matrices):
        """
        Charge et affiche tous les segments du robot
        
        Args:
            matrices: liste des matrices de transformation pour chaque segment
        """
        # Nettoyer les anciens mesh
        self.clear_robot_links()
        
        # Couleurs KUKA
        kuka_orange = (1.0, 0.4, 0.0, 0.5)
        kuka_black = (0.1, 0.1, 0.1, 0.5)
        kuka_grey = (0.5, 0.5, 0.5, 0.5)
        
        for i in range(min(7, len(matrices))):
            chemin_stl = f"./robot_stl/rocky{i}.stl"
            T = matrices[i]
            
            # Choisir la couleur selon le segment
            if i == 0:
                kuka_color = kuka_black
            elif i == 6:
                kuka_color = kuka_grey
            else:
                kuka_color = kuka_orange
            
            mesh_item = self.load_robot_mesh(chemin_stl, T, kuka_color)
            if mesh_item:
                self.robot_links.append(mesh_item)
                self.viewer.addItem(mesh_item)
    
    def update_robot_poses(self, matrices):
        """Met à jour les poses de tous les segments du robot"""
        for i in range(min(len(self.robot_links), len(matrices))):
            mesh_item = self.robot_links[i]
            T = matrices[i]
            
            if mesh_item:
                mesh_item.resetTransform()
                
                # Convertir la matrice numpy (4x4) en QMatrix4x4
                qmat = QtGui.QMatrix4x4(
                    T[0,0], T[0,1], T[0,2], T[0,3],
                    T[1,0], T[1,1], T[1,2], T[1,3],
                    T[2,0], T[2,1], T[2,2], T[2,3],
                    T[3,0], T[3,1], T[3,2], T[3,3]
                )
                mesh_item.setTransform(qmat)
                self.viewer.addItem(mesh_item)
    
    def clear_robot_links(self):
        """Supprime tous les mesh items du robot"""
        for mesh_item in self.robot_links:
            self.viewer.removeItem(mesh_item)
        self.robot_links.clear()
    
    def set_robot_visibility(self, visible):
        """Bascule la visibilité du robot"""
        for mesh_item in self.robot_links:
            if visible:
                mesh_item.show()
            else:
                mesh_item.hide()
    
    def set_transparency(self, enabled):
        """Active/désactive la transparence des mesh"""
        self.transparency_enabled = enabled
        for mesh_item in self.robot_links:
            if enabled:
                mesh_item.setGLOptions('translucent')
            else:
                mesh_item.setGLOptions('opaque')
    
    def set_navigation_buttons_visible(self, visible):
        """Affiche/masque les boutons de navigation pas à pas"""
        self.btn_prev.setVisible(visible)
        self.btn_next.setVisible(visible)
