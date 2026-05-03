from PyQt6.QtCore import QObject, pyqtSignal
from typing import List, Tuple
import math
from utils.mgi import *
import utils.math_utils as math_utils
from models.types import FkResult, Pose6, XYZ3
from models.collider_models import (
    default_axis_colliders,
)
from models.primitive_collider_models import (
    RobotAxisColliderData,
)
from models.robot_configuration_file import RobotConfigurationFile

class RobotModel(QObject):
    """Modele centralise contenant tous les parametres et l'etat du robot"""

    DEFAULT_AXIS_LIMITS: List[Tuple[float, float]] = [
        (-170.0, 170.0),
        (-190.0, 45.0),
        (-120.0, 156.0),
        (-185.0, 185.0),
        (-120.0, 120.0),
        (-350.0, 350.0),
    ]
    UNCONFIGURED_AXIS_LIMITS: List[Tuple[float, float]] = [(0.0, 0.0) for _ in range(6)]
    DEFAULT_AXIS_SPEED_LIMITS: List[float] = [300.0, 225.0, 255.0, 381.0, 311.0, 492.0]
    UNCONFIGURED_AXIS_SPEED_LIMITS: List[float] = [0.0] * 6
    # Estimated defaults (deg/s^3), can be refined from real traces.
    DEFAULT_AXIS_JERK_LIMITS: List[float] = [6000.0, 5000.0, 5000.0, 7500.0, 6500.0, 9000.0]
    UNCONFIGURED_AXIS_JERK_LIMITS: List[float] = [0.0] * 6
    DEFAULT_AXIS_ACCEL_LIMITS: List[float] = [
        round(math.sqrt(speed * jerk), 3)
        for speed, jerk in zip(DEFAULT_AXIS_SPEED_LIMITS, DEFAULT_AXIS_JERK_LIMITS)
    ]
    UNCONFIGURED_AXIS_ACCEL_LIMITS: List[float] = [0.0] * 6
    DEFAULT_AXIS_COLLIDERS: List[RobotAxisColliderData] = default_axis_colliders(6)
    UNCONFIGURED_AXIS_COLLIDERS: List[RobotAxisColliderData] = [
        collider.copy() for collider in DEFAULT_AXIS_COLLIDERS
    ]
    DEFAULT_CARTESIAN_SLIDER_LIMITS_XYZ: List[Tuple[float, float]] = [
        (-1000.0, 1000.0),
        (-1000.0, 1000.0),
        (-1000.0, 1000.0),
    ]
    UNCONFIGURED_CARTESIAN_SLIDER_LIMITS_XYZ: List[Tuple[float, float]] = list(DEFAULT_CARTESIAN_SLIDER_LIMITS_XYZ)
    DEFAULT_ROBOT_CAD_MODELS: List[str] = [""] * 7
    DEFAULT_HOME_POSITION: List[float] = [0.0, -90.0, 90.0, 0.0, 90.0, 0.0]
    UNCONFIGURED_HOME_POSITION: List[float] = [0.0] * 6
    POSITION_ZERO: List[float] = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
    UNCONFIGURED_POSITION_ZERO: List[float] = [0.0] * 6
    POSITION_CALIBRATION: List[float] = [0.0, -105.0, 156.0, 0.0, 120.0, 0.0]
    UNCONFIGURED_POSITION_CALIBRATION: List[float] = [0.0] * 6
    
    # ============================================================================
    # SIGNAUX
    # ============================================================================
    
    # Configuration générale
    configuration_changed = pyqtSignal()
    robot_name_changed = pyqtSignal(str)
    
    # Paramètres DH
    dh_params_changed = pyqtSignal()
    measured_dh_params_changed = pyqtSignal()
    measured_dh_enabled_changed = pyqtSignal()

    allowed_config_changed = pyqtSignal()

    cad_models_changed = pyqtSignal()
    robot_cad_models_changed = pyqtSignal()
    axis_colliders_changed = pyqtSignal()

    # Joints et axes
    joints_changed = pyqtSignal()
    axis_reversed_changed = pyqtSignal()
    axis_limits_changed = pyqtSignal()
    cartesian_slider_limits_changed = pyqtSignal()
    axis_speed_limits_changed = pyqtSignal()
    axis_accel_limits_changed = pyqtSignal()
    axis_jerk_limits_changed = pyqtSignal()
    joint_weights_changed = pyqtSignal()
    
    # Corrections
    corrections_changed = pyqtSignal()
    
    # Résultats
    tcp_pose_changed = pyqtSignal()
    
    # Mesures
    measurements_changed = pyqtSignal()
    measurements_points_changed = pyqtSignal()
    
    def __init__(self, parent: QObject = None):
        super().__init__(parent)
        
        # ====================================================================
        # RÉGION: Configuration générale
        # ====================================================================
        self.robot_name = ""
        self.has_configuration = False
        self.current_config_file = None

        # ====================================================================
        # RÉGION: Paramètres des joints et axes
        # ====================================================================
        # Limites des axes (min, max) pour chaque joint
        self.axis_limits: List[Tuple[float, float]] = list(RobotModel.UNCONFIGURED_AXIS_LIMITS)
        self.cartesian_slider_limits_xyz: List[Tuple[float, float]] = list(
            RobotModel.UNCONFIGURED_CARTESIAN_SLIDER_LIMITS_XYZ
        )
        self.axis_speed_limits: List[float] = list(RobotModel.UNCONFIGURED_AXIS_SPEED_LIMITS)
        self.axis_accel_limits: List[float] = list(RobotModel.UNCONFIGURED_AXIS_ACCEL_LIMITS)
        self.axis_jerk_limits: List[float] = list(RobotModel.UNCONFIGURED_AXIS_JERK_LIMITS)
        self.robot_cad_models: List[str] = list(RobotModel.DEFAULT_ROBOT_CAD_MODELS)
        self.axis_colliders: List[RobotAxisColliderData] = [
            collider.copy() for collider in RobotModel.UNCONFIGURED_AXIS_COLLIDERS
        ]
        self._axis_colliders_revision: int = 0
               
        # Position home du robot
        self.home_position: List[float] = list(RobotModel.UNCONFIGURED_HOME_POSITION)
        self.position_zero: List[float] = list(RobotModel.UNCONFIGURED_POSITION_ZERO)
        self.position_calibration: List[float] = list(RobotModel.UNCONFIGURED_POSITION_CALIBRATION)
        
        # Valeurs actuelles des joints (en degrés)
        self.joint_values: List[float] = [0, 0, 0, 0, 0, 0]
        self.joint_values_not_inverted: List[float] = [0, 0, 0, 0, 0, 0]
        
        # Multiplicateurs d'axes (1 = normal, -1 = inversé)
        self.axis_reversed: List[int] = [1, 1, 1, 1, 1, 1]
        
        # Poids des joints pour la sélection de la meilleure solution MGI
        self.joint_weights: List[float] = [1.0] * 6
        
        # ====================================================================
        # RÉGION: Paramètres Denavit-Hartenberg
        # ====================================================================
        # 7 lignes : 6 joints + outil (tool frame)
        # Chaque ligne contient [a, alpha, d, theta]
        self.dh_params: List[List[float]] = [[0, 0, 0, 0] for _ in range(7)]
        self.measured_dh_params: List[List[float]] = [[0, 0, 0, 0] for _ in range(6)]
        self.measured_dh_enabled: bool = False
        
        self.current_tcp_dh_matrices: List[np.ndarray] = []
        self.current_tcp_corrected_dh_matrices: List[np.ndarray] = []

        # ====================================================================
        # REGION: MGI
        # ====================================================================

        self.mgi_kuka_config_identifier = KukaConfigurationIdentifier()

        self.mgi_params = MgiParams(
            self.mgi_kuka_config_identifier, 
            RobotModel._mgi_build_geometric_params(self.dh_params), 
            RobotModel._mgi_build_invert_table(self.axis_reversed), 
            RobotModel._mgi_build_axis_limits(self.axis_limits), 
            MgiSingularitiesBehavior(MgiSingularityBehavior.CONTINUE),
            MgiConfigurationFilter.allow_all())
        
        self.MGI_solver = MGI(self.mgi_params, RobotTool())

        self.current_tcp_mgi_result: MgiResult = MgiResult()
        self.current_axis_config: MgiConfigKey = MgiConfigKey.FUN

        # ====================================================================
        # RÉGION: Résultats cinématique
        # ====================================================================
        # Pose TCP non corrigée [X, Y, Z, Rx, Ry, Rz]
        self.tcp_pose: Pose6 = Pose6.zeros()
        
        # Pose TCP corrigée (appliquant les corrections)
        self.corrected_tcp_pose: Pose6 = Pose6.zeros()
        
        # Déviation entre TCP et TCP corrigé
        self.pose_deviation: Pose6 = Pose6.zeros()
        
        self._tcp_rotation_matrix: list[list[float]] = []

        # ====================================================================
        # RÉGION: Corrections 6D
        # ====================================================================
        # 6 lignes pour 6 joints, 6 colonnes pour 6 DDL (X, Y, Z, Rx, Ry, Rz)
        self.corrections: List[List[float]] = [[0, 0, 0, 0, 0, 0] for _ in range(6)]

        # ====================================================================
        # RÉGION: Mesures et points de mesure
        # ====================================================================
        # Liste des mesures enregistrées

        self.measurements_filename: str = ""

        self.measurements: List[float] = []
        
        # Points de mesure (positions de référence)
        self.measurement_points: List[List[float]] = []

        self._user_inhibit_compute_fk = False
        self._inhibit_compute_fk = False
        self.current_tool = RobotTool()
        self.current_tool_transform = RobotModel.build_tool_transform(self.current_tool)

    # ====================================================================
    # RÉGION: MGI Fonctions utilitaires
    # ====================================================================

    @staticmethod
    def _mgi_build_axis_limits(axis_limits: list[tuple[float, float]]):
        return MgiAxisLimits(False, axis_limits)

    @staticmethod
    def _mgi_build_invert_table(axis_reversed: list[int]):
        return [b == -1 for b in axis_reversed]

    @staticmethod
    def _mgi_build_geometric_params(dh_table: List[List[float]]) -> MgiGeometricParams:
        """
        Construit les paramètres géométriques du MGI à partir
        d'une table DH standard [a, alpha, d, theta].

        Hypothèse :
        - Robot 6 axes anthropomorphique
        - dh_table contient au moins 6 lignes
        """

        if len(dh_table) < 6:
            raise ValueError("La table DH doit contenir au moins 6 lignes")

        # Raccourcis de lecture
        # dh_table[i] = [a_i, alpha_i, d_i, theta_i]

        # Base  Axe 1
        r1 = dh_table[0][3]   # a1

        # Axe 2
        d2 = dh_table[1][1]   # d2

        # Axe 3
        d3 = dh_table[2][1]   # d3

        # Axe 4
        d4 = dh_table[3][1]   # d4
        r4 = dh_table[3][3]   # a4

        # Axe 6 / flange
        r6 = dh_table[5][3]   # d6

        return MgiGeometricParams(r1, d2, d3, d4, r4, r6)
    
    def get_config_identifier(self):
        return self.mgi_kuka_config_identifier
    
    def get_current_axis_config(self):
        return self.current_axis_config

    def _update_current_axis_config(self):
        self.current_axis_config = MgiConfigKey.identify_configuration_deg(self.joint_values_not_inverted, self.mgi_kuka_config_identifier)

    def set_mgi_configuration_filter(self, config_filter: MgiConfigurationFilter) -> None:
        """Définit le filtre de configuration MGI"""
        self.MGI_solver.set_configuration_filter(config_filter)
        self.allowed_config_changed.emit()
        self._update_tcp_pose()
    
    def get_mgi_configuration_filter(self) -> MgiConfigurationFilter:
        """Récupère le filtre de configuration MGI actuel"""
        return self.MGI_solver.get_configuration_filter()
    
    def set_allowed_configurations(self, allowed_configs: set[MgiConfigKey]) -> None:
        """Définit les configurations autorisées à partir d'un ensemble"""
        config_filter = MgiConfigurationFilter(allowed_configs) if allowed_configs else MgiConfigurationFilter.allow_all()
        self.set_mgi_configuration_filter(config_filter)
    
    def get_allowed_configurations(self) -> set[MgiConfigKey]:
        """Récupère l'ensemble des configurations autorisées"""
        filter = self.get_mgi_configuration_filter()
        if filter.allowed_configs is None:
            # Toutes les configs sont autorisées
            return set(MgiConfigKey)
        return filter.allowed_configs.copy()
    
    def get_configuration_states(self) -> dict[MgiConfigKey, bool]:
        """Récupère l'état de chaque configuration (autorisée ou non)"""
        allowed = self.get_allowed_configurations()
        return {key: key in allowed for key in MgiConfigKey}

    # ====================================================================
    # RÉGION: Inverse Kinematics
    # ====================================================================

    def compute_ik_optimise(
        self,
        target: Pose6,
        q_initial: list[float],
        params=None,
        tool: RobotTool | None = None,
    ):
        """
        Solveur MGI optimisé par la Jacobienne inverse (Levenberg-Marquardt).

        Affine itérativement q_initial en utilisant le MGD CORRIGÉ pour tenir compte
        de toutes les corrections de calibration (Tx, Ty, Tz, Rx, Ry, Rz par joint).

        Args:
            target:    Pose cible [x, y, z, a, b, c] en mm et degrés (ZYX Euler)
            q_initial: Estimation initiale en degrés (issue du MGI analytique)
            params:    MgiJacobienParams (None → valeurs par défaut)

        Returns:
            MgiJacobienResultat avec joints raffinés et métriques de convergence
        """
        from utils.mgi_jacobien import mgi_jacobien, MgiJacobienParams
        if not isinstance(target, Pose6):
            raise TypeError("target must be a Pose6")
        if params is None:
            params = MgiJacobienParams()
        return mgi_jacobien(target.to_list(), self, q_initial, params, tool=tool)

    def compute_ik(
        self,
        x: float,
        y: float,
        z: float,
        a: float,
        b: float,
        c: float,
        tool: RobotTool | None = None,
    ):
        self.MGI_solver.set_tool(self._resolve_tool(tool))
        self.MGI_solver.set_q1ValueIfSingularityQ1(self.joint_values[0])
        self.MGI_solver.set_q4ValueIfSingularityQ5(self.joint_values[4])
        self.MGI_solver.set_q6ValueIfSingularityQ5(self.joint_values[5])
        return self.MGI_solver.compute_mgi(x, y, z, a, b, c)

    def compute_ik_target(self, target: Pose6, tool: RobotTool | None = None):
        if not isinstance(target, Pose6):
            raise TypeError("target must be a Pose6")
        return self.compute_ik(target.x, target.y, target.z, target.a, target.b, target.c, tool=tool)

    def get_best_mgi_solution(self, mgi_result: MgiResult):
        joints_rad = [math.radians(q) for q in self.joint_values]
        return mgi_result.get_best_solution_from_current(joints_rad, self.joint_weights)
    
    # ====================================================================
    # RÉGION: Forward Kinematics
    # ====================================================================

    @staticmethod
    def build_tool_transform(tool: RobotTool | None = None):
        current_tool = tool if tool is not None else RobotTool()
        return math_utils.pose_zyx_to_matrix(
            Pose6(
                current_tool.x,
                current_tool.y,
                current_tool.z,
                current_tool.a,
                current_tool.b,
                current_tool.c,
            )
        )

    @staticmethod
    def _copy_tool(tool: RobotTool | None = None) -> RobotTool:
        source_tool = tool if tool is not None else RobotTool()
        return RobotTool(
            source_tool.x,
            source_tool.y,
            source_tool.z,
            source_tool.a,
            source_tool.b,
            source_tool.c,
        )

    def set_tool(self, tool: RobotTool | None = None) -> None:
        new_tool = RobotModel._copy_tool(tool)
        self.current_tool = new_tool
        self.current_tool_transform = RobotModel.build_tool_transform(new_tool)
        self.MGI_solver.set_tool(new_tool)
        self._update_tcp_pose()

    def _resolve_tool(self, tool: RobotTool | None = None) -> RobotTool:
        if tool is not None:
            return tool
        return self.current_tool

    def _resolve_tool_transform(self, tool: RobotTool | None = None) -> np.ndarray:
        if tool is not None:
            return RobotModel.build_tool_transform(tool)
        return self.current_tool_transform.copy()

    def compute_fk(
        self,
        q1: float,
        q2: float,
        q3: float,
        q4: float,
        q5: float,
        q6: float,
        tool: RobotTool | None = None,
    ) -> FkResult:
        """Calcule le MGD avec la liste complète des matrices de transformation
    
        Args:
            robot_model: RobotModel instance
        
        Returns:
            Tuple (dh_matrices, corrected_matrices, dh_pose, corrected_pose, deviation)
            - dh_matrices: List de matrices 4x4 (sans correction)
            - corrected_matrices: List de matrices 4x4 (avec corrections)
            - dh_pose: Array [x, y, z, rx, ry, rz] sans correction
            - corrected_pose: Array [x, y, z, rx, ry, rz] avec correction
            - deviation: Array des écarts
        """
        dh_matrices = [np.eye(4)]
        corrected_matrices = [np.eye(4)]
        
        T_dh = np.eye(4)
        T_corrected = np.eye(4)

        compute_joints = [
            q1 * self.axis_reversed[0],
            q2 * self.axis_reversed[1],
            q3 * self.axis_reversed[2],
            q4 * self.axis_reversed[3],
            q5 * self.axis_reversed[4],
            q6 * self.axis_reversed[5]
        ]
        
        # Calcul itératif des transformations pour 6 joints + outil
        for i in range(6):
            # Récupérer les paramètres DH
            alpha = np.radians(self.get_dh_param(i, 0))
            d = self.get_dh_param(i, 1)
            theta_offset = np.radians(self.get_dh_param(i, 2))
            r = self.get_dh_param(i, 3)

            q_deg = compute_joints[i]
            q = np.radians(q_deg)
            theta = theta_offset + q
            corr = self.get_correction_joint(i)
            
            # Transformation DH standard
            T_dh = T_dh @ math_utils.dh_modified(alpha, d, theta, r)
            dh_matrices.append(T_dh.copy())
            
            # Transformation avec correction
            T_corrected = T_corrected @ math_utils.dh_modified(alpha, d, theta, r)
            T_corrected = math_utils.correction_6d(T_corrected, *corr)
            corrected_matrices.append(T_corrected.copy())
        
        tool_transform = self._resolve_tool_transform(tool)
        T_dh = T_dh @ tool_transform
        dh_matrices.append(T_dh.copy())

        T_corrected = T_corrected @ tool_transform
        T_corrected = math_utils.correction_6d(T_corrected, 0, 0, 0, 0, 0, 0)
        corrected_matrices.append(T_corrected.copy())

        # Extraction position et orientation
        dh_pos = T_dh[:3, 3]
        dh_ori = math_utils.matrix_to_euler_zyx(T_dh)
        dh_pose = Pose6(dh_pos[0], dh_pos[1], dh_pos[2], dh_ori[0], dh_ori[1], dh_ori[2])
        
        corrected_pos = T_corrected[:3, 3]
        corrected_ori = math_utils.matrix_to_euler_zyx(T_corrected)
        corrected_pose = Pose6(
            corrected_pos[0],
            corrected_pos[1],
            corrected_pos[2],
            corrected_ori[0],
            corrected_ori[1],
            corrected_ori[2],
        )
        
        # Calcul de la déviation
        pos_dev = corrected_pos - dh_pos
        ori_dev = corrected_ori - dh_ori
        deviation = Pose6(pos_dev[0], pos_dev[1], pos_dev[2], ori_dev[0], ori_dev[1], ori_dev[2])
        
        return FkResult(
            dh_matrices=dh_matrices,
            corrected_matrices=corrected_matrices,
            dh_pose=dh_pose,
            corrected_pose=corrected_pose,
            deviation=deviation,
        )

    def compute_fk_joints(self, joints: list[float], tool: RobotTool | None = None) -> FkResult | None:
        if len(joints) < 6:
            return None
        return self.compute_fk(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], tool=tool)

    def inhibit_auto_compute_fk_tcp(self, inhibit: bool):
        self._user_inhibit_compute_fk = inhibit

    def compute_fk_tcp(self, tool: RobotTool | None = None):
        self._update_tcp_pose(tool=tool)

    def _update_tcp_pose(self, tool: RobotTool | None = None):
        if self._inhibit_compute_fk or self._user_inhibit_compute_fk or not self.has_configuration:
            return
        
        # update current axis config
        self._update_current_axis_config()
        # update TCP from FK

        fk_result = self.compute_fk_joints(
            self.joint_values,
            tool=tool,
        )
        if fk_result is None:
            return

        self.current_tcp_dh_matrices = fk_result.dh_matrices
        self.current_tcp_corrected_dh_matrices = fk_result.corrected_matrices

        self._set_tcp_pose(fk_result.dh_pose)
        self._set_corrected_tcp_pose(fk_result.corrected_pose, True)

        self._tcp_rotation_matrix = math_utils.euler_to_rotation_matrix(
            self.tcp_pose.a,
            self.tcp_pose.b,
            self.tcp_pose.c,
        )

        # update MGI for current TCP
        self.current_tcp_mgi_result = self.compute_ik_target(self.tcp_pose, tool=tool)

        # emit quand tout a été mis à jour
        self.tcp_pose_changed.emit()
    
    def get_current_tcp_dh_matrices(self):
        return self.current_tcp_dh_matrices

    def get_current_tcp_corrected_dh_matrices(self):
        return self.current_tcp_corrected_dh_matrices

    def get_current_tcp_mgi_result(self):
        return self.current_tcp_mgi_result

    def get_tcp_rotation_matrix(self):
        return self._tcp_rotation_matrix

    # ============================================================================
    # RÉGION: Tool
    # ============================================================================
    
    def get_T_tool(self, tool: RobotTool | None = None):
        return self._resolve_tool_transform(tool)

    def get_robot_cad_models(self) -> list[str]:
        """Retourne les chemins CAD du robot (base + axes)."""
        return [str(path) for path in self.robot_cad_models]

    def set_robot_cad_models(self, cad_models: list[str]) -> None:
        """
        Définit les chemins CAD du robot (base + axes).
        Les entrées vides sont conservées pour permettre de masquer certains maillages.
        """
        if not isinstance(cad_models, list):
            return
        normalized = [str(path) for path in cad_models]
        if normalized == self.robot_cad_models:
            return
        self.robot_cad_models = normalized
        self.robot_cad_models_changed.emit()
        self.cad_models_changed.emit()

    def get_axis_colliders(self) -> list[RobotAxisColliderData]:
        return self.get_axis_collider_data()

    def get_axis_collider_data(self) -> list[RobotAxisColliderData]:
        return [collider.copy() for collider in self.axis_colliders]

    def get_axis_colliders_revision(self) -> int:
        return int(self._axis_colliders_revision)

    def set_axis_colliders(
        self,
        axis_colliders: list[RobotAxisColliderData],
    ) -> None:
        if not all(isinstance(collider, RobotAxisColliderData) for collider in axis_colliders):
            raise TypeError("axis_colliders must contain RobotAxisColliderData")
        if len(axis_colliders) != 6:
            raise ValueError("axis_colliders must contain 6 values")
        normalized = [collider.copy() for collider in axis_colliders[:6]]
        if normalized == self.axis_colliders:
            return
        self.axis_colliders = normalized
        self._axis_colliders_revision += 1
        self.axis_colliders_changed.emit()

    # ============================================================================
    # RÉGION: Getters - Configuration générale
    # ============================================================================
    
    def get_robot_name(self) -> str:
        """Retourne le nom du robot"""
        return self.robot_name
    
    def get_has_configuration(self) -> bool:
        """Retourne True si le robot a une configuration chargée"""
        return self.has_configuration

    def get_current_config_file(self) -> str:
        """Retourne le chemin du fichier de configuration actuel"""
        return self.current_config_file
    
    # ============================================================================
    # RÉGION: Setters - Configuration générale
    # ============================================================================
    
    def set_robot_name(self, name: str):
        """Définit le nom du robot"""
        self.robot_name = name
        self.robot_name_changed.emit(name)
        
    # ============================================================================
    # RÉGION: Home position
    # ============================================================================

    def get_home_position(self):
        """Retourne la position home"""
        return self.home_position.copy()

    def set_home_position(self, home_pos: list[float]):
        """Définit la position home"""
        if len(home_pos) >= 6:
            self.home_position = list(home_pos[:6])
    
    def go_to_home_position(self):
        """Move joints to home position."""
        self.set_joints(self.home_position)

    def get_position_zero(self):
        """Retourne la position 0 (fixe)"""
        return self.position_zero.copy()

    def set_position_zero(self, position_zero: list[float]):
        """Définit la position 0"""
        if len(position_zero) >= 6:
            self.position_zero = list(position_zero[:6])

    def go_to_position_zero(self):
        """Déplace les joints à la position 0 (fixe)"""
        self.set_joints(self.position_zero)

    def get_position_calibration(self):
        """Retourne la position calibration (fixe)"""
        return self.position_calibration.copy()

    def set_position_calibration(self, position_calibration: list[float]):
        """Définit la position calibration"""
        if len(position_calibration) >= 6:
            self.position_calibration = list(position_calibration[:6])

    def go_to_position_calibration(self):
        """Déplace les joints à la position de calibration (fixe)"""
        self.set_joints(self.position_calibration)

    # ============================================================================
    # RÉGION: Getters - Joints et axes
    # ============================================================================
    
    def get_joint(self, index: int) -> float:
        """Retourne la valeur d'un joint spécifique ou 0 si l'index est invalide"""
        return self.joint_values[index] if 0 <= index < 6 else 0
    
    def get_joints(self):
        """Retourne toutes les valeurs des joints"""
        return self.joint_values.copy()

    def get_axis_limit(self, index: int):
        """Retourne les limites d'un axe spécifique (min, max)"""
        return self.axis_limits[index] if 0 <= index < 6 else (-180, 180)
    
    def get_axis_limits(self):
        """Retourne les limites de tous les axes"""
        return self.axis_limits.copy()

    def get_cartesian_slider_limits_xyz(self) -> list[tuple[float, float]]:
        """Retourne les limites XYZ utilisees par les sliders cartesiens."""
        return self.cartesian_slider_limits_xyz.copy()

    def get_axis_speed_limit(self, index: int) -> float:
        """Retourne la limite de vitesse d'un axe spécifique en deg/s"""
        return self.axis_speed_limits[index] if 0 <= index < 6 else 0.0

    def get_axis_speed_limits(self):
        """Retourne les limites de vitesse de tous les axes en deg/s"""
        return self.axis_speed_limits.copy()

    def get_axis_accel_limit(self, index: int) -> float:
        """Retourne la limite d'acceleration d'un axe specifique en deg/s^2"""
        return self.axis_accel_limits[index] if 0 <= index < 6 else 0.0

    def get_axis_accel_limits(self):
        """Retourne les limites d'acceleration de tous les axes en deg/s^2"""
        return self.axis_accel_limits.copy()

    def get_axis_jerk_limit(self, index: int) -> float:
        """Retourne la limite de jerk d'un axe spécifique en deg/s^3"""
        return self.axis_jerk_limits[index] if 0 <= index < 6 else 0.0

    def get_axis_jerk_limits(self):
        """Retourne les limites de jerk de tous les axes en deg/s^3"""
        return self.axis_jerk_limits.copy()

    def get_axis_estimated_accel_limit(self, index: int) -> float:
        """
        Retourne l'accélération maximale estimée pour un axe (deg/s^2),
        via alpha_max ~= sqrt(v_max * jerk_max).
        """
        if not (0 <= index < 6):
            return 0.0
        speed = max(0.0, float(self.axis_speed_limits[index]))
        jerk = max(0.0, float(self.axis_jerk_limits[index]))
        return math.sqrt(speed * jerk)

    def get_axis_estimated_accel_limits(self) -> list[float]:
        """Retourne les accélérations maximales estimées de tous les axes (deg/s^2)."""
        return [self.get_axis_estimated_accel_limit(i) for i in range(6)]

    def get_axis_reversed_single(self, index: int) -> bool:
        """Retourne True si l'axe est inversé"""
        return self.axis_reversed[index] == -1 if 0 <= index < 6 else False

    def get_axis_reversed(self):
        """Retourne les multiplicateurs d'axes"""
        return self.axis_reversed.copy()

    def get_joint_weights(self):
        """Retourne les poids des joints"""
        return self.joint_weights.copy()

    def set_joint_weights(self, joint_weights: list[float]):
        """Définit les poids des joints"""
        if len(joint_weights) >= 6:
            self.joint_weights = list(joint_weights[:6])
            self.joint_weights_changed.emit()

    def set_joint_weights_single(self, index: int, weight: float):
        """Définit le poids d'un joint spécifique"""
        if 0 <= index < 6:
            self.joint_weights[index] = weight
            self.joint_weights_changed.emit()

    # ============================================================================
    # RÉGION: Setters - Joints et axes
    # ============================================================================
    
    def set_joint(self, index: int, value: float):
        """Modifie la valeur d'un joint spécifique"""
        if 0 <= index < 6:
            self._set_joint_idx(index, float(value))
            self.joints_changed.emit()
            self._update_tcp_pose()
    
    def set_joints(self, values: list[float]):
        """Définit toutes les valeurs des joints"""
        if len(values) >= 6:
            for i in range(6):
                self._set_joint_idx(i, float(values[i]))

            self.joints_changed.emit()
            self._update_tcp_pose()
    
    def _set_joint_idx(self, index: int, value: float):
        self.joint_values[index] = value
        self.joint_values_not_inverted[index] = value * self.axis_reversed[index]

    def _set_joints_from_best_sol(self, mgi_result: MgiResultItem):
        """Définit toutes les valeurs des joints sans émettre de signal ni mettre à jour la pose"""
        for i in range(6):
            self._set_joint_idx(i, float(mgi_result.joints[i]))

    def set_axis_limit(self, index: int, min_val: float, max_val: float):
        """Définit les limites d'un axe spécifique"""
        if 0 <= index < 6:
            self.axis_limits[index] = (min_val, max_val)
            self.MGI_solver.set_axis_limits(RobotModel._mgi_build_axis_limits(self.axis_limits))
            self.axis_limits_changed.emit()
            self._update_tcp_pose()

    def set_axis_limits(self, limits: List[Tuple[float, float]]):
        """Définit les limites de tous les axes"""
        self.axis_limits = limits
        self.MGI_solver.set_axis_limits(RobotModel._mgi_build_axis_limits(self.axis_limits))
        self.axis_limits_changed.emit()
        self._update_tcp_pose()

    def set_cartesian_slider_limits_xyz(self, limits: list[tuple[float, float]]):
        """Définit les limites XYZ utilisees par les sliders cartesiens."""
        normalized: list[tuple[float, float]] = []
        for index in range(3):
            default_min, default_max = RobotModel.DEFAULT_CARTESIAN_SLIDER_LIMITS_XYZ[index]
            if index < len(limits):
                try:
                    min_val = float(limits[index][0])
                    max_val = float(limits[index][1])
                except (TypeError, ValueError, IndexError):
                    min_val, max_val = default_min, default_max
            else:
                min_val, max_val = default_min, default_max
            normalized.append((min_val, max_val))

        self.cartesian_slider_limits_xyz = normalized
        self.cartesian_slider_limits_changed.emit()

    def set_axis_speed_limit(self, index: int, value: float):
        """Définit la limite de vitesse d'un axe spécifique en deg/s"""
        if 0 <= index < 6:
            self.axis_speed_limits[index] = float(value)
            self.axis_speed_limits_changed.emit()

    def set_axis_speed_limits(self, limits: list[float]):
        """Définit les limites de vitesse de tous les axes en deg/s"""
        if len(limits) >= 6:
            self.axis_speed_limits = [float(v) for v in limits[:6]]
            self.axis_speed_limits_changed.emit()

    def set_axis_accel_limit(self, index: int, value: float):
        """Définit la limite d'acceleration d'un axe specifique en deg/s^2"""
        if 0 <= index < 6:
            self.axis_accel_limits[index] = max(0.0, float(value))
            self.axis_accel_limits_changed.emit()

    def set_axis_accel_limits(self, limits: list[float]):
        """Définit les limites d'acceleration de tous les axes en deg/s^2"""
        if len(limits) >= 6:
            self.axis_accel_limits = [max(0.0, float(v)) for v in limits[:6]]
            self.axis_accel_limits_changed.emit()

    def set_axis_jerk_limit(self, index: int, value: float):
        """Définit la limite de jerk d'un axe spécifique en deg/s^3"""
        if 0 <= index < 6:
            self.axis_jerk_limits[index] = max(0.0, float(value))
            self.axis_jerk_limits_changed.emit()

    def set_axis_jerk_limits(self, limits: list[float]):
        """Définit les limites de jerk de tous les axes en deg/s^3"""
        if len(limits) >= 6:
            self.axis_jerk_limits = [max(0.0, float(v)) for v in limits[:6]]
            self.axis_jerk_limits_changed.emit()

    def set_axis_reversed_single(self, index: int, reversed_value: bool):
        """Inverse un axe spécifique"""
        if 0 <= index < 6:
            self.axis_reversed[index] = -1 if reversed_value else 1
            self.joint_values_not_inverted[index] = self.joint_values[index] * self.axis_reversed[index]

            self.MGI_solver.set_invert_table(RobotModel._mgi_build_invert_table(self.axis_reversed))
            self.axis_reversed_changed.emit()
            self._update_tcp_pose()
        
    def set_axis_reversed(self, axis_reversed: list[int]):
        """Définit les multiplicateurs d'axes (1 ou -1)"""
        if len(axis_reversed) >= 6:
            self.axis_reversed = list(axis_reversed[:6])
            for i in range(6):
                self.joint_values_not_inverted[i] = self.joint_values[i] * self.axis_reversed[i]

            self.MGI_solver.set_invert_table(RobotModel._mgi_build_invert_table(self.axis_reversed))
            self.axis_reversed_changed.emit()
            self._update_tcp_pose()

    # ============================================================================
    # RÉGION: Getters - Paramètres DH
    # ============================================================================
    
    def get_dh_params(self):
        """Retourne tous les paramètres DH"""
        return [row.copy() for row in self.dh_params]
    
    def get_dh_param(self, row: int, col: int):
        """Retourne un paramètre DH spécifique (mesurés si activés, sinon théoriques)"""
        dh_source = self.measured_dh_params if self.measured_dh_enabled else self.dh_params
        return dh_source[row][col] if 0 <= row < 6 and 0 <= col < 4 else 0
    
    def get_dh_row(self, row: int):
        """Retourne une ligne complète de paramètres DH"""
        return self.dh_params[row].copy() if 0 <= row < 6 else [0, 0, 0, 0]
    
    # ============================================================================
    # RÉGION: Setters - Paramètres DH
    # ============================================================================
    
    def set_dh_params(self, params: list[list[float]]):
        """Définit tous les paramètres DH"""
        self.dh_params = [list(row) for row in params]
        # Assurer 6 lignes
        while len(self.dh_params) < 6:
            self.dh_params.append([0, 0, 0, 0])
        self.dh_params = self.dh_params[:6]
        self.dh_params_changed.emit()
        self.MGI_solver.set_geometric_params(RobotModel._mgi_build_geometric_params(self.dh_params))
        self._update_tcp_pose()
    
    def set_dh_param(self, row: int, col: int, value: float):
        """Définit un paramètre DH spécifique"""
        if 0 <= row < 6 and 0 <= col < 4:
            try:
                self.dh_params[row][col] = float(value)
                self.dh_params_changed.emit()
                self.MGI_solver.set_geometric_params(RobotModel._mgi_build_geometric_params(self.dh_params))
                self._update_tcp_pose()

            except (ValueError, TypeError):
                print(f"Erreur: valeur DH invalide [{row},{col}] = {value}")
        else:
            print(f"Erreur: index DH invalide [{row},{col}]")
    
    def set_dh_row(self, row: int, values: list[float]):
        """Définit une ligne complète de paramètres DH"""
        if 0 <= row < 6 and len(values) >= 4:
            try:
                self.dh_params[row] = [float(v) for v in values[:4]]
                self.dh_params_changed.emit()
                self.MGI_solver.set_geometric_params(RobotModel._mgi_build_geometric_params(self.dh_params))
                self._update_tcp_pose()

            except (ValueError, TypeError):
                print(f"Erreur: valeurs DH invalides pour la ligne {row}")

    def get_measured_dh_params(self) -> list[list[float]]:
        """Retourne les paramètres DH mesurés"""
        return [row.copy() for row in self.measured_dh_params]

    def set_measured_dh_params(self, params: list[list[float]]):
        """Définit les paramètres DH mesurés"""
        self.measured_dh_params = [list(row) for row in params]
        while len(self.measured_dh_params) < 6:
            self.measured_dh_params.append([0.0, 0.0, 0.0, 0.0])
        self.measured_dh_params = self.measured_dh_params[:6]
        self.measured_dh_params_changed.emit()

    def get_measured_dh_enabled(self) -> bool:
        return self.measured_dh_enabled

    def set_measured_dh_enabled(self, enabled: bool):
        self.measured_dh_enabled = bool(enabled)
        self.measured_dh_enabled_changed.emit()
        self.measured_dh_params_changed.emit()

    # ============================================================================
    # RÉGION: Corrections
    # ============================================================================
    
    def get_corrections(self):
        """Retourne toutes les corrections 6D"""
        return [row.copy() for row in self.corrections]
    
    def get_correction(self, row: int, col: int):
        """Retourne une correction 6D spécifique"""
        return self.corrections[row][col] if 0 <= row < 6 and 0 <= col < 6 else 0
    
    def get_correction_row(self, row: int):
        """Retourne une ligne complète de corrections"""
        return self.corrections[row].copy() if 0 <= row < 6 else [0, 0, 0, 0, 0, 0]
    
    def get_correction_joint(self, joint_index: int):
        """Retourne le vecteur de correction 6D pour un joint"""
        return self.corrections[joint_index].copy() if 0 <= joint_index < 6 else [0, 0, 0, 0, 0, 0]
    
    def _set_corrections(self, corrections: list[list[float]]):
        """Définit toutes les corrections 6D"""
        self.corrections = [list(row) for row in corrections]
        # Assurer 6 lignes x 6 colonnes
        while len(self.corrections) < 6:
            self.corrections.append([0, 0, 0, 0, 0, 0])
        self.corrections = [row[:6] + [0]*(6-len(row)) for row in self.corrections[:6]]
        self.corrections_changed.emit()

    def compute_corrections(self):
        """Calcule les corrections 6D basées sur les mesures"""
        # TODO : Implémenter le calcul des corrections
        self.corrections_changed.emit()
        self._update_tcp_pose()

    # ============================================================================
    # RÉGION: Getters - Résultats cinématique
    # ============================================================================
    
    def get_tcp_pose(self) -> Pose6:
        """Retourne la pose TCP non corrigée"""
        return self.tcp_pose.copy()
    
    def get_corrected_tcp_pose(self) -> Pose6:
        """Retourne la pose TCP corrigée"""
        return self.corrected_tcp_pose.copy()
    
    def get_tcp_deviation(self) -> Pose6:
        """Retourne la déviation entre TCP et TCP corrigé"""
        return self.pose_deviation.copy()
    
    def get_tcp_position(self):
        """Retourne la position (X, Y, Z) du TCP"""
        return [self.tcp_pose.x, self.tcp_pose.y, self.tcp_pose.z]
    
    def get_tcp_rotation(self):
        """Retourne la rotation (Rx, Ry, Rz) du TCP"""
        return [self.tcp_pose.a, self.tcp_pose.b, self.tcp_pose.c]
    
    # ============================================================================
    # RÉGION: Setters - Résultats cinématique
    # ============================================================================
    
    def _set_tcp_pose(self, pose: Pose6):
        """Définit la pose TCP non corrigée"""
        if not isinstance(pose, Pose6):
            raise TypeError("pose must be a Pose6")
        self.tcp_pose = pose.copy()
    
    def _set_corrected_tcp_pose(self, pose: Pose6, compute_deviation: bool=True):
        """Définit la pose TCP non corrigée"""
        if not isinstance(pose, Pose6):
            raise TypeError("pose must be a Pose6")
        self.corrected_tcp_pose = pose.copy()
        if compute_deviation:
            self._compute_deviation()

    def _compute_deviation(self):
        """Calcule la déviation entre TCP et TCP corrigé"""
        self.pose_deviation = Pose6(
            self.corrected_tcp_pose.x - self.tcp_pose.x,
            self.corrected_tcp_pose.y - self.tcp_pose.y,
            self.corrected_tcp_pose.z - self.tcp_pose.z,
            self.corrected_tcp_pose.a - self.tcp_pose.a,
            self.corrected_tcp_pose.b - self.tcp_pose.b,
            self.corrected_tcp_pose.c - self.tcp_pose.c,
        )
    
    # ============================================================================
    # RÉGION: Getters - Mesures
    # ============================================================================
    
    def get_measurements(self):
        """Retourne la liste des mesures enregistrées"""
        return self.measurements.copy()
    
    def get_measurement(self, index: int):
        """Retourne une mesure spécifique"""
        return self.measurements[index] if 0 <= index < len(self.measurements) else None
    
    def get_measurement_count(self):
        """Retourne le nombre de mesures enregistrées"""
        return len(self.measurements)
    
    def get_measurement_points(self):
        """Retourne la liste des points de mesure"""
        return self.measurement_points.copy()
    
    def get_measurement_point(self, index):
        """Retourne un point de mesure spécifique"""
        return self.measurement_points[index] if 0 <= index < len(self.measurement_points) else  None
    
    # ============================================================================
    # RÉGION: Setters - Mesures
    # ============================================================================
    
    def get_measurements_filename(self) -> str:
        """Retourne le nom du fichier de mesures"""
        return self.measurements_filename

    def add_measurement(self, measurement: float):
        """Ajoute une nouvelle mesure"""
        self.measurements.append(measurement)
        self.measurements_changed.emit()

    def add_measurement_point(self, point: list[float]):
        """Ajoute un nouveau point de mesure"""
        self.measurement_points.append(point)
        self.measurements_points_changed.emit()

    def set_measurements(self, filename: str, measurements: list[float]):
        """Définit la liste des mesures"""
        self.measurements_filename = filename
        self.measurements = list(measurements)
        self.measurements_changed.emit()
    
    def set_measurement_points(self, points: list[list[float]]):
        """Définit la liste des points de mesure"""
        self.measurement_points = list(points)
        self.measurements_points_changed.emit()
    
    def clear_measurements(self):
        """Efface toutes les mesures"""
        self.measurements.clear()
        self.measurements_changed.emit()
        self.compute_corrections()
    
    def clear_measurement_points(self):
        """Efface tous les points de mesure"""
        self.measurement_points.clear()
        self.measurements_points_changed.emit()
        self.compute_corrections()
    
    # ============================================================================
    # RÉGION: Sérialisation / Désérialisation
    # ============================================================================

    def to_configuration_file(self) -> RobotConfigurationFile:
        """Construit un objet de configuration depuis l'etat courant."""
        return RobotConfigurationFile.from_robot_model(self)

    def reset_to_unconfigured_state(self) -> None:
        """Remet le robot dans l'etat initial correspondant a aucune configuration chargee."""
        self._inhibit_compute_fk = True

        self.robot_name = ""
        self.has_configuration = False
        self.current_config_file = None

        self.axis_limits = list(RobotModel.UNCONFIGURED_AXIS_LIMITS)
        self.cartesian_slider_limits_xyz = list(RobotModel.UNCONFIGURED_CARTESIAN_SLIDER_LIMITS_XYZ)
        self.axis_speed_limits = list(RobotModel.UNCONFIGURED_AXIS_SPEED_LIMITS)
        self.axis_accel_limits = list(RobotModel.UNCONFIGURED_AXIS_ACCEL_LIMITS)
        self.axis_jerk_limits = list(RobotModel.UNCONFIGURED_AXIS_JERK_LIMITS)
        self.robot_cad_models = list(RobotModel.DEFAULT_ROBOT_CAD_MODELS)
        self.axis_colliders = [collider.copy() for collider in RobotModel.UNCONFIGURED_AXIS_COLLIDERS]
        self._axis_colliders_revision += 1

        self.home_position = list(RobotModel.UNCONFIGURED_HOME_POSITION)
        self.position_zero = list(RobotModel.UNCONFIGURED_POSITION_ZERO)
        self.position_calibration = list(RobotModel.UNCONFIGURED_POSITION_CALIBRATION)

        self.joint_values = [0.0] * 6
        self.joint_values_not_inverted = [0.0] * 6
        self.axis_reversed = [1] * 6
        self.joint_weights = [1.0] * 6

        self.dh_params = [[0.0, 0.0, 0.0, 0.0] for _ in range(6)]
        self.measured_dh_params = [[0.0, 0.0, 0.0, 0.0] for _ in range(6)]
        self.measured_dh_enabled = False
        self.current_tcp_dh_matrices = []
        self.current_tcp_corrected_dh_matrices = []
        self.corrections = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] for _ in range(6)]

        self.measurements_filename = ""
        self.measurements = []
        self.measurement_points = []

        self.MGI_solver.set_geometric_params(RobotModel._mgi_build_geometric_params(self.dh_params))
        self.MGI_solver.set_axis_limits(RobotModel._mgi_build_axis_limits(self.axis_limits))
        self.MGI_solver.set_invert_table(RobotModel._mgi_build_invert_table(self.axis_reversed))
        self.current_tcp_mgi_result = MgiResult()
        self.current_axis_config = MgiConfigKey.FUN
        self._set_tcp_pose(Pose6.zeros())
        self._set_corrected_tcp_pose(Pose6.zeros(), True)
        self._tcp_rotation_matrix = math_utils.euler_to_rotation_matrix(0.0, 0.0, 0.0)

        self._inhibit_compute_fk = False
        self.configuration_changed.emit()
        self.robot_name_changed.emit(self.robot_name)
        self.dh_params_changed.emit()
        self.measured_dh_params_changed.emit()
        self.measured_dh_enabled_changed.emit()
        self.axis_limits_changed.emit()
        self.cartesian_slider_limits_changed.emit()
        self.axis_speed_limits_changed.emit()
        self.axis_accel_limits_changed.emit()
        self.axis_jerk_limits_changed.emit()
        self.axis_reversed_changed.emit()
        self.joint_weights_changed.emit()
        self.robot_cad_models_changed.emit()
        self.cad_models_changed.emit()
        self.axis_colliders_changed.emit()
        self.corrections_changed.emit()
        self.joints_changed.emit()
        self.tcp_pose_changed.emit()
        self.measurements_changed.emit()
        self.measurements_points_changed.emit()

    def load_from_configuration_file(self, config: RobotConfigurationFile, file_name: str = None):
        """Charge l'etat du robot depuis un objet de configuration."""
        self._inhibit_compute_fk = True

        if file_name:
            self.current_config_file = file_name

        config.apply_to_robot_model(self)
        self.go_to_home_position()
        self.has_configuration = True
        self.configuration_changed.emit()

        self._inhibit_compute_fk = False
        self._update_tcp_pose()

    def to_dict(self):
        """Export vers dictionnaire (pour sauvegarde JSON)"""
        return self.to_configuration_file().to_dict()
    
    def load_from_dict(self, data: dict, file_name: str=None):
        """Import depuis dictionnaire (pour chargement JSON)"""
        config = RobotConfigurationFile.from_dict(data)
        self.load_from_configuration_file(config, file_name)

