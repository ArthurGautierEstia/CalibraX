from __future__ import annotations
from math import radians, degrees, atan2, sin, cos, sqrt, pi
import numpy as np
from enum import Enum
from typing import Set, override

EPSILON = 1e-6

DEFAULT_AXIS_LIMITS = [
        (-170, 170),  # q1
        (-190, 45),   # q2
        (-120, 156),  # q3
        (-185, 185),  # q4
        (-120, 120),  # q5
        (-350, 350),  # q6
    ]

DEFAULT_INVERT_TABLE = [True, False, False, True, False, True]

class ConfigurationIdentifier():
    def is_front(self, j1: float):
        return True
    
    def is_up(self, j3: float):
        return True
    
    def is_flipped(self, j4: float):
        return False

class MgiConfigKey(Enum):
    FUN = 0 # Front, Up, No Flip
    FUF = 1 # Front, Up, Flip
    FDN = 2 # Front, Down, No Flip
    FDF = 3 # Front, Down, Flip
    BUN = 4 # Back, Up, No Flip
    BUF = 5 # Back, Up, Flip
    BDN = 6 # Back, Down, No Flip
    BDF = 7 # Back, Down, Flip

    @staticmethod
    def get_mgi_config_key_from(is_front: bool, is_up: bool, is_flipped: bool) -> "MgiConfigKey":
        idx = (0 if is_front else 4) + (0 if is_up else 2) + (1 if is_flipped else 0)
        return MgiConfigKey(idx)

    @staticmethod
    def identify_configuration(joints: list[float],  config_identifier: ConfigurationIdentifier) -> "MgiConfigKey":
        """
        Identifie la configuration à partir des valeurs articulaires.
        
        Args:
            joints: Liste des 6 angles articulaires en radians [q1, q2, q3, q4, q5, q6]
            config_identifier: L'identifieur de configuration du robot
        
        Returns:
            La clé de configuration correspondante
        """
        is_front = config_identifier.is_front(joints[0])
        is_up = config_identifier.is_up(joints[2])
        is_flipped = config_identifier.is_flipped(joints[3])
        return MgiConfigKey.get_mgi_config_key_from(is_front, is_up, is_flipped)

    @staticmethod
    def identify_configuration_deg(joints: list[float],  config_identifier: ConfigurationIdentifier) -> "MgiConfigKey":
        """
        Identifie la configuration à partir des valeurs articulaires.
        
        Args:
            joints: Liste des 6 angles articulaires en degrés [q1, q2, q3, q4, q5, q6]
            config_identifier: L'identifieur de configuration du robot
        
        Returns:
            La clé de configuration correspondante
        """
        return MgiConfigKey.identify_configuration([radians(deg) for deg in joints], config_identifier)


FRONT_CONFIG_KEYS = [MgiConfigKey.FUN, MgiConfigKey.FUF, MgiConfigKey.FDN, MgiConfigKey.FDF]
BACK_CONFIG_KEYS = [MgiConfigKey.BUN, MgiConfigKey.BUF, MgiConfigKey.BDN, MgiConfigKey.BDF]

UP_CONFIG_KEYS = [MgiConfigKey.FUN, MgiConfigKey.FUF, MgiConfigKey.BUN, MgiConfigKey.BUF]
DOWN_CONFIG_KEYS = [MgiConfigKey.FDN, MgiConfigKey.FDF, MgiConfigKey.BDN, MgiConfigKey.BDF]

FLIP_CONFIG_KEYS = [MgiConfigKey.FUF, MgiConfigKey.FDF, MgiConfigKey.BUF, MgiConfigKey.BDF]
NO_FLIP_CONFIG_KEYS = [MgiConfigKey.FUN, MgiConfigKey.FDN, MgiConfigKey.BUN, MgiConfigKey.BDN]

FRONT_UP_CONFIG_KEYS = [MgiConfigKey.FUN, MgiConfigKey.FUF]
FRONT_DOWN_CONFIG_KEYS = [MgiConfigKey.FDN, MgiConfigKey.FDF]

BACK_UP_CONFIG_KEYS = [MgiConfigKey.BUN, MgiConfigKey.BUF]
BACK_DOWN_CONFIG_KEYS = [MgiConfigKey.BDN, MgiConfigKey.BDF]


class RobotTool():
    def __init__(self, x = 0., y = 0., z = 0., a = 0., b = 0., c = 0.):
        self.x = x
        self.y = y
        self.z = z
        self.a = a
        self.b = b
        self.c = c
    
    def is_identity(self) -> bool:
        return (abs(self.x) < EPSILON and abs(self.y) < EPSILON and abs(self.z) < EPSILON and
                abs(self.a) < EPSILON and abs(self.b) < EPSILON and abs(self.c) < EPSILON)

class KukaConfigurationIdentifier(ConfigurationIdentifier):
    @override
    def is_front(self, j1: float):
        return abs(j1) <= pi/2
    
    @override
    def is_up(self, j3: float):
        return j3 >= 0
    
    @override
    def is_flipped(self, j4: float):
        return abs(j4) > pi/2

class MgiGeometricParams():
    def __init__(self, r1=400, d2=25, d3=560, d4=35, r4=515, r6=80):
        #Constantes du robot
        self.R1 = r1 # offset de la base
        self.R6 = r6 # offset du flange

        # Paramètres géométriques
        self.D2 = d2
        self.D3 = d3
        self.D4 = d4
        self.R4 = r4

class MgiAxisLimits:
    def __init__(self, radians: bool=False, axis_limits: list[tuple[float, float]] = DEFAULT_AXIS_LIMITS):
        self.radians = radians
        self.axis_limits = axis_limits

    def to_radians(self):
        if not self.radians:
            self.radians = True
            self.axis_limits = [(radians(qMin), radians(qMax)) for (qMin, qMax) in self.axis_limits]

class MgiSingularityBehavior(Enum):
    STOP = 0
    CONTINUE = 1

class MgiSingularitiesBehavior:
    def __init__(self, 
                 q1_behavior: MgiSingularityBehavior = MgiSingularityBehavior.STOP):
                 #q5_behavior: MgiSingularityBehavior = MgiSingularityBehavior.STOP):
        self.q1_behavior = q1_behavior
        # self.q5_behavior = q5_behavior

class MgiConfigurationFilter:
    """Filtre pour autoriser/interdire certaines configurations MGI"""
    
    def __init__(self, allowed_configs: Set[MgiConfigKey] | None = None):
        """
        Args:
            allowed_configs: Ensemble des configurations autorisées. 
                           Si None, toutes les configurations sont autorisées.
        """
        self.allowed_configs = allowed_configs
    
    def __repr__(self):
        if self.allowed_configs is None:
            return "MgiConfigurationFilter(ALL)"
        return f"MgiConfigurationFilter({[k.name for k in self.allowed_configs]})"

    def is_allowed(self, config_key: MgiConfigKey) -> bool:
        """Vérifie si une configuration est autorisée"""
        if self.allowed_configs is None:
            return True
        return config_key in self.allowed_configs
    
    @staticmethod
    def allow_all() -> 'MgiConfigurationFilter':
        """Crée un filtre qui autorise toutes les configurations"""
        return MgiConfigurationFilter(None)
    
    @staticmethod
    def allow_only_front() -> 'MgiConfigurationFilter':
        """Autorise uniquement les configurations Front"""
        return MgiConfigurationFilter(set(FRONT_CONFIG_KEYS))
    
    @staticmethod
    def allow_only_back() -> 'MgiConfigurationFilter':
        """Autorise uniquement les configurations Back"""
        return MgiConfigurationFilter(set(BACK_CONFIG_KEYS))
    
    @staticmethod
    def allow_only_up() -> 'MgiConfigurationFilter':
        """Autorise uniquement les configurations Up"""
        return MgiConfigurationFilter(set(UP_CONFIG_KEYS))
    
    @staticmethod
    def allow_only_down() -> 'MgiConfigurationFilter':
        """Autorise uniquement les configurations Down"""
        return MgiConfigurationFilter(set(DOWN_CONFIG_KEYS))
    
    @staticmethod
    def allow_no_flip() -> 'MgiConfigurationFilter':
        """Autorise uniquement les configurations sans Flip"""
        return MgiConfigurationFilter(set(NO_FLIP_CONFIG_KEYS))
    
    @staticmethod
    def allow_only_flip() -> 'MgiConfigurationFilter':
        """Autorise uniquement les configurations avec Flip"""
        return MgiConfigurationFilter(set(FLIP_CONFIG_KEYS))
    
    @staticmethod
    def allow_custom(config_keys: list[MgiConfigKey]) -> 'MgiConfigurationFilter':
        """Autorise une liste personnalisée de configurations"""
        return MgiConfigurationFilter(set(config_keys))
    
    @staticmethod
    def combine_filters(*filters: 'MgiConfigurationFilter') -> 'MgiConfigurationFilter':
        """Combine plusieurs filtres (intersection des configurations autorisées)"""
        if not filters:
            return MgiConfigurationFilter.allow_all()
        
        # Si un filtre autorise tout, on l'ignore
        active_filters = [f for f in filters if f.allowed_configs is not None]
        
        if not active_filters:
            return MgiConfigurationFilter.allow_all()
        
        # Intersection de tous les ensembles
        allowed = set(active_filters[0].allowed_configs)
        for f in active_filters[1:]:
            allowed &= f.allowed_configs
        
        return MgiConfigurationFilter(allowed)

class MgiParams:
    def __init__(self, 
                 configuration_identifier: ConfigurationIdentifier,
                 geometric_params: MgiGeometricParams = MgiGeometricParams(), 
                 invert_table: list[bool] = DEFAULT_INVERT_TABLE,
                 axis_limits: MgiAxisLimits = MgiAxisLimits(),
                 singularities_behavior: MgiSingularitiesBehavior = MgiSingularitiesBehavior(),
                 configuration_filter: MgiConfigurationFilter = MgiConfigurationFilter.allow_all()):
        self.configuration_identifier = configuration_identifier
        self.geometric_params = geometric_params
        self.invert_table = invert_table
        self.axis_limits = axis_limits
        self.singularities_behavior = singularities_behavior
        self.configuration_filter = configuration_filter

class MgiResultStatus(Enum):
    """Statuts possibles pour une solution MGI"""
    VALID = 0
    UNREACHABLE = 1
    SINGULARITY = 2
    AXIS_LIMIT_VIOLATED = 3
    FORBIDDEN_CONFIGURATION = 4

class MgiResultItem():
    def __init__(self):
        self.status = MgiResultStatus.VALID
        self.radians = True
        self.joints = [0, 0, 0, 0, 0, 0]
        self.j1Singularity = False
        self.j3Singularity = False
        self.j5Singularity = False
        self.violated_limits = []  # list of joint indices that violated limits
    
    def clear_joints(self):
        self.joints = [0, 0, 0, 0, 0, 0]

    def setQ1(self, q1: float):
        self.joints[0] = q1

    def setQ2(self, q2: float):
        self.joints[1] = q2
    
    def setQ3(self, q3: float):
        self.joints[2] = q3
    
    def setQ4(self, q4: float):
        self.joints[3] = q4
    
    def setQ5(self, q5: float):
        self.joints[4] = q5
    
    def setQ6(self, q6: float):
        self.joints[5] = q6
    
    def setQ456(self, j4: float, j5: float, j6: float):
        self.joints[3] = j4
        self.joints[4] = j5
        self.joints[5] = j6
    
    def invert_joints(self, invert_table: list[bool]):
        self.joints = MGI._joints_invert(self.joints, invert_table)

    def to_radians(self):
        if not self.radians:
            self.radians = True
            self.joints = [radians(q) for q in self.joints]
    
    def to_degrees(self):
        if self.radians:
            self.radians = False
            self.joints = [degrees(q) for q in self.joints]

class MgiResult():
    def __init__(self):
        self.all_solutions_evaluated = False
        self.solutions = {
            MgiConfigKey.FUN: MgiResultItem(),
            MgiConfigKey.FUF: MgiResultItem(),
            MgiConfigKey.FDN: MgiResultItem(),
            MgiConfigKey.FDF: MgiResultItem(),
            MgiConfigKey.BUN: MgiResultItem(),
            MgiConfigKey.BUF: MgiResultItem(),
            MgiConfigKey.BDN: MgiResultItem(),
            MgiConfigKey.BDF: MgiResultItem(),
        }

    def get_solution(self, key: MgiConfigKey) -> MgiResultItem:
        return self.solutions[key]
    
    def set_solution_joints(self, key: MgiConfigKey, joints: list[float]):
        self.solutions[key].joints = joints

    def clear_solutions_joints(self):
        for key in self.solutions:
            self.solutions[key].clear_joints()
    
    def get_front_solutions(self) -> dict[MgiConfigKey, MgiResultItem]:
        return {key: self.solutions[key] for key in FRONT_CONFIG_KEYS}
    
    def get_back_solutions(self) -> dict[MgiConfigKey, MgiResultItem]:
        return {key: self.solutions[key] for key in BACK_CONFIG_KEYS}

    def get_q1_front_back(self):
        return self.solutions[MgiConfigKey.FUN].joints[0], self.solutions[MgiConfigKey.BUN].joints[0]

    def apply_invert_table(self, invert_table: list[bool]):
        if True not in invert_table:
            return

        for key, item in self.solutions.items():
            self.solutions[key].joints = MgiResult._joints_invert(item.joints, invert_table)

    def apply_axis_limits(self, axis_limits: MgiAxisLimits):
        for sol in self.solutions.values():
            if sol.status == MgiResultStatus.VALID:
                for i, (j, (jMin, jMax)) in enumerate(zip(sol.joints, axis_limits.axis_limits)):
                    if j < jMin or j > jMax:
                        sol.status = MgiResultStatus.AXIS_LIMIT_VIOLATED
                        sol.violated_limits.append(i)

    def filter_configurations(self, config_filter: MgiConfigurationFilter):
        """Applique un filtre de configuration et marque les configs interdites"""
        for key, sol in self.solutions.items():
            if sol.status == MgiResultStatus.VALID:
                if not config_filter.is_allowed(key):
                    sol.status = MgiResultStatus.FORBIDDEN_CONFIGURATION

    def to_degrees(self):
        for sol in self.solutions.values():
            sol.to_degrees()
        
    def to_radians(self):
        for sol in self.solutions.values():
            sol.to_radians()
    
    @property
    def has_valid_solution(self) -> bool:
        return any(sol.status == MgiResultStatus.VALID for sol in self.solutions.values())

    @property
    def valid_count(self) -> int:
        return sum(1 for sol in self.solutions.values() if sol.status == MgiResultStatus.VALID)

    @staticmethod
    def _joints_invert(joints, invert_table):
        """Inversion des joints selon une table de correspondance"""
        inverted_joints = []
        if (len(joints) != len(invert_table)):
            raise ValueError("La taille de la table d'inversion ne correspond pas au nombre de joints.")
        
        for i in range(len(joints)):
            inverted_joints.append(-joints[i] if invert_table[i] else joints[i])

        return inverted_joints

    def get_valid_solutions(self) -> dict[MgiConfigKey, MgiResultItem]:
        """Retourne uniquement les solutions VALID"""
        return {key: sol for key, sol in self.solutions.items() if sol.status == MgiResultStatus.VALID}

    def get_best_solution(self, prefer_front: bool = True, prefer_up: bool = True, prefer_no_flip: bool = True) -> tuple[MgiConfigKey, MgiResultItem] | None:
        """Retourne la meilleure solution selon les préférences"""
        valid = self.get_valid_solutions()
        if not valid:
            return None
        
        # Scoring simple basé sur les préférences
        def score(key: MgiConfigKey):
            s = 0
            if prefer_front and key in FRONT_CONFIG_KEYS: s += 4
            if prefer_up and key in UP_CONFIG_KEYS: s += 2
            if prefer_no_flip and key in NO_FLIP_CONFIG_KEYS: s += 1
            return s
        
        best_key = max(valid.keys(), key=score)
        return best_key, valid[best_key]

    def get_best_solution_from_current(self,
                                       current_joints_rad: list[float],
                                       joint_weights: list[float] = None) -> tuple[MgiConfigKey, MgiResultItem] | None:
        """
        Trouve la meilleure solution en fonction de la position courante.
        
        Args:
            current_joints_rad: Position articulaire courante [q1, q2, q3, q4, q5, q6]
        
        Returns:
            (MgiConfigKey, MgiResultItem) de la meilleure solution, ou None si aucune solution valide
        """
        valid_solutions = self.get_valid_solutions()
        if not valid_solutions:
            return None
        
        if not joint_weights:
            joint_weights = [1.0] * 6
        
        while len(joint_weights) < 6:
            joint_weights.append(1.0)
        
        best_key: MgiConfigKey = None
        best_distance = float('inf')
        
        for key, sol in valid_solutions.items():
            sol_joints_rad = [radians(q) for q in sol.joints]
            distance = sum((joint_weights[i] * (c - s)**2) for i, (c, s) in enumerate(zip(current_joints_rad, sol_joints_rad)))
            if distance < best_distance:
                best_distance = distance
                best_key = key
        
        return best_key, valid_solutions[best_key]


class MGI():

    class ResolutionVariables:
        def __init__(self):
            self.a_rad = 0.0
            self.b_rad = 0.0
            self.c_rad = 0.0

            self.ca = 0.0
            self.cb = 0.0
            self.cc = 0.0

            self.sa = 0.0
            self.sb = 0.0
            self.sc = 0.0
        
        def compute_trig(self, a_deg: float, b_deg: float, c_deg: float):
            self.a_rad = radians(a_deg)
            self.b_rad = radians(b_deg)
            self.c_rad = radians(c_deg)

            self.ca = cos(self.a_rad)
            self.cb = cos(self.b_rad)
            self.cc = cos(self.c_rad)

            self.sa = sin(self.a_rad)
            self.sb = sin(self.b_rad)
            self.sc = sin(self.c_rad)

    def __init__(self, params: MgiParams, tool: RobotTool|None = None):
        self.params = params
        self.tool = tool
        self.defaultQ1RadSingularityValue = 0.0
        self.defaultQ4RadSingularityValue = 0.0
        self.defaultQ6RadSingularityValue = 0.0
        self._resolution_vars = MGI.ResolutionVariables()

    def set_params(self, params: MgiGeometricParams):
        self.params = params

    def set_invert_table(self, invert_table: list[bool]):
        self.params.invert_table = invert_table

    def set_axis_limits(self, axis_limits: MgiAxisLimits):
        self.params.axis_limits = axis_limits

    def set_geometric_params(self, geometric_params: MgiGeometricParams):
        self.params.geometric_params = geometric_params

    def set_configuration_filter(self, configuration_filter: MgiConfigurationFilter):
        self.params.configuration_filter = configuration_filter

    def get_configuration_filter(self) -> MgiConfigurationFilter:
        return self.params.configuration_filter

    def set_tool(self, tool: RobotTool):
        self.tool = tool
    
    def set_q1ValueIfSingularityQ1(self, rad: float):
        self.defaultQ1RadSingularityValue = rad

    def set_q1ValueIfSingularityQ1Deg(self, deg: float):
        self.defaultQ1RadSingularityValue = radians(deg)

    def set_q4ValueIfSingularityQ5(self, rad: float):
        self.defaultQ4RadSingularityValue = rad
    
    def set_q4ValueIfSingularityQ5Deg(self, deg: float):
        self.defaultQ4RadSingularityValue = radians(deg)

    def set_q6ValueIfSingularityQ5(self, rad: float):
        self.defaultQ6RadSingularityValue = rad
    
    def set_q6ValueIfSingularityQ5Deg(self, deg: float):
        self.defaultQ6RadSingularityValue = radians(deg)

    def _compute_radians(self, a_deg: float, b_deg: float, c_deg: float):
        self._resolution_vars.compute_trig(a_deg, b_deg, c_deg)

    @staticmethod
    def _rot_z(a: float):
        ca, sa = cos(a), sin(a)
        return np.array([[ca, -sa, 0.0],
                [sa,  ca, 0.0],
                [0.0, 0.0, 1.0]])

    @staticmethod
    def _rot_y(b: float):
        cb, sb = cos(b), sin(b)
        return np.array([[ cb, 0.0, sb],
                [0.0, 1.0, 0.0],
                [-sb, 0.0, cb]])

    @staticmethod
    def _rot_x(c: float):
        cc, sc = cos(c), sin(c)
        return np.array([[1.0, 0.0, 0.0],
                [0.0,  cc, -sc],
                [0.0,  sc,  cc]])
    
    @staticmethod
    def _add_pi(angle: float)-> float:
        return angle + pi if angle <= 0 else angle - pi

    @staticmethod
    def _compute_tool_to_flange_coordinates(x: float, y: float, z: float, 
                                a_deg: float, b_deg: float, c_deg: float, 
                                tool: RobotTool|None):
        """
        Entrée : pose TCP (Tool Center Point) en base KUKA {X,Y,Z,A,B,C} avec un tool actif.
        Sortie : pose du flange {Xf,Yf,Zf,Af,Bf,Cf} correspondante.
        
        Principe : TCP = Flange * Tool
                => Flange = TCP * Tool^(-1)
        
        On suppose que RobotTool contient : tool.x, tool.y, tool.z, tool.a, tool.b, tool.c
        """
        if (tool is None or tool.is_identity()):
            return x, y, z, a_deg, b_deg, c_deg

        # 1. Matrice de transformation TCP (pose demandée)
        a_rad = radians(a_deg)
        b_rad = radians(b_deg)
        c_rad = radians(c_deg)
        R_tcp = MGI._rot_z(a_rad) @ MGI._rot_y(b_rad) @ MGI._rot_x(c_rad)
        T_tcp = np.eye(4)
        T_tcp[:3, :3] = R_tcp
        T_tcp[:3, 3] = [x, y, z]
        
        # 2. Matrice de transformation du Tool (offset par rapport au flange)
        ta_rad = radians(tool.a)
        tb_rad = radians(tool.b)
        tc_rad = radians(tool.c)
        R_tool = MGI._rot_z(ta_rad) @ MGI._rot_y(tb_rad) @ MGI._rot_x(tc_rad)
        T_tool = np.eye(4)
        T_tool[:3, :3] = R_tool
        T_tool[:3, 3] = [tool.x, tool.y, tool.z]
        
        # 3. Calcul de la pose du flange : T_flange = T_tcp * T_tool^(-1)
        T_tool_inv = np.linalg.inv(T_tool)
        T_flange = T_tcp @ T_tool_inv
        
        # 4. Extraction de la position du flange
        xf = T_flange[0, 3]
        yf = T_flange[1, 3]
        zf = T_flange[2, 3]
        
        # 5. Extraction des angles d'Euler ZYX du flange
        R_flange = T_flange[:3, :3]
        
        # Conversion rotation matrix -> Euler ZYX (convention KUKA: Rz*Ry*Rx)
        # R = [r11 r12 r13]
        #     [r21 r22 r23]
        #     [r31 r32 r33]
        # B = atan2(-r31, sqrt(r11² + r21²))
        # A = atan2(r21/cos(B), r11/cos(B))
        # C = atan2(r32/cos(B), r33/cos(B))
        
        r11, r12, _ = R_flange[0, :]
        r21, r22, _ = R_flange[1, :]
        r31, r32, r33 = R_flange[2, :]
        
        # Calcul de B (rotation autour de Y)
        bf_rad = atan2(-r31, sqrt(r11**2 + r21**2))
        
        # Gestion du cas singulier (cos(B) ≈ 0)
        cos_b = cos(bf_rad)
        if abs(cos_b) > EPSILON:
            af_rad = atan2(r21 / cos_b, r11 / cos_b)
            cf_rad = atan2(r32 / cos_b, r33 / cos_b)
        else:
            # Singularité : B = ±90°, on pose arbitrairement C = 0
            af_rad = atan2(-r12, r22)
            cf_rad = 0.0
        
        af_deg = degrees(af_rad)
        bf_deg = degrees(bf_rad)
        cf_deg = degrees(cf_rad)
        
        return xf, yf, zf, af_deg, bf_deg, cf_deg

    @staticmethod
    def _compute_flange_to_mgi_coordinates(x: float, y: float, z: float,
                            a_deg: float, b_deg: float, c_deg: float,
                            r1: float, r6: float):
        """
        Entrée : pose TCP en base KUKA {X,Y,Z,A,B,C} et offsets r1 (base Z) + r6 (tool Z).
        Sortie : pose corrigée {Xc,Yc,Zc,A,B,C} pour alimenter un MGI simplifié:
                - R0 placé sur R1  => translation -r1 sur Z_base
                - repères 4/5/6 confondus (centre poignet) => translation -r6 sur Z_tool
        """
        # Rotation KUKA: R = Rz(A)*Ry(B)*Rx(C)
        a_rad = radians(a_deg)
        b_rad = radians(b_deg)
        c_rad = radians(c_deg)

        R = MGI._rot_z(a_rad) @ MGI._rot_y(b_rad) @ MGI._rot_x(c_rad)

        # Axe Z_tool exprimé dans la base = 3e colonne de R
        ztx, zty, ztz = R[0][2], R[1][2], R[2][2]

        # Correction translation:
        # -r1 sur Z_base, puis -r6 le long de Z_tool
        xc = x - r6 * ztx
        yc = y - r6 * zty
        zc = z - r1 - r6 * ztz

        return xc, yc, zc, a_deg, b_deg, c_deg

    @staticmethod
    def _solve_eq_type2(x: float, y: float, z: float):
        """
        Résout l'équation: x * sin(q) + y * cos(q) = z
        
        Returns:
            solve_case: int (+/- [1...4]). 1 to 4: Solvable cases, -1 to -4: No solution cases

            q_a, q_b: tuple of solutions (None if no solution)
        """
        q_a, q_b = 0, 0

        if abs(x) < EPSILON and abs(y) < EPSILON:
            #x = 0, y = 0
            solve_case = -3
            q_a = 0
            q_b = pi

        elif abs(x) < EPSILON and abs(y) > EPSILON:
            # x = 0, y != 0 => y*cos(q) = z
            cosq = z / y
            if abs(cosq) <= 1.0 + EPSILON:
                solve_case = 1
                cosq = max(-1.0, min(1.0, cosq))  # Clamp
                sinq_sq = 1 - cosq**2
                if sinq_sq >= 0:
                    sinq = sqrt(sinq_sq)
                    q_a = atan2(+sinq, cosq) 
                    q_b = atan2(-sinq, cosq)
            else:
                solve_case = -1  # Pas de solution
        
        elif abs(x) > EPSILON and abs(y) < EPSILON:
            # x != 0, y = 0 => x*sin(q) = z
            sinq = z / x
            if abs(sinq) <= 1.0 + EPSILON:
                solve_case = 2
                sinq = max(-1.0, min(1.0, sinq))  # Clamp
                cosq_sq = 1 - sinq**2
                if cosq_sq >= 0:
                    cosq = sqrt(cosq_sq)
                    q_a = atan2(sinq, +cosq) 
                    q_b = atan2(sinq, -cosq)
            else:
                solve_case = -2  # Pas de solution

        elif abs(z) < EPSILON:
            # x != 0, y != 0, z = 0 => x*sin(q) + y*cos(q) = 0
            solve_case = 3
            q_a = atan2(-y, x)
            q_b = MGI._add_pi(q_a)
        
        else:
            # Cas général: x != 0, y != 0, z != 0
            x2 = x * x
            y2 = y * y
            z2 = z * z
            x2y2 = x2 + y2
            
            if x2y2 >= z2 - EPSILON:
                solve_case = 4
                x2y2_z2 = max(0, x2y2 - z2)  # Éviter racine négative par erreur numérique
                sqrt_term = sqrt(x2y2_z2)

                xz = x * z
                yz = y * z
                y_sqrt = y * sqrt_term
                x_sqrt = x * sqrt_term

                # Solution 1: epsilon = +1
                sinq = (xz + y_sqrt) / x2y2
                cosq = (yz - x_sqrt) / x2y2
                q_a = atan2(sinq, cosq)

                # Solution 2: epsilon = -1
                sinq = (xz - y_sqrt) / x2y2
                cosq = (yz + x_sqrt) / x2y2
                q_b = atan2(sinq, cosq)
            else:
                solve_case = -4  # Pas de solution

        return solve_case, q_a, q_b

    @staticmethod
    def _tryAtan2(sinq: float, cosq: float):
        """Essaie de calculer atan2(sinq, cosq) si sinq != 0 et cosq != 0, retourne (True, 0) si singularité, (False, value) sinon."""
        return ((abs(sinq) < EPSILON and abs(cosq) < EPSILON), atan2(sinq, cosq))

    @staticmethod
    def _joints_invert(joints, invert_table):
        """Inversion des joints selon une table de correspondance"""
        inverted_joints = []
        if (len(joints) != len(invert_table)):
            raise ValueError("La taille de la table d'inversion ne correspond pas au nombre de joints.")
        
        for i in range(len(joints)):
            inverted_joints.append(-joints[i] if invert_table[i] else joints[i])

        return inverted_joints

    def _compute_q1(self, x: float, y: float):
        """
        Calcul de q1 en fonction depuis : T1_0 * U0 = T1_6. Equation (a24).
        Returns:
            singularity: bool

            q1: angle en radians
        """
        return MGI._tryAtan2(y, x)

    def _compute_q2(self, x: float, y: float, z: float, q1: float):
        """
        Calcul de q2 depuis : T2_1 * T1_0 * U0 = T2_6. Equations (a14)^2 + (a24)^2.
        Returns:
            q2A: angle en radians

            q2B: angle en radians
        """
        K1 = x * cos(q1) + y * sin(q1) - self.params.geometric_params.D2
        L1 = z
        L2 = -K1
        L3 = (self.params.geometric_params.R4**2 + self.params.geometric_params.D4**2 - K1**2 - z**2 - self.params.geometric_params.D3**2) / (2*self.params.geometric_params.D3)
        return MGI._solve_eq_type2(L1, L2, L3)

    def _compute_q3(self, x: float, y: float, z: float, q1: float):
        """
        Calcul de q3 depuis : T3_2 * T2_0 * U0 = T3_6. Equations (a14)^2 + (a24)^2.
        Returns:
            q3A: angle en radians
            
            q3B: angle en radians
        """
        K1 = x * cos(q1) + y * sin(q1) - self.params.geometric_params.D2
        L1 = self.params.geometric_params.D4
        L2 = self.params.geometric_params.R4
        L3 = (K1**2 + z**2 - self.params.geometric_params.D3**2 - self.params.geometric_params.R4**2 - self.params.geometric_params.D4**2) / (2*self.params.geometric_params.D3)
        return MGI._solve_eq_type2(L1, L2, L3)

    def _compute_q4(self, q1: float, q2: float, q3: float):
        """
        Calcul de q1 en fonction depuis : T3_2 * T2_0 * U0 = T3_6. Equations (a13) et (a33).
        Returns:
            singularity: bool

            q4: angle en radians
        """
               
        q23 = q2 + q3
        aq1 = self._resolution_vars.a_rad - q1
        
        Saq1 = sin(aq1)
        Caq1 = cos(aq1)

        Sb = self._resolution_vars.sb
        Cb = self._resolution_vars.cb

        Sc = self._resolution_vars.sc
        Cc = self._resolution_vars.cc

        Sb_Sc = Sb * Cc

        s5c4 = -(Sb_Sc * Caq1 + Sc * Saq1) * sin(q23) - Cb * Cc * cos(q23)
        s5s4 = Sb_Sc * Saq1  - Sc * Caq1

        return MGI._tryAtan2(s5s4, s5c4)

    def _compute_q5(self, q1: float, q2: float, q3: float, q4: float):
        """
        Calcul de q5 en fonction depuis : T4_3 * T3_0 * U0 = T4_6. Equations (a13) et (a33).
        Returns:
            q5: angle en radians
        """
        q23 = q2 + q3
        aq1 = self._resolution_vars.a_rad - q1
        
        Caq1 = cos(aq1)
        Saq1 = sin(aq1)

        Sb = self._resolution_vars.sb
        Cb = self._resolution_vars.cb

        Sc = self._resolution_vars.sc
        Cc = self._resolution_vars.cc

        Sb_Cc = Sb*Cc
        Cb_Cc = Cb*Cc

        S23 = sin(q23)
        C23 = cos(q23)

        s5 = (Sb_Cc * Saq1 - Sc * Caq1) * sin(q4) - ((Sb_Cc * Caq1 + Sc * Saq1) * S23 + Cb_Cc * C23) * cos(q4)
        c5 = (Sb_Cc * Caq1 + Sc * Saq1) * C23 - Cb_Cc * S23

        return MGI._tryAtan2(s5, c5)

    def _compute_q6(self, q1: float, q2: float, q3: float, q4: float):
        """
        Calcul de q6 en fonction depuis : T4_3 * T3_0 * U0 = T4_6. Equations (a21) et (a22).
        Returns:
            q6: angle en radians
        """
        q23 = q2 + q3
        aq1 = self._resolution_vars.a_rad - q1
        
        Caq1 = cos(aq1)
        Saq1 = sin(aq1)

        Sb = self._resolution_vars.sb
        Cb = self._resolution_vars.cb

        Sc = self._resolution_vars.sc
        Cc = self._resolution_vars.cc

        S4 = sin(q4)
        C4 = cos(q4)

        S23 = sin(q23)
        C23 = cos(q23)

        Sb_Sc = Sb * Sc
        S23_Caq1 = S23 * Caq1

        s6 = (-Sb*C23 + Cb*S23_Caq1)*S4 + Saq1*Cb*C4
        c6 = (Sb_Sc*Saq1 + Cc*Caq1)*C4 + (Sb_Sc*S23_Caq1 + Sc*Cb*C23 - Saq1*S23*Cc)*S4
        
        return MGI._tryAtan2(s6, c6)

    def _feed_q1(self, results: MgiResult, x: float, y: float, verbose=False) -> bool:
        """
        :param results: The result object to feed
        :type results: MgiResult
        :param x: X coordinate
        :type x: float
        :param y: Y coordinate
        :type y: float
        :param verbose: Should print verbose information
        :return: True if the process encountered a singularity on Q1
        :rtype: bool
        """
        singularityQ1 , q1Front = self._compute_q1(x, y)
        q1Back = MGI._add_pi(q1Front)
        if not self.params.configuration_identifier.is_front(q1Front):
            q1Front, q1Back = q1Back, q1Front

        if singularityQ1:
            if verbose:
                print("Singularité en Q1.")
                print()
        
            if self.params.singularities_behavior.q1_behavior == MgiSingularityBehavior.CONTINUE:
                q1Front = self.defaultQ1RadSingularityValue
                q1Back = MGI._add_pi(q1Front)
                if not self.params.configuration_identifier.is_front(q1Front):
                    q1Front, q1Back = q1Back, q1Front
                
                for result in results.solutions.items():
                    result[1].j1Singularity = True
                    result[1].setQ1(q1Front if result[0] in FRONT_CONFIG_KEYS else q1Back)

            else: # MgiSingularityBehavior.STOP
                for result in results.solutions.values():
                    result.status = MgiResultStatus.SINGULARITY
                    result.j1Singularity = True

        else: # No singularity
            for result in results.solutions.items():
                result[1].setQ1(q1Front if result[0] in FRONT_CONFIG_KEYS else q1Back)
        
        return singularityQ1

    def _feed_q2_q3(self, results: MgiResult, x: float, y: float, z: float, verbose=False):
        q1Front, q1Back = results.get_q1_front_back()
    
        # Front solutions

        solve_case_2A, q2Aa, q2Ab = self._compute_q2(x, y, z, q1Front)
        solve_case_3A, q3Aa, q3Ab = self._compute_q3(x, y, z, q1Front)

        if (solve_case_2A < 0 or solve_case_3A < 0):
            if verbose:
                print("Pas de solution pour", "Q2" if solve_case_2A else "Q3", "avec Q1Front.")
                print()
            
            for result in results.get_front_solutions().values():
                result.status = MgiResultStatus.UNREACHABLE
        else:
            if self.params.configuration_identifier.is_up(q3Aa):
                q2Up, q2Down = q2Aa, q2Ab
                q3Up, q3Down = q3Aa, q3Ab
            else:
                q2Up, q2Down = q2Ab, q2Aa
                q3Up, q3Down = q3Ab, q3Aa

            for result_key, result_value in results.get_front_solutions().items():
                if result_key in FRONT_UP_CONFIG_KEYS:
                    result_value.setQ2(q2Up)
                    result_value.setQ3(q3Up)
                else:
                    result_value.setQ2(q2Down)
                    result_value.setQ3(q3Down)

        # Back solutions

        solve_case_2A, q2Aa, q2Ab = self._compute_q2(x, y, z, q1Back)
        solve_case_3A, q3Aa, q3Ab = self._compute_q3(x, y, z, q1Back)

        if (solve_case_2A < 0 or solve_case_3A < 0):
            if verbose:
                print("Pas de solution pour", "Q2" if solve_case_2A else "Q3", "avec q1Back.")
                print()
            
            for result in results.get_back_solutions().values():
                result.status = MgiResultStatus.UNREACHABLE
        else:
            if self.params.configuration_identifier.is_up(q3Aa):
                q2Up, q2Down = q2Aa, q2Ab
                q3Up, q3Down = q3Aa, q3Ab
            else:
                q2Up, q2Down = q2Ab, q2Aa
                q3Up, q3Down = q3Ab, q3Aa

            for result_key, result_value in results.get_back_solutions().items():
                if result_key in BACK_UP_CONFIG_KEYS:
                    result_value.setQ2(q2Up)
                    result_value.setQ3(q3Up)
                else:
                    result_value.setQ2(q2Down)
                    result_value.setQ3(q3Down)

    def _feed_q4_q5_q6(self, results: MgiResult, verbose=False):
        
        front_up_no_flipped_solution = results.get_solution(MgiConfigKey.FUN)
        front_up_flipped_solution = results.get_solution(MgiConfigKey.FUF)

        front_down_no_flipped_solution = results.get_solution(MgiConfigKey.FDN)
        front_down_flipped_solution = results.get_solution(MgiConfigKey.FDF)

        back_up_no_flipped_solution = results.get_solution(MgiConfigKey.BUN)
        back_up_flipped_solution = results.get_solution(MgiConfigKey.BUF)
        
        back_down_no_flipped_solution = results.get_solution(MgiConfigKey.BDN)
        back_down_flipped_solution = results.get_solution(MgiConfigKey.BDF)

        q1A = front_up_no_flipped_solution.joints[0]
        q1B = back_up_no_flipped_solution.joints[0]

        q2Aa = front_up_no_flipped_solution.joints[1]
        q2Ab = front_down_no_flipped_solution.joints[1]
        q2Ba = back_up_no_flipped_solution.joints[1]
        q2Bb = back_down_no_flipped_solution.joints[1]

        q3Aa = front_up_no_flipped_solution.joints[2]
        q3Ab = front_down_no_flipped_solution.joints[2]
        q3Ba = back_up_no_flipped_solution.joints[2]
        q3Bb = back_down_no_flipped_solution.joints[2]

        # Front, Up, Flip and No Flip solutions

        if front_up_no_flipped_solution.status != MgiResultStatus.UNREACHABLE:
            singularityQ5, q4Aa1 = self._compute_q4(q1A, q2Aa, q3Aa)

            if singularityQ5:
                if verbose:
                    print("Singularité en Q5 - Configuration Front, Up")
                    print()

                front_up_no_flipped_solution.j5Singularity = True
                front_up_flipped_solution.j5Singularity = True

                q4Aa1 = self.defaultQ4RadSingularityValue
                q4Aa2 = MGI._add_pi(q4Aa1)

                if self.params.configuration_identifier.is_flipped(q4Aa1):
                    q4Aa1, q4Aa2 = q4Aa2, q4Aa1 # q4Aa1 is no flipped

                q5Aa1, q5Aa2 = 0, 0 # Peut etre qu'il faudra ajouter un test pour savoir si c'est 0, pi ou -pi

                q6Aa1  = self.defaultQ6RadSingularityValue
                q6Aa2 = MGI._add_pi(q6Aa1)

                front_up_no_flipped_solution.setQ456(q4Aa1, q5Aa1, q6Aa1)
                front_up_flipped_solution.setQ456(q4Aa2, q5Aa2, q6Aa2)

            else:
                q4Aa2 = MGI._add_pi(q4Aa1)
                if self.params.configuration_identifier.is_flipped(q4Aa1):
                    q4Aa1, q4Aa2 = q4Aa2, q4Aa1 # q4Aa1 is no flipped

                _, q5Aa1 = self._compute_q5(q1A, q2Aa, q3Aa, q4Aa1)
                _, q5Aa2 = self._compute_q5(q1A, q2Aa, q3Aa, q4Aa2)
                _, q6Aa1 = self._compute_q6(q1A, q2Aa, q3Aa, q4Aa1)
                _, q6Aa2 = self._compute_q6(q1A, q2Aa, q3Aa, q4Aa2)

                front_up_no_flipped_solution.setQ456(q4Aa1, q5Aa1, q6Aa1)
                front_up_flipped_solution.setQ456(q4Aa2, q5Aa2, q6Aa2)

        # Front, Down, Flip and No Flip solutions

        if front_down_no_flipped_solution.status != MgiResultStatus.UNREACHABLE:
            singularityQ5, q4Ab1 = self._compute_q4(q1A, q2Ab, q3Ab)

            if singularityQ5:
                if verbose:
                    print("Singularité en Q5 - Configuration Front, Down")
                    print()

                front_down_no_flipped_solution.j5Singularity = True
                front_down_flipped_solution.j5Singularity = True

                q4Ab1 = self.defaultQ4RadSingularityValue
                q4Ab2 = MGI._add_pi(q4Ab1)
                if self.params.configuration_identifier.is_flipped(q4Ab1):
                    q4Ab1, q4Ab2 = q4Ab2, q4Ab1 # q4Ab1 is no flipped

                q5Ab1, q5Ab2 = 0, 0 # Peut etre qu'il faudra ajouter un test pour savoir si c'est 0, pi ou -pi
                q6Ab1  = self.defaultQ6RadSingularityValue
                q6Ab2 = MGI._add_pi(q6Ab1)

                front_down_no_flipped_solution.setQ456(q4Ab1, q5Ab1, q6Ab1)
                front_down_flipped_solution.setQ456(q4Ab2, q5Ab2, q6Ab2)

            else:
                q4Ab2 = MGI._add_pi(q4Ab1)
                if self.params.configuration_identifier.is_flipped(q4Ab1):
                    q4Ab1, q4Ab2 = q4Ab2, q4Ab1 # q4Ab1 is no flipped

                _, q5Ab1 = self._compute_q5(q1A, q2Ab, q3Ab, q4Ab1)
                _, q5Ab2 = self._compute_q5(q1A, q2Ab, q3Ab, q4Ab2)
                _, q6Ab1 = self._compute_q6(q1A, q2Ab, q3Ab, q4Ab1)
                _, q6Ab2 = self._compute_q6(q1A, q2Ab, q3Ab, q4Ab2)

                front_down_no_flipped_solution.setQ456(q4Ab1, q5Ab1, q6Ab1)
                front_down_flipped_solution.setQ456(q4Ab2, q5Ab2, q6Ab2)

        # Back, Up, Flip and No Flip solutions

        if back_up_no_flipped_solution.status != MgiResultStatus.UNREACHABLE:
            singularityQ5, q4Ba1 = self._compute_q4(q1B, q2Ba, q3Ba)

            if singularityQ5:
                if verbose:
                    print("Singularité en Q5 - Configuration Back, Up")
                    print()

                back_up_no_flipped_solution.j5Singularity = True
                back_up_flipped_solution.j5Singularity = True

                q4Ba1 = self.defaultQ4RadSingularityValue
                q4Ba2 = MGI._add_pi(q4Ba1)
                if self.params.configuration_identifier.is_flipped(q4Ba1):
                    q4Ba1, q4Ba2 = q4Ba2, q4Ba1 # q4Ba1 is no flipped

                q5Ba1, q5Ba2 = 0, 0 # Peut etre qu'il faudra ajouter un test pour savoir si c'est 0, pi ou -pi

                q6Ba1  = self.defaultQ6RadSingularityValue
                q6Ba2 = MGI._add_pi(q6Ba1)

                back_up_no_flipped_solution.setQ456(q4Ba1, q5Ba1, q6Ba1)
                back_up_flipped_solution.setQ456(q4Ba2, q5Ba2, q6Ba2)

            else:
                q4Ba2 = MGI._add_pi(q4Ba1)
                if self.params.configuration_identifier.is_flipped(q4Ba1):
                    q4Ba1, q4Ba2 = q4Ba2, q4Ba1 # q4Ba1 is no flipped

                _, q5Ba1 = self._compute_q5(q1B, q2Ba, q3Ba, q4Ba1)
                _, q5Ba2 = self._compute_q5(q1B, q2Ba, q3Ba, q4Ba2)
                _, q6Ba1 = self._compute_q6(q1B, q2Ba, q3Ba, q4Ba1)
                _, q6Ba2 = self._compute_q6(q1B, q2Ba, q3Ba, q4Ba2)

                back_up_no_flipped_solution.setQ456(q4Ba1, q5Ba1, q6Ba1)
                back_up_flipped_solution.setQ456(q4Ba2, q5Ba2, q6Ba2)

        # Back, Down, Flip and No Flip solutions

        if back_down_no_flipped_solution.status != MgiResultStatus.UNREACHABLE:
            singularityQ5, q4Bb1 = self._compute_q4(q1B, q2Bb, q3Bb)

            if singularityQ5:
                if verbose:
                    print("Singularité en Q5 - Configuration Back, Down")
                    print()

                back_down_no_flipped_solution.j5Singularity = True
                back_down_flipped_solution.j5Singularity = True

                q4Bb1 = self.defaultQ4RadSingularityValue
                q4Bb2 = MGI._add_pi(q4Bb1)
                if self.params.configuration_identifier.is_flipped(q4Bb1):
                    q4Bb1, q4Bb2 = q4Bb2, q4Bb1 # q4Bb1 is no flipped

                q5Bb1, q5Bb2 = 0, 0 # Peut etre qu'il faudra ajouter un test pour savoir si c'est 0, pi ou -pi

                q6Bb1  = self.defaultQ6RadSingularityValue
                q6Bb2 = MGI._add_pi(q6Bb1)

                back_down_no_flipped_solution.setQ456(q4Bb1, q5Bb1, q6Bb1)
                back_down_flipped_solution.setQ456(q4Bb2, q5Bb2, q6Bb2)

            else:
                q4Bb2 = MGI._add_pi(q4Bb1)
                if self.params.configuration_identifier.is_flipped(q4Bb1):
                    q4Bb1, q4Bb2 = q4Bb2, q4Bb1 # q4Bb1 is no flipped

                _, q5Bb1 = self._compute_q5(q1B, q2Bb, q3Bb, q4Bb1)
                _, q5Bb2 = self._compute_q5(q1B, q2Bb, q3Bb, q4Bb2)
                _, q6Bb1 = self._compute_q6(q1B, q2Bb, q3Bb, q4Bb1)
                _, q6Bb2 = self._compute_q6(q1B, q2Bb, q3Bb, q4Bb2)

                back_down_no_flipped_solution.setQ456(q4Bb1, q5Bb1, q6Bb1)
                back_down_flipped_solution.setQ456(q4Bb2, q5Bb2, q6Bb2)

    def compute_mgi(self, x: float, y: float, z: float, a_deg: float, b_deg: float, c_deg: float, returnDegrees: bool = True, verbose=False):
        if verbose:
            MGI._display_coordinates("Position cible demandée :", x, y, z, a_deg, b_deg, c_deg)

        # Tool to Flange
        _x, _y, _z, _a, _b, _c = MGI._compute_tool_to_flange_coordinates(x, y, z, a_deg, b_deg, c_deg, self.tool)
        if verbose:
            MGI._display_coordinates("Position du flange :", _x, _y, _z, _a, _b, _c, "f")

        # MGI Coordinates
        _x, _y, _z, _a, _b, _c = MGI._compute_flange_to_mgi_coordinates(_x, _y, _z, _a, _b, _c, self.params.geometric_params.R1, self.params.geometric_params.R6)
        if verbose:
            MGI._display_coordinates("Position pour MGI simplifié :", _x, _y, _z, _a, _b, _c, "c")
        
        self._compute_radians(_a, _b, _c)

        results = MgiResult()
        # Créer une copie pour ne pas modifier l'original
        axis_limits_to_use = self.params.axis_limits

        # Q1
        if self._feed_q1(results, _x, _y, verbose) and self.params.singularities_behavior.q1_behavior == MgiSingularityBehavior.STOP:
            # Solutions are marked as SINGULARITY
            return results

        # Q2, Q3
        self._feed_q2_q3(results, _x, _y, _z, verbose)
        
        # After Q1, Q2, Q3, some solutions can already be marked as UNREACHABLE
        # UNREACHABLE solutions won't be processed further

        # Q4, Q5, Q6
        self._feed_q4_q5_q6(results, verbose)
        
        # Inversion des axes selon constructeur
        results.apply_invert_table(self.params.invert_table)

        # Verification des limites d'axes
        # Créer une copie pour ne pas modifier l'original
        axis_limits_to_use = self.params.axis_limits
        
        if returnDegrees:
            if self.params.axis_limits.radians:
                results.apply_axis_limits(axis_limits_to_use)
                results.to_degrees()
            else:
                results.to_degrees() 
                results.apply_axis_limits(axis_limits_to_use)
        else:
            if not self.params.axis_limits.radians:
                # Créer une copie temporaire convertie
                axis_limits_rad = MgiAxisLimits(True, [(radians(qMin), radians(qMax)) for (qMin, qMax) in axis_limits_to_use.axis_limits])
                results.apply_axis_limits(axis_limits_rad)
            else:
                results.apply_axis_limits(axis_limits_to_use)
            
        # Appliquer le filtre des configurations
        results.filter_configurations(self.params.configuration_filter)

        results.all_solutions_evaluated = True
        return results

    def compute_mgi_target(self, target: list[float], returnDegrees: bool = True, verbose=False):
        return self.compute_mgi(target[0], target[1], target[2], target[3], target[4], target[5], returnDegrees, verbose)

    @staticmethod
    def _display_coordinates(title: str, x: float, y: float, z: float, a: float, b: float, c: float, lblSuffix: str = ""):
        print(title)
        print("X{}: {:.3f} mm".format(lblSuffix, x))
        print("Y{}: {:.3f} mm".format(lblSuffix, y))
        print("Z{}: {:.3f} mm".format(lblSuffix, z))
        print("A{}: {:.3f} deg".format(lblSuffix, a))
        print("B{}: {:.3f} deg".format(lblSuffix, b))
        print("C{}: {:.3f} deg".format(lblSuffix, c))
        print()
