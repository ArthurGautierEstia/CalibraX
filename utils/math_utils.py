import numpy as np

# ============================================================================
# RÉGION: Parsing et utilitaires
# ============================================================================

def parse_value(expr):
    """Parse une expression mathématique contenant potentiellement 'pi'"""
    try:
        expr = expr.replace("pi", "np.pi")
        return eval(expr, {"np": np})
    except Exception:
        raise ValueError(f"Expression invalide: {expr}")

def get_cell_value(table, row, col, default=0):
    """Récupère la valeur d'une cellule de table Qt"""
    item = table.item(row, col)
    if item and item.text().strip() != "":
        return parse_value(item.text())
    return default

# ============================================================================
# RÉGION: Transformations Denavit-Hartenberg
# ============================================================================

def dh_modified(alpha, d, theta, r):
    """Calcule la matrice de transformation DH modifiée (4x4)
    
    Args:
        alpha: Angle de rotation autour de X (radians)
        d: Distance selon Z
        theta: Angle de rotation autour de Z (radians)
        r: Distance selon X
    
    Returns:
        Matrice homogène 4x4
    """
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st, 0, d],
        [st*ca, ct*ca, -sa, -r*sa],
        [st*sa, ct*sa, ca, r*ca],
        [0, 0, 0, 1]
    ])

# ============================================================================
# RÉGION: Corrections 6D
# ============================================================================

def correction_6d(T, tx, ty, tz, rx, ry, rz):
    """Applique une correction 6D (translation + rotation ZYX) à une matrice homogène
    
    Args:
        T: Matrice homogène 4x4
        tx, ty, tz: Translation en mm
        rx, ry, rz: Rotation en degrés (ZYX Euler angles)
    
    Returns:
        Matrice homogène corrigée
    """
    rx, ry, rz = np.radians([rx, ry, rz])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    R = Rz @ Ry @ Rx  # Rotation Fixed angles ZYX
    corr = np.eye(4)
    corr[:3, :3] = R
    corr[:3, 3] = [tx, ty, tz]
    return T @ corr

# ============================================================================
# RÉGION: Conversions angles d'Euler
# ============================================================================

def matrix_to_euler_zyx(T):
    """Extrait les angles d'Euler ZYX (en degrés) d'une matrice homogène 4x4
    
    Args:
        T: Matrice homogène 4x4
    
    Returns:
        Array [Rz, Ry, Rx] en degrés
    """
    rx = np.degrees(np.arctan2(T[2, 1], T[2, 2]))
    ry = np.degrees(np.arctan2(-T[2, 0], np.sqrt(T[2, 1]**2 + T[2, 2]**2)))
    rz = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
    return np.array([rz, ry, rx])

def euler_to_rotation_matrix(A, B, C, degrees=True):
    """Convertit des angles d'Euler ZYX en matrice de rotation 3x3
    
    Args:
        A, B, C: Angles de rotation (degrés ou radians)
        degrees: Si True, les angles sont en degrés
    
    Returns:
        Matrice de rotation 3x3
    """
    if degrees:
        A, B, C = np.radians([A, B, C])
    
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(C), -np.sin(C)],
                   [0, np.sin(C), np.cos(C)]])
    Ry = np.array([[np.cos(B), 0, np.sin(B)],
                   [0, 1, 0],
                   [-np.sin(B), 0, np.cos(B)]])
    Rz = np.array([[np.cos(A), -np.sin(A), 0],
                   [np.sin(A), np.cos(A), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

def rotation_matrix_to_euler_zyx(R):
    """Extrait les angles d'Euler ZYX (en degrés) d'une matrice de rotation 3x3
    
    Args:
        R: Matrice de rotation 3x3
    
    Returns:
        Array [A, B, C] en degrés
    """
    B = np.arcsin(-R[2, 0])
    A = np.arctan2(R[2, 1], R[2, 2])
    C = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([A, B, C])

# ============================================================================
# RÉGION: Cinématique directe (MGD)
# ============================================================================

def compute_forward_kinematics(robot_model):
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
    
    # Calcul itératif des transformations pour 6 joints + outil
    for i in range(7):
        # Récupérer les paramètres DH
        alpha = np.radians(robot_model.get_dh_param(i, 0))
        d = robot_model.get_dh_param(i, 1)
        theta_offset = np.radians(robot_model.get_dh_param(i, 2))
        r = robot_model.get_dh_param(i, 3)
        
        # Pour les 6 premiers joints, ajouter la valeur articulaire
        if i < 6:
            q_deg = robot_model.get_reel_joint_value(i)
            q = np.radians(q_deg)
            theta = theta_offset + q
            corr = robot_model.get_correction_joint(i)
        else:
            # Joint 7 (tool) : pas de variable articulaire
            theta = theta_offset
            corr = [0, 0, 0, 0, 0, 0]
        
        # Transformation DH standard
        T_dh = T_dh @ dh_modified(alpha, d, theta, r)
        dh_matrices.append(T_dh.copy())
        
        # Transformation avec correction
        T_corrected = T_corrected @ dh_modified(alpha, d, theta, r)
        T_corrected = correction_6d(T_corrected, *corr)
        corrected_matrices.append(T_corrected.copy())
    
    # Extraction position et orientation
    dh_pos = T_dh[:3, 3]
    dh_ori = matrix_to_euler_zyx(T_dh)
    dh_pose = np.concatenate([dh_pos, dh_ori])
    
    corrected_pos = T_corrected[:3, 3]
    corrected_ori = matrix_to_euler_zyx(T_corrected)
    corrected_pose = np.concatenate([corrected_pos, corrected_ori])
    
    # Calcul de la déviation
    pos_dev = corrected_pos - dh_pos
    ori_dev = corrected_ori - dh_ori
    deviation = np.concatenate([pos_dev, ori_dev])
    
    return dh_matrices, corrected_matrices, dh_pose, corrected_pose, deviation

def get_tcp_pose(robot_model):
    """Retourne la pose TCP standard (sans correction)
    
    Args:
        robot_model: RobotModel instance
    
    Returns:
        Array [x, y, z, rx, ry, rz]
    """
    _, _, dh_pose, _, _ = compute_forward_kinematics(robot_model)
    return dh_pose

def get_corrected_tcp_pose(robot_model):
    """Retourne la pose TCP corrigée
    
    Args:
        robot_model: RobotModel instance
    
    Returns:
        Array [x, y, z, rx, ry, rz]
    """
    _, _, _, corrected_pose, _ = compute_forward_kinematics(robot_model)
    return corrected_pose

def get_pose_deviation(robot_model):
    """Retourne les écarts entre TCP standard et corrigé
    
    Args:
        robot_model: RobotModel instance
    
    Returns:
        Array [dx, dy, dz, drx, dry, drz]
    """
    _, _, _, _, deviation = compute_forward_kinematics(robot_model)
    return deviation

def get_all_matrices(robot_model, corrected=False):
    """Retourne toutes les matrices de transformation jusqu'au TCP
    
    Args:
        robot_model: RobotModel instance
        corrected: Si True, retourne les matrices corrigées
    
    Returns:
        List de matrices 4x4
    """
    dh_matrices, corrected_matrices, _, _, _ = compute_forward_kinematics(robot_model)
    return corrected_matrices if corrected else dh_matrices

def get_matrix_at_joint(robot_model, joint_index, corrected=False):
    """Retourne la matrice de transformation à un joint donné
    
    Args:
        robot_model: RobotModel instance
        joint_index: Index du joint (0-6, où 6 est l'outil)
        corrected: Si True, retourne la matrice corrigée
    
    Returns:
        Matrice homogène 4x4
    """
    matrices = get_all_matrices(robot_model, corrected)
    if 0 <= joint_index < len(matrices):
        return matrices[joint_index]
    return np.eye(4)