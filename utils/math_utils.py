import numpy as np
from PyQt6.QtWidgets import QTableWidget

# ============================================================================
# RÉGION: Parsing et utilitaires
# ============================================================================

def parse_value(expr: str):
    """Parse une expression mathématique contenant potentiellement 'pi'"""
    
    try:
        expr = expr.replace("pi", "np.pi")
        return eval(expr, {"np": np})
    except Exception:
        raise ValueError(f"Expression invalide: {expr}")

def get_cell_value(table: QTableWidget, row: int, col: int, default=0):
    """Récupère la valeur d'une cellule de table Qt"""
    item = table.item(row, col)
    if item and item.text().strip() != "":
        return parse_value(item.text())
    return default

# ============================================================================
# RÉGION: Transformations Denavit-Hartenberg
# ============================================================================

def dh_modified(alpha: float, d: float, theta: float, r: float):
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

def correction_6d(T, tx: float, ty: float, tz: float, rx: float, ry: float, rz: float):
    """Applique une correction 6D (translation + rotation ZYX) à une matrice homogène
    
    Args:
        T: Matrice homogène 4x4
        tx, ty, tz: Translation en mm
        rx, ry, rz: Rotation en degrés (ZYX Euler angles)
    
    Returns:
        Matrice homogène corrigée
    """
    rx, ry, rz = np.radians([rx, ry, rz])
    #Rx = np.array([[1, 0, 0],
    #               [0, np.cos(rx), -np.sin(rx)],
    #               [0, np.sin(rx), np.cos(rx)]])
    #Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
    #               [0, 1, 0],
    #               [-np.sin(ry), 0, np.cos(ry)]])
    #Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
    #               [np.sin(rz), np.cos(rz), 0],
    #               [0, 0, 1]])
    #R = Rz @ Ry @ Rx  # Rotation Fixed angles ZYX
    R = rot_z(rz) @ rot_y(ry) @ rot_x(rx)

    corr = np.eye(4)
    corr[:3, :3] = R
    corr[:3, 3] = [tx, ty, tz]
    return T @ corr

# ============================================================================
# RÉGION: Conversions angles d'Euler
# ============================================================================

def rot_x(angle: float, degrees=True):
    if degrees:
        angle = np.radians(angle)
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])

def rot_y(angle: float, degrees=True):
    if degrees:
        angle = np.radians(angle)
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])

def rot_z(angle: float, degrees=True):
    if degrees:
        angle = np.radians(angle)
    c = np.cos(angle)
    s = np.sin(angle)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])

def euler_to_rotation_matrix(A: float, B: float, C: float, degrees=True):
    """Convertit des angles d'Euler ZYX en matrice de rotation 3x3
    
    Args:
        A, B, C: Angles de rotation (degrés ou radians)
        degrees: Si True, les angles sont en degrés
    
    Returns:
        Matrice de rotation 3x3
    """   
    return rot_z(A, degrees) @ rot_y(B, degrees) @ rot_x(C, degrees)

def matrix_to_euler_zyx(T):
    """
    Extrait les angles d'Euler ZYX (en degrés) d'une matrice homogène 4x4.
    Args:
        T matrice 4x4
    Returns:
        Array [A, B, C] en degrés
    """
    return rotation_matrix_to_euler_zyx(T[:3, :3])

def rotation_matrix_to_euler_zyx(R):
    """Extrait les angles d'Euler ZYX (en degrés) d'une matrice de rotation 3x3
    
    Args:
        R: Matrice de rotation 3x3
    
    Returns:
        Array [A, B, C] en degrés
    """
    B = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))

    if np.isclose(B, np.pi/2, atol=1e-5):
        # B == pi/2
        A = 0
        C = np.atan2(R[0, 1], R[1, 1])
    elif np.isclose(B, -np.pi/2, atol=1e-5):
        # B == -pi/2
        A = 0
        C = -np.atan2(R[0, 1], R[1, 1])
    else:
        C = np.atan2(R[2, 1], R[2, 2])
        A = np.atan2(R[1, 0], R[0, 0])
    return np.degrees([A, B, C])
