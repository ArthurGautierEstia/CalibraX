import numpy as np

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

def dh_modified(alpha, d, theta, r):
    """Calcule la matrice de transformation DH modifiée (4x4)"""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st, 0, d],
        [st*ca, ct*ca, -sa, -r*sa],
        [st*sa, ct*sa, ca, r*ca],
        [0, 0, 0, 1]
    ])

def correction_6d(T, tx, ty, tz, rx, ry, rz):
    """Applique une correction 6D (translation + rotation ZYX) à une matrice homogène"""
    rx, ry, rz = np.radians([rx, ry, rz])
    Rx = np.array([[1,0,0],
                   [0,np.cos(rx),-np.sin(rx)],
                   [0,np.sin(rx),np.cos(rx)]])
    Ry = np.array([[np.cos(ry),0,np.sin(ry)],
                   [0,1,0],
                   [-np.sin(ry),0,np.cos(ry)]])
    Rz = np.array([[np.cos(rz),-np.sin(rz),0],
                   [np.sin(rz),np.cos(rz),0],
                   [0,0,1]])
    R = Rz @ Ry @ Rx  # Rotation Fixed angles ZYX
    corr = np.eye(4)
    corr[:3,:3] = R
    corr[:3,3] = [tx, ty, tz]
    return T @ corr

def matrix_to_euler_zyx(T):
    """Extrait les angles d'Euler ZYX (en degrés) d'une matrice homogène 4x4"""
    rx = np.degrees(np.arctan2(T[2,1], T[2,2]))
    ry = np.degrees(np.arctan2(-T[2,0], np.sqrt(T[2,1]**2 + T[2,2]**2)))
    rz = np.degrees(np.arctan2(T[1,0], T[0,0]))
    return np.array([rx, ry, rz])

def euler_to_rotation_matrix(A, B, C, degrees=True):
    """Convertit des angles d'Euler ZYX en matrice de rotation 3x3"""
    if degrees:
        A, B, C = np.radians([A, B, C])
    
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(A), -np.sin(A)],
                   [0, np.sin(A), np.cos(A)]])
    Ry = np.array([[np.cos(B), 0, np.sin(B)],
                   [0, 1, 0],
                   [-np.sin(B), 0, np.cos(B)]])
    Rz = np.array([[np.cos(C), -np.sin(C), 0],
                   [np.sin(C), np.cos(C), 0],
                   [0, 0, 1]])
    return Rz @ Ry @ Rx

def rotation_matrix_to_euler_zyx(R):
    """Extrait les angles d'Euler ZYX (en degrés) d'une matrice de rotation 3x3"""
    B = np.arcsin(-R[2, 0])
    A = np.arctan2(R[2, 1], R[2, 2])
    C = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([A, B, C])
