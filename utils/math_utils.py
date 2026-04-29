from collections.abc import Iterable

import numpy as np

# ============================================================================
# RÉGION: Parsing et utilitaires
# ============================================================================

def safe_float(value: object, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        if isinstance(value, str):
            stripped = value.strip()
            if stripped == "":
                return default
            return float(stripped)
        return float(value)
    except (TypeError, ValueError):
        return default


def norm3(x: float, y: float, z: float) -> float:
    """Euclidean norm in 3D."""
    return float(np.sqrt(x * x + y * y + z * z))


def vector_norm3(v: list[float] | tuple[float, float, float]) -> float:
    """Euclidean norm of [x, y, z]."""
    if len(v) < 3:
        return 0.0
    return norm3(float(v[0]), float(v[1]), float(v[2]))


def normalize3(v: list[float] | tuple[float, float, float], epsilon: float = 1e-9) -> list[float]:
    """Normalize [x, y, z], returning [0,0,0] if norm is too small."""
    n = vector_norm3(v)
    if n <= float(epsilon):
        return [0.0, 0.0, 0.0]
    return [float(v[0]) / n, float(v[1]) / n, float(v[2]) / n]

def is_near_zero_vector_xyz(vector_xyz: list[float], epsilon: float = 1e-9) -> bool:
    if len(vector_xyz) < 3:
        return False
    return (
        abs(float(vector_xyz[0])) <= epsilon
        and abs(float(vector_xyz[1])) <= epsilon
        and abs(float(vector_xyz[2])) <= epsilon
    )


def normalize_pose6(values: object) -> list[float]:
    """Normalize a project XYZABC pose to six floats."""
    if isinstance(values, dict):
        out = [
            safe_float(values.get("x", 0.0), 0.0),
            safe_float(values.get("y", 0.0), 0.0),
            safe_float(values.get("z", 0.0), 0.0),
            safe_float(values.get("a", 0.0), 0.0),
            safe_float(values.get("b", 0.0), 0.0),
            safe_float(values.get("c", 0.0), 0.0),
        ]
    elif isinstance(values, Iterable) and not isinstance(values, (str, bytes)):
        seq = list(values)
        out = []
        for idx in range(6):
            out.append(safe_float(seq[idx] if idx < len(seq) else 0.0, 0.0))
    else:
        out = [0.0] * 6
    return out[:6]

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
    """Construit une matrice 3x3 depuis les angles Kuka ZYX [A, B, C].

    Convention canonique du projet:
        R = Rz(A) @ Ry(B) @ Rx(C)

    Args:
        A, B, C: angles avec A=Rz, B=Ry, C=Rx
        degrees: Si True, les angles sont en degrés

    Returns:
        Matrice de rotation 3x3
    """
    return rot_z(A, degrees) @ rot_y(B, degrees) @ rot_x(C, degrees)

def matrix_to_euler_zyx(T):
    """
    Extrait les angles Kuka ZYX [A, B, C] d'une matrice homogène 4x4.

    Args:
        T: matrice homogène 4x4

    Returns:
        Array [A, B, C] en degrés avec A=Rz, B=Ry, C=Rx
    """
    return rotation_matrix_to_euler_zyx(T[:3, :3])


def rotation_matrix_to_euler_zyx(R):
    """Extrait les angles Kuka ZYX [A, B, C] d'une matrice de rotation 3x3.

    Convention canonique du projet:
        R = Rz(A) @ Ry(B) @ Rx(C)

    Args:
        R: Matrice de rotation 3x3

    Returns:
        Array [A, B, C] en degrés avec A=Rz, B=Ry, C=Rx
    """
    B = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
    cos_b = np.cos(B)

    if np.abs(cos_b) > 1e-9:
        A = np.arctan2(R[1, 0], R[0, 0])
        C = np.arctan2(R[2, 1], R[2, 2])
    else:
        # Gimbal lock: une infinité de couples (A, C) décrivent la même
        # orientation. On fige C à 0 et on absorbe le résiduel dans A.
        A = np.arctan2(-R[0, 1], R[1, 1])
        C = 0.0

    return np.degrees([A, B, C])


def rotation_matrix_to_fixed_xyz(R):
    """Extrait les angles Fixed XYZ [Rx, Ry, Rz] d'une matrice 3x3.

    Cette représentation décrit la même orientation que la convention
    canonique ZYX du projet, mais avec des rotations extrinsèques XYZ.

    Args:
        R: Matrice de rotation 3x3

    Returns:
        Array [Rx, Ry, Rz] en degrés
    """
    # ZYX canonique renvoie [A=Rz, B=Ry, C=Rx].
    # Fixed XYZ attend [Rx, Ry, Rz] => [C, B, A].
    euler_zyx = rotation_matrix_to_euler_zyx(R)
    return np.array([euler_zyx[2], euler_zyx[1], euler_zyx[0]], dtype=float)


def rotation_matrix_to_euler_xyz(R):
    """Extrait les angles Euler XYZ [Rx, Ry, Rz] d'une matrice de rotation 3x3.

    Convention Euler XYZ (intrinsèque): R = Rx(A) @ Ry(B) @ Rz(C)

    Returns:
        Array [Rx, Ry, Rz] en degrés
    """
    B = np.arctan2(R[0, 2], np.sqrt(R[0, 0] ** 2 + R[0, 1] ** 2))
    cos_b = np.cos(B)

    if np.abs(cos_b) > 1e-9:
        A = np.arctan2(-R[1, 2], R[2, 2])
        C = np.arctan2(-R[0, 1], R[0, 0])
    else:
        # Gimbal lock: une infinité de couples (Rx, Rz) décrivent la même
        # orientation. On fige Rz à 0 et on absorbe le résiduel dans Rx.
        C = 0.0
        if B >= 0.0:
            A = np.arctan2(R[1, 0], -R[2, 0])
        else:
            A = np.arctan2(-R[1, 0], R[2, 0])

    return np.degrees([A, B, C])


def rotation_matrix_to_fixed_zyx(R):
    """Extrait les angles Fixed ZYX [Rz, Ry, Rx] d'une matrice de rotation 3x3.

    Convention Fixed ZYX (extrinsèque): R = Rx(C) @ Ry(B) @ Rz(A)

    Returns:
        Array [Rz, Ry, Rx] en degrés
    """
    # Euler XYZ renvoie [Rx, Ry, Rz].
    # Fixed ZYX attend [Rz, Ry, Rx].
    euler_xyz = rotation_matrix_to_euler_xyz(R)
    return np.array([euler_xyz[2], euler_xyz[1], euler_xyz[0]], dtype=float)


# ============================================================================
# RÉGION: Transitions
# ============================================================================

def pose_zyx_to_matrix(pose: object) -> np.ndarray:
    """Build a 4x4 matrix from a project pose [X, Y, Z, A, B, C]."""
    values = normalize_pose6(pose)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = euler_to_rotation_matrix(values[3], values[4], values[5], degrees=True)
    transform[:3, 3] = [values[0], values[1], values[2]]
    return transform


def matrix_to_pose_zyx(transform: np.ndarray) -> list[float]:
    """Extract a project pose [X, Y, Z, A, B, C] from a 4x4 matrix."""
    matrix = np.array(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("La matrice doit etre de taille 4x4")
    angles = rotation_matrix_to_euler_zyx(matrix[:3, :3])
    return [
        float(matrix[0, 3]),
        float(matrix[1, 3]),
        float(matrix[2, 3]),
        float(angles[0]),
        float(angles[1]),
        float(angles[2]),
    ]


def invert_homogeneous_transform(transform: np.ndarray, validate: bool = False) -> np.ndarray:
    """Invert a rigid 4x4 homogeneous transform."""
    matrix = np.array(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("La matrice doit etre de taille 4x4")
    if validate and not np.allclose(matrix[3, :], np.array([0.0, 0.0, 0.0, 1.0]), atol=1e-12):
        raise ValueError("La matrice n'est pas une transformation homogene rigide valide")

    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    inverse = np.eye(4, dtype=float)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -(rotation.T @ translation)
    return inverse


def orthonormalize_rotation(rotation: np.ndarray) -> np.ndarray:
    """Project a near-rotation matrix onto SO(3)."""
    matrix = np.array(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("La matrice de rotation doit etre de taille 3x3")
    U, _, Vt = np.linalg.svd(matrix)
    rotation_ortho = U @ Vt
    if np.linalg.det(rotation_ortho) < 0:
        U[:, -1] *= -1.0
        rotation_ortho = U @ Vt
    return rotation_ortho


def rotation_matrix_to_rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Return an axis-angle vector in radians from a 3x3 rotation matrix."""
    matrix = np.array(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("La matrice de rotation doit etre de taille 3x3")

    cos_theta = np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_theta)

    if abs(theta) < 1e-10:
        return np.zeros(3)

    if abs(theta - np.pi) < 1e-6:
        axis_matrix = matrix + np.eye(3)
        col_norms = [np.linalg.norm(axis_matrix[:, j]) for j in range(3)]
        best_col = int(np.argmax(col_norms))
        best_norm = col_norms[best_col]
        if best_norm <= 1e-12:
            return np.zeros(3)
        axis = axis_matrix[:, best_col] / best_norm
        return axis * theta

    axis = np.array([
        matrix[2, 1] - matrix[1, 2],
        matrix[0, 2] - matrix[2, 0],
        matrix[1, 0] - matrix[0, 1],
    ]) / (2.0 * np.sin(theta))
    return axis * theta


def homogeneous_rotation_z(angle: float, degrees: bool = True) -> np.ndarray:
    """Build a 4x4 homogeneous rotation around Z."""
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rot_z(angle, degrees=degrees)
    return transform


def transform_xyz_limits_yaw_only(
    limits_xyz: list[tuple[float, float]],
    pose_world: object,
) -> list[tuple[float, float]]:
    """Transform base XYZ slider limits into world using translation and A/Rz only."""
    pose = normalize_pose6(pose_world)
    tx, ty, tz = pose[:3]
    rz_rad = np.radians(pose[3])
    cos_rz = float(np.cos(rz_rad))
    sin_rz = float(np.sin(rz_rad))

    normalized_limits: list[tuple[float, float]] = []
    for idx in range(3):
        try:
            min_val = float(limits_xyz[idx][0])
            max_val = float(limits_xyz[idx][1])
        except (TypeError, ValueError, IndexError):
            min_val = 0.0
            max_val = 0.0
        normalized_limits.append((min(min_val, max_val), max(min_val, max_val)))

    x_min, x_max = normalized_limits[0]
    y_min, y_max = normalized_limits[1]
    z_min, z_max = normalized_limits[2]

    xy_world_points = []
    for x in (x_min, x_max):
        for y in (y_min, y_max):
            xy_world_points.append(
                (
                    tx + x * cos_rz - y * sin_rz,
                    ty + x * sin_rz + y * cos_rz,
                )
            )

    world_x = [point[0] for point in xy_world_points]
    world_y = [point[1] for point in xy_world_points]
    return [
        (float(min(world_x)), float(max(world_x))),
        (float(min(world_y)), float(max(world_y))),
        (float(tz + z_min), float(tz + z_max)),
    ]


def cubique_transition(t: float) -> float:
    """
    Description:
        Smooth time using f(t) = -2 * t^3 + 3 * t^2.
    Argument:
        t, in [0;1].
    Return:
        smoothed t, in [0;1].
    """
    # Clamp
    if t < 0:
        t = 0
    elif t > 1:
        t = 1
    t2 = t * t
    return -2 * t2 * t + 3 * t2

def cubic_transition(t: float) -> float:
    """
    Description:
        Smooth time using f(t) = -2t^3 + 3t^2.
    Argument:
        t, in [0;1].
    Return:
        smoothed t, in [0;1].
    """
    # Clamp
    if t < 0:
        t = 0
    elif t > 1:
        t = 1
    t2 = t * t
    return -2 * t2 * t + 3 * t2

def quintic_transition(t: float) -> float:
    """
    Description:
        Smooth time using f(t) = 6t^5 - 15t^4 + 10t^3.
    Argument:
        t, in [0;1].
    Return:
        smoothed t, in [0;1].
    """
    # Clamp
    if t < 0:
        t = 0
    elif t > 1:
        t = 1
    t2 = t * t
    t3 = t2 * t
    return 6 * t3*t2 - 15 * t2*t2 + 10 * t3

def pair_cubic_quintic_transition(t: float) -> tuple[float, float]:
    """
    Description:
        Smooth time using cubic_transition and quintic_transition.
    Argument:
        t, in [0;1].
    Return:
        cubic smoothed t in [0;1] and quintic smoothed t in [0;1].
    """
    if t < 0:
        t = 0
    elif t > 1:
        t = 1
    
    t2 = t * t
    t3 = t2 * t

    return -2 * t2 * t + 3 * t2, 6 * t3*t2 - 15 * t2*t2 + 10 * t3
