"""Parser CATNCcode (ISO G-code CATIA) → RobotProgram.

Zéro dépendance Qt. Frontière unique : seul ce module lit le format CATNCcode.
"""
from __future__ import annotations

import math
import re
from pathlib import Path

from models.robot_program import (
    ProgramOrigin,
    RobotProgram,
    RobotProgramBrand,
    RobotProgramMotion,
    RobotProgramMotionMode,
    RobotProgramTarget,
    RobotProgramTargetType,
)
from models.types import Pose6

_WORD_RE = re.compile(r"([A-Za-z])([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)")
_COMMENT_RE = re.compile(r"\(.*?\)")

_MM_PER_MIN_TO_MPS = 1.0 / 60_000.0
_DEFAULT_SPEED_MPS = 0.2


def _strip_nc_comment(line: str) -> str:
    line = _COMMENT_RE.sub("", line)
    line = line.split(";", 1)[0]
    return line.strip()


def _parse_words(line: str) -> dict[str, float]:
    """Retourne les mots G-code. Pour G, garde le PREMIER code de mouvement (0-3) si présent,
    sinon le premier G rencontré (pour compatibilité). Évite l'écrasement par G94/G90 etc."""
    result: dict[str, float] = {}
    g_motion: float | None = None  # premier G code de mouvement (0,1,2,3)
    for m in _WORD_RE.finditer(line):
        letter = m.group(1).upper()
        value = float(m.group(2))
        if letter == "G":
            if g_motion is None and int(round(value)) in {0, 1, 2, 3}:
                g_motion = value
            # Ne pas écraser avec des G modaux (G90, G94…)
            if "G" not in result:
                result["G"] = value
        else:
            result[letter] = value
    if g_motion is not None:
        result["G"] = g_motion
    return result


def _arc_midpoint(
    x0: float, y0: float, x1: float, y1: float,
    cx: float, cy: float, clockwise: bool
) -> tuple[float, float]:
    """Calcule le point intermédiaire d'un arc G02/G03 dans le plan XY."""
    angle_start = math.atan2(y0 - cy, x0 - cx)
    angle_end = math.atan2(y1 - cy, x1 - cx)
    r = math.hypot(x0 - cx, y0 - cy)

    if clockwise:
        if angle_end > angle_start:
            angle_end -= 2 * math.pi
    else:
        if angle_end < angle_start:
            angle_end += 2 * math.pi

    angle_mid = (angle_start + angle_end) / 2.0
    return cx + r * math.cos(angle_mid), cy + r * math.sin(angle_mid)


def load_catnc_program(path: str | Path) -> RobotProgram:
    """Parse un fichier CATNCcode (G-code) et retourne un RobotProgram."""
    program_path = Path(path)
    source_text = program_path.read_text(encoding="utf-8", errors="replace")
    motions: list[RobotProgramMotion] = []
    warnings: list[str] = []

    # État modal
    g_mode: int = 0  # 0=rapide, 1=linéaire, 2=arc CW, 3=arc CCW
    current_x: float = 0.0
    current_y: float = 0.0
    current_z: float = 0.0
    active_speed_mps: float = _DEFAULT_SPEED_MPS
    default_abc = Pose6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    for line_number, raw_line in enumerate(source_text.splitlines(), start=1):
        line = _strip_nc_comment(raw_line)
        if not line:
            continue

        words = _parse_words(line)

        # Mise à jour vitesse
        if "F" in words:
            feed_mm_min = words["F"]
            active_speed_mps = feed_mm_min * _MM_PER_MIN_TO_MPS

        # Mise à jour mode G
        if "G" in words:
            g_code = int(round(words["G"]))
            if g_code in {0, 1, 2, 3}:
                g_mode = g_code
            elif g_code in {17, 18, 19, 20, 21, 40, 41, 42, 43, 44, 49, 54, 55, 90, 91}:
                pass  # codes modaux sans effet sur la trajectoire ici
            # S, M, T → ignorés

        # Pas de mouvement si pas de coordonnées cibles
        has_position = "X" in words or "Y" in words or "Z" in words
        if not has_position:
            continue

        target_x = words.get("X", current_x)
        target_y = words.get("Y", current_y)
        target_z = words.get("Z", current_z)

        if g_mode == 0:
            # G00 : PTP rapide
            target = RobotProgramTarget(
                target_type=RobotProgramTargetType.CARTESIAN,
                cartesian_pose=Pose6(target_x, target_y, target_z, default_abc.a, default_abc.b, default_abc.c),
            )
            motions.append(RobotProgramMotion(
                mode=RobotProgramMotionMode.PTP,
                target=target,
                line_number=line_number,
                source=raw_line.rstrip("\r\n"),
                cp_speed_mps=active_speed_mps,
            ))

        elif g_mode == 1:
            # G01 : linéaire
            target = RobotProgramTarget(
                target_type=RobotProgramTargetType.CARTESIAN,
                cartesian_pose=Pose6(target_x, target_y, target_z, default_abc.a, default_abc.b, default_abc.c),
            )
            motions.append(RobotProgramMotion(
                mode=RobotProgramMotionMode.LINEAR,
                target=target,
                line_number=line_number,
                source=raw_line.rstrip("\r\n"),
                cp_speed_mps=active_speed_mps,
            ))

        elif g_mode in {2, 3}:
            # G02/G03 : arc circulaire → CIRCULAR avec point intermédiaire calculé
            i_offset = words.get("I", 0.0)
            j_offset = words.get("J", 0.0)
            # K ignoré (on reste dans le plan XY)
            cx = current_x + i_offset
            cy = current_y + j_offset
            clockwise = g_mode == 2

            mid_x, mid_y = _arc_midpoint(current_x, current_y, target_x, target_y, cx, cy, clockwise)
            mid_z = (current_z + target_z) / 2.0

            via_target = RobotProgramTarget(
                target_type=RobotProgramTargetType.CARTESIAN,
                cartesian_pose=Pose6(mid_x, mid_y, mid_z, default_abc.a, default_abc.b, default_abc.c),
            )
            end_target = RobotProgramTarget(
                target_type=RobotProgramTargetType.CARTESIAN,
                cartesian_pose=Pose6(target_x, target_y, target_z, default_abc.a, default_abc.b, default_abc.c),
            )
            motions.append(RobotProgramMotion(
                mode=RobotProgramMotionMode.CIRCULAR,
                target=end_target,
                via_target=via_target,
                line_number=line_number,
                source=raw_line.rstrip("\r\n"),
                cp_speed_mps=active_speed_mps,
            ))

        else:
            warnings.append(f"Ligne {line_number}: mode G{g_mode} non supporte, ignore.")

        current_x = target_x
        current_y = target_y
        current_z = target_z

    return RobotProgram(
        brand=RobotProgramBrand.KUKA,
        source_path=str(program_path),
        source_text=source_text,
        motions=motions,
        warnings=warnings,
        origin=ProgramOrigin.IMPORTED_CATNC,
    )
