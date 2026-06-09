"""Parser APT/CLDATA → RobotProgram.

Zéro dépendance Qt. Frontière unique : seul ce module lit le format APT.
"""
from __future__ import annotations

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
from models.types import JointAngles6, Pose6, XYZ3
from utils.math_utils import orientation_from_tool_axis

# Commandes APT reconnues
_GOTO_RE = re.compile(
    r"^\s*GOTO\s*/\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)"
    r"\s*,\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)"
    r"\s*,\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)"
    r"(?:\s*,\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)"
    r"\s*,\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)"
    r"\s*,\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?))?",
    re.IGNORECASE,
)
_FROM_RE = re.compile(
    r"^\s*FROM\s*/\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)"
    r"\s*,\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)"
    r"\s*,\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)",
    re.IGNORECASE,
)
_RAPID_RE = re.compile(r"^\s*(RAPID|RAPIDTO)\b", re.IGNORECASE)
_FEDRAT_RE = re.compile(
    r"^\s*FEDRAT\s*/\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)",
    re.IGNORECASE,
)
_GOHOME_RE = re.compile(r"^\s*GOHOME\b", re.IGNORECASE)
_IGNORED_RE = re.compile(r"^\s*(SPINDL|CUTCOM|MULTAX|PPRINT|COOLNT|CUTTER|TLAXIS|PARTNO|FINI)\b", re.IGNORECASE)

_MM_PER_MIN_TO_MPS = 1.0 / 60_000.0
_DEFAULT_SPEED_MPS = 0.2


def _strip_apt_comment(line: str) -> str:
    """Supprime les commentaires $$ ou ;"""
    line = line.split("$$", 1)[0]
    line = line.split(";", 1)[0]
    return line.strip()


def load_aptsource_program(path: str | Path) -> RobotProgram:
    """Parse un fichier APT/CLDATA et retourne un RobotProgram."""
    program_path = Path(path)
    source_text = program_path.read_text(encoding="utf-8", errors="replace")
    motions: list[RobotProgramMotion] = []
    warnings: list[str] = []

    active_mode = RobotProgramMotionMode.LINEAR
    active_speed_mps: float = _DEFAULT_SPEED_MPS
    default_orientation = Pose6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    for line_number, raw_line in enumerate(source_text.splitlines(), start=1):
        line = _strip_apt_comment(raw_line)
        if not line:
            continue

        if _IGNORED_RE.match(line):
            keyword = line.split()[0].upper() if line.split() else ""
            if keyword in {"CUTCOM", "MULTAX"}:
                warnings.append(f"Ligne {line_number}: {keyword} ignore (non supporte).")
            continue

        if _GOHOME_RE.match(line):
            # GOHOME → fin de programme, on ignore (HOME sera géré par les settings)
            continue

        if _RAPID_RE.match(line):
            active_mode = RobotProgramMotionMode.PTP
            continue

        fedrat_match = _FEDRAT_RE.match(line)
        if fedrat_match:
            feed_mm_min = float(fedrat_match.group(1))
            active_speed_mps = feed_mm_min * _MM_PER_MIN_TO_MPS
            # Après un FEDRAT, on revient au mode linéaire
            active_mode = RobotProgramMotionMode.LINEAR
            continue

        from_match = _FROM_RE.match(line)
        if from_match:
            # FROM : position initiale, on crée un PTP
            x, y, z = float(from_match.group(1)), float(from_match.group(2)), float(from_match.group(3))
            target = RobotProgramTarget(
                target_type=RobotProgramTargetType.CARTESIAN,
                cartesian_pose=Pose6(x, y, z, default_orientation.a, default_orientation.b, default_orientation.c),
            )
            motions.append(RobotProgramMotion(
                mode=RobotProgramMotionMode.PTP,
                target=target,
                line_number=line_number,
                source=raw_line.rstrip("\r\n"),
                cp_speed_mps=active_speed_mps,
            ))
            continue

        goto_match = _GOTO_RE.match(line)
        if goto_match:
            x = float(goto_match.group(1))
            y = float(goto_match.group(2))
            z = float(goto_match.group(3))

            if goto_match.group(4) is not None:
                # i, j, k présents → orientation depuis vecteur axe-outil
                i = float(goto_match.group(4))
                j = float(goto_match.group(5))
                k = float(goto_match.group(6))
                orientation = orientation_from_tool_axis(XYZ3(i, j, k))
            else:
                orientation = default_orientation

            target = RobotProgramTarget(
                target_type=RobotProgramTargetType.CARTESIAN,
                cartesian_pose=Pose6(x, y, z, orientation.a, orientation.b, orientation.c),
            )
            motions.append(RobotProgramMotion(
                mode=active_mode,
                target=target,
                line_number=line_number,
                source=raw_line.rstrip("\r\n"),
                cp_speed_mps=active_speed_mps,
            ))
            # Après un GOTO, on repasse en linéaire (RAPID est one-shot)
            active_mode = RobotProgramMotionMode.LINEAR
            continue

    return RobotProgram(
        brand=RobotProgramBrand.KUKA,
        source_path=str(program_path),
        source_text=source_text,
        motions=motions,
        warnings=warnings,
        origin=ProgramOrigin.IMPORTED_APT,
    )
