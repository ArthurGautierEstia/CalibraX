from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import re

from models.robot_program import (
    RobotProgram,
    RobotProgramBrand,
    RobotProgramMotion,
    RobotProgramMotionMode,
    RobotProgramTarget,
    RobotProgramTargetType,
)
from models.types import JointAngles6, Pose6


_MOTION_RE = re.compile(r"^\s*(PTP|LIN|CIRC|JOINT)\b(.*)$", re.IGNORECASE)
_TOOL_RE = re.compile(r"^\s*\$TOOL\b", re.IGNORECASE)
_BASE_RE = re.compile(r"^\s*\$BASE\b", re.IGNORECASE)
_VEL_CP_RE = re.compile(
    r"^\s*\$VEL\s*\.\s*CP\s*=\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][-+]?\d+)?)",
    re.IGNORECASE,
)
_BRACE_RE = re.compile(r"\{([^{}]+)\}")
_FIELD_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)\s+([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][-+]?\d+)?)")


def load_kuka_src_program(path: str | Path) -> RobotProgram:
    program_path = Path(path)
    source_text = program_path.read_text(encoding="utf-8", errors="replace")
    motions: list[RobotProgramMotion] = []
    warnings: list[str] = []
    active_base = Pose6.zeros()
    active_tool = Pose6.zeros()
    active_cp_speed_mps: float | None = None
    first_explicit_tool_pose: Pose6 | None = None
    first_explicit_tool_line: int | None = None

    for line_number, raw_line in enumerate(source_text.splitlines(), start=1):
        line = _strip_comment(raw_line)
        if not line:
            continue

        velocity_match = _VEL_CP_RE.match(line)
        if velocity_match:
            active_cp_speed_mps = float(velocity_match.group(1))
            continue

        if _TOOL_RE.match(line):
            blocks = _BRACE_RE.findall(line)
            if not blocks:
                warnings.append(f"Ligne {line_number}: $TOOL sans pose inline supportee.")
                continue
            parsed_tool = _parse_pose_block(blocks[0])
            if parsed_tool is None:
                warnings.append(f"Ligne {line_number}: $TOOL invalide.")
                continue
            active_tool = parsed_tool
            if first_explicit_tool_pose is None:
                first_explicit_tool_pose = parsed_tool.copy()
                first_explicit_tool_line = line_number
            continue

        if _BASE_RE.match(line):
            blocks = _BRACE_RE.findall(line)
            if not blocks:
                warnings.append(f"Ligne {line_number}: $BASE sans pose inline supportee.")
                continue
            parsed_base = _parse_pose_block(blocks[0])
            if parsed_base is None:
                warnings.append(f"Ligne {line_number}: $BASE invalide.")
                continue
            active_base = parsed_base
            continue

        motion_match = _MOTION_RE.match(line)
        if motion_match is None:
            continue

        raw_mode = motion_match.group(1).upper()
        mode = RobotProgramMotionMode.PTP if raw_mode in {"PTP", "JOINT"} else (
            RobotProgramMotionMode.LINEAR if raw_mode == "LIN" else RobotProgramMotionMode.CIRCULAR
        )
        blocks = _BRACE_RE.findall(motion_match.group(2))
        if not blocks:
            warnings.append(f"Ligne {line_number}: cible inline absente ou non supportee.")
            continue

        if mode == RobotProgramMotionMode.CIRCULAR:
            if len(blocks) < 2:
                warnings.append(f"Ligne {line_number}: CIRC attend un point intermediaire et un point final.")
                continue
            via_target = _parse_target_block(blocks[0])
            end_target = _parse_target_block(blocks[1])
            if via_target is None or end_target is None:
                warnings.append(f"Ligne {line_number}: CIRC contient une cible invalide.")
                continue
            if via_target.target_type != RobotProgramTargetType.CARTESIAN or end_target.target_type != RobotProgramTargetType.CARTESIAN:
                warnings.append(f"Ligne {line_number}: CIRC supporte uniquement des cibles cartesiennes.")
                continue
            motions.append(
                RobotProgramMotion(
                    mode=mode,
                    target=end_target,
                    via_target=via_target,
                    line_number=line_number,
                    source=raw_line.rstrip("\r\n"),
                    base_pose=active_base.copy(),
                    tool_pose=active_tool.copy(),
                    cp_speed_mps=active_cp_speed_mps,
                )
            )
            continue

        target = _parse_target_block(blocks[0])
        if target is None:
            warnings.append(f"Ligne {line_number}: cible invalide ou incomplete.")
            continue
        motions.append(
            RobotProgramMotion(
                mode=mode,
                target=target,
                line_number=line_number,
                source=raw_line.rstrip("\r\n"),
                base_pose=active_base.copy(),
                tool_pose=active_tool.copy(),
                cp_speed_mps=active_cp_speed_mps,
            )
        )

    if first_explicit_tool_pose is not None and first_explicit_tool_line is not None:
        backfilled_motions: list[RobotProgramMotion] = []
        for motion in motions:
            if motion.line_number < first_explicit_tool_line and motion.tool_pose == Pose6.zeros():
                backfilled_motions.append(replace(motion, tool_pose=first_explicit_tool_pose.copy()))
                continue
            backfilled_motions.append(motion)
        motions = backfilled_motions

    return RobotProgram(
        brand=RobotProgramBrand.KUKA,
        source_path=str(program_path),
        source_text=source_text,
        program_base_pose=active_base.copy(),
        motions=motions,
        warnings=warnings,
    )


def export_kuka_src_program(
    path: str | Path,
    source_text: str,
    motions: list[RobotProgramMotion],
    program_base_pose: Pose6 | None = None,
) -> None:
    motions_by_line: dict[int, list[RobotProgramMotion]] = {}
    for motion in motions:
        motions_by_line.setdefault(int(motion.line_number), []).append(motion)

    lines = source_text.splitlines(keepends=True)
    base_line_indexes: list[int] = []
    for line_number, line in enumerate(lines, start=1):
        if _BASE_RE.match(_strip_comment(line)):
            base_line_indexes.append(line_number - 1)
        line_motions = motions_by_line.get(line_number)
        if not line_motions:
            continue
        indent_match = re.match(r"^\s*", line)
        indent = indent_match.group(0) if indent_match is not None else ""
        line_ending = "\r\n" if line.endswith("\r\n") else "\n"
        lines[line_number - 1] = "".join(
            f"{indent}{_format_kuka_motion_line(motion)}{line_ending}"
            for motion in line_motions
        )

    if program_base_pose is not None:
        formatted_base_line = _format_kuka_base_line(program_base_pose)
        if base_line_indexes:
            for line_index in base_line_indexes:
                source_line = lines[line_index]
                indent_match = re.match(r"^\s*", source_line)
                indent = indent_match.group(0) if indent_match is not None else ""
                line_ending = "\r\n" if source_line.endswith("\r\n") else "\n"
                lines[line_index] = f"{indent}{formatted_base_line}{line_ending}"
        else:
            insert_index = 0
            for index, line in enumerate(lines):
                if _MOTION_RE.match(_strip_comment(line)):
                    insert_index = index
                    break
            line_ending = "\r\n" if any(line.endswith("\r\n") for line in lines) else "\n"
            lines.insert(insert_index, f"{formatted_base_line}{line_ending}")

    Path(path).write_text("".join(lines), encoding="utf-8")


def _strip_comment(line: str) -> str:
    return line.split(";", 1)[0].strip()


def _parse_target_block(block: str) -> RobotProgramTarget | None:
    fields = {name.upper(): float(value) for name, value in _FIELD_RE.findall(block)}
    joint_keys = [f"A{index}" for index in range(1, 7)]
    if all(key in fields for key in joint_keys):
        return RobotProgramTarget(
            target_type=RobotProgramTargetType.JOINT,
            joint_angles=JointAngles6.from_values([fields[key] for key in joint_keys]),
        )

    cartesian_keys = ["X", "Y", "Z", "A", "B", "C"]
    if all(key in fields for key in cartesian_keys[:3]):
        return RobotProgramTarget(
            target_type=RobotProgramTargetType.CARTESIAN,
            cartesian_pose=Pose6.from_values([fields.get(key, 0.0) for key in cartesian_keys]),
        )
    return None


def _parse_pose_block(block: str) -> Pose6 | None:
    target = _parse_target_block(block)
    if target is None or target.target_type != RobotProgramTargetType.CARTESIAN:
        return None
    return target.cartesian_pose.copy()


def _format_kuka_motion_line(motion: RobotProgramMotion) -> str:
    if motion.mode == RobotProgramMotionMode.PTP:
        return f"PTP {_format_kuka_target(motion.target)}"
    if motion.mode == RobotProgramMotionMode.LINEAR:
        return f"LIN {_format_kuka_target(motion.target)}"
    if motion.mode == RobotProgramMotionMode.CIRCULAR and motion.via_target is not None:
        return f"CIRC {_format_kuka_target(motion.via_target)}, {_format_kuka_target(motion.target)}"
    return motion.source


def _format_kuka_target(target: RobotProgramTarget) -> str:
    if target.target_type == RobotProgramTargetType.JOINT:
        values = target.joint_angles.to_list()
        return (
            "{A1 "
            f"{values[0]:.3f},A2 {values[1]:.3f},A3 {values[2]:.3f},"
            f"A4 {values[3]:.3f},A5 {values[4]:.3f},A6 {values[5]:.3f}"
            "}"
        )
    pose = target.cartesian_pose.to_list()
    return (
        "{X "
        f"{pose[0]:.3f},Y {pose[1]:.3f},Z {pose[2]:.3f},"
        f"A {pose[3]:.3f},B {pose[4]:.3f},C {pose[5]:.3f}"
        "}"
    )


def _format_kuka_base_line(base_pose: Pose6) -> str:
    return f"$BASE = {_format_kuka_pose(base_pose)}"


def _format_kuka_pose(pose: Pose6) -> str:
    values = pose.to_list()
    return (
        "{X "
        f"{values[0]:.3f},Y {values[1]:.3f},Z {values[2]:.3f},"
        f"A {values[3]:.3f},B {values[4]:.3f},C {values[5]:.3f}"
        "}"
    )
