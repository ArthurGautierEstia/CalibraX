from __future__ import annotations

from pathlib import Path

from controllers.calibration_view.measurement_controller import MeasurementController
from controllers.robot_view.robot_configuration_controller import RobotConfigurationController
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from utils.trajectory_paths import get_trajectories_directory


def _resolve_project_directory(directory_path: str, root_dir: Path) -> Path:
    path = Path(directory_path)
    if path.is_absolute():
        return path.resolve()
    return (root_dir / path).resolve()


def ensure_user_data_directories(root_dir: Path | None = None) -> None:
    root = Path.cwd() if root_dir is None else Path(root_dir)
    directories = [
        _resolve_project_directory(RobotConfigurationController.DEFAULT_ROBOT_CONFIG_DIRECTORY, root),
        _resolve_project_directory(ToolModel.DEFAULT_TOOL_PROFILES_DIRECTORY, root),
        _resolve_project_directory(WorkspaceModel.DEFAULT_WORKSPACE_DIRECTORY, root),
        _resolve_project_directory(MeasurementController.DEFAULT_MEASUREMENTS_DIRECTORY, root),
        get_trajectories_directory(create=True, root_dir=root),
    ]

    for directory in directories:
        directory.mkdir(parents=True, exist_ok=True)
