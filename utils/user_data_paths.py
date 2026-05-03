from __future__ import annotations

import ctypes
import os
from pathlib import Path

from controllers.calibration_view.measurement_controller import MeasurementController
from controllers.robot_view.robot_configuration_controller import RobotConfigurationController
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from utils.trajectory_paths import get_trajectories_directory


DEFAULT_DATA_DIRECTORY = Path("default_data")


class _WindowsGuid(ctypes.Structure):
    _fields_ = [
        ("data1", ctypes.c_ulong),
        ("data2", ctypes.c_ushort),
        ("data3", ctypes.c_ushort),
        ("data4", ctypes.c_ubyte * 8),
    ]


def _windows_guid(
    data1: int,
    data2: int,
    data3: int,
    data4: tuple[int, int, int, int, int, int, int, int],
) -> _WindowsGuid:
    return _WindowsGuid(data1, data2, data3, (ctypes.c_ubyte * 8)(*data4))


def _resolve_project_directory(directory_path: str, root_dir: Path) -> Path:
    path = Path(directory_path)
    if path.is_absolute():
        return path.resolve()
    return (root_dir / path).resolve()


def _user_data_directories(root: Path) -> list[Path]:
    return [
        _resolve_project_directory(RobotConfigurationController.DEFAULT_ROBOT_CONFIG_DIRECTORY, root),
        _resolve_project_directory(ToolModel.DEFAULT_TOOL_PROFILES_DIRECTORY, root),
        _resolve_project_directory(WorkspaceModel.DEFAULT_WORKSPACE_DIRECTORY, root),
        _resolve_project_directory(MeasurementController.DEFAULT_MEASUREMENTS_DIRECTORY, root),
        get_trajectories_directory(create=True, root_dir=root),
    ]


def _default_data_directory(root: Path, user_directory: Path) -> Path:
    return (root / DEFAULT_DATA_DIRECTORY / user_directory.name).resolve()


def _default_shortcut_name(default_directory: Path) -> str:
    folder_name = default_directory.name
    if folder_name.endswith("ies"):
        shortcut_base = f"{folder_name[:-3]}y"
    elif folder_name.endswith("s"):
        shortcut_base = folder_name[:-1]
    else:
        shortcut_base = folder_name
    return f"default_{shortcut_base}.lnk"


def _check_hresult(result: int, action: str) -> None:
    if result < 0:
        raise OSError(ctypes.c_long(result).value, action)


def _call_com_method(pointer: ctypes.c_void_p, index: int, result_type: object, *argument_types: object) -> object:
    method_address = ctypes.cast(pointer, ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p))).contents[index]
    method_type = ctypes.WINFUNCTYPE(result_type, ctypes.c_void_p, *argument_types)
    return method_type(method_address)


def _release_com_pointer(pointer: ctypes.c_void_p) -> None:
    if pointer:
        release = _call_com_method(pointer, 2, ctypes.c_ulong)
        release(pointer)


def _create_windows_shortcut(shortcut_path: Path, target_path: Path) -> None:
    if os.name != "nt":
        return

    ole32 = ctypes.WinDLL("ole32")
    shell_link_class_id = _windows_guid(
        0x00021401,
        0x0000,
        0x0000,
        (0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46),
    )
    shell_link_interface_id = _windows_guid(
        0x000214F9,
        0x0000,
        0x0000,
        (0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46),
    )
    persist_file_interface_id = _windows_guid(
        0x0000010B,
        0x0000,
        0x0000,
        (0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46),
    )

    coinitialize_result = ole32.CoInitialize(None)
    should_uninitialize = coinitialize_result >= 0
    shell_link_pointer = ctypes.c_void_p()
    persist_file_pointer = ctypes.c_void_p()

    try:
        ole32.CoCreateInstance.argtypes = [
            ctypes.POINTER(_WindowsGuid),
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(_WindowsGuid),
            ctypes.POINTER(ctypes.c_void_p),
        ]
        ole32.CoCreateInstance.restype = ctypes.c_long

        _check_hresult(
            ole32.CoCreateInstance(
                ctypes.byref(shell_link_class_id),
                None,
                1,
                ctypes.byref(shell_link_interface_id),
                ctypes.byref(shell_link_pointer),
            ),
            "Impossible de creer l'objet ShellLink",
        )

        set_path = _call_com_method(shell_link_pointer, 20, ctypes.c_long, ctypes.c_wchar_p)
        _check_hresult(set_path(shell_link_pointer, str(target_path)), "Impossible de definir la cible du raccourci")

        set_working_directory = _call_com_method(shell_link_pointer, 9, ctypes.c_long, ctypes.c_wchar_p)
        _check_hresult(
            set_working_directory(shell_link_pointer, str(target_path)),
            "Impossible de definir le dossier de travail du raccourci",
        )

        query_interface = _call_com_method(
            shell_link_pointer,
            0,
            ctypes.c_long,
            ctypes.POINTER(_WindowsGuid),
            ctypes.POINTER(ctypes.c_void_p),
        )
        _check_hresult(
            query_interface(
                shell_link_pointer,
                ctypes.byref(persist_file_interface_id),
                ctypes.byref(persist_file_pointer),
            ),
            "Impossible d'acceder a IPersistFile",
        )

        save = _call_com_method(persist_file_pointer, 6, ctypes.c_long, ctypes.c_wchar_p, ctypes.c_int)
        _check_hresult(save(persist_file_pointer, str(shortcut_path), 1), "Impossible d'enregistrer le raccourci")
    finally:
        _release_com_pointer(persist_file_pointer)
        _release_com_pointer(shell_link_pointer)
        if should_uninitialize:
            ole32.CoUninitialize()


def _ensure_default_data_shortcuts(root: Path, directories: list[Path]) -> None:
    for user_directory in directories:
        default_directory = _default_data_directory(root, user_directory)
        if not default_directory.is_dir():
            continue
        shortcut_path = user_directory / _default_shortcut_name(default_directory)
        try:
            _create_windows_shortcut(shortcut_path, default_directory)
        except OSError as exc:
            print(f"Impossible de creer le raccourci {shortcut_path}: {exc}")


def ensure_user_data_directories(root_dir: Path | None = None) -> None:
    root = Path.cwd() if root_dir is None else Path(root_dir)
    directories = _user_data_directories(root)

    for directory in directories:
        directory.mkdir(parents=True, exist_ok=True)

    _ensure_default_data_shortcuts(root, directories)
