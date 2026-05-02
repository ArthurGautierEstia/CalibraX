from pathlib import Path


DEFAULT_TRAJECTORIES_DIRECTORY = Path("user_data") / "trajectories"


def get_trajectories_directory(create: bool = False, root_dir: Path | None = None) -> Path:
    root = Path.cwd() if root_dir is None else Path(root_dir)
    directory = (root / DEFAULT_TRAJECTORIES_DIRECTORY).resolve()
    if create:
        directory.mkdir(parents=True, exist_ok=True)
    if directory.exists():
        return directory
    return root.resolve()
