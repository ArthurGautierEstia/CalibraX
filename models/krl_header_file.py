from __future__ import annotations

from pathlib import Path

_DEFAULT_HEADER_PATH = Path(__file__).resolve().parents[1] / "default_data" / "programs" / "krl_header_template.src"
_USER_HEADER_PATH = Path(__file__).resolve().parents[1] / "user_data" / "programs" / "krl_header_template.src"


def load_header_template() -> str:
    """Charge le template header KRL : override utilisateur sinon défaut distribué."""
    if _USER_HEADER_PATH.exists():
        return _USER_HEADER_PATH.read_text(encoding="utf-8")
    if _DEFAULT_HEADER_PATH.exists():
        return _DEFAULT_HEADER_PATH.read_text(encoding="utf-8")
    return ""


def save_header_template(text: str) -> None:
    """Sauvegarde le template header dans l'override utilisateur."""
    _USER_HEADER_PATH.parent.mkdir(parents=True, exist_ok=True)
    _USER_HEADER_PATH.write_text(text, encoding="utf-8")


def reset_header_template() -> None:
    """Supprime l'override utilisateur pour revenir au défaut distribué."""
    if _USER_HEADER_PATH.exists():
        _USER_HEADER_PATH.unlink()
