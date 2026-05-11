from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass

from models.app_session_file import ViewerThemeState


@dataclass(frozen=True)
class StoredViewerTheme:
    name: str
    file_stem: str
    state: ViewerThemeState


class ViewerThemeStore:
    THEMES_DIRECTORY_RELATIVE_PATH = os.path.join("user_data", "viewer_themes")
    DEFAULT_THEME_SETTINGS_FILE_NAME = "_default_theme.json"

    def __init__(self, project_root: str) -> None:
        self._project_root = os.path.abspath(project_root)
        self._themes_directory_path = os.path.join(self._project_root, self.THEMES_DIRECTORY_RELATIVE_PATH)

    def ensure_storage_directory(self) -> None:
        os.makedirs(self._themes_directory_path, exist_ok=True)

    def list_themes(self) -> list[StoredViewerTheme]:
        self.ensure_storage_directory()
        themes: list[StoredViewerTheme] = []
        for file_name in sorted(os.listdir(self._themes_directory_path)):
            if not file_name.lower().endswith(".json"):
                continue
            if file_name == self.DEFAULT_THEME_SETTINGS_FILE_NAME:
                continue
            file_path = os.path.join(self._themes_directory_path, file_name)
            theme = self._load_theme_file(file_path)
            if theme is not None:
                themes.append(theme)
        return themes

    def load_theme(self, theme_name: str) -> ViewerThemeState | None:
        normalized_theme_name = str(theme_name or "").strip()
        if normalized_theme_name == "":
            return None
        for stored_theme in self.list_themes():
            if stored_theme.name == normalized_theme_name:
                return stored_theme.state
        return None

    def save_theme(self, theme_name: str, theme_state: ViewerThemeState) -> str:
        normalized_theme_name = str(theme_name or "").strip()
        if normalized_theme_name == "":
            raise ValueError("Le nom du theme viewer ne peut pas etre vide.")

        file_stem = self._build_file_stem(normalized_theme_name)
        file_path = os.path.join(self._themes_directory_path, f"{file_stem}.json")
        payload = {
            "name": normalized_theme_name,
            "theme": theme_state.to_dict(),
        }

        self.ensure_storage_directory()
        with open(file_path, "w", encoding="utf-8") as file:
            json.dump(payload, file, indent=4)
        return normalized_theme_name

    def load_default_theme_name(self) -> str:
        self.ensure_storage_directory()
        settings_file_path = os.path.join(self._themes_directory_path, self.DEFAULT_THEME_SETTINGS_FILE_NAME)
        if not os.path.exists(settings_file_path):
            return ""
        try:
            with open(settings_file_path, "r", encoding="utf-8") as file:
                payload = json.load(file)
        except (OSError, ValueError, TypeError):
            return ""

        if not isinstance(payload, dict):
            return ""
        theme_name = payload.get("theme_name")
        return "" if theme_name is None else str(theme_name).strip()

    def save_default_theme_name(self, theme_name: str) -> None:
        normalized_theme_name = str(theme_name or "").strip()
        if normalized_theme_name == "":
            raise ValueError("Le theme viewer par defaut ne peut pas etre vide.")

        self.ensure_storage_directory()
        settings_file_path = os.path.join(self._themes_directory_path, self.DEFAULT_THEME_SETTINGS_FILE_NAME)
        with open(settings_file_path, "w", encoding="utf-8") as file:
            json.dump({"theme_name": normalized_theme_name}, file, indent=4)

    def delete_theme(self, theme_name: str) -> bool:
        normalized_theme_name = str(theme_name or "").strip()
        if normalized_theme_name == "":
            return False
        for stored_theme in self.list_themes():
            if stored_theme.name != normalized_theme_name:
                continue
            file_path = os.path.join(self._themes_directory_path, f"{stored_theme.file_stem}.json")
            try:
                os.remove(file_path)
            except OSError:
                return False
            if self.load_default_theme_name() == normalized_theme_name:
                default_settings_path = os.path.join(self._themes_directory_path, self.DEFAULT_THEME_SETTINGS_FILE_NAME)
                try:
                    os.remove(default_settings_path)
                except OSError:
                    pass
            return True
        return False

    @staticmethod
    def _build_file_stem(theme_name: str) -> str:
        normalized = re.sub(r"[^A-Za-z0-9_-]+", "_", theme_name.strip())
        collapsed = re.sub(r"_+", "_", normalized).strip("_")
        return collapsed if collapsed != "" else "theme_viewer"

    def _load_theme_file(self, file_path: str) -> StoredViewerTheme | None:
        try:
            with open(file_path, "r", encoding="utf-8") as file:
                payload = json.load(file)
        except (OSError, ValueError, TypeError):
            return None

        if not isinstance(payload, dict):
            return None
        theme_payload = payload.get("theme")
        if not isinstance(theme_payload, dict):
            return None

        file_stem = os.path.splitext(os.path.basename(file_path))[0]
        theme_name = payload.get("name")
        normalized_theme_name = file_stem if theme_name is None else str(theme_name).strip()
        if normalized_theme_name == "":
            normalized_theme_name = file_stem

        return StoredViewerTheme(
            name=normalized_theme_name,
            file_stem=file_stem,
            state=ViewerThemeState.from_dict(theme_payload),
        )
