from __future__ import annotations

from dataclasses import dataclass, field
import json
from typing import Any


PROJECT_FORMAT = "calibrax_project"
PROJECT_VERSION = 1


@dataclass
class ProjectFile:
    name: str = ""
    configurations: dict[str, str] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ProjectFile":
        if not isinstance(data, dict):
            raise TypeError("Le projet doit etre un objet JSON.")
        if str(data.get("format", "")) != PROJECT_FORMAT:
            raise ValueError("Le fichier n'est pas un projet CalibraX.")
        version = int(data.get("version", 0))
        if version != PROJECT_VERSION:
            raise ValueError(f"Version de projet non supportee: {version}.")

        raw_configurations = data.get("configurations") or {}
        if not isinstance(raw_configurations, dict):
            raise TypeError("Le champ configurations doit etre un objet JSON.")
        configurations = {
            str(key): "" if value is None else str(value)
            for key, value in raw_configurations.items()
        }
        return cls(
            name="" if data.get("name") is None else str(data.get("name")),
            configurations=configurations,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "format": PROJECT_FORMAT,
            "version": PROJECT_VERSION,
            "name": self.name,
            "configurations": dict(self.configurations),
        }

    def save(self, file_path: str) -> None:
        with open(file_path, "w", encoding="utf-8") as file:
            json.dump(self.to_dict(), file, indent=4, ensure_ascii=False)

    @classmethod
    def load(cls, file_path: str) -> "ProjectFile":
        with open(file_path, "r", encoding="utf-8") as file:
            data = json.load(file)
        return cls.from_dict(data)
