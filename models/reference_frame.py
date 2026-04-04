from __future__ import annotations

from enum import Enum


class ReferenceFrame(str, Enum):
    BASE = "BASE"
    WORLD = "WORLD"

    @classmethod
    def from_value(cls, value: object, default: "ReferenceFrame" = None) -> "ReferenceFrame":
        fallback = cls.BASE if default is None else default
        if isinstance(value, cls):
            return value
        try:
            normalized = str(value).strip().upper()
        except Exception:
            return fallback
        try:
            return cls(normalized)
        except ValueError:
            return fallback
