from __future__ import annotations

from collections.abc import Iterable, Mapping, Iterator


class XYZ3:
    __slots__ = ("x", "y", "z")

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    @staticmethod
    def _safe_float(value: object, default: float = 0.0) -> float:
        try:
            if value is None:
                return default
            if isinstance(value, str):
                stripped = value.strip()
                if stripped == "":
                    return default
                return float(stripped)
            return float(value)
        except (TypeError, ValueError):
            return default

    @classmethod
    def zeros(cls) -> "XYZ3":
        return cls()

    @classmethod
    def from_values(cls, x: object = 0.0, y: object = 0.0, z: object = 0.0) -> "XYZ3":
        return cls(
            cls._safe_float(x, 0.0),
            cls._safe_float(y, 0.0),
            cls._safe_float(z, 0.0),
        )

    @classmethod
    def from_sequence(cls, values: object, fill_missing: bool = False) -> "XYZ3":
        if isinstance(values, XYZ3):
            return values.copy()
        if not isinstance(values, Iterable) or isinstance(values, (str, bytes, Mapping)):
            raise TypeError("XYZ3 sequence must be an iterable with 3 values")

        seq = list(values)
        if len(seq) < 3 and not fill_missing:
            raise ValueError("XYZ3 sequence must contain at least 3 values")
        while len(seq) < 3:
            seq.append(0.0)
        return cls.from_values(*seq[:3])

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "XYZ3":
        return cls.from_values(
            values.get("x", 0.0),
            values.get("y", 0.0),
            values.get("z", 0.0),
        )

    @classmethod
    def from_any(cls, values: object, fill_missing: bool = True) -> "XYZ3":
        if isinstance(values, XYZ3):
            return values.copy()
        if isinstance(values, Mapping):
            return cls.from_mapping(values)
        if isinstance(values, Iterable) and not isinstance(values, (str, bytes)):
            return cls.from_sequence(values, fill_missing=fill_missing)
        if fill_missing:
            return cls.zeros()
        raise TypeError("Unsupported XYZ3 value")

    def copy(self) -> "XYZ3":
        return XYZ3(self.x, self.y, self.z)

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    def to_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def __len__(self) -> int:
        return 3

    def __iter__(self) -> Iterator[float]:
        yield self.x
        yield self.y
        yield self.z

    def __getitem__(self, index: int | slice) -> float | list[float]:
        values = self.to_list()
        return values[index]

    def __setitem__(self, index: int, value: object) -> None:
        normalized = self._safe_float(value, 0.0)
        if index == 0:
            self.x = normalized
            return
        if index == 1:
            self.y = normalized
            return
        if index == 2:
            self.z = normalized
            return
        raise IndexError("XYZ3 index out of range")

    def __eq__(self, other: object) -> bool:
        if isinstance(other, XYZ3):
            return self.to_tuple() == other.to_tuple()
        if isinstance(other, Mapping):
            try:
                return self == XYZ3.from_mapping(other)
            except (TypeError, ValueError):
                return False
        if isinstance(other, Iterable) and not isinstance(other, (str, bytes)):
            try:
                return self == XYZ3.from_sequence(other, fill_missing=False)
            except (TypeError, ValueError):
                return False
        return False

    def __repr__(self) -> str:
        return f"XYZ3(x={self.x}, y={self.y}, z={self.z})"
