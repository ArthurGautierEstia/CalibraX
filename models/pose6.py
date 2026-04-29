from __future__ import annotations

from collections.abc import Iterable, Mapping, Iterator


class Pose6:
    __slots__ = ("x", "y", "z", "a", "b", "c")

    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        a: float = 0.0,
        b: float = 0.0,
        c: float = 0.0,
    ) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.a = float(a)
        self.b = float(b)
        self.c = float(c)

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
    def zeros(cls) -> "Pose6":
        return cls()

    @classmethod
    def from_values(
        cls,
        x: object = 0.0,
        y: object = 0.0,
        z: object = 0.0,
        a: object = 0.0,
        b: object = 0.0,
        c: object = 0.0,
    ) -> "Pose6":
        return cls(
            cls._safe_float(x, 0.0),
            cls._safe_float(y, 0.0),
            cls._safe_float(z, 0.0),
            cls._safe_float(a, 0.0),
            cls._safe_float(b, 0.0),
            cls._safe_float(c, 0.0),
        )

    @classmethod
    def from_sequence(cls, values: object, fill_missing: bool = False) -> "Pose6":
        if isinstance(values, Pose6):
            return values.copy()
        if not isinstance(values, Iterable) or isinstance(values, (str, bytes, Mapping)):
            raise TypeError("Pose6 sequence must be an iterable with 6 values")

        seq = list(values)
        if len(seq) < 6 and not fill_missing:
            raise ValueError("Pose6 sequence must contain at least 6 values")
        while len(seq) < 6:
            seq.append(0.0)
        return cls.from_values(*seq[:6])

    @classmethod
    def from_mapping(cls, values: Mapping[str, object]) -> "Pose6":
        return cls.from_values(
            values.get("x", 0.0),
            values.get("y", 0.0),
            values.get("z", 0.0),
            values.get("a", 0.0),
            values.get("b", 0.0),
            values.get("c", 0.0),
        )

    @classmethod
    def from_any(cls, values: object, fill_missing: bool = True) -> "Pose6":
        if isinstance(values, Pose6):
            return values.copy()
        if isinstance(values, Mapping):
            return cls.from_mapping(values)
        if isinstance(values, Iterable) and not isinstance(values, (str, bytes)):
            return cls.from_sequence(values, fill_missing=fill_missing)
        if fill_missing:
            return cls.zeros()
        raise TypeError("Unsupported Pose6 value")

    def copy(self) -> "Pose6":
        return Pose6(self.x, self.y, self.z, self.a, self.b, self.c)

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.a, self.b, self.c]

    def to_tuple(self) -> tuple[float, float, float, float, float, float]:
        return (self.x, self.y, self.z, self.a, self.b, self.c)

    def xyz(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def abc(self) -> tuple[float, float, float]:
        return (self.a, self.b, self.c)

    def __len__(self) -> int:
        return 6

    def __iter__(self) -> Iterator[float]:
        yield self.x
        yield self.y
        yield self.z
        yield self.a
        yield self.b
        yield self.c

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
        if index == 3:
            self.a = normalized
            return
        if index == 4:
            self.b = normalized
            return
        if index == 5:
            self.c = normalized
            return
        raise IndexError("Pose6 index out of range")

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Pose6):
            return self.to_tuple() == other.to_tuple()
        if isinstance(other, Mapping):
            try:
                return self == Pose6.from_mapping(other)
            except (TypeError, ValueError):
                return False
        if isinstance(other, Iterable) and not isinstance(other, (str, bytes)):
            try:
                return self == Pose6.from_sequence(other, fill_missing=False)
            except (TypeError, ValueError):
                return False
        return False

    def __repr__(self) -> str:
        return (
            f"Pose6(x={self.x}, y={self.y}, z={self.z}, "
            f"a={self.a}, b={self.b}, c={self.c})"
        )
