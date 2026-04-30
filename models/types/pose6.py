from __future__ import annotations

from collections.abc import Sequence


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

    @classmethod
    def zeros(cls) -> "Pose6":
        return cls()

    @classmethod
    def from_values(cls, values: "Pose6 | Sequence[float] | None") -> "Pose6":
        if isinstance(values, Pose6):
            return values.copy()
        if values is None:
            return cls()
        return cls(
            float(values[0]) if len(values) > 0 else 0.0,
            float(values[1]) if len(values) > 1 else 0.0,
            float(values[2]) if len(values) > 2 else 0.0,
            float(values[3]) if len(values) > 3 else 0.0,
            float(values[4]) if len(values) > 4 else 0.0,
            float(values[5]) if len(values) > 5 else 0.0,
        )

    def copy(self) -> "Pose6":
        return Pose6(self.x, self.y, self.z, self.a, self.b, self.c)

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.a, self.b, self.c]

    def to_tuple(self) -> tuple[float, float, float, float, float, float]:
        return (self.x, self.y, self.z, self.a, self.b, self.c)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Pose6) and self.to_tuple() == other.to_tuple()

    def __repr__(self) -> str:
        return (
            f"Pose6(x={self.x}, y={self.y}, z={self.z}, "
            f"a={self.a}, b={self.b}, c={self.c})"
        )
