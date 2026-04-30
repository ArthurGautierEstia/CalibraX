from __future__ import annotations

from collections.abc import Sequence
from math import sqrt


class XYZ3:
    __slots__ = ("x", "y", "z")

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    @classmethod
    def zeros(cls) -> "XYZ3":
        return cls()

    @classmethod
    def from_values(cls, values: "XYZ3 | Sequence[float] | None") -> "XYZ3":
        if isinstance(values, XYZ3):
            return values.copy()
        if values is None:
            return cls()
        return cls(
            float(values[0]) if len(values) > 0 else 0.0,
            float(values[1]) if len(values) > 1 else 0.0,
            float(values[2]) if len(values) > 2 else 0.0,
        )

    def copy(self) -> "XYZ3":
        return XYZ3(self.x, self.y, self.z)

    def norm(self) -> float:
        return sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalized(self, epsilon: float = 1e-9) -> "XYZ3":
        n = self.norm()
        if n <= float(epsilon):
            return XYZ3()
        inv_n = 1.0 / n
        return XYZ3(self.x * inv_n, self.y * inv_n, self.z * inv_n)

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    def to_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def __neg__(self) -> "XYZ3":
        return XYZ3(-self.x, -self.y, -self.z)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, XYZ3) and self.to_tuple() == other.to_tuple()

    def __repr__(self) -> str:
        return f"XYZ3(x={self.x}, y={self.y}, z={self.z})"
