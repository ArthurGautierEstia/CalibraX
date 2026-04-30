from __future__ import annotations


class XYZ3:
    __slots__ = ("x", "y", "z")

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    @classmethod
    def zeros(cls) -> "XYZ3":
        return cls()

    def copy(self) -> "XYZ3":
        return XYZ3(self.x, self.y, self.z)

    def to_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    def to_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, XYZ3) and self.to_tuple() == other.to_tuple()

    def __repr__(self) -> str:
        return f"XYZ3(x={self.x}, y={self.y}, z={self.z})"
