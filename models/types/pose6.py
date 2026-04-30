from __future__ import annotations


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
