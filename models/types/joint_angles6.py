from __future__ import annotations

from collections.abc import Sequence


class JointAngles6:
    __slots__ = ("j1", "j2", "j3", "j4", "j5", "j6")

    def __init__(
        self,
        j1: float = 0.0,
        j2: float = 0.0,
        j3: float = 0.0,
        j4: float = 0.0,
        j5: float = 0.0,
        j6: float = 0.0,
    ) -> None:
        self.j1 = float(j1)
        self.j2 = float(j2)
        self.j3 = float(j3)
        self.j4 = float(j4)
        self.j5 = float(j5)
        self.j6 = float(j6)

    @classmethod
    def zeros(cls) -> "JointAngles6":
        return cls()

    @classmethod
    def from_values(cls, values: "JointAngles6 | Sequence[float] | None") -> "JointAngles6":
        if isinstance(values, JointAngles6):
            return values.copy()
        if values is None:
            return cls()
        if len(values) < 6:
            raise ValueError("JointAngles6 requires at least 6 values")
        return cls(
            float(values[0]),
            float(values[1]),
            float(values[2]),
            float(values[3]),
            float(values[4]),
            float(values[5]),
        )

    def copy(self) -> "JointAngles6":
        return JointAngles6(self.j1, self.j2, self.j3, self.j4, self.j5, self.j6)

    def to_list(self) -> list[float]:
        return [self.j1, self.j2, self.j3, self.j4, self.j5, self.j6]

    def to_tuple(self) -> tuple[float, float, float, float, float, float]:
        return (self.j1, self.j2, self.j3, self.j4, self.j5, self.j6)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, JointAngles6) and self.to_tuple() == other.to_tuple()

    def __repr__(self) -> str:
        return (
            f"JointAngles6(j1={self.j1}, j2={self.j2}, j3={self.j3}, "
            f"j4={self.j4}, j5={self.j5}, j6={self.j6})"
        )
