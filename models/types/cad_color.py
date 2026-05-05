from __future__ import annotations


class CadColor:
    __slots__ = ("hex_value",)

    def __init__(self, hex_value: str = "") -> None:
        self.hex_value = CadColor.normalize_hex(hex_value)

    @staticmethod
    def normalize_hex(value: str | None, default: str = "") -> str:
        if value is None:
            return default
        text = str(value).strip().upper()
        if text == "":
            return default
        if text.startswith("#"):
            text = text[1:]
        if len(text) != 6:
            return default
        if not all(char in "0123456789ABCDEF" for char in text):
            return default
        return f"#{text}"

    @classmethod
    def from_value(cls, value: "CadColor | str | None", default: str = "") -> "CadColor":
        if isinstance(value, CadColor):
            return value.copy()
        return cls(cls.normalize_hex(value, default))

    def copy(self) -> "CadColor":
        return CadColor(self.hex_value)

    def to_hex(self) -> str:
        return self.hex_value

    def to_rgb_float_tuple(self, alpha: float = 1.0) -> tuple[float, float, float, float]:
        if self.hex_value == "":
            return (0.5, 0.5, 0.5, float(alpha))
        red = int(self.hex_value[1:3], 16) / 255.0
        green = int(self.hex_value[3:5], 16) / 255.0
        blue = int(self.hex_value[5:7], 16) / 255.0
        return (red, green, blue, float(alpha))

    def __eq__(self, other: object) -> bool:
        return isinstance(other, CadColor) and self.hex_value == other.hex_value

    def __repr__(self) -> str:
        return f"CadColor(hex_value={self.hex_value!r})"
