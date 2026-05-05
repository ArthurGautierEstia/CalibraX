from __future__ import annotations

from collections.abc import Sequence

from models.types.cad_color import CadColor


class CadColorPalette:
    __slots__ = ("_colors",)

    def __init__(self, colors: Sequence[CadColor] | None = None) -> None:
        self._colors = [CadColor.from_value(color) for color in (colors or [])]

    @classmethod
    def from_values(
        cls,
        values: Sequence[CadColor | str] | None,
        expected_count: int,
        default_hex_values: Sequence[str],
    ) -> "CadColorPalette":
        colors: list[CadColor] = []
        raw_values = list(values or [])
        for index in range(expected_count):
            default_hex = default_hex_values[index] if index < len(default_hex_values) else "#808080"
            raw_value = raw_values[index] if index < len(raw_values) else default_hex
            colors.append(CadColor.from_value(raw_value, default_hex))
        return cls(colors)

    def copy(self) -> "CadColorPalette":
        return CadColorPalette(self._colors)

    def to_list(self) -> list[CadColor]:
        return [color.copy() for color in self._colors]

    def to_hex_list(self) -> list[str]:
        return [color.to_hex() for color in self._colors]

    def __len__(self) -> int:
        return len(self._colors)

    def __getitem__(self, index: int) -> CadColor:
        return self._colors[index].copy()

    def __eq__(self, other: object) -> bool:
        return isinstance(other, CadColorPalette) and self.to_hex_list() == other.to_hex_list()
