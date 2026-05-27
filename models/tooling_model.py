from __future__ import annotations

from PyQt6.QtCore import QObject, pyqtSignal

from models.tooling_element import ToolingElement


class ToolingModel(QObject):
    """Modèle de l'outillage (un ou plusieurs éléments CAO chaînés).

    Le repère de chaque élément sert de repère parent à l'élément suivant.
    Le repère du dernier élément = repère outillage (disponible pour la pièce).
    """

    tooling_changed = pyqtSignal()

    # Préfixes partagés avec WorkpieceModel pour identifier les repères parents
    PREFIX_EXT = "ext:"
    PREFIX_WS = "ws:"
    FRAME_WORLD = ""
    FRAME_ROBOT = "robot"

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._parent_frame_id: str = ""
        self._elements: list[ToolingElement] = []

    # ------------------------------------------------------------------
    # Repère parent
    # ------------------------------------------------------------------

    def get_parent_frame_id(self) -> str:
        return self._parent_frame_id

    def set_parent_frame_id(self, frame_id: str) -> None:
        self._parent_frame_id = str(frame_id)
        self.tooling_changed.emit()

    # ------------------------------------------------------------------
    # Éléments
    # ------------------------------------------------------------------

    def get_elements(self) -> list[ToolingElement]:
        return [e.copy() for e in self._elements]

    def get_element(self, element_id: str) -> ToolingElement | None:
        for e in self._elements:
            if e.id == element_id:
                return e.copy()
        return None

    def add_element(self, element: ToolingElement) -> None:
        self._elements.append(element.copy())
        self.tooling_changed.emit()

    def remove_element(self, element_id: str) -> None:
        self._elements = [e for e in self._elements if e.id != element_id]
        self.tooling_changed.emit()

    def update_element(self, element_id: str, element: ToolingElement) -> None:
        for i, e in enumerate(self._elements):
            if e.id == element_id:
                new = element.copy()
                new.id = element_id
                self._elements[i] = new
                self.tooling_changed.emit()
                return

    def move_element_up(self, element_id: str) -> None:
        for i, e in enumerate(self._elements):
            if e.id == element_id and i > 0:
                self._elements[i - 1], self._elements[i] = self._elements[i], self._elements[i - 1]
                self.tooling_changed.emit()
                return

    def move_element_down(self, element_id: str) -> None:
        for i, e in enumerate(self._elements):
            if e.id == element_id and i < len(self._elements) - 1:
                self._elements[i], self._elements[i + 1] = self._elements[i + 1], self._elements[i]
                self.tooling_changed.emit()
                return

    def has_elements(self) -> bool:
        return bool(self._elements)

    # ------------------------------------------------------------------
    # Sérialisation
    # ------------------------------------------------------------------

    def to_dict(self) -> dict:
        return {
            "parent_frame_id": self._parent_frame_id,
            "elements": [e.to_dict() for e in self._elements],
        }

    def from_dict(self, data: dict) -> None:
        self._parent_frame_id = str(data.get("parent_frame_id", ""))
        self._elements = [ToolingElement.from_dict(d) for d in data.get("elements", [])]
        self.tooling_changed.emit()
