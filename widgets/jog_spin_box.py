from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QMouseEvent
from PyQt6.QtWidgets import QDoubleSpinBox, QStyle, QStyleOptionSpinBox


class JogSpinBox(QDoubleSpinBox):
    """Spin box that exposes press/release events on its arrow buttons."""

    jog_button_pressed = pyqtSignal(int)
    jog_button_released = pyqtSignal(int)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._pressed_direction: int | None = None
        self._allow_jog_while_read_only = False

    def set_allow_jog_while_read_only(self, allow: bool) -> None:
        self._allow_jog_while_read_only = bool(allow)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            direction = self._direction_from_position(event.position().toPoint())
            if (
                direction is not None
                and self.isEnabled()
                and (not self.isReadOnly() or self._allow_jog_while_read_only)
            ):
                self._pressed_direction = direction
                self.jog_button_pressed.emit(direction)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if event.button() == Qt.MouseButton.LeftButton and self._pressed_direction is not None:
            direction = self._pressed_direction
            self._pressed_direction = None
            self.jog_button_released.emit(direction)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def _direction_from_position(self, position) -> int | None:
        option = QStyleOptionSpinBox()
        self.initStyleOption(option)
        control = self.style().hitTestComplexControl(
            QStyle.ComplexControl.CC_SpinBox,
            option,
            position,
            self,
        )
        if control == QStyle.SubControl.SC_SpinBoxUp:
            return 1
        if control == QStyle.SubControl.SC_SpinBoxDown:
            return -1
        return None
