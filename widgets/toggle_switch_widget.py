from __future__ import annotations

from PyQt6.QtCore import QEvent, QEasingCurve, QPropertyAnimation, QRect, Qt, pyqtProperty, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QPainter
from PyQt6.QtWidgets import QSizePolicy, QWidget


class ToggleSwitchWidget(QWidget):
    """Compact animated switch with configurable on/off labels."""

    toggled = pyqtSignal(bool)
    checkedChanged = pyqtSignal(bool)
    labelChanged = pyqtSignal(str)

    def __init__(
        self,
        off_label: str = "Off",
        on_label: str = "On",
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._off_label = off_label
        self._on_label = on_label
        self._checked = False
        self._animation: QPropertyAnimation | None = None
        self._animation_progress = 0.0

        self._color_off = QColor("#999999")
        self._color_on = QColor("#1ab439")
        self._color_disabled = QColor("#777777")
        self._color_circle = QColor("#ffffff")
        self._color_text = QColor("#ffffff")
        self._color_text_disabled = QColor("#d6d6d6")

        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setMinimumSize(220, 34)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)

    def isChecked(self) -> bool:
        return self._checked

    def setChecked(self, checked: bool) -> None:
        checked = bool(checked)
        if checked == self._checked:
            self.animationProgress = 1.0 if checked else 0.0
            return

        self._checked = checked
        self.toggled.emit(self._checked)
        self.checkedChanged.emit(self._checked)
        self.labelChanged.emit(self.label)
        self._start_animation()

    def offLabel(self) -> str:
        return self._off_label

    def setOffLabel(self, label: str) -> None:
        label = str(label)
        if label == self._off_label:
            return
        was_current_label = not self._checked
        self._off_label = label
        if was_current_label:
            self.labelChanged.emit(self.label)
        self.update()

    def onLabel(self) -> str:
        return self._on_label

    def setOnLabel(self, label: str) -> None:
        label = str(label)
        if label == self._on_label:
            return
        was_current_label = self._checked
        self._on_label = label
        if was_current_label:
            self.labelChanged.emit(self.label)
        self.update()

    def setLabels(self, off_label: str, on_label: str) -> None:
        current_label = self.label
        self._off_label = str(off_label)
        self._on_label = str(on_label)
        if self.label != current_label:
            self.labelChanged.emit(self.label)
        self.update()

    def currentLabel(self) -> str:
        return self._on_label if self._checked else self._off_label

    def animationProgress(self) -> float:
        return self._animation_progress

    def setAnimationProgress(self, progress: float) -> None:
        self._animation_progress = max(0.0, min(1.0, float(progress)))
        self.update()

    def _start_animation(self) -> None:
        if self._animation is not None:
            self._animation.stop()

        self._animation = QPropertyAnimation(self, b"animationProgress")
        self._animation.setStartValue(self._animation_progress)
        self._animation.setEndValue(1.0 if self._checked else 0.0)
        self._animation.setDuration(300)
        self._animation.setEasingCurve(QEasingCurve.Type.InOutCubic)
        self._animation.finished.connect(self._on_animation_finished)
        self._animation.start()

    def _on_animation_finished(self) -> None:
        self._animation = None

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MouseButton.LeftButton and self.isEnabled():
            self.setChecked(not self._checked)
            event.accept()
            return
        super().mousePressEvent(event)

    def changeEvent(self, event) -> None:
        super().changeEvent(event)
        if event.type() == QEvent.Type.EnabledChange:
            self.setCursor(
                Qt.CursorShape.PointingHandCursor
                if self.isEnabled()
                else Qt.CursorShape.ArrowCursor
            )
            self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        margin = 4
        track_height = 20
        track_width = 46
        track_rect = QRect(margin, (self.height() - track_height) // 2, track_width, track_height)
        circle_diameter = track_height - 4

        track_color = self._color_on if self._checked else self._color_off
        if not self.isEnabled():
            track_color = self._color_disabled
        painter.setBrush(track_color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(track_rect, track_height / 2, track_height / 2)

        circle_x = track_rect.x() + 2 + self._animation_progress * (track_width - circle_diameter - 4)
        circle_y = track_rect.y() + 2
        painter.setBrush(self._color_circle)
        painter.drawEllipse(int(circle_x), int(circle_y), circle_diameter, circle_diameter)

        painter.setFont(QFont("Arial", 9, QFont.Weight.Normal))
        painter.setPen(self._color_text if self.isEnabled() else self._color_text_disabled)

        text_rect = QRect(
            track_rect.right() + 10,
            0,
            self.width() - track_rect.right() - margin - 10,
            self.height(),
        )
        painter.drawText(text_rect, Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.label)

    checked = pyqtProperty(bool, fget=isChecked, fset=setChecked, notify=checkedChanged)
    offLabel = pyqtProperty(str, fget=offLabel, fset=setOffLabel)
    onLabel = pyqtProperty(str, fget=onLabel, fset=setOnLabel)
    label = pyqtProperty(str, fget=currentLabel, notify=labelChanged)
    animationProgress = pyqtProperty(float, fget=animationProgress, fset=setAnimationProgress)
