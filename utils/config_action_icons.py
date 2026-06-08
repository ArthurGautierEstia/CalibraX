from __future__ import annotations

from PyQt6.QtCore import Qt, QSize
from PyQt6.QtGui import QColor, QIcon, QPainter, QPainterPath, QPalette, QPixmap


CONFIG_ACTION_BUTTON_SIZE = 36
CONFIG_ACTION_ICON_SIZE = QSize(22, 22)


def build_save_icon(palette: QPalette, include_pencil: bool = False) -> QIcon:
    size = 22
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.GlobalColor.transparent)

    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    color = QColor(palette.color(QPalette.ColorRole.ButtonText))
    painter.setPen(color)
    painter.setBrush(Qt.BrushStyle.NoBrush)

    body = QPainterPath()
    body.addRoundedRect(2.0, 2.0, 14.5, 16.5, 1.8, 1.8)
    notch = QPainterPath()
    notch.addRect(4.5, 3.8, 7.2, 3.8)
    label = QPainterPath()
    label.addRect(4.5, 11.2, 8.8, 5.0)
    painter.drawPath(body)
    painter.fillPath(notch, color)
    painter.drawPath(label)

    if include_pencil:
        pencil = QPainterPath()
        pencil.moveTo(13.4, 13.6)
        pencil.lineTo(18.4, 8.6)
        pencil.lineTo(20.0, 10.2)
        pencil.lineTo(15.0, 15.2)
        pencil.closeSubpath()
        tip = QPainterPath()
        tip.moveTo(12.5, 16.1)
        tip.lineTo(13.4, 13.6)
        tip.lineTo(15.0, 15.2)
        tip.closeSubpath()
        painter.fillPath(pencil, color)
        painter.fillPath(tip, color)

    painter.end()
    return QIcon(pixmap)


def build_new_icon(palette: QPalette) -> QIcon:
    size = 22
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.GlobalColor.transparent)

    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    color = QColor(palette.color(QPalette.ColorRole.ButtonText))
    painter.setPen(color)
    painter.setBrush(Qt.BrushStyle.NoBrush)

    page = QPainterPath()
    page.moveTo(5.0, 2.5)
    page.lineTo(13.0, 2.5)
    page.lineTo(17.5, 7.0)
    page.lineTo(17.5, 19.0)
    page.lineTo(5.0, 19.0)
    page.closeSubpath()
    fold = QPainterPath()
    fold.moveTo(13.0, 2.5)
    fold.lineTo(13.0, 7.0)
    fold.lineTo(17.5, 7.0)
    painter.drawPath(page)
    painter.drawPath(fold)

    painter.end()
    return QIcon(pixmap)
