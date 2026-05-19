from __future__ import annotations

from PyQt6.QtCore import QEvent, QPoint, QSize, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QIcon, QPainter, QPalette, QPixmap, QPolygon
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget


class _PlaybackIconButton(QPushButton):
    def __init__(self, icon_kind: str, tooltip: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._icon_kind = icon_kind
        self.setToolTip(tooltip)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFixedSize(40, 40)
        self.setIconSize(QSize(22, 22))
        self._refresh_icon()

    def changeEvent(self, event) -> None:  # type: ignore[override]
        super().changeEvent(event)
        if event.type() in (QEvent.Type.PaletteChange, QEvent.Type.EnabledChange):
            self._refresh_icon()

    def _refresh_icon(self) -> None:
        accent_color = self.palette().color(QPalette.ColorRole.Highlight)
        if not self.isEnabled():
            accent_color = self.palette().color(QPalette.ColorRole.ButtonText)
        self.setIcon(self._build_icon(accent_color))

    def _build_icon(self, color: QColor) -> QIcon:
        pixmap = QPixmap(self.iconSize())
        pixmap.fill(Qt.GlobalColor.transparent)

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(color)

        width = pixmap.width()
        height = pixmap.height()

        if self._icon_kind == "play":
            polygon = QPolygon([
                self._point(4, 2),
                self._point(width - 4, height // 2),
                self._point(4, height - 2),
            ])
            painter.drawPolygon(polygon)
        elif self._icon_kind == "pause":
            painter.drawRoundedRect(3, 2, 4, height - 4, 1, 1)
            painter.drawRoundedRect(width - 7, 2, 4, height - 4, 1, 1)
        else:
            painter.drawRoundedRect(3, 3, width - 6, height - 6, 2, 2)

        painter.end()
        return QIcon(pixmap)

    @staticmethod
    def _point(x: int, y: int):
        return QPoint(x, y)


class ProgramPlaybackWidget(QWidget):
    play_requested = pyqtSignal()
    pause_requested = pyqtSignal()
    stop_requested = pyqtSignal()
    time_value_changed = pyqtSignal(float)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._time_range = (0.0, 0.0)
        self._is_playing = False

        self._background_widget = QWidget(self)
        self._background_widget.setObjectName("programPlaybackBar")
        self.btn_play_pause = _PlaybackIconButton("play", "Demarrer", self._background_widget)
        self.btn_stop = _PlaybackIconButton("stop", "Stop", self._background_widget)
        self.time_slider = QSlider(Qt.Orientation.Horizontal, self._background_widget)
        self.time_label = QLabel("0.00 s", self._background_widget)

        self._setup_ui()
        self._setup_connections()
        self._apply_styles()
        self.set_playback_enabled(False)

    def _setup_ui(self) -> None:
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.addWidget(self._background_widget)

        layout = QHBoxLayout(self._background_widget)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(6)

        self.time_slider.setRange(0, 1000)
        self.time_slider.setValue(0)
        self.time_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.time_label.setMinimumWidth(64)

        layout.addWidget(self.btn_play_pause)
        layout.addWidget(self.btn_stop)
        layout.addWidget(self.time_slider, 1)
        layout.addWidget(self.time_label)

    def _setup_connections(self) -> None:
        self.btn_play_pause.clicked.connect(self._on_play_pause_clicked)
        self.btn_stop.clicked.connect(self.stop_requested.emit)
        self.time_slider.valueChanged.connect(self._on_slider_changed)

    def changeEvent(self, event) -> None:  # type: ignore[override]
        super().changeEvent(event)
        if event.type() == QEvent.Type.PaletteChange:
            self._apply_styles()

    def _apply_styles(self) -> None:
        base_color = self.palette().color(QPalette.ColorRole.Base)
        background_rgba = (
            f"rgba({base_color.red()}, {base_color.green()}, {base_color.blue()}, 215)"
        )
        border_color = self.palette().color(QPalette.ColorRole.Mid).name()

        self._background_widget.setStyleSheet(
            f"""
            QWidget#programPlaybackBar {{
                background-color: {background_rgba};
                border: 1px solid {border_color};
                border-radius: 10px;
            }}
            """
        )

    def _on_slider_changed(self, value: int) -> None:
        time_value = self._slider_to_time(value)
        self.time_label.setText(f"{time_value:.2f} s")
        self.time_value_changed.emit(time_value)

    def set_time_range(self, min_t: float, max_t: float) -> None:
        self._time_range = (float(min_t), float(max_t))
        self.set_time_value(min_t)

    def set_time_value(self, time_value: float) -> None:
        self.time_slider.blockSignals(True)
        self.time_slider.setValue(self._time_to_slider(time_value))
        self.time_slider.blockSignals(False)
        self.time_label.setText(f"{float(time_value):.2f} s")

    def set_playback_enabled(self, enabled: bool) -> None:
        playback_enabled = bool(enabled)
        self.btn_play_pause.setEnabled(playback_enabled)
        self.btn_stop.setEnabled(playback_enabled)
        self.time_slider.setEnabled(playback_enabled)
        if not playback_enabled:
            self.set_playing(False)

    def set_playing(self, is_playing: bool) -> None:
        self._is_playing = bool(is_playing)
        icon_kind = "pause" if self._is_playing else "play"
        tooltip = "Pause" if self._is_playing else "Demarrer"
        self.btn_play_pause._icon_kind = icon_kind
        self.btn_play_pause.setToolTip(tooltip)
        self.btn_play_pause._refresh_icon()

    def _on_play_pause_clicked(self) -> None:
        if self._is_playing:
            self.pause_requested.emit()
            return
        self.play_requested.emit()

    def _slider_to_time(self, slider_value: int) -> float:
        min_t, max_t = self._time_range
        if max_t == min_t:
            return min_t
        return min_t + (float(slider_value) / 1000.0) * (max_t - min_t)

    def _time_to_slider(self, time_value: float) -> int:
        min_t, max_t = self._time_range
        if max_t == min_t:
            return 0
        ratio = (float(time_value) - min_t) / (max_t - min_t)
        return int(round(max(0.0, min(1.0, ratio)) * 1000.0))
