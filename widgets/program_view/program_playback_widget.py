from __future__ import annotations

from PyQt6.QtCore import QEvent, QPoint, QSize, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QIcon, QPainter, QPalette, QPixmap, QPolygon
from PyQt6.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget


class _PlaybackIconButton(QPushButton):
    def __init__(self, icon_kind: str, tooltip: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._icon_kind = icon_kind
        self.setToolTip(tooltip)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFixedSize(40, 40)
        self.setIconSize(QSize(18, 18))
        self._refresh_icon()

    def changeEvent(self, event) -> None:  # type: ignore[override]
        super().changeEvent(event)
        if event.type() in (QEvent.Type.PaletteChange, QEvent.Type.EnabledChange):
            self._refresh_icon()

    def _refresh_icon(self) -> None:
        accent_color = self.palette().color(QPalette.ColorRole.Highlight)
        if not self.isEnabled():
            accent_color.setAlpha(90)
        self.setIcon(self._build_icon(accent_color))

    def _build_icon(self, color: QColor) -> QIcon:
        pixmap = QPixmap(self.iconSize())
        pixmap.fill(Qt.GlobalColor.transparent)

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(color)
        painter.setBrush(color)

        width = pixmap.width()
        height = pixmap.height()

        if self._icon_kind == "play":
            painter.setPen(Qt.PenStyle.NoPen)
            polygon = QPolygon([
                self._point(4, 2),
                self._point(width - 4, height // 2),
                self._point(4, height - 2),
            ])
            painter.drawPolygon(polygon)
        elif self._icon_kind == "pause":
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRoundedRect(3, 2, 4, height - 4, 1, 1)
            painter.drawRoundedRect(width - 7, 2, 4, height - 4, 1, 1)
        elif self._icon_kind == "chevron_left":
            painter.drawPolyline(QPolygon([
                self._point(11, 3),
                self._point(6, height // 2),
                self._point(11, height - 3),
            ]))
        elif self._icon_kind == "double_chevron_left":
            painter.drawPolyline(QPolygon([
                self._point(12, 3),
                self._point(8, height // 2),
                self._point(12, height - 3),
            ]))
            painter.drawPolyline(QPolygon([
                self._point(8, 3),
                self._point(4, height // 2),
                self._point(8, height - 3),
            ]))
        elif self._icon_kind == "chevron_right":
            painter.drawPolyline(QPolygon([
                self._point(7, 3),
                self._point(12, height // 2),
                self._point(7, height - 3),
            ]))
        elif self._icon_kind == "double_chevron_right":
            painter.drawPolyline(QPolygon([
                self._point(6, 3),
                self._point(10, height // 2),
                self._point(6, height - 3),
            ]))
            painter.drawPolyline(QPolygon([
                self._point(10, 3),
                self._point(14, height // 2),
                self._point(10, height - 3),
            ]))
        else:
            painter.setPen(Qt.PenStyle.NoPen)
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
    speed_offset_changed = pyqtSignal(int)

    _SPEED_STEP_PERCENT = 50
    _SPEED_MIN_PERCENT = -100
    _SPEED_MAX_PERCENT = 100

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._time_range = (0.0, 0.0)
        self._is_playing = False
        self._speed_offset_percent = 0
        self._playback_enabled = False

        self._background_widget = QWidget(self)
        self._background_widget.setObjectName("programPlaybackBar")
        self.btn_play_pause = _PlaybackIconButton("play", "Demarrer", self._background_widget)
        self.btn_stop = _PlaybackIconButton("stop", "Stop", self._background_widget)
        self.btn_slower = _PlaybackIconButton("chevron_left", "Ralentir", self._background_widget)
        self.btn_faster = _PlaybackIconButton("chevron_right", "Accelerer", self._background_widget)
        self.loop_checkbox = QCheckBox("Loop", self._background_widget)
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
        layout.addWidget(self.btn_slower)
        layout.addWidget(self.btn_faster)
        layout.addWidget(self.loop_checkbox)
        layout.addWidget(self.time_slider, 1)
        layout.addWidget(self.time_label)
        self._update_speed_buttons()

    def _setup_connections(self) -> None:
        self.btn_play_pause.clicked.connect(self._on_play_pause_clicked)
        self.btn_stop.clicked.connect(self.stop_requested.emit)
        self.btn_slower.clicked.connect(self._on_slower_clicked)
        self.btn_faster.clicked.connect(self._on_faster_clicked)
        self.time_slider.valueChanged.connect(self._on_slider_changed)

    def changeEvent(self, event) -> None:  # type: ignore[override]
        super().changeEvent(event)
        if event.type() == QEvent.Type.PaletteChange:
            self._apply_styles()

    def _apply_styles(self) -> None:
        text_color = self.palette().color(QPalette.ColorRole.Text)
        text_rgba = (
            f"rgba({text_color.red()}, {text_color.green()}, {text_color.blue()}, {text_color.alpha()})"
        )

        self._background_widget.setStyleSheet(
            f"""
            QWidget#programPlaybackBar {{
                background-color: rgba(0, 0, 0, 18);
                border: 1px solid rgba(255, 255, 255, 20);
                border-radius: 10px;
            }}
            QLabel {{
                color: {text_rgba};
            }}
            """
        )
        overlay_button_style = """
            QPushButton {
                background-color: transparent;
                border: 1px solid rgba(255, 255, 255, 28);
                border-radius: 8px;
                padding: 0px;
            }
            QPushButton:hover {
                background-color: rgba(255, 255, 255, 20);
                border-color: rgba(255, 255, 255, 55);
            }
            QPushButton:pressed {
                background-color: rgba(255, 255, 255, 28);
            }
            QPushButton:disabled {
                border-color: rgba(255, 255, 255, 14);
                background-color: transparent;
            }
        """
        self.btn_play_pause.setStyleSheet(overlay_button_style)
        self.btn_stop.setStyleSheet(overlay_button_style)
        self.btn_slower.setStyleSheet(overlay_button_style)
        self.btn_faster.setStyleSheet(overlay_button_style)

    def _on_slider_changed(self, value: int) -> None:
        time_value = self._slider_to_time(value)
        self.time_label.setText(f"{time_value:.2f} s")
        self._update_stop_button()
        self.time_value_changed.emit(time_value)

    def set_time_range(self, min_t: float, max_t: float) -> None:
        self._time_range = (float(min_t), float(max_t))
        self.set_time_value(min_t)

    def set_time_value(self, time_value: float) -> None:
        self.time_slider.blockSignals(True)
        self.time_slider.setValue(self._time_to_slider(time_value))
        self.time_slider.blockSignals(False)
        self.time_label.setText(f"{float(time_value):.2f} s")
        self._update_stop_button()

    def set_playback_enabled(self, enabled: bool) -> None:
        self._playback_enabled = bool(enabled)
        self.btn_play_pause.setEnabled(self._playback_enabled)
        self.loop_checkbox.setEnabled(self._playback_enabled)
        self.time_slider.setEnabled(self._playback_enabled)
        self._update_speed_buttons(self._playback_enabled)
        if not self._playback_enabled:
            self.set_playing(False)
            return
        self._update_stop_button()

    def set_playing(self, is_playing: bool) -> None:
        self._is_playing = bool(is_playing)
        self._update_stop_button()
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

    def _on_slower_clicked(self) -> None:
        self._set_speed_offset_percent(self._speed_offset_percent - self._SPEED_STEP_PERCENT, emit=True)

    def _on_faster_clicked(self) -> None:
        self._set_speed_offset_percent(self._speed_offset_percent + self._SPEED_STEP_PERCENT, emit=True)

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

    def get_speed_offset_percent(self) -> int:
        return int(self._speed_offset_percent)

    def is_loop_enabled(self) -> bool:
        return self.loop_checkbox.isChecked()

    def set_speed_offset_percent(self, value: int) -> None:
        self._set_speed_offset_percent(value, emit=False)

    def _set_speed_offset_percent(self, value: int, emit: bool) -> None:
        normalized = self._snap_speed_offset_percent(value)
        if normalized == self._speed_offset_percent:
            self._update_speed_buttons()
            return
        self._speed_offset_percent = normalized
        self._update_speed_buttons()
        if emit:
            self.speed_offset_changed.emit(int(self._speed_offset_percent))

    def _update_speed_buttons(self, playback_enabled: bool | None = None) -> None:
        can_interact = self.btn_play_pause.isEnabled() if playback_enabled is None else bool(playback_enabled)
        self.btn_slower._icon_kind = (
            "double_chevron_left" if self._speed_offset_percent <= -50 else "chevron_left"
        )
        self.btn_faster._icon_kind = (
            "double_chevron_right" if self._speed_offset_percent >= 50 else "chevron_right"
        )
        self.btn_slower._refresh_icon()
        self.btn_faster._refresh_icon()
        self.btn_slower.setEnabled(can_interact and self._speed_offset_percent > self._SPEED_MIN_PERCENT)
        self.btn_faster.setEnabled(can_interact and self._speed_offset_percent < self._SPEED_MAX_PERCENT)

    def _update_stop_button(self) -> None:
        self.btn_stop.setEnabled(self._playback_enabled and (self._is_playing or self.time_slider.value() > 0))

    @classmethod
    def _snap_speed_offset_percent(cls, value: int) -> int:
        clamped_value = max(cls._SPEED_MIN_PERCENT, min(cls._SPEED_MAX_PERCENT, int(value)))
        snapped_steps = int(round(clamped_value / cls._SPEED_STEP_PERCENT))
        return snapped_steps * cls._SPEED_STEP_PERCENT
