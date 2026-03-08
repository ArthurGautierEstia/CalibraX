from __future__ import annotations

from enum import Enum
from statistics import median
from typing import Dict, List, Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget
import pyqtgraph as pg

from models.trajectory_result import TrajectorySample
from utils.mgi import MgiConfigKey, MgiResultStatus


class ConfigValidityStatus(Enum):
    VALID = "valid"
    FORBIDDEN = "forbidden"
    INVALID = "invalid"


class TrajectoryConfigTimelineWidget(QWidget):
    """Configuration validity timeline for all MGI configurations."""

    TIME_LABEL = "Temps"
    TITLE = "Configurations MGI"
    RECT_HEIGHT = 0.62

    STATUS_COLORS: Dict[ConfigValidityStatus, str] = {
        ConfigValidityStatus.VALID: "#22c55e",
        ConfigValidityStatus.FORBIDDEN: "#f59e0b",
        ConfigValidityStatus.INVALID: "#ef4444",
    }
    SELECTED_COLOR = "#facc15"
    CONFIG_ORDER = [
        MgiConfigKey.FUN,
        MgiConfigKey.FUF,
        MgiConfigKey.FDN,
        MgiConfigKey.FDF,
        MgiConfigKey.BUN,
        MgiConfigKey.BUF,
        MgiConfigKey.BDN,
        MgiConfigKey.BDF,
    ]

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.title_label = QLabel(self.TITLE)
        self.plot = pg.PlotWidget()
        self._status_items: list[pg.BarGraphItem] = []
        self._selected_item: Optional[pg.PlotDataItem] = None
        self._key_time_lines: list[pg.InfiniteLine] = []
        self._time_indicator_line: Optional[pg.InfiniteLine] = None
        self._setup_ui()
        self._setup_plot()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        self.title_label.setStyleSheet("font-size: 12px; font-weight: bold;")
        layout.addWidget(self.title_label)
        layout.addWidget(self.plot)

    def _setup_plot(self) -> None:
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", f"{self.TIME_LABEL} (s)")
        self.plot.setLabel("left", "Config")
        self.plot.setYRange(-0.5, len(self.CONFIG_ORDER) - 0.5, padding=0.0)
        self.plot.setMouseEnabled(x=True, y=False)
        self.plot.setLimits(yMin=-0.5, yMax=len(self.CONFIG_ORDER) - 0.5)

        left_axis = self.plot.getAxis("left")
        ticks = [(self._config_y_value(config_key), config_key.name) for config_key in self.CONFIG_ORDER]
        left_axis.setTicks([ticks])

    def clear(self) -> None:
        for item in self._status_items:
            self.plot.removeItem(item)
        self._status_items = []

        if self._selected_item is not None:
            self.plot.removeItem(self._selected_item)
            self._selected_item = None

        self.set_key_times([])
        self.set_time_indicator(None)
        self.plot.setXRange(0.0, 1.0, padding=0.0)

    def set_configuration_data(self, time_s: List[float], samples: List[TrajectorySample]) -> None:
        count = min(len(time_s), len(samples))
        if count <= 0:
            self.clear()
            return

        times = [float(value) for value in time_s[:count]]
        bounded_samples = samples[:count]
        left_edges, right_edges = self._build_sample_edges(times)

        self._clear_status_items()
        status_batches: dict[ConfigValidityStatus, dict[str, list[float]]] = {
            ConfigValidityStatus.VALID: {"x": [], "width": [], "y0": []},
            ConfigValidityStatus.FORBIDDEN: {"x": [], "width": [], "y0": []},
            ConfigValidityStatus.INVALID: {"x": [], "width": [], "y0": []},
        }

        for config_key in self.CONFIG_ORDER:
            y_center = self._config_y_value(config_key)
            for status, start_idx, end_idx in self._build_status_segments_for_config(bounded_samples, config_key):
                left = left_edges[start_idx]
                right = right_edges[end_idx]
                width = max(1e-6, right - left)
                status_batches[status]["x"].append((left + right) * 0.5)
                status_batches[status]["width"].append(width)
                status_batches[status]["y0"].append(y_center - (self.RECT_HEIGHT * 0.5))

        for status, batches in status_batches.items():
            if not batches["x"]:
                continue
            item = pg.BarGraphItem(
                x=batches["x"],
                y0=batches["y0"],
                width=batches["width"],
                height=self.RECT_HEIGHT,
                brush=pg.mkBrush(self.STATUS_COLORS[status]),
                pen=None,
            )
            self.plot.addItem(item)
            self._status_items.append(item)

        self._set_selected_segments_overlay(bounded_samples, left_edges, right_edges)

        min_x = min(0.0, left_edges[0])
        max_x = max(right_edges[-1], times[-1])
        if max_x <= min_x:
            max_x = min_x + 1.0
        self.plot.setXRange(min_x, max_x, padding=0.02)

    def set_key_times(self, times: List[float]) -> None:
        for line in self._key_time_lines:
            self.plot.removeItem(line)
        self._key_time_lines = []

        for value in times:
            line = pg.InfiniteLine(
                pos=float(value),
                angle=90,
                pen=pg.mkPen(color="#808080", width=1, style=Qt.PenStyle.DashLine),
            )
            self.plot.addItem(line)
            self._key_time_lines.append(line)

    def set_time_indicator(self, time_s: Optional[float]) -> None:
        line = self._time_indicator_line
        if time_s is None:
            if line is not None:
                self.plot.removeItem(line)
            self._time_indicator_line = None
            return

        if line is None:
            line = pg.InfiniteLine(pos=float(time_s), angle=90, pen=pg.mkPen(color="#ff3b30", width=2))
            self.plot.addItem(line)
            self._time_indicator_line = line
            return

        line.setValue(float(time_s))

    def _clear_status_items(self) -> None:
        for item in self._status_items:
            self.plot.removeItem(item)
        self._status_items = []
        if self._selected_item is not None:
            self.plot.removeItem(self._selected_item)
            self._selected_item = None

    @classmethod
    def _config_y_value(cls, config_key: MgiConfigKey) -> float:
        try:
            index = cls.CONFIG_ORDER.index(config_key)
        except ValueError:
            index = 0
        return float((len(cls.CONFIG_ORDER) - 1) - index)

    @staticmethod
    def _status_for_sample_config(sample: TrajectorySample, config_key: MgiConfigKey) -> ConfigValidityStatus:
        solution = sample.mgi_solutions.get(config_key)
        if solution is None:
            return ConfigValidityStatus.INVALID
        if solution.status == MgiResultStatus.VALID.name:
            return ConfigValidityStatus.VALID
        if solution.status == MgiResultStatus.FORBIDDEN_CONFIGURATION.name:
            return ConfigValidityStatus.FORBIDDEN
        return ConfigValidityStatus.INVALID

    def _build_status_segments_for_config(
        self,
        samples: list[TrajectorySample],
        config_key: MgiConfigKey,
    ) -> list[tuple[ConfigValidityStatus, int, int]]:
        if not samples:
            return []

        segments: list[tuple[ConfigValidityStatus, int, int]] = []
        current_status = self._status_for_sample_config(samples[0], config_key)
        current_start = 0

        for idx in range(1, len(samples)):
            sample_status = self._status_for_sample_config(samples[idx], config_key)
            if sample_status == current_status:
                continue
            segments.append((current_status, current_start, idx - 1))
            current_status = sample_status
            current_start = idx

        segments.append((current_status, current_start, len(samples) - 1))
        return segments

    def _set_selected_segments_overlay(
        self,
        samples: list[TrajectorySample],
        left_edges: list[float],
        right_edges: list[float],
    ) -> None:
        if self._selected_item is not None:
            self.plot.removeItem(self._selected_item)
            self._selected_item = None

        if not samples:
            return

        selected_segments: list[tuple[MgiConfigKey, int, int]] = []
        current_key: MgiConfigKey | None = None
        current_start: int | None = None

        for idx, sample in enumerate(samples):
            sample_key = sample.configuration
            if sample_key is None:
                if current_key is not None and current_start is not None:
                    selected_segments.append((current_key, current_start, idx - 1))
                current_key = None
                current_start = None
                continue

            if current_key is None:
                current_key = sample_key
                current_start = idx
                continue

            if sample_key != current_key:
                if current_start is not None:
                    selected_segments.append((current_key, current_start, idx - 1))
                current_key = sample_key
                current_start = idx

        if current_key is not None and current_start is not None:
            selected_segments.append((current_key, current_start, len(samples) - 1))

        if not selected_segments:
            return

        x_values: list[float] = []
        y_values: list[float] = []
        for config_key, start_idx, end_idx in selected_segments:
            left = left_edges[start_idx]
            right = right_edges[end_idx]
            y_line = self._config_y_value(config_key)
            x_values.extend([left, right, float("nan")])
            y_values.extend([y_line, y_line, float("nan")])

        self._selected_item = pg.PlotDataItem(
            x=x_values,
            y=y_values,
            pen=pg.mkPen(color=self.SELECTED_COLOR, width=2),
            connect="finite",
        )
        self.plot.addItem(self._selected_item)

    @staticmethod
    def _build_sample_edges(times: list[float]) -> tuple[list[float], list[float]]:
        if not times:
            return [], []

        if len(times) == 1:
            dt = max(1e-6, times[0]) if times[0] > 0.0 else 0.004
            return [max(0.0, times[0] - 0.5 * dt)], [times[0] + 0.5 * dt]

        deltas = [times[i] - times[i - 1] for i in range(1, len(times))]
        positive_deltas = [delta for delta in deltas if delta > 0.0]
        fallback_dt = median(positive_deltas) if positive_deltas else 0.004
        if fallback_dt <= 0.0:
            fallback_dt = 0.004

        left_edges: list[float] = []
        right_edges: list[float] = []
        for idx, current_time in enumerate(times):
            if idx == 0:
                left = max(0.0, current_time - 0.5 * fallback_dt)
            else:
                left = max(0.0, 0.5 * (times[idx - 1] + current_time))

            if idx == len(times) - 1:
                right = current_time + 0.5 * fallback_dt
            else:
                right = 0.5 * (current_time + times[idx + 1])
                if right <= left:
                    right = left + max(1e-6, 0.5 * fallback_dt)

            left_edges.append(left)
            right_edges.append(right)

        return left_edges, right_edges
