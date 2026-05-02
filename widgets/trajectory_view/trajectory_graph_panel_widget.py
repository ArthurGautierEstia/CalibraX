from typing import List, Optional
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QWIDGETSIZE_MAX
from PyQt6.QtCore import Qt
import pyqtgraph as pg
from enum import Enum

class GraphMode(Enum):
    CARTESIAN = 0
    ARTICULAR = 1


class GraphDisplayMode(Enum):
    LINE = "line"
    DOT = "dot"

class TrajectoryGraphPanelWidget(QWidget):
    """Graph panel for a trajectory (articular or cartesian)."""

    POSITION_LBL = "Position"
    VELOCITY_LBL = "Vitesse"
    ACCELERATION_LBL = "Acceleration"
    JERK_LBL = "Jerk"
    TIME_LBL = "Temps"
    PLOT_MIN_HEIGHT_PX = 150
    PANEL_MIN_HEIGHT_PX = 180
    PANEL_HEADER_HEIGHT_PX = 92
    PANEL_PLOT_HEIGHT_PX = 170

    AXIS_COLORS = ["#ff3b30", "#34c759", "#007aff", "#ff00ff", "#ffd60a", "#00ffff"]
    AXIS_LABELS = {
        GraphMode.CARTESIAN: ["X", "Y", "Z", "A", "B", "C"],
        GraphMode.ARTICULAR: ["J1", "J2", "J3", "J4", "J5", "J6"],
    }
    TITLE_MAP = {
        GraphMode.CARTESIAN: "Cartesien",
        GraphMode.ARTICULAR: "Articulaire",
    }

    @staticmethod
    def lblWithUnit(lbl: str, unit: str) -> str:
        return f"{lbl} ({unit})"

    def __init__(self, mode: GraphMode = GraphMode.CARTESIAN, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.mode = mode
        self.title_label = QLabel()
        self.axis_checkboxes: List[QCheckBox] = []
        self.axis_labels: List[QCheckBox] = []

        self.position_plot = pg.PlotWidget()
        self.velocity_plot = pg.PlotWidget()
        self.acceleration_plot = pg.PlotWidget()
        self.jerk_plot = pg.PlotWidget()

        self._plots = [self.position_plot, self.velocity_plot, self.acceleration_plot, self.jerk_plot]
        self._plot_items: List[List[pg.PlotDataItem]] = []
        self._axis_pens = [pg.mkPen(color=color, width=2) for color in self.AXIS_COLORS]
        self._plot_data: List[List[List[float]]] = [
            [[] for _ in range(6)],
            [[] for _ in range(6)],
            [[] for _ in range(6)],
            [[] for _ in range(6)],
        ]
        self._time_data: List[List[float]] = [[], [], [], []]
        self._key_times: List[float] = []
        self._key_time_lines: List[List[pg.InfiniteLine]] = [[], [], [], []]
        self._time_indicator_lines: List[Optional[pg.InfiniteLine]] = [None, None, None, None]
        self._time_indicator_value: Optional[float] = None
        self._display_mode = GraphDisplayMode.LINE
        self._plot_visible = [True, True, False, False]
        self._plot_data_dirty = [False, False, False, False]
        self._plot_range_dirty = [False, False, False, False]
        self._plot_key_times_dirty = [False, False, False, False]
        self._plot_time_indicator_dirty = [False, False, False, False]
        self._in_page_mode = True

        self._setup_ui()
        self._setup_plots()
        self.set_mode(mode)
        self.set_in_page_mode(True)
        self._apply_all_plot_visibility()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        self.title_label.setStyleSheet("font-size: 12px; font-weight: bold;")
        layout.addWidget(self.title_label)

        header_layout = QHBoxLayout()
        for i in range(6):
            row_layout = QHBoxLayout()

            cb = QCheckBox()
            cb.setChecked(True)
            cb.toggled.connect(self._update_visibility)
            self.axis_checkboxes.append(cb)

            lbl = QLabel()
            lbl.setStyleSheet(f"color: {self.AXIS_COLORS[i]}")
            self.axis_labels.append(lbl)

            row_layout.addWidget(cb)
            row_layout.addWidget(lbl)
            
            header_layout.addLayout(row_layout)

        header_layout.addStretch()
        layout.addLayout(header_layout)

        self.position_plot.setMinimumHeight(self.PLOT_MIN_HEIGHT_PX)
        self.velocity_plot.setMinimumHeight(self.PLOT_MIN_HEIGHT_PX)
        self.acceleration_plot.setMinimumHeight(self.PLOT_MIN_HEIGHT_PX)
        self.jerk_plot.setMinimumHeight(self.PLOT_MIN_HEIGHT_PX)
        layout.addWidget(self.position_plot)
        layout.addWidget(self.velocity_plot)
        layout.addWidget(self.acceleration_plot)
        layout.addWidget(self.jerk_plot)
        layout.setStretch(2, 1)
        layout.setStretch(3, 1)
        layout.setStretch(4, 1)
        layout.setStretch(5, 1)

    def set_in_page_mode(self, in_page: bool) -> None:
        self._in_page_mode = bool(in_page)
        self.setMinimumHeight(self.PANEL_MIN_HEIGHT_PX)
        if in_page:
            self.setFixedHeight(self._preferred_panel_height())
            return
        self.setMinimumHeight(self._preferred_panel_height())
        self.setMaximumHeight(QWIDGETSIZE_MAX)

    def _preferred_panel_height(self) -> int:
        visible_count = sum(1 for visible in self._plot_visible if visible)
        return max(self.PANEL_MIN_HEIGHT_PX, self.PANEL_HEADER_HEIGHT_PX + visible_count * self.PANEL_PLOT_HEIGHT_PX)

    def _setup_plots(self) -> None:
        titles = [self.POSITION_LBL, self.VELOCITY_LBL, self.ACCELERATION_LBL, self.JERK_LBL]
        for plot, title in zip(self._plots, titles):
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setTitle(title)
            plot.setLabel("bottom", TrajectoryGraphPanelWidget.lblWithUnit(self.TIME_LBL, "s"))

        for plot in self._plots:
            items = []
            for _color in self.AXIS_COLORS:
                item = plot.plot([], [])
                items.append(item)
            self._plot_items.append(items)

    def set_mode(self, mode: GraphMode) -> None:
        if mode not in self.AXIS_LABELS:
            mode = GraphMode.CARTESIAN
        self.mode = mode

        self.title_label.setText(self.TITLE_MAP.get(mode, mode))

        labels = self.AXIS_LABELS[mode]
        for lbl, label in zip(self.axis_labels, labels):
            lbl.setText(label)

        position_unit, velocity_unit, acceleration_unit, jerk_unit = self._unit_labels()
        self.position_plot.setLabel("left", TrajectoryGraphPanelWidget.lblWithUnit(self.POSITION_LBL, position_unit))
        self.velocity_plot.setLabel("left", TrajectoryGraphPanelWidget.lblWithUnit(self.VELOCITY_LBL, velocity_unit))
        self.acceleration_plot.setLabel("left", TrajectoryGraphPanelWidget.lblWithUnit(self.ACCELERATION_LBL, acceleration_unit))
        self.jerk_plot.setLabel("left", TrajectoryGraphPanelWidget.lblWithUnit(self.JERK_LBL, jerk_unit))

    def _unit_labels(self) -> tuple[str, str, str, str]:
        return (
            ("deg", "deg/s", "deg/s^2", "deg/s^3")
            if self.mode == GraphMode.ARTICULAR
            else ("mm", "mm/s", "mm/s^2", "mm/s^3")
        )

    def set_trajectories(
        self,
        time_s: List[float],
        positions: Optional[List[List[float]]] = None,
        velocities: Optional[List[List[float]]] = None,
        accelerations: Optional[List[List[float]]] = None,
        jerks: Optional[List[List[float]]] = None,
    ) -> None:
        if positions is not None:
            self._set_plot_data(0, time_s, positions)
        if velocities is not None:
            self._set_plot_data(1, time_s, velocities)
        if accelerations is not None:
            self._set_plot_data(2, time_s, accelerations)
        if jerks is not None:
            self._set_plot_data(3, time_s, jerks)

    def set_plot_visibility(
        self,
        position_visible: bool,
        velocity_visible: bool,
        acceleration_visible: bool,
        jerk_visible: bool,
    ) -> None:
        requested = [position_visible, velocity_visible, acceleration_visible, jerk_visible]
        for plot_idx, visible in enumerate(requested):
            self._set_single_plot_visibility(plot_idx, visible)

    def _apply_all_plot_visibility(self) -> None:
        for plot_idx, visible in enumerate(self._plot_visible):
            self._plots[plot_idx].setVisible(visible)
        self._apply_panel_height()

    def _apply_panel_height(self) -> None:
        preferred_height = self._preferred_panel_height()
        if self._in_page_mode:
            self.setFixedHeight(preferred_height)
            return
        self.setMinimumHeight(preferred_height)
        self.setMaximumHeight(QWIDGETSIZE_MAX)

    def _set_single_plot_visibility(self, plot_idx: int, visible: bool) -> None:
        if plot_idx < 0 or plot_idx >= len(self._plots):
            return

        normalized_visible = bool(visible)
        if self._plot_visible[plot_idx] == normalized_visible:
            return

        self._plot_visible[plot_idx] = normalized_visible
        self._plots[plot_idx].setVisible(normalized_visible)
        self._apply_panel_height()

        if not normalized_visible:
            return

        self._apply_axis_visibility(plot_idx)
        if self._plot_data_dirty[plot_idx]:
            self._refresh_plot_items(plot_idx)
        if self._plot_key_times_dirty[plot_idx]:
            self._refresh_key_time_lines(plot_idx)
        if self._plot_time_indicator_dirty[plot_idx]:
            self._refresh_time_indicator_line(plot_idx)
        if self._plot_range_dirty[plot_idx]:
            self._update_ranges(plot_idx)

    def set_display_mode(self, display_mode: GraphDisplayMode | str) -> None:
        if isinstance(display_mode, GraphDisplayMode):
            normalized = display_mode
        else:
            try:
                normalized = GraphDisplayMode(str(display_mode).strip().lower())
            except ValueError:
                normalized = GraphDisplayMode.LINE

        if normalized == self._display_mode:
            return

        self._display_mode = normalized
        for plot_idx in range(len(self._plots)):
            if self._plot_visible[plot_idx]:
                self._refresh_plot_items(plot_idx)
            else:
                self._plot_data_dirty[plot_idx] = True

    def set_key_times(self, times: List[float]) -> None:
        self._key_times = list(times)
        for idx in range(len(self._plots)):
            if self._plot_visible[idx]:
                self._refresh_key_time_lines(idx)
            else:
                self._plot_key_times_dirty[idx] = True

    def set_time_indicator(self, time_s: Optional[float]) -> None:
        self._time_indicator_value = time_s
        for idx in range(len(self._plots)):
            if self._plot_visible[idx]:
                self._refresh_time_indicator_line(idx)
            else:
                self._plot_time_indicator_dirty[idx] = True

    def _set_plot_data(self, plot_idx: int, time_s: List[float], series: List[List[float]]) -> None:
        if len(series) < 6:
            return
        self._time_data[plot_idx] = list(time_s)
        self._plot_data[plot_idx] = [list(values) for values in series[:6]]
        self._plot_range_dirty[plot_idx] = True
        if not self._plot_visible[plot_idx]:
            self._plot_data_dirty[plot_idx] = True
            return
        self._refresh_plot_items(plot_idx)
        self._update_ranges(plot_idx)

    def _refresh_plot_items(self, plot_idx: int) -> None:
        time_s = self._time_data[plot_idx]
        for axis in range(6):
            self._plot_items[plot_idx][axis].setData(
                time_s,
                self._plot_data[plot_idx][axis],
                **self._display_kwargs(axis),
            )
        self._plot_data_dirty[plot_idx] = False

    def _display_kwargs(self, axis: int) -> dict:
        color = self.AXIS_COLORS[axis]
        pen = self._axis_pens[axis]
        if self._display_mode == GraphDisplayMode.DOT:
            return {
                "pen": None,
                "symbol": "o",
                "symbolSize": 2,
                "symbolPen": None,
                "symbolBrush": color,
            }
        return {
            "pen": pen,
            "symbol": None,
        }

    def _update_visibility(self) -> None:
        for plot_idx in range(len(self._plots)):
            if not self._plot_visible[plot_idx]:
                self._plot_range_dirty[plot_idx] = True
                continue
            self._apply_axis_visibility(plot_idx)
            self._update_ranges(plot_idx)

    def _apply_axis_visibility(self, plot_idx: int) -> None:
        for axis, cb in enumerate(self.axis_checkboxes):
            self._plot_items[plot_idx][axis].setVisible(cb.isChecked())

    def _refresh_key_time_lines(self, plot_idx: int) -> None:
        plot = self._plots[plot_idx]
        for line in self._key_time_lines[plot_idx]:
            plot.removeItem(line)
        self._key_time_lines[plot_idx] = []
        for t in self._key_times:
            line = pg.InfiniteLine(
                pos=t,
                angle=90,
                pen=pg.mkPen(color="#808080", width=1, style=Qt.PenStyle.DashLine),
            )
            plot.addItem(line)
            self._key_time_lines[plot_idx].append(line)
        self._plot_key_times_dirty[plot_idx] = False

    def _refresh_time_indicator_line(self, plot_idx: int) -> None:
        plot = self._plots[plot_idx]
        line = self._time_indicator_lines[plot_idx]
        time_s = self._time_indicator_value
        if time_s is None:
            if line is not None:
                plot.removeItem(line)
            self._time_indicator_lines[plot_idx] = None
            self._plot_time_indicator_dirty[plot_idx] = False
            return

        if line is None:
            line = pg.InfiniteLine(pos=time_s, angle=90, pen=pg.mkPen(color="#ff3b30", width=2))
            plot.addItem(line)
            self._time_indicator_lines[plot_idx] = line
        else:
            line.setValue(time_s)
        self._plot_time_indicator_dirty[plot_idx] = False

    def _update_ranges(self, plot_idx: int) -> None:
        if not self._plot_visible[plot_idx]:
            self._plot_range_dirty[plot_idx] = True
            return

        time_s = self._time_data[plot_idx]
        if time_s:
            self._plots[plot_idx].setXRange(min(time_s), max(time_s), padding=0.02)

        visible_axes = [i for i, cb in enumerate(self.axis_checkboxes) if cb.isChecked()]
        if not visible_axes:
            self._plots[plot_idx].setYRange(-1.0, 1.0)
            self._plot_range_dirty[plot_idx] = False
            return

        min_val = None
        max_val = None
        for axis in visible_axes:
            values = self._plot_data[plot_idx][axis]
            if not values:
                continue
            local_min = min(values)
            local_max = max(values)
            min_val = local_min if min_val is None else min(min_val, local_min)
            max_val = local_max if max_val is None else max(max_val, local_max)

        if min_val is None or max_val is None:
            self._plots[plot_idx].setYRange(-1.0, 1.0)
            self._plot_range_dirty[plot_idx] = False
            return

        if min_val == max_val:
            min_val -= 1.0
            max_val += 1.0
        self._plots[plot_idx].setYRange(min_val, max_val, padding=0.1)
        self._plot_range_dirty[plot_idx] = False
