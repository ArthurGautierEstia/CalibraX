from typing import List, Optional
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox
from PyQt6.QtCore import Qt
import pyqtgraph as pg
from enum import Enum

class GraphMode(Enum):
    CARTESIAN = 0
    ARTICULAR = 1

class TrajectoryGraphPanelWidget(QWidget):
    """Graph panel for a trajectory (articular or cartesian)."""

    POSITION_LBL = "Position"
    VELOCITY_LBL = "Vitesse"
    ACCELERATION_LBL = "Acceleration"
    TIME_LBL = "Temps"

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

        self._plots = [self.position_plot, self.velocity_plot, self.acceleration_plot]
        self._plot_items: List[List[pg.PlotDataItem]] = []
        self._plot_data: List[List[List[float]]] = [
            [[] for _ in range(6)],
            [[] for _ in range(6)],
            [[] for _ in range(6)],
        ]
        self._time_data: List[List[float]] = [[], [], []]
        self._key_time_lines: List[List[pg.InfiniteLine]] = [[], [], []]
        self._time_indicator_lines: List[Optional[pg.InfiniteLine]] = [None, None, None]

        self._setup_ui()
        self._setup_plots()
        self.set_mode(mode)

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

        layout.addWidget(self.position_plot)
        layout.addWidget(self.velocity_plot)
        layout.addWidget(self.acceleration_plot)

    def _setup_plots(self) -> None:
        titles = [self.POSITION_LBL, self.VELOCITY_LBL, self.ACCELERATION_LBL]
        for plot, title in zip(self._plots, titles):
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.setTitle(title)
            plot.setLabel("bottom", TrajectoryGraphPanelWidget.lblWithUnit(self.TIME_LBL, "s"))

        for plot in self._plots:
            items = []
            for color in self.AXIS_COLORS:
                pen = pg.mkPen(color=color, width=2)
                item = plot.plot([], [], pen=pen)
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

        position_unit, velocity_unit, acceleration_unit = self._unit_labels()
        self.position_plot.setLabel("left", TrajectoryGraphPanelWidget.lblWithUnit(self.POSITION_LBL, position_unit))
        self.velocity_plot.setLabel("left", TrajectoryGraphPanelWidget.lblWithUnit(self.VELOCITY_LBL, velocity_unit))
        self.acceleration_plot.setLabel("left", TrajectoryGraphPanelWidget.lblWithUnit(self.ACCELERATION_LBL, acceleration_unit))

    def _unit_labels(self) -> tuple[str, str, str]:
        return ("°", "°/s", "°/s²") if self.mode == GraphMode.ARTICULAR else ("mm", "mm/s", "mm/s²")

    def set_trajectories(
        self,
        time_s: List[float],
        positions: Optional[List[List[float]]] = None,
        velocities: Optional[List[List[float]]] = None,
        accelerations: Optional[List[List[float]]] = None,
    ) -> None:
        if positions is not None:
            self._set_plot_data(0, time_s, positions)
        if velocities is not None:
            self._set_plot_data(1, time_s, velocities)
        if accelerations is not None:
            self._set_plot_data(2, time_s, accelerations)

    def set_key_times(self, times: List[float]) -> None:
        for idx, plot in enumerate(self._plots):
            for line in self._key_time_lines[idx]:
                plot.removeItem(line)
            self._key_time_lines[idx] = []
            for t in times:
                line = pg.InfiniteLine(
                    pos=t,
                    angle=90,
                    pen=pg.mkPen(color="#808080", width=1, style=Qt.PenStyle.DashLine),
                )
                plot.addItem(line)
                self._key_time_lines[idx].append(line)

    def set_time_indicator(self, time_s: Optional[float]) -> None:
        for idx, plot in enumerate(self._plots):
            line = self._time_indicator_lines[idx]
            if time_s is None:
                if line is not None:
                    plot.removeItem(line)
                self._time_indicator_lines[idx] = None
                continue
            if line is None:
                line = pg.InfiniteLine(pos=time_s, angle=90, pen=pg.mkPen(color="#ff3b30", width=2))
                plot.addItem(line)
                self._time_indicator_lines[idx] = line
            else:
                line.setValue(time_s)

    def _set_plot_data(self, plot_idx: int, time_s: List[float], series: List[List[float]]) -> None:
        if len(series) < 6:
            return
        self._time_data[plot_idx] = list(time_s)
        self._plot_data[plot_idx] = [list(values) for values in series[:6]]
        for axis in range(6):
            self._plot_items[plot_idx][axis].setData(time_s, series[axis])
        self._update_ranges(plot_idx)

    def _update_visibility(self) -> None:
        for plot_idx in range(3):
            for axis, cb in enumerate(self.axis_checkboxes):
                self._plot_items[plot_idx][axis].setVisible(cb.isChecked())
            self._update_ranges(plot_idx)

    def _update_ranges(self, plot_idx: int) -> None:
        time_s = self._time_data[plot_idx]
        if time_s:
            self._plots[plot_idx].setXRange(min(time_s), max(time_s), padding=0.02)

        visible_axes = [i for i, cb in enumerate(self.axis_checkboxes) if cb.isChecked()]
        if not visible_axes:
            self._plots[plot_idx].setYRange(-1.0, 1.0)
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
            return

        if min_val == max_val:
            min_val -= 1.0
            max_val += 1.0
        self._plots[plot_idx].setYRange(min_val, max_val, padding=0.1)
