from __future__ import annotations

import pyqtgraph as pg
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QVBoxLayout, QWidget

from models.types.machining_result import MachiningResult

# Cohérent avec TrajectoryGraphPanelWidget.AXIS_COLORS
_AXIS_COLORS = ["#ff3b30", "#34c759", "#007aff", "#ff00ff", "#ffd60a", "#00ffff"]
_FORCE_COLORS = ["#ff3b30", "#34c759", "#007aff"]  # F_t, F_r, F_a
_FORCE_LABELS = ["F_t", "F_r", "F_a"]
_JOINT_LABELS = ["J1", "J2", "J3", "J4", "J5", "J6"]


class MachiningGraphsWidget(QWidget):
    """Affichage des résultats de simulation d'usinage en 4 graphes pyqtgraph."""

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        pg.setConfigOptions(antialias=True)

        self._plot_forces = pg.PlotWidget(title="Efforts de coupe (repère outil)")
        self._plot_torques = pg.PlotWidget(title="Couples articulaires τ_total (N·m)")
        self._plot_ratios = pg.PlotWidget(title="Ratio charge / couple max")
        self._plot_tcp = pg.PlotWidget(title="Déviation TCP ‖δx‖ (mm)")

        self._setup_plots()
        self._setup_ui()

        self._force_curves: list[pg.PlotDataItem] = []
        self._torque_curves: list[pg.PlotDataItem] = []
        self._ratio_curves: list[pg.PlotDataItem] = []
        self._tcp_curve: pg.PlotDataItem | None = None
        self._limit_line: pg.InfiniteLine | None = None

    def _setup_plots(self) -> None:
        for plot in (self._plot_forces, self._plot_torques, self._plot_ratios, self._plot_tcp):
            plot.setLabel("bottom", "Temps (s)")
            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.addLegend()

        self._plot_forces.setLabel("left", "Effort (N)")
        self._plot_torques.setLabel("left", "Couple (N·m)")
        self._plot_ratios.setLabel("left", "Ratio |τ| / τ_max")
        self._plot_tcp.setLabel("left", "‖δx_TCP‖ (mm)")

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        for plot in (self._plot_forces, self._plot_torques, self._plot_ratios, self._plot_tcp):
            layout.addWidget(plot)

    # ------------------------------------------------------------------
    # API publique
    # ------------------------------------------------------------------

    def set_result(self, result: MachiningResult) -> None:
        """Met à jour les 4 graphes avec le résultat de simulation."""
        self.clear()

        if not result.samples:
            return

        t = [s.time_s for s in result.samples]

        # --- Efforts de coupe (constants en v1, tracés quand même sur l'axe temps) ---
        force_series = [
            [s.force_tool_N[0] for s in result.samples],  # F_t
            [s.force_tool_N[1] for s in result.samples],  # F_r
            [s.force_tool_N[2] for s in result.samples],  # F_a
        ]
        for i, (data, label) in enumerate(zip(force_series, _FORCE_LABELS)):
            pen = pg.mkPen(color=_FORCE_COLORS[i], width=2)
            curve = self._plot_forces.plot(t, data, pen=pen, name=label)
            self._force_curves.append(curve)

        # --- Couples articulaires ---
        for i in range(6):
            data = [s.torque_total_Nm[i] for s in result.samples]
            pen = pg.mkPen(color=_AXIS_COLORS[i], width=2)
            curve = self._plot_torques.plot(t, data, pen=pen, name=_JOINT_LABELS[i])
            self._torque_curves.append(curve)

        # --- Ratios de charge ---
        for i in range(6):
            data = [s.torque_ratio[i] for s in result.samples]
            pen = pg.mkPen(color=_AXIS_COLORS[i], width=2)
            curve = self._plot_ratios.plot(t, data, pen=pen, name=_JOINT_LABELS[i])
            self._ratio_curves.append(curve)

        # Ligne limite à ratio = 1.0
        self._limit_line = pg.InfiniteLine(
            pos=1.0, angle=0,
            pen=pg.mkPen(color="#ff3b30", width=1, style=Qt.PenStyle.DashLine),
            label="limite",
        )
        self._plot_ratios.addItem(self._limit_line)

        # --- Déviation TCP ---
        tcp_data = [s.delta_tcp_mm for s in result.samples]
        pen_tcp = pg.mkPen(color="#007aff", width=2)
        self._tcp_curve = self._plot_tcp.plot(t, tcp_data, pen=pen_tcp, name="‖δx‖")

    def clear(self) -> None:
        """Efface tous les graphes."""
        self._plot_forces.clear()
        self._plot_torques.clear()
        self._plot_ratios.clear()
        self._plot_tcp.clear()

        self._force_curves.clear()
        self._torque_curves.clear()
        self._ratio_curves.clear()
        self._tcp_curve = None
        self._limit_line = None
