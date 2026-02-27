import math

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QWidget,
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QTableWidget,
    QTableWidgetItem,
    QPushButton,
    QApplication,
    QCheckBox,
)


class AxisLimitsDialog(QDialog):
    """Dialog pour configurer les paramètres axes: min/max, vitesse, jerk, accel estimée, inversion."""

    COL_MIN = 0
    COL_MAX = 1
    COL_SPEED = 2
    COL_ACCEL_EST = 3
    COL_JERK = 4
    COL_REVERSED = 5

    def __init__(
        self,
        parent: QWidget,
        current_limits: list[tuple[float, float]],
        axis_speed_limits: list[float] | None = None,
        axis_jerk_limits: list[float] | None = None,
        reversed_axes: list[int] | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Parametrage des axes")
        self.setGeometry(100, 100, 920, 420)

        self.current_limits = list(current_limits[:6])
        while len(self.current_limits) < 6:
            self.current_limits.append((-180.0, 180.0))

        self.axis_speed_limits = list((axis_speed_limits or [])[:6])
        while len(self.axis_speed_limits) < 6:
            self.axis_speed_limits.append(0.0)

        self.axis_jerk_limits = list((axis_jerk_limits or [])[:6])
        while len(self.axis_jerk_limits) < 6:
            self.axis_jerk_limits.append(0.0)

        self.reversed_axes = list((reversed_axes or [1, 1, 1, 1, 1, 1])[:6])
        while len(self.reversed_axes) < 6:
            self.reversed_axes.append(1)

        self.setup_ui()
        self.center_on_screen()

    def setup_ui(self):
        """Initialise l'interface du dialog."""
        limits_layout = QVBoxLayout()

        self.table_limits = QTableWidget(6, 6)
        self.table_limits.setHorizontalHeaderLabels(
            [
                "Min (deg)",
                "Max (deg)",
                "Vitesse max (deg/s)",
                "Accel estimee (deg/s^2)",
                "Jerk max (deg/s^3)",
                "Inverse",
            ]
        )
        self.table_limits.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_limits.horizontalHeader().setDefaultSectionSize(135)
        limits_layout.addWidget(self.table_limits)

        for i in range(6):
            self.table_limits.setItem(i, AxisLimitsDialog.COL_MIN, QTableWidgetItem(str(self.current_limits[i][0])))
            self.table_limits.setItem(i, AxisLimitsDialog.COL_MAX, QTableWidgetItem(str(self.current_limits[i][1])))
            self.table_limits.setItem(i, AxisLimitsDialog.COL_SPEED, QTableWidgetItem(str(self.axis_speed_limits[i])))
            self.table_limits.setItem(i, AxisLimitsDialog.COL_JERK, QTableWidgetItem(str(self.axis_jerk_limits[i])))

            accel_item = QTableWidgetItem("")
            accel_item.setFlags(accel_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table_limits.setItem(i, AxisLimitsDialog.COL_ACCEL_EST, accel_item)

            checkbox = QCheckBox()
            checkbox.setChecked(self.reversed_axes[i] == -1)
            self.table_limits.setCellWidget(i, AxisLimitsDialog.COL_REVERSED, checkbox)

        self._refresh_estimated_accel_column()
        self.table_limits.itemChanged.connect(self._on_item_changed)

        btn_layout = QHBoxLayout()
        btn_ok = QPushButton("Valider")
        btn_cancel = QPushButton("Annuler")
        btn_ok.clicked.connect(self.accept)
        btn_cancel.clicked.connect(self.reject)
        btn_layout.addWidget(btn_ok)
        btn_layout.addWidget(btn_cancel)
        limits_layout.addLayout(btn_layout)

        self.setLayout(limits_layout)

    def center_on_screen(self):
        """Centre la fenetre de dialogue sur l'ecran."""
        screen_geometry = QApplication.primaryScreen().geometry()
        dialog_width = self.width()
        dialog_height = self.height()
        x = (screen_geometry.width() - dialog_width) // 2
        y = (screen_geometry.height() - dialog_height) // 2
        self.move(x, y)

    def _cell_to_float(self, row: int, column: int) -> float:
        """Retourne la valeur float de la cellule spécifiée."""
        item = self.table_limits.item(row, column)
        return float(item.text()) if item is not None and item.text() else 0.0

    def _on_item_changed(self, item: QTableWidgetItem) -> None:
        if item.column() in (AxisLimitsDialog.COL_SPEED, AxisLimitsDialog.COL_JERK):
            self._refresh_estimated_accel_for_row(item.row())

    def _refresh_estimated_accel_for_row(self, row: int) -> None:
        speed = max(0.0, self._cell_to_float(row, AxisLimitsDialog.COL_SPEED))
        jerk = max(0.0, self._cell_to_float(row, AxisLimitsDialog.COL_JERK))
        accel = math.sqrt(speed * jerk)
        accel_item = self.table_limits.item(row, AxisLimitsDialog.COL_ACCEL_EST)
        if accel_item is None:
            accel_item = QTableWidgetItem("")
            accel_item.setFlags(accel_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table_limits.setItem(row, AxisLimitsDialog.COL_ACCEL_EST, accel_item)
        accel_item.setText(f"{accel:.3f}")

    def _refresh_estimated_accel_column(self) -> None:
        self.table_limits.blockSignals(True)
        try:
            for i in range(6):
                self._refresh_estimated_accel_for_row(i)
        finally:
            self.table_limits.blockSignals(False)

    def get_limits(self):
        """Retourne les nouvelles limites configurées."""
        limits: list[tuple[float, float]] = []
        for i in range(6):
            min_val = self._cell_to_float(i, AxisLimitsDialog.COL_MIN)
            max_val = self._cell_to_float(i, AxisLimitsDialog.COL_MAX)
            limits.append((min_val, max_val))
        return limits

    def get_axis_speed_limits(self):
        """Retourne les nouvelles limites de vitesse configurées."""
        return [self._cell_to_float(i, AxisLimitsDialog.COL_SPEED) for i in range(6)]

    def get_axis_jerk_limits(self):
        """Retourne les nouvelles limites de jerk configurées."""
        return [self._cell_to_float(i, AxisLimitsDialog.COL_JERK) for i in range(6)]

    def get_axis_reversed(self):
        """Retourne l'état d'inversion pour chaque axe."""
        axis_reversed: list[int] = []
        for i in range(6):
            checkbox: QCheckBox = self.table_limits.cellWidget(i, AxisLimitsDialog.COL_REVERSED)
            axis_reversed.append(-1 if checkbox.isChecked() else 1)
        return axis_reversed
