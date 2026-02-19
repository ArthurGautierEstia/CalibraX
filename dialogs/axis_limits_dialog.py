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
    """Dialog pour configurer les limites des axes, vitesses et positions speciales."""

    def __init__(
        self,
        parent: QWidget,
        current_limits: list[tuple[float, float]],
        axis_speed_limits: list[float] | None = None,
        home_position: list[float] | None = None,
        reversed_axes: list[int] | None = None,
        position_zero: list[float] | None = None,
        position_transport: list[float] | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Parametrage des axes")
        self.setGeometry(100, 100, 900, 420)

        self.current_limits = list(current_limits[:6])
        while len(self.current_limits) < 6:
            self.current_limits.append((-180.0, 180.0))

        self.axis_speed_limits = list((axis_speed_limits or [])[:6])
        while len(self.axis_speed_limits) < 6:
            self.axis_speed_limits.append(0.0)

        self.home_position = list((home_position or [0.0, -90.0, 90.0, 0.0, 90.0, 0.0])[:6])
        while len(self.home_position) < 6:
            self.home_position.append(0.0)

        self.position_zero = list((position_zero or [0.0, -90.0, 90.0, 0.0, 0.0, 0.0])[:6])
        while len(self.position_zero) < 6:
            self.position_zero.append(0.0)

        self.position_transport = list((position_transport or [0.0, -105.0, 156.0, 0.0, 120.0, 0.0])[:6])
        while len(self.position_transport) < 6:
            self.position_transport.append(0.0)

        self.reversed_axes = list((reversed_axes or [1, 1, 1, 1, 1, 1])[:6])
        while len(self.reversed_axes) < 6:
            self.reversed_axes.append(1)

        self.setup_ui()
        self.center_on_screen()

    def setup_ui(self):
        """Initialise l'interface du dialog"""
        limits_layout = QVBoxLayout()

        self.table_limits = QTableWidget(6, 7)
        self.table_limits.setHorizontalHeaderLabels(
            [
                "Min (deg)",
                "Max (deg)",
                "Vitesse max (deg/s)",
                "Position 0 (deg)",
                "Position transport (deg)",
                "Home (deg)",
                "Inverse",
            ]
        )
        self.table_limits.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_limits.horizontalHeader().setDefaultSectionSize(120)
        limits_layout.addWidget(self.table_limits)

        for i in range(6):
            min_item = QTableWidgetItem(str(self.current_limits[i][0]))
            self.table_limits.setItem(i, 0, min_item)

            max_item = QTableWidgetItem(str(self.current_limits[i][1]))
            self.table_limits.setItem(i, 1, max_item)

            speed_item = QTableWidgetItem(str(self.axis_speed_limits[i]))
            self.table_limits.setItem(i, 2, speed_item)

            position_zero_item = QTableWidgetItem(str(self.position_zero[i]))
            position_zero_item.setFlags(position_zero_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table_limits.setItem(i, 3, position_zero_item)

            position_transport_item = QTableWidgetItem(str(self.position_transport[i]))
            position_transport_item.setFlags(position_transport_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.table_limits.setItem(i, 4, position_transport_item)

            home_item = QTableWidgetItem(str(self.home_position[i]))
            self.table_limits.setItem(i, 5, home_item)

            checkbox = QCheckBox()
            checkbox.setChecked(self.reversed_axes[i] == -1)
            self.table_limits.setCellWidget(i, 6, checkbox)

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
        """Centre la fenetre de dialogue sur l'ecran"""
        screen_geometry = QApplication.primaryScreen().geometry()
        dialog_width = self.width()
        dialog_height = self.height()
        x = (screen_geometry.width() - dialog_width) // 2
        y = (screen_geometry.height() - dialog_height) // 2
        self.move(x, y)

    def _cell_to_float(self, row: int, column: int) -> float:
        """Retourne la valeur float de la cellule specifiee"""
        item = self.table_limits.item(row, column)
        return float(item.text()) if item is not None else 0.0

    def get_limits(self):
        """Retourne les nouvelles limites configurees"""
        limits: list[tuple[float, float]] = []
        for i in range(6):
            min_val = self._cell_to_float(i, 0)
            max_val = self._cell_to_float(i, 1)
            limits.append((min_val, max_val))
        return limits

    def get_axis_speed_limits(self):
        """Retourne les nouvelles limites de vitesse configurees"""
        speed_limits: list[float] = []
        for i in range(6):
            speed_limits.append(self._cell_to_float(i, 2))
        return speed_limits

    def get_home_position(self):
        """Retourne la position home configuree"""
        home_pos: list[float] = []
        for i in range(6):
            home_val = self._cell_to_float(i, 5)
            home_pos.append(home_val)
        return home_pos

    def get_axis_reversed(self):
        """Retourne l'etat d'inversion pour chaque axe"""
        axis_reversed: list[int] = []
        for i in range(6):
            checkbox: QCheckBox = self.table_limits.cellWidget(i, 6)
            axis_reversed.append(-1 if checkbox.isChecked() else 1)
        return axis_reversed
