from PyQt6.QtWidgets import (
    QWidget,
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QTableWidget,
    QTableWidgetItem,
    QPushButton,
    QApplication,
)


class AxisPositionsDialog(QDialog):
    """Dialog pour configurer les positions articulaires de référence."""

    COL_POSITION_ZERO = 0
    COL_POSITION_TRANSPORT = 1
    COL_HOME = 2

    def __init__(
        self,
        parent: QWidget,
        home_position: list[float] | None = None,
        position_zero: list[float] | None = None,
        position_transport: list[float] | None = None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Parametrage des positions")
        self.setGeometry(100, 100, 760, 400)

        self.home_position = list((home_position or [0.0, -90.0, 90.0, 0.0, 90.0, 0.0])[:6])
        while len(self.home_position) < 6:
            self.home_position.append(0.0)

        self.position_zero = list((position_zero or [0.0, -90.0, 90.0, 0.0, 0.0, 0.0])[:6])
        while len(self.position_zero) < 6:
            self.position_zero.append(0.0)

        self.position_transport = list((position_transport or [0.0, -105.0, 156.0, 0.0, 120.0, 0.0])[:6])
        while len(self.position_transport) < 6:
            self.position_transport.append(0.0)

        self.setup_ui()
        self.center_on_screen()

    def setup_ui(self):
        layout = QVBoxLayout()

        self.table_positions = QTableWidget(6, 3)
        self.table_positions.setHorizontalHeaderLabels(
            [
                "Position 0 (deg)",
                "Position transport (deg)",
                "Position home (deg)",
            ]
        )
        self.table_positions.setVerticalHeaderLabels([f"q{i + 1}" for i in range(6)])
        self.table_positions.horizontalHeader().setDefaultSectionSize(180)
        layout.addWidget(self.table_positions)

        for i in range(6):
            self.table_positions.setItem(i, AxisPositionsDialog.COL_POSITION_ZERO, QTableWidgetItem(str(self.position_zero[i])))
            self.table_positions.setItem(
                i, AxisPositionsDialog.COL_POSITION_TRANSPORT, QTableWidgetItem(str(self.position_transport[i]))
            )
            self.table_positions.setItem(i, AxisPositionsDialog.COL_HOME, QTableWidgetItem(str(self.home_position[i])))

        btn_layout = QHBoxLayout()
        btn_ok = QPushButton("Valider")
        btn_cancel = QPushButton("Annuler")
        btn_ok.clicked.connect(self.accept)
        btn_cancel.clicked.connect(self.reject)
        btn_layout.addWidget(btn_ok)
        btn_layout.addWidget(btn_cancel)
        layout.addLayout(btn_layout)

        self.setLayout(layout)

    def center_on_screen(self):
        screen_geometry = QApplication.primaryScreen().geometry()
        dialog_width = self.width()
        dialog_height = self.height()
        x = (screen_geometry.width() - dialog_width) // 2
        y = (screen_geometry.height() - dialog_height) // 2
        self.move(x, y)

    def _cell_to_float(self, row: int, column: int) -> float:
        item = self.table_positions.item(row, column)
        return float(item.text()) if item is not None and item.text() else 0.0

    def get_position_zero(self) -> list[float]:
        return [self._cell_to_float(i, AxisPositionsDialog.COL_POSITION_ZERO) for i in range(6)]

    def get_position_transport(self) -> list[float]:
        return [self._cell_to_float(i, AxisPositionsDialog.COL_POSITION_TRANSPORT) for i in range(6)]

    def get_home_position(self) -> list[float]:
        return [self._cell_to_float(i, AxisPositionsDialog.COL_HOME) for i in range(6)]
