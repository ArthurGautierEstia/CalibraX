from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem,
    QPushButton, QTabWidget, QAbstractItemView
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QBrush

from RobotTab.widgets.mgi_configuration_selector_widget import MgiConfigurationSelectorWidget

from mgi import (
    MgiResult,
    MgiConfigKey,
    MgiResultStatus
)

# ------------------------------------------------------------
# Helpers visuels
# ------------------------------------------------------------

STATUS_COLORS = {
    MgiResultStatus.VALID: QColor("#37CA4B"),                 # vert
    MgiResultStatus.UNREACHABLE: QColor("#F13A2C"),           # rouge
    MgiResultStatus.SINGULARITY: QColor("#F13A2C"),            # orange
    MgiResultStatus.AXIS_LIMIT_VIOLATED: QColor("#F13A2C"),    # jaune
    MgiResultStatus.FORBIDDEN_CONFIGURATION: QColor("#EEAD22") # gris
}

def status_to_text(status: MgiResultStatus) -> str:
    return status.name.replace("_", " ").title()

# ------------------------------------------------------------
# Widget principal
# ------------------------------------------------------------

class MgiSolutionsWidget(QWidget):
    """
    Widget d'affichage des solutions MGI
    """

    solution_selected = pyqtSignal(MgiConfigKey)

    def __init__(self, parent: QWidget=None):
        super().__init__(parent)

        self._mgi_result: MgiResult | None = None
        self._selected_key: MgiConfigKey | None = None

        self._tabs = QTabWidget()
        self._solutions_tab = QWidget()
        self._filters_tab = QWidget()

        self._allowed_configs = {key: True for key in MgiConfigKey}

        self._config_selector = MgiConfigurationSelectorWidget(self._allowed_configs)

        self._init_solutions_tab()
        self._init_filters_tab()

        self._tabs.addTab(self._solutions_tab, "Solutions")
        self._tabs.addTab(self._filters_tab, "Configurations acceptées")

        layout = QVBoxLayout(self)
        layout.addWidget(self._tabs)

    # --------------------------------------------------------
    # Public API
    # --------------------------------------------------------

    def set_mgi_result(self, result: MgiResult, selected_key: MgiConfigKey|None):
        self._mgi_result = result
        self._selected_key = selected_key
        self._populate_table()
    
    def set_selected_key(self, selected_key: MgiConfigKey|None):
        self._selected_key = selected_key
        if self._mgi_result:
            self._populate_table()

    # --------------------------------------------------------
    # Solutions tab
    # --------------------------------------------------------

    def _init_solutions_tab(self):
        layout = QVBoxLayout(self._solutions_tab)

        self._table = QTableWidget()
        self._table.setColumnCount(9)
        self._table.horizontalHeader().setDefaultSectionSize(110)
        self._table.setHorizontalHeaderLabels([
            "Config",
            "Statut",
            "J1", "J2", "J3", "J4", "J5", "J6",
            "Action"
        ])

        self._table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._table.setSelectionMode(QAbstractItemView.NoSelection)
        self._table.horizontalHeader().setStretchLastSection(True)

        layout.addWidget(self._table)

    def _populate_table(self):
        self._table.setRowCount(0)

        if self._mgi_result is None:
            return
        
        for row, (config_key, item) in enumerate(self._mgi_result.solutions.items()):
            self._table.insertRow(row)

            col_index = 0

            # --- Configuration
            configuration_item = QTableWidgetItem(config_key.name)
            configuration_item.setForeground(QBrush(QColor("orange" if config_key == self._selected_key else "white" )))
            self._table.setItem(row, col_index, configuration_item)
            
            col_index += 1

            # --- Statut
            status_item = QTableWidgetItem(status_to_text(item.status))
            status_item.setForeground(QBrush(STATUS_COLORS.get(item.status, QColor("white"))))
            status_item.setToolTip(self._build_status_tooltip(item))
            self._table.setItem(row, col_index, status_item)
            col_index += 1

            # --- Joints
            for i, joint in enumerate(item.joints):
                joint_item = QTableWidgetItem(f"{joint:.3f}")
                joint_item.setTextAlignment(Qt.AlignCenter)
                self._table.setItem(row, col_index, joint_item)
                col_index += 1

            # --- Bouton sélectionner
            btn = QPushButton("Sélectionner")
            btn.setEnabled(item.status == MgiResultStatus.VALID)
            if not btn.isEnabled():
                btn.setStyleSheet("color: gray")
            
            btn.clicked.connect(lambda _, k=config_key: self.solution_selected.emit(k))
            self._table.setCellWidget(row, col_index, btn)
            col_index += 1

        self._table.resizeColumnsToContents()

    def _build_status_tooltip(self, item) -> str:
        lines = [f"Statut : {item.status.name}"]

        if item.j1Singularity:
            lines.append("• Singularité Q1")
        if item.j3Singularity:
            lines.append("• Singularité Q3")
        if item.j5Singularity:
            lines.append("• Singularité Q5")

        if item.violated_limits:
            violated = ", ".join(f"J{idx+1}" for idx in item.violated_limits)
            lines.append(f"• Axes hors limites : {violated}")

        return "\n".join(lines)

    # --------------------------------------------------------
    # Filters tab (base)
    # --------------------------------------------------------

    def _init_filters_tab(self):
        layout = QVBoxLayout(self._filters_tab)

        self._config_selector = MgiConfigurationSelectorWidget(
            {key: True for key in MgiConfigKey}
        )

        self._config_selector.configurations_changed.connect(
            self._on_configurations_changed
        )

        layout.addWidget(self._config_selector)
        layout.addStretch()

    def _on_configurations_changed(self, states: dict):
        """
        Callback lorsque l'utilisateur modifie les configurations autorisées
        """
        allowed = {k for k, v in states.items() if v}

        # Exemple : reconstruire un filtre MGI
        # self._mgi_params.configuration_filter = MgiConfigurationFilter(allowed)

        # Ou simplement stocker l'état
        self._allowed_configs = allowed
