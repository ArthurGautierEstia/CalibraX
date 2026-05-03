from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor, QBrush
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QPushButton,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from utils.mgi import MgiConfigKey, MgiResult, MgiResultItem, MgiResultStatus
from utils.mgi_jacobien import MgiJacobienParams, MgiJacobienResultat
from widgets.cartesian_control_view.mgi_jacobien_widget import MgiJacobienWidget


STATUS_COLORS = {
    MgiResultStatus.VALID: QColor("#37CA4B"),
    MgiResultStatus.UNREACHABLE: QColor("#F13A2C"),
    MgiResultStatus.SINGULARITY: QColor("#F13A2C"),
    MgiResultStatus.AXIS_LIMIT_VIOLATED: QColor("#F13A2C"),
    MgiResultStatus.FORBIDDEN_CONFIGURATION: QColor("#EEAD22"),
}


def status_to_text(status: MgiResultStatus) -> str:
    return status.name.replace("_", " ").title()


class MgiSolutionsWidget(QWidget):
    """Runtime display for MGI solutions and Jacobian refinement."""

    solution_selected = pyqtSignal(MgiConfigKey)
    solution_item_selected = pyqtSignal(MgiConfigKey, list)
    jacobien_enabled_changed = pyqtSignal(bool)
    jacobien_params_changed = pyqtSignal()

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        self._mgi_result: MgiResult | None = None
        self._selected_key: MgiConfigKey | None = None
        self._selected_joints: list[float] | None = None
        self._axis_limits: list[tuple[float, float]] = [(-180.0, 180.0) for _ in range(6)]
        self._joint_weights: list[float] = [1.0] * 6
        self._show_expanded_solutions = False

        self._tabs = QTabWidget()
        self._solutions_tab = QWidget()
        self._jacobien_tab = QWidget()

        self._init_solutions_tab()
        self._init_jacobien_tab()

        self._tabs.addTab(self._solutions_tab, "Solutions")
        self._tabs.addTab(self._jacobien_tab, "MGI Optimisé")

        layout = QVBoxLayout(self)
        layout.addWidget(self._tabs)

    def set_axis_limits(self, limits: list[tuple[float, float]]) -> None:
        self._axis_limits = list(limits)

    def set_mgi_result(
        self,
        result: MgiResult,
        selected_key: MgiConfigKey | None,
        selected_joints: list[float] | None = None,
    ) -> None:
        self._mgi_result = result
        self._selected_key = selected_key
        self._selected_joints = self._normalize_joints(selected_joints) if selected_joints is not None else None
        self._populate_table()

    def set_selected_key(
        self,
        selected_key: MgiConfigKey | None,
        selected_joints: list[float] | None = None,
    ) -> None:
        self._selected_key = selected_key
        if selected_joints is not None:
            self._selected_joints = self._normalize_joints(selected_joints)
        if self._mgi_result is not None:
            self._populate_table()

    def set_weights(self, weights: list[float]) -> None:
        self._joint_weights = self._normalize_weights(weights)

    def is_jacobien_enabled(self) -> bool:
        return self._jacobien_widget.is_enabled()

    def get_jacobien_params(self) -> MgiJacobienParams:
        return self._jacobien_widget.get_params()

    def set_jacobien_resultat(self, resultat: MgiJacobienResultat | None) -> None:
        self._jacobien_widget.set_resultat(resultat)

    def _init_solutions_tab(self) -> None:
        layout = QVBoxLayout(self._solutions_tab)

        self._cb_show_expanded = QCheckBox("Afficher solutions étendues")
        self._cb_show_expanded.setChecked(False)
        self._cb_show_expanded.setToolTip("Affiche les variantes equivalentes (tours +/-360) quand disponibles.")
        self._cb_show_expanded.toggled.connect(self._on_show_expanded_toggled)
        layout.addWidget(self._cb_show_expanded)

        self._table = QTableWidget()
        self._table.setColumnCount(9)
        self._table.horizontalHeader().setDefaultSectionSize(110)
        self._table.setHorizontalHeaderLabels(["Config", "Statut", "q1", "q2", "q3", "q4", "q5", "q6", "Action"])
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        layout.addWidget(self._table)

    def _init_jacobien_tab(self) -> None:
        layout = QVBoxLayout(self._jacobien_tab)
        self._jacobien_widget = MgiJacobienWidget()
        self._jacobien_widget.enabled_changed.connect(self.jacobien_enabled_changed)
        self._jacobien_widget.params_changed.connect(self.jacobien_params_changed)
        layout.addWidget(self._jacobien_widget)

    def _populate_table(self) -> None:
        self._table.setRowCount(0)
        if self._mgi_result is None:
            return

        rows = self._display_rows()
        selected_row_index = self._resolve_selected_row_index(rows)

        for row, (config_key, item) in enumerate(rows):
            self._table.insertRow(row)
            is_selected_row = selected_row_index is not None and row == selected_row_index

            configuration_item = QTableWidgetItem(config_key.name)
            configuration_item.setForeground(QBrush(QColor("orange" if is_selected_row else "white")))
            self._table.setItem(row, 0, configuration_item)

            status_item = QTableWidgetItem(status_to_text(item.status))
            status_item.setForeground(QBrush(STATUS_COLORS.get(item.status, QColor("white"))))
            status_item.setToolTip(self._build_status_tooltip(item))
            self._table.setItem(row, 1, status_item)

            for joint_index, joint in enumerate(item.joints[:6]):
                joint_item = QTableWidgetItem(f"{joint:.3f}")
                joint_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self._table.setItem(row, 2 + joint_index, joint_item)

            select_button = QPushButton("Sélectionner")
            select_button.setEnabled(item.status == MgiResultStatus.VALID)
            if not select_button.isEnabled():
                select_button.setStyleSheet("color: gray")
            row_joints = self._normalize_joints([float(value) for value in item.joints[:6]])
            select_button.clicked.connect(lambda _, k=config_key, j=row_joints: self._emit_solution_selection(k, j))
            self._table.setCellWidget(row, 8, select_button)

        self._table.resizeColumnsToContents()

    def _display_rows(self) -> list[tuple[MgiConfigKey, MgiResultItem]]:
        if self._mgi_result is None:
            return []
        if self._show_expanded_solutions and self._mgi_result.expanded_solutions:
            config_order = {key: idx for idx, key in enumerate(MgiConfigKey)}
            rows = [
                (item.config_key, item)
                for item in self._mgi_result.expanded_solutions
                if item.config_key is not None
            ]
            rows.sort(key=lambda pair: (config_order.get(pair[0], 999), pair[1].joints))
            return rows
        return list(self._mgi_result.solutions.items())

    def _on_show_expanded_toggled(self, checked: bool) -> None:
        self._show_expanded_solutions = bool(checked)
        self._populate_table()

    def _emit_solution_selection(self, config_key: MgiConfigKey, joints: list[float]) -> None:
        copied = self._normalize_joints(joints)
        self.solution_item_selected.emit(config_key, copied)
        self.solution_selected.emit(config_key)

    def _resolve_selected_row_index(self, rows: list[tuple[MgiConfigKey, MgiResultItem]]) -> int | None:
        if not rows:
            return None
        if self._selected_joints is None:
            return self._resolve_selected_key_row_index(rows)

        selected = self._normalize_joints(self._selected_joints)
        weights = self._normalize_weights(self._joint_weights)
        exact_candidates: list[int] = []
        best_idx = 0
        best_distance = float("inf")

        for idx, (_, item) in enumerate(rows):
            joints = self._normalize_joints([float(value) for value in item.joints[:6]])
            deltas = [selected[axis] - joints[axis] for axis in range(6)]
            if max(abs(value) for value in deltas) <= 1e-6:
                exact_candidates.append(idx)

            distance = sum(weights[axis] * (deltas[axis] ** 2) for axis in range(6))
            if distance < best_distance:
                best_distance = distance
                best_idx = idx

        if exact_candidates:
            if self._selected_key is not None:
                for idx in exact_candidates:
                    cfg, _ = rows[idx]
                    if cfg == self._selected_key:
                        return idx
            return exact_candidates[0]
        return best_idx

    def _resolve_selected_key_row_index(self, rows: list[tuple[MgiConfigKey, MgiResultItem]]) -> int | None:
        if self._selected_key is None:
            return None
        for idx, (cfg, _) in enumerate(rows):
            if cfg == self._selected_key:
                return idx
        return None

    def _build_status_tooltip(self, item: MgiResultItem) -> str:
        lines = [f"Statut : {item.status.name}"]
        if item.j1Singularity:
            lines.append("- Singularite Q1")
        if item.j3Singularity:
            lines.append("- Singularite Q3")
        if item.j5Singularity:
            lines.append("- Singularite Q5")
        if item.violated_limits and len(item.violated_limits) > 0:
            lines.append("Axes hors limites :")
            for violated_axis in item.violated_limits:
                min_limit, max_limit = self._axis_limits[violated_axis]
                lines.append(f"- J{violated_axis + 1}: {item.joints[violated_axis]} ({min_limit}, {max_limit})")
        return "\n".join(lines)

    @staticmethod
    def _normalize_joints(values: list[float] | None) -> list[float]:
        normalized = [] if values is None else [float(value) for value in values[:6]]
        while len(normalized) < 6:
            normalized.append(0.0)
        return normalized

    @staticmethod
    def _normalize_weights(values: list[float]) -> list[float]:
        normalized = [float(value) for value in values[:6]]
        while len(normalized) < 6:
            normalized.append(1.0)
        return normalized
