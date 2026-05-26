"""Panneau principal de l'onglet Axes externes (liste + config + montage robot)."""
from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox, QGroupBox, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QMenu, QPushButton, QSplitter, QVBoxLayout, QWidget,
)
from PyQt6.QtCore import Qt

from models.external_axis import ExternalAxis
from models.external_axes_model import ExternalAxesModel
from widgets.external_axes_view.external_axis_config_widget import ExternalAxisConfigWidget


class ExternalAxesPanelWidget(QWidget):
    """Widget complet de l'onglet Axes externes.

    Il communique avec le contrôleur via les signaux :
    - axis_added(ExternalAxis)
    - axis_removed(str id)
    - axis_updated(str id, ExternalAxis)
    - robot_mount_parent_changed(str | None)
    """

    axis_added = pyqtSignal(object)       # ExternalAxis
    axis_removed = pyqtSignal(str)        # axis_id
    axis_updated = pyqtSignal(str, object) # axis_id, ExternalAxis
    robot_mount_parent_changed = pyqtSignal(object)  # str | None

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._axes: list[ExternalAxis] = []
        self._current_id: str | None = None
        self._updating_ui = False
        self._setup_ui()

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # ── Panneau gauche : liste + contrôles ────────────────────────
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setSpacing(6)

        left_layout.addWidget(QLabel("<b>Axes externes</b>"))

        self._list_widget = QListWidget()
        self._list_widget.currentRowChanged.connect(self._on_selection_changed)
        left_layout.addWidget(self._list_widget)

        btn_row = QWidget()
        btn_row_layout = QHBoxLayout(btn_row)
        btn_row_layout.setContentsMargins(0, 0, 0, 0)

        btn_add = QPushButton("+ Ajouter")
        btn_add.clicked.connect(self._show_add_menu)
        btn_remove = QPushButton("- Supprimer")
        btn_remove.clicked.connect(self._remove_selected)
        btn_row_layout.addWidget(btn_add)
        btn_row_layout.addWidget(btn_remove)
        left_layout.addWidget(btn_row)

        # Section montage robot
        robot_box = QGroupBox("Montage robot")
        robot_layout = QHBoxLayout(robot_box)
        robot_layout.addWidget(QLabel("Base robot sur :"))
        self._robot_mount_combo = QComboBox()
        self._robot_mount_combo.addItem("Monde (libre)", None)
        self._robot_mount_combo.currentIndexChanged.connect(self._on_robot_mount_changed)
        robot_layout.addWidget(self._robot_mount_combo)
        left_layout.addWidget(robot_box)

        # ── Panneau droit : configuration de l'axe sélectionné ────────
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setSpacing(0)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self._config_widget = ExternalAxisConfigWidget()
        self._config_widget.setEnabled(False)
        self._config_widget.axis_changed.connect(self._on_axis_config_changed)
        right_layout.addWidget(self._config_widget)

        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setSizes([250, 500])

        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.addWidget(splitter)

    # ------------------------------------------------------------------
    # Ajout rapide via menu
    # ------------------------------------------------------------------

    def _show_add_menu(self) -> None:
        menu = QMenu(self)
        menu.addAction("Rail linéaire X", lambda: self._add_template(ExternalAxis.make_linear_rail()))
        menu.addAction("Positionneur 1 axe (rotation Z)", lambda: self._add_template(ExternalAxis.make_rotary_1axis()))
        menu.addAction("Positionneur 2 axes", lambda: self._add_template(ExternalAxis.make_rotary_2axis()))
        menu.addSeparator()
        menu.addAction("Axe personnalisé (vide)", lambda: self._add_template(ExternalAxis()))
        btn = self.sender()
        if btn:
            menu.exec(btn.mapToGlobal(btn.rect().bottomLeft()))

    def _add_template(self, axis: ExternalAxis) -> None:
        self.axis_added.emit(axis)

    def _remove_selected(self) -> None:
        if self._current_id is not None:
            self.axis_removed.emit(self._current_id)

    # ------------------------------------------------------------------
    # Sélection
    # ------------------------------------------------------------------

    def _on_selection_changed(self, row: int) -> None:
        if self._updating_ui:
            return
        if row < 0 or row >= len(self._axes):
            self._current_id = None
            self._config_widget.setEnabled(False)
            return
        axis = self._axes[row]
        self._current_id = axis.id
        self._config_widget.setEnabled(True)
        self._config_widget.set_axis(axis, self._axes)

    # ------------------------------------------------------------------
    # Config modifiée → signal vers contrôleur
    # ------------------------------------------------------------------

    def _on_axis_config_changed(self) -> None:
        if self._current_id is None or self._updating_ui:
            return
        axis = self._config_widget.get_axis(self._current_id)
        # Préserver les valeurs articulaires actuelles (modifiées par le jog)
        # pour ne pas réinitialiser la position en cours d'édition de la config.
        for existing in self._axes:
            if existing.id == self._current_id:
                for new_j, old_j in zip(axis.joints, existing.joints):
                    new_j.value = old_j.value
                break
        self.axis_updated.emit(self._current_id, axis)

    # ------------------------------------------------------------------
    # Montage robot
    # ------------------------------------------------------------------

    def _on_robot_mount_changed(self, _index: int) -> None:
        if self._updating_ui:
            return
        parent_id = self._robot_mount_combo.currentData()
        self.robot_mount_parent_changed.emit(parent_id)

    # ------------------------------------------------------------------
    # Mise à jour depuis le modèle (appelée par le contrôleur)
    # ------------------------------------------------------------------

    def refresh_from_model(self, model: ExternalAxesModel) -> None:
        self._updating_ui = True
        axes = model.get_axes()
        self._axes = axes

        # Reconstruire la liste
        previous_id = self._current_id
        self._list_widget.clear()
        new_row = -1
        for i, a in enumerate(axes):
            label = f"{a.name}  ({len(a.joints)} axe{'s' if len(a.joints) > 1 else ''})"
            self._list_widget.addItem(QListWidgetItem(label))
            if a.id == previous_id:
                new_row = i

        # Mettre à jour le combo montage robot
        current_mount = model.get_robot_mount_parent_id()
        self._robot_mount_combo.blockSignals(True)
        self._robot_mount_combo.clear()
        self._robot_mount_combo.addItem("Monde (libre)", None)
        for a in axes:
            self._robot_mount_combo.addItem(a.name, a.id)
        for i in range(self._robot_mount_combo.count()):
            if self._robot_mount_combo.itemData(i) == current_mount:
                self._robot_mount_combo.setCurrentIndex(i)
                break
        self._robot_mount_combo.blockSignals(False)

        self._updating_ui = False

        # Restaurer la sélection
        if new_row >= 0:
            self._list_widget.setCurrentRow(new_row)
        elif axes:
            self._list_widget.setCurrentRow(0)
        else:
            self._current_id = None
            self._config_widget.setEnabled(False)

    def refresh_config_panel(self, model: ExternalAxesModel) -> None:
        """Met à jour uniquement le panneau de config (sans toucher à la liste)."""
        if self._current_id is None:
            return
        axes = model.get_axes()
        self._axes = axes
        for a in axes:
            if a.id == self._current_id:
                self._updating_ui = True
                self._config_widget.set_axis(a, axes)
                self._updating_ui = False
                return
