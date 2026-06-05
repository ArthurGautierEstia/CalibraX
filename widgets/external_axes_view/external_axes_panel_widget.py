"""Panneau principal de l'onglet Axes externes (liste + config + montage robot)."""
from __future__ import annotations

from PyQt6.QtCore import pyqtSignal, QSize
from PyQt6.QtGui import QColor, QIcon, QPainter, QPainterPath, QPixmap
from PyQt6.QtWidgets import (
    QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QListWidget, QListWidgetItem,
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

    axis_added = pyqtSignal(object)          # ExternalAxis
    axis_removed = pyqtSignal(str)           # axis_id
    axis_updated = pyqtSignal(str, object)   # axis_id, ExternalAxis
    robot_mount_parent_changed = pyqtSignal(object)   # str | None
    save_config_requested = pyqtSignal()
    save_as_config_requested = pyqtSignal()
    load_config_requested = pyqtSignal()
    new_config_requested = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._axes: list[ExternalAxis] = []
        self._current_id: str | None = None
        self._updating_ui = False
        self.current_config_name_field: QLineEdit | None = None
        self.status_label: QLabel | None = None
        self._setup_ui()

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _build_save_icon(self, include_pencil: bool = False) -> QIcon:
        size = 22
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.GlobalColor.transparent)

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        color = QColor(self.palette().color(self.palette().ColorRole.ButtonText))
        painter.setPen(color)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        body = QPainterPath()
        body.addRoundedRect(2.0, 2.0, 14.5, 16.5, 1.8, 1.8)
        notch = QPainterPath()
        notch.addRect(4.5, 3.8, 7.2, 3.8)
        label = QPainterPath()
        label.addRect(4.5, 11.2, 8.8, 5.0)
        painter.drawPath(body)
        painter.fillPath(notch, color)
        painter.drawPath(label)

        if include_pencil:
            pencil = QPainterPath()
            pencil.moveTo(13.4, 13.6)
            pencil.lineTo(18.4, 8.6)
            pencil.lineTo(20.0, 10.2)
            pencil.lineTo(15.0, 15.2)
            pencil.closeSubpath()
            tip = QPainterPath()
            tip.moveTo(12.5, 16.1)
            tip.lineTo(13.4, 13.6)
            tip.lineTo(15.0, 15.2)
            tip.closeSubpath()
            painter.fillPath(pencil, color)
            painter.fillPath(tip, color)

        painter.end()
        return QIcon(pixmap)

    def _build_new_icon(self) -> QIcon:
        size = 22
        pixmap = QPixmap(size, size)
        pixmap.fill(Qt.GlobalColor.transparent)

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        color = QColor(self.palette().color(self.palette().ColorRole.ButtonText))
        painter.setPen(color)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        page = QPainterPath()
        page.moveTo(5.0, 2.5)
        page.lineTo(13.0, 2.5)
        page.lineTo(17.5, 7.0)
        page.lineTo(17.5, 19.0)
        page.lineTo(5.0, 19.0)
        page.closeSubpath()
        fold = QPainterPath()
        fold.moveTo(13.0, 2.5)
        fold.lineTo(13.0, 7.0)
        fold.lineTo(17.5, 7.0)
        painter.drawPath(page)
        painter.drawPath(fold)

        painter.end()
        return QIcon(pixmap)

    def _setup_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        top_layout = QVBoxLayout()

        title_row = QHBoxLayout()
        title_label = QLabel("Configuration axe externe")
        title_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        title_row.addWidget(title_label)
        title_row.addStretch()

        self.status_label = QLabel("Configuration non enregistrée")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.status_label.setStyleSheet("color: #808080; font-size: 13px; font-weight: 400;")
        title_row.addWidget(self.status_label)
        top_layout.addLayout(title_row)

        header_layout = QVBoxLayout()

        fields_layout = QGridLayout()
        current_config_title_label = QLabel("Configuration courante :")
        current_config_title_label.setMinimumWidth(150)
        fields_layout.addWidget(current_config_title_label, 0, 0)

        self.current_config_name_field = QLineEdit()
        self.current_config_name_field.setReadOnly(True)
        self.current_config_name_field.setText("Aucune configuration")
        self.current_config_name_field.setMinimumWidth(220)
        fields_layout.addWidget(self.current_config_name_field, 0, 1)

        action_button_size = 36
        action_icon_size = QSize(22, 22)

        self.btn_load = QPushButton("...")
        self.btn_load.setFixedSize(action_button_size, action_button_size)
        self.btn_load.setToolTip("Charger une configuration des axes externes")
        self.btn_load.clicked.connect(self.load_config_requested.emit)
        fields_layout.addWidget(self.btn_load, 0, 2)

        self.btn_new = QPushButton()
        self.btn_new.setIcon(self._build_new_icon())
        self.btn_new.setIconSize(action_icon_size)
        self.btn_new.setFixedSize(action_button_size, action_button_size)
        self.btn_new.setToolTip("Créer une nouvelle configuration des axes externes")
        self.btn_new.clicked.connect(self.new_config_requested.emit)
        fields_layout.addWidget(self.btn_new, 0, 3)

        self.btn_save = QPushButton()
        self.btn_save.setIcon(self._build_save_icon())
        self.btn_save.setIconSize(action_icon_size)
        self.btn_save.setFixedSize(action_button_size, action_button_size)
        self.btn_save.setToolTip("Enregistrer la configuration des axes externes courante")
        self.btn_save.clicked.connect(self.save_config_requested.emit)
        fields_layout.addWidget(self.btn_save, 0, 4)

        self.btn_save_as = QPushButton()
        self.btn_save_as.setIcon(self._build_save_icon(include_pencil=True))
        self.btn_save_as.setIconSize(action_icon_size)
        self.btn_save_as.setFixedSize(action_button_size, action_button_size)
        self.btn_save_as.setToolTip("Enregistrer la configuration des axes externes dans un nouveau fichier JSON")
        self.btn_save_as.clicked.connect(self.save_as_config_requested.emit)
        fields_layout.addWidget(self.btn_save_as, 0, 5)
        fields_layout.setColumnStretch(0, 0)
        fields_layout.setColumnStretch(1, 1)
        fields_layout.setColumnStretch(2, 0)
        fields_layout.setColumnStretch(3, 0)
        fields_layout.setColumnStretch(4, 0)
        fields_layout.setColumnStretch(5, 0)
        header_layout.addLayout(fields_layout)
        top_layout.addLayout(header_layout)
        main_layout.addLayout(top_layout)

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

        btn_add = QPushButton("Ajouter")
        btn_add.setToolTip("Ajouter un axe externe")
        btn_add.clicked.connect(self._show_add_menu)
        btn_remove = QPushButton("Supprimer")
        btn_remove.setToolTip("Supprimer l'axe externe sélectionné")
        btn_remove.clicked.connect(self._remove_selected)
        btn_row_layout.addWidget(btn_add, 1)
        btn_row_layout.addWidget(btn_remove, 1)
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
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)
        splitter.setSizes([333, 667])
        main_layout.addWidget(splitter, 1)

    def set_current_configuration_name(self, file_name: str) -> None:
        if self.current_config_name_field is None:
            return
        self.current_config_name_field.setText(file_name or "Aucune configuration")

    def set_configuration_status(self, text: str, color: str) -> None:
        if self.status_label is None:
            return
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {color}; font-size: 13px; font-weight: 400;")

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
        # Préserver les valeurs articulaires actuelles pendant l'édition de la config.
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
