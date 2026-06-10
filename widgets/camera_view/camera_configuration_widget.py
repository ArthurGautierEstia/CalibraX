from __future__ import annotations

import os

from PyQt6.QtCore import Qt, QSize, pyqtSignal
from PyQt6.QtGui import QColor, QPalette
from PyQt6.QtWidgets import (
    QCheckBox,
    QColorDialog,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QScrollArea,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from models.camera_model import (
    CameraConfiguration,
    CameraFov,
    CameraStl,
    CameraVisibilityResult,
    CameraVisibilityState,
    CameraVisual,
)
from models.types import Pose6
from utils import math_utils
from utils.config_action_icons import (
    CONFIG_ACTION_BUTTON_SIZE,
    CONFIG_ACTION_ICON_SIZE,
    build_new_icon,
    build_save_icon,
)
from utils.status_badge import apply_status_badge


class CameraDetailWidget(QScrollArea):
    camera_changed = pyqtSignal(object)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWidgetResizable(True)
        self._building = False
        self._camera: CameraConfiguration | None = None
        self._stl_color = "#2B8CBE"
        self._visual_color = "#00AEEF"
        self._pose_spinboxes: dict[str, list[QDoubleSpinBox]] = {}
        self._last_mount_pose_values: list[float] = list(Pose6.zeros().to_tuple())

        container = QWidget()
        self._layout = QVBoxLayout(container)
        self._layout.setSpacing(8)
        self._setup_ui()
        self.setWidget(container)

    def _setup_ui(self) -> None:
        general_group = QGroupBox("General")
        general_layout = QGridLayout(general_group)
        self.id_edit = QLineEdit()
        self.name_edit = QLineEdit()
        self.enabled_checkbox = QCheckBox("Active")
        self.parent_frame_combo = QComboBox()
        self.parent_frame_combo.addItems(["truss", "world"])

        general_layout.addWidget(QLabel("ID"), 0, 0)
        general_layout.addWidget(self.id_edit, 0, 1)
        general_layout.addWidget(QLabel("Nom"), 1, 0)
        general_layout.addWidget(self.name_edit, 1, 1)
        general_layout.addWidget(self.enabled_checkbox, 2, 1)
        general_layout.addWidget(QLabel("Repere parent"), 3, 0)
        general_layout.addWidget(self.parent_frame_combo, 3, 1)
        self._layout.addWidget(general_group)

        self._layout.addWidget(self._build_pose_group("Pose montage truss (edition locale camera)", "mount"))
        self._layout.addWidget(self._build_pose_group("Offset centre optique", "optical"))

        fov_group = QGroupBox("FOV")
        fov_layout = QGridLayout(fov_group)
        self.fov_h_spin = self._build_spinbox(1.0, 179.0, 60.0, 1.0, " deg")
        self.fov_v_spin = self._build_spinbox(1.0, 179.0, 45.0, 1.0, " deg")
        self.range_spin = self._build_spinbox(1.0, 100000.0, 4500.0, 10.0, " mm")
        fov_layout.addWidget(QLabel("Horizontal"), 0, 0)
        fov_layout.addWidget(self.fov_h_spin, 0, 1)
        fov_layout.addWidget(QLabel("Vertical"), 1, 0)
        fov_layout.addWidget(self.fov_v_spin, 1, 1)
        fov_layout.addWidget(QLabel("Portee"), 2, 0)
        fov_layout.addWidget(self.range_spin, 2, 1)
        self._layout.addWidget(fov_group)

        stl_group = QGroupBox("STL camera")
        stl_layout = QGridLayout(stl_group)
        self.stl_path_edit = QLineEdit()
        self.stl_path_edit.setReadOnly(True)
        self.stl_path_edit.setPlaceholderText("Aucun fichier STL")
        self.stl_color_button = QPushButton()
        self.stl_color_button.setFixedSize(26, 26)
        self.stl_color_button.clicked.connect(lambda: self._pick_color("stl"))
        browse_button = QPushButton("Parcourir")
        browse_button.clicked.connect(self._on_browse_stl)
        clear_button = QPushButton("Vider")
        clear_button.clicked.connect(self._on_clear_stl)
        stl_layout.addWidget(QLabel("Fichier"), 0, 0)
        stl_layout.addWidget(self.stl_path_edit, 0, 1)
        stl_layout.addWidget(self.stl_color_button, 0, 2, alignment=Qt.AlignmentFlag.AlignLeft)
        stl_layout.addWidget(browse_button, 0, 3)
        stl_layout.addWidget(clear_button, 0, 4)
        self._layout.addWidget(stl_group)

        visual_group = QGroupBox("Visualisation")
        visual_layout = QGridLayout(visual_group)
        self.show_frustum_checkbox = QCheckBox("Afficher le FOV")
        self.show_line_checkbox = QCheckBox("Afficher ligne TCP")
        self.verify_fov_checkbox = QCheckBox("Verifier TCP dans FOV")
        self.verify_occlusion_checkbox = QCheckBox("Verifier occlusion")
        self.visual_color_button = QPushButton()
        self.visual_color_button.setFixedSize(26, 26)
        self.visual_color_button.clicked.connect(lambda: self._pick_color("visual"))
        visual_layout.addWidget(self.show_frustum_checkbox, 0, 0)
        visual_layout.addWidget(self.verify_fov_checkbox, 0, 1)
        visual_layout.addWidget(self.show_line_checkbox, 1, 0)
        visual_layout.addWidget(self.verify_occlusion_checkbox, 1, 1)
        visual_layout.addWidget(QLabel("Couleur FOV"), 2, 0)
        visual_layout.addWidget(self.visual_color_button, 2, 1, alignment=Qt.AlignmentFlag.AlignLeft)
        self._layout.addWidget(visual_group)
        self._layout.addStretch()

        self._connect_change_signals()

    def _connect_change_signals(self) -> None:
        for line_edit in (self.id_edit, self.name_edit):
            line_edit.editingFinished.connect(self._emit_camera_changed)
        self.enabled_checkbox.toggled.connect(self._emit_camera_changed)
        self.parent_frame_combo.currentIndexChanged.connect(self._emit_camera_changed)
        for index, spinbox in enumerate(self._pose_spinboxes["mount"]):
            spinbox.valueChanged.connect(lambda _value, i=index: self._on_mount_pose_value_changed(i))
        for spinbox in self._pose_spinboxes["optical"]:
            spinbox.valueChanged.connect(self._emit_camera_changed)
        for spinbox in (self.fov_h_spin, self.fov_v_spin, self.range_spin):
            spinbox.valueChanged.connect(self._emit_camera_changed)
        self.show_frustum_checkbox.toggled.connect(self._emit_camera_changed)
        self.show_line_checkbox.toggled.connect(self._emit_camera_changed)
        self.verify_fov_checkbox.toggled.connect(self._emit_camera_changed)
        self.verify_occlusion_checkbox.toggled.connect(self._emit_camera_changed)

    def _build_pose_group(self, title: str, key: str) -> QGroupBox:
        group = QGroupBox(title)
        layout = QGridLayout(group)
        labels = ("X", "Y", "Z", "A", "B", "C")
        suffixes = (" mm", " mm", " mm", " deg", " deg", " deg")
        spinboxes: list[QDoubleSpinBox] = []
        for index, label in enumerate(labels):
            spin = self._build_spinbox(-100000.0, 100000.0, 0.0, 1.0, suffixes[index])
            spinboxes.append(spin)
            row = index // 3
            col = (index % 3) * 2
            layout.addWidget(QLabel(label), row, col)
            layout.addWidget(spin, row, col + 1)
        self._pose_spinboxes[key] = spinboxes
        return group

    @staticmethod
    def _build_spinbox(minimum: float, maximum: float, value: float, step: float, suffix: str) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(float(minimum), float(maximum))
        spin.setValue(float(value))
        spin.setSingleStep(float(step))
        spin.setDecimals(4 if abs(step) < 0.01 else 2)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def set_camera(self, camera: CameraConfiguration | None) -> None:
        self._building = True
        self._camera = camera
        self.setEnabled(camera is not None)
        if camera is None:
            self._building = False
            return

        self._stl_color = camera.stl.color
        self._visual_color = camera.visual.color
        self.id_edit.setText(camera.camera_id)
        self.name_edit.setText(camera.name)
        self.enabled_checkbox.setChecked(camera.enabled)
        self.parent_frame_combo.setCurrentText(camera.parent_frame)
        self._set_pose("mount", camera.mount_pose)
        self._last_mount_pose_values = list(camera.mount_pose.to_tuple())
        self._set_pose("optical", camera.optical_pose)
        self.fov_h_spin.setValue(camera.fov.horizontal_deg)
        self.fov_v_spin.setValue(camera.fov.vertical_deg)
        self.range_spin.setValue(camera.fov.range_mm)
        self.stl_path_edit.setText(camera.stl.path)
        self.show_frustum_checkbox.setChecked(camera.visual.show_frustum)
        self.show_line_checkbox.setChecked(camera.visual.show_line_to_tcp)
        self.verify_fov_checkbox.setChecked(camera.visual.verify_tcp_in_fov)
        self.verify_occlusion_checkbox.setChecked(camera.visual.verify_line_of_sight)
        self._update_color_button(self.stl_color_button, self._stl_color)
        self._update_color_button(self.visual_color_button, self._visual_color)
        self._building = False

    def _set_pose(self, key: str, pose: Pose6) -> None:
        for spin, value in zip(self._pose_spinboxes[key], pose.to_tuple()):
            spin.setValue(float(value))

    def _get_pose(self, key: str) -> Pose6:
        return Pose6.from_values([spin.value() for spin in self._pose_spinboxes[key]])

    def _on_mount_pose_value_changed(self, index: int) -> None:
        if self._building or self._camera is None:
            return
        if not (0 <= index < 6):
            return

        spinboxes = self._pose_spinboxes["mount"]
        edited_value = float(spinboxes[index].value())
        previous_values = list(self._last_mount_pose_values)
        previous_value = float(previous_values[index])
        delta = edited_value - previous_value
        if abs(delta) <= 1e-9:
            return

        previous_pose = Pose6.from_values(previous_values)
        delta_pose_values = [0.0] * 6
        delta_pose_values[index] = delta
        delta_pose = Pose6.from_values(delta_pose_values)

        previous_transform = math_utils.pose_zyx_to_matrix(previous_pose)
        local_delta_transform = math_utils.pose_zyx_to_matrix(delta_pose)
        new_pose = math_utils.matrix_to_pose_zyx(previous_transform @ local_delta_transform)

        self._building = True
        try:
            self._set_pose("mount", new_pose)
            self._last_mount_pose_values = list(new_pose.to_tuple())
        finally:
            self._building = False
        self._emit_camera_changed()

    def get_camera(self) -> CameraConfiguration:
        return CameraConfiguration(
            camera_id=self.id_edit.text().strip(),
            name=self.name_edit.text().strip() or "Camera",
            enabled=self.enabled_checkbox.isChecked(),
            parent_frame=self.parent_frame_combo.currentText().strip() or "truss",
            mount_pose=self._get_pose("mount"),
            optical_pose=self._get_pose("optical"),
            fov=CameraFov(
                horizontal_deg=self.fov_h_spin.value(),
                vertical_deg=self.fov_v_spin.value(),
                range_mm=self.range_spin.value(),
            ),
            stl=CameraStl(
                path=self.stl_path_edit.text().strip(),
                color=self._stl_color,
            ),
            visual=CameraVisual(
                color=self._visual_color,
                show_frustum=self.show_frustum_checkbox.isChecked(),
                show_line_to_tcp=self.show_line_checkbox.isChecked(),
                verify_tcp_in_fov=self.verify_fov_checkbox.isChecked(),
                verify_line_of_sight=self.verify_occlusion_checkbox.isChecked(),
            ),
        )

    def _emit_camera_changed(self, *_args) -> None:
        if self._building or self._camera is None:
            return
        self.camera_changed.emit(self.get_camera())

    def _on_browse_stl(self) -> None:
        selected_path, _ = QFileDialog.getOpenFileName(
            self,
            "Importer STL camera",
            os.getcwd(),
            "STL Files (*.stl);;All Files (*.*)",
        )
        if not selected_path:
            return
        self.stl_path_edit.setText(self._normalize_project_path(selected_path))
        self._emit_camera_changed()

    def _on_clear_stl(self) -> None:
        self.stl_path_edit.clear()
        self._emit_camera_changed()

    def _pick_color(self, target: str) -> None:
        current = QColor(self._stl_color if target == "stl" else self._visual_color)
        selected = QColorDialog.getColor(current, self, "Couleur camera")
        if not selected.isValid():
            return
        if target == "stl":
            self._stl_color = selected.name().upper()
            self._update_color_button(self.stl_color_button, self._stl_color)
        else:
            self._visual_color = selected.name().upper()
            self._update_color_button(self.visual_color_button, self._visual_color)
        self._emit_camera_changed()

    @staticmethod
    def _update_color_button(button: QPushButton, color_hex: str) -> None:
        button.setText("")
        button.setStyleSheet(
            "min-width: 26px; max-width: 26px; min-height: 26px; max-height: 26px; "
            f"border: 1px solid #555; border-radius: 4px; background-color: {color_hex};"
        )

    @staticmethod
    def _normalize_project_path(path: str) -> str:
        absolute_path = os.path.abspath(path)
        project_root = os.path.abspath(os.getcwd())
        try:
            common_path = os.path.commonpath([project_root, absolute_path])
        except ValueError:
            return absolute_path
        if common_path != project_root:
            return absolute_path
        relative_path = os.path.relpath(absolute_path, project_root).replace("\\", "/")
        return f"./{relative_path}" if not relative_path.startswith(".") else relative_path


class CameraListItemWidget(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(3, 3, 3, 3)
        outer_layout.setSpacing(0)

        self.card = QWidget(self)
        self.card.setObjectName("cameraStatusCard")
        card_layout = QHBoxLayout(self.card)
        card_layout.setContentsMargins(8, 5, 8, 5)
        card_layout.setSpacing(8)

        text_layout = QVBoxLayout()
        text_layout.setContentsMargins(0, 0, 0, 0)
        text_layout.setSpacing(1)

        self.name_label = QLabel(self.card)
        self.name_label.setStyleSheet("font-weight: 600;")
        self.status_label = QLabel(self.card)
        self.status_label.setStyleSheet("font-size: 11px;")
        text_layout.addWidget(self.name_label)
        text_layout.addWidget(self.status_label)
        card_layout.addLayout(text_layout, 1)

        self.status_dot = QWidget(self.card)
        self.status_dot.setObjectName("cameraStatusDot")
        self.status_dot.setFixedSize(18, 18)
        card_layout.addWidget(self.status_dot, 0, Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        outer_layout.addWidget(self.card)
        self.card.setAutoFillBackground(True)
        self._set_selected(False)

    def set_data(self, camera: CameraConfiguration, status: str, color: QColor, selected: bool = False) -> None:
        self.name_label.setText(camera.name)
        self.status_label.setText(f"{camera.camera_id} - {status}")
        self._set_selected(selected)
        self.status_dot.setStyleSheet(
            "QWidget#cameraStatusDot {"
            f"background-color: {color.name()};"
            "border: 1px solid #707070;"
            "border-radius: 9px;"
            "}"
        )

    def set_selected(self, selected: bool) -> None:
        self._set_selected(selected)

    def _set_selected(self, selected: bool) -> None:
        palette = self.palette()
        if selected:
            background_role = QPalette.ColorRole.Highlight
            foreground_role = QPalette.ColorRole.HighlightedText
        else:
            background_role = QPalette.ColorRole.Base
            foreground_role = QPalette.ColorRole.Text
        palette.setColor(QPalette.ColorRole.Window, palette.color(background_role))
        self.card.setPalette(palette)
        self.name_label.setPalette(palette)
        self.status_label.setPalette(palette)
        self.name_label.setForegroundRole(foreground_role)
        self.status_label.setForegroundRole(foreground_role)


class CameraConfigurationWidget(QWidget):
    new_config_requested = pyqtSignal()
    load_config_requested = pyqtSignal()
    save_config_requested = pyqtSignal()
    save_as_config_requested = pyqtSignal()
    add_camera_requested = pyqtSignal()
    duplicate_camera_requested = pyqtSignal(int)
    remove_camera_requested = pyqtSignal(int)
    camera_updated = pyqtSignal(int, object)
    selection_changed = pyqtSignal(int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._cameras: list[CameraConfiguration] = []
        self._visibility_results: dict[str, CameraVisibilityResult] = {}
        self._current_id: str | None = None
        self._updating_ui = False
        self._setup_ui()

    def _setup_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(6)

        header_row = QHBoxLayout()
        title_label = QLabel("Configuration cameras")
        title_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        header_row.addWidget(title_label)
        header_row.addStretch()
        header_row.addWidget(QLabel("Statut :"))
        self.status_label = QLabel()
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        apply_status_badge(self.status_label, "Configuration non chargée", "#808080")
        header_row.addWidget(self.status_label)
        main_layout.addLayout(header_row)

        fields_layout = QGridLayout()
        fields_layout.setHorizontalSpacing(8)
        fields_layout.setVerticalSpacing(6)
        current_config_title_label = QLabel("Configuration courante :")
        current_config_title_label.setMinimumWidth(150)
        fields_layout.addWidget(current_config_title_label, 0, 0)
        self.current_config_label = QLabel("Aucune configuration")
        self.current_config_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self.current_config_label.setMinimumWidth(220)
        self._apply_current_config_label_style()
        self.current_config_label.setFixedHeight(self.current_config_label.sizeHint().height())
        fields_layout.addWidget(self.current_config_label, 0, 1, Qt.AlignmentFlag.AlignVCenter)
        fields_layout.setColumnStretch(0, 0)
        fields_layout.setColumnStretch(1, 1)

        action_row = QHBoxLayout()
        action_row.addStretch()
        self.load_button = QPushButton("...")
        self.load_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        self.load_button.setToolTip("Charger une configuration camera")
        self.load_button.clicked.connect(self.load_config_requested.emit)
        action_row.addWidget(self.load_button)
        self.new_button = QPushButton()
        self.new_button.setIcon(build_new_icon(self.palette()))
        self.new_button.setIconSize(CONFIG_ACTION_ICON_SIZE)
        self.new_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        self.new_button.setToolTip("Creer une nouvelle configuration camera")
        self.new_button.clicked.connect(self.new_config_requested.emit)
        action_row.addWidget(self.new_button)
        self.save_button = QPushButton()
        self.save_button.setIcon(build_save_icon(self.palette()))
        self.save_button.setIconSize(CONFIG_ACTION_ICON_SIZE)
        self.save_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        self.save_button.setToolTip("Enregistrer la configuration camera courante")
        self.save_button.clicked.connect(self.save_config_requested.emit)
        action_row.addWidget(self.save_button)
        self.save_as_button = QPushButton()
        self.save_as_button.setIcon(build_save_icon(self.palette(), include_pencil=True))
        self.save_as_button.setIconSize(CONFIG_ACTION_ICON_SIZE)
        self.save_as_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        self.save_as_button.setToolTip("Enregistrer la configuration camera dans un nouveau fichier JSON")
        self.save_as_button.clicked.connect(self.save_as_config_requested.emit)
        action_row.addWidget(self.save_as_button)
        fields_layout.addLayout(action_row, 1, 0, 1, 2)
        main_layout.addLayout(fields_layout)

        splitter = QSplitter(Qt.Orientation.Horizontal)

        left = QWidget()
        left.setMinimumWidth(190)
        left.setMaximumWidth(240)
        left_layout = QVBoxLayout(left)
        left_layout.setSpacing(6)
        left_layout.addWidget(QLabel("<b>Cameras</b>"))
        self.list_widget = QListWidget()
        self.list_widget.setSpacing(2)
        self.list_widget.currentRowChanged.connect(self._on_selection_changed)
        left_layout.addWidget(self.list_widget, 1)

        list_buttons = QHBoxLayout()
        self.add_button = QPushButton("+ Ajouter")
        self.add_button.clicked.connect(self.add_camera_requested.emit)
        list_buttons.addWidget(self.add_button)
        self.duplicate_button = QPushButton("Dupliquer")
        self.duplicate_button.clicked.connect(self._on_duplicate_clicked)
        list_buttons.addWidget(self.duplicate_button)
        self.remove_button = QPushButton("- Supprimer")
        self.remove_button.clicked.connect(self._on_remove_clicked)
        list_buttons.addWidget(self.remove_button)
        left_layout.addLayout(list_buttons)

        self.detail_widget = CameraDetailWidget()
        self.detail_widget.setEnabled(False)
        self.detail_widget.camera_changed.connect(self._on_detail_camera_changed)

        splitter.addWidget(left)
        splitter.addWidget(self.detail_widget)
        splitter.setSizes([220, 660])
        main_layout.addWidget(splitter, 1)

    def set_configuration_status(self, text: str, color: str = "#808080") -> None:
        apply_status_badge(self.status_label, text, color)

    def changeEvent(self, event) -> None:  # type: ignore[override]
        super().changeEvent(event)
        if event.type() == event.Type.PaletteChange:
            self._apply_current_config_label_style()

    def _apply_current_config_label_style(self) -> None:
        accent = self.palette().color(QPalette.ColorRole.Highlight).name()
        self.current_config_label.setStyleSheet(
            f"border: 1px solid #555; padding: 2px; background-color: #2a2a2a; color: {accent};"
        )

    def set_current_file_path(self, file_path: str) -> None:
        normalized = str(file_path or "").strip()
        self.set_current_configuration_name(os.path.basename(normalized) if normalized else "")

    def set_current_configuration_name(self, configuration_name: str) -> None:
        name = str(configuration_name or "").strip()
        self.current_config_label.setText(name or "Aucune configuration")

    def set_cameras(self, cameras: list[CameraConfiguration]) -> None:
        previous_id = self._current_id
        self._cameras = list(cameras)
        self._updating_ui = True
        self.list_widget.clear()
        new_row = -1
        for row, camera in enumerate(self._cameras):
            item = QListWidgetItem()
            item.setSizeHint(QSize(0, 54))
            self._apply_item_status(item, camera)
            self.list_widget.addItem(item)
            self.list_widget.setItemWidget(item, self._build_camera_item_widget(camera))
            if camera.camera_id == previous_id:
                new_row = row
        self._updating_ui = False

        if new_row >= 0:
            self.list_widget.setCurrentRow(new_row)
        elif self._cameras:
            self.list_widget.setCurrentRow(0)
        else:
            self._current_id = None
            self.detail_widget.set_camera(None)
        self._refresh_selection_indicators()

    def set_visibility_results(self, results: dict[str, CameraVisibilityResult]) -> None:
        self._visibility_results = dict(results)
        for row, camera in enumerate(self._cameras):
            item = self.list_widget.item(row)
            if item is None:
                continue
            self._apply_item_status(item, camera)
            widget = self.list_widget.itemWidget(item)
            if isinstance(widget, CameraListItemWidget):
                status, color = self._status_text_and_color(self._visibility_results.get(camera.camera_id))
                widget.set_data(camera, status, color, row == self.list_widget.currentRow())

    def selected_camera_index(self) -> int:
        row = self.list_widget.currentRow()
        return row if 0 <= row < len(self._cameras) else -1

    def _camera_list_label(self, camera: CameraConfiguration) -> str:
        result = self._visibility_results.get(camera.camera_id)
        status, _color = self._status_text_and_color(result)
        return f"{camera.name}\n{camera.camera_id} - {status}"

    def _build_camera_item_widget(self, camera: CameraConfiguration) -> CameraListItemWidget:
        status, color = self._status_text_and_color(self._visibility_results.get(camera.camera_id))
        widget = CameraListItemWidget(self.list_widget)
        widget.set_data(camera, status, color, False)
        widget.setToolTip(
            f"FOV {camera.fov.horizontal_deg:.1f} x {camera.fov.vertical_deg:.1f} deg / {camera.fov.range_mm:.0f} mm\n"
            f"STL: {camera.stl.path or '-'}"
        )
        return widget

    def _apply_item_status(self, item: QListWidgetItem, camera: CameraConfiguration) -> None:
        item.setText("")
        item.setToolTip(
            f"FOV {camera.fov.horizontal_deg:.1f} x {camera.fov.vertical_deg:.1f} deg / {camera.fov.range_mm:.0f} mm\n"
            f"STL: {camera.stl.path or '-'}"
        )

    @staticmethod
    def _status_text_and_color(result: CameraVisibilityResult | None):
        if result is None:
            return "-", QColor("#505050")
        if result.state == CameraVisibilityState.VISIBLE:
            return "TCP visible", QColor("#198754")
        if result.state == CameraVisibilityState.OCCLUDED:
            name = f" ({result.occluder_name})" if result.occluder_name else ""
            return f"Occlusion{name}", QColor("#DC3545")
        if result.state == CameraVisibilityState.OUT_OF_FOV:
            return "Hors FOV", QColor("#FD7E14")
        if result.state == CameraVisibilityState.OUT_OF_RANGE:
            return "Hors portee", QColor("#FD7E14")
        if result.state == CameraVisibilityState.DISABLED:
            return "Desactivee", QColor("#6C757D")
        return "Invalide", QColor("#842029")

    def _on_selection_changed(self, row: int) -> None:
        if self._updating_ui:
            return
        if row < 0 or row >= len(self._cameras):
            self._current_id = None
            self.detail_widget.set_camera(None)
            self._refresh_selection_indicators()
            self.selection_changed.emit(-1)
            return
        camera = self._cameras[row]
        self._current_id = camera.camera_id
        self.detail_widget.set_camera(camera)
        self._refresh_selection_indicators()
        self.selection_changed.emit(row)

    def _refresh_selection_indicators(self) -> None:
        current_row = self.list_widget.currentRow()
        for row in range(self.list_widget.count()):
            item = self.list_widget.item(row)
            widget = self.list_widget.itemWidget(item)
            if isinstance(widget, CameraListItemWidget):
                widget.set_selected(row == current_row)

    def _on_detail_camera_changed(self, camera: CameraConfiguration) -> None:
        index = self.selected_camera_index()
        if index >= 0 and not self._updating_ui:
            self.camera_updated.emit(index, camera)

    def _on_duplicate_clicked(self) -> None:
        index = self.selected_camera_index()
        if index >= 0:
            self.duplicate_camera_requested.emit(index)

    def _on_remove_clicked(self) -> None:
        index = self.selected_camera_index()
        if index >= 0:
            self.remove_camera_requested.emit(index)
