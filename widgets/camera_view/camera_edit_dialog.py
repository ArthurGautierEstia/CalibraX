from __future__ import annotations

import os

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QCheckBox,
    QColorDialog,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QDoubleSpinBox,
    QVBoxLayout,
    QWidget,
)

from models.camera_model import (
    CameraConfiguration,
    CameraFov,
    CameraStl,
    CameraVisual,
)
from models.types import Pose6


class CameraEditDialog(QDialog):
    def __init__(self, camera: CameraConfiguration, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Edition camera")
        self._camera = camera
        self._stl_color = camera.stl.color
        self._visual_color = camera.visual.color
        self._pose_spinboxes: dict[str, list[QDoubleSpinBox]] = {}
        self._setup_ui()
        self._load_camera(camera)

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

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
        layout.addWidget(general_group)

        layout.addWidget(self._build_pose_group("Pose montage truss", "mount"))
        layout.addWidget(self._build_pose_group("Offset centre optique", "optical"))

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
        layout.addWidget(fov_group)

        stl_group = QGroupBox("STL camera")
        stl_layout = QGridLayout(stl_group)
        self.stl_path_edit = QLineEdit()
        browse_button = QPushButton("Parcourir")
        browse_button.clicked.connect(self._on_browse_stl)
        self.stl_color_button = QPushButton()
        self.stl_color_button.setFixedSize(26, 26)
        self.stl_color_button.clicked.connect(lambda: self._pick_color("stl"))
        stl_layout.addWidget(QLabel("Fichier"), 0, 0)
        stl_layout.addWidget(self.stl_path_edit, 0, 1)
        stl_layout.addWidget(self.stl_color_button, 0, 2, alignment=Qt.AlignmentFlag.AlignLeft)
        stl_layout.addWidget(browse_button, 0, 3)
        layout.addWidget(stl_group)

        visual_group = QGroupBox("Visualisation")
        visual_layout = QGridLayout(visual_group)
        self.show_frustum_checkbox = QCheckBox("Afficher le FOV")
        self.show_line_checkbox = QCheckBox("Afficher ligne TCP")
        self.visual_color_button = QPushButton()
        self.visual_color_button.setFixedSize(26, 26)
        self.visual_color_button.clicked.connect(lambda: self._pick_color("visual"))
        visual_layout.addWidget(self.show_frustum_checkbox, 0, 1)
        visual_layout.addWidget(self.show_line_checkbox, 1, 1)
        visual_layout.addWidget(QLabel("Couleur FOV"), 2, 0)
        visual_layout.addWidget(self.visual_color_button, 2, 1, alignment=Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(visual_group)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

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
    def _build_spinbox(
        minimum: float,
        maximum: float,
        value: float,
        step: float,
        suffix: str,
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(float(minimum), float(maximum))
        spin.setValue(float(value))
        spin.setSingleStep(float(step))
        spin.setDecimals(4 if abs(step) < 0.01 else 2)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    def _load_camera(self, camera: CameraConfiguration) -> None:
        self.id_edit.setText(camera.camera_id)
        self.name_edit.setText(camera.name)
        self.enabled_checkbox.setChecked(camera.enabled)
        self.parent_frame_combo.setCurrentText(camera.parent_frame)
        self._set_pose("mount", camera.mount_pose)
        self._set_pose("optical", camera.optical_pose)
        self.fov_h_spin.setValue(camera.fov.horizontal_deg)
        self.fov_v_spin.setValue(camera.fov.vertical_deg)
        self.range_spin.setValue(camera.fov.range_mm)
        self.stl_path_edit.setText(camera.stl.path)
        self.show_frustum_checkbox.setChecked(camera.visual.show_frustum)
        self.show_line_checkbox.setChecked(camera.visual.show_line_to_tcp)
        self._update_color_button(self.stl_color_button, self._stl_color)
        self._update_color_button(self.visual_color_button, self._visual_color)

    def _set_pose(self, key: str, pose: Pose6) -> None:
        for spin, value in zip(self._pose_spinboxes[key], pose.to_tuple()):
            spin.setValue(float(value))

    def _get_pose(self, key: str) -> Pose6:
        return Pose6.from_values([spin.value() for spin in self._pose_spinboxes[key]])

    def get_camera(self) -> CameraConfiguration:
        return CameraConfiguration(
            camera_id=self.id_edit.text().strip(),
            name=self.name_edit.text().strip(),
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
            ),
        )

    def _on_browse_stl(self) -> None:
        start_dir = os.getcwd()
        selected_path, _ = QFileDialog.getOpenFileName(
            self,
            "Importer STL camera",
            start_dir,
            "STL Files (*.stl);;All Files (*.*)",
        )
        if selected_path:
            self.stl_path_edit.setText(self._normalize_project_path(selected_path))

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
