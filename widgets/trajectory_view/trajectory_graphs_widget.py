from typing import Optional

from PyQt6.QtWidgets import QCheckBox, QComboBox, QDialog, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from widgets.trajectory_view.trajectory_config_timeline_widget import TrajectoryConfigTimelineWidget
from widgets.trajectory_view.trajectory_graph_panel_widget import (
    GraphDisplayMode,
    GraphMode,
    TrajectoryGraphPanelWidget,
)


class TrajectoryGraphsWidget(QWidget):
    """Widget holding articular and cartesian trajectory graphs."""

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.articular_panel = TrajectoryGraphPanelWidget(GraphMode.ARTICULAR)
        self.cartesian_panel = TrajectoryGraphPanelWidget(GraphMode.CARTESIAN)
        self.config_timeline = TrajectoryConfigTimelineWidget()
        self.config_timeline.setMinimumHeight(230)

        self.btn_popout = QPushButton("Détacher les graphes")
        self.display_mode_combo = QComboBox()
        self.position_checkbox = QCheckBox("Position")
        self.velocity_checkbox = QCheckBox("Vitesse")
        self.acceleration_checkbox = QCheckBox("Acceleration")
        self.jerk_checkbox = QCheckBox("Jerk")
        self._detachable_panels = QWidget(self)

        self._popout_dialog: Optional[QDialog] = None
        self._dock_layout = None
        self._dock_index: Optional[int] = None

        self._setup_ui()
        self._setup_connections()
        self.articular_panel.set_in_page_mode(True)
        self.cartesian_panel.set_in_page_mode(True)

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        header = QHBoxLayout()
        header.addWidget(QLabel("Affichage"))
        self.display_mode_combo.addItem("Line", GraphDisplayMode.LINE.value)
        self.display_mode_combo.addItem("Dot", GraphDisplayMode.DOT.value)
        header.addWidget(self.display_mode_combo)
        header.addSpacing(12)
        header.addWidget(QLabel("Graphes"))
        self.position_checkbox.setChecked(True)
        self.velocity_checkbox.setChecked(True)
        self.acceleration_checkbox.setChecked(False)
        self.jerk_checkbox.setChecked(False)
        header.addWidget(self.position_checkbox)
        header.addWidget(self.velocity_checkbox)
        header.addWidget(self.acceleration_checkbox)
        header.addWidget(self.jerk_checkbox)
        header.addWidget(self.btn_popout)
        header.addStretch()
        layout.addLayout(header)

        panels = QHBoxLayout(self._detachable_panels)
        panels.setContentsMargins(0, 0, 0, 0)
        panels.addWidget(self.articular_panel, 1)
        panels.addWidget(self.cartesian_panel, 1)

        layout.addWidget(self._detachable_panels)
        layout.addWidget(self.config_timeline)

    def _setup_connections(self) -> None:
        self.btn_popout.clicked.connect(self._on_popout_clicked)
        self.display_mode_combo.currentIndexChanged.connect(self._on_display_mode_changed)
        self.position_checkbox.toggled.connect(self._on_graph_visibility_changed)
        self.velocity_checkbox.toggled.connect(self._on_graph_visibility_changed)
        self.acceleration_checkbox.toggled.connect(self._on_graph_visibility_changed)
        self.jerk_checkbox.toggled.connect(self._on_graph_visibility_changed)

    def _on_display_mode_changed(self, _index: int) -> None:
        selected_mode = self.display_mode_combo.currentData()
        self.articular_panel.set_display_mode(selected_mode)
        self.cartesian_panel.set_display_mode(selected_mode)

    def _on_graph_visibility_changed(self, _checked: bool) -> None:
        self.articular_panel.set_plot_visibility(
            self.position_checkbox.isChecked(),
            self.velocity_checkbox.isChecked(),
            self.acceleration_checkbox.isChecked(),
            self.jerk_checkbox.isChecked(),
        )
        self.cartesian_panel.set_plot_visibility(
            self.position_checkbox.isChecked(),
            self.velocity_checkbox.isChecked(),
            self.acceleration_checkbox.isChecked(),
            self.jerk_checkbox.isChecked(),
        )

    def _on_popout_clicked(self) -> None:
        if self._popout_dialog is None:
            self._pop_out()
        else:
            self._dock_back(close_dialog=True)

    def _pop_out(self) -> None:
        if self._popout_dialog is not None:
            return

        self._dock_layout = self.layout()
        self._dock_index = None

        if self._dock_layout is not None:
            for i in range(self._dock_layout.count()):
                if self._dock_layout.itemAt(i).widget() is self._detachable_panels:
                    self._dock_index = i
                    break
            self._dock_layout.removeWidget(self._detachable_panels)

        self._popout_dialog = QDialog(self)
        self._popout_dialog.setWindowTitle("Graphes de trajectoire")
        dialog_layout = QVBoxLayout(self._popout_dialog)
        dialog_layout.addWidget(self._detachable_panels)
        self._popout_dialog.finished.connect(self._on_popout_closed)
        self._popout_dialog.resize(1300, 760)
        self._popout_dialog.show()

        self.btn_popout.setText("Attacher les graphes")
        self.articular_panel.set_in_page_mode(False)
        self.cartesian_panel.set_in_page_mode(False)

    def _on_popout_closed(self, _result: int) -> None:
        self._dock_back(close_dialog=False)

    def _dock_back(self, close_dialog: bool) -> None:
        dialog = self._popout_dialog
        if dialog is None and close_dialog:
            return

        if dialog is not None:
            dialog.finished.disconnect(self._on_popout_closed)
            if dialog.layout() is not None:
                dialog.layout().removeWidget(self._detachable_panels)
            if close_dialog:
                dialog.close()

        self._popout_dialog = None

        if self._dock_layout is not None:
            if self._dock_index is not None:
                self._dock_layout.insertWidget(self._dock_index, self._detachable_panels)
            else:
                self._dock_layout.addWidget(self._detachable_panels)

        self.btn_popout.setText("Détacher les graphes")
        self.articular_panel.set_in_page_mode(True)
        self.cartesian_panel.set_in_page_mode(True)

    def get_articular_panel(self) -> TrajectoryGraphPanelWidget:
        return self.articular_panel

    def get_cartesian_panel(self) -> TrajectoryGraphPanelWidget:
        return self.cartesian_panel

    def get_configuration_timeline_widget(self) -> TrajectoryConfigTimelineWidget:
        return self.config_timeline
