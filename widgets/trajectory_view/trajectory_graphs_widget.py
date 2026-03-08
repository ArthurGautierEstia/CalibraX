from typing import Optional
from PyQt6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QPushButton, QDialog
from widgets.trajectory_view.trajectory_graph_panel_widget import TrajectoryGraphPanelWidget, GraphMode
from widgets.trajectory_view.trajectory_config_timeline_widget import TrajectoryConfigTimelineWidget


class TrajectoryGraphsWidget(QWidget):
    """Widget holding articular and cartesian trajectory graphs."""

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.articular_panel = TrajectoryGraphPanelWidget(GraphMode.ARTICULAR)
        self.cartesian_panel = TrajectoryGraphPanelWidget(GraphMode.CARTESIAN)
        self.config_timeline = TrajectoryConfigTimelineWidget()
        self.config_timeline.setMinimumHeight(230)
        self.btn_popout = QPushButton("Détacher les graphes")

        self._popout_dialog: Optional[QDialog] = None
        self._dock_parent: Optional[QWidget] = None
        self._dock_layout = None
        self._dock_index: Optional[int] = None

        self._setup_ui()
        self._setup_connections()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        header = QHBoxLayout()
        header.addWidget(self.btn_popout)
        header.addStretch()
        layout.addLayout(header)

        panels = QHBoxLayout()
        panels.addWidget(self.articular_panel, 1)
        panels.addWidget(self.cartesian_panel, 1)
        layout.addLayout(panels)
        layout.addWidget(self.config_timeline)

    def _setup_connections(self) -> None:
        self.btn_popout.clicked.connect(self._on_popout_clicked)

    def _on_popout_clicked(self) -> None:
        if self._popout_dialog is None:
            self._pop_out()
        else:
            self._dock_back(close_dialog=True)

    def _pop_out(self) -> None:
        if self._popout_dialog is not None:
            return

        self._dock_parent = self.parentWidget()
        self._dock_layout = self._dock_parent.layout() if self._dock_parent else None
        self._dock_index = None

        if self._dock_layout is not None:
            for i in range(self._dock_layout.count()):
                if self._dock_layout.itemAt(i).widget() is self:
                    self._dock_index = i
                    break
            self._dock_layout.removeWidget(self)

        self._popout_dialog = QDialog(self._dock_parent)
        self._popout_dialog.setWindowTitle("Graphes de trajectoire")
        dialog_layout = QVBoxLayout(self._popout_dialog)
        dialog_layout.addWidget(self)
        self._popout_dialog.finished.connect(self._on_popout_closed)
        self._popout_dialog.resize(1200, 800)
        self._popout_dialog.show()
        self.btn_popout.setText("Attacher les graphes")

    def _on_popout_closed(self, _result: int) -> None:
        self._dock_back(close_dialog=False)

    def _dock_back(self, close_dialog: bool) -> None:
        dialog = self._popout_dialog
        if dialog is None and close_dialog:
            return

        if dialog is not None:
            dialog.finished.disconnect(self._on_popout_closed)
            if dialog.layout() is not None:
                dialog.layout().removeWidget(self)
            if close_dialog:
                dialog.close()

        self._popout_dialog = None

        if self._dock_layout is not None:
            if self._dock_index is not None:
                self._dock_layout.insertWidget(self._dock_index, self)
            else:
                self._dock_layout.addWidget(self)
        elif self._dock_parent is not None:
            self.setParent(self._dock_parent)

        self.btn_popout.setText("Détacher les graphes")

    def get_articular_panel(self) -> TrajectoryGraphPanelWidget:
        return self.articular_panel

    def get_cartesian_panel(self) -> TrajectoryGraphPanelWidget:
        return self.cartesian_panel

    def get_configuration_timeline_widget(self) -> TrajectoryConfigTimelineWidget:
        return self.config_timeline
