from PyQt6.QtCore import QTimer, Qt, QSize, QRectF
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QAction, QActionGroup, QCloseEvent, QColor, QIcon, QPainter, QPainterPath, QPixmap
from PyQt6.QtWidgets import QApplication, QMainWindow, QMenu, QSizePolicy, QSplitter, QTabBar, QTabWidget, QVBoxLayout, QWidget

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from views.calibration_view import CalibrationView
from views.camera_view import CameraView
from views.cartesian_control_view import CartesianControlView
from views.external_axes_view import ExternalAxesView
from views.joint_control_view import JointControlView
from views.machining_view import MachiningView
from views.program_view import ProgramView
from views.robot_view import RobotView
from views.tool_view import ToolView
from views.trajectory_view import TrajectoryView
from views.workspace_view import WorkspaceView
from views.workpiece_view import WorkpieceView
from widgets.cartesian_control_view.mgi_solutions_widget import MgiSolutionsWidget
from widgets.program_view.program_playback_widget import ProgramPlaybackWidget
from widgets.viewer_3d_widget import Viewer3DWidget


class MainTabsBar(QTabBar):
    PRIMARY_TAB_COUNT = 5

    def tabSizeHint(self, index: int) -> QSize:
        default_size = super().tabSizeHint(index)
        if index < MainTabsBar.PRIMARY_TAB_COUNT:
            return default_size.expandedTo(QSize(100, 40))
        return default_size


class PersistentCheckMenu(QMenu):
    def mouseReleaseEvent(self, event) -> None:
        action = self.activeAction()
        if action is not None and action.isCheckable() and action.isEnabled():
            action.trigger()
            event.accept()
            return
        super().mouseReleaseEvent(event)


class MainWindow(QMainWindow):
    new_project_requested = pyqtSignal()
    open_project_requested = pyqtSignal()
    open_recent_project_requested = pyqtSignal(str)
    save_project_requested = pyqtSignal()
    save_project_as_requested = pyqtSignal()
    verify_configurations_requested = pyqtSignal()
    fit_scene_view_requested = pyqtSignal()
    manage_viewer_themes_requested = pyqtSignal()
    main_tabs_visibility_changed = pyqtSignal()
    show_keyboard_shortcuts_requested = pyqtSignal()
    show_about_requested = pyqtSignal()
    close_requested = pyqtSignal()

    ROBOT_TAB_INDEX = 0
    TOOL_TAB_INDEX = 1
    EXTERNAL_AXES_TAB_INDEX = 2
    WORKSPACE_TAB_INDEX = 3
    WORKPIECE_TAB_INDEX = 4
    CAMERA_TAB_INDEX = 5

    _VALIDATED_TAB_TOOLTIPS = {
        ROBOT_TAB_INDEX: "Configuration robot chargée et validée",
        TOOL_TAB_INDEX: "Configuration outil chargée et validée",
        EXTERNAL_AXES_TAB_INDEX: "Axes externes configurés et validés",
        WORKSPACE_TAB_INDEX: "Configuration scène chargée et validée",
        WORKPIECE_TAB_INDEX: "Configuration pièce chargée et validée",
        CAMERA_TAB_INDEX: "Configuration caméra chargée et validée",
    }

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        parent: QWidget = None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Calibrax")

        self.tabs = QTabWidget()
        self.tabs.setTabBar(MainTabsBar())
        self.cell_configuration_tabs = QTabWidget()
        self._empty_tab_icon = QIcon()
        self._validated_tab_icon: QIcon | None = None
        self.main_splitter: QSplitter | None = None
        self._initial_splitter_sizes_applied = False
        self._main_splitter_ratio = (1, 1)
        self._maximize_on_first_show = False
        self._was_maximized_before_fullscreen = False
        self._geometry_before_fullscreen = None
        self._recent_project_actions: list[QAction] = []
        self._optional_main_tab_actions: dict[str, QAction] = {}
        self._optional_main_tab_widgets: dict[str, QWidget] = {}

        self.robot_view = RobotView()
        self.tool_view = ToolView()
        self.external_axes_view = ExternalAxesView()
        self.workpiece_view = WorkpieceView()
        self.workspace_view = WorkspaceView()
        self.calibration_view = CalibrationView()
        self.camera_view = CameraView()
        self.joint_control_view = JointControlView()
        self.cartesian_control_view = CartesianControlView()
        self.mgi_solutions_widget = MgiSolutionsWidget()
        self.trajectory_view = TrajectoryView(robot_model, tool_model, workspace_model)
        self.program_view = ProgramView(robot_model, tool_model, workspace_model)
        self.machining_view = MachiningView()

        self.viewer3d = Viewer3DWidget()
        self.viewer_playback_widget = ProgramPlaybackWidget()

        self._setup_ui()

    def show_maximized_on_startup(self) -> None:
        """Affiche la fenetre en mode maximise, sans passer en plein ecran."""
        self._maximize_on_first_show = True
        self.show()

    def _setup_ui(self) -> None:
        self._setup_menu_bar()

        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        self.cell_configuration_tabs.addTab(self.robot_view, "Robot")
        self.cell_configuration_tabs.addTab(self.tool_view, "Tool")
        self.cell_configuration_tabs.addTab(self.external_axes_view, "Axe externe")
        self.cell_configuration_tabs.addTab(self.workspace_view, "Scene")
        self.cell_configuration_tabs.addTab(self.workpiece_view, "Pièce")
        self.cell_configuration_tabs.addTab(self.camera_view, "Camera")

        self.tabs.addTab(self.cell_configuration_tabs, "Configuration cellule")
        self.tabs.addTab(self.calibration_view, "Calibration")
        self.tabs.addTab(self.trajectory_view, "Trajectoire")
        self.tabs.addTab(self.program_view, "Programme")
        self.tabs.addTab(self.machining_view, "Usinage")
        self._optional_main_tab_widgets = {
            "calibration": self.calibration_view,
            "trajectory": self.trajectory_view,
            "program": self.program_view,
            "machining": self.machining_view,
        }

        self.robot_view.add_tab(self.mgi_solutions_widget, "Solutions")
        self.calibration_view.add_tab(self.mgi_solutions_widget.get_jacobien_widget(), "MGI optimisé")

        self.main_splitter = QSplitter(Qt.Orientation.Horizontal, central_widget)
        self.main_splitter.setHandleWidth(6)

        self.tabs.setMinimumWidth(0)
        self.tabs.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Expanding)
        self.viewer3d.setMinimumWidth(0)
        self.viewer3d.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.viewer_playback_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        viewer_column = QWidget(central_widget)
        viewer_column_layout = QVBoxLayout(viewer_column)
        viewer_column_layout.setContentsMargins(0, 0, 0, 0)
        viewer_column_layout.setSpacing(6)
        viewer_column_layout.addWidget(self.viewer3d, 1)
        viewer_column_layout.addWidget(self.viewer_playback_widget, 0)

        self.main_splitter.addWidget(self.tabs)
        self.main_splitter.addWidget(viewer_column)
        self.main_splitter.setStretchFactor(0, 1)
        self.main_splitter.setStretchFactor(1, 1)
        self.main_splitter.setChildrenCollapsible(False)
        self.main_splitter.setCollapsible(0, True)
        self.main_splitter.setCollapsible(1, False)

        layout = QVBoxLayout(central_widget)
        layout.addWidget(self.main_splitter)

    def _setup_menu_bar(self) -> None:
        app = QApplication.instance()
        menu_font = app.font() if app is not None else self.font()
        menu_font.setPointSize(max(8, menu_font.pointSize() - 1))
        self.menuBar().setFont(menu_font)

        project_menu = self.menuBar().addMenu("Projet")
        project_menu.setFont(menu_font)

        self.action_new_project = QAction("Nouveau projet", self)
        self.action_new_project.setShortcut("Ctrl+N")
        self.action_new_project.triggered.connect(self.new_project_requested.emit)
        project_menu.addAction(self.action_new_project)

        self.action_open_project = QAction("Charger projet...", self)
        self.action_open_project.setShortcut("Ctrl+O")
        self.action_open_project.triggered.connect(self.open_project_requested.emit)
        project_menu.addAction(self.action_open_project)

        self.action_save_project = QAction("Enregistrer projet", self)
        self.action_save_project.setShortcut("Ctrl+S")
        self.action_save_project.triggered.connect(self.save_project_requested.emit)
        project_menu.addAction(self.action_save_project)

        self.action_save_project_as = QAction("Enregistrer projet sous...", self)
        self.action_save_project_as.setShortcut("Ctrl+Shift+S")
        self.action_save_project_as.triggered.connect(self.save_project_as_requested.emit)
        project_menu.addAction(self.action_save_project_as)

        project_menu.addSeparator()
        self.recent_projects_menu = project_menu.addMenu("Derniers projets")
        self.recent_projects_menu.setFont(menu_font)
        self.recent_projects_menu.setEnabled(False)
        project_menu.addSeparator()

        self.action_quit = QAction("Quitter", self)
        self.action_quit.setShortcut("Ctrl+Q")
        self.action_quit.triggered.connect(self.close)
        project_menu.addAction(self.action_quit)

        configuration_menu = self.menuBar().addMenu("Configuration")
        configuration_menu.setFont(menu_font)
        self.action_verify_configurations = QAction("Vérifier les configurations", self)
        self.action_verify_configurations.triggered.connect(self.verify_configurations_requested.emit)
        configuration_menu.addAction(self.action_verify_configurations)

        display_menu = self.menuBar().addMenu("Affichage")
        display_menu.setFont(menu_font)
        self.action_toggle_fullscreen = QAction("Plein écran", self)
        self.action_toggle_fullscreen.setShortcut("F11")
        self.action_toggle_fullscreen.triggered.connect(self._toggle_fullscreen)
        display_menu.addAction(self.action_toggle_fullscreen)

        self.action_fit_scene = QAction("Vue isométrique", self)
        self.action_fit_scene.setShortcut("Ctrl+0")
        self.action_fit_scene.triggered.connect(self.fit_scene_view_requested.emit)
        display_menu.addAction(self.action_fit_scene)

        display_menu.addSeparator()
        layout_menu = display_menu.addMenu("Répartition écran")
        layout_menu.setFont(menu_font)
        self.splitter_layout_actions = QActionGroup(self)
        self.splitter_layout_actions.setExclusive(True)
        for label, ratio, checked in (
            ("Onglets 1:1 Viewer", (1, 1), True),
            ("Onglets 1:2 Viewer", (1, 2), False),
            ("Viewer uniquement", (0, 1), False),
        ):
            action = QAction(label, self)
            action.setCheckable(True)
            action.setChecked(checked)
            action.triggered.connect(
                lambda is_checked=False, selected_ratio=ratio: (
                    self._set_main_splitter_ratio(*selected_ratio) if is_checked else None
                )
            )
            self.splitter_layout_actions.addAction(action)
            layout_menu.addAction(action)

        display_menu.addSeparator()
        tabs_menu = PersistentCheckMenu("Onglets principaux", display_menu)
        tabs_menu.setFont(menu_font)
        display_menu.addMenu(tabs_menu)
        for key, label in (
            ("calibration", "Calibration"),
            ("trajectory", "Trajectoire"),
            ("program", "Programme"),
            ("machining", "Usinage"),
        ):
            action = QAction(label, self)
            action.setCheckable(True)
            action.setChecked(True)
            action.toggled.connect(
                lambda checked=False, tab_key=key: self.set_optional_main_tab_visible(tab_key, checked)
            )
            tabs_menu.addAction(action)
            self._optional_main_tab_actions[key] = action

        display_menu.addSeparator()
        self.action_manage_viewer_themes = QAction("Thème du viewer", self)
        self.action_manage_viewer_themes.triggered.connect(self.manage_viewer_themes_requested.emit)
        display_menu.addAction(self.action_manage_viewer_themes)

        help_menu = self.menuBar().addMenu("Aide")
        help_menu.setFont(menu_font)
        self.action_keyboard_shortcuts = QAction("Raccourcis clavier", self)
        self.action_keyboard_shortcuts.triggered.connect(self.show_keyboard_shortcuts_requested.emit)
        help_menu.addAction(self.action_keyboard_shortcuts)

        self.action_about = QAction("À propos de CalibraX", self)
        self.action_about.triggered.connect(self.show_about_requested.emit)
        help_menu.addAction(self.action_about)

    def set_recent_projects(self, project_paths: list[str]) -> None:
        self.recent_projects_menu.clear()
        self._recent_project_actions.clear()
        for path in project_paths[:10]:
            action = QAction(path, self)
            action.triggered.connect(lambda _checked=False, p=path: self.open_recent_project_requested.emit(p))
            self.recent_projects_menu.addAction(action)
            self._recent_project_actions.append(action)
        self.recent_projects_menu.setEnabled(bool(self._recent_project_actions))

    def select_cell_configuration_tab(self, tab_index: int) -> None:
        self.tabs.setCurrentWidget(self.cell_configuration_tabs)
        self.cell_configuration_tabs.setCurrentIndex(tab_index)

    def _toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            if self._was_maximized_before_fullscreen:
                self.showMaximized()
            else:
                self.showNormal()
                if self._geometry_before_fullscreen is not None:
                    self.setGeometry(self._geometry_before_fullscreen)
        else:
            self._was_maximized_before_fullscreen = self.isMaximized()
            self._geometry_before_fullscreen = self.geometry()
            self.showFullScreen()

    def get_optional_main_tabs_visibility(self) -> dict[str, bool]:
        return {
            key: self.tabs.isTabVisible(index)
            for key, widget in self._optional_main_tab_widgets.items()
            if (index := self.tabs.indexOf(widget)) >= 0
        }

    def set_optional_main_tabs_visibility(self, visibility: dict[str, bool]) -> None:
        for key in self._optional_main_tab_widgets:
            if key in visibility:
                self.set_optional_main_tab_visible(key, bool(visibility[key]), emit_changed=False)

    def set_optional_main_tab_visible(self, key: str, visible: bool, emit_changed: bool = True) -> None:
        widget = self._optional_main_tab_widgets.get(key)
        if widget is None:
            return

        tab_index = self.tabs.indexOf(widget)
        if tab_index < 0:
            return

        if self.tabs.isTabVisible(tab_index) == visible:
            self._set_optional_main_tab_action_checked(key, visible)
            return

        self.tabs.setTabVisible(tab_index, visible)
        self._set_optional_main_tab_action_checked(key, visible)
        if not visible and self.tabs.currentWidget() == widget:
            self._select_first_visible_main_tab()
        if emit_changed:
            self.main_tabs_visibility_changed.emit()

    def _set_optional_main_tab_action_checked(self, key: str, checked: bool) -> None:
        action = self._optional_main_tab_actions.get(key)
        if action is None or action.isChecked() == checked:
            return
        previous_block_state = action.blockSignals(True)
        action.setChecked(checked)
        action.blockSignals(previous_block_state)

    def _select_first_visible_main_tab(self) -> None:
        cell_tab_index = self.tabs.indexOf(self.cell_configuration_tabs)
        if cell_tab_index >= 0 and self.tabs.isTabVisible(cell_tab_index):
            self.tabs.setCurrentIndex(cell_tab_index)
            return
        for tab_index in range(self.tabs.count()):
            if self.tabs.isTabVisible(tab_index):
                self.tabs.setCurrentIndex(tab_index)
                return

    def set_robot_tab_validated(self, is_validated: bool) -> None:
        self._set_tab_validated(MainWindow.ROBOT_TAB_INDEX, is_validated)

    def set_tool_tab_validated(self, is_validated: bool) -> None:
        self._set_tab_validated(MainWindow.TOOL_TAB_INDEX, is_validated)

    def set_external_axes_tab_validated(self, is_validated: bool) -> None:
        self._set_tab_validated(MainWindow.EXTERNAL_AXES_TAB_INDEX, is_validated)

    def set_workspace_tab_validated(self, is_validated: bool) -> None:
        self._set_tab_validated(MainWindow.WORKSPACE_TAB_INDEX, is_validated)

    def set_workpiece_tab_validated(self, is_validated: bool) -> None:
        self._set_tab_validated(MainWindow.WORKPIECE_TAB_INDEX, is_validated)

    def set_camera_tab_validated(self, is_validated: bool) -> None:
        self._set_tab_validated(MainWindow.CAMERA_TAB_INDEX, is_validated)

    def _set_tab_validated(self, tab_index: int, is_validated: bool) -> None:
        self.cell_configuration_tabs.setTabIcon(
            tab_index,
            self._get_validated_tab_icon() if is_validated else self._empty_tab_icon,
        )
        tooltip = self._VALIDATED_TAB_TOOLTIPS.get(tab_index, "") if is_validated else ""
        self.cell_configuration_tabs.tabBar().setTabToolTip(tab_index, tooltip)

    def _get_validated_tab_icon(self) -> QIcon:
        if self._validated_tab_icon is None:
            self._validated_tab_icon = self._build_validated_tab_icon()
        return self._validated_tab_icon

    def _build_validated_tab_icon(self) -> QIcon:
        icon_size_px = 18
        pixmap = QPixmap(icon_size_px, icon_size_px)
        pixmap.fill(Qt.GlobalColor.transparent)

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(self.palette().color(self.palette().ColorRole.Highlight)))

        check_path = QPainterPath()
        check_path.moveTo(2.5, 9.5)
        check_path.lineTo(5.2, 6.8)
        check_path.lineTo(8.0, 9.6)
        check_path.lineTo(13.8, 3.8)
        check_path.lineTo(16.0, 6.0)
        check_path.lineTo(8.0, 14.0)
        check_path.closeSubpath()

        bounds = check_path.boundingRect()
        target_bounds = QRectF(1.0, 1.0, icon_size_px - 2.0, icon_size_px - 2.0)
        scale_factor = min(
            target_bounds.width() / bounds.width(),
            target_bounds.height() / bounds.height(),
        )

        painter.translate(target_bounds.center())
        painter.scale(scale_factor, scale_factor)
        painter.translate(-bounds.center())
        painter.drawPath(check_path)
        painter.end()

        return QIcon(pixmap)

    def changeEvent(self, event) -> None:
        super().changeEvent(event)
        if event.type() == event.Type.PaletteChange:
            self._validated_tab_icon = None
            self._refresh_validated_tab_icons()

    def _refresh_validated_tab_icons(self) -> None:
        for tab_index in (
            MainWindow.ROBOT_TAB_INDEX,
            MainWindow.TOOL_TAB_INDEX,
            MainWindow.EXTERNAL_AXES_TAB_INDEX,
            MainWindow.WORKSPACE_TAB_INDEX,
            MainWindow.WORKPIECE_TAB_INDEX,
            MainWindow.CAMERA_TAB_INDEX,
        ):
            current_icon = self.cell_configuration_tabs.tabIcon(tab_index)
            if current_icon.isNull():
                continue
            self.cell_configuration_tabs.setTabIcon(tab_index, self._get_validated_tab_icon())

    def closeEvent(self, event: QCloseEvent) -> None:
        event.ignore()
        self.close_requested.emit()

    def showEvent(self, event) -> None:
        super().showEvent(event)
        if self._maximize_on_first_show:
            self._maximize_on_first_show = False
            QTimer.singleShot(0, self.showMaximized)
        self._apply_initial_splitter_sizes()

    def _apply_initial_splitter_sizes(self) -> None:
        if self._initial_splitter_sizes_applied or self.main_splitter is None:
            return
        self._apply_main_splitter_ratio()
        self._initial_splitter_sizes_applied = True

    def _set_main_splitter_ratio(self, tabs_weight: int, viewer_weight: int) -> None:
        self._main_splitter_ratio = (tabs_weight, viewer_weight)
        self._apply_main_splitter_ratio()

    def _apply_main_splitter_ratio(self) -> None:
        if self.main_splitter is None:
            return
        tabs_weight, viewer_weight = self._main_splitter_ratio
        total_weight = max(1, tabs_weight + viewer_weight)
        total_width = max(2, self.main_splitter.size().width())
        tabs_width = int(total_width * tabs_weight / total_weight)
        viewer_width = total_width - tabs_width
        self.main_splitter.setSizes([tabs_width, viewer_width])

    ####################
    # VIEW GETTERS
    ####################

    def get_robot_view(self) -> RobotView:
        """Retourne la vue de configuration du robot"""
        return self.robot_view

    def get_calibration_view(self) -> CalibrationView:
        """Retourne la vue de calibration du robot"""
        return self.calibration_view

    def get_tool_view(self) -> ToolView:
        """Retourne la vue de configuration du tool."""
        return self.tool_view

    def get_external_axes_view(self) -> ExternalAxesView:
        """Retourne la vue des axes externes."""
        return self.external_axes_view

    def get_workpiece_view(self) -> WorkpieceView:
        """Retourne la vue pièce."""
        return self.workpiece_view

    def get_workspace_view(self) -> WorkspaceView:
        """Retourne la vue workspace."""
        return self.workspace_view

    def get_camera_view(self) -> CameraView:
        """Retourne la vue camera."""
        return self.camera_view

    def get_joint_control_view(self) -> JointControlView:
        """Retourne la vue de controle articulaire"""
        return self.joint_control_view

    def get_cartesian_control_view(self) -> CartesianControlView:
        """Retourne la vue de controle cartesien"""
        return self.cartesian_control_view

    def get_mgi_solutions_widget(self) -> MgiSolutionsWidget:
        """Retourne le widget des solutions MGI."""
        return self.mgi_solutions_widget

    def get_trajectory_view(self) -> TrajectoryView:
        """Retourne la vue de trajectoire"""
        return self.trajectory_view

    def get_viewer3d(self) -> Viewer3DWidget:
        """Retourne la vue du viewer 3D"""
        return self.viewer3d

    def get_viewer_playback_widget(self) -> ProgramPlaybackWidget:
        """Retourne le widget playback positionne sous le viewer 3D."""
        return self.viewer_playback_widget

    def get_program_view(self) -> ProgramView:
        """Retourne la vue programme."""
        return self.program_view

    def get_machining_view(self) -> MachiningView:
        """Retourne la vue de simulation d'usinage."""
        return self.machining_view

    #####################
    # Functions
    #####################

    def update_enabled_tabs(self, robot_has_configuration: bool) -> None:
        """Active ou desactive les onglets de controle en fonction de la configuration du robot"""
        self.tabs.setTabEnabled(self.tabs.indexOf(self.cell_configuration_tabs), True)

        always_enabled_cell_views = (
            self.robot_view,
            self.tool_view,
            self.external_axes_view,
            self.workpiece_view,
            self.camera_view,
        )
        for control_view in always_enabled_cell_views:
            tab_index = self.cell_configuration_tabs.indexOf(control_view)
            if tab_index >= 0:
                self.cell_configuration_tabs.setTabEnabled(tab_index, True)

        workspace_tab_index = self.cell_configuration_tabs.indexOf(self.workspace_view)
        if workspace_tab_index >= 0:
            self.cell_configuration_tabs.setTabEnabled(workspace_tab_index, robot_has_configuration)

        for control_view in (self.calibration_view,):
            tab_index = self.tabs.indexOf(control_view)
            if tab_index >= 0:
                self.tabs.setTabEnabled(tab_index, True)

        for control_view in (self.trajectory_view, self.program_view, self.machining_view):
            tab_index = self.tabs.indexOf(control_view)
            if tab_index >= 0:
                self.tabs.setTabEnabled(tab_index, robot_has_configuration)
