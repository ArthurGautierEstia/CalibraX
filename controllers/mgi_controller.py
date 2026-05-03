from PyQt6.QtCore import QObject

from controllers.cartesian_control_view.mgi_solutions_controller import MgiSolutionsController
from models.robot_model import RobotModel
from utils.mgi import MgiConfigKey, MgiResult
from utils.mgi_jacobien import MgiJacobienParams, MgiJacobienResultat
from views.mgi_view import MgiView


class MgiController(QObject):
    def __init__(
        self,
        robot_model: RobotModel,
        mgi_view: MgiView,
        parent: QObject | None = None,
    ):
        super().__init__(parent)
        self.robot_model = robot_model
        self.mgi_view = mgi_view
        self.mgi_solutions_widget = self.mgi_view.get_mgi_solutions_widget()
        self.mgi_solutions_controller = MgiSolutionsController(self.robot_model, self.mgi_solutions_widget, self)

        self._setup_connections()

    def _setup_connections(self) -> None:
        self.mgi_solutions_widget.jacobien_enabled_changed.connect(self._on_jacobien_enabled_changed)
        self.mgi_solutions_widget.jacobien_params_changed.connect(self._on_jacobien_params_changed)

    def is_jacobien_enabled(self) -> bool:
        return self.mgi_solutions_widget.is_jacobien_enabled()

    def get_jacobien_params(self) -> MgiJacobienParams:
        return self.mgi_solutions_widget.get_jacobien_params()

    def display_mgi_result(self, mgi_result: MgiResult, selected_key: MgiConfigKey | None) -> None:
        self.mgi_solutions_controller.display_mgi_result(mgi_result, selected_key)

    def set_jacobien_resultat(self, resultat: MgiJacobienResultat | None) -> None:
        self.mgi_solutions_controller.afficher_resultat_jacobien(resultat)

    def _on_jacobien_enabled_changed(self, enabled: bool) -> None:
        if not enabled:
            self.set_jacobien_resultat(None)

    def _on_jacobien_params_changed(self) -> None:
        pass
