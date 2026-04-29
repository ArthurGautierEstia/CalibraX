from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from views.cartesian_control_view import CartesianControlView
from controllers.cartesian_control_view.cartesian_widget_controller import CartesianWidgetController
from controllers.cartesian_control_view.mgi_solutions_controller import MgiSolutionsController


class CartesianControlController(QObject):
    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        cartesian_control_view: CartesianControlView,
        parent: QObject = None,
    ):
        super().__init__(parent)

        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model
        self.cartesian_control_view = cartesian_control_view

        self.cartesian_widget_controller = CartesianWidgetController(
            self.robot_model,
            self.workspace_model,
            self.cartesian_control_view.get_cartesian_control_widget(),
        )
        self.mgi_solutions_controller = MgiSolutionsController(self.robot_model, self.cartesian_control_view.get_mgi_solutions_widget())

        self._setup_connections()

    def _setup_connections(self):
        self.cartesian_widget_controller.new_target_computed.connect(self._on_cartesian_new_target_computed)

        # Connexions du solveur MGI Jacobienne (via le widget dédié)
        mgi_widget = self.cartesian_control_view.get_mgi_solutions_widget()
        mgi_widget.jacobien_enabled_changed.connect(self._on_jacobien_enabled_changed)
        mgi_widget.jacobien_params_changed.connect(self._on_jacobien_params_changed)

    def _on_jacobien_enabled_changed(self, enabled: bool):
        """Réinitialise l'affichage de convergence quand le solveur est activé/désactivé."""
        if not enabled:
            self.mgi_solutions_controller.afficher_resultat_jacobien(None)

    def _on_jacobien_params_changed(self):
        """Callback réservé à de futurs besoins (ex. recalcul immédiat)."""
        pass

    def _on_cartesian_new_target_computed(self):
        target = self.cartesian_widget_controller.get_new_target()

        # --- Étape 1 : MGI analytique (rapide, donne l'estimation initiale) ---
        mgi_result = self.robot_model.compute_ik_target(target, tool=self.tool_model.get_tool())
        best_sol = self.robot_model.get_best_mgi_solution(mgi_result)

        if not best_sol:
            # Aucune solution analytique valide — afficher le tableau des solutions
            self.mgi_solutions_controller.display_mgi_result(mgi_result, None)
            return

        _config_key, sol_analytique = best_sol

        mgi_widget = self.cartesian_control_view.get_mgi_solutions_widget()

        if mgi_widget.is_jacobien_enabled():
            # --- Étape 2 : Raffinement par Jacobienne inverse ---
            # Utilise le MGD CORRIGÉ pour tenir compte des corrections de calibration.
            # q_initial = solution analytique pour la configuration sélectionnée.
            params = mgi_widget.get_jacobien_params()
            jacobien_result = self.robot_model.compute_ik_optimise(
                target,
                sol_analytique.joints,
                params,
                tool=self.tool_model.get_tool(),
            )
            # Conserver les joints analytiques pour comparaison dans l'UI
            jacobien_result.joints_analytiques = list(sol_analytique.joints)

            self.robot_model.set_joints(jacobien_result.joints)

            # Mise à jour de l'affichage de convergence
            self.mgi_solutions_controller.afficher_resultat_jacobien(jacobien_result)
        else:
            # --- Mode analytique standard ---
            self.robot_model.set_joints(sol_analytique.joints)
