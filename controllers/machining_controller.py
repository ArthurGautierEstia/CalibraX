from __future__ import annotations

from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.types.machining_params import MachiningSimulationParams
from utils.machining_simulator import simulate_machining
from views.machining_view import MachiningView
from widgets.machining_view.machining_actions_widget import MachiningActionsWidget
from widgets.machining_view.machining_graphs_widget import MachiningGraphsWidget
from widgets.machining_view.machining_params_widget import MachiningParamsWidget


class MachiningController(QObject):
    """Contrôleur de l'onglet Usinage.

    Lit la trajectoire simulée depuis ProgramController.current_result au moment
    du clic sur « Simuler » (déclenchement manuel, calcul synchrone).
    """

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        view: MachiningView,
        program_controller,
        parent: QObject = None,
    ) -> None:
        super().__init__(parent)

        self.robot_model = robot_model
        self.tool_model = tool_model
        self.view = view
        self.program_controller = program_controller

        self.params_widget: MachiningParamsWidget = view.get_params_widget()
        self.actions_widget: MachiningActionsWidget = view.get_actions_widget()
        self.graphs_widget: MachiningGraphsWidget = view.get_graphs_widget()

        self._apply_default_params()
        self._setup_connections()

    def _setup_connections(self) -> None:
        self.actions_widget.simulate_requested.connect(self._on_simulate_requested)

    def _apply_default_params(self) -> None:
        """Pré-remplit l'interface avec les valeurs par défaut Al7075 / KR500-3."""
        self.params_widget.set_params(MachiningSimulationParams())

    def _on_simulate_requested(self) -> None:
        program_result = getattr(self.program_controller, "current_result", None)

        if program_result is None or not program_result.nominal_samples:
            self.actions_widget.set_issue_messages([
                "Aucune simulation disponible. "
                "Lancez d'abord une simulation dans l'onglet Programme."
            ])
            self.actions_widget.set_warning_messages([])
            self.actions_widget.set_status_text("")
            self.graphs_widget.clear()
            return

        # Vérification cohérence a_e ≤ D avant de lancer
        params = self.params_widget.get_params()
        warnings_pre: list[str] = []
        if params.cutting.a_e > params.cutting.diameter:
            warnings_pre.append(
                f"Engagement radial a_e ({params.cutting.a_e} mm) > diamètre D "
                f"({params.cutting.diameter} mm) — a_e limité à D."
            )
            from dataclasses import replace
            params = MachiningSimulationParams(
                cutting=replace(params.cutting, a_e=params.cutting.diameter),
                mechanical=params.mechanical,
            )

        try:
            result = simulate_machining(
                program_result,
                params,
                self.robot_model,
                self.tool_model.get_tool(),
            )
        except Exception as exc:
            self.actions_widget.set_issue_messages([f"Erreur de simulation : {exc}"])
            self.actions_widget.set_warning_messages([])
            self.actions_widget.set_status_text("")
            self.graphs_widget.clear()
            return

        self.actions_widget.set_issue_messages([])
        self.actions_widget.set_warning_messages(warnings_pre + result.warnings)
        self.actions_widget.set_status_text(
            f"Simulation terminée — {len(result.samples)} échantillon(s), "
            f"{result.overload_count} dépassement(s) de couple."
        )
        self.graphs_widget.set_result(result)
