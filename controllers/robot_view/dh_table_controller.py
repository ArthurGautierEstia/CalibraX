from PyQt6.QtCore import QObject
from PyQt6.QtCore import pyqtSignal
import os

from models.robot_model import RobotModel
from models.robot_configuration_file import RobotConfigurationFile
from utils.file_io import FileIOHandler
from utils.popup import show_error_popup
from utils.str_utils import str_to_float
from widgets.robot_view.dh_table_widget import DHTableWidget

class DHTableController(QObject):

    configuration_loaded = pyqtSignal()

    def __init__(self, robot_model: RobotModel, dh_table_widget: DHTableWidget, parent: QObject = None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.dh_table_widget = dh_table_widget
        self._setup_connections()
    
    def _setup_connections(self) -> None:
        # Signals from Robot Model
        self.robot_model.configuration_changed.connect(self._on_robot_configuration_changed)
        self.robot_model.robot_name_changed.connect(self._on_robot_name_changed)
        self.robot_model.dh_params_changed.connect(self._on_robot_dh_table_changed)

        # Signals from View
        self.dh_table_widget.text_changed_requested.connect(self._on_view_name_changed)
        self.dh_table_widget.dh_value_changed.connect(self._on_view_dh_value_changed)
        self.dh_table_widget.tool_changed.connect(self._on_view_tool_changed)
        self.dh_table_widget.load_config_requested.connect(self._on_view_load_config_requested)
        self.dh_table_widget.export_config_requested.connect(self._on_view_export_config_requested)

    # ======
    # Connection callbacks
    # ======

    def _on_robot_configuration_changed(self) -> None:
        self.update_robot_name_view()
        self.update_dh_table_view()
        self.update_tool_view()

    def _on_robot_name_changed(self) -> None:
        self.update_robot_name_view()

    def _on_robot_dh_table_changed(self) -> None:
        self.update_dh_table_view()

    def _on_view_name_changed(self) -> None:
        self.robot_model.set_robot_name(self.dh_table_widget.get_robot_name())

    def _on_view_dh_value_changed(self, row: int, col: int, value: str) -> None:
        fval = str_to_float(value)
        self.robot_model.set_dh_param(row, col, fval)
    
    def _on_view_tool_changed(self, tool) -> None:
        self.robot_model.set_tool(tool)
    
    def _on_view_load_config_requested(self) -> None:
        self.load_configuration()

    def _on_view_export_config_requested(self) -> None:
        self.export_configuration()

    # ======
    # Methods
    # ======

    def update_robot_name_view(self) -> None:
        self.dh_table_widget.set_robot_name(self.robot_model.get_robot_name())
    
    def update_dh_table_view(self) -> None:
        self.dh_table_widget.set_dh_params(self.robot_model.get_dh_params())

    def update_tool_view(self) -> None:
        self.dh_table_widget.set_tool(self.robot_model.get_tool())

    def load_configuration(self):
        """Charger une configuration depuis un fichier json"""
        currentDir = os.getcwd()
        configurationDir = os.path.join(currentDir, 'configurations') 

        file_path, data = FileIOHandler.select_and_load_json(
            self.dh_table_widget,
            "Charger une configuration robot",
            configurationDir if os.path.exists(configurationDir) else currentDir
        )

        if data:
            if not isinstance(data, dict):
                show_error_popup("Erreur d'importation", "Le fichier de configuration n'est pas au format adapté. Veuillez vérifier le contenu.")
                return
            
            config = RobotConfigurationFile.from_dict(data)
            self.robot_model.load_from_configuration_file(config, file_path)
            self.configuration_loaded.emit()

    def export_configuration(self):
        """Exporter la configuration actuelle"""
        currentDir = os.getcwd()
        configurationDir = os.path.join(currentDir, 'configurations') 

        config = RobotConfigurationFile.from_robot_model(self.robot_model)
        FileIOHandler.save_json(
            self.dh_table_widget,
            "Exporter/Sauvegarder une configuration robot",
            config.to_dict(),
            configurationDir if os.path.exists(configurationDir) else currentDir
        )
