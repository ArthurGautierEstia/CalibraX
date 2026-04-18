from pathlib import Path
from typing import List

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QPlainTextEdit,
    QTabWidget,
    QFileDialog,
)
from PyQt6.QtCore import Qt


class OptimizedWidget(QWidget):
    """Widget pour importer et visualiser les fichiers de données d'optimisation DH."""

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.robot_file_path: str = ""
        self.tracker_file_paths: List[str] = []
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(5)

        title = QLabel("Optimisation DH")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(title)

        import_layout = QHBoxLayout()
        self.btn_import_robot = QPushButton("Importer coordonnées robot")
        self.btn_import_robot.clicked.connect(self._import_robot_coordinates)
        self.btn_import_tracker = QPushButton("Importer mesures laser tracker")
        self.btn_import_tracker.clicked.connect(self._import_tracker_measurements)
        import_layout.addWidget(self.btn_import_robot)
        import_layout.addWidget(self.btn_import_tracker)
        layout.addLayout(import_layout)

        self.robot_label = QLabel("Fichier robot : aucun fichier chargé")
        self.robot_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self.robot_label)

        self.robot_content = QPlainTextEdit()
        self.robot_content.setReadOnly(True)
        self.robot_content.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.robot_content.setPlaceholderText("Contenu du fichier de coordonnées robot affiché ici")
        layout.addWidget(self.robot_content, 1)

        self.tracker_label = QLabel("Fichiers laser tracker : aucun fichier chargé")
        self.tracker_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self.tracker_label)

        self.tracker_tabs = QTabWidget()
        layout.addWidget(self.tracker_tabs, 2)

        self.setLayout(layout)

    def _import_robot_coordinates(self) -> None:
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Importer coordonnées robot",
            "",
            "Fichiers CSV (*.csv);;Tous les fichiers (*.*)",
        )
        if not file_path:
            return

        self.robot_file_path = file_path
        self.robot_label.setText(f"Fichier robot : {Path(file_path).name}")
        self.robot_content.setPlainText(self._load_file_content(file_path))

    def _import_tracker_measurements(self) -> None:
        file_paths, _ = QFileDialog.getOpenFileNames(
            self,
            "Importer mesures laser tracker",
            "",
            "Fichiers CSV (*.csv);;Tous les fichiers (*.*)",
        )
        if not file_paths:
            return

        self.tracker_file_paths = file_paths[:4]
        self.tracker_label.setText(
            f"Fichiers laser tracker : {len(self.tracker_file_paths)} fichier(s) chargés"
        )
        self._populate_tracker_tabs(self.tracker_file_paths)

    def _populate_tracker_tabs(self, file_paths: List[str]) -> None:
        self.tracker_tabs.clear()
        for index, file_path in enumerate(file_paths, start=1):
            file_name = Path(file_path).name
            tab = QWidget()
            tab_layout = QVBoxLayout(tab)
            text_edit = QPlainTextEdit()
            text_edit.setReadOnly(True)
            text_edit.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
            text_edit.setPlainText(self._load_file_content(file_path))
            tab_layout.addWidget(text_edit)
            tab.setLayout(tab_layout)
            self.tracker_tabs.addTab(tab, f"Mesure {index}: {file_name}")

    @staticmethod
    def _load_file_content(file_path: str) -> str:
        try:
            with open(file_path, "r", encoding="utf-8") as file:
                return file.read()
        except Exception as error:
            return f"Erreur lors de la lecture du fichier : {error}"
