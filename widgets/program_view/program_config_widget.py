from pathlib import Path

from PyQt6.QtCore import QEvent, Qt, pyqtSignal
from PyQt6.QtGui import QPalette
from PyQt6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit, QVBoxLayout, QWidget


class ProgramConfigWidget(QWidget):
    load_program_requested = pyqtSignal()
    clear_requested = pyqtSignal()

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.btn_load_program = QPushButton("Importer")
        self.btn_clear = QPushButton("Effacer")
        self.lbl_program = QLabel("Aucun programme")
        self.lbl_summary_title = QLabel("Mouvements :")
        self.lbl_summary_value = QLabel("0")
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self._setup_ui()
        self._setup_connections()
        self.log_text.hide()

    def changeEvent(self, event) -> None:
        super().changeEvent(event)
        if event.type() == QEvent.Type.PaletteChange:
            self._apply_current_program_label_style()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        title_label = QLabel("Editer un programme KRL")
        title_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(title_label)

        fields_layout = QGridLayout()
        current_program_title_label = QLabel("Programme courant :")
        current_program_title_label.setMinimumWidth(150)
        fields_layout.addWidget(current_program_title_label, 0, 0)

        self.lbl_program.setWordWrap(True)
        self.lbl_program.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self.lbl_program.setMinimumWidth(220)
        self._apply_current_program_label_style()
        fields_layout.addWidget(self.lbl_program, 0, 1)

        self.lbl_summary_title.setWordWrap(True)
        fields_layout.addWidget(self.lbl_summary_title, 1, 0)
        self.lbl_summary_value.setWordWrap(True)
        self.lbl_summary_value.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        fields_layout.addWidget(self.lbl_summary_value, 1, 1)
        fields_layout.setColumnStretch(0, 0)
        fields_layout.setColumnStretch(1, 1)

        actions_layout = QHBoxLayout()
        actions_layout.addStretch()
        self.btn_load_program.setFixedWidth(120)
        self.btn_clear.setFixedWidth(120)
        actions_layout.addWidget(self.btn_load_program)
        actions_layout.addWidget(self.btn_clear)

        self.log_text.setMinimumHeight(140)
        layout.addLayout(fields_layout)
        layout.addLayout(actions_layout)
        layout.addWidget(self.log_text)

    def _setup_connections(self) -> None:
        self.btn_load_program.clicked.connect(self.load_program_requested.emit)
        self.btn_clear.clicked.connect(self.clear_requested.emit)

    def set_program_info(self, program_path: str, motion_count: int) -> None:
        self.lbl_program.setText(self._format_program_name(program_path))
        self.lbl_summary_value.setText(str(int(motion_count)))

    def set_log_lines(self, lines: list[str]) -> None:
        visible_lines = [line for line in lines if line.strip()]
        self.log_text.setPlainText("\n".join(visible_lines))
        self.log_text.setVisible(bool(visible_lines))

    def _apply_current_program_label_style(self) -> None:
        accent_hex = self.palette().color(QPalette.ColorRole.Highlight).name()
        self.lbl_program.setStyleSheet(
            f"border: 1px solid #555; padding: 2px; background-color: #2a2a2a; color: {accent_hex};"
        )

    @staticmethod
    def _format_program_name(program_path: str) -> str:
        if not program_path:
            return "Aucun programme"
        return Path(program_path).name
