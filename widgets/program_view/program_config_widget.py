from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QLabel, QPushButton, QTextEdit, QVBoxLayout, QWidget


class ProgramConfigWidget(QWidget):
    load_program_requested = pyqtSignal()

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        self.btn_load_program = QPushButton("Importer programme robot")
        self.lbl_program = QLabel("Programme: aucun")
        self.lbl_brand = QLabel("Marque: aucune")
        self.lbl_summary = QLabel("Mouvements: 0")
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self._setup_ui()
        self._setup_connections()
        self.log_text.hide()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        self.lbl_program.setWordWrap(True)
        self.lbl_brand.setWordWrap(True)
        self.lbl_summary.setWordWrap(True)
        self.log_text.setMinimumHeight(140)
        layout.addWidget(self.btn_load_program)
        layout.addWidget(self.lbl_program)
        layout.addWidget(self.lbl_brand)
        layout.addWidget(self.lbl_summary)
        layout.addWidget(self.log_text)

    def _setup_connections(self) -> None:
        self.btn_load_program.clicked.connect(self.load_program_requested.emit)

    def set_program_info(self, program_path: str, brand_text: str, motion_count: int) -> None:
        self.lbl_program.setText(f"Programme: {program_path if program_path else 'aucun'}")
        self.lbl_brand.setText(f"Marque: {brand_text if brand_text else 'aucune'}")
        self.lbl_summary.setText(f"Mouvements: {int(motion_count)}")

    def set_log_lines(self, lines: list[str]) -> None:
        visible_lines = [line for line in lines if line.strip()]
        self.log_text.setPlainText("\n".join(visible_lines))
        self.log_text.setVisible(bool(visible_lines))
