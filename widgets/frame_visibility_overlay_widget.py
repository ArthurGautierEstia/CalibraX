from PyQt6.QtCore import QSize, Qt, pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class FrameVisibilityOverlayWidget(QWidget):
    frame_clicked = pyqtSignal(int)
    geometry_changed = pyqtSignal()

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._collapsed = False
        self._overlay_width = 180
        self._max_visible_items = 10
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        self.toggle_button = QPushButton(self)
        self.toggle_button.setFixedWidth(self._overlay_width)
        self.toggle_button.setStyleSheet(
            """
            QPushButton {
                background-color: rgba(25, 25, 28, 160);
                color: lightgray;
                border: 1px solid rgba(255, 255, 255, 35);
                border-radius: 6px;
                padding: 6px 10px;
                text-align: left;
            }
            QPushButton:hover {
                background-color: rgba(40, 40, 45, 185);
            }
            """
        )

        self.list_widget = QListWidget(self)
        self.list_widget.setFixedWidth(self._overlay_width)
        self.list_widget.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.list_widget.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.list_widget.setStyleSheet(
            """
            QListWidget {
                background-color: rgba(25, 25, 28, 130);
                color: lightgray;
                border: 1px solid rgba(255, 255, 255, 35);
                border-radius: 6px;
                outline: 0;
            }
            QListWidget::item {
                padding: 6px 8px;
            }
            """
        )

        layout.addWidget(self.toggle_button)
        layout.addWidget(self.list_widget)

        self.toggle_button.clicked.connect(self._on_toggle_clicked)
        self.list_widget.itemClicked.connect(self._on_item_clicked)

        self._refresh_ui()
        self.hide()

    def _on_toggle_clicked(self) -> None:
        self.set_collapsed(not self._collapsed)

    def _on_item_clicked(self, item: QListWidgetItem) -> None:
        index = self.list_widget.row(item)
        if index >= 0:
            self.frame_clicked.emit(index)

    def set_collapsed(self, collapsed: bool) -> None:
        collapsed = bool(collapsed)
        if self._collapsed == collapsed:
            return
        self._collapsed = collapsed
        self._refresh_ui()
        self.geometry_changed.emit()

    def is_collapsed(self) -> bool:
        return self._collapsed

    def set_frames_visibility(self, frames_visibility: list[bool]) -> None:
        count = len(frames_visibility)

        if self.list_widget.count() != count:
            self.list_widget.clear()
            for i in range(count):
                item = QListWidgetItem(f"Frame {i}")
                item.setTextAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
                item.setSizeHint(QSize(0, 28))
                self.list_widget.addItem(item)

        font_bold = QFont()
        font_bold.setBold(True)

        font_normal = QFont()
        font_normal.setBold(False)

        for i, is_visible in enumerate(frames_visibility):
            item = self.list_widget.item(i)
            if item is None:
                continue
            if is_visible:
                item.setFont(font_bold)
                item.setForeground(Qt.GlobalColor.lightGray)
            else:
                item.setFont(font_normal)
                item.setForeground(Qt.GlobalColor.darkGray)

        was_hidden = self.isHidden()
        self._refresh_ui()
        is_hidden = self.isHidden()
        if was_hidden != is_hidden or count > 0:
            self.geometry_changed.emit()

    def _refresh_ui(self) -> None:
        count = self.list_widget.count()
        self.toggle_button.setText(f"Frames ({count}) {'▸' if self._collapsed else '▾'}")

        has_items = count > 0
        list_visible = has_items and not self._collapsed
        self.list_widget.setVisible(list_visible)

        if list_visible:
            row_height = max(28, self.list_widget.sizeHintForRow(0))
            visible_rows = min(count, self._max_visible_items)
            height = (row_height * visible_rows) + (2 * self.list_widget.frameWidth())
            self.list_widget.setFixedHeight(height)

        self.setVisible(has_items)
        self.setFixedWidth(self._overlay_width)
        self.adjustSize()
        self.setFixedHeight(self.sizeHint().height())
