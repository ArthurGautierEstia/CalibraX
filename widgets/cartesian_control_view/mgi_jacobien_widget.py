"""
Widget de contrôle et d'affichage du solveur MGI Jacobienne (Levenberg-Marquardt).

Permet à l'utilisateur :
- D'activer / désactiver le solveur MGI optimisé
- De configurer les paramètres du solveur
- De visualiser les métriques de convergence en temps réel
- De comparer les joints analytiques vs Jacobienne
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox,
    QCheckBox, QLabel, QSpinBox, QDoubleSpinBox,
    QFormLayout, QFrame, QTableWidget, QTableWidgetItem,
    QAbstractItemView, QHeaderView
)
from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtGui import QColor, QBrush

from utils.mgi_jacobien import MgiJacobienParams, MgiJacobienResultat


# ============================================================================
# Helpers visuels
# ============================================================================

def _make_value_label(text: str = "—") -> QLabel:
    lbl = QLabel(text)
    lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    lbl.setStyleSheet("font-family: monospace;")
    return lbl


def _separator() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.Shape.HLine)
    line.setFrameShadow(QFrame.Shadow.Sunken)
    return line


_COLOR_OK = "#37CA4B"
_COLOR_WARN = "#EEAD22"
_COLOR_GRAY = "gray"

# Seuil visuel pour mettre en évidence un écart articulaire significatif (degrés)
_SEUIL_DIFF_DEG = 0.01


# ============================================================================
# Widget principal
# ============================================================================

class MgiJacobienWidget(QWidget):
    """
    Widget de configuration et de monitoring du solveur MGI Jacobienne.

    Signaux :
        enabled_changed(bool): Émis quand l'utilisateur active/désactive le solveur
        params_changed():      Émis quand un paramètre est modifié
    """

    enabled_changed = pyqtSignal(bool)
    params_changed = pyqtSignal()

    def __init__(self, parent: QWidget = None):
        super().__init__(parent)

        self._building = True

        layout = QVBoxLayout(self)
        layout.setSpacing(8)

        # --- Checkbox d'activation ---
        self._checkbox_enabled = QCheckBox("Activer le MGI optimisé (Jacobienne inverse)")
        self._checkbox_enabled.setChecked(False)
        layout.addWidget(self._checkbox_enabled)

        layout.addWidget(_separator())

        # --- Groupe paramètres ---
        self._group_params = QGroupBox("Paramètres du solveur")
        layout.addWidget(self._group_params)
        self._init_params_group()

        layout.addWidget(_separator())

        # --- Groupe convergence ---
        self._group_convergence = QGroupBox("Dernière convergence")
        layout.addWidget(self._group_convergence)
        self._init_convergence_group()

        layout.addWidget(_separator())

        # --- Tableau de comparaison joints ---
        self._group_comparaison = QGroupBox("Comparaison joints (analytique vs Jacobienne)")
        layout.addWidget(self._group_comparaison)
        self._init_comparaison_group()

        layout.addStretch()

        # Connexions
        self._checkbox_enabled.toggled.connect(self._on_enabled_toggled)
        self._spinbox_max_iter.valueChanged.connect(self._on_param_changed)
        self._spinbox_seuil_pos.valueChanged.connect(self._on_param_changed)
        self._spinbox_seuil_ori.valueChanged.connect(self._on_param_changed)
        self._spinbox_epsilon.valueChanged.connect(self._on_param_changed)
        self._spinbox_lambda.valueChanged.connect(self._on_param_changed)

        self._building = False
        self._update_params_enabled()

    # --------------------------------------------------------
    # Initialisation des sous-groupes
    # --------------------------------------------------------

    def _init_params_group(self):
        form = QFormLayout(self._group_params)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignLeft)

        self._spinbox_max_iter = QSpinBox()
        self._spinbox_max_iter.setRange(1, 100)
        self._spinbox_max_iter.setValue(20)
        self._spinbox_max_iter.setKeyboardTracking(False)
        self._spinbox_max_iter.setToolTip(
            "Nombre maximum de mises à jour Jacobienne.\n"
            "Avec un bon point de départ analytique, 2-5 mises à jour suffisent."
        )
        form.addRow("Max. mises à jour :", self._spinbox_max_iter)

        self._spinbox_seuil_pos = QDoubleSpinBox()
        self._spinbox_seuil_pos.setRange(1e-6, 10.0)
        self._spinbox_seuil_pos.setDecimals(4)
        self._spinbox_seuil_pos.setSingleStep(0.001)
        self._spinbox_seuil_pos.setValue(0.001)
        self._spinbox_seuil_pos.setSuffix(" mm")
        self._spinbox_seuil_pos.setKeyboardTracking(False)
        self._spinbox_seuil_pos.setToolTip("Seuil de convergence en position (norme 3D).")
        form.addRow("Seuil position :", self._spinbox_seuil_pos)

        self._spinbox_seuil_ori = QDoubleSpinBox()
        self._spinbox_seuil_ori.setRange(1e-6, 10.0)
        self._spinbox_seuil_ori.setDecimals(4)
        self._spinbox_seuil_ori.setSingleStep(0.001)
        self._spinbox_seuil_ori.setValue(0.001)
        self._spinbox_seuil_ori.setSuffix(" °")
        self._spinbox_seuil_ori.setKeyboardTracking(False)
        self._spinbox_seuil_ori.setToolTip("Seuil de convergence en orientation (norme axis-angle).")
        form.addRow("Seuil orientation :", self._spinbox_seuil_ori)

        self._spinbox_epsilon = QDoubleSpinBox()
        self._spinbox_epsilon.setRange(1e-9, 1e-3)
        self._spinbox_epsilon.setDecimals(9)
        self._spinbox_epsilon.setSingleStep(1e-7)
        self._spinbox_epsilon.setValue(1e-6)
        self._spinbox_epsilon.setSuffix(" rad")
        self._spinbox_epsilon.setKeyboardTracking(False)
        self._spinbox_epsilon.setToolTip(
            "Pas de différentiation pour la Jacobienne numérique.\n"
            "Valeur typique : 1e-6 rad."
        )
        form.addRow("Epsilon (Jacobienne) :", self._spinbox_epsilon)

        self._spinbox_lambda = QDoubleSpinBox()
        self._spinbox_lambda.setRange(0.0001, 1.0)
        self._spinbox_lambda.setDecimals(4)
        self._spinbox_lambda.setSingleStep(0.005)
        self._spinbox_lambda.setValue(0.01)
        self._spinbox_lambda.setKeyboardTracking(False)
        self._spinbox_lambda.setToolTip(
            "Facteur d'amortissement λ (Levenberg-Marquardt).\n"
            "Augmenter si le solveur diverge près des singularités (q5 ≈ 0°)."
        )
        form.addRow("Amortissement λ :", self._spinbox_lambda)

    def _init_convergence_group(self):
        form = QFormLayout(self._group_convergence)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignLeft)

        self._lbl_statut = QLabel("Désactivé")
        self._lbl_statut.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        form.addRow("Statut :", self._lbl_statut)

        self._lbl_iterations = _make_value_label("—")
        form.addRow("Mises à jour :", self._lbl_iterations)

        self._lbl_erreur_pos = _make_value_label("—")
        form.addRow("Erreur position :", self._lbl_erreur_pos)

        self._lbl_erreur_ori = _make_value_label("—")
        form.addRow("Erreur orientation :", self._lbl_erreur_ori)

        self._lbl_message = QLabel("—")
        self._lbl_message.setWordWrap(True)
        self._lbl_message.setStyleSheet("font-size: 10px; color: gray;")
        form.addRow("Info :", self._lbl_message)

    def _init_comparaison_group(self):
        layout = QVBoxLayout(self._group_comparaison)

        # Tableau : 4 colonnes (Joint, Analytique, Jacobienne, Écart)
        self._table_comparaison = QTableWidget(6, 4)
        self._table_comparaison.setHorizontalHeaderLabels([
            "Joint", "Analytique (°)", "Jacobienne (°)", "Écart (°)"
        ])
        self._table_comparaison.setVerticalHeaderLabels([""] * 6)
        self._table_comparaison.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table_comparaison.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self._table_comparaison.verticalHeader().setVisible(False)
        self._table_comparaison.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )

        # Pré-remplissage des noms de joints
        for i in range(6):
            item = QTableWidgetItem(f"q{i+1}")
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            item.setForeground(QBrush(QColor("gray")))
            self._table_comparaison.setItem(i, 0, item)
            for col in range(1, 4):
                cell = QTableWidgetItem("—")
                cell.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self._table_comparaison.setItem(i, col, cell)

        # Hauteur fixe
        self._table_comparaison.setFixedHeight(6 * 26 + 30)

        layout.addWidget(self._table_comparaison)

    # --------------------------------------------------------
    # API publique
    # --------------------------------------------------------

    def is_enabled(self) -> bool:
        return self._checkbox_enabled.isChecked()

    def get_params(self) -> MgiJacobienParams:
        return MgiJacobienParams(
            max_iterations=self._spinbox_max_iter.value(),
            seuil_position=self._spinbox_seuil_pos.value(),
            seuil_orientation=self._spinbox_seuil_ori.value(),
            epsilon=self._spinbox_epsilon.value(),
            lambda_damping=self._spinbox_lambda.value()
        )

    def set_params(self, params: MgiJacobienParams):
        self._building = True
        self._spinbox_max_iter.setValue(params.max_iterations)
        self._spinbox_seuil_pos.setValue(params.seuil_position)
        self._spinbox_seuil_ori.setValue(params.seuil_orientation)
        self._spinbox_epsilon.setValue(params.epsilon)
        self._spinbox_lambda.setValue(params.lambda_damping)
        self._building = False

    def set_resultat(self, resultat: MgiJacobienResultat | None):
        """
        Met à jour l'affichage de convergence et le tableau de comparaison.

        Args:
            resultat: Résultat du solveur (None pour réinitialiser).
        """
        if resultat is None or not self.is_enabled():
            self._reset_display()
            return

        # --- Statut ---
        if resultat.converge:
            self._lbl_statut.setText("Convergé")
            self._lbl_statut.setStyleSheet(f"color: {_COLOR_OK}; font-weight: bold;")
        else:
            self._lbl_statut.setText("Non convergé")
            self._lbl_statut.setStyleSheet(f"color: {_COLOR_WARN}; font-weight: bold;")

        # --- Métriques ---
        max_iter = self._spinbox_max_iter.value()
        self._lbl_iterations.setText(f"{resultat.nb_mises_a_jour} / {max_iter}")
        self._lbl_erreur_pos.setText(f"{resultat.erreur_position:.4f} mm")
        self._lbl_erreur_ori.setText(f"{resultat.erreur_orientation:.4f} °")
        self._lbl_message.setText(resultat.message)

        # --- Tableau de comparaison ---
        self._update_table_comparaison(
            resultat.joints_analytiques,
            resultat.joints
        )

    # --------------------------------------------------------
    # Slots internes
    # --------------------------------------------------------

    def _on_enabled_toggled(self, checked: bool):
        self._update_params_enabled()
        if not checked:
            self._reset_display()
        self.enabled_changed.emit(checked)

    def _on_param_changed(self):
        if not self._building:
            self.params_changed.emit()

    def _update_params_enabled(self):
        enabled = self._checkbox_enabled.isChecked()
        self._group_params.setEnabled(enabled)
        self._group_convergence.setEnabled(enabled)
        self._group_comparaison.setEnabled(enabled)

    def _reset_display(self):
        """Réinitialise tous les affichages à l'état vide."""
        self._lbl_statut.setText("Désactivé")
        self._lbl_statut.setStyleSheet(f"color: {_COLOR_GRAY};")
        self._lbl_iterations.setText("—")
        self._lbl_erreur_pos.setText("—")
        self._lbl_erreur_ori.setText("—")
        self._lbl_message.setText("—")
        for i in range(6):
            for col in range(1, 4):
                self._table_comparaison.item(i, col).setText("—")
                self._table_comparaison.item(i, col).setForeground(
                    QBrush(QColor("white"))
                )

    def _update_table_comparaison(self,
                                   joints_analytiques: list[float] | None,
                                   joints_jacobien: list[float]):
        """
        Remplit le tableau de comparaison.

        Les écarts dépassant _SEUIL_DIFF_DEG sont mis en évidence en orange.
        """
        for i in range(6):
            q_jac = joints_jacobien[i] if i < len(joints_jacobien) else 0.0

            # Colonne Jacobienne
            item_jac = self._table_comparaison.item(i, 2)
            item_jac.setText(f"{q_jac:.4f}")
            item_jac.setForeground(QBrush(QColor("white")))

            if joints_analytiques is not None and i < len(joints_analytiques):
                q_ana = joints_analytiques[i]
                ecart = q_jac - q_ana

                # Colonne Analytique
                item_ana = self._table_comparaison.item(i, 1)
                item_ana.setText(f"{q_ana:.4f}")
                item_ana.setForeground(QBrush(QColor("white")))

                # Colonne Écart — couleur selon l'amplitude
                item_ecart = self._table_comparaison.item(i, 3)
                item_ecart.setText(f"{ecart:+.4f}")
                if abs(ecart) >= _SEUIL_DIFF_DEG:
                    item_ecart.setForeground(QBrush(QColor(_COLOR_WARN)))
                else:
                    item_ecart.setForeground(QBrush(QColor(_COLOR_OK)))
            else:
                # Pas de joints analytiques disponibles
                self._table_comparaison.item(i, 1).setText("—")
                self._table_comparaison.item(i, 3).setText("—")
