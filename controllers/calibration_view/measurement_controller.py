from typing import Dict
from PyQt6.QtCore import QObject
from PyQt6.QtWidgets import QFileDialog

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from widgets.calibration_view.measurement_widget import MeasurementWidget

from utils.file_io import FileIOHandler
import utils.math_utils as math_utils
import numpy as np
import os
import csv
import math


class MeasurementController(QObject):
    DEFAULT_MEASUREMENTS_DIRECTORY = os.path.join(".", "user_data", "measures")
    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        measurement_widget: MeasurementWidget,
        parent: QObject = None,
    ):
        super().__init__(parent)
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.measurement_widget = measurement_widget
        self._setup_connections()
    
    def _setup_connections(self) -> None:
        # Signals from Robot Model
        self.robot_model.measurements_changed.connect(self._on_robot_measurements_changed)
        self.robot_model.measurements_points_changed.connect(self._on_robot_measurements_points_changed)

        # Signals from View
        self.measurement_widget.import_measurements_requested.connect(self._on_view_import_measurements_requested)
        self.measurement_widget.clear_measurements_requested.connect(self._on_view_clear_measurements_requested)
        self.measurement_widget.set_as_reference_requested.connect(self._on_view_set_as_reference_requested)
        self.measurement_widget.repere_selected.connect(self._on_view_repere_selected)
        self.measurement_widget.rotation_type_changed.connect(self._on_view_rotation_type_changed)
        self.measurement_widget.dh_checkboxes_changed.connect(self._on_dh_checkboxes_changed)

        self.robot_model.dh_params_changed.connect(self._update_tcp_offsets_from_selection)
        self.robot_model.joints_changed.connect(self._update_tcp_offsets_from_selection)
        self.robot_model.axis_reversed_changed.connect(self._update_tcp_offsets_from_selection)
        self.tool_model.tool_changed.connect(self._update_tcp_offsets_from_selection)

    # ======
    # Connection callbacks
    # ======

    def _on_robot_measurements_changed(self) -> None:
        self.measurement_widget.set_measure_filename(self.robot_model.get_measurements_filename())

    def _on_robot_measurements_points_changed(self) -> None:
        self.measurement_widget.set_measurements_data(self.robot_model.get_measurements())
    
    def _on_view_import_measurements_requested(self) -> None:
        currentDir = os.getcwd()
        measurementsDir = os.path.join(currentDir, MeasurementController.DEFAULT_MEASUREMENTS_DIRECTORY)

        file_path, _ = QFileDialog.getOpenFileName(
            self.measurement_widget,
            "Importer un fichier de mesures",
            measurementsDir if os.path.exists(measurementsDir) else currentDir,
            "Fichiers CSV (*.csv)"
        )

        if not file_path:
            return

        if not file_path.lower().endswith('.csv'):
            print("Format de fichier non supporte. Utiliser un CSV.")
            return

        try:
            data = self._parse_csv_measurements(file_path)
        except Exception as e:
            print(f"Erreur lors de l'import du fichier: {e}")
            return

        if not data or not isinstance(data, list):
            print("Aucune mesure trouvee dans le fichier")
            return

        # Reset existing measurements
        self.robot_model.clear_measurements()
        self.measurement_widget.clear_measurements()

        self.robot_model.blockSignals(True)
        for measurement in data:
            self.robot_model.add_measurement(measurement)
        self.robot_model.blockSignals(False)

        self.measurement_widget.set_measurements_data(data)

        repere_names = [m.get("name", f"Repere {i}") for i, m in enumerate(data)]
        self.measurement_widget.populate_tree(repere_names)

        reference_name = self._find_identity_reference_name(data)
        if reference_name:
            self.measurement_widget.set_reference_bold(reference_name)

        file_name = os.path.splitext(os.path.basename(file_path))[0]
        self.measurement_widget.set_measure_filename(file_name)

        if self.measurement_widget.tree.topLevelItemCount() > 0:
            first_item = self.measurement_widget.tree.topLevelItem(0)
            self.measurement_widget.tree.setCurrentItem(first_item)
            self._on_view_repere_selected(first_item.text(0))

        self.display_measured_dh_parameters()
        self.measurement_widget.set_measured_controls_enabled(True)

    def _parse_csv_measurements(self, file_path: str) -> list:
        """
        Parse un fichier CSV contenant des matrices 4x4 avec le format :
        // Col::Object,T c0,T c1,T c2,T c3,[Timestamp]
        World_Frame,1.000000,-0.000000,-0.000000,0.0000
        ,...
        Geo robot::R1,...
        ...

        Retourne une liste de dictionnaires contenant les mesures
        (World_Frame + R1 a R6). La rotation est conservee en matrice 3x3.
        """
        measurements = []

        try:
            with open(file_path, 'r', encoding='utf-8') as csvfile:
                next(csvfile)  # skip header
                lines = csvfile.readlines()
                i = 0

                while i < len(lines):
                    line = lines[i].strip()
                    if not line:
                        i += 1
                        continue

                    if not line.startswith(','):
                        try:
                            row = line.split(',')
                            label = row[0].strip()
                            repere_name = label.split("::")[-1] if "::" in label else label
                            repere_name = repere_name.replace("'", "").strip()

                            matrix_data = []
                            for j in range(1, 5):
                                if j < len(row):
                                    matrix_data.append(float(row[j]))

                            for _ in range(3):
                                if i + 1 < len(lines):
                                    i += 1
                                    next_line = lines[i].strip()
                                    if next_line.startswith(','):
                                        row = next_line[1:].split(',')
                                        for j in range(4):
                                            if j < len(row):
                                                matrix_data.append(float(row[j]))

                            if len(matrix_data) == 16:
                                T = np.array(matrix_data).reshape(4, 4)

                                x = float(T[0, 3])
                                y = float(T[1, 3])
                                z = float(T[2, 3])
                                R = T[:3, :3]

                                measurement = {
                                    "name": repere_name,
                                    "X": x,
                                    "Y": y,
                                    "Z": z,
                                    "R": R,
                                    "T": T,
                                }
                                measurements.append(measurement)
                            else:
                                print(f"Erreur: matrice incomplete pour {repere_name}")
                        except (ValueError, IndexError) as e:
                            print(f"Erreur lors du parsing du repere: {e}")

                    i += 1

        except IOError as e:
            print(f"Erreur lors de la lecture du fichier CSV: {e}")
            return []

        return measurements

    def _on_view_clear_measurements_requested(self) -> None:
        self.robot_model.clear_measurements()
        self.robot_model.clear_measurement_points()
        self.measurement_widget.clear_measurements()
        self.measurement_widget.set_measured_controls_enabled(False)
        self._update_tcp_offsets_from_selection()
    
    def _invert_homogeneous_transform(self, T: np.ndarray) -> np.ndarray:
        """Invert a 4x4 rigid homogeneous transform (SE(3))."""
        return math_utils.invert_homogeneous_transform(T, validate=True)

    def _orthonormalize_rotation(self, R: np.ndarray) -> np.ndarray:
        """Project a near-rotation matrix onto SO(3)."""
        return math_utils.orthonormalize_rotation(R)

    def _on_view_set_as_reference_requested(self) -> None:
        current_item = self.measurement_widget.tree.currentItem()
        if not current_item:
            print("Aucun repère sélectionné")
            return

        ref_repere_name = current_item.text(0)

        ref_measurement = None
        for measurement in self.measurement_widget.measurements:
            if measurement.get("name") == ref_repere_name:
                ref_measurement = measurement
                break

        if ref_measurement is None:
            print(f"Repère {ref_repere_name} non trouvé")
            return

        try:
            T_ref = self._build_transform_from_measurement(ref_measurement)
            T_ref_inv = self._invert_homogeneous_transform(T_ref)

            for measurement in self.measurement_widget.measurements:
                T = self._build_transform_from_measurement(measurement)
                T_new = T_ref_inv @ T

                measurement["T"] = T_new
                measurement["X"] = float(T_new[0, 3])
                measurement["Y"] = float(T_new[1, 3])
                measurement["Z"] = float(T_new[2, 3])
                measurement["R"] = T_new[:3, :3].copy()

                # Keep angles in sync for compatibility with other display paths.
                angles = math_utils.rotation_matrix_to_fixed_xyz(measurement["R"])
                measurement["A"] = float(angles[0])
                measurement["B"] = float(angles[1])
                measurement["C"] = float(angles[2])

            self.measurement_widget.set_reference_bold(ref_repere_name)

            # Refresh currently displayed frame data in table_me.
            current_after = self.measurement_widget.tree.currentItem()
            if current_after:
                self._on_view_repere_selected(current_after.text(0))
            else:
                self._on_view_repere_selected(ref_repere_name)

            print(f"Repère '{ref_repere_name}' défini comme référence.")

        except (ValueError, TypeError, np.linalg.LinAlgError) as e:
            print(f"Erreur lors de la définition de la référence: {e}")
    
    def _on_view_repere_selected(self, repere_name: str) -> None:
        # Chercher la mesure correspondante dans le widget
        for i, measurement in enumerate(self.measurement_widget.measurements):
            if measurement.get("name") == repere_name:
                # display_mode combo was removed from the widget.
                # Keep backward compatibility if it is ever reintroduced.
                display_mode_combo = getattr(self.measurement_widget, "display_mode", None)
                if display_mode_combo is None:
                    self.measurement_widget.display_measurement(measurement)
                else:
                    display_mode = display_mode_combo.currentText()
                    if display_mode == "Ecarts":
                        self._display_measurement_deviations(i, measurement)
                    else:
                        self.measurement_widget.display_measurement(measurement)
                break

    def _display_measurement_deviations(self, index: int, measurement: Dict[str, float]) -> None:
        """Fallback display for deviation mode when dedicated deviation computation is unavailable."""
        self.measurement_widget.display_measurement(measurement)

    def _on_view_rotation_type_changed(self, rotation_type: str) -> None:
        current_item = self.measurement_widget.tree.currentItem()
        if current_item:
            self._on_view_repere_selected(current_item.text(0))

    def _on_dh_checkboxes_changed(self) -> None:
        self._update_tcp_offsets_from_selection()

    def _compute_tcp_xyz_from_dh(self, dh_params: list[list[float]], joints_deg: list[float]) -> np.ndarray:
        """Compute TCP XYZ from DH parameters and current joints."""
        T = np.eye(4)
        axis_reversed = self.robot_model.get_axis_reversed()

        for i in range(6):
            row = dh_params[i] if i < len(dh_params) else [0.0, 0.0, 0.0, 0.0]
            alpha = math.radians(float(row[0]))
            d = float(row[1])
            theta_offset = math.radians(float(row[2]))
            r = float(row[3])

            q_deg = float(joints_deg[i]) if i < len(joints_deg) else 0.0
            theta = theta_offset + math.radians(q_deg * axis_reversed[i])

            T = T @ math_utils.dh_modified(alpha, d, theta, r)

        T = T @ self.robot_model.get_T_tool(self.tool_model.get_tool())
        return T[:3, 3].astype(float)

    def _build_selected_dh_params(self, theoretical_dh: list[list[float]]) -> list[list[float]]:
        """Merge theoretical DH with checked measured values from calibration table."""
        selected = [list(row[:4]) for row in theoretical_dh[:6]]
        while len(selected) < 6:
            selected.append([0.0, 0.0, 0.0, 0.0])

        measured = self.measurement_widget.get_measured_dh_params()
        states = self.measurement_widget.get_dh_checkboxes_state()
        param_names = ["alpha", "d", "theta", "r"]

        for row in range(6):
            for col, param_name in enumerate(param_names):
                if states.get(f"{param_name}{row + 1}", False):
                    if row < len(measured) and col < len(measured[row]):
                        selected[row][col] = float(measured[row][col])

        return selected

    def get_selected_measured_dh_params(self) -> list[list[float]]:
        theoretical_dh = self.robot_model.get_dh_params()
        return self._build_selected_dh_params(theoretical_dh)

    def _update_tcp_offsets_from_selection(self) -> None:
        theoretical_dh = self.robot_model.get_dh_params()
        selected_dh = self._build_selected_dh_params(theoretical_dh)
        joints_deg = self.robot_model.get_joints()

        tcp_theoretical = self._compute_tcp_xyz_from_dh(theoretical_dh, joints_deg)
        tcp_selected = self._compute_tcp_xyz_from_dh(selected_dh, joints_deg)
        offsets = tcp_selected - tcp_theoretical

        self.measurement_widget.set_tcp_offsets_values(
            [float(v) for v in tcp_selected.tolist()],
            [float(v) for v in offsets.tolist()],
        )

    def _order_measurements_for_dh(self, measurements: list[Dict[str, float]]) -> list[Dict[str, float]]:
        """Order repere list as World/Base then R1..R6 when possible."""
        import re

        if not measurements:
            return []

        def name_of(m):
            return str(m.get("name", "")).strip()

        world_candidates = []
        r_frames = []
        others = []

        for m in measurements:
            name = name_of(m)
            low = name.lower()
            if "world" in low or "base" in low:
                world_candidates.append(m)
                continue
            match = re.search(r"\br\s*([0-9]+)\b", low)
            if match:
                idx = int(match.group(1))
                r_frames.append((idx, m))
            else:
                others.append(m)

        ordered = []
        if world_candidates:
            ordered.append(world_candidates[0])
            if len(world_candidates) > 1:
                others = world_candidates[1:] + others

        r_frames.sort(key=lambda it: it[0])
        ordered.extend([m for _, m in r_frames])
        ordered.extend(others)
        return ordered

    def _build_transform_from_measurement(self, measurement: Dict[str, float]) -> np.ndarray:
        """Build 4x4 transform from a measurement entry."""
        T = measurement.get("T")
        if isinstance(T, np.ndarray) and T.shape == (4, 4):
            return T.astype(float).copy()

        x = float(measurement.get("X", 0))
        y = float(measurement.get("Y", 0))
        z = float(measurement.get("Z", 0))

        if "R" in measurement and isinstance(measurement["R"], np.ndarray):
            R = measurement["R"]
        else:
            R = np.eye(3)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def _find_identity_reference_name(self, measurements: list[Dict[str, float]]) -> str | None:
        """Return frame name whose transform is identity (within numerical tolerance)."""
        identity = np.eye(4)
        for measurement in measurements:
            T = self._build_transform_from_measurement(measurement)
            if np.allclose(T, identity, rtol=0.0, atol=1e-9):
                name = measurement.get("name")
                if isinstance(name, str) and name:
                    return name
        return None

    def _extract_dh_params_from_matrix(self, T: np.ndarray) -> Dict[str, float]:
        """
        Invert math_utils.dh_modified() to get alpha, d, theta, r from T.
        T = [
          [ ct,   -st,    0,    d ],
          [ st*ca, ct*ca, -sa, -r*sa ],
          [ st*sa, ct*sa,  ca,  r*ca ],
          [ 0,     0,     0,    1 ]
        ]
        """
        r11 = float(T[0, 0])
        r12 = float(T[0, 1])
        r23 = float(T[1, 2])
        r33 = float(T[2, 2])

        theta_rad = math.atan2(-r12, r11)
        theta = math.degrees(theta_rad)

        alpha_rad = math.atan2(-r23, r33)
        alpha = math.degrees(alpha_rad)

        translation = T[:3, 3].astype(float)

        # Khalil-Kleinfinger: d is the signed distance along x_(n-1) between
        # z_(n-1) and z_n. In this relative transform, x_(n-1) is the first
        # basis axis of the previous frame, expressed here as [1, 0, 0].
        x_prev = np.array([1.0, 0.0, 0.0], dtype=float)
        d = float(np.dot(translation, x_prev))

        # Khalil-Kleinfinger: r is the signed distance along z_n between
        # x_(n-1) and x_n. In this matrix, z_n is the 3rd column.
        z_n = T[:3, 2].astype(float)
        r = float(np.dot(translation, z_n))

        return {"alpha": alpha, "d": d, "theta": theta, "r": r}

    def calculate_measured_dh_parameters(self) -> list[Dict[str, float]]:
        """Compute DH parameters from imported frames (World + R1..R6)."""
        measurements = self._order_measurements_for_dh(self.measurement_widget.measurements)
        if len(measurements) < 2:
            return []

        # Build cumulative transforms (World -> Ri)
        cumulative = [self._build_transform_from_measurement(m) for m in measurements]

        # Limit to World + R1..R6
        if len(cumulative) > 7:
            cumulative = cumulative[:7]

        # Known measurement pose (deg)
        q_measured = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
        axis_reversed = self.robot_model.get_axis_reversed()
        q_effective = [q_measured[i] * axis_reversed[i] for i in range(6)]

        dh_measured = []
        for joint_idx in range(1, min(7, len(cumulative))):
            T_prev = cumulative[joint_idx - 1]
            T_curr = cumulative[joint_idx]
            T_joint = math_utils.invert_homogeneous_transform(T_prev) @ T_curr

            params = self._extract_dh_params_from_matrix(T_joint)

            # Remove known pose to recover theta offset
            if (joint_idx - 1) < len(q_effective):
                params["theta"] = params["theta"] - q_effective[joint_idx - 1]

            dh_measured.append(params)

        while len(dh_measured) < 6:
            dh_measured.append({"alpha": 0.0, "d": 0.0, "theta": 0.0, "r": 0.0})

        return dh_measured

    def display_measured_dh_parameters(self) -> None:
        """Compute and display measured DH parameters in table_dh_measured."""
        dh_measured = self.calculate_measured_dh_parameters()
        if not dh_measured:
            print("Aucune mesure DH calculée")
            self._update_tcp_offsets_from_selection()
            return
        self.measurement_widget.populate_dh_measured_deviations(dh_measured)
        self._update_tcp_offsets_from_selection()
