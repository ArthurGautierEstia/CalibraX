import math

from models.robot_model import RobotModel
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.trajectory_result import (
    SegmentResult,
    TrajectoryBuilderBehavior,
    TrajectoryComputationStatus,
    TrajectoryResult,
    TrajectorySample,
    TrajectorySegment,
)
from utils.bezier3 import Bezier3Coefficients3D
import utils.math_utils as math_utils
from utils.mgi import MgiConfigKey


class TrajectoryBuilder:
    DEFAULT_SAMPLE_DT_S = 0.004  # 4 ms
    DEFAULT_ARC_LENGTH_SAMPLES = 200
    MAX_SAMPLES_PER_SEGMENT = 50_000
    _EPS = 1e-9

    def __init__(
        self,
        robot_model: RobotModel,
        behavior: TrajectoryBuilderBehavior = TrajectoryBuilderBehavior.CONTINUE_ON_ERROR,
        sample_dt_s: float = DEFAULT_SAMPLE_DT_S,
    ) -> None:
        self.robot_model = robot_model
        self.behavior = behavior
        self.sample_dt_s = sample_dt_s if sample_dt_s > 0.0 else TrajectoryBuilder.DEFAULT_SAMPLE_DT_S

    @staticmethod
    def linear_speed_mps_to_mmps(speed_mps: float) -> float:
        return float(speed_mps) * 1000.0

    @staticmethod
    def _new_empty_segment(last_time_s: float) -> SegmentResult:
        segment = SegmentResult()
        segment.last_time = float(last_time_s)
        return segment

    @staticmethod
    def _extract_previous_sample(result: SegmentResult) -> TrajectorySample | None:
        if not result.samples:
            return None
        return result.samples[-1]

    @staticmethod
    def _accumulate_status(
        trajectory: TrajectoryResult,
        segment_result: SegmentResult,
        segment_index: int,
    ) -> None:
        if segment_result.status == TrajectoryComputationStatus.SUCCESS:
            return
        if trajectory.status != TrajectoryComputationStatus.SUCCESS:
            return
        trajectory.status = segment_result.status
        trajectory.first_error_segment_index = segment_index

    @staticmethod
    def _get_interpolated_time(linear_t: float) -> float:
        """
        Description:
            Smooth time using f(t) = -2 * t^3 + 3 * t^2.
        Argument:
            t, in [0;1].
        Return:
            smoothed t, in [0;1].
        """
        # Clamp
        if linear_t < 0:
            linear_t = 0
        elif linear_t > 1:
            linear_t = 1
        t2 = linear_t * linear_t
        return -2 * t2 * linear_t + 3 * t2

    def _should_stop_on_error(self, status: TrajectoryComputationStatus) -> bool:
        if status == TrajectoryComputationStatus.SUCCESS:
            return False
        return self.behavior == TrajectoryBuilderBehavior.STOP_ON_ERROR

    def _resolve_keypoint_pose(self, keypoint: TrajectoryKeypoint) -> list[float] | None:
        if keypoint.target_type == KeypointTargetType.CARTESIAN:
            return [float(v) for v in keypoint.cartesian_target[:6]]
        return self._resolve_pose_from_joints(keypoint.joint_target)

    def _resolve_pose_from_joints(self, joints_deg: list[float]) -> list[float] | None:
        fk_result = self.robot_model.compute_fk_joints(joints_deg)
        if fk_result is None:
            return None
        _, _, dh_pose, _, _ = fk_result
        if len(dh_pose) < 6:
            return None
        return [float(v) for v in dh_pose[:6]]

    @staticmethod
    def _linear_tangents_from_points(p0: list[float], p3: list[float]) -> tuple[list[float], list[float]]:
        dx = (p3[0] - p0[0]) / 3.0
        dy = (p3[1] - p0[1]) / 3.0
        dz = (p3[2] - p0[2]) / 3.0
        return [dx, dy, dz], [-dx, -dy, -dz]

    @staticmethod
    def _estimate_arc_length(coeffs: Bezier3Coefficients3D, samples_count: int = DEFAULT_ARC_LENGTH_SAMPLES) -> float:
        """
        Approximation de l'abscisse curviligne
        """
        if samples_count < 2:
            samples_count = 2

        prev = coeffs.point(0.0)
        total = 0.0
        for i in range(1, samples_count + 1):
            t = i / samples_count
            p = coeffs.point(t)
            total += math_utils.norm3(p[0] - prev[0], p[1] - prev[1], p[2] - prev[2])
            prev = p
        return total

    @staticmethod
    def _resolve_num_intervals(arc_length_mm: float, speed_mmps: float, sample_dt_s: float) -> int:
        """
        Identifier le nombre d'intervalle pour parcourir la courbe à une vitesse donnée, pour un échantillonage temporel donnée.
        """
        if arc_length_mm <= TrajectoryBuilder._EPS or speed_mmps <= TrajectoryBuilder._EPS:
            return 1

        # T = D/V
        theoretical_duration_s = arc_length_mm / speed_mmps
        # intervals = T/dT
        intervals = int(math.ceil(theoretical_duration_s / sample_dt_s))
        return min(max(1, intervals), TrajectoryBuilder.MAX_SAMPLES_PER_SEGMENT) # 1 <= interval <= MAX

    def compute_trajectory(
        self,
        current_joints: list[float],
        segments: list[TrajectorySegment],
    ) -> TrajectoryResult:
        trajectory = TrajectoryResult()
        if not segments:
            return trajectory

        previous_sample_1: TrajectorySample | None = None
        start_time_s = 0.0

        first_result = self.compute_first_segment(current_joints, segments[0].from_keypoint, start_time_s)
        trajectory.segments.append(first_result)
        TrajectoryBuilder._accumulate_status(trajectory, first_result, segment_index=0)
        if self._should_stop_on_error(first_result.status):
            return trajectory

        previous_sample_1 = TrajectoryBuilder._extract_previous_sample(first_result)
        start_time_s = first_result.last_time

        for idx, segment in enumerate(segments, start=1):
            segment_result = self.compute_segment(segment, previous_sample_1, start_time_s)
            trajectory.segments.append(segment_result)
            TrajectoryBuilder._accumulate_status(trajectory, segment_result, segment_index=idx)
            if self._should_stop_on_error(segment_result.status):
                break

            previous_sample_1 = TrajectoryBuilder._extract_previous_sample(segment_result)
            start_time_s = segment_result.last_time

        return trajectory

    def compute_first_segment(
        self,
        current_joints: list[float],
        to_keypoint: TrajectoryKeypoint,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        joints_6 = [float(v) for v in current_joints[:6]]
        while len(joints_6) < 6:
            joints_6.append(0.0)

        config_identifier = self.robot_model.get_config_identifier()
        identified_config = MgiConfigKey.identify_configuration_deg(joints_6, config_identifier)

        synthetic_from = TrajectoryKeypoint(
            target_type=KeypointTargetType.JOINT,
            joint_target=joints_6,
            mode=to_keypoint.mode,
            cubic_vectors=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            allowed_configs=[identified_config],
            favorite_config=identified_config,
            ptp_speed_percent=to_keypoint.ptp_speed_percent,
            linear_speed_mps=to_keypoint.linear_speed_mps,
        )
        segment = TrajectorySegment(synthetic_from, to_keypoint)
        return self.compute_segment(segment, None, start_time_s)

    def compute_segment(
        self,
        segment: TrajectorySegment,
        previous_sample_1: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        if segment.to_keypoint.mode == KeypointMotionMode.LINEAR:
            return self.compute_LIN_segment(segment, previous_sample_1, start_time_s)
        if segment.to_keypoint.mode == KeypointMotionMode.CUBIC:
            return self.compute_cubique_segment(segment, previous_sample_1, start_time_s)
        return self.compute_PTP_segment(segment, previous_sample_1, start_time_s)

    def compute_PTP_segment(
        self,
        segment: TrajectorySegment,
        previous_sample_1: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        # Temporary fallback: for now, PTP uses straight-bezier XYZ.
        return self._generate_bezier_segment(
            segment=segment,
            previous_sample_1=previous_sample_1,
            start_time_s=start_time_s,
            force_linear_handles=True,
        )

    def compute_LIN_segment(
        self,
        segment: TrajectorySegment,
        previous_sample_1: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        # Straight line encoded as cubic bezier.
        return self._generate_bezier_segment(
            segment=segment,
            previous_sample_1=previous_sample_1,
            start_time_s=start_time_s,
            force_linear_handles=True,
        )

    def compute_cubique_segment(
        self,
        segment: TrajectorySegment,
        previous_sample_1: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        return self._generate_bezier_segment(
            segment=segment,
            previous_sample_1=previous_sample_1,
            start_time_s=start_time_s,
            force_linear_handles=False,
        )

    def _generate_bezier_segment(
        self,
        segment: TrajectorySegment,
        previous_sample_1: TrajectorySample | None = None,
        start_time_s: float = 0.0,
        force_linear_handles: bool = False,
    ) -> SegmentResult:
        from_pose = self._resolve_keypoint_pose(segment.from_keypoint)
        to_pose = self._resolve_keypoint_pose(segment.to_keypoint)
        if from_pose is None or to_pose is None:
            result = TrajectoryBuilder._new_empty_segment(start_time_s)
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            return result

        # Extraction des coordonnées cartésiennes xyz
        p0 = [from_pose[0], from_pose[1], from_pose[2]]
        p3 = [to_pose[0], to_pose[1], to_pose[2]]

        # Définition des vecteurs d'entrée / sortie du segment
        if force_linear_handles: # Lineaire
            t_out, t_in = self._linear_tangents_from_points(p0, p3)
        else: # Courbe
            t_out = [float(v) for v in segment.from_keypoint.cubic_vectors[0][:3]]
            t_in = [float(v) for v in segment.to_keypoint.cubic_vectors[1][:3]]

        # Déterminer les coefficients des polynomes x(t), y(t), z(t)
        coeffs = Bezier3Coefficients3D(p0, p3, t_out, t_in)
        # Estimation de l'abscisse curviligne
        arc_length_mm = TrajectoryBuilder._estimate_arc_length(coeffs)
        # Conversion vitesse m/s -> mm/s
        speed_mmps = TrajectoryBuilder.linear_speed_mps_to_mmps(segment.to_keypoint.linear_speed_mps)
        # Déterminaison du nombre d'intervalle en fonction de la longueur, de la vitesse, et de l'échantionnage temporel
        intervals = TrajectoryBuilder._resolve_num_intervals(arc_length_mm, speed_mmps, self.sample_dt_s)

        # Préparation du résultat
        result = SegmentResult()
        result.status = TrajectoryComputationStatus.SUCCESS
        result.duration = intervals * self.sample_dt_s
        result.last_time = start_time_s + result.duration
        result.out_direction = math_utils.normalize3(coeffs.first_derivative(1.0), self._EPS)

        dA = to_pose[3] - from_pose[3]
        dB = to_pose[4] - from_pose[4]
        dC = to_pose[5] - from_pose[5]

        # Boucle pour itérer sur la courbe
        previous_sample = previous_sample_1
        for i in range(1, intervals + 1):
            time_s = start_time_s + i * self.sample_dt_s
            # linear time
            t = i / intervals
            # smoothed time
            smooth_t = TrajectoryBuilder._get_interpolated_time(t)
            # Extract XYZ from the Bezier curve at a time
            xyz = coeffs.point(smooth_t)
            # Compute Rotation
            orientation_abc = [
                from_pose[3] + dA * smooth_t,
                from_pose[4] + dB * smooth_t,
                from_pose[5] + dC * smooth_t
            ]
            # Fill sample
            sample = self._build_sample_from_cartesian(
                time_s,
                xyz,
                orientation_abc,
                previous_sample,
            )
            result.samples.append(sample)
            previous_sample = sample
        return result

    def _build_sample_from_cartesian(
        self,
        time_s: float,
        xyz: list[float],
        orientation_abc: list[float],
        previous_sample: TrajectorySample | None,
    ) -> TrajectorySample:
        
        sample = TrajectorySample()
        sample.time = float(time_s)
        sample.pose[0] = float(xyz[0])
        sample.pose[1] = float(xyz[1])
        sample.pose[2] = float(xyz[2])
        sample.pose[3] = float(orientation_abc[0])
        sample.pose[4] = float(orientation_abc[1])
        sample.pose[5] = float(orientation_abc[2])

        # TODO : Déterminer si le point est accessible
        # avec le MGI
        sample.reachable = True
        sample.configuration = None
        sample.joints = [0.0] * 6
        # TODO : Au lieu de n'avoir qu'un seul joint [float], on pourrait aussi enregistrer les résultats du MGI.
        # Permettrai de changer les joints sans devoir refaire le MGI si config différente

        if previous_sample is None:
            sample.cartesian_velocity = [0.0] * 6
            sample.cartesian_acceleration = [0.0] * 6
            sample.velocity = 0.0
            sample.acceleration = 0.0
            return sample

        # dt - normalement devrait toujours etre self.sample_dt_s (0.004 s)
        dt = sample.time - previous_sample.time
        if dt <= self._EPS:
            dt = self.sample_dt_s

        # cartesian velocity
        vx = (sample.pose[0] - previous_sample.pose[0]) / dt
        vy = (sample.pose[1] - previous_sample.pose[1]) / dt
        vz = (sample.pose[2] - previous_sample.pose[2]) / dt
        sample.cartesian_velocity[0] = vx
        sample.cartesian_velocity[1] = vy
        sample.cartesian_velocity[2] = vz

        # cartesian acceleration
        ax = (vx - previous_sample.cartesian_velocity[0]) / dt
        ay = (vy - previous_sample.cartesian_velocity[1]) / dt
        az = (vz - previous_sample.cartesian_velocity[2]) / dt
        sample.cartesian_acceleration[0] = ax
        sample.cartesian_acceleration[1] = ay
        sample.cartesian_acceleration[2] = az

        # euclidian velocity & acceleration
        sample.velocity = math_utils.norm3(vx, vy, vz)
        sample.acceleration = math_utils.norm3(ax, ay, az)

        # TODO : Dynamique des joints (dJ, d2J)

        return sample
