from models.robot_model import RobotModel
from models.types import Pose6
from utils.reference_frame_utils import FrameTransform, convert_pose_to_base_frame
from models.trajectory_keypoint import KeypointTargetType, TrajectoryKeypoint
from utils.mgi import RobotTool


def resolve_keypoint_xyz(
    robot_model: RobotModel,
    keypoint: TrajectoryKeypoint,
    tool: RobotTool | None = None,
    robot_base_pose_world: FrameTransform | Pose6 | None = None,
) -> list[float] | None:
    if keypoint.target_type == KeypointTargetType.CARTESIAN:
        cartesian_values = keypoint.cartesian_target
        target_pose = Pose6(*cartesian_values[:6])
        target = convert_pose_to_base_frame(
            target_pose,
            keypoint.cartesian_frame,
            Pose6.zeros() if robot_base_pose_world is None else robot_base_pose_world,
        )
        return [target.x, target.y, target.z]

    fk_result = robot_model.compute_fk_joints(keypoint.joint_target, tool=tool)
    if fk_result is None:
        return None
    _, _, pose, _, _ = fk_result
    return [pose.x, pose.y, pose.z]
