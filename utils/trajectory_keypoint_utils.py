from models.robot_model import RobotModel
from models.trajectory_keypoint import KeypointTargetType, TrajectoryKeypoint
from utils.mgi import RobotTool


def resolve_keypoint_xyz(
    robot_model: RobotModel,
    keypoint: TrajectoryKeypoint,
    tool: RobotTool | None = None,
) -> list[float] | None:
    if keypoint.target_type == KeypointTargetType.CARTESIAN:
        target = keypoint.cartesian_target
        if len(target) < 3:
            return None
        return [float(target[0]), float(target[1]), float(target[2])]

    fk_result = robot_model.compute_fk_joints(keypoint.joint_target, tool=tool)
    if fk_result is None:
        return None
    _, _, pose, _, _ = fk_result
    if len(pose) < 3:
        return None
    return [float(pose[0]), float(pose[1]), float(pose[2])]
