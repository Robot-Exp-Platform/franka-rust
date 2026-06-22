from typing import Callable

from robot_behavior import (
    Arm,
    ArmState,
    ArmTorqueControl,
    CartesianPoseControl,
    CartesianVelocityControl,
    FlangeMotion,
    JointMotion,
    JointPositionControl,
    JointState,
    JointTorqueControl,
    JointVelocityControl,
    LoadState,
    MotionType,
    Pose,
    SpatialSample,
    SpatialState,
    JointSample,
    Vec,
)

JointControlFn = Callable[[JointState, float], tuple[Vec, bool]]
ArmControlFn = Callable[[ArmState, float], tuple[Vec, bool]]
ArmPoseControlFn = Callable[[ArmState, float], tuple[Pose, bool]]


class _FrankaArm(
    Arm,
    JointMotion,
    FlangeMotion,
    JointPositionControl,
    JointVelocityControl,
    JointTorqueControl,
    ArmTorqueControl,
    CartesianVelocityControl,
    CartesianPoseControl,
):
    def set_collision_behavior(
        self,
        lower_torque_thresholds_acceleration: Vec,
        upper_torque_thresholds_acceleration: Vec,
        lower_torque_thresholds_nominal: Vec,
        upper_torque_thresholds_nominal: Vec,
        lower_force_thresholds_acceleration: Vec,
        upper_force_thresholds_acceleration: Vec,
        lower_force_thresholds_nominal: Vec,
        upper_force_thresholds_nominal: Vec,
    ) -> None: ...
    def set_joint_impedance(self, data: Vec) -> None: ...
    def set_cartesian_impedance(self, data: Vec) -> None: ...
    def set_guiding_mode(self, guiding_mode: list[bool], nullspace: bool) -> None: ...
    def set_ee_to_k(self, pose: Vec) -> None: ...
    def set_ne_to_ee(self, pose: Vec) -> None: ...
    def set_default_behavior(self) -> None: ...
    def model(self) -> "RobotModel": ...


class FrankaEmika(_FrankaArm):
    def __init__(self, ip: str) -> None: ...


class FrankaFR3(_FrankaArm):
    def __init__(self, ip: str) -> None: ...


class FrankaGripper:
    def __init__(self, ip: str) -> None: ...
    def homing(self) -> bool: ...
    def grasp(self, width: float, speed: float, force: float) -> bool: ...
    def move_gripper(self, width: float, speed: float) -> bool: ...
    def stop(self) -> bool: ...


class RobotModel:
    def __init__(self, path: str) -> None: ...
    def coriolis(self, q: Vec, dq: Vec, m: float, x: Vec, i: Vec) -> Vec: ...
    def gravity(self, q: Vec, m: float, x: Vec, gravity: Vec) -> Vec: ...
