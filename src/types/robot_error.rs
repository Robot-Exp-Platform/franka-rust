use bitflags::bitflags;
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use std::fmt;

#[derive(Debug, FromPrimitive)]
pub enum FrankaError {
    /// The robot moved past the joint limits.
    JointPositionLimitsViolation,
    /// The robot moved past any of the virtual walls.
    CartesianPositionLimitsViolation,
    /// The robot would have collided with itself
    SelfCollisionAvoidanceViolation,
    /// The robot exceeded joint velocity limits.
    JointVelocityViolation,
    /// The robot exceeded Cartesian velocity limits.
    CartesianVelocityViolation,
    /// The robot exceeded safety threshold during force control.
    ForceControlSafetyViolation,
    /// A collision was detected, i.e.\ the robot exceeded a torque threshold in a joint motion.
    JointReflex,
    /// A collision was detected, i.e.\ the robot exceeded a torque threshold in a Cartesian motion.
    CartesianReflex,
    /// Internal motion generator did not reach the goal pose.
    MaxGoalPoseDeviationViolation,
    /// True if internal motion generator deviated from the path.
    MaxPathPoseDeviationViolation,
    /// Cartesian velocity profile for internal motions was exceeded.
    CartesianVelocityProfileSafetyViolation,
    /// An external joint position motion generator was started with a pose too far from the current pose.
    JointPositionMotionGeneratorStartPoseInvalid,
    /// An external joint motion generator would move into a joint limit.
    JointMotionGeneratorPositionLimitsViolation,
    /// An external joint motion generator exceeded velocity limits.
    JointMotionGeneratorVelocityLimitsViolation,
    /// Commanded velocity in joint motion generators is discontinuous (target values are too far apart).
    JointMotionGeneratorVelocityDiscontinuity,
    /// Commanded acceleration in joint motion generators is discontinuous (target values are too far apart).
    JointMotionGeneratorAccelerationDiscontinuity,
    /// An external Cartesian position motion generator was started with a pose too far from  the current pose.
    CartesianPositionMotionGeneratorStartPoseInvalid,
    /// An external Cartesian motion generator would move into an elbow limit.
    CartesianMotionGeneratorElbowLimitViolation,
    /// An external Cartesian motion generator would move with too high velocity.
    CartesianMotionGeneratorVelocityLimitsViolation,
    /// Commanded velocity in Cartesian motion generators is discontinuous (target values are too far apart).
    CartesianMotionGeneratorVelocityDiscontinuity,
    /// Commanded acceleration in Cartesian motion generators is discontinuous (target values are too far apart).
    CartesianMotionGeneratorAccelerationDiscontinuity,
    /// Commanded elbow values in Cartesian motion generators are inconsistent.
    CartesianMotionGeneratorElbowSignInconsistent,
    /// The first elbow value in Cartesian motion generators is too far from initial one.
    CartesianMotionGeneratorStartElbowInvalid,
    /// The torque set by the external controller is discontinuous.
    ForceControllerDesiredForceToleranceViolation,
    /// The start elbow sign was inconsistent.
    /// Applies only to motions started from Desk.
    StartElbowSignInconsistent,
    /// Minimum network communication quality could not be held during a motion.
    CommunicationConstraintsViolation,
    /// Commanded values would result in exceeding the power limit.
    PowerLimitViolation,
    /// The joint position limits would be exceeded after IK calculation.
    CartesianMotionGeneratorJointPositionLimitsViolation,
    /// The joint velocity limits would be exceeded after IK calculation.
    CartesianMotionGeneratorJointVelocityLimitsViolation,
    /// The joint velocity in Cartesian motion generators is discontinuous after IK calculation
    CartesianMotionGeneratorJointVelocityDiscontinuity,
    /// The joint acceleration in Cartesian motion generators is discontinuous after IK calculation.
    CartesianMotionGeneratorJointAccelerationDiscontinuity,
    /// Cartesian pose is not a valid transformation matrix.
    CartesianPositionMotionGeneratorInvalidFrame,
    /// The torque set by the external controller is discontinuous.
    ControllerTorqueDiscontinuity,
    /// The robot is overloaded for the required motion.\
    /// Applies only to motions started from Desk.
    JointP2PInsufficientTorqueForPlanning,
    /// The measured torque signal is out of the safe range.
    TauJRangeViolation,
    /// An instability is detected.
    InstabilityDetection,
    /// The robot is in joint position limits violation error and the user guides the robot further towards the limit.
    JointMoveInWrongDirection,
    /// The generated motion violates a joint limit.
    CartesianSplineViolation,
    /// The generated motion violates a joint limit.
    JointViaPlanLimitViolation,
    /// The gravity vector could not be initialized by measuring the base acceleration.
    BaseAccelerationInitializationTimeout,
    /// Base acceleration O_ddP_O cannot be determined.
    BaseAccelerationInvalidReading,
}

impl fmt::Display for FrankaError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let description = match self {
            FrankaError::JointPositionLimitsViolation => {
                "[Joint position limits violation]the robot moved past the joint limits."
            }
            FrankaError::CartesianPositionLimitsViolation => {
                "[Cartesian position limits violation]the robot moved past any of the virtual walls."
            }
            FrankaError::SelfCollisionAvoidanceViolation => {
                "[Self-collision avoidance violation]the robot would have collided with itself."
            }
            FrankaError::JointVelocityViolation => {
                "[Joint velocity violation]the robot exceeded joint velocity limits."
            }
            FrankaError::CartesianVelocityViolation => {
                "[Cartesian velocity violation]the robot exceeded Cartesian velocity limits."
            }
            FrankaError::ForceControlSafetyViolation => {
                "[Force control safety violation]the robot exceeded safety threshold during force control."
            }
            FrankaError::JointReflex => {
                "[Joint reflex]a collision was detected, i.e. the robot exceeded a torque threshold in a joint motion."
            }
            FrankaError::CartesianReflex => {
                "[Cartesian reflex]a collision was detected, i.e. the robot exceeded a torque threshold in a Cartesian motion."
            }
            FrankaError::MaxGoalPoseDeviationViolation => {
                "[Max goal pose deviation violation] internal motion generator did not reach the goal pose."
            }
            FrankaError::MaxPathPoseDeviationViolation => {
                "[Max path pose deviation violation]True if internal motion generator deviated from the path."
            }
            FrankaError::CartesianVelocityProfileSafetyViolation => {
                "[Cartesian velocity profile safety violation]True if Cartesian velocity profile for internal motions was exceeded."
            }
            FrankaError::JointPositionMotionGeneratorStartPoseInvalid => {
                "[Joint position motion generator start pose invalid]an external joint position motion generator was started with a pose too far from the current pose."
            }
            FrankaError::JointMotionGeneratorPositionLimitsViolation => {
                "[Joint motion generator position limits violation]an external joint motion generator would move into a joint limit."
            }
            FrankaError::JointMotionGeneratorVelocityLimitsViolation => {
                "[Joint motion generator velocity limits violation]an external joint motion generator exceeded velocity limits."
            }
            FrankaError::JointMotionGeneratorVelocityDiscontinuity => {
                "[Joint motion generator velocity discontinuity]commanded velocity in joint motion generators is discontinuous (target values are too far apart)."
            }
            FrankaError::JointMotionGeneratorAccelerationDiscontinuity => {
                "[Joint motion generator acceleration discontinuity]commanded acceleration in joint motion generators is discontinuous (target values are too far apart)."
            }
            FrankaError::CartesianPositionMotionGeneratorStartPoseInvalid => {
                "[Cartesian position motion generator start pose invalid]an external Cartesian position motion generator was started with a pose too far from the current pose."
            }
            FrankaError::CartesianMotionGeneratorElbowLimitViolation => {
                "[Cartesian motion generator elbow limit violation]an external Cartesian motion generator would move into an elbow limit."
            }
            FrankaError::CartesianMotionGeneratorVelocityLimitsViolation => {
                "[Cartesian motion generator velocity limits violation]an external Cartesian motion generator would move with too high velocity."
            }
            FrankaError::CartesianMotionGeneratorVelocityDiscontinuity => {
                "[Cartesian motion generator velocity discontinuity]commanded velocity in Cartesian motion generators is discontinuous (target values are too far apart)."
            }
            FrankaError::CartesianMotionGeneratorAccelerationDiscontinuity => {
                "[Cartesian motion generator acceleration discontinuity]commanded acceleration in Cartesian motion generators is discontinuous (target values are too far apart)."
            }
            FrankaError::CartesianMotionGeneratorElbowSignInconsistent => {
                "[Cartesian motion generator elbow sign inconsistent]commanded elbow values in Cartesian motion generators are inconsistent."
            }
            FrankaError::CartesianMotionGeneratorStartElbowInvalid => {
                "[Cartesian motion generator start elbow invalid]the first elbow value in Cartesian motion generators is too far from initial one."
            }
            FrankaError::ForceControllerDesiredForceToleranceViolation => {
                "[Force controller desired force tolerance violation]desired force exceeds the safety thresholds."
            }
            FrankaError::StartElbowSignInconsistent => {
                "[Start elbow sign inconsistent]the start elbow sign was inconsistent."
            }
            FrankaError::CommunicationConstraintsViolation => {
                "[Communication constraints violation]minimum network communication quality could not be held during a motion."
            }
            FrankaError::PowerLimitViolation => {
                "[Power limit violation]commanded values would result in exceeding the power limit."
            }
            FrankaError::CartesianMotionGeneratorJointPositionLimitsViolation => {
                "[Cartesian motion generator joint position limits violation]the joint position limits would be exceeded after IK calculation."
            }
            FrankaError::CartesianMotionGeneratorJointVelocityLimitsViolation => {
                "[Cartesian motion generator joint velocity limits violation]the joint velocity limits would be exceeded after IK calculation."
            }
            FrankaError::CartesianMotionGeneratorJointVelocityDiscontinuity => {
                "[Cartesian motion generator joint velocity discontinuity]the joint velocity in Cartesian motion generators is discontinuous after IK calculation."
            }
            FrankaError::CartesianMotionGeneratorJointAccelerationDiscontinuity => {
                "[Cartesian motion generator joint acceleration discontinuity]the joint acceleration in Cartesian motion generators is discontinuous after IK calculation."
            }
            FrankaError::CartesianPositionMotionGeneratorInvalidFrame => {
                "[Cartesian position motion generator invalid frame] the Cartesian pose is not a valid transformation matrix."
            }
            FrankaError::ControllerTorqueDiscontinuity => {
                "[Controller torque discontinuity]the torque set by the external controller is discontinuous."
            }
            FrankaError::JointP2PInsufficientTorqueForPlanning => {
                "[Joint P2P insufficient torque for planning]the robot is overloaded for the required motion."
            }
            FrankaError::TauJRangeViolation => {
                "[TauJ range violation]the measured torque signal is out of the safe range."
            }
            FrankaError::InstabilityDetection => {
                "[Instability detection]an instability is detected."
            }
            FrankaError::JointMoveInWrongDirection => {
                "[Joint move in wrong direction]the robot is in joint position limits violation error and the user guides the robot further towards the limit."
            }
            FrankaError::CartesianSplineViolation => {
                "[Cartesian spline violation]the generated motion violates a joint limit."
            }
            FrankaError::JointViaPlanLimitViolation => {
                "[Joint via plan limit violation]the generated motion violates a joint limit."
            }
            FrankaError::BaseAccelerationInitializationTimeout => {
                "[Base acceleration initialization timeout]the gravity vector could not be initialized by measureing the base acceleration."
            }
            FrankaError::BaseAccelerationInvalidReading => {
                "[Base acceleration invalid reading]the base acceleration O_ddP_O cannot be determined."
            }
        };
        write!(f, "{description}")
    }
}

bitflags! {
    pub struct ErrorFlag: u64{
        const JOINT_POSITION_LIMITS_VIOLATION = 1 << (FrankaError::JointPositionLimitsViolation as u64);
        const CARTESIAN_POSITION_LIMITS_VIOLATION = 1 << (FrankaError::CartesianPositionLimitsViolation as u64);
        const SELFCOLLISION_AVOIDANCE_VIOLATION = 1 << (FrankaError::SelfCollisionAvoidanceViolation as u64);
        const JOINT_VELOCITY_VIOLATION = 1 << (FrankaError::JointVelocityViolation as u64);
        const CARTESIAN_VELOCITY_VIOLATION = 1 << (FrankaError::CartesianVelocityViolation as u64);
        const FORCE_CONTROL_SAFETY_VIOLATION = 1 << (FrankaError::ForceControlSafetyViolation as u64);
        const JOINT_REFLEX = 1 << (FrankaError::JointReflex as u64);
        const CARTESIAN_REFLEX = 1 << (FrankaError::CartesianReflex as u64);
        const MAX_GOAL_POSE_DEVIATION_VIOLATION = 1 << (FrankaError::MaxGoalPoseDeviationViolation as u64);
        const MAX_PATH_POSE_DEVIATION_VIOLATION = 1 << (FrankaError::MaxPathPoseDeviationViolation as u64);
        const CARTESIAN_VELOCITY_PROFILE_SAFETY_VIOLATION = 1 << (FrankaError::CartesianVelocityProfileSafetyViolation as u64);
        const JOINT_POSITION_MOTION_GENERATOR_START_POSE_INVALID = 1 << (FrankaError::JointPositionMotionGeneratorStartPoseInvalid as u64);
        const JOINT_MOTION_GENERATOR_POSITION_LIMITS_VIOLATION = 1 << (FrankaError::JointMotionGeneratorPositionLimitsViolation as u64);
        const JOINT_MOTION_GENERATOR_VELOCITY_LIMITS_VIOLATION = 1 << (FrankaError::JointMotionGeneratorVelocityLimitsViolation as u64);
        const JOINT_MOTION_GENERATOR_VELOCITY_DISCONTINUITY = 1 << (FrankaError::JointMotionGeneratorVelocityDiscontinuity as u64);
        const JOINT_MOTION_GENERATOR_ACCELERATION_DISCONTINUITY = 1 << (FrankaError::JointMotionGeneratorAccelerationDiscontinuity as u64);
        const CARTESIAN_POSITION_MOTION_GENERATOR_START_POSE_INVALID = 1 << (FrankaError::CartesianPositionMotionGeneratorStartPoseInvalid as u64);
        const CARTESIAN_MOTION_GENERATOR_ELBOW_LIMIT_VIOLATION = 1 << (FrankaError::CartesianMotionGeneratorElbowLimitViolation as u64);
        const CARTESIAN_MOTION_GENERATOR_VELOCITY_LIMITS_VIOLATION = 1 << (FrankaError::CartesianMotionGeneratorVelocityLimitsViolation as u64);
        const CARTESIAN_MOTION_GENERATOR_VELOCITY_DISCONTINUITY = 1 << (FrankaError::CartesianMotionGeneratorVelocityDiscontinuity as u64);
        const CARTESIAN_MOTION_GENERATOR_ACCELERATION_DISCONTINUITY = 1 << (FrankaError::CartesianMotionGeneratorAccelerationDiscontinuity as u64);
        const CARTESIAN_MOTION_GENERATOR_ELBOW_SIGN_INCONSISTENT = 1 << (FrankaError::CartesianMotionGeneratorElbowSignInconsistent as u64);
        const CARTESIAN_MOTION_GENERATOR_START_ELBOW_INVALID = 1 << (FrankaError::CartesianMotionGeneratorStartElbowInvalid as u64);
        const FORCE_CONTROLLER_DESIRED_FORCE_TOLERANCE_VIOLATION = 1 << (FrankaError::ForceControllerDesiredForceToleranceViolation as u64);
        const START_ELBOW_SIGN_INCONSISTENT = 1 << (FrankaError::StartElbowSignInconsistent as u64);
        const COMMUNICATION_CONSTRAINTS_VIOLATION = 1 << (FrankaError::CommunicationConstraintsViolation as u64);
        const POWER_LIMIT_VIOLATION = 1 << (FrankaError::PowerLimitViolation as u64);
        const CARTESIAN_MOTION_GENERATOR_JOINT_POSITION_LIMITS_VIOLATION = 1 << (FrankaError::CartesianMotionGeneratorJointPositionLimitsViolation as u64);
        const CARTESIAN_MOTION_GENERATOR_JOINT_VELOCITY_LIMITS_VIOLATION = 1 << (FrankaError::CartesianMotionGeneratorJointVelocityLimitsViolation as u64);
        const CARTESIAN_MOTION_GENERATOR_JOINT_VELOCITY_DISCONTINUITY = 1 << (FrankaError::CartesianMotionGeneratorJointVelocityDiscontinuity as u64);
        const CARTESIAN_MOTION_GENERATOR_JOINT_ACCELERATION_DISCONTINUITY = 1 << (FrankaError::CartesianMotionGeneratorJointAccelerationDiscontinuity as u64);
        const CARTESIAN_POSITION_MOTION_GENERATOR_INVALID_FRAME = 1 << (FrankaError::CartesianPositionMotionGeneratorInvalidFrame as u64);
        const CONTROLLER_TORQUE_DISCONTINUITY = 1 << (FrankaError::ControllerTorqueDiscontinuity as u64);
        const JOINT_P2P_INSUFFICIENT_TORQUE_FOR_PLANNING = 1 << (FrankaError::JointP2PInsufficientTorqueForPlanning as u64);
        const TAU_J_RANGE_VIOLATION = 1 << (FrankaError::TauJRangeViolation as u64);
        const INSTABILITY_DETECTION = 1 << (FrankaError::InstabilityDetection as u64);
        const JOINT_MOVE_IN_WRONG_DIRECTION = 1 << (FrankaError::JointMoveInWrongDirection as u64);
        const CARTESIAN_SPLINE_VIOLATION = 1 << (FrankaError::CartesianSplineViolation as u64);
        const JOINT_VIA_PLAN_LIMIT_VIOLATION = 1 << (FrankaError::JointViaPlanLimitViolation as u64);
        const BASE_ACCELERATION_INITIALIZATION_TIMEOUT = 1 << (FrankaError::BaseAccelerationInitializationTimeout as u64);
        const BASE_ACCELERATION_INVALID_READING = 1 << (FrankaError::BaseAccelerationInvalidReading as u64);
    }
}

impl From<[bool; 41]> for ErrorFlag {
    fn from(value: [bool; 41]) -> Self {
        let mut flags = Self::empty();
        for (i, value) in value.iter().enumerate() {
            if *value {
                flags.insert(Self::from_bits(1 << i).unwrap());
            }
        }
        flags
    }
}

impl From<ErrorFlag> for [bool; 41] {
    fn from(value: ErrorFlag) -> Self {
        let mut flags = [false; 41];
        for (i, flag) in flags.iter_mut().enumerate() {
            *flag = value.contains(ErrorFlag::from_bits(1 << i).unwrap());
        }
        flags
    }
}

impl fmt::Display for ErrorFlag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut errors = Vec::new();
        for i in 0..41 {
            if self.contains(Self::from_bits(1 << i).unwrap()) {
                errors.push(format!("{}", FrankaError::from_u8(i).unwrap()));
            }
        }
        write!(f, "[{}]", errors.join(","))
    }
}

impl From<ErrorFlag> for Option<FrankaError> {
    fn from(value: ErrorFlag) -> Self {
        for i in 0..41 {
            if value.contains(ErrorFlag::from_bits(1 << i).unwrap()) {
                return Some(FrankaError::from_u8(i).unwrap());
            }
        }
        None
    }
}

#[cfg(test)]
mod test {}
