use nalgebra as na;

pub type Pose = na::Isometry3<f64>;

#[derive(Debug, Clone)]
pub struct ControlType<V> {
    value: V,
    motion_finished: bool,
}

impl<V> ControlType<V> {
    /// Determines whether to finish a currently running motion.
    pub fn is_finished(&self) -> bool {
        self.motion_finished
    }
    /// Sets the attribute which decide if the currently running motion should be finished
    pub fn set_motion_finished(&mut self, finished: bool) {
        self.motion_finished = finished;
    }
}

pub type Torques = ControlType<na::SVector<f64, 7>>;
pub type JointPositions = ControlType<na::SVector<f64, 7>>;
pub type JointVelocities = ControlType<na::SVector<f64, 7>>;
pub type CartesianPose = ControlType<(na::Matrix4<f64>, Option<[f64; 2]>)>;
pub type CartesianVelocities = ControlType<(na::SVector<f64, 6>, Option<[f64; 2]>)>;

impl<V> From<V> for ControlType<V> {
    fn from(value: V) -> Self {
        Self {
            value,
            motion_finished: false,
        }
    }
}

impl<const N: usize> From<&[f64; N]> for ControlType<na::SVector<f64, N>> {
    fn from(value: &[f64; N]) -> Self {
        Self {
            value: na::SVector::from_column_slice(value),
            motion_finished: false,
        }
    }
}

impl From<&[f64; 6]> for CartesianVelocities {
    fn from(value: &[f64; 6]) -> Self {
        Self {
            value: (na::SVector::from_column_slice(value), None),
            motion_finished: false,
        }
    }
}

impl From<&[f64; 16]> for CartesianPose {
    fn from(value: &[f64; 16]) -> Self {
        Self {
            value: (na::Matrix4::from_column_slice(value), None),
            motion_finished: false,
        }
    }
}

impl From<&Pose> for CartesianPose {
    fn from(value: &Pose) -> Self {
        Self {
            value: (value.to_homogeneous(), None),
            motion_finished: false,
        }
    }
}
