use super::{Orient, RealVector};

pub struct Pose3d<T> {
    pub position: RealVector<3, T>,
    pub orient: Orient<T>,
}
