use super::{Angle, RealVector};

pub struct Pose2d<T> {
    pub position: RealVector<2, T>,
    pub angle: Angle<T>,
}
