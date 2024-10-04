use super::{Orient, Vector};

pub struct Pose3d<T> {
    pub position: Vector<3, T>,
    pub orient: Orient<T>,
}
