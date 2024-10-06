use super::{Orient, Vector};

/// A position in 3 dimensions.
pub struct Pose3d<T> {
    /// The translation vector.
    pub position: Vector<3, T>,
    /// The orientation.
    pub orient: Orient<T>,
}
