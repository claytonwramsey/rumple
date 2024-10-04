mod angle;
mod orient;
mod pose2d;
mod pose3d;
mod vector;

pub use angle::Angle;
pub use orient::Orient;
pub use pose2d::Pose2d;
pub use pose3d::Pose3d;
pub use vector::Vector;

pub struct Product<T>(T);

/// A distance value between poses.
pub struct PoseDistance<T> {
    angle_dist: T,
    position_dist: T,
}
