//! State spaces, or configuration spaces.

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

use crate::{nn::DistanceAabb, Metric};
use num_traits::{float::FloatCore, FloatConst};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// A distance value between poses.
pub struct PoseRadius<T> {
    /// The angular distance between orientations.
    pub angle_dist: T,
    /// The distance between translations.
    pub position_dist: T,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// A distance metric for poses that takes a weighted sum of their distance in position and angle
pub struct WeightedPoseDistance<T, MP, MA> {
    /// The metric for comparing positions.
    pub position_metric: MP,
    /// The weight to apply to positions.
    pub position_weight: T,
    /// The metric for comparing angles.
    pub angle_metric: MA,
    /// The weight to apply to angles.
    pub angle_weight: T,
}

impl<T, MP, MA> Metric<Pose2d<T>> for WeightedPoseDistance<T, MP, MA>
where
    MP: Metric<Vector<2, T>, Distance = T>,
    MA: Metric<Angle<T>, Distance = T>,
    T: FloatCore + FloatConst + Ord,
{
    type Distance = T;
    fn distance(&self, c1: &Pose2d<T>, c2: &Pose2d<T>) -> Self::Distance {
        let pd = self.position_metric.distance(&c1.position, &c2.position);
        let ad = self.angle_metric.distance(&c1.angle, &c2.angle);

        self.position_weight * pd + self.angle_weight * ad
    }
}
impl<T, MP, MA> DistanceAabb<Pose2d<T>> for WeightedPoseDistance<T, MP, MA>
where
    MP: DistanceAabb<Vector<2, T>, Distance = T>,
    MA: DistanceAabb<Angle<T>, Distance = T>,
    T: FloatCore + FloatConst + Ord,
{
    fn distance_to_aabb(
        &self,
        c: &Pose2d<T>,
        aabb_lo: &Pose2d<T>,
        aabb_hi: &Pose2d<T>,
    ) -> Self::Distance {
        self.position_weight
            * self.position_metric.distance_to_aabb(
                &c.position,
                &aabb_lo.position,
                &aabb_hi.position,
            )
            + self.angle_weight
                * self
                    .angle_metric
                    .distance_to_aabb(&c.angle, &aabb_lo.angle, &aabb_hi.angle)
    }
}
