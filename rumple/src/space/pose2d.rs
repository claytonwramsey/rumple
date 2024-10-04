use core::iter::Sum;

use num_traits::float::FloatCore;

use crate::Interpolate;

use super::{Angle, PoseDistance, Vector};

pub struct Pose2d<T> {
    pub position: Vector<2, T>,
    pub angle: Angle<T>,
}

impl<T> Interpolate for Pose2d<T>
where
    T: FloatCore + Sum,
{
    type Distance = PoseDistance<T>;
    fn interpolate(&self, end: &Self, radius: Self::Distance) -> Result<Self, Self> {
        let pos_res = self
            .position
            .interpolate(&end.position, radius.position_dist);
        let angle_res = self.angle.interpolate(&end.angle, radius.angle_dist);

        match (pos_res, angle_res) {
            (Ok(position), Ok(angle)) => Ok(Self { position, angle }),
            (Ok(position) | Err(position), Ok(angle) | Err(angle)) => Err(Self { position, angle }),
        }
    }
}
