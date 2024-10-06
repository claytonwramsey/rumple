use core::iter::Sum;

use num_traits::{float::FloatCore, FloatConst};

use crate::{geo::Interpolate, nn::KdKey, Sample};

use super::{Angle, PoseRadius, Vector};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pose2d<T> {
    pub position: Vector<2, T>,
    pub angle: Angle<T>,
}

impl<T> Interpolate for Pose2d<T>
where
    T: FloatCore + FloatConst + Sum,
{
    type Distance = PoseRadius<T>;
    fn interpolate(&self, end: &Self, radius: Self::Distance) -> Result<Self, Self> {
        let pos_res = self
            .position
            .interpolate(&end.position, radius.position_dist);
        let angle_res = self.angle.interpolate(&end.angle, radius.angle_dist);

        match (pos_res, angle_res) {
            (Err(position), Err(angle)) => Err(Self { position, angle }),
            (Ok(position) | Err(position), Ok(angle) | Err(angle)) => Ok(Self { position, angle }),
        }
    }
}

impl<T, RNG> Sample<Self, RNG> for Pose2d<T>
where
    T: Clone,
{
    fn sample(&self, _: &mut RNG) -> Self {
        self.clone()
    }
}

impl<T> KdKey for Pose2d<T>
where
    Vector<2, T>: KdKey,
    Angle<T>: KdKey,
    Self: Clone,
{
    fn dimension() -> usize {
        3
    }

    fn assign(&mut self, src: &Self, k: usize) {
        match k {
            k if k < 2 => self.position.assign(&src.position, k),
            2 => self.angle.assign(&src.angle, 0),
            _ => panic!("cannot assign dimension greater than 2"),
        };
    }

    fn compare(&self, rhs: &Self, k: usize) -> core::cmp::Ordering {
        match k {
            k if k < 2 => self.position.compare(&rhs.position, k),
            2 => self.angle.compare(&rhs.angle, 0),
            _ => panic!("cannot test dimension greater than 2"),
        }
    }

    fn lower_bound() -> Self {
        Self {
            position: Vector::lower_bound(),
            angle: Angle::lower_bound(),
        }
    }

    fn upper_bound() -> Self {
        Self {
            position: Vector::upper_bound(),
            angle: Angle::upper_bound(),
        }
    }
}
