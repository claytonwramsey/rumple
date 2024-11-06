use num_traits::{float::FloatCore, FloatConst};

use crate::{nn::KdKey, sample::Sample, space::Interpolate};

use super::{Angle, PoseRadius, Vector};

#[derive(Clone, Copy, Debug)]
/// A pose in 2 dimensions.
pub struct Pose2d<T = f64> {
    /// The translation vector.
    pub position: Vector<2, T>,
    /// The orientation.
    pub angle: Angle<T>,
}

impl<T> PartialEq for Pose2d<T>
where
    Angle<T>: PartialEq,
    Vector<2, T>: PartialEq,
{
    fn eq(&self, other: &Self) -> bool {
        self.position.eq(&other.position) && self.angle.eq(&other.angle)
    }
}

impl<T> Eq for Pose2d<T>
where
    Angle<T>: Eq,
    Vector<2, T>: Eq,
{
}

impl<T> PartialOrd for Pose2d<T>
where
    Angle<T>: PartialOrd,
    Vector<2, T>: PartialOrd,
{
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        self.position
            .partial_cmp(&other.position)
            .into_iter()
            .find_map(|ord| {
                if ord.is_eq() {
                    self.angle.partial_cmp(&other.angle)
                } else {
                    Some(ord)
                }
            })
    }
}

impl<T> Ord for Pose2d<T>
where
    Angle<T>: Ord,
    Vector<2, T>: Ord,
{
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        let c = self.position.cmp(&other.position);
        if c.is_eq() {
            self.angle.cmp(&other.angle)
        } else {
            c
        }
    }
}

impl<T> Interpolate for Pose2d<T>
where
    T: FloatCore + FloatConst,
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
