use core::{cmp::Ordering, fmt::Debug};

use num_traits::{float::Float, FloatConst, NumCast};

use crate::{
    metric::{Euclidean, Metric},
    nn::KdKey,
    sample::Sample,
    space::Interpolate,
};

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
    T: Float + FloatConst + Debug,
{
    type Distance = PoseRadius<T>;
    type Interpolation<'a>
        = Pose2dInterpolation<T>
    where
        T: 'a;
    fn interpolate(&self, end: &Self, radius: Self::Distance) -> Self::Interpolation<'_> {
        let pos_dist = Euclidean.distance(&self.position, &end.position);
        let ang_dist = self.angle.signed_distance(end.angle);
        let ang_n = <usize as NumCast>::from((ang_dist.abs() / radius.angle_dist).floor()).unwrap();
        let pos_n = <usize as NumCast>::from((pos_dist / radius.position_dist).floor()).unwrap();
        let n = match ang_n.cmp(&pos_n) {
            Ordering::Less => pos_n,
            Ordering::Greater | Ordering::Equal if ang_n != 0 => ang_n,
            _ => {
                return Pose2dInterpolation {
                    n: 0,
                    step: *self,
                    start: *self,
                }
            }
        };
        // fencepost problem - `n_gaps` is the number of empty spaces between samples and the
        // borders
        let n_gaps = <T as NumCast>::from(n + 1).unwrap();
        let mut ang_step = ang_dist / n_gaps;
        if ang_step.is_sign_negative() && !ang_step.is_zero() {
            ang_step = ang_step + T::TAU();
        }
        assert!(ang_step.is_zero() || ang_step.is_sign_positive());
        assert!(ang_step < T::TAU());
        let pos_step = [
            (end.position[0] - self.position[0]) / n_gaps,
            (end.position[1] - self.position[1]) / n_gaps,
        ];

        let step = Self {
            position: Vector(pos_step),
            angle: Angle::new(ang_step),
        };
        Pose2dInterpolation {
            n,
            start: *self,
            step,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Pose2dInterpolation<T> {
    n: usize,
    start: Pose2d<T>,
    step: Pose2d<T>,
}

impl<T: Float + FloatConst + Debug> Iterator for Pose2dInterpolation<T> {
    type Item = Pose2d<T>;
    fn next(&mut self) -> Option<Self::Item> {
        (self.n > 0).then(|| {
            self.n -= 1;
            self.start.position.0 = [
                self.start.position[0] + self.step.position[0],
                self.start.position[1] + self.step.position[1],
            ];
            self.start.angle = self.start.angle + self.step.angle;
            self.start
        })
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

#[cfg(test)]
mod tests {
    use core::f64::consts::FRAC_PI_2;

    use super::*;

    #[test]
    #[expect(clippy::unreadable_literal)]
    fn interp() {
        let x1 = Pose2d {
            position: Vector([4.19049121592329, 2.9071816974125753]),
            angle: Angle::new(0.7566420685098816),
        };
        let x2 = Pose2d {
            position: Vector([7.0, 3.0]),
            angle: Angle::new(FRAC_PI_2),
        };
        let grow_radius = PoseRadius {
            angle_dist: f64::PI() / 4.0,
            position_dist: 2.0,
        };

        let interp = x1.interpolate(&x2, grow_radius);
        let mut prev = x1;
        for new in interp {
            assert!(
                Euclidean.distance(&prev.position, &new.position)
                    <= grow_radius.position_dist + 1e-5
            );
            assert!(Euclidean.distance(&prev.angle, &new.angle) <= grow_radius.angle_dist + 1e-5);
            prev = new;
        }

        assert!(
            Euclidean.distance(&prev.position, &x2.position) <= grow_radius.position_dist + 1e-5
        );
        assert!(Euclidean.distance(&prev.angle, &x2.angle) <= grow_radius.angle_dist + 1e-5);
    }
}
