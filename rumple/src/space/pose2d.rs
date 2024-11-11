use core::array;

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
    T: Float + FloatConst,
{
    type Distance = PoseRadius<T>;
    type Interpolation<'a> = Pose2dInterpolation<T> where T: 'a;
    fn interpolate(&self, end: &Self, radius: Self::Distance) -> Self::Interpolation<'_> {
        let pos_dist = Euclidean.distance(&self.position, &end.position);
        let ang_dist = self.angle.signed_distance(end.angle);
        let ang_n = <usize as NumCast>::from((ang_dist.abs() / radius.angle_dist).floor()).unwrap();
        let pos_n = <usize as NumCast>::from((pos_dist / radius.position_dist).floor()).unwrap();
        let n_min = ang_n.min(pos_n);
        if n_min == 0 {
            println!("zero-step");
            return Pose2dInterpolation {
                n: 0,
                start: *self,
                step: *self,
            };
        }
        let pos_scl = radius.position_dist / pos_dist;
        let mut pos_step = array::from_fn(|i| pos_scl * (end.position[i] - self.position[i]));
        let mut ang_step = (ang_dist.signum() * radius.angle_dist).rem(T::TAU());
        assert!(
            Euclidean.distance(&Vector(pos_step), &Vector([T::zero(); 2]))
                <= radius.position_dist
                    + (-(T::one()
                        + T::one()
                        + T::one()
                        + T::one()
                        + T::one()
                        + T::one()
                        + T::one()
                        + T::one()))
                    .exp2()
        );

        let n = if ang_n < pos_n {
            ang_step = ang_step * T::from(ang_n).unwrap() / T::from(pos_n).unwrap();
            pos_n
        } else {
            let scl = T::from(pos_n).unwrap() / T::from(ang_n).unwrap();
            pos_step = pos_step.map(|x| x * scl);
            ang_n
        };
        if ang_step.is_sign_negative() {
            ang_step = ang_step + T::TAU();
        }
        assert!(ang_step.is_sign_positive() && ang_step < T::TAU());
        println!("n={n}");

        let step = Self {
            position: Vector(pos_step),
            angle: Angle::new(ang_step),
        };
        assert!(
            Euclidean.distance(&step.position, &Vector([T::zero(); 2]))
                <= radius.position_dist
                    + (-(T::one()
                        + T::one()
                        + T::one()
                        + T::one()
                        + T::one()
                        + T::one()
                        + T::one()
                        + T::one()))
                    .exp2()
        );
        Pose2dInterpolation {
            n,
            start: *self,
            step,
        }
    }
}

#[expect(clippy::module_name_repetitions)]
pub struct Pose2dInterpolation<T> {
    n: usize,
    start: Pose2d<T>,
    step: Pose2d<T>,
}

impl<T: Float + FloatConst> Iterator for Pose2dInterpolation<T> {
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
