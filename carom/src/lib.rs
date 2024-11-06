#![cfg_attr(not(feature = "std"), no_std)]
#![feature(portable_simd)]

use core::{
    array,
    ops::{Add, Deref, DerefMut, Div, Mul, Sub},
    simd::{
        cmp::{SimdPartialEq, SimdPartialOrd},
        num::SimdFloat,
        LaneCount, Mask, Simd, SimdElement, SupportedLaneCount,
    },
};

use num_traits::{float::FloatCore, NumCast};
use rumple::{
    metric::{Metric, SquaredEuclidean},
    space::Vector,
    valid::{GeoValidate, Validate},
};

extern crate alloc;

pub mod env;
pub mod robot;

pub struct Block<const N: usize, const L: usize, F>([Simd<F, L>; N])
where
    F: SimdElement,
    LaneCount<L>: SupportedLaneCount;

impl<const N: usize, const L: usize, F> Deref for Block<N, L, F>
where
    F: SimdElement,
    LaneCount<L>: SupportedLaneCount,
{
    type Target = [Simd<F, L>; N];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize, const L: usize, F> DerefMut for Block<N, L, F>
where
    F: SimdElement,
    LaneCount<L>: SupportedLaneCount,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

pub trait Robot<const N: usize, F>
where
    F: SimdElement,
{
    type World;

    fn resolution(&self) -> F;

    fn is_valid<const L: usize>(&self, cfgs: &Block<N, L, F>, world: &Self::World) -> bool
    where
        LaneCount<L>: SupportedLaneCount,
        F: SimdElement + FloatCore,
        Simd<F, L>: Add<Output = Simd<F, L>>
            + Sub<Output = Simd<F, L>>
            + Mul<Output = Simd<F, L>>
            + SimdPartialOrd
            + SimdPartialEq<Mask = Mask<F::Mask, L>>
            + SimdFloat;
}

/// Determine whether the transition from `start` to `end` is valid by sampling points spaced
/// `resolution` apart with `L` lanes of SIMD parallelism.
/// Assumes the distance between the points is `distance`.
pub fn is_valid_transition<R: Robot<N, F>, const N: usize, const L: usize, F>(
    robot: &R,
    start: &Vector<N, F>,
    end: &Vector<N, F>,
    resolution: F,
    world: &R::World,
) -> bool
where
    F: SimdElement + FloatCore,
    LaneCount<L>: SupportedLaneCount,
    Simd<F, L>: Add<Output = Simd<F, L>>
        + Sub<Output = Simd<F, L>>
        + Mul<Output = Simd<F, L>>
        + Div<Output = Simd<F, L>>
        + SimdPartialOrd
        + SimdPartialEq<Mask = Mask<F::Mask, L>>
        + SimdFloat,
{
    let distance = SquaredEuclidean.distance(start, end);
    let l_float = <F as NumCast>::from(L).unwrap();
    let diff: Block<N, L, _> = Block(array::from_fn(|i| Simd::splat(end[i] - start[i])));

    // start from "end" state as it is more likely to have unchecked configurations
    let percents = Simd::from_array(array::from_fn(|i| {
        <F as NumCast>::from(i + 1).unwrap() / l_float
    }));

    // assumes rake is same as vector width. hack!
    let mut block: Block<N, L, _> = Block(array::from_fn(|i| {
        Simd::splat(start[i]) + percents * diff[i]
    }));

    let n_float = (distance / (l_float * resolution)).ceil().max(F::one());
    let n = <usize as NumCast>::from(n_float).unwrap();

    let backstep: Block<N, L, _> =
        Block(array::from_fn(|i| diff[i] / Simd::splat(l_float * n_float)));

    (1..n).all(|_| {
        for j in 0..N {
            block[j] -= backstep[j];
        }

        robot.is_valid(&block, world)
    })
}

pub struct Validation<R, W, const L: usize> {
    pub robot: R,
    pub world: W,
}

impl<R, W, const N: usize, const L: usize, F> Validate<Vector<N, F>> for Validation<R, W, L>
where
    R: Robot<N, F, World = W>,
    F: SimdElement + FloatCore,
    Simd<F, 1>: Add<Output = Simd<F, 1>>
        + Sub<Output = Simd<F, 1>>
        + Mul<Output = Simd<F, 1>>
        + Div<Output = Simd<F, 1>>
        + SimdPartialOrd
        + SimdPartialEq<Mask = Mask<F::Mask, 1>>
        + SimdFloat,
{
    fn is_valid_configuration(&self, c: &Vector<N, F>) -> bool {
        self.robot.is_valid(&Block(c.map(Simd::splat)), &self.world)
    }
}

impl<R, W, const N: usize, const L: usize, F> GeoValidate<Vector<N, F>> for Validation<R, W, L>
where
    R: Robot<N, F, World = W>,
    F: SimdElement + FloatCore,
    LaneCount<L>: SupportedLaneCount,
    Simd<F, L>: Add<Output = Simd<F, L>>
        + Sub<Output = Simd<F, L>>
        + Mul<Output = Simd<F, L>>
        + Div<Output = Simd<F, L>>
        + SimdPartialOrd
        + SimdPartialEq<Mask = Mask<F::Mask, L>>
        + SimdFloat,
    Simd<F, 1>: Add<Output = Simd<F, 1>>
        + Sub<Output = Simd<F, 1>>
        + Mul<Output = Simd<F, 1>>
        + Div<Output = Simd<F, 1>>
        + SimdPartialOrd
        + SimdPartialEq<Mask = Mask<F::Mask, 1>>
        + SimdFloat,
{
    fn is_valid_transition(&self, start: &Vector<N, F>, end: &Vector<N, F>) -> bool {
        is_valid_transition(
            &self.robot,
            start,
            end,
            self.robot.resolution(),
            &self.world,
        )
    }
}
