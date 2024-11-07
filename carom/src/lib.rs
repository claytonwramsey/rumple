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

use num_traits::{Float, NumCast};
use rumple::{
    metric::{Euclidean, Metric},
    space::Vector,
    valid::{GeoValidate, Validate},
};

extern crate alloc;

pub mod env;
pub mod robot;

pub trait SimdArithmetic<F, const L: usize>:
    Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + SimdPartialOrd
    + SimdPartialEq<Mask = Mask<F::Mask, L>>
    + SimdFloat
where
    LaneCount<L>: SupportedLaneCount,
    F: SimdElement,
{
}

impl<F, const L: usize> SimdArithmetic<F, L> for Simd<F, L>
where
    Simd<F, L>: Add<Output = Self>
        + Sub<Output = Self>
        + Mul<Output = Self>
        + Div<Output = Self>
        + SimdPartialOrd
        + SimdPartialEq<Mask = Mask<F::Mask, L>>
        + SimdFloat,
    LaneCount<L>: SupportedLaneCount,
    F: SimdElement,
{
}

pub struct Block<const N: usize, const L: usize, F>(pub [Simd<F, L>; N])
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
        F: SimdElement + Float,
        Simd<F, L>: SimdArithmetic<F, L>;
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
    F: SimdElement + Float + Float + core::fmt::Debug,
    LaneCount<L>: SupportedLaneCount,
    Simd<F, L>: SimdArithmetic<F, L>,
{
    let distance = Euclidean.distance(start, end);
    if distance < resolution {
        return true;
    }
    let l_float = <F as NumCast>::from(L).unwrap();
    let diff: [F; N] = array::from_fn(|i| end[i] - start[i]);

    // start from "end" state as it is more likely to have unchecked configurations
    let percents = Simd::from_array(array::from_fn(|j| <F as NumCast>::from(j + 1).unwrap()))
        * Simd::splat((distance - resolution) / l_float / distance);

    // assumes rake is same as vector width. hack!
    let mut block: Block<N, L, _> = Block(array::from_fn(|i| {
        Simd::splat(start[i]) + percents * Simd::splat(diff[i])
    }));

    let n_float = ((distance / resolution - F::one()) / l_float).ceil();
    let n = <usize as NumCast>::from(n_float).unwrap();
    let backstep: Vector<N, _> = Vector(array::from_fn(|i| diff[i] / (l_float * n_float)));

    for _ in 0..n {
        if !robot.is_valid(&block, world) {
            return false;
        }
        for j in 0..N {
            block[j] -= Simd::splat(backstep[j]);
        }
    }
    true
}

pub struct Rake<R, W, const L: usize> {
    pub robot: R,
    pub world: W,
}

impl<R, W, const N: usize, const L: usize, F> Validate<Vector<N, F>> for Rake<R, W, L>
where
    R: Robot<N, F, World = W>,
    F: SimdElement + Float,
    Simd<F, 1>: SimdArithmetic<F, 1>,
{
    fn is_valid_configuration(&self, c: &Vector<N, F>) -> bool {
        self.robot.is_valid(&Block(c.map(Simd::splat)), &self.world)
    }
}

impl<R, W, const N: usize, const L: usize, F> GeoValidate<Vector<N, F>> for Rake<R, W, L>
where
    R: Robot<N, F, World = W>,
    F: SimdElement + Float + Float + core::fmt::Debug,
    LaneCount<L>: SupportedLaneCount,
    Simd<F, L>: SimdArithmetic<F, L>,
    Simd<F, 1>: SimdArithmetic<F, 1>,
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
