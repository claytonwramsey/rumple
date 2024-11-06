use core::{
    array,
    iter::Sum,
    ops::{Add, Div, Mul, Sub},
};
use std::simd::{
    cmp::{SimdPartialEq, SimdPartialOrd},
    num::SimdFloat,
    LaneCount, Mask, Simd, SimdElement, SupportedLaneCount,
};

use num_traits::float::FloatCore;
use rumple::{
    metric::{Metric, SquaredEuclidean},
    space::Vector,
};

use crate::env::World3d;

pub type States<F, const L: usize> = [Simd<F, L>; 3];

pub fn is_valid_state<F>(pos: Vector<3, F>, r: F, env: &World3d<F>) -> bool
where
    F: SimdElement,
{
    is_valid_state_simd::<F, 1>(
        Simd::splat(pos[0]),
        Simd::splat(pos[1]),
        Simd::splat(pos[2]),
        Simd::splat(r),
        env,
    )
}

pub fn is_valid_transition<F>(
    start: Vector<3, F>,
    end: Vector<3, F>,
    r: F,
    spacing: F,
    env: &World3d<F>,
) -> bool
where
    F: SimdElement,
{
    todo!()
}

/// Determine whether a transition from `start` to `end` is valid, using a spacing of `spacing`
/// distance-squared between configurations.
pub fn is_valid_transition_simd<F, const L: usize>(
    start: &Vector<3, F>,
    end: &Vector<3, F>,
    r: F,
    spacing: F,
) -> bool
where
    F: SimdElement + FloatCore + Sum,
    Simd<F, L>: Add<Output = Simd<F, L>>
        + Sub<Output = Simd<F, L>>
        + Mul<Output = Simd<F, L>>
        + Div<Output = Simd<F, L>>
        + SimdPartialOrd
        + SimdPartialEq<Mask = Mask<F::Mask, L>>
        + SimdFloat,
    LaneCount<L>: SupportedLaneCount,
{
    let distsq = SquaredEuclidean.distance(start, end);
    let rakesq = spacing;
    let start_offsets = Simd::<F, L>::from_array(array::from_fn(|i| F::from(i).unwrap()))
        / Simd::splat(F::from(L).unwrap());
    let rake_frac = rakesq / distsq;
    let [x_add, y_add, z_add] = array::from_fn(|i| Simd::splat((end[i] - start[i]) * rake_frac));
    let n_steps = u32::from((F::from(rake_frac * L).unwrap()).recip().ceil()).unwrap();
    let [mut x, mut y, mut z] =
        array::from_fn(|i| Simd::splat(start[i]) + Simd::splat(end[i] - start[i]) * start_offsets);
    let rs = Simd::splat(self.radius);
    // println!("{n_steps}");
    for _ in 0..n_steps {
        if self.env.collides_balls(x, y, z, rs) {
            return false;
        }
        x += x_add;
        y += y_add;
        z += z_add;
    }
    true
}

pub fn is_valid_state_simd<F, const L: usize>(
    x: Simd<F, L>,
    y: Simd<F, L>,
    z: Simd<F, L>,
    r: Simd<F, L>,
    env: &World3d<F>,
) -> bool
where
    F: SimdElement + FloatCore,
    Simd<F, L>: Add<Output = Simd<F, L>>
        + Sub<Output = Simd<F, L>>
        + Mul<Output = Simd<F, L>>
        + SimdPartialOrd
        + SimdPartialEq<Mask = Mask<F::Mask, L>>
        + SimdFloat,
    LaneCount<L>: SupportedLaneCount,
{
    env.collides_balls(x, y, z, r)
}
