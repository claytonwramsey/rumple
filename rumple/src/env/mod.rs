//! Sample environments for testing planners.

use alloc::vec::Vec;

mod world2d;

pub use world2d::World2d;

struct Aabbs<const N: usize, T> {
    los: [Vec<T>; N],
    his: [Vec<T>; N],
}

struct Balls<const N: usize, T> {
    pos: [Vec<T>; N],
    r: Vec<T>,
}
