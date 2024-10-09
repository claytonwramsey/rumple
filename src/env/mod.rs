//! Sample environments for testing planners.
mod world2d;
mod world3d;

pub use world2d::World2d;
pub use world3d::World3d;

struct Aabb<const N: usize, T> {
    los: [T; N],
    his: [T; N],
}

struct Ball<const N: usize, T> {
    pos: [T; N],
    r: T,
}
