use crate::float::Real;

#[repr(C)]
/// An orientation in 3D.
pub struct Orient<T> {
    x: Real<T>,
    y: Real<T>,
    z: Real<T>,
    w: Real<T>,
}
