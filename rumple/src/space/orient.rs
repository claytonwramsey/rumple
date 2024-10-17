#[repr(C)]
/// An orientation in 3D.
pub struct Orient<T> {
    x: T,
    y: T,
    z: T,
    w: T,
}
