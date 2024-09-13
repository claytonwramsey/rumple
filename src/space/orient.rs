use ordered_float::NotNan;

#[repr(C)]
/// An orientation in 3D.
pub struct Orient<T> {
    x: NotNan<T>,
    y: NotNan<T>,
    z: NotNan<T>,
    w: NotNan<T>,
}
