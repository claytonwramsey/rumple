use crate::float::Real;

/// An angle (an element of [-_π_, _π_).).
///
/// Angles can uniquely model the special orthogonal group in 2 dimensions (_SO_(2)), also known as
/// the set of 2D rotations.
///
/// You might be tempted to model orientations in 3D using a product of 3 `Angle`s.
/// **This is a bad idea!**
/// You should instead use a [`crate::space::Orient`] for 3D rotation instead.
pub struct Angle<T>(Real<T>);
