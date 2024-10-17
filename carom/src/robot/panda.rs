use rumple::{
    space::Vector,
    valid::{GeoValidate, Validate},
};

pub struct PandaCc;

impl<T> Validate<Vector<7, T>> for PandaCc {
    fn is_valid_configuration(&self, _: &Vector<7, T>) -> bool {
        todo!()
    }
}

impl<T> GeoValidate<Vector<7, T>> for PandaCc {
    fn is_valid_transition(&self, _: &Vector<7, T>, _: &Vector<7, T>) -> bool {
        todo!()
    }
}
