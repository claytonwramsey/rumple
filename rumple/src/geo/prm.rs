use std::collections::HashMap;

pub struct Prm<C, NN> {
    configurations: Vec<C>,
    edges: HashMap<usize, Vec<usize>>,
    nn: NN,
}
