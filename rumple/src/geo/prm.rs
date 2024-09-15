use core::ops::Add;

use alloc::{collections::VecDeque, vec::Vec};

use crate::{Metric, RangeNearestNeighborsMap, Sample, Timeout, Validate};

pub struct Prm<C, NN> {
    configurations: Vec<C>,
    edges: Vec<Vec<usize>>,
    nn: NN,
}

mod private {
    pub struct Node(pub usize);
}
use private::Node;

impl<C, NN> Prm<C, NN> {
    pub fn new_radius<R, V, TC, S, RNG>(
        radius: R,
        timeout: &mut TC,
        valid: &V,
        sample: &S,
        mut nn: NN,
        rng: &mut RNG,
    ) -> Self
    where
        V: Validate<C>,
        NN: RangeNearestNeighborsMap<C, Node, Distance = R>,
        TC: Timeout,
        S: Sample<C, RNG>,
        C: Clone,
        R: Clone,
    {
        let mut configurations = Vec::new();
        let mut edges = Vec::new();

        while !timeout.is_over() {
            timeout.update_sample_count(1);
            let c = sample.sample(rng);
            if !valid.is_valid_configuration(&c) {
                continue;
            }
            timeout.update_node_count(1);

            edges.push(Vec::new());
            let i = edges.len();
            for n in nn
                .nearest_within_r(&c, radius.clone())
                .filter_map(|&Node(n)| {
                    valid
                        .is_valid_transition(&c, &configurations[n])
                        .then_some(n)
                })
            {
                // assume bidirectionality
                edges[i].push(n);
                edges[n].push(i);
            }
            nn.insert(c.clone(), Node(i));
            configurations.push(c);
        }

        Self {
            configurations,
            edges,
            nn,
        }
    }
}
