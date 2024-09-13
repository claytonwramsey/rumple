use rand::{distributions::Bernoulli, SeedableRng};
use rand_chacha::ChaCha20Rng;
use rumple::{
    metric::SquaredEuclidean,
    nn::KdTreeMap,
    rrt::Rrt,
    sample::Everywhere,
    space::{Interpolate, RealVector},
    time::Forever,
    AlwaysValid,
};

use ordered_float::NotNan;

fn main() {
    let mut rrt = Rrt::<RealVector<2>, KdTreeMap<_, _, _>>::new(
        RealVector::from_floats([0.0, 0.0]),
        KdTreeMap::new(SquaredEuclidean),
    );
    let res = rrt
        .grow_toward(
            &Interpolate,
            &AlwaysValid,
            &Everywhere,
            &RealVector::from_floats([10.0, 10.0]),
            NotNan::new(0.1).unwrap(),
            &mut Forever,
            &Bernoulli::new(0.05).unwrap(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
        .unwrap();

    println!("{res:?}");
}
