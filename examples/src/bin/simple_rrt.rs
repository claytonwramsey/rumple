use rand::{distributions::Bernoulli, SeedableRng};
use rand_chacha::ChaCha20Rng;
use rumple::{
    float::R64,
    metric::SquaredEuclidean,
    nn::KdTreeMap,
    rrt::Rrt,
    sample::Everywhere,
    space::{LinearInterpolate, RealVector},
    time::Forever,
    AlwaysValid, Metric,
};

fn main() {
    let mut rrt = Rrt::<RealVector<2>, KdTreeMap<_, _, _>>::new(
        RealVector::from_floats([0.0, 0.0]),
        KdTreeMap::new(SquaredEuclidean),
    );
    let radius = R64::new(0.05);
    let res = rrt
        .grow_toward(
            &LinearInterpolate,
            &AlwaysValid,
            &Everywhere,
            &RealVector::from_floats([1.0, 1.0]),
            radius,
            &mut Forever,
            &Bernoulli::new(0.05).unwrap(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
        .unwrap();

    println!("Created {} nodes", rrt.num_nodes());
    println!("{res:?}");
    assert!(
        res.windows(2)
            .all(|a| SquaredEuclidean.distance(&a[0], &a[1]) <= radius),
        "all transitions must be within growth radius"
    );
}
