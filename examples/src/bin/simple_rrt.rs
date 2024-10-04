use rand::{distributions::Bernoulli, SeedableRng};
use rand_chacha::ChaCha20Rng;
use rumple::{
    float::r64, geo::Rrt, metric::SquaredEuclidean, nn::KdTreeMap, sample::Rectangle,
    space::Vector, time::LimitNodes, AlwaysValid, Metric,
};

fn main() {
    let mut rrt = Rrt::new(
        Vector::new([0.0, 0.0].map(r64)),
        KdTreeMap::new(SquaredEuclidean),
        &AlwaysValid,
    );
    let radius = r64(0.05);
    let res = rrt
        .grow_toward(
            &Rectangle {
                min: Vector::new([0.0, 1.1].map(r64)),
                max: Vector::new([0.0, 1.1].map(r64)),
            },
            &Vector::new([1.0, 1.0].map(r64)),
            radius,
            &mut LimitNodes::new(10_000),
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
