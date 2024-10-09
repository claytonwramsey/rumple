use rand::{distributions::Bernoulli, SeedableRng};
use rand_chacha::ChaCha20Rng;
use rumple::{
    geo::Rrt, metric::SquaredEuclidean, nn::KdTreeMap, sample::Rectangle, space::Vector,
    time::Solved, valid::AlwaysValid, Metric,
};

#[cfg_attr(test, test)]
fn main() {
    let mut rrt = Rrt::new(
        Vector::new([0.0, 0.0]),
        KdTreeMap::new(SquaredEuclidean),
        &AlwaysValid,
    );
    let radius = 0.05;
    let res = rrt
        .grow_toward(
            &Rectangle {
                min: Vector::new([0.0, 1.1]),
                max: Vector::new([0.0, 1.1]),
            },
            &Vector::new([1.0, 1.0]),
            radius,
            &mut Solved::new(),
            &Bernoulli::new(0.05).unwrap(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
        .unwrap();

    #[cfg(feature = "std")]
    {
        println!("Created {} nodes", rrt.num_nodes());
        println!("{res:?}");
    }
    assert!(
        res.windows(2)
            .all(|a| SquaredEuclidean.distance(&a[0], &a[1]) <= radius),
        "all transitions must be within growth radius"
    );
}
