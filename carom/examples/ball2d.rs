use rand::{distributions::Bernoulli, SeedableRng};
use rand_chacha::ChaCha20Rng;
use rumple::{
    geo::Rrt,
    metric::{Metric, SquaredEuclidean},
    nn::KdTreeMap,
    sample::Rectangle,
    space::Vector,
    time::Solved,
    valid::SampleInterpolate,
};

use carom::env::World2d;

#[cfg_attr(test, test)]
fn main() {
    let mut env = World2d::new();
    env.add_aabb(-1.0, 0.0, -0.5, 1.0);
    env.add_aabb(0.5, 0.0, 1.0, 1.0);
    env.add_aabb(0.5, 1.0, 2.0, 1.5);
    env.add_aabb(-1.0, -0.5, 1.0, 0.0);

    let start = Vector::new([0.0, 0.5]);
    let goal = Vector::new([2.0, 2.0]);

    let ball_r = 0.375;

    let valid = SampleInterpolate::new(
        |rv: &Vector<2, f64>| !env.collides_ball(rv[0], rv[1], ball_r),
        0.1,
    );

    let grow_radius = 1.0;
    let mut rrt = Rrt::new(start, KdTreeMap::new(SquaredEuclidean), &valid);
    let traj = rrt
        .grow_toward(
            &Rectangle {
                min: Vector::new([-0.5, -0.5]),
                max: Vector::new([2.5, 2.5]),
            },
            &goal,
            grow_radius,
            &mut Solved::new(),
            &Bernoulli::new(0.2).unwrap(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
        .unwrap();

    #[cfg(feature = "std")]
    {
        println!("Created {} nodes", rrt.num_nodes());
        println!("{traj:?}");
    }
    assert!(
        traj.windows(2)
            .all(|a| SquaredEuclidean.distance(&a[0], &a[1]) <= grow_radius + 1e-5),
        "all transitions must be within growth radius"
    );
}
