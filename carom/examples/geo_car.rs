use carom::env::World2d;
use num_traits::FloatConst;
use rand::SeedableRng;
use rand_chacha::ChaCha20Rng;
use rumple::{
    geo::RrtConnect,
    metric::SquaredEuclidean,
    nn::KdTreeMap,
    sample::Rectangle,
    space::{Angle, Pose2d, PoseRadius, Vector, WeightedPoseDistance},
    time::Solved,
    valid::SampleInterpolate,
};

fn main() {
    let mut env = World2d::new();
    // left bug trap
    env.add_aabb(0.0, 0.0, 4.0, 1.0);
    env.add_aabb(0.0, 1.0, 1.0, 4.0);
    env.add_aabb(3.0, 1.0, 4.0, 8.0);
    env.add_aabb(0.0, 5.0, 1.0, 9.0);
    env.add_aabb(1.0, 8.0, 4.0, 9.0);

    // right bug trap
    env.add_aabb(5.0, 0.0, 9.0, 1.0);
    env.add_aabb(8.0, 1.0, 9.0, 4.0);
    env.add_aabb(5.0, 1.0, 6.0, 8.0);
    env.add_aabb(5.0, 8.0, 9.0, 9.0);
    env.add_aabb(8.0, 5.0, 9.0, 8.0);

    let start = Pose2d {
        position: Vector([2.0, 3.0]),
        angle: Angle::new(f64::PI() / 2.0),
    };
    let goal = Pose2d {
        position: Vector([7.0, 3.0]),
        angle: Angle::new(f64::PI() / 2.0),
    };

    let half_w = 0.5;
    let half_h = 0.25;

    let valid = SampleInterpolate::new(
        |&Pose2d {
             position: Vector([x, y]),
             angle,
         }: &Pose2d| { !env.collides_rect(x, y, angle.get(), half_w, half_h) },
        PoseRadius {
            angle_dist: f64::PI() / 180.0,
            position_dist: 0.01,
        },
    );

    let grow_radius = PoseRadius {
        angle_dist: f64::PI() / 4.0,
        position_dist: 2.0,
    };

    let mut rrtc = RrtConnect::new(
        KdTreeMap::new(WeightedPoseDistance {
            position_metric: SquaredEuclidean,
            position_weight: 1.0,
            angle_metric: SquaredEuclidean,
            angle_weight: 1.0,
        }),
        start,
        goal,
        &valid,
    );

    let _traj = rrtc
        .grow(
            &Rectangle {
                min: Vector::new([-2.0; 2]),
                max: Vector::new([11.0; 2]),
            },
            grow_radius,
            &mut Solved::new(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
        .unwrap();

    #[cfg(feature = "std")]
    {
        println!("Created {} nodes", rrtc.num_nodes());
        println!("{_traj:?}");
    }
}