use carom::env::World2d;
use num_traits::FloatConst;
use rand::{distributions::Bernoulli, SeedableRng};
use rand_chacha::ChaCha20Rng;
use rumple::{
    geo::Rrt,
    metric::{Euclidean, Metric, SquaredEuclidean},
    nn::KdTreeMap,
    sample::Rectangle,
    space::{Angle, Pose2d, PoseRadius, Vector, WeightedPoseDistance},
    time::Solved,
    valid::{SampleInterpolate, Validate},
};

#[test]
fn geo_car() {
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

    let ang_interp = f64::PI() / 180.0;
    let pos_interp = 0.01;

    #[derive(Clone, Debug)]
    struct MyValid {
        env: World2d<f64>,
        half_w: f64,
        half_h: f64,
    }

    impl Validate<Pose2d<f64>> for MyValid {
        fn is_valid_configuration(
            &self,
            &Pose2d {
                position: Vector([x, y]),
                angle,
            }: &Pose2d<f64>,
        ) -> bool {
            !self
                .env
                .collides_rect(x, y, angle.get(), self.half_w, self.half_h)
        }
    }

    let valid = SampleInterpolate::new(
        MyValid {
            env,
            half_w,
            half_h,
        },
        PoseRadius {
            angle_dist: ang_interp,
            position_dist: pos_interp,
        },
    );

    let grow_radius = PoseRadius {
        angle_dist: f64::PI() / 4.0,
        position_dist: 2.0,
    };
    let mut rrt = Rrt::new(
        start,
        KdTreeMap::new(WeightedPoseDistance {
            position_metric: SquaredEuclidean,
            position_weight: 1.0,
            angle_metric: SquaredEuclidean,
            angle_weight: 1.0,
        }),
        &valid,
    );
    let traj = rrt
        .grow_toward(
            &Rectangle {
                min: Vector::new([-2.0; 2]),
                max: Vector::new([11.0; 2]),
            },
            &goal,
            grow_radius,
            &mut Solved::new(),
            &Bernoulli::new(0.05).unwrap(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
        .unwrap();

    println!("{rrt:?}");

    println!("{traj:?}");

    for a in traj.windows(2) {
        let x1 = a[0];
        let x2 = a[1];

        dbg!(x1, x2, Euclidean.distance(&x1.position, &x2.position));
        assert!(valid.is_valid_configuration(&x1));
        assert!(Euclidean.distance(&x1.position, &x2.position) <= pos_interp + 1e-5);
        assert!(Euclidean.distance(&x1.angle, &x2.angle) <= ang_interp + 1e-5);
    }
}
