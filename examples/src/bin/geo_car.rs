use std::f64::consts::PI;

use rand::{distributions::Bernoulli, SeedableRng};
use rand_chacha::ChaCha20Rng;
use rumple::{
    env::World2d,
    float::{r64, R64},
    geo::Rrt,
    metric::SquaredEuclidean,
    nn::KdTreeMap,
    sample::Rectangle,
    space::{Angle, Pose2d, PoseRadius, Vector, WeightedPoseDistance},
    time::Solved,
    valid::SampleValidate,
    Metric, NearestNeighborsMap,
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
        position: Vector([2.0, 3.0].map(r64)),
        angle: Angle::new(r64(PI / 2.0)),
    };
    let goal = Pose2d {
        position: Vector([7.0, 3.0].map(r64)),
        angle: Angle::new(r64(PI / 2.0)),
    };

    let half_w = 0.5;
    let half_h = 0.25;

    let valid = SampleValidate::new(
        |&Pose2d {
             position: Vector([x, y]),
             angle,
         }: &Pose2d<R64>| {
            !env.collides_rect(
                x.into_inner(),
                y.into_inner(),
                angle.get().into_inner(),
                half_w,
                half_h,
            )
        },
        PoseRadius {
            angle_dist: r64(PI / 90.0),
            position_dist: r64(0.3),
        },
    );

    let grow_radius = PoseRadius {
        angle_dist: r64(PI / 5.0),
        position_dist: r64(2.0),
    };
    let mut rrt = Rrt::new(
        start,
        BruteForce {
            poses: Vec::new(),
            values: Vec::new(),
            metric: WeightedPoseDistance {
                position_metric: SquaredEuclidean,
                position_weight: r64(1.0),
                angle_metric: SquaredEuclidean,
                angle_weight: r64(1.0),
            },
        },
        &valid,
    );
    // let mut rrt = Rrt::new(
    //     start,
    //     KdTreeMap::new(WeightedPoseDistance {
    //         position_metric: SquaredEuclidean,
    //         position_weight: r64(1.0),
    //         angle_metric: SquaredEuclidean,
    //         angle_weight: r64(1.0),
    //     }),
    //     &valid,
    // );
    let traj = rrt
        .grow_toward(
            &Rectangle {
                min: Vector::new([-2.0; 2].map(r64)),
                max: Vector::new([11.0; 2].map(r64)),
            },
            &goal,
            grow_radius,
            &mut Solved::new(),
            &Bernoulli::new(0.2).unwrap(),
            &mut ChaCha20Rng::seed_from_u64(2707),
        )
        .unwrap();

    println!("Created {} nodes", rrt.num_nodes());
    println!("{traj:?}");
}

struct BruteForce<V, M> {
    poses: Vec<Pose2d<R64>>,
    values: Vec<V>,
    metric: M,
}

impl<V, M> NearestNeighborsMap<Pose2d<R64>, V> for BruteForce<V, M>
where
    M: Metric<Pose2d<R64>>,
{
    fn insert(&mut self, key: Pose2d<R64>, value: V) {
        self.poses.push(key);
        self.values.push(value);
    }

    fn nearest<'q>(&'q self, key: &Pose2d<R64>) -> Option<(&'q Pose2d<R64>, &'q V)> {
        let mut best_i = 0;
        let mut best_dist = self.metric.distance(self.poses.first()?, key);
        for (i, pose) in self.poses.iter().enumerate() {
            let dist = self.metric.distance(pose, key);
            if dist < best_dist {
                best_i = i;
                best_dist = dist;
            }
        }

        Some((&self.poses[best_i], &self.values[best_i]))
    }
}
