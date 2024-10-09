use num_traits::float::FloatCore;

use super::Ball;

pub struct World3d<T> {
    balls: Vec<Ball<3, T>>,
}

impl<T> World3d<T> {
    #[must_use]
    pub const fn new() -> Self {
        Self { balls: Vec::new() }
    }

    pub fn add_ball(&mut self, x: T, y: T, z: T, r: T) {
        self.balls.push(Ball { pos: [x, y, z], r });
    }

    pub fn collides_ball(&self, x: T, y: T, z: T, r: T) -> bool
    where
        T: FloatCore,
    {
        self.balls.iter().any(
            |&Ball {
                 pos: [xb, yb, zb],
                 r: rb,
             }| {
                let xdiff = xb - x;
                let ydiff = yb - y;
                let zdiff = zb - z;
                let rplus = rb + r;
                xdiff * xdiff + ydiff * ydiff + zdiff * zdiff <= rplus * rplus
            },
        )
    }
}

impl<T> Default for World3d<T> {
    fn default() -> Self {
        Self::new()
    }
}
