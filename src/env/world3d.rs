use alloc::vec::Vec;
use num_traits::float::FloatCore;

use super::{Aabb, Ball};

pub struct World3d<T> {
    balls: Vec<Ball<3, T>>,
    aabbs: Vec<Aabb<3, T>>,
}

impl<T> World3d<T> {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            balls: Vec::new(),
            aabbs: Vec::new(),
        }
    }

    pub fn add_ball(&mut self, x: T, y: T, z: T, r: T) {
        self.balls.push(Ball { pos: [x, y, z], r });
    }

    pub fn add_aabb(&mut self, xl: T, yl: T, zl: T, xh: T, yh: T, zh: T) {
        self.aabbs.push(Aabb {
            los: [xl, yl, zl],
            his: [xh, yh, zh],
        });
    }

    pub fn collides_ball(&self, x: T, y: T, z: T, r: T) -> bool
    where
        T: FloatCore,
    {
        let rsq = r * r;
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
        ) || self.aabbs.iter().any(
            |&Aabb {
                 los: [xl, yl, zl],
                 his: [xh, yh, zh],
             }| {
                let xdiff = if x < xl {
                    xl - x
                } else if x > xh {
                    x - xh
                } else {
                    T::zero()
                };

                let ydiff = if y < yl {
                    yl - y
                } else if y > yh {
                    y - yh
                } else {
                    T::zero()
                };

                let zdiff = if z < zl {
                    zl - z
                } else if z > zh {
                    z - zh
                } else {
                    T::zero()
                };

                xdiff * xdiff + ydiff * ydiff + zdiff * zdiff <= rsq
            },
        )
    }
}

impl<T> Default for World3d<T> {
    fn default() -> Self {
        Self::new()
    }
}
