use super::{Aabbs, Balls};
use alloc::vec::Vec;
use core::array;
use num_traits::float::FloatCore;

pub struct World2d<T = f64> {
    aabbs: Aabbs<2, T>,
    balls: Balls<2, T>,
}

impl<T> Default for World2d<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> World2d<T> {
    #[must_use]
    pub fn new() -> Self {
        Self {
            aabbs: Aabbs {
                los: array::from_fn(|_| Vec::new()),
                his: array::from_fn(|_| Vec::new()),
            },
            balls: Balls {
                pos: array::from_fn(|_| Vec::new()),
                r: Vec::new(),
            },
        }
    }
}

impl<T> World2d<T>
where
    T: FloatCore + core::fmt::Debug,
{
    pub fn collides_ball(&self, x: T, y: T, r: T) -> bool {
        // todo use SIMD
        self.aabbs.los[0]
            .iter()
            .zip(&self.aabbs.los[1])
            .zip(&self.aabbs.his[0])
            .zip(&self.aabbs.his[1])
            .any(|(((&lx, &ly), &hx), &hy)| {
                let nx = x.clamp(lx, hx);
                let ny = y.clamp(ly, hy);
                ((nx - x).powi(2) + (ny - y).powi(2)) <= r.powi(2)
            })
            || self.balls.pos[0]
                .iter()
                .zip(&self.balls.pos[1])
                .zip(&self.balls.r)
                .any(|((&xb, &yb), &rb)| {
                    let xdiff = xb - x;
                    let ydiff = yb - y;
                    let rpsq = rb + r;
                    xdiff * xdiff + ydiff * ydiff <= rpsq * rpsq
                })
    }

    pub fn collides_point(&self, x: T, y: T) -> bool {
        self.aabbs.los[0]
            .iter()
            .zip(&self.aabbs.los[1])
            .zip(&self.aabbs.his[0])
            .zip(&self.aabbs.his[1])
            .any(|(((&lx, &ly), &hx), &hy)| x >= lx && x <= hx && y >= ly && y <= hy)
            || self.balls.pos[0]
                .iter()
                .zip(&self.balls.pos[1])
                .zip(&self.balls.r)
                .any(|((&xb, &yb), &r)| {
                    let xdiff = xb - x;
                    let ydiff = yb - y;
                    xdiff * xdiff + ydiff * ydiff <= r * r
                })
    }

    pub fn add_ball(&mut self, x: T, y: T, r: T) {
        self.balls.pos[0].push(x);
        self.balls.pos[1].push(y);
        self.balls.r.push(r);
    }

    pub fn add_aabb(&mut self, xl: T, yl: T, xh: T, yh: T) {
        self.aabbs.los[0].push(xl);
        self.aabbs.los[1].push(yl);
        self.aabbs.his[0].push(xh);
        self.aabbs.his[1].push(yh);
    }
}
