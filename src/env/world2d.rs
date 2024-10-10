use super::{Aabb, Ball};
use alloc::vec::Vec;
use num_traits::float::{FloatConst, FloatCore};

/// A 2-dimensional collision-checking environment.
pub struct World2d<T = f64> {
    aabbs: Vec<Aabb<2, T>>,
    balls: Vec<Ball<2, T>>,
}

impl<T> Default for World2d<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> World2d<T> {
    #[must_use]
    /// Create a new empty world.
    pub const fn new() -> Self {
        Self {
            aabbs: Vec::new(),
            balls: Vec::new(),
        }
    }
}

impl<T> World2d<T>
where
    T: FloatCore,
{
    /// Determine whether a ball at position `(x, y)` with radius `r` collides with the world.
    ///
    /// # Panics
    ///
    /// This function may panic or give incorrect results if `r < 0.0`.
    pub fn collides_ball(&self, x: T, y: T, r: T) -> bool {
        debug_assert!(T::zero() <= r, "radius of ball must be positive");
        // todo use SIMD
        self.aabbs.iter().any(
            |&Aabb {
                 los: [lx, ly],
                 his: [hx, hy],
             }| {
                let nx = x.clamp(lx, hx);
                let ny = y.clamp(ly, hy);
                ((nx - x).powi(2) + (ny - y).powi(2)) <= r.powi(2)
            },
        ) || self.balls.iter().any(
            |&Ball {
                 pos: [xb, yb],
                 r: rb,
             }| {
                let xdiff = xb - x;
                let ydiff = yb - y;
                let rpsq = rb + r;
                xdiff * xdiff + ydiff * ydiff <= rpsq * rpsq
            },
        )
    }

    /// Determine whether a point at position `(x, y)` collides with any geometry in this world.
    pub fn collides_point(&self, x: T, y: T) -> bool {
        self.aabbs.iter().any(
            |&Aabb {
                 los: [lx, ly],
                 his: [hx, hy],
             }| x >= lx && x <= hx && y >= ly && y <= hy,
        ) || self.balls.iter().any(|&Ball { pos: [xb, yb], r }| {
            let xdiff = xb - x;
            let ydiff = yb - y;
            xdiff * xdiff + ydiff * ydiff <= r * r
        })
    }

    /// Add a ball to this world at position `(x, y)` and radius `r`.
    ///
    /// # Panics
    ///
    /// This function may panic or produce incorrect results if `r < 0`.
    pub fn add_ball(&mut self, x: T, y: T, r: T) {
        debug_assert!(r >= T::zero(), "ball must have positive radius");
        self.balls.push(Ball { pos: [x, y], r });
    }

    /// Add an axis-aligned bounding box to this world with bottom-left point at `(xl, yl)` and
    /// upper-right point at `(xh, yh)`.
    ///
    /// # Panics
    ///
    /// This function may panic or produce incorrect results if `xl > xh` or `yl > yh`.
    pub fn add_aabb(&mut self, xl: T, yl: T, xh: T, yh: T) {
        debug_assert!(T::zero() <= xh - xl, "aabb must have positive width");
        debug_assert!(T::zero() <= yh - yl, "aabb must have positive height");
        self.aabbs.push(Aabb {
            los: [xl, yl],
            his: [xh, yh],
        });
    }
}

impl<T> World2d<T>
where
    T: FloatConst + Copy + num_traits::Float,
{
    #[expect(clippy::similar_names)]
    /// Determine whether a rectangle collides with any object in this world.
    /// Returns `true` if the rectangle is in collision and `false` otherwise.
    ///
    /// The rectangle is centered at position `(x, x)` and when oriented with `theta = 0` has width
    /// `2 * half_w` and height `2 * half_h`.
    ///
    /// # Panics
    ///
    /// This function may panic (but may also return an erroneous result) if `w < 0` or if `h < 0`.
    ///
    /// # Examples
    ///
    /// ```
    /// use rumple::{env::World2d, space::Angle};
    /// let mut world = World2d::new();
    ///
    /// // create ball of radius 0.5 at position (1.0, 1.0)
    /// world.add_ball(1.0, 1.0, 0.5);
    ///
    /// // rectangle centered at (0.0, 1.0) with width 1.5 and height 0.25 collides with the ball
    /// assert!(world.collides_rect(0.0, 1.0, 0.0, 1.5, 0.25));
    ///
    /// // but if we rotate the rectangle, it won't collide
    /// assert!(!world.collides_rect(0.0, 1.0, std::f64::consts::PI / 2.0, 0.75, 0.25));
    /// ```
    pub fn collides_rect(&self, x: T, y: T, theta: T, half_w: T, half_h: T) -> bool {
        debug_assert!(
            T::zero() <= half_w,
            "width of rect for collision checking must be positive"
        );
        debug_assert!(
            T::zero() <= half_h,
            "height of rect for collision checking must be positive",
        );
        let cos = theta.cos();
        let sin = theta.sin();
        self.balls.iter().any(|&Ball { pos: [xc, yc], r }| {
            let delta_x = xc - x;
            let delta_y = yc - y;

            // transform to coordinate frame of rect
            // multiply by inverse rotation matrix
            let x_trans = delta_x * cos + delta_y * sin;
            let y_trans = -delta_x * sin + delta_y * cos;

            // (x_trans, y_trans) is the position of the center of the ball
            let x_clamp = x_trans.clamp(-half_w, half_w);
            let y_clamp = y_trans.clamp(-half_h, half_h);

            // compare to closest point in rectangle body
            let x_diff = x_clamp - x_trans;
            let y_diff = y_clamp - y_trans;

            // dbg!(xc, yc, delta_x, delta_y, x_trans, y_trans, x_clamp, y_clamp, x_diff, y_diff);

            x_diff * x_diff + y_diff * y_diff <= r * r
        }) || self.aabbs.iter().any(
            |&Aabb {
                 los: [xl, yl],
                 his: [xh, yh],
             }| {
                // compute locations of corners on rect
                let diag_x = half_w * cos - half_h * sin;
                let diag_y = half_w * sin + half_h * cos;
                let x11 = x + diag_x;
                let y11 = y + diag_y;

                let x00 = x - diag_x;
                let y00 = y - diag_y;

                let anti_x = -half_w * cos - half_h * sin;
                let anti_y = -half_w * sin + half_h * cos;
                let x10 = x - anti_x;
                let y10 = y - anti_y;

                let x01 = x + anti_x;
                let y01 = y + anti_y;

                // check SAT for normal axes of AABB
                let xs = [x00, x01, x10, x11];
                if xs.into_iter().all(|x| x > xh) {
                    return false;
                }
                if xs.into_iter().all(|x| x < xl) {
                    return false;
                }
                let ys = [y00, y01, y10, y11];
                if ys.into_iter().all(|y| y > yh) {
                    return false;
                }
                if ys.into_iter().all(|y| y < yl) {
                    return false;
                }

                // transform AABB corners onto rect coordinate frame
                // first, undo translation
                let delta_xh = xh - x;
                let delta_yh = yh - y;
                let delta_xl = xl - x;
                let delta_yl = yl - y;

                // undo rotation
                let ax00_t = delta_xl * cos + delta_yl * sin;
                let ay00_t = -delta_xl * sin + delta_yl * cos;
                let ax01_t = delta_xh * cos + delta_yl * sin;
                let ay01_t = -delta_xh * sin + delta_yl * cos;
                let ax10_t = delta_xl * cos + delta_yh * sin;
                let ay10_t = -delta_xl * sin + delta_yh * cos;
                let ax11_t = delta_xh * cos + delta_yh * sin;
                let ay11_t = -delta_xh * sin + delta_yh * cos;

                // ax..t is now the x-position of an AABB corner in the coordinate frame of the rect
                let xs = [ax00_t, ax01_t, ax10_t, ax11_t];
                let ys = [ay00_t, ay01_t, ay10_t, ay11_t];

                if xs.into_iter().all(|x| x > half_w) {
                    return false;
                }
                if xs.into_iter().all(|x| x < -half_w) {
                    return false;
                }

                if ys.into_iter().all(|y| y > half_h) {
                    return false;
                }
                if ys.into_iter().all(|y| y < -half_h) {
                    return false;
                }
                true
            },
        )
    }
}

#[cfg(test)]
mod tests {

    #[test]
    #[cfg(feature = "std")]
    fn aabb_rect() {
        use super::World2d;
        use core::f64::consts::PI;
        let mut world = World2d::new();
        world.add_aabb(1.0, 2.0, 3.0, 4.0);

        // no intersecting corners
        assert!(world.collides_rect(2.0, 3.0, 0.0, 0.1, 5.0));

        // completely contained
        assert!(world.collides_rect(2.0, 3.0, 0.0, 0.1, 0.1));

        // one intersecting corner
        assert!(world.collides_rect(2.0, 4.5, PI / 4.0, 0.45, 0.45));

        // parallel and above
        assert!(!world.collides_rect(2.0, 5.1, 0.0, 0.1, 1.0));

        // no intersecting corner
        assert!(!world.collides_rect(2.0, 5.0, PI / 4.0, 0.45, 0.45));
    }
}
