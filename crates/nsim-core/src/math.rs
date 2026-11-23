//! Math types used throughout NSIM. All quantities are `f64` and SI.
//!
//! Frame convention (same as VDF): the **world frame is ENU** (x east, y north,
//! z up) and **body frames are FLU** (x forward, y left, z up), following ROS
//! REP-103. Conversions to aviation frames (NED/FRD) live in [`crate::frames`].

use std::ops::{Add, AddAssign, Neg};

pub use nalgebra;

/// 3D vector.
pub type Vec3 = nalgebra::Vector3<f64>;
/// Unit quaternion (rotation).
pub type Quat = nalgebra::UnitQuaternion<f64>;
/// Rigid transform: rotation followed by translation.
pub type Iso3 = nalgebra::Isometry3<f64>;
/// 3×3 matrix.
pub type Mat3 = nalgebra::Matrix3<f64>;

/// Linear and angular velocity of a body.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Twist {
    /// Linear velocity of the center of mass [m/s].
    pub linear: Vec3,
    /// Angular velocity [rad/s].
    pub angular: Vec3,
}

/// Force and torque acting on a body.
///
/// Unless stated otherwise a wrench is expressed in the **world frame** and
/// applied at the body's **center of mass**.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Wrench {
    /// Force [N].
    pub force: Vec3,
    /// Torque [N·m].
    pub torque: Vec3,
}

impl Wrench {
    /// A wrench with zero force and torque.
    pub const ZERO: Self = Self {
        force: Vec3::new(0.0, 0.0, 0.0),
        torque: Vec3::new(0.0, 0.0, 0.0),
    };

    /// A pure force applied at the center of mass.
    pub fn from_force(force: Vec3) -> Self {
        Self {
            force,
            torque: Vec3::zeros(),
        }
    }

    /// A force applied at `point` (world frame), relative to the center of
    /// mass `com` (world frame). The lever arm adds `r × F` torque.
    pub fn from_force_at(force: Vec3, point: Vec3, com: Vec3) -> Self {
        Self {
            force,
            torque: (point - com).cross(&force),
        }
    }

    /// `true` if all components are finite (no NaN or infinity).
    pub fn is_finite(&self) -> bool {
        self.force
            .iter()
            .chain(self.torque.iter())
            .all(|v| v.is_finite())
    }
}

impl Add for Wrench {
    type Output = Wrench;
    fn add(self, rhs: Wrench) -> Wrench {
        Wrench {
            force: self.force + rhs.force,
            torque: self.torque + rhs.torque,
        }
    }
}

impl AddAssign for Wrench {
    fn add_assign(&mut self, rhs: Wrench) {
        *self = *self + rhs;
    }
}

impl Neg for Wrench {
    type Output = Wrench;
    fn neg(self) -> Wrench {
        Wrench {
            force: -self.force,
            torque: -self.torque,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn force_at_offset_point_creates_torque() {
        // 10 N upward, applied 1 m east of the CoM → torque about -y (north).
        let w = Wrench::from_force_at(Vec3::z() * 10.0, Vec3::x(), Vec3::zeros());
        assert_eq!(w.force, Vec3::new(0.0, 0.0, 10.0));
        assert_eq!(w.torque, Vec3::new(0.0, -10.0, 0.0));
    }

    #[test]
    fn wrenches_add_and_negate() {
        let a = Wrench::from_force(Vec3::x());
        let b = Wrench {
            force: Vec3::y(),
            torque: Vec3::z(),
        };
        let mut sum = a + b;
        assert_eq!(sum.force, Vec3::new(1.0, 1.0, 0.0));
        sum += -b;
        assert_eq!(sum, a);
    }

    #[test]
    fn non_finite_wrench_is_detected() {
        assert!(Wrench::ZERO.is_finite());
        assert!(!Wrench::from_force(Vec3::new(f64::NAN, 0.0, 0.0)).is_finite());
    }
}
