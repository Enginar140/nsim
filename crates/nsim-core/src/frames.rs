//! Conversions between NSIM's frames (world ENU, body FLU) and the aviation
//! frames used by flight controllers such as ArduPilot and PX4 (world NED,
//! body FRD).
//!
//! Both changes of basis are proper rotations:
//! - ENU → NED swaps x/y and negates z: `(e, n, u) → (n, e, -u)`.
//! - FLU → FRD is a 180° rotation about x: `(f, l, u) → (f, -l, -u)`.
//!
//! Getting these wrong flies the aircraft upside down or mirrored, so every
//! function here is covered by tests with physically obvious cases.

use crate::math::{Quat, Vec3};
use nalgebra::{Matrix3, Rotation3};

/// Rotation that maps ENU coordinates to NED coordinates (and back: it is its
/// own inverse).
fn enu_ned_rotation() -> Rotation3<f64> {
    Rotation3::from_matrix_unchecked(Matrix3::new(
        0.0, 1.0, 0.0, //
        1.0, 0.0, 0.0, //
        0.0, 0.0, -1.0,
    ))
}

/// Converts a world-frame vector from ENU to NED.
pub fn enu_to_ned(v: Vec3) -> Vec3 {
    Vec3::new(v.y, v.x, -v.z)
}

/// Converts a world-frame vector from NED to ENU.
pub fn ned_to_enu(v: Vec3) -> Vec3 {
    // The ENU↔NED swap is an involution.
    enu_to_ned(v)
}

/// Converts a body-frame vector from FLU to FRD.
pub fn flu_to_frd(v: Vec3) -> Vec3 {
    Vec3::new(v.x, -v.y, -v.z)
}

/// Converts a body-frame vector from FRD to FLU.
pub fn frd_to_flu(v: Vec3) -> Vec3 {
    flu_to_frd(v)
}

/// Converts a body attitude.
///
/// `q_enu_flu` rotates FLU body vectors into the ENU world frame. The result
/// rotates FRD body vectors into the NED world frame — the attitude
/// quaternion flight controllers expect.
pub fn attitude_enu_flu_to_ned_frd(q_enu_flu: Quat) -> Quat {
    let t = Quat::from_rotation_matrix(&enu_ned_rotation());
    let flu_frd = Quat::from_axis_angle(&Vec3::x_axis(), std::f64::consts::PI);
    // v_ned = T · R_enu_flu · (FRD→FLU) · v_frd
    t * q_enu_flu * flu_frd
}

/// Inverse of [`attitude_enu_flu_to_ned_frd`].
pub fn attitude_ned_frd_to_enu_flu(q_ned_frd: Quat) -> Quat {
    let t = Quat::from_rotation_matrix(&enu_ned_rotation());
    let frd_flu = Quat::from_axis_angle(&Vec3::x_axis(), std::f64::consts::PI);
    t * q_ned_frd * frd_flu
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const EPS: f64 = 1e-12;

    fn assert_vec_eq(a: Vec3, b: Vec3) {
        assert!((a - b).norm() < EPS, "{a:?} != {b:?}");
    }

    fn assert_same_rotation(a: Quat, b: Quat) {
        assert!(a.angle_to(&b) < 1e-9, "{a:?} != {b:?}");
    }

    #[test]
    fn world_vectors_convert_both_ways() {
        let north_up = Vec3::new(0.0, 5.0, 2.0); // ENU: 5 m north, 2 m up
        assert_vec_eq(enu_to_ned(north_up), Vec3::new(5.0, 0.0, -2.0));
        assert_vec_eq(ned_to_enu(enu_to_ned(north_up)), north_up);
    }

    #[test]
    fn body_vectors_convert_both_ways() {
        let fwd_left_up = Vec3::new(1.0, 2.0, 3.0);
        assert_vec_eq(flu_to_frd(fwd_left_up), Vec3::new(1.0, -2.0, -3.0));
        assert_vec_eq(frd_to_flu(flu_to_frd(fwd_left_up)), fwd_left_up);
    }

    #[test]
    fn gravity_points_down_in_both_world_frames() {
        assert_vec_eq(
            enu_to_ned(Vec3::new(0.0, 0.0, -9.81)),
            Vec3::new(0.0, 0.0, 9.81),
        );
    }

    #[test]
    fn level_aircraft_facing_north_has_identity_ned_attitude() {
        // In ENU, nose north means yaw +90° about up.
        let q_enu = Quat::from_axis_angle(&Vec3::z_axis(), FRAC_PI_2);
        assert_same_rotation(attitude_enu_flu_to_ned_frd(q_enu), Quat::identity());
    }

    #[test]
    fn level_aircraft_facing_east_has_ned_yaw_90() {
        // In ENU, nose east is the identity attitude.
        let q_ned = attitude_enu_flu_to_ned_frd(Quat::identity());
        let (roll, pitch, yaw) = q_ned.euler_angles();
        assert!(roll.abs() < EPS && pitch.abs() < EPS);
        assert!((yaw - FRAC_PI_2).abs() < EPS, "yaw = {yaw}");
    }

    #[test]
    fn attitude_conversion_maps_body_axes_consistently() {
        // For any attitude, converting a body vector then rotating must equal
        // rotating then converting the world vector.
        let q_enu = Quat::from_euler_angles(0.3, -0.2, 1.1);
        let q_ned = attitude_enu_flu_to_ned_frd(q_enu);
        let v_flu = Vec3::new(0.7, -1.3, 2.1);
        let via_enu = enu_to_ned(q_enu * v_flu);
        let via_ned = q_ned * flu_to_frd(v_flu);
        assert_vec_eq(via_enu, via_ned);
    }

    #[test]
    fn nose_up_pitch_is_positive_in_ned() {
        // Facing north, pitched nose-up 10° (positive pitch in aviation).
        let pitch = 10f64.to_radians();
        let facing_north = Quat::from_axis_angle(&Vec3::z_axis(), FRAC_PI_2);
        // In FLU, nose-up is a negative rotation about +y (left).
        let q_enu = facing_north * Quat::from_axis_angle(&Vec3::y_axis(), -pitch);
        let (_, p, _) = attitude_enu_flu_to_ned_frd(q_enu).euler_angles();
        assert!((p - pitch).abs() < 1e-12, "pitch = {p}");
    }

    #[test]
    fn attitude_round_trip() {
        let q = Quat::from_euler_angles(-0.4, 0.25, -2.0);
        assert_same_rotation(
            attitude_ned_frd_to_enu_flu(attitude_enu_flu_to_ned_frd(q)),
            q,
        );
    }
}
