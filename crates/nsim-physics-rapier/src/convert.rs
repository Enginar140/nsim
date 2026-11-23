//! Conversions between NSIM math types (nalgebra) and Rapier's (glam).
//!
//! Conversions copy components exactly (no renormalization) so that a
//! round trip never changes a single bit.

use nalgebra::{Quaternion, Translation3, Unit};
use nsim_core::{Iso3, Quat, Vec3};
use rapier3d_f64::math::{Pose, Rotation, Vector};

pub(crate) fn vec_to_rapier(v: &Vec3) -> Vector {
    Vector::new(v.x, v.y, v.z)
}

pub(crate) fn vec_from_rapier(v: Vector) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

pub(crate) fn quat_to_rapier(q: &Quat) -> Rotation {
    Rotation::from_xyzw(q.i, q.j, q.k, q.w)
}

pub(crate) fn quat_from_rapier(q: Rotation) -> Quat {
    // Rapier keeps rotations normalized; avoid renormalizing to stay bit-exact.
    Unit::new_unchecked(Quaternion::new(q.w, q.x, q.y, q.z))
}

pub(crate) fn pose_to_rapier(p: &Iso3) -> Pose {
    Pose::from_parts(
        vec_to_rapier(&p.translation.vector),
        quat_to_rapier(&p.rotation),
    )
}

pub(crate) fn pose_from_rapier(p: &Pose) -> Iso3 {
    Iso3::from_parts(
        Translation3::from(vec_from_rapier(p.translation)),
        quat_from_rapier(p.rotation),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_round_trip_is_bit_exact() {
        let p = Iso3::from_parts(
            Translation3::new(1.25, -3.5, 1e-9),
            Quat::from_euler_angles(0.3, -1.2, 2.9),
        );
        let back = pose_from_rapier(&pose_to_rapier(&p));
        assert_eq!(back.translation, p.translation);
        assert_eq!(back.rotation.coords, p.rotation.coords);
    }

    #[test]
    fn rotation_acts_identically_in_both_libraries() {
        let q = Quat::from_euler_angles(0.4, 0.1, -0.7);
        let v = Vec3::new(0.3, -2.0, 1.1);
        let rotated_na = q * v;
        let rotated_glam = vec_from_rapier(quat_to_rapier(&q) * vec_to_rapier(&v));
        assert!((rotated_na - rotated_glam).norm() < 1e-14);
    }
}
