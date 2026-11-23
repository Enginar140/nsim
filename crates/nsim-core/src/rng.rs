//! Deterministic random number streams.
//!
//! Every stochastic model (turbulence, sensor noise, …) draws from its own
//! [`RngStream`], derived from the world seed and a stable stream name. So:
//! - the same seed reproduces every random draw bit for bit on every OS;
//! - adding a new sensor does not change the noise of existing ones
//!   (Gazebo uses one global generator, where it does).
//!
//! The generator is xoshiro256** seeded through SplitMix64; Gaussian samples
//! use the Box–Muller transform with [`crate::fmath`] functions. The exact
//! output sequence is part of NSIM's compatibility contract and is pinned by
//! tests.

use crate::fmath;

/// SplitMix64 step: used to expand seeds into generator state.
fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// FNV-1a hash of a stream name: stable across Rust versions, unlike
/// `std::hash`.
fn name_hash(name: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for byte in name.bytes() {
        h ^= u64::from(byte);
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}

/// An independent, reproducible stream of random numbers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RngStream {
    s: [u64; 4],
    /// Second Box–Muller sample, kept for the next call.
    spare_normal: Option<u64>,
}

impl RngStream {
    /// The stream called `name` of the world with seed `world_seed`.
    pub fn new(world_seed: u64, name: &str) -> Self {
        let mut sm = world_seed ^ name_hash(name).rotate_left(17);
        let s = [
            splitmix64(&mut sm),
            splitmix64(&mut sm),
            splitmix64(&mut sm),
            splitmix64(&mut sm),
        ];
        Self {
            s,
            spare_normal: None,
        }
    }

    /// Next 64 uniformly distributed bits (xoshiro256**).
    pub fn next_u64(&mut self) -> u64 {
        let result = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// Uniform sample in `[0, 1)` with 53 bits of precision.
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Standard normal sample (mean 0, standard deviation 1).
    pub fn normal(&mut self) -> f64 {
        if let Some(bits) = self.spare_normal.take() {
            return f64::from_bits(bits);
        }
        // u1 in (0, 1] so ln(u1) is finite.
        let u1 = 1.0 - self.uniform();
        let u2 = self.uniform();
        let r = (-2.0 * fmath::ln(u1)).sqrt();
        let theta = 2.0 * std::f64::consts::PI * u2;
        self.spare_normal = Some((r * fmath::sin(theta)).to_bits());
        r * fmath::cos(theta)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_and_name_reproduce() {
        let mut a = RngStream::new(7, "imu0/gyro");
        let mut b = RngStream::new(7, "imu0/gyro");
        for _ in 0..1000 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn different_names_or_seeds_give_different_streams() {
        let first = |seed, name| RngStream::new(seed, name).next_u64();
        assert_ne!(first(7, "imu0/gyro"), first(7, "imu0/accel"));
        assert_ne!(first(7, "imu0/gyro"), first(8, "imu0/gyro"));
    }

    #[test]
    fn output_sequence_is_pinned() {
        // Changing these values breaks reproducibility of every saved
        // experiment; only change them deliberately with a migration note.
        let mut r = RngStream::new(42, "turbulence");
        let got: Vec<u64> = (0..3).map(|_| r.next_u64()).collect();
        assert_eq!(got, PINNED_U64);
        let n = r.normal();
        assert_eq!(n.to_bits(), PINNED_NORMAL_BITS, "normal = {n}");
    }

    const PINNED_U64: [u64; 3] = [
        14_939_526_363_748_401_801,
        1_757_333_643_213_126_207,
        6_545_126_974_325_809_903,
    ];
    const PINNED_NORMAL_BITS: u64 = 13_801_014_326_360_856_006;

    #[test]
    fn uniform_is_in_unit_interval_with_correct_mean() {
        let mut r = RngStream::new(1, "u");
        let n = 200_000;
        let mut sum = 0.0;
        for _ in 0..n {
            let u = r.uniform();
            assert!((0.0..1.0).contains(&u));
            sum += u;
        }
        assert!((sum / n as f64 - 0.5).abs() < 0.005);
    }

    #[test]
    fn normal_has_zero_mean_and_unit_variance() {
        let mut r = RngStream::new(2, "n");
        let n = 400_000;
        let (mut sum, mut sum_sq) = (0.0, 0.0);
        for _ in 0..n {
            let x = r.normal();
            sum += x;
            sum_sq += x * x;
        }
        let mean = sum / n as f64;
        let var = sum_sq / n as f64 - mean * mean;
        // Standard errors: mean ≈ 0.0016, variance ≈ 0.0022 → 4σ bounds.
        assert!(mean.abs() < 0.0065, "mean = {mean}");
        assert!((var - 1.0).abs() < 0.009, "variance = {var}");
    }
}
