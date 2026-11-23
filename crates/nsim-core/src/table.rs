//! Lookup tables with linear interpolation (1D and bilinear 2D).
//!
//! Used for propeller coefficients CT/CQ(J), aerodynamic coefficients
//! CL/CD(α, δ), battery OCV(SoC), … Semantics match VDF tables: breakpoints
//! strictly increasing, and an explicit [`Extrapolation`] policy outside the
//! table — never a silent guess.

use std::fmt;

/// What to do when the input is outside the breakpoints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Extrapolation {
    /// Hold the value at the nearest end.
    #[default]
    Clamp,
    /// Continue the slope of the first/last segment.
    Linear,
    /// Return an error.
    Error,
}

/// Errors when building or evaluating a table.
#[derive(Debug, Clone, PartialEq)]
pub enum TableError {
    /// Breakpoints and values have different lengths, or there are fewer than 2.
    Shape(String),
    /// Breakpoints are not strictly increasing or contain NaN/∞.
    Breakpoints(String),
    /// A value is NaN or infinite.
    Value(String),
    /// Input outside the table with [`Extrapolation::Error`].
    OutOfRange {
        /// The input.
        x: f64,
        /// Table range.
        min: f64,
        /// Table range.
        max: f64,
    },
}

impl fmt::Display for TableError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Shape(m) | Self::Breakpoints(m) | Self::Value(m) => f.write_str(m),
            Self::OutOfRange { x, min, max } => {
                write!(f, "input {x} is outside the table range [{min}, {max}]")
            }
        }
    }
}

impl std::error::Error for TableError {}

fn check_breakpoints(name: &str, xs: &[f64]) -> Result<(), TableError> {
    if xs.len() < 2 {
        return Err(TableError::Shape(format!(
            "{name}: need at least 2 breakpoints, got {}",
            xs.len()
        )));
    }
    if xs.iter().any(|x| !x.is_finite()) {
        return Err(TableError::Breakpoints(format!(
            "{name}: breakpoints must be finite"
        )));
    }
    if let Some(i) = xs.windows(2).position(|w| w[1] <= w[0]) {
        return Err(TableError::Breakpoints(format!(
            "{name}: breakpoints must be strictly increasing (index {} → {}: {} → {})",
            i,
            i + 1,
            xs[i],
            xs[i + 1]
        )));
    }
    Ok(())
}

/// Index of the segment containing `x` and the interpolation fraction
/// (unclamped, so it is < 0 or > 1 outside the table).
fn locate(xs: &[f64], x: f64) -> (usize, f64) {
    let last = xs.len() - 2;
    let i = match xs.binary_search_by(|p| p.total_cmp(&x)) {
        Ok(i) => i.min(last),
        Err(i) => i.saturating_sub(1).min(last),
    };
    (i, (x - xs[i]) / (xs[i + 1] - xs[i]))
}

fn apply_policy(xs: &[f64], x: f64, t: f64, policy: Extrapolation) -> Result<f64, TableError> {
    if (0.0..=1.0).contains(&t) {
        return Ok(t);
    }
    match policy {
        Extrapolation::Clamp => Ok(t.clamp(0.0, 1.0)),
        Extrapolation::Linear => Ok(t),
        Extrapolation::Error => Err(TableError::OutOfRange {
            x,
            min: xs[0],
            max: xs[xs.len() - 1],
        }),
    }
}

/// `y = f(x)` by piecewise-linear interpolation.
#[derive(Debug, Clone, PartialEq)]
pub struct Table1D {
    xs: Vec<f64>,
    ys: Vec<f64>,
    extrapolation: Extrapolation,
}

impl Table1D {
    /// Builds a table; validates shape, ordering and finiteness.
    pub fn new(
        xs: Vec<f64>,
        ys: Vec<f64>,
        extrapolation: Extrapolation,
    ) -> Result<Self, TableError> {
        check_breakpoints("x", &xs)?;
        if xs.len() != ys.len() {
            return Err(TableError::Shape(format!(
                "{} breakpoints but {} values",
                xs.len(),
                ys.len()
            )));
        }
        if ys.iter().any(|y| !y.is_finite()) {
            return Err(TableError::Value("table values must be finite".into()));
        }
        Ok(Self {
            xs,
            ys,
            extrapolation,
        })
    }

    /// Evaluates the table at `x`.
    pub fn eval(&self, x: f64) -> Result<f64, TableError> {
        if x.is_nan() {
            return Err(TableError::Value("table input is NaN".into()));
        }
        let (i, t) = locate(&self.xs, x);
        let t = apply_policy(&self.xs, x, t, self.extrapolation)?;
        Ok(self.ys[i] + t * (self.ys[i + 1] - self.ys[i]))
    }

    /// Breakpoints.
    pub fn breakpoints(&self) -> &[f64] {
        &self.xs
    }
}

/// `z = f(x, y)` by bilinear interpolation on a rectangular grid.
#[derive(Debug, Clone, PartialEq)]
pub struct Table2D {
    xs: Vec<f64>,
    ys: Vec<f64>,
    /// Row-major: `values[i * ys.len() + j]` = f(xs[i], ys[j]).
    values: Vec<f64>,
    extrapolation: Extrapolation,
}

impl Table2D {
    /// Builds a table from rows: `rows[i][j]` = f(xs[i], ys[j]).
    pub fn new(
        xs: Vec<f64>,
        ys: Vec<f64>,
        rows: &[Vec<f64>],
        extrapolation: Extrapolation,
    ) -> Result<Self, TableError> {
        check_breakpoints("x", &xs)?;
        check_breakpoints("y", &ys)?;
        if rows.len() != xs.len() || rows.iter().any(|r| r.len() != ys.len()) {
            return Err(TableError::Shape(format!(
                "expected {} rows of {} values",
                xs.len(),
                ys.len()
            )));
        }
        let values: Vec<f64> = rows.iter().flatten().copied().collect();
        if values.iter().any(|v| !v.is_finite()) {
            return Err(TableError::Value("table values must be finite".into()));
        }
        Ok(Self {
            xs,
            ys,
            values,
            extrapolation,
        })
    }

    /// Evaluates the table at `(x, y)`.
    pub fn eval(&self, x: f64, y: f64) -> Result<f64, TableError> {
        if x.is_nan() || y.is_nan() {
            return Err(TableError::Value("table input is NaN".into()));
        }
        let (i, tx) = locate(&self.xs, x);
        let (j, ty) = locate(&self.ys, y);
        let tx = apply_policy(&self.xs, x, tx, self.extrapolation)?;
        let ty = apply_policy(&self.ys, y, ty, self.extrapolation)?;
        let n = self.ys.len();
        let v = |a: usize, b: usize| self.values[a * n + b];
        let low = v(i, j) + ty * (v(i, j + 1) - v(i, j));
        let high = v(i + 1, j) + ty * (v(i + 1, j + 1) - v(i + 1, j));
        Ok(low + tx * (high - low))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t1(policy: Extrapolation) -> Table1D {
        Table1D::new(vec![0.0, 1.0, 3.0], vec![10.0, 20.0, 0.0], policy).unwrap()
    }

    #[test]
    fn interpolates_and_hits_breakpoints_exactly() {
        let t = t1(Extrapolation::Clamp);
        assert_eq!(t.eval(0.0).unwrap(), 10.0);
        assert_eq!(t.eval(1.0).unwrap(), 20.0);
        assert_eq!(t.eval(3.0).unwrap(), 0.0);
        assert_eq!(t.eval(0.5).unwrap(), 15.0);
        assert_eq!(t.eval(2.0).unwrap(), 10.0);
    }

    #[test]
    fn extrapolation_policies() {
        assert_eq!(t1(Extrapolation::Clamp).eval(-5.0).unwrap(), 10.0);
        assert_eq!(t1(Extrapolation::Clamp).eval(9.0).unwrap(), 0.0);
        assert_eq!(t1(Extrapolation::Linear).eval(-1.0).unwrap(), 0.0);
        assert_eq!(t1(Extrapolation::Linear).eval(4.0).unwrap(), -10.0);
        assert!(matches!(
            t1(Extrapolation::Error).eval(3.5),
            Err(TableError::OutOfRange { .. })
        ));
    }

    #[test]
    fn invalid_tables_are_rejected() {
        let bad = |xs: Vec<f64>, ys: Vec<f64>| Table1D::new(xs, ys, Extrapolation::Clamp).is_err();
        assert!(bad(vec![0.0], vec![1.0]));
        assert!(bad(vec![0.0, 0.0], vec![1.0, 2.0]));
        assert!(bad(vec![1.0, 0.0], vec![1.0, 2.0]));
        assert!(bad(vec![0.0, 1.0], vec![1.0]));
        assert!(bad(vec![0.0, f64::NAN], vec![1.0, 2.0]));
        assert!(bad(vec![0.0, 1.0], vec![1.0, f64::INFINITY]));
        assert!(t1(Extrapolation::Clamp).eval(f64::NAN).is_err());
    }

    #[test]
    fn bilinear_matches_a_plane_exactly() {
        // f(x, y) = 2x + 3y + 1 is reproduced exactly by bilinear interpolation.
        let xs = vec![0.0, 1.0, 4.0];
        let ys = vec![-1.0, 2.0];
        let rows: Vec<Vec<f64>> = xs
            .iter()
            .map(|x| ys.iter().map(|y| 2.0 * x + 3.0 * y + 1.0).collect())
            .collect();
        let t = Table2D::new(xs, ys, &rows, Extrapolation::Linear).unwrap();
        for (x, y) in [(0.5, 0.0), (3.0, 1.5), (4.0, -1.0), (5.0, 3.0)] {
            assert!((t.eval(x, y).unwrap() - (2.0 * x + 3.0 * y + 1.0)).abs() < 1e-12);
        }
    }

    #[test]
    fn bilinear_clamps_each_axis() {
        let t = Table2D::new(
            vec![0.0, 1.0],
            vec![0.0, 1.0],
            &[vec![0.0, 1.0], vec![2.0, 3.0]],
            Extrapolation::Clamp,
        )
        .unwrap();
        assert_eq!(t.eval(-1.0, 5.0).unwrap(), 1.0);
        assert_eq!(t.eval(9.0, 9.0).unwrap(), 3.0);
    }
}
