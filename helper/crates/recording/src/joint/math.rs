pub(super) type V = [f64; 3];
pub(super) fn add(a: V, b: V) -> V {
    std::array::from_fn(|i| a[i] + b[i])
}
pub(super) fn sub(a: V, b: V) -> V {
    std::array::from_fn(|i| a[i] - b[i])
}
pub(super) fn mul(a: V, b: f64) -> V {
    a.map(|x| x * b)
}
pub(super) fn dot(a: V, b: V) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
pub(super) fn norm(a: V) -> f64 {
    dot(a, a).sqrt()
}
pub(super) fn unit(a: V) -> V {
    mul(a, 1. / norm(a))
}
pub(super) fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub(super) fn mv(m: [V; 3], v: V) -> V {
    m.map(|r| dot(r, v))
}
#[derive(Clone, Copy, Debug)]
pub(super) struct Q(pub [f64; 4]);
impl Q {
    pub const ID: Self = Self([1., 0., 0., 0.]);
    pub fn normalized(self) -> Self {
        let n = self.0.iter().map(|x| x * x).sum::<f64>().sqrt();
        Self(self.0.map(|x| x / n))
    }
    pub fn times(self, rhs: Self) -> Self {
        let [w, x, y, z] = self.0;
        let [a, b, c, d] = rhs.0;
        Self([
            w * a - x * b - y * c - z * d,
            w * b + x * a + y * d - z * c,
            w * c - x * d + y * a + z * b,
            w * d + x * c - y * b + z * a,
        ])
        .normalized()
    }
    pub fn inverse_rotate(self, v: V) -> V {
        let [w, x, y, z] = self.0;
        mv(
            [
                [
                    1. - 2. * y * y - 2. * z * z,
                    2. * x * y + 2. * w * z,
                    2. * x * z - 2. * w * y,
                ],
                [
                    2. * x * y - 2. * w * z,
                    1. - 2. * x * x - 2. * z * z,
                    2. * y * z + 2. * w * x,
                ],
                [
                    2. * x * z + 2. * w * y,
                    2. * y * z - 2. * w * x,
                    1. - 2. * x * x - 2. * y * y,
                ],
            ],
            v,
        )
    }
    pub fn step(self, v: V) -> Self {
        let n = norm(v);
        let f = if n > 1e-12 { (n * 0.5).sin() / n } else { 0.5 };
        self.times(Self([(n * 0.5).cos(), v[0] * f, v[1] * f, v[2] * f]))
    }
    pub fn angle(self) -> f64 {
        2. * self.0[0].abs().min(1.).acos().to_degrees()
    }
}

/// Householder QR; never form normal equations. Refuse a rank-deficient fit.
pub(super) fn least_squares(a: &[Vec<f64>], b: &[f64]) -> Result<(Vec<f64>, f64), String> {
    let m = a.len();
    let n = a.first().map_or(0, Vec::len);
    if n == 0 || m < n || b.len() != m || a.iter().any(|r| r.len() != n) {
        return Err("Invalid least-squares dimensions".into());
    }
    let mut r = a.to_vec();
    let mut y = b.to_vec();
    for k in 0..n {
        let length = (k..m).map(|i| r[i][k] * r[i][k]).sum::<f64>().sqrt();
        if !length.is_finite() || length < 1e-9 {
            return Err("Rank-deficient calibration geometry".into());
        }
        let alpha = if r[k][k] >= 0. { -length } else { length };
        let mut v: Vec<_> = (k..m).map(|i| r[i][k]).collect();
        v[0] -= alpha;
        let beta = 2. / v.iter().map(|x| x * x).sum::<f64>();
        let mut projection = vec![0.; n - k];
        for (row, weight) in r[k..].iter().zip(&v) {
            for (sum, value) in projection.iter_mut().zip(&row[k..]) {
                *sum += weight * value;
            }
        }
        for (row, weight) in r[k..].iter_mut().zip(&v) {
            for (value, sum) in row[k..].iter_mut().zip(&projection) {
                *value -= beta * sum * weight;
            }
        }
        let d = beta * (k..m).map(|i| v[i - k] * y[i]).sum::<f64>();
        for i in k..m {
            y[i] -= d * v[i - k];
        }
    }
    let mut x = vec![0.; n];
    for k in (0..n).rev() {
        x[k] = (y[k] - (k + 1..n).map(|j| r[k][j] * x[j]).sum::<f64>()) / r[k][k];
    }
    let rms = residual(a, b, &x);
    if x.iter().any(|x| !x.is_finite()) || !rms.is_finite() {
        return Err("Nonfinite fit".into());
    }
    Ok((x, rms))
}
pub(super) fn residual(a: &[Vec<f64>], b: &[f64], x: &[f64]) -> f64 {
    (a.iter()
        .zip(b)
        .map(|(row, b)| (row.iter().zip(x).map(|(a, x)| a * x).sum::<f64>() - b).powi(2))
        .sum::<f64>()
        / b.len() as f64)
        .sqrt()
}
