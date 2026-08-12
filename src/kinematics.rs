//! Forward kinematics using DH parameters.

use std::f64::consts::PI;

/// Forward kinematics for the Piper arm.
#[derive(Clone)]
pub struct PiperForwardKinematics {
    /// radians per degree
    pub radian: f64,
    _a: [f64; 6],
    _alpha: [f64; 6],
    _theta: [f64; 6],
    _d: [f64; 6],
}

impl Default for PiperForwardKinematics {
    fn default() -> Self {
        Self::new(true)
    }
}

impl PiperForwardKinematics {
    /// `dh_is_offset` selects the new (V1.6-3+) DH parameter set when `true`.
    pub fn new(dh_is_offset: bool) -> Self {
        let radian = 180.0 / PI;
        let (a, alpha, theta, d) = if dh_is_offset {
            (
                [0.0, 0.0, 285.03, -21.98, 0.0, 0.0],
                [0.0, -PI / 2.0, 0.0, PI / 2.0, -PI / 2.0, PI / 2.0],
                [
                    0.0,
                    -PI * 172.22 / 180.0,
                    -102.78 / 180.0 * PI,
                    0.0,
                    0.0,
                    0.0,
                ],
                [123.0, 0.0, 0.0, 250.75, 0.0, 91.0],
            )
        } else {
            (
                [0.0, 0.0, 285.03, -21.98, 0.0, 0.0],
                [0.0, -PI / 2.0, 0.0, PI / 2.0, -PI / 2.0, PI / 2.0],
                [
                    0.0,
                    -PI * 174.22 / 180.0,
                    -100.78 / 180.0 * PI,
                    0.0,
                    0.0,
                    0.0,
                ],
                [123.0, 0.0, 0.0, 250.75, 0.0, 91.0],
            )
        };
        Self {
            radian,
            _a: a,
            _alpha: alpha,
            _theta: theta,
            _d: d,
        }
    }

    /// Link transformation matrix (row-major 4x4) from DH parameters.
    fn link_transform(alpha: f64, a: f64, theta: f64, d: f64) -> [f64; 16] {
        // sin_cos() returns (sin, cos).
        let (salpha, calpha) = alpha.sin_cos();
        let (stheta, ctheta) = theta.sin_cos();
        [
            ctheta,
            -stheta,
            0.0,
            a,
            stheta * calpha,
            ctheta * calpha,
            -salpha,
            -salpha * d,
            stheta * salpha,
            ctheta * salpha,
            calpha,
            calpha * d,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
    }

    fn mat_mul(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
        let mut out = [0.0; 16];
        for i in 0..4 {
            for j in 0..4 {
                let mut sum = 0.0;
                for k in 0..4 {
                    sum += a[i * 4 + k] * b[k * 4 + j];
                }
                out[i * 4 + j] = sum;
            }
        }
        out
    }

    fn matrix_to_euler(t: &[f64; 16]) -> [f64; 6] {
        // Pos = [x, y, z, rx, ry, rz] (mm, degrees)
        let mut pos = [0.0; 6];
        pos[0] = t[3];
        pos[1] = t[7];
        pos[2] = t[11];
        if t[8] < -1.0 + 0.0001 {
            pos[4] = PI / 2.0 * self_radian();
            pos[5] = 0.0;
            pos[3] = t[1].atan2(t[5]) * self_radian();
        } else if t[8] > 1.0 - 0.0001 {
            pos[4] = -PI / 2.0 * self_radian();
            pos[5] = 0.0;
            pos[3] = -t[1].atan2(t[5]) * self_radian();
        } else {
            let bt = (-t[8]).atan2((t[0] * t[0] + t[4] * t[4]).sqrt());
            pos[4] = bt * self_radian();
            pos[5] = (t[4] / bt.cos()).atan2(t[0] / bt.cos()) * self_radian();
            pos[3] = (t[9] / bt.cos()).atan2(t[10] / bt.cos()) * self_radian();
        }
        pos
    }

    /// Compute forward kinematics for a joint configuration in radians.
    ///
    /// Returns the pose `[x, y, z, rx, ry, rz]` for each of the 6 joints
    /// relative to the base (xyz in mm, rpy in degrees).
    pub fn cal_fk(&self, cur_j: &[f64; 6]) -> [[f64; 6]; 6] {
        let mut rt = [[0.0; 16]; 6];
        for i in 0..6 {
            let c_theta = cur_j[i] + self._theta[i];
            rt[i] = Self::link_transform(self._alpha[i], self._a[i], c_theta, self._d[i]);
        }
        let r02 = Self::mat_mul(&rt[0], &rt[1]);
        let r03 = Self::mat_mul(&r02, &rt[2]);
        let r04 = Self::mat_mul(&r03, &rt[3]);
        let r05 = Self::mat_mul(&r04, &rt[4]);
        let r06 = Self::mat_mul(&r05, &rt[5]);

        let mut j_pos = [[0.0; 6]; 6];
        j_pos[0] = Self::matrix_to_euler(&rt[0]);
        j_pos[1] = Self::matrix_to_euler(&r02);
        j_pos[2] = Self::matrix_to_euler(&r03);
        j_pos[3] = Self::matrix_to_euler(&r04);
        j_pos[4] = Self::matrix_to_euler(&r05);
        j_pos[5] = Self::matrix_to_euler(&r06);
        j_pos
    }
}

fn self_radian() -> f64 {
    180.0 / PI
}
