//! Quaternion <-> Euler angle conversions (sxyz axis order).

/// Normalize a quaternion `(qx, qy, qz, qw)`.
pub fn normalize_quat(qx: f64, qy: f64, qz: f64, qw: f64) -> (f64, f64, f64, f64) {
    let norm = (qx * qx + qy * qy + qz * qz + qw * qw).sqrt();
    (qx / norm, qy / norm, qz / norm, qw / norm)
}

/// Convert a quaternion `(qx, qy, qz, qw)` to Euler angles `(roll, pitch, yaw)`
/// in radians, using `sxyz` axis order.
pub fn quat_convert_euler(qx: f64, qy: f64, qz: f64, qw: f64) -> (f64, f64, f64) {
    let (qx, qy, qz, qw) = normalize_quat(qx, qy, qz, qw);
    let m = [
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ];
    let eps = 1e-10;
    let cy = (m[0][0] * m[0][0] + m[1][0] * m[1][0]).sqrt();
    if cy > eps {
        (
            m[2][1].atan2(m[2][2]),
            (-m[2][0]).atan2(cy),
            m[1][0].atan2(m[0][0]),
        )
    } else {
        (
            (-m[1][2]).atan2(m[1][1]),
            (-m[2][0]).atan2(cy),
            0.0,
        )
    }
}

/// Convert Euler angles `(roll, pitch, yaw)` in radians to a quaternion
/// `(qx, qy, qz, qw)`, using `sxyz` axis order.
pub fn euler_convert_quat(roll: f64, pitch: f64, yaw: f64) -> (f64, f64, f64, f64) {
    let (h_roll, h_pitch, h_yaw) = (roll * 0.5, pitch * 0.5, yaw * 0.5);
    let (cr, sr) = h_roll.sin_cos();
    let (cp, sp) = h_pitch.sin_cos();
    let (cy, sy) = h_yaw.sin_cos();
    let cc = cr * cy;
    let cs = cr * sy;
    let sc = sr * cy;
    let ss = sr * sy;
    (
        cp * sc - sp * cs,
        cp * ss + sp * cc,
        cp * cs - sp * sc,
        cp * cc + sp * ss,
    )
}
