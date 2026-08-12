//! Forward kinematics tests against known-answer values.

use piper_arm::PiperForwardKinematics;

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() < tol
}

#[test]
fn fk_matches_known_answers() {
    let fk = PiperForwardKinematics::new(true);
    let cur_j = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6];
    let res = fk.cal_fk(&cur_j);

    // Known-answer values for the V1.6-3+ (dh_is_offset) parameter set.
    let expected: [[f64; 6]; 6] = [
        [0.0, -0.0, 123.0, 0.0, -0.0, 5.729578],
        [0.0, 0.0, 123.0, 90.0, 3.679156, -174.270422],
        [-283.021533, -28.396873, 104.709835, -95.729578, 90.0, 0.0],
        [-33.265988, -3.337732, 123.495204, -91.872941, 67.070704, -85.995468],
        [-33.265988, -3.337732, 123.495204, 130.699067, 53.33465, -139.359252],
        [47.375617, -12.321322, 82.297794, -120.314181, 26.242281, -110.851857],
    ];

    let tol = 1e-4;
    for (i, row) in res.iter().enumerate() {
        for (j, v) in row.iter().enumerate() {
            assert!(
                close(*v, expected[i][j], tol),
                "joint {i} axis {j}: got {v}, expected {}",
                expected[i][j]
            );
        }
    }
}

#[test]
fn fk_old_dh_differs_from_offset() {
    let fk_new = PiperForwardKinematics::new(true);
    let fk_old = PiperForwardKinematics::new(false);
    let cur_j = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6];
    let a = fk_new.cal_fk(&cur_j);
    let b = fk_old.cal_fk(&cur_j);
    // j2/j3 offsets differ, so the tool pose must differ.
    assert!((a[5][0] - b[5][0]).abs() > 1e-6);
}
