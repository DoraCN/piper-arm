//! Enable the arm and command joint positions.
//!
//! Sequence: enable motors -> set CAN joint mode -> send joint angles (deg).
//!
//! Usage: cargo run --example ctrl_joint -- [can_name]

use std::time::Duration;

use piper_arm::PiperInterface;

fn millideg(deg: f64) -> i32 {
    (deg * 1000.0) as i32
}

fn main() {
    let can_name = std::env::args().nth(1).unwrap_or_else(|| "can0".into());
    let arm = PiperInterface::open_socketcan(&can_name).expect("open socketcan");

    println!("enabling motors...");
    arm.enable_arm(7, 0x02).expect("enable arm");
    std::thread::sleep(Duration::from_millis(200));

    // Switch to CAN command control + joint move mode.
    arm.mode_ctrl(0x01, 0x01, 30, 0x00).expect("set mode");
    std::thread::sleep(Duration::from_millis(100));

    let mut angle = -60.0f64;
    loop {
        // All other joints held at zero.
        arm.joint_ctrl(
            millideg(angle),
            millideg(0.0),
            millideg(0.0),
            millideg(0.0),
            millideg(0.0),
            millideg(0.0),
        )
        .expect("joint_ctrl");
        angle += 0.5;
        if angle > 60.0 {
            angle = -60.0;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
}
