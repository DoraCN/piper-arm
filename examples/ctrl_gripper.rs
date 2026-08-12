//! Enable the gripper and cycle its position (0.001 mm units).
//!
//! Usage: cargo run --example ctrl_gripper -- [can_name]

use std::time::Duration;

use piper_arm::PiperInterface;

fn main() {
    let can_name = std::env::args().nth(1).unwrap_or_else(|| "can0".into());
    let arm = PiperInterface::open_socketcan(&can_name).expect("open socketcan");

    // Enable the gripper motor (7 = all motors, 0x02 = enable).
    arm.enable_arm(7, 0x02).expect("enable arm");
    std::thread::sleep(Duration::from_millis(200));

    let mut open = true;
    loop {
        let angle_mm = if open { 60.0 } else { 0.0 };
        arm.gripper_ctrl(
            (angle_mm * 1000.0) as i32,
            1000,   // 1.0 N·m
            0x01,   // enable
            0x00,   // no zeroing
        )
        .expect("gripper_ctrl");
        println!("gripper -> {angle_mm} mm");
        open = !open;
        std::thread::sleep(Duration::from_millis(1500));
    }
}
