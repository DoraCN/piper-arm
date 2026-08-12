//! Read joint angles, arm status and end pose at ~200 Hz.
//!
//! Usage: cargo run --example read_joint -- [can_name]

use std::time::Duration;

use piper_arm::PiperInterface;

fn main() {
    let can_name = std::env::args().nth(1).unwrap_or_else(|| "can0".into());
    let arm = PiperInterface::open_socketcan(&can_name).expect("open socketcan");

    println!("CAN: {can_name}");

    loop {
        let status = arm.get_arm_status();
        let joint = arm.get_arm_joint_msgs();
        let end = arm.get_arm_end_pose();

        println!(
            "status={:?} joint=[{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}] deg \
             end=[{:.1}, {:.1}, {:.1}] mm fps={:.0}",
            status.msg.arm_status,
            joint.msg.joint_1 as f64 / 1000.0,
            joint.msg.joint_2 as f64 / 1000.0,
            joint.msg.joint_3 as f64 / 1000.0,
            joint.msg.joint_4 as f64 / 1000.0,
            joint.msg.joint_5 as f64 / 1000.0,
            joint.msg.joint_6 as f64 / 1000.0,
            end.msg.x_axis as f64 / 1000.0,
            end.msg.y_axis as f64 / 1000.0,
            end.msg.z_axis as f64 / 1000.0,
            arm.get_can_fps(),
        );

        std::thread::sleep(Duration::from_millis(5));
    }
}
