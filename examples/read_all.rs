//! Print all feedback channels once per second.
//!
//! Usage: cargo run --example read_all -- [can_name]

use std::time::Duration;

use piper_arm::PiperInterface;

fn main() {
    let can_name = std::env::args().nth(1).unwrap_or_else(|| "can0".into());
    let arm = PiperInterface::open_socketcan(&can_name).expect("open socketcan");

    loop {
        let status = arm.get_arm_status();
        let joint = arm.get_arm_joint_msgs();
        let gripper = arm.get_arm_gripper_msgs();
        let high = arm.get_motor_states();
        let low = arm.get_driver_states();

        println!("--- status ---");
        println!(
            "ctrl_mode={:?} arm_status={:?} mode_feed={:?} teach={:?} motion={:?} traj={} err=0x{:04X}",
            status.msg.ctrl_mode,
            status.msg.arm_status,
            status.msg.mode_feed,
            status.msg.teach_status,
            status.msg.motion_status,
            status.msg.trajectory_num,
            status.msg.err_code
        );
        println!(
            "joints = [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}] deg",
            joint.msg.joint_1 as f64 / 1000.0,
            joint.msg.joint_2 as f64 / 1000.0,
            joint.msg.joint_3 as f64 / 1000.0,
            joint.msg.joint_4 as f64 / 1000.0,
            joint.msg.joint_5 as f64 / 1000.0,
            joint.msg.joint_6 as f64 / 1000.0
        );
        println!(
            "gripper = {:.3} mm, {:.3} N·m, status=0x{:02X}",
            gripper.msg.grippers_angle as f64 / 1000.0,
            gripper.msg.grippers_effort as f64 / 1000.0,
            gripper.msg.status_code
        );
        for i in 0..6 {
            println!(
                "motor{}: speed={} cur={} pos={} eff={:.3} | vol={:.1}V temp={}°C enabled={}",
                i + 1,
                high[i].motor_speed,
                high[i].current,
                high[i].pos,
                high[i].effort,
                low[i].vol as f64 * 0.1,
                low[i].foc_temp,
                low[i].foc_status.driver_enable_status,
            );
        }
        println!("can fps = {:.0}", arm.get_can_fps());

        std::thread::sleep(Duration::from_secs(1));
    }
}
