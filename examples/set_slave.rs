//! Set the arm as a motion output arm (slave) via master-slave config.
//!
//! Usage: cargo run --example set_slave -- [can_name]

use piper_arm::PiperInterface;

fn main() {
    let can_name = std::env::args().nth(1).unwrap_or_else(|| "can0".into());
    let arm = PiperInterface::open_socketcan(&can_name).expect("open socketcan");

    // 0xFC = motion output arm, offsets all 0 (default).
    arm.master_slave_config(0xFC, 0x00, 0x00, 0x00)
        .expect("master_slave_config");
    println!("arm on {can_name} set to slave (motion output) mode");
}
