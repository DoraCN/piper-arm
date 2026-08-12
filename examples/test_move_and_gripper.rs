//! 测试程序：将机械臂移动到指定关节位姿，然后夹爪开合 5 次。
//!
//! 目标关节角（单位 deg）：
//!   joint = [5.156, 102.197, -26.090, -1.272, -69.826, 117.305]
//!
//! 用法：
//!   cargo run --release --example test_move_and_gripper -- can_left
//!   （can 名缺省为 can0，双机械臂时传 can_left / can_right）

use std::time::Duration;

use piper_arm::PiperInterface;

/// 目标关节角（deg）-> 发送单位 0.001 deg
const JOINT_TARGET: [i32; 6] = [
    5_156,    //  5.156°
    102_197,  // 102.197°
    -26_090,  // -26.090°
    -1_272,   // -1.272°
    -69_826,  // -69.826°
    117_305,  // 117.305°
];

/// 夹爪开合参数（行程单位 0.001 mm，力矩单位 0.001 N·m）
const GRIPPER_OPEN_MM: i32 = 60_000; // 60 mm
const GRIPPER_CLOSE_MM: i32 = 0; // 0 mm
const GRIPPER_EFFORT: u16 = 1_000; // 1.0 N·m
const GRIPPER_CYCLES: u32 = 5;

fn sleep(ms: u64) {
    std::thread::sleep(Duration::from_millis(ms));
}

fn main() -> piper_arm::Result<()> {
    let can_name = std::env::args().nth(1).unwrap_or_else(|| "can0".into());
    println!("打开 CAN 接口: {can_name}");
    let arm = PiperInterface::open_socketcan(&can_name).expect("open socketcan");

    // 1. 使能机械臂（电机）
    println!("使能机械臂电机...");
    arm.enable_arm(7, 0x02)?;
    sleep(500);

    // 2. 切换为 CAN 指令 + 关节控制模式
    println!("设置关节控制模式...");
    arm.mode_ctrl(0x01, 0x01, 30, 0x00)?;
    sleep(200);

    // 3. 使能夹爪（先失能清错误，再使能）
    arm.gripper_ctrl(0, GRIPPER_EFFORT, 0x02, 0)?;
    arm.gripper_ctrl(0, GRIPPER_EFFORT, 0x01, 0)?;
    sleep(300);

    // 4. 移动到目标关节位姿（持续下发 6 秒，让关节到达目标）
    println!(
        "移动至关节位姿: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}] deg",
        JOINT_TARGET[0] as f64 / 1000.0,
        JOINT_TARGET[1] as f64 / 1000.0,
        JOINT_TARGET[2] as f64 / 1000.0,
        JOINT_TARGET[3] as f64 / 1000.0,
        JOINT_TARGET[4] as f64 / 1000.0,
        JOINT_TARGET[5] as f64 / 1000.0,
    );
    let deadline = std::time::Instant::now() + Duration::from_secs(6);
    while std::time::Instant::now() < deadline {
        arm.joint_ctrl(
            JOINT_TARGET[0],
            JOINT_TARGET[1],
            JOINT_TARGET[2],
            JOINT_TARGET[3],
            JOINT_TARGET[4],
            JOINT_TARGET[5],
        )?;
        sleep(10);
    }

    // 5. 夹爪张开/关闭 5 次
    for i in 1..=GRIPPER_CYCLES {
        println!("[{}] 张开夹爪 -> {} mm", i, GRIPPER_OPEN_MM as f64 / 1000.0);
        arm.gripper_ctrl(GRIPPER_OPEN_MM, GRIPPER_EFFORT, 0x01, 0)?;
        sleep(1500);

        println!("[{}] 关闭夹爪 -> {} mm", i, GRIPPER_CLOSE_MM as f64 / 1000.0);
        arm.gripper_ctrl(GRIPPER_CLOSE_MM, GRIPPER_EFFORT, 0x01, 0)?;
        sleep(1500);

        // 打印当前关节与夹爪反馈
        let joint = arm.get_arm_joint_msgs();
        let gripper = arm.get_arm_gripper_msgs();
        println!(
            "    关节: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}] deg | 夹爪: {:.3} mm",
            joint.msg.joint_1 as f64 / 1000.0,
            joint.msg.joint_2 as f64 / 1000.0,
            joint.msg.joint_3 as f64 / 1000.0,
            joint.msg.joint_4 as f64 / 1000.0,
            joint.msg.joint_5 as f64 / 1000.0,
            joint.msg.joint_6 as f64 / 1000.0,
            gripper.msg.grippers_angle as f64 / 1000.0,
        );
    }

    println!("测试完成");
    Ok(())
}
