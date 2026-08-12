//! 测试程序：将机械臂移动到指定关节位姿，然后夹爪开合 5 次。
//!
//! 目标关节角（单位 deg）：
//!   joint = [5.156, 102.197, -26.090, -1.272, -69.826, 117.305]
//!
//! 用法：
//!   cargo run --release --example test_move_and_gripper -- can_left
//!   （can 名缺省为 can0，双机械臂时传 can_left / can_right）

use std::time::{Duration, Instant};

use piper_arm::PiperInterface;

/// 目标关节角（0.001 deg 单位）：[5.156, 102.197, -26.090, -1.272, -69.826, 117.305]°
const JOINT_TARGET: [i32; 6] = [5_156, 102_197, -26_090, -1_272, -69_826, 117_305];

/// 夹爪参数（行程 0.001 mm，力矩 0.001 N·m）
const GRIPPER_OPEN_MM: i32 = 60_000;
const GRIPPER_CLOSE_MM: i32 = 0;
const GRIPPER_EFFORT: u16 = 1_000;
const GRIPPER_CYCLES: u32 = 5;

/// 关节到位判定容差（0.001 deg）
const JOINT_TOL: i32 = 3_000; // 3°
const MOVE_TIMEOUT: Duration = Duration::from_secs(15);
const ENABLE_TIMEOUT: Duration = Duration::from_secs(10);

fn sleep(ms: u64) {
    std::thread::sleep(Duration::from_millis(ms));
}

fn print_status(arm: &PiperInterface) {
    let s = arm.get_arm_status();
    let enable = arm.get_arm_enable_status();
    println!(
        "  [status] ctrl_mode={:?} arm_status={:?} mode_feed={:?} err=0x{:04X} \
         enabled=[{}]",
        s.msg.ctrl_mode,
        s.msg.arm_status,
        s.msg.mode_feed,
        s.msg.err_code,
        enable
            .iter()
            .map(|&e| if e { "1" } else { "0" })
            .collect::<Vec<_>>()
            .join(","),
    );
}

fn main() -> piper_arm::Result<()> {
    let can_name = std::env::args().nth(1).unwrap_or_else(|| "can0".into());
    println!("打开 CAN 接口: {can_name}");
    let arm = PiperInterface::open_socketcan(&can_name).expect("open socketcan");
    sleep(300);

    println!("当前机械臂状态:");
    print_status(&arm);

    // 1. 使能电机：持续发送直到反馈确认全部使能（超时 10s）
    println!("使能机械臂电机...");
    let t0 = Instant::now();
    loop {
        arm.enable_arm(7, 0x02)?;
        sleep(200);
        let enabled = arm.get_arm_enable_status();
        if enabled.iter().all(|&e| e) {
            println!("  电机已使能: {:?}", enabled);
            break;
        }
        if t0.elapsed() > ENABLE_TIMEOUT {
            println!("[WARN] 使能超时，当前状态:");
            print_status(&arm);
            println!("  继续执行，观察模式校验结果...");
            break;
        }
    }
    sleep(300);

    // 2. 切换为 CAN 指令 + 关节控制模式，并校验 mode_feed
    println!("设置关节控制模式 (0x151)...");
    arm.mode_ctrl(0x01, 0x01, 30, 0x00)?;
    sleep(500);
    print_status(&arm);

    // 3. 使能夹爪（先 0x02 清错误，再 0x01 使能）
    println!("使能夹爪...");
    arm.gripper_ctrl(0, GRIPPER_EFFORT, 0x02, 0)?;
    sleep(200);
    arm.gripper_ctrl(0, GRIPPER_EFFORT, 0x01, 0)?;
    sleep(300);

    // 4. 移动到目标关节位姿，轮询反馈直到到位或超时
    println!(
        "移动至关节位姿: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}] deg",
        JOINT_TARGET[0] as f64 / 1000.0,
        JOINT_TARGET[1] as f64 / 1000.0,
        JOINT_TARGET[2] as f64 / 1000.0,
        JOINT_TARGET[3] as f64 / 1000.0,
        JOINT_TARGET[4] as f64 / 1000.0,
        JOINT_TARGET[5] as f64 / 1000.0,
    );
    let deadline = Instant::now() + MOVE_TIMEOUT;
    loop {
        arm.joint_ctrl(
            JOINT_TARGET[0],
            JOINT_TARGET[1],
            JOINT_TARGET[2],
            JOINT_TARGET[3],
            JOINT_TARGET[4],
            JOINT_TARGET[5],
        )?;
        let j = arm.get_arm_joint_msgs().msg;
        let cur = [j.joint_1, j.joint_2, j.joint_3, j.joint_4, j.joint_5, j.joint_6];
        let reached = cur
            .iter()
            .zip(JOINT_TARGET.iter())
            .all(|(c, t)| (c - t).abs() <= JOINT_TOL);
        if reached {
            println!("  已到达目标位姿");
            break;
        }
        if Instant::now() > deadline {
            println!(
                "  [WARN] 移动超时，当前关节: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}] deg",
                cur[0] as f64 / 1000.0,
                cur[1] as f64 / 1000.0,
                cur[2] as f64 / 1000.0,
                cur[3] as f64 / 1000.0,
                cur[4] as f64 / 1000.0,
                cur[5] as f64 / 1000.0,
            );
            print_status(&arm);
            break;
        }
        sleep(10);
    }

    // 5. 夹爪张开/关闭 5 次
    for i in 1..=GRIPPER_CYCLES {
        println!("[{}] 张开夹爪 -> 60 mm", i);
        arm.gripper_ctrl(GRIPPER_OPEN_MM, GRIPPER_EFFORT, 0x01, 0)?;
        sleep(1500);

        println!("[{}] 关闭夹爪 -> 0 mm", i);
        arm.gripper_ctrl(GRIPPER_CLOSE_MM, GRIPPER_EFFORT, 0x01, 0)?;
        sleep(1500);

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
