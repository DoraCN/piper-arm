//! 测试程序：将机械臂移动到指定关节位姿，然后夹爪开合 5 次。
//!
//! 目标关节角（单位 deg）：
//!   joint = [5.156, 102.197, -26.090, -1.272, -69.826, 117.305]
//!
//! 用法：
//!   cargo run --release --example test_move_and_gripper -- can_left
//!   （can 名缺省为 can0，双机械臂时传 can_left / can_right）

use std::time::{Duration, Instant};

use piper_arm::protocol::v2::messages::CtrlMode;
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

    // 0a. 查询并打印本机机械臂真实关节限位（0.1° 单位）
    println!("查询关节限位...");
    arm.search_all_motor_max_angle_spd()?;
    sleep(800);
    let limits = arm.get_all_motor_angle_limit_max_spd();
    for (i, l) in limits.iter().enumerate().skip(1) {
        println!(
            "  joint{}: max={:.1}° min={:.1}° (0x473 原始: {}..{})",
            i,
            l.max_angle_limit as f64 * 0.1,
            l.min_angle_limit as f64 * 0.1,
            l.max_angle_limit,
            l.min_angle_limit,
        );
    }

    // 0b. 把目标钳制到本机限位内（0x7FFF 表示该值无效，跳过）
    let mut target = JOINT_TARGET;
    let mut clamped = false;
    for i in 0..6 {
        let l = limits[i + 1];
        if l.max_angle_limit != 0x7FFF && l.min_angle_limit != 0x7FFF {
            let lo = l.min_angle_limit as i32 * 10; // 0.1° -> 0.001°
            let hi = l.max_angle_limit as i32 * 10;
            if target[i] < lo || target[i] > hi {
                println!(
                    "  [clamp] joint{} 目标 {:.3}° 超出限位 [{:.1}°, {:.1}°]，钳制为 {:.3}°",
                    i + 1,
                    target[i] as f64 / 1000.0,
                    lo as f64 / 1000.0,
                    hi as f64 / 1000.0,
                    target[i].clamp(lo, hi) as f64 / 1000.0,
                );
                target[i] = target[i].clamp(lo, hi);
                clamped = true;
            }
        }
    }
    if clamped {
        println!("  钳制后目标: {:?}", target);
    }

    // 0. 若机械臂处于示教模式 (TeachingMode) 等非 CAN 指令模式，先复位退出
    //    reset 会使机械臂立刻失电，请确保周围无遮挡/人。
    let in_can_mode = arm.get_arm_status().msg.ctrl_mode == CtrlMode::CanCtrl;
    if !in_can_mode {
        println!("机械臂未处于 CAN 指令模式，发送复位指令退出...");
        arm.reset_piper()?;
        sleep(1000);
        print_status(&arm);
        // 复位后电机失电，等待状态稳定
        sleep(500);
    }

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

    // 2. 切换为 CAN 指令 + 关节控制模式，并校验 ctrl_mode / mode_feed
    println!("设置关节控制模式 (0x151)...");
    arm.mode_ctrl(0x01, 0x01, 30, 0x00)?;
    sleep(500);
    print_status(&arm);
    if arm.get_arm_status().msg.ctrl_mode != CtrlMode::CanCtrl {
        println!("[WARN] ctrl_mode 仍未切换为 CAN 指令模式，指令可能不被执行");
    }

    // 3. 使能夹爪（先 0x02 清错误，再 0x01 使能）
    println!("使能夹爪...");
    arm.gripper_ctrl(0, GRIPPER_EFFORT, 0x02, 0)?;
    sleep(200);
    arm.gripper_ctrl(0, GRIPPER_EFFORT, 0x01, 0)?;
    sleep(300);

    // 4. 移动到目标关节位姿，轮询反馈直到到位或超时
    println!(
        "移动至关节位姿: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}] deg",
        target[0] as f64 / 1000.0,
        target[1] as f64 / 1000.0,
        target[2] as f64 / 1000.0,
        target[3] as f64 / 1000.0,
        target[4] as f64 / 1000.0,
        target[5] as f64 / 1000.0,
    );
    let deadline = Instant::now() + MOVE_TIMEOUT;
    let mut last_mode_ok = true;
    loop {
        // 与官方 demo 一致：持续重发模式指令 + 关节指令
        arm.mode_ctrl(0x01, 0x01, 30, 0x00)?;
        arm.joint_ctrl(target[0], target[1], target[2], target[3], target[4], target[5])?;
        let s = arm.get_arm_status();
        if s.msg.ctrl_mode != CtrlMode::CanCtrl {
            last_mode_ok = false;
            println!("[WARN] 机械臂退出了 CAN 指令模式: ctrl_mode={:?}", s.msg.ctrl_mode);
            print_status(&arm);
            break;
        }
        let j = arm.get_arm_joint_msgs().msg;
        let cur = [j.joint_1, j.joint_2, j.joint_3, j.joint_4, j.joint_5, j.joint_6];
        let reached = cur
            .iter()
            .zip(target.iter())
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
    if !last_mode_ok {
        println!("[STOP] 机械臂已退出控制模式，中止后续夹爪测试");
        return Ok(());
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
