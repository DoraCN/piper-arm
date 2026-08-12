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
    let live = s.time_stamp > 0.0;
    println!(
        "  [status] ts={:.1}s{} ctrl_mode={:?} teach={:?} arm_status={:?} mode_feed={:?} \
         err=0x{:04X} enabled=[{}]",
        s.time_stamp,
        if live { "" } else { " (无数据!未收到机械臂反馈)" },
        s.msg.ctrl_mode,
        s.msg.teach_status,
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

/// 退出非 CAN 指令模式，返回是否成功进入可控制状态。
/// 示教模式先尝试不失电的 exit_teaching，无效则 reset（会失电）并重试。
fn enter_can_mode(arm: &PiperInterface) -> bool {
    let mut s = arm.get_arm_status();
    if s.msg.ctrl_mode == CtrlMode::CanCtrl {
        return true;
    }

    if s.msg.ctrl_mode == CtrlMode::TeachingMode {
        println!("示教模式：尝试『使能保持位姿 + 结束示教』不失电切换...");
        if arm.enable_arm(7, 0x02).is_ok() {
            std::thread::sleep(Duration::from_millis(400));
        }
        if arm.exit_teaching().is_ok() {
            std::thread::sleep(Duration::from_millis(800));
        }
        print_status(arm);
        s = arm.get_arm_status();
        if s.msg.ctrl_mode != CtrlMode::TeachingMode {
            return true;
        }
        println!("  结束示教未生效，改用 reset 退出（会失电）");
    } else {
        println!("机械臂未处于 CAN 指令模式，发送复位指令退出...");
    }

    // reset 重试并校验是否真的退出（若仍 TeachingMode，说明命令被忽略）
    for attempt in 1..=3 {
        println!("  reset 第 {attempt} 次...");
        if arm.reset_piper().is_ok() {
            std::thread::sleep(Duration::from_millis(1500));
        }
        print_status(arm);
        s = arm.get_arm_status();
        if s.msg.ctrl_mode != CtrlMode::TeachingMode {
            return true;
        }
    }

    println!(
        "[ERROR] 连续 reset 后机械臂仍在示教模式，控制指令被忽略。\n\
         \t请检查机械臂上的示教开关/按钮是否按下（物理锁存），\n\
         \t或使用上位机/断电重启退出示教模式后重试。"
    );
    false
}

fn main() -> piper_arm::Result<()> {
    let can_name = std::env::args().nth(1).unwrap_or_else(|| "can0".into());
    println!("打开 CAN 接口: {can_name}");
    let arm = PiperInterface::open_socketcan(&can_name).expect("open socketcan");
    sleep(300);

    // 0a. 等待第一帧反馈：收不到说明机械臂未上电/不在该 CAN 总线上
    println!("等待机械臂反馈...");
    let t0 = Instant::now();
    loop {
        if arm.get_arm_status().time_stamp > 0.0 {
            println!("  已收到机械臂反馈");
            break;
        }
        if t0.elapsed() > Duration::from_secs(5) {
            println!(
                "[ERROR] 5 秒内未收到任何机械臂反馈。\n\
                 \t请确认:\n\
                 \t  1. 机械臂已上电并完成启动\n\
                 \t  2. 机械臂 CAN 线接在 {can_name} 对应的 USB-CAN 转接器上\n\
                 \t  3. 可先用 candump {can_name} -T 3000 验证能否收到 0x2A1 等帧"
            );
            return Ok(());
        }
        sleep(100);
    }
    print_status(&arm);

    // 0. 退出非 CAN 指令模式（示教模式优先不失电切换，失败则 reset 重试）
    if !enter_can_mode(&arm) {
        return Ok(());
    }

    // 1. 先设置 CAN 指令 + 关节控制模式（官方文档顺序：reset -> 设模式 -> 使能）
    println!("设置关节控制模式 (0x151)...");
    arm.mode_ctrl(0x01, 0x01, 30, 0x00)?;
    sleep(500);

    // 2. 使能电机：持续发送直到反馈确认全部使能（超时 10s）
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
            break;
        }
    }
    sleep(300);

    // 3. 再次下发模式并校验（使能后可能需要重发一次 0x151）
    arm.mode_ctrl(0x01, 0x01, 30, 0x00)?;
    sleep(500);
    print_status(&arm);
    if arm.get_arm_status().msg.ctrl_mode != CtrlMode::CanCtrl {
        println!("[WARN] ctrl_mode 未切换为 CAN 指令模式，指令可能不被执行");
    }

    // 4a. 查询并打印本机机械臂真实关节限位（0.1° 单位）。
    //     注意：示教模式下机械臂不响应 0x472 查询，务必在 CAN 模式下查。
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

    // 4b. 把目标钳制到本机限位内。
    //     0x7FFF 或 0/0 表示限位无效/未返回，跳过该关节的钳制。
    let mut target = JOINT_TARGET;
    let mut clamped = false;
    for i in 0..6 {
        let l = limits[i + 1];
        if l.max_angle_limit == 0x7FFF
            || l.min_angle_limit == 0x7FFF
            || (l.max_angle_limit == 0 && l.min_angle_limit == 0)
        {
            continue;
        }
        let lo = l.min_angle_limit as i32 * 100; // 0.1° -> 0.001°
        let hi = l.max_angle_limit as i32 * 100;
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
    if clamped {
        println!("  钳制后目标: {:?}", target);
    }

    // 5. 使能夹爪（先 0x02 清错误，再 0x01 使能）
    println!("使能夹爪...");
    arm.gripper_ctrl(0, GRIPPER_EFFORT, 0x02, 0)?;
    sleep(200);
    arm.gripper_ctrl(0, GRIPPER_EFFORT, 0x01, 0)?;
    sleep(300);

    // 6. 移动到目标关节位姿，轮询反馈直到到位或超时
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

    // 5. 夹爪张开/关闭 5 次：每次张开后先读反馈，再关闭后读反馈
    for i in 1..=GRIPPER_CYCLES {
        println!("[{}] 张开夹爪 -> {} mm", i, GRIPPER_OPEN_MM as f64 / 1000.0);
        arm.gripper_ctrl(GRIPPER_OPEN_MM, GRIPPER_EFFORT, 0x01, 0)?;
        let open_mm = wait_gripper(&arm, GRIPPER_OPEN_MM, Duration::from_secs(3));
        println!("      张开到位反馈: {:.3} mm", open_mm);

        println!("[{}] 关闭夹爪 -> {} mm", i, GRIPPER_CLOSE_MM as f64 / 1000.0);
        arm.gripper_ctrl(GRIPPER_CLOSE_MM, GRIPPER_EFFORT, 0x01, 0)?;
        let close_mm = wait_gripper(&arm, GRIPPER_CLOSE_MM, Duration::from_secs(3));
        println!("      关闭到位反馈: {:.3} mm", close_mm);

        let joint = arm.get_arm_joint_msgs();
        println!(
            "      关节: [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}] deg",
            joint.msg.joint_1 as f64 / 1000.0,
            joint.msg.joint_2 as f64 / 1000.0,
            joint.msg.joint_3 as f64 / 1000.0,
            joint.msg.joint_4 as f64 / 1000.0,
            joint.msg.joint_5 as f64 / 1000.0,
            joint.msg.joint_6 as f64 / 1000.0,
        );
    }

    println!("测试完成");
    Ok(())
}

/// 持续下发夹爪目标并轮询反馈，直到夹爪到位或超时；返回最后读取的行程（mm）。
fn wait_gripper(arm: &PiperInterface, target_0_001mm: i32, timeout: Duration) -> f64 {
    let deadline = Instant::now() + timeout;
    loop {
        let g = arm.get_arm_gripper_msgs().msg;
        let last = g.grippers_angle as f64 / 1000.0;
        if (g.grippers_angle - target_0_001mm).abs() <= 1_000 {
            return last; // 到位（1mm 容差）
        }
        if Instant::now() > deadline {
            println!("      [WARN] 夹爪未到位，当前反馈 {:.3} mm", last);
            return last;
        }
        sleep(20);
    }
}
