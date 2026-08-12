//! CAN / 机械臂诊断工具：逐步定位"无反馈 / TX ENOBUFS / 示教模式卡死"等问题。
//!
//! 用法：
//!   cargo run --release --example diagnose -- can_left
//!   （can 名缺省为 can_left）
//!
//! 诊断流程：
//!   1. 打开接口，等待机械臂反馈（收不到 => 臂未上电 / 不在该总线 / 接错口）
//!   2. 发送测试帧（固件查询 0x4AF），报告 TX 是否成功（ENOBUFS => 总线上无节点 ACK）
//!   3. 打印实时状态、固件版本、关节限位
//!   4. 若处于示教模式，尝试不失电切换；失败则 reset 重试并报告

use std::time::{Duration, Instant};

use piper_arm::protocol::v2::messages::CtrlMode;
use piper_arm::PiperInterface;

fn sleep(ms: u64) {
    std::thread::sleep(Duration::from_millis(ms));
}

fn print_status(arm: &PiperInterface) {
    let s = arm.get_arm_status();
    let enable = arm.get_arm_enable_status();
    let live = s.time_stamp > 0.0;
    println!(
        "  ts={:.1}s{} ctrl_mode={:?} teach={:?} arm_status={:?} mode_feed={:?} \
         err=0x{:04X} enabled=[{}]",
        s.time_stamp,
        if live { "" } else { "  <== 无数据!未收到机械臂反馈" },
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

fn main() -> piper_arm::Result<()> {
    let can_name = std::env::args().nth(1).unwrap_or_else(|| "can_left".into());
    println!("===== Piper 机械臂诊断 =====");
    println!("接口: {can_name}\n");

    let arm = PiperInterface::open_socketcan(&can_name).expect("open socketcan");

    // ---- 1. 等待反馈 ----
    println!("[1/5] 等待机械臂反馈 (5s)...");
    let t0 = Instant::now();
    loop {
        if arm.get_arm_status().time_stamp > 0.0 {
            println!("      已收到反馈 -> 机械臂在线");
            break;
        }
        if t0.elapsed() > Duration::from_secs(5) {
            println!(
                "      [FAIL] 5s 内无反馈。\n\
                 \t机械臂未上电 / 未完成启动 / CAN 线未接在 {can_name} 上。\n\
                 \t可用 `candump {can_name} -T 3000` 验证是否收到 0x2A1 等帧。\n\
                 \t此时若发送还报 ENOBUFS，说明总线上无节点，属正常现象。"
            );
            return Ok(());
        }
        sleep(100);
    }

    // ---- 2. TX 测试（固件查询） ----
    println!("[2/5] 发送测试帧 (0x4AF 固件查询)...");
    match arm.search_piper_firmware_version() {
        Ok(()) => println!("      TX 发送成功 (机械臂在应答)"),
        Err(e) => println!("      [FAIL] TX 失败: {e}\n      (ENOBUFS = 总线上无节点 ACK, 检查接线/上电)"),
    }
    sleep(500);
    match arm.get_piper_firmware_version() {
        Ok(v) => println!("      固件版本: {v}"),
        Err(_) => println!("      固件版本: 未获取到"),
    }

    // ---- 3. 状态 + 限位 ----
    println!("[3/5] 当前状态:");
    print_status(&arm);

    println!("     查询关节限位...");
    arm.search_all_motor_max_angle_spd()?;
    sleep(800);
    let limits = arm.get_all_motor_angle_limit_max_spd();
    for (i, l) in limits.iter().enumerate().skip(1) {
        println!(
            "       joint{}: [{:.1}°, {:.1}°]",
            i,
            l.min_angle_limit as f64 * 0.1,
            l.max_angle_limit as f64 * 0.1,
        );
    }

    // ---- 4. 退出示教模式（如需） ----
    println!("[4/5] 检查示教模式...");
    let mut s = arm.get_arm_status();
    if s.msg.ctrl_mode == CtrlMode::CanCtrl {
        println!("      已是 CAN 指令模式, 无需切换");
    } else if s.msg.ctrl_mode == CtrlMode::TeachingMode {
        println!("      示教模式: 尝试『使能保持 + 结束示教』不失电切换...");
        let _ = arm.enable_arm(7, 0x02);
        sleep(400);
        let _ = arm.exit_teaching();
        sleep(800);
        print_status(&arm);
        s = arm.get_arm_status();
        if s.msg.ctrl_mode == CtrlMode::TeachingMode {
            println!("      结束示教未生效, 改用 reset 重试...");
            for _ in 1..=3 {
                let _ = arm.reset_piper();
                sleep(1500);
                print_status(&arm);
                s = arm.get_arm_status();
                if s.msg.ctrl_mode != CtrlMode::TeachingMode {
                    break;
                }
            }
            if s.msg.ctrl_mode == CtrlMode::TeachingMode {
                println!(
                    "      [FAIL] 仍卡在示教模式。\n\
                     \t检查机械臂示教开关/按钮是否按下, 或断电重启后重试"
                );
                return Ok(());
            }
        }
    } else {
        println!("      非 CAN 模式 (ctrl_mode={:?}), 发送 reset 退出...", s.msg.ctrl_mode);
        let _ = arm.reset_piper();
        sleep(1200);
        print_status(&arm);
    }

    // ---- 5. 设模式 + 使能，校验 ----
    println!("[5/5] 设置 CAN 关节模式 + 使能电机...");
    arm.mode_ctrl(0x01, 0x01, 30, 0x00)?;
    sleep(500);
    let t0 = Instant::now();
    loop {
        arm.enable_arm(7, 0x02)?;
        sleep(200);
        if arm.get_arm_enable_status().iter().all(|&e| e) {
            break;
        }
        if t0.elapsed() > Duration::from_secs(10) {
            break;
        }
    }
    arm.mode_ctrl(0x01, 0x01, 30, 0x00)?;
    sleep(500);
    print_status(&arm);

    let s = arm.get_arm_status();
    if s.msg.ctrl_mode == CtrlMode::CanCtrl {
        println!("\n诊断通过: 机械臂已就绪, 可运行 test_move_and_gripper 等示例");
    } else {
        println!("\n诊断未完全通过: ctrl_mode={:?}, 请结合上方输出排查", s.msg.ctrl_mode);
    }
    Ok(())
}
