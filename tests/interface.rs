//! Interface integration tests using the in-memory `MockBus`.

use std::sync::Arc;
use std::time::Duration;

use piper_arm::can::MockBus;
use piper_arm::protocol::v2::messages::{
    ArmMsgFeedbackStatus, ArmStatus, CtrlMode, ModeFeed, MotionStatus, TeachStatus,
};
use piper_arm::protocol::DecodedMessage;
use piper_arm::{PiperInterface, Result};

fn wait(ms: u64) {
    std::thread::sleep(Duration::from_millis(ms));
}

/// Push a decoded message into the mock bus as a raw CAN frame.
fn push(mock: &MockBus, msg: &DecodedMessage) {
    let id = msg.can_id();
    let data = match msg {
        DecodedMessage::StatusFeedback(s) => {
            let mut d = [0u8; 8];
            d[0] = s.ctrl_mode as u8;
            d[1] = s.arm_status as u8;
            d[2] = s.mode_feed as u8;
            d[3] = s.teach_status as u8;
            d[4] = s.motion_status as u8;
            d[5] = s.trajectory_num;
            d[6..8].copy_from_slice(&s.err_code.to_be_bytes());
            d.to_vec()
        }
        DecodedMessage::JointFeedback12(a, b) => {
            let mut d = Vec::with_capacity(8);
            d.extend_from_slice(&a.to_be_bytes());
            d.extend_from_slice(&b.to_be_bytes());
            d
        }
        DecodedMessage::GripperFeedback(g) => {
            let mut d = Vec::with_capacity(8);
            d.extend_from_slice(&g.grippers_angle.to_be_bytes());
            d.extend_from_slice(&g.grippers_effort.to_be_bytes());
            d.push(g.status_code);
            d.push(0);
            d
        }
        DecodedMessage::HighSpdFeedback(n, m) => {
            let mut d = Vec::with_capacity(8);
            d.extend_from_slice(&m.motor_speed.to_be_bytes());
            d.extend_from_slice(&m.current.to_be_bytes());
            d.extend_from_slice(&m.pos.to_be_bytes());
            let _ = n;
            d
        }
        _ => vec![0u8; 8],
    };
    mock.push_incoming(id, &data);
}

#[test]
fn interface_reads_feedback() {
    let mock = Arc::new(MockBus::new());
    let arm = PiperInterface::new(mock.clone()).unwrap();

    push(&mock, &DecodedMessage::StatusFeedback(ArmMsgFeedbackStatus {
        ctrl_mode: CtrlMode::CanCtrl,
        arm_status: ArmStatus::EmergencyStop,
        mode_feed: ModeFeed::MoveJ,
        teach_status: TeachStatus::Disabled,
        motion_status: MotionStatus::Reached,
        trajectory_num: 3,
        err_code: 0x0001,
    }));
    push(&mock, &DecodedMessage::JointFeedback12(1111, -2222));

    wait(200);

    let status = arm.get_arm_status();
    assert_eq!(status.msg.ctrl_mode, CtrlMode::CanCtrl);
    assert_eq!(status.msg.arm_status, ArmStatus::EmergencyStop);
    assert!(status.msg.joint_communication_status(1));
    assert!(status.time_stamp > 0.0);

    let joint = arm.get_arm_joint_msgs();
    assert_eq!(joint.msg.joint_1, 1111);
    assert_eq!(joint.msg.joint_2, -2222);

    arm.disconnect();
}

#[test]
fn interface_sends_control() {
    let mock = Arc::new(MockBus::new());
    let arm = PiperInterface::new(mock.clone()).unwrap();

    arm.enable_arm(7, 0x02).unwrap();
    arm.joint_ctrl(12345, 0, 0, 0, 0, 0).unwrap();
    arm.gripper_ctrl(50000, 1000, 0x01, 0xAE).unwrap();

    let frames = mock.sent_frames();
    assert!(frames.iter().any(|(id, _)| *id == 0x471));
    assert!(frames.iter().any(|(id, _)| *id == 0x155));
    assert!(frames.iter().any(|(id, _)| *id == 0x159));

    // joint_ctrl sends three frames (0x155/0x156/0x157)
    let joint_frames: Vec<_> = frames.iter().filter(|(id, _)| *id == 0x155).collect();
    assert_eq!(joint_frames.len(), 1);

    arm.disconnect();
}

#[test]
fn interface_firmware_version() {
    let mock = Arc::new(MockBus::new());
    let arm = PiperInterface::new(mock.clone()).unwrap();

    // Firmware response is assembled across frames; the decoder stores raw bytes.
    mock.push_incoming(0x4AF, b"S-V1.7-");
    wait(150);
    // Version string is only matched once full bytes are available.
    let _ = arm.get_piper_firmware_version();

    mock.push_incoming(0x4AF, b"2      ");
    wait(150);
    assert_eq!(arm.get_piper_firmware_version().unwrap(), "S-V1.7-2");

    arm.disconnect();
}

#[test]
fn interface_is_ok_flips_when_silent() {
    let mock = Arc::new(MockBus::new());
    let arm = PiperInterface::new(mock.clone()).unwrap();

    // With no traffic the monitor should eventually report not-ok.
    let mut seen_not_ok = false;
    for _ in 0..60 {
        wait(100);
        if !arm.is_ok() {
            seen_not_ok = true;
            break;
        }
    }
    assert!(seen_not_ok, "is_ok should flip false without traffic");

    arm.disconnect();
}

#[test]
fn sdk_limits_clamp_control() {
    let mock = Arc::new(MockBus::new());
    let arm = PiperInterface::new(mock.clone()).unwrap();
    arm.set_sdk_joint_limit_enabled(true);

    // j2 limit is [0, 3.14] rad -> [0, 180000] millideg; j1 limit [-150,150] deg.
    arm.joint_ctrl(200_000, -5_000_000, 0, 0, 0, 0).unwrap();

    let frames = mock.sent_frames();
    let f12 = frames.iter().find(|(id, _)| *id == 0x155).unwrap();
    let j1 = i32::from_be_bytes(f12.1[0..4].try_into().unwrap());
    let j2 = i32::from_be_bytes(f12.1[4..8].try_into().unwrap());
    assert_eq!(j1, 149_995); // clamped to +149.995° (= round(deg(2.6179)*1000))
    assert_eq!(j2, 0); // clamped to 0

    arm.disconnect();
}

#[test]
fn piper_init_sends_queries() {
    let mock = Arc::new(MockBus::new());
    let arm = PiperInterface::new(mock.clone()).unwrap();

    arm.piper_init().unwrap();

    let frames = mock.sent_frames();
    // 6x angle/spd queries (0x472 content 0x01) + 6x acc queries (0x472 content 0x02) + firmware query
    let search: Vec<_> = frames.iter().filter(|(id, _)| *id == 0x472).collect();
    assert_eq!(search.len(), 12);
    let fw: Vec<_> = frames.iter().filter(|(id, _)| *id == 0x4AF).collect();
    assert_eq!(fw.len(), 1);

    arm.disconnect();
}

#[test]
fn fk_enabled_updates_fk_state() -> Result<()> {
    let mock = Arc::new(MockBus::new());
    let arm = PiperInterface::new(mock.clone())?;
    arm.enable_fk_cal();

    push(&mock, &DecodedMessage::JointFeedback12(0, 0));
    push(&mock, &DecodedMessage::JointFeedback34(0, 0));
    push(&mock, &DecodedMessage::JointFeedback56(0, 0));
    wait(200);

    let fk = arm.get_fk_feedback();
    // At all-zero joints the tool is near the home pose.
    assert!(fk[5][2].abs() < 600.0);
    arm.disable_fk_cal();
    arm.disconnect();
    Ok(())
}
