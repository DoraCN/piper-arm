//! Protocol codec tests.

use piper_arm::protocol::base::*;
use piper_arm::protocol::v2::messages::*;
use piper_arm::protocol::MsgType;
use piper_arm::protocol::{decode, ControlFrame};
use piper_arm::protocol::{
    encode_cartesian_xy, encode_circular_pattern, encode_crash_protection,
    encode_end_vel_acc_param, encode_gripper_ctrl, encode_gripper_teaching_param,
    encode_joint_config, encode_joint_ctrl_12, encode_joint_mit_ctrl, encode_master_slave_config,
    encode_motion_ctrl_1, encode_motion_ctrl_2, encode_motor_angle_limit_set,
    encode_motor_enable_disable, encode_param_enquiry, encode_search_firmware, encode_search_motor,
};

fn assert_frame(f: &ControlFrame, expected_id: u32, expected_data: &[u8; 8]) {
    assert_eq!(f.id, expected_id, "frame id mismatch");
    assert_eq!(&f.data, expected_data, "frame data mismatch");
}

#[test]
fn base_conversions() {
    assert_eq!(to_signed_8(0x80, true), -128);
    assert_eq!(to_signed_8(0x80, false), 128);
    assert_eq!(to_signed_16(0x8000, true), -32768);
    assert_eq!(to_signed_16(0x8000, false), 32768);
    assert_eq!(to_signed_32(0x8000_0000), i32::MIN);
    assert_eq!(bytes_to_int(&[0x01, 0x02, 0x03], 0, 3).unwrap(), 0x010203);
    let b = int_to_bytes(-1, 2, true).unwrap();
    assert_eq!(&b[..2], &[0xFF, 0xFF]);
    let b = int_to_bytes(0x0102, 2, false).unwrap();
    assert_eq!(&b[..2], &[0x01, 0x02]);
    assert!(int_to_bytes(300, 1, true).is_err());
    // float_to_uint: int((x - x_min) * (2^bits-1) / (x_max - x_min))
    assert_eq!(float_to_uint(0.0, -12.5, 12.5, 16), 32767);
    assert_eq!(float_to_uint(12.5, -12.5, 12.5, 16), 65535);
    assert_eq!(float_to_uint(0.0, -45.0, 45.0, 12), 2047);
}

#[test]
fn test_encode_motion_ctrl_1() {
    let m = ArmMsgMotionCtrl1 { emergency_stop: 0x01, track_ctrl: 0x03, grag_teach_ctrl: 0x02 };
    assert_frame(&encode_motion_ctrl_1(&m).unwrap(), 0x150, &[0x01, 0x03, 0x02, 0, 0, 0, 0, 0]);
}

#[test]
fn test_encode_motion_ctrl_2() {
    let m = ArmMsgMotionCtrl2 {
        ctrl_mode: 0x01,
        move_mode: 0x01,
        move_spd_rate_ctrl: 50,
        mit_mode: 0x00,
        residence_time: 0,
        installation_pos: 0x01,
    };
    assert_frame(&encode_motion_ctrl_2(&m).unwrap(), 0x151, &[0x01, 0x01, 0x32, 0x00, 0x00, 0x01, 0, 0]);
}

#[test]
fn test_encode_cartesian_xy() {
    assert_frame(
        &encode_cartesian_xy(123456, -654321).unwrap(),
        0x152,
        &[0x00, 0x01, 0xE2, 0x40, 0xFF, 0xF6, 0x04, 0x0F],
    );
}

#[test]
fn test_encode_joint_ctrl_12() {
    assert_frame(
        &encode_joint_ctrl_12(12345, -67890).unwrap(),
        0x155,
        &[0x00, 0x00, 0x30, 0x39, 0xFF, 0xFE, 0xF6, 0xCE],
    );
}

#[test]
fn test_encode_gripper_ctrl() {
    let m = ArmMsgGripperCtrl {
        grippers_angle: 50000,
        grippers_effort: 1000,
        status_code: 0x01,
        set_zero: 0xAE,
    };
    assert_frame(&encode_gripper_ctrl(&m).unwrap(), 0x159, &[0x00, 0x00, 0xC3, 0x50, 0x03, 0xE8, 0x01, 0xAE]);
}

#[test]
fn encode_master_slave() {
    let m = ArmMsgMasterSlaveModeConfig {
        linkage_config: 0xFC,
        feedback_offset: 0x10,
        ctrl_offset: 0x20,
        linkage_offset: 0x10,
    };
    assert_frame(&encode_master_slave_config(&m).unwrap(), 0x470, &[0xFC, 0x10, 0x20, 0x10, 0, 0, 0, 0]);
}

#[test]
fn encode_enable_disable() {
    let m = ArmMsgMotorEnableDisableConfig { motor_num: 7, enable_flag: 0x02 };
    assert_frame(&encode_motor_enable_disable(&m).unwrap(), 0x471, &[7, 2, 0, 0, 0, 0, 0, 0]);
}

#[test]
fn test_encode_search_motor() {
    let m = ArmMsgSearchMotorMaxAngleSpdAccLimit { motor_num: 3, search_content: 0x01 };
    assert_frame(&encode_search_motor(&m).unwrap(), 0x472, &[3, 1, 0, 0, 0, 0, 0, 0]);
}

#[test]
fn encode_angle_limit_set() {
    let m = ArmMsgMotorAngleLimitMaxSpdSet {
        motor_num: 2,
        max_angle_limit: 1500,
        min_angle_limit: -1500,
        max_joint_spd: 3000,
    };
    assert_frame(&encode_motor_angle_limit_set(&m).unwrap(), 0x474, &[2, 0x05, 0xDC, 0xFA, 0x24, 0x0B, 0xB8, 0]);
}

#[test]
fn test_encode_joint_config() {
    let m = ArmMsgJointConfig {
        joint_motor_num: 7,
        set_motor_current_pos_as_zero: 0,
        acc_param_config_is_effective_or_not: 0xAE,
        max_joint_acc: 500,
        clear_joint_err: 0,
    };
    assert_frame(&encode_joint_config(&m).unwrap(), 0x475, &[7, 0, 0xAE, 0x01, 0xF4, 0, 0, 0]);
}

#[test]
fn test_encode_param_enquiry() {
    let m = ArmMsgParamEnquiryAndConfig {
        param_enquiry: 0x01,
        param_setting: 0,
        data_feedback_0x48x: 0,
        end_load_param_setting_effective: 0,
        set_end_load: 0x03,
    };
    assert_frame(&encode_param_enquiry(&m).unwrap(), 0x477, &[0x01, 0, 0, 0, 0x03, 0, 0, 0]);
}

#[test]
fn encode_end_vel_acc() {
    assert_frame(
        &encode_end_vel_acc_param(&ArmMsgEndVelAccParamConfig {
            end_max_linear_vel: 1000,
            end_max_angular_vel: 2000,
            end_max_linear_acc: 3000,
            end_max_angular_acc: 4000,
        })
        .unwrap(),
        0x479,
        &[0x03, 0xE8, 0x07, 0xD0, 0x0B, 0xB8, 0x0F, 0xA0],
    );
}

#[test]
fn test_encode_crash_protection() {
    let m = ArmMsgCrashProtectionRatingConfig {
        joint_1_protection_level: 1,
        joint_2_protection_level: 2,
        joint_3_protection_level: 3,
        joint_4_protection_level: 4,
        joint_5_protection_level: 5,
        joint_6_protection_level: 6,
    };
    assert_frame(&encode_crash_protection(&m).unwrap(), 0x47A, &[1, 2, 3, 4, 5, 6, 0, 0]);
}

#[test]
fn encode_teaching_param() {
    let m = ArmMsgGripperTeachingPendantParamConfig {
        teaching_range_per: 100,
        max_range_config: 70,
        teaching_friction: 1,
    };
    assert_frame(&encode_gripper_teaching_param(&m).unwrap(), 0x47D, &[0x64, 0x46, 0x01, 0, 0, 0, 0, 0]);
}

#[test]
fn encode_circular() {
    assert_frame(&encode_circular_pattern(0x02).unwrap(), 0x158, &[2, 0, 0, 0, 0, 0, 0, 0]);
}

#[test]
fn encode_firmware_query() {
    assert_frame(&encode_search_firmware().unwrap(), 0x4AF, &[1, 0, 0, 0, 0, 0, 0, 0]);
}

#[test]
fn encode_mit_bit_packing_and_crc() {
    let m = ArmMsgJointMitCtrl {
        pos_ref: 12345,
        vel_ref: 123,
        kp: 100,
        kd: 50,
        t_ref: 20,
        crc: 0,
    };
    let f = encode_joint_mit_ctrl(1, &m).unwrap();
    assert_eq!(f.id, 0x15A);
    assert_eq!(&f.data, &[0x30, 0x39, 0x07, 0xB0, 0x64, 0x03, 0x21, 0x48]);
    // CRC must equal XOR of first 7 bytes & 0x0F
    let expect = f.data[..7].iter().fold(0u8, |a, &b| a ^ b) & 0x0F;
    assert_eq!(f.data[7] & 0x0F, expect);
    // t_ref low nibble
    assert_eq!(f.data[7] >> 4, 0x04);

    // motor 6 maps to 0x15F
    let f6 = encode_joint_mit_ctrl(6, &m).unwrap();
    assert_eq!(f6.id, 0x15F);
    // invalid motor
    assert!(encode_joint_mit_ctrl(7, &m).is_err());
}

#[test]
fn decode_status_feedback() {
    let data = [0x01, 0x07, 0x02, 0x00, 0x01, 0x05, 0x42, 0x0A];
    match decode(0x2A1, &data).unwrap() {
        Some(piper_arm::DecodedMessage::StatusFeedback(s)) => {
            assert_eq!(s.ctrl_mode, CtrlMode::CanCtrl);
            assert_eq!(s.arm_status, ArmStatus::CollisionOccurred);
            assert_eq!(s.mode_feed, ModeFeed::MoveL);
            assert_eq!(s.teach_status, TeachStatus::Disabled);
            assert_eq!(s.motion_status, MotionStatus::NotReached);
            assert_eq!(s.trajectory_num, 0x05);
            assert_eq!(s.err_code, 0x420A);
            assert!(s.joint_communication_status(2)); // bit1
            assert!(s.joint_communication_status(4)); // bit3
            assert!(s.joint_angle_limit(2)); // bit 9 -> err_code bit 8+2
        }
        other => panic!("unexpected decode result: {other:?}"),
    }
}

#[test]
fn decode_high_speed_feedback() {
    // speed=-123, current=456, pos=-1_000_000
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&(-123i16).to_be_bytes());
    data[2..4].copy_from_slice(&456i16.to_be_bytes());
    data[4..8].copy_from_slice(&(-1_000_000i32).to_be_bytes());
    match decode(0x251, &data).unwrap() {
        Some(piper_arm::DecodedMessage::HighSpdFeedback(n, m)) => {
            assert_eq!(n, 1);
            assert_eq!(m.motor_speed, -123);
            assert_eq!(m.current, 456);
            assert_eq!(m.pos, -1_000_000);
            // effort coefficient for joints 1-3
            assert!((m.effort - 456.0 * 1.18125).abs() < 1e-9);
        }
        other => panic!("unexpected decode result: {other:?}"),
    }
}

#[test]
fn decode_low_speed_feedback() {
    let mut data = [0u8; 8];
    data[0..2].copy_from_slice(&240u16.to_be_bytes()); // 24.0V
    data[2..4].copy_from_slice(&40i16.to_be_bytes()); // 40°C
    data[4] = 0x64; // motor temp = 100°C
    data[5] = 0b1100_0001; // vol low + enable + stall
    data[6..8].copy_from_slice(&500u16.to_be_bytes());
    match decode(0x266, &data).unwrap() {
        Some(piper_arm::DecodedMessage::LowSpdFeedback(n, m)) => {
            assert_eq!(n, 6);
            assert_eq!(m.vol, 240);
            assert_eq!(m.foc_temp, 40);
            assert_eq!(m.motor_temp, 100);
            assert_eq!(m.bus_current, 500);
            assert!(m.foc_status.voltage_too_low);
            assert!(m.foc_status.driver_enable_status);
            assert!(m.foc_status.stall_status);
            assert!(!m.foc_status.collision_status);
        }
        other => panic!("unexpected decode result: {other:?}"),
    }
}

#[test]
fn decode_roundtrip_all_encodes() {
    // Encode a message, decode it, and verify key fields survive.
    let j = ArmMsgJointCtrl { joint_1: 12345, joint_2: -67890, joint_3: 111, joint_4: -222, joint_5: 333, joint_6: -444 };
    // Only frames that are both encodable and decoded on receive round-trip.
    let f = encode_joint_ctrl_12(j.joint_1, j.joint_2).unwrap();
    let d = decode(f.id, &f.data).unwrap().expect("should decode");
    assert_eq!(d.can_id(), f.id);
    let g = ArmMsgGripperCtrl { grippers_angle: 50000, grippers_effort: 1000, status_code: 0x01, set_zero: 0xAE };
    let f = encode_gripper_ctrl(&g).unwrap();
    match decode(f.id, &f.data).unwrap().unwrap() {
        piper_arm::DecodedMessage::GripperCtrl(g2) => {
            assert_eq!(g2.grippers_angle, 50000);
            assert_eq!(g2.grippers_effort, 1000);
            assert_eq!(g2.set_zero, 0xAE);
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn decode_unknown_id() {
    assert!(decode(0x777, &[0; 8]).unwrap().is_none());
}

#[test]
fn mapping_roundtrip() {
    use piper_arm::protocol::mapping::{id_from_type, type_from_id};
    assert_eq!(type_from_id(0x2A1), Some(MsgType::StatusFeedback));
    assert_eq!(type_from_id(0x15F), Some(MsgType::JointMitCtrl6));
    assert_eq!(type_from_id(0x476), Some(MsgType::FeedbackRespSetInstruction));
    assert_eq!(id_from_type(MsgType::StatusFeedback).unwrap(), 0x2A1);
    assert_eq!(id_from_type(MsgType::FirmwareRead).unwrap(), 0x4AF);
    assert_eq!(type_from_id(0x999), None);
}
