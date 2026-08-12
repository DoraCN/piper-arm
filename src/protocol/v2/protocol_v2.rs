//! Piper V2 protocol codec.

use crate::error::{Error, Result};
use crate::protocol::base::{bytes_to_int, i32_to_bytes, to_signed_16, to_signed_32, to_signed_8, u16_to_bytes};

use super::can_id as id;
use super::messages::*;

/// A fully decoded CAN message.
#[derive(Debug, Clone)]
pub enum DecodedMessage {
    /// 机械臂状态反馈 (0x2A1)
    StatusFeedback(ArmMsgFeedbackStatus),
    /// 末端位姿反馈 XY (0x2A2)
    EndPoseXY(i32, i32),
    /// 末端位姿反馈 Z/RX (0x2A3)
    EndPoseZRX(i32, i32),
    /// 末端位姿反馈 RY/RZ (0x2A4)
    EndPoseRYRZ(i32, i32),
    /// 关节反馈 1/2 (0x2A5)
    JointFeedback12(i32, i32),
    /// 关节反馈 3/4 (0x2A6)
    JointFeedback34(i32, i32),
    /// 关节反馈 5/6 (0x2A7)
    JointFeedback56(i32, i32),
    /// 夹爪反馈 (0x2A8)
    GripperFeedback(ArmMsgFeedbackGripper),
    /// 驱动器高速反馈, 参数为电机序号 1..=6 (0x251~0x256)
    HighSpdFeedback(usize, ArmMsgFeedbackHighSpd),
    /// 驱动器低速反馈, 参数为电机序号 1..=6 (0x261~0x266)
    LowSpdFeedback(usize, ArmMsgFeedbackLowSpd),
    /// 设置指令应答反馈 (0x476)
    FeedbackRespSetInstruction(ArmMsgFeedbackRespSetInstruction),
    /// 反馈当前电机限制角度/最大速度 (0x473)
    FeedbackCurrentMotorAngleLimitMaxSpd(ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd),
    /// 反馈当前末端速度/加速度参数 (0x478)
    FeedbackCurrentEndVelAccParam(ArmMsgFeedbackCurrentEndVelAccParam),
    /// 碰撞防护等级反馈 (0x47B)
    CrashProtectionRatingFeedback(ArmMsgFeedbackCrashProtectionRating),
    /// 反馈当前电机最大加速度限制 (0x47C)
    FeedbackCurrentMotorMaxAccLimit(ArmMsgFeedbackCurrentMotorMaxAccLimit),
    /// 夹爪/示教器参数反馈 (0x47E)
    GripperTeachingPendantParamFeedback(ArmMsgFeedbackGripperTeachingPendantParam),
    /// 固件读取 (0x4AF)
    FirmwareRead(Vec<u8>),
    /// 关节控制指令读取 (主臂发送, 0x155~0x157)
    JointCtrl(ArmMsgJointCtrl),
    /// 夹爪控制指令读取 (主臂发送, 0x159)
    GripperCtrl(ArmMsgGripperCtrl),
    /// 运动控制指令2读取 (主臂发送, 0x151)
    MotionCtrl2(ArmMsgMotionCtrl2),
    /// 反馈当前关节的末端速度/加速度 (0x481~0x486)
    JointVelAcc(usize, ArmMsgFeedbackJointVelAcc),
}

impl DecodedMessage {
    /// The CAN ID this message was decoded from.
    pub fn can_id(&self) -> u32 {
        use DecodedMessage::*;
        match self {
            StatusFeedback(_) => id::ARM_STATUS_FEEDBACK,
            EndPoseXY(_, _) => id::ARM_END_POSE_FEEDBACK_1,
            EndPoseZRX(_, _) => id::ARM_END_POSE_FEEDBACK_2,
            EndPoseRYRZ(_, _) => id::ARM_END_POSE_FEEDBACK_3,
            JointFeedback12(_, _) => id::ARM_JOINT_FEEDBACK_12,
            JointFeedback34(_, _) => id::ARM_JOINT_FEEDBACK_34,
            JointFeedback56(_, _) => id::ARM_JOINT_FEEDBACK_56,
            GripperFeedback(_) => id::ARM_GRIPPER_FEEDBACK,
            HighSpdFeedback(n, _) => id::ARM_INFO_HIGH_SPD_FEEDBACK_1 + (*n as u32) - 1,
            LowSpdFeedback(n, _) => id::ARM_INFO_LOW_SPD_FEEDBACK_1 + (*n as u32) - 1,
            FeedbackRespSetInstruction(_) => id::ARM_FEEDBACK_RESP_SET_INSTRUCTION,
            FeedbackCurrentMotorAngleLimitMaxSpd(_) => id::ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD,
            FeedbackCurrentEndVelAccParam(_) => id::ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM,
            CrashProtectionRatingFeedback(_) => id::ARM_CRASH_PROTECTION_RATING_FEEDBACK,
            FeedbackCurrentMotorMaxAccLimit(_) => id::ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT,
            GripperTeachingPendantParamFeedback(_) => id::ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK,
            FirmwareRead(_) => id::ARM_FIRMWARE_READ,
            JointCtrl(_) => id::ARM_JOINT_CTRL_12,
            GripperCtrl(_) => id::ARM_GRIPPER_CTRL,
            MotionCtrl2(_) => id::ARM_MOTION_CTRL_2,
            JointVelAcc(n, _) => id::ARM_FEEDBACK_JOINT_VEL_ACC_1 + (*n as u32) - 1,
        }
    }
}

/// Decode a CAN frame into a [`DecodedMessage`].
///
/// Returns `Ok(None)` for unknown CAN IDs. All multi-byte fields are
/// big-endian, as defined by the protocol.
pub fn decode(can_id: u32, data: &[u8]) -> Result<Option<DecodedMessage>> {
    use DecodedMessage::*;

    // Helper closures for partial fields.
    let msg = match can_id {
        id::ARM_STATUS_FEEDBACK => {
            let m = ArmMsgFeedbackStatus {
                ctrl_mode: CtrlMode::from(to_signed_8(bytes_to_int(data, 0, 1)?, false) as u8),
                arm_status: ArmStatus::from(to_signed_8(bytes_to_int(data, 1, 2)?, false) as u8),
                mode_feed: ModeFeed::from(to_signed_8(bytes_to_int(data, 2, 3)?, false) as u8),
                teach_status: TeachStatus::from(to_signed_8(bytes_to_int(data, 3, 4)?, false) as u8),
                motion_status: MotionStatus::from(to_signed_8(bytes_to_int(data, 4, 5)?, false) as u8),
                trajectory_num: to_signed_8(bytes_to_int(data, 5, 6)?, false) as u8,
                err_code: bytes_to_int(data, 6, 8)? as u16,
            };
            StatusFeedback(m)
        }
        id::ARM_END_POSE_FEEDBACK_1 => EndPoseXY(
            to_signed_32(bytes_to_int(data, 0, 4)?),
            to_signed_32(bytes_to_int(data, 4, 8)?),
        ),
        id::ARM_END_POSE_FEEDBACK_2 => EndPoseZRX(
            to_signed_32(bytes_to_int(data, 0, 4)?),
            to_signed_32(bytes_to_int(data, 4, 8)?),
        ),
        id::ARM_END_POSE_FEEDBACK_3 => EndPoseRYRZ(
            to_signed_32(bytes_to_int(data, 0, 4)?),
            to_signed_32(bytes_to_int(data, 4, 8)?),
        ),
        id::ARM_JOINT_FEEDBACK_12 => JointFeedback12(
            to_signed_32(bytes_to_int(data, 0, 4)?),
            to_signed_32(bytes_to_int(data, 4, 8)?),
        ),
        id::ARM_JOINT_FEEDBACK_34 => JointFeedback34(
            to_signed_32(bytes_to_int(data, 0, 4)?),
            to_signed_32(bytes_to_int(data, 4, 8)?),
        ),
        id::ARM_JOINT_FEEDBACK_56 => JointFeedback56(
            to_signed_32(bytes_to_int(data, 0, 4)?),
            to_signed_32(bytes_to_int(data, 4, 8)?),
        ),
        id::ARM_GRIPPER_FEEDBACK => {
            let mut m = ArmMsgFeedbackGripper {
                grippers_angle: to_signed_32(bytes_to_int(data, 0, 4)?),
                grippers_effort: to_signed_16(bytes_to_int(data, 4, 6)?, true) as i16,
                ..Default::default()
            };
            m.set_status_code(to_signed_8(bytes_to_int(data, 6, 7)?, false) as u8);
            GripperFeedback(m)
        }
        id::ARM_INFO_HIGH_SPD_FEEDBACK_1
        | id::ARM_INFO_HIGH_SPD_FEEDBACK_2
        | id::ARM_INFO_HIGH_SPD_FEEDBACK_3
        | id::ARM_INFO_HIGH_SPD_FEEDBACK_4
        | id::ARM_INFO_HIGH_SPD_FEEDBACK_5
        | id::ARM_INFO_HIGH_SPD_FEEDBACK_6 => {
            let n = (can_id - id::ARM_INFO_HIGH_SPD_FEEDBACK_1 + 1) as usize;
            let mut m = ArmMsgFeedbackHighSpd {
                can_id,
                motor_speed: to_signed_16(bytes_to_int(data, 0, 2)?, true) as i16,
                current: to_signed_16(bytes_to_int(data, 2, 4)?, true) as i16,
                pos: to_signed_32(bytes_to_int(data, 4, 8)?),
                ..Default::default()
            };
            m.cal_effort();
            HighSpdFeedback(n, m)
        }
        id::ARM_INFO_LOW_SPD_FEEDBACK_1
        | id::ARM_INFO_LOW_SPD_FEEDBACK_2
        | id::ARM_INFO_LOW_SPD_FEEDBACK_3
        | id::ARM_INFO_LOW_SPD_FEEDBACK_4
        | id::ARM_INFO_LOW_SPD_FEEDBACK_5
        | id::ARM_INFO_LOW_SPD_FEEDBACK_6 => {
            let n = (can_id - id::ARM_INFO_LOW_SPD_FEEDBACK_1 + 1) as usize;
            let mut m = ArmMsgFeedbackLowSpd {
                can_id,
                vol: bytes_to_int(data, 0, 2)? as u16,
                foc_temp: to_signed_16(bytes_to_int(data, 2, 4)?, true) as i16,
                motor_temp: to_signed_8(bytes_to_int(data, 4, 5)?, true) as i8,
                bus_current: bytes_to_int(data, 6, 8)? as u16,
                ..Default::default()
            };
            m.set_foc_status_code(to_signed_8(bytes_to_int(data, 5, 6)?, false) as u8);
            LowSpdFeedback(n, m)
        }
        id::ARM_FEEDBACK_RESP_SET_INSTRUCTION => {
            let m = ArmMsgFeedbackRespSetInstruction {
                instruction_index: to_signed_8(bytes_to_int(data, 0, 1)?, false) as i16,
                is_set_zero_successfully: to_signed_8(bytes_to_int(data, 1, 2)?, false) as i16,
            };
            FeedbackRespSetInstruction(m)
        }
        id::ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD => {
            let m = ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd {
                motor_num: to_signed_8(bytes_to_int(data, 0, 1)?, false) as u8,
                max_angle_limit: to_signed_16(bytes_to_int(data, 1, 3)?, true) as i16,
                min_angle_limit: to_signed_16(bytes_to_int(data, 3, 5)?, true) as i16,
                max_joint_spd: bytes_to_int(data, 5, 7)? as u16,
            };
            FeedbackCurrentMotorAngleLimitMaxSpd(m)
        }
        id::ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM => {
            let m = ArmMsgFeedbackCurrentEndVelAccParam {
                end_max_linear_vel: bytes_to_int(data, 0, 2)? as u16,
                end_max_angular_vel: bytes_to_int(data, 2, 4)? as u16,
                end_max_linear_acc: bytes_to_int(data, 4, 6)? as u16,
                end_max_angular_acc: bytes_to_int(data, 6, 8)? as u16,
            };
            FeedbackCurrentEndVelAccParam(m)
        }
        id::ARM_CRASH_PROTECTION_RATING_FEEDBACK => {
            let m = ArmMsgFeedbackCrashProtectionRating {
                joint_1_protection_level: to_signed_8(bytes_to_int(data, 0, 1)?, false) as u8,
                joint_2_protection_level: to_signed_8(bytes_to_int(data, 1, 2)?, false) as u8,
                joint_3_protection_level: to_signed_8(bytes_to_int(data, 2, 3)?, false) as u8,
                joint_4_protection_level: to_signed_8(bytes_to_int(data, 3, 4)?, false) as u8,
                joint_5_protection_level: to_signed_8(bytes_to_int(data, 4, 5)?, false) as u8,
                joint_6_protection_level: to_signed_8(bytes_to_int(data, 5, 6)?, false) as u8,
            };
            CrashProtectionRatingFeedback(m)
        }
        id::ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT => {
            let m = ArmMsgFeedbackCurrentMotorMaxAccLimit {
                joint_motor_num: to_signed_8(bytes_to_int(data, 0, 1)?, false) as u8,
                max_joint_acc: bytes_to_int(data, 1, 3)? as u16,
            };
            FeedbackCurrentMotorMaxAccLimit(m)
        }
        id::ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK => {
            let m = ArmMsgFeedbackGripperTeachingPendantParam {
                teaching_range_per: to_signed_8(bytes_to_int(data, 0, 1)?, false) as u8,
                max_range_config: to_signed_8(bytes_to_int(data, 1, 2)?, false) as u8,
                teaching_friction: to_signed_8(bytes_to_int(data, 2, 3)?, false) as u8,
            };
            GripperTeachingPendantParamFeedback(m)
        }
        id::ARM_FIRMWARE_READ => FirmwareRead(data.to_vec()),
        id::ARM_JOINT_CTRL_12 => JointCtrl(ArmMsgJointCtrl {
            joint_1: to_signed_32(bytes_to_int(data, 0, 4)?),
            joint_2: to_signed_32(bytes_to_int(data, 4, 8)?),
            ..Default::default()
        }),
        id::ARM_JOINT_CTRL_34 => JointCtrl(ArmMsgJointCtrl {
            joint_3: to_signed_32(bytes_to_int(data, 0, 4)?),
            joint_4: to_signed_32(bytes_to_int(data, 4, 8)?),
            ..Default::default()
        }),
        id::ARM_JOINT_CTRL_56 => JointCtrl(ArmMsgJointCtrl {
            joint_5: to_signed_32(bytes_to_int(data, 0, 4)?),
            joint_6: to_signed_32(bytes_to_int(data, 4, 8)?),
            ..Default::default()
        }),
        id::ARM_GRIPPER_CTRL => {
            let m = ArmMsgGripperCtrl {
                grippers_angle: to_signed_32(bytes_to_int(data, 0, 4)?),
                grippers_effort: bytes_to_int(data, 4, 6)? as u16,
                status_code: to_signed_8(bytes_to_int(data, 6, 7)?, false) as u8,
                set_zero: to_signed_8(bytes_to_int(data, 7, 8)?, false) as u8,
            };
            GripperCtrl(m)
        }
        id::ARM_MOTION_CTRL_2 => {
            let m = ArmMsgMotionCtrl2 {
                ctrl_mode: to_signed_8(bytes_to_int(data, 0, 1)?, false) as u8,
                move_mode: to_signed_8(bytes_to_int(data, 1, 2)?, false) as u8,
                move_spd_rate_ctrl: to_signed_8(bytes_to_int(data, 2, 3)?, false) as u8,
                mit_mode: to_signed_8(bytes_to_int(data, 3, 4)?, false) as u8,
                residence_time: to_signed_8(bytes_to_int(data, 4, 5)?, false) as u8,
                ..Default::default()
            };
            MotionCtrl2(m)
        }
        id::ARM_FEEDBACK_JOINT_VEL_ACC_1
        | id::ARM_FEEDBACK_JOINT_VEL_ACC_2
        | id::ARM_FEEDBACK_JOINT_VEL_ACC_3
        | id::ARM_FEEDBACK_JOINT_VEL_ACC_4
        | id::ARM_FEEDBACK_JOINT_VEL_ACC_5
        | id::ARM_FEEDBACK_JOINT_VEL_ACC_6 => {
            let n = (can_id - id::ARM_FEEDBACK_JOINT_VEL_ACC_1 + 1) as usize;
            let m = ArmMsgFeedbackJointVelAcc {
                can_id,
                end_linear_vel: to_signed_16(bytes_to_int(data, 0, 2)?, false) as u16,
                end_angular_vel: to_signed_16(bytes_to_int(data, 2, 4)?, false) as u16,
                end_linear_acc: to_signed_16(bytes_to_int(data, 4, 6)?, false) as u16,
                end_angular_acc: to_signed_16(bytes_to_int(data, 6, 8)?, false) as u16,
            };
            JointVelAcc(n, m)
        }
        _ => return Ok(None),
    };
    Ok(Some(msg))
}

/// A CAN control frame ready to transmit.
#[derive(Debug, Clone, Copy)]
pub struct ControlFrame {
    pub id: u32,
    pub data: [u8; 8],
}

/// 运动控制指令1 (0x150)
pub fn encode_motion_ctrl_1(m: &ArmMsgMotionCtrl1) -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    data[0] = m.emergency_stop;
    data[1] = m.track_ctrl;
    data[2] = m.grag_teach_ctrl;
    Ok(ControlFrame { id: id::ARM_MOTION_CTRL_1, data })
}

/// 运动控制指令2 (0x151)
pub fn encode_motion_ctrl_2(m: &ArmMsgMotionCtrl2) -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    data[0] = m.ctrl_mode;
    data[1] = m.move_mode;
    data[2] = m.move_spd_rate_ctrl;
    data[3] = m.mit_mode;
    data[4] = m.residence_time;
    data[5] = m.installation_pos;
    Ok(ControlFrame { id: id::ARM_MOTION_CTRL_2, data })
}

/// 运动控制直角坐标系指令 XY (0x152)
pub fn encode_cartesian_xy(x: i32, y: i32) -> Result<ControlFrame> {
    let xb = i32_to_bytes(x);
    let yb = i32_to_bytes(y);
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&xb);
    data[4..8].copy_from_slice(&yb);
    Ok(ControlFrame { id: id::ARM_MOTION_CTRL_CARTESIAN_1, data })
}

/// 运动控制直角坐标系指令 Z/RX (0x153)
pub fn encode_cartesian_zrx(z: i32, rx: i32) -> Result<ControlFrame> {
    let zb = i32_to_bytes(z);
    let rx_b = i32_to_bytes(rx);
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&zb);
    data[4..8].copy_from_slice(&rx_b);
    Ok(ControlFrame { id: id::ARM_MOTION_CTRL_CARTESIAN_2, data })
}

/// 运动控制直角坐标系指令 RY/RZ (0x154)
pub fn encode_cartesian_ryrz(ry: i32, rz: i32) -> Result<ControlFrame> {
    let ryb = i32_to_bytes(ry);
    let rzb = i32_to_bytes(rz);
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&ryb);
    data[4..8].copy_from_slice(&rzb);
    Ok(ControlFrame { id: id::ARM_MOTION_CTRL_CARTESIAN_3, data })
}

/// 关节控制指令 1/2 (0x155)
pub fn encode_joint_ctrl_12(j1: i32, j2: i32) -> Result<ControlFrame> {
    let j1b = i32_to_bytes(j1);
    let j2b = i32_to_bytes(j2);
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&j1b);
    data[4..8].copy_from_slice(&j2b);
    Ok(ControlFrame { id: id::ARM_JOINT_CTRL_12, data })
}

/// 关节控制指令 3/4 (0x156)
pub fn encode_joint_ctrl_34(j3: i32, j4: i32) -> Result<ControlFrame> {
    let j3b = i32_to_bytes(j3);
    let j4b = i32_to_bytes(j4);
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&j3b);
    data[4..8].copy_from_slice(&j4b);
    Ok(ControlFrame { id: id::ARM_JOINT_CTRL_34, data })
}

/// 关节控制指令 5/6 (0x157)
pub fn encode_joint_ctrl_56(j5: i32, j6: i32) -> Result<ControlFrame> {
    let j5b = i32_to_bytes(j5);
    let j6b = i32_to_bytes(j6);
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&j5b);
    data[4..8].copy_from_slice(&j6b);
    Ok(ControlFrame { id: id::ARM_JOINT_CTRL_56, data })
}

/// MoveC 模式坐标序号更新指令 (0x158)
pub fn encode_circular_pattern(instruction_num: u8) -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    data[0] = instruction_num;
    Ok(ControlFrame { id: id::ARM_CIRCULAR_PATTERN_COORD_NUM_UPDATE_CTRL, data })
}

/// 夹爪控制指令 (0x159)
pub fn encode_gripper_ctrl(m: &ArmMsgGripperCtrl) -> Result<ControlFrame> {
    let angle_b = i32_to_bytes(m.grippers_angle);
    let effort_b = u16_to_bytes(m.grippers_effort);
    let mut data = [0u8; 8];
    data[0..4].copy_from_slice(&angle_b);
    data[4] = effort_b[0];
    data[5] = effort_b[1];
    data[6] = m.status_code;
    data[7] = m.set_zero;
    Ok(ControlFrame { id: id::ARM_GRIPPER_CTRL, data })
}

/// 关节 MIT 控制指令 (0x15A~0x15F)
pub fn encode_joint_mit_ctrl(motor_num: u8, m: &ArmMsgJointMitCtrl) -> Result<ControlFrame> {
    let frame_id = match motor_num {
        1 => id::ARM_JOINT_MIT_CTRL_1,
        2 => id::ARM_JOINT_MIT_CTRL_2,
        3 => id::ARM_JOINT_MIT_CTRL_3,
        4 => id::ARM_JOINT_MIT_CTRL_4,
        5 => id::ARM_JOINT_MIT_CTRL_5,
        6 => id::ARM_JOINT_MIT_CTRL_6,
        _ => {
            return Err(Error::ValueError(format!(
                "motor_num {motor_num} out of range 1-6 for MIT control"
            )))
        }
    };
    let pos_ref = m.pos_ref;
    let vel_ref = m.vel_ref & 0xFFF;
    let kp = m.kp & 0xFFF;
    let kd = m.kd & 0xFFF;
    let t_ref = m.t_ref;

    let mut data = [0u8; 8];
    data[0] = (pos_ref >> 8) as u8;
    data[1] = pos_ref as u8;
    data[2] = (vel_ref >> 4) as u8;
    data[3] = (((vel_ref & 0xF) << 4) | ((kp >> 8) & 0x0F)) as u8;
    data[4] = kp as u8;
    data[5] = (kd >> 4) as u8;
    let kd_hi = (((kd & 0xF) as u8) << 4) | ((t_ref >> 4) & 0x0F);
    data[6] = kd_hi;
    // crc: XOR of first 7 bytes, low 4 bits.
    let crc = data[..7].iter().fold(0u8, |acc, &b| acc ^ b) & 0x0F;
    data[7] = ((t_ref << 4) & 0xF0) | crc;
    Ok(ControlFrame { id: frame_id, data })
}

/// 随动主从模式设置指令 (0x470)
pub fn encode_master_slave_config(m: &ArmMsgMasterSlaveModeConfig) -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    data[0] = m.linkage_config;
    data[1] = m.feedback_offset;
    data[2] = m.ctrl_offset;
    data[3] = m.linkage_offset;
    Ok(ControlFrame { id: id::ARM_MASTER_SLAVE_MODE_CONFIG, data })
}

/// 电机使能/失能设置指令 (0x471)
pub fn encode_motor_enable_disable(m: &ArmMsgMotorEnableDisableConfig) -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    data[0] = m.motor_num;
    data[1] = m.enable_flag;
    Ok(ControlFrame { id: id::ARM_MOTOR_ENABLE_DISABLE_CONFIG, data })
}

/// 查询电机角度/最大速度/最大加速度限制指令 (0x472)
pub fn encode_search_motor(m: &ArmMsgSearchMotorMaxAngleSpdAccLimit) -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    data[0] = m.motor_num;
    data[1] = m.search_content;
    Ok(ControlFrame { id: id::ARM_SEARCH_MOTOR_MAX_SPD_ACC_LIMIT, data })
}

/// 电机角度限制/最大速度设置指令 (0x474)
pub fn encode_motor_angle_limit_set(m: &ArmMsgMotorAngleLimitMaxSpdSet) -> Result<ControlFrame> {
    let max_angle = u16_to_bytes(m.max_angle_limit as u16);
    let min_angle = u16_to_bytes(m.min_angle_limit as u16);
    let max_spd = u16_to_bytes(m.max_joint_spd);
    let mut data = [0u8; 8];
    data[0] = m.motor_num;
    data[1] = max_angle[0];
    data[2] = max_angle[1];
    data[3] = min_angle[0];
    data[4] = min_angle[1];
    data[5] = max_spd[0];
    data[6] = max_spd[1];
    Ok(ControlFrame { id: id::ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET, data })
}

/// 关节设置指令 (0x475)
pub fn encode_joint_config(m: &ArmMsgJointConfig) -> Result<ControlFrame> {
    let acc = u16_to_bytes(m.max_joint_acc);
    let mut data = [0u8; 8];
    data[0] = m.joint_motor_num;
    data[1] = m.set_motor_current_pos_as_zero;
    data[2] = m.acc_param_config_is_effective_or_not;
    data[3] = acc[0];
    data[4] = acc[1];
    data[5] = m.clear_joint_err;
    Ok(ControlFrame { id: id::ARM_JOINT_CONFIG, data })
}

/// 设置指令应答 (0x476)
pub fn encode_instruction_response(m: &ArmMsgInstructionResponseConfig) -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    data[0] = m.instruction_index;
    data[1] = m.zero_config_success_flag;
    Ok(ControlFrame { id: id::ARM_INSTRUCTION_RESPONSE_CONFIG, data })
}

/// 机械臂参数查询与设置指令 (0x477)
pub fn encode_param_enquiry(m: &ArmMsgParamEnquiryAndConfig) -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    data[0] = m.param_enquiry;
    data[1] = m.param_setting;
    data[2] = m.data_feedback_0x48x;
    data[3] = m.end_load_param_setting_effective;
    data[4] = m.set_end_load;
    Ok(ControlFrame { id: id::ARM_PARAM_ENQUIRY_AND_CONFIG, data })
}

/// 末端速度/加速度参数设置指令 (0x479)
pub fn encode_end_vel_acc_param(m: &ArmMsgEndVelAccParamConfig) -> Result<ControlFrame> {
    let a = u16_to_bytes(m.end_max_linear_vel);
    let b = u16_to_bytes(m.end_max_angular_vel);
    let c = u16_to_bytes(m.end_max_linear_acc);
    let d = u16_to_bytes(m.end_max_angular_acc);
    let mut data = [0u8; 8];
    data[0] = a[0];
    data[1] = a[1];
    data[2] = b[0];
    data[3] = b[1];
    data[4] = c[0];
    data[5] = c[1];
    data[6] = d[0];
    data[7] = d[1];
    Ok(ControlFrame { id: id::ARM_END_VEL_ACC_PARAM_CONFIG, data })
}

/// 碰撞防护等级设置指令 (0x47A)
pub fn encode_crash_protection(m: &ArmMsgCrashProtectionRatingConfig) -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    data[0] = m.joint_1_protection_level;
    data[1] = m.joint_2_protection_level;
    data[2] = m.joint_3_protection_level;
    data[3] = m.joint_4_protection_level;
    data[4] = m.joint_5_protection_level;
    data[5] = m.joint_6_protection_level;
    Ok(ControlFrame { id: id::ARM_CRASH_PROTECTION_RATING_CONFIG, data })
}

/// 夹爪/示教器参数设置指令 (0x47D)
pub fn encode_gripper_teaching_param(m: &ArmMsgGripperTeachingPendantParamConfig) -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    data[0] = m.teaching_range_per;
    data[1] = m.max_range_config;
    data[2] = m.teaching_friction;
    Ok(ControlFrame { id: id::ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG, data })
}

/// 固件版本查询指令 (0x4AF)
pub fn encode_search_firmware() -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    data[0] = 0x01;
    Ok(ControlFrame { id: id::ARM_FIRMWARE_READ, data })
}

/// 请求主臂回零指令 (0x191)
pub fn encode_req_master_arm_move_to_home(mode: u8) -> Result<ControlFrame> {
    let mut data = [0u8; 8];
    match mode {
        0 => {
            data[0] = 0x01; // 恢复主从臂模式
        }
        1 => {
            data[0] = 0x01;
            data[1] = 0x01;
            data[2] = 0x01; // 主臂回零
        }
        2 => {
            data[0] = 0x01;
            data[2] = 0x01; // 主从臂一起回零
        }
        _ => {
            return Err(Error::ValueError(format!(
                "mode {mode} out of range 0-2 for ReqMasterArmMoveToHome"
            )))
        }
    }
    Ok(ControlFrame { id: 0x191, data })
}
