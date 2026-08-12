//! Feedback message data structures.

/// 控制模式 (0x2A1 byte 0)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[derive(Default)]
pub enum CtrlMode {
    #[default]
    Standby = 0x00,
    CanCtrl = 0x01,
    TeachingMode = 0x02,
    EthernetControlMode = 0x03,
    WifiControlMode = 0x04,
    RemoteControlMode = 0x05,
    LinkageTeachingInputMode = 0x06,
    OfflineTrajectoryMode = 0x07,
    Unknown = 0xFF,
}

/// 机械臂状态 (0x2A1 byte 1)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[derive(Default)]
pub enum ArmStatus {
    #[default]
    Normal = 0x00,
    EmergencyStop = 0x01,
    NoSolution = 0x02,
    SingularityPoint = 0x03,
    TargetPosExceedsLimit = 0x04,
    JointCommunicationErr = 0x05,
    JointBrakeNotReleased = 0x06,
    CollisionOccurred = 0x07,
    OverspeedDuringTeachingDrag = 0x08,
    JointStatusErr = 0x09,
    OtherErr = 0x0A,
    TeachingRecord = 0x0B,
    TeachingExecution = 0x0C,
    TeachingPause = 0x0D,
    MainControllerNtcOverTemperature = 0x0E,
    ReleaseResistorNtcOverTemperature = 0x0F,
    Unknown = 0xFF,
}

/// 模式反馈 (0x2A1 byte 2)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[derive(Default)]
pub enum ModeFeed {
    #[default]
    MoveP = 0x00,
    MoveJ = 0x01,
    MoveL = 0x02,
    MoveC = 0x03,
    MoveM = 0x04,
    MoveCpv = 0x05,
    Unknown = 0xFF,
}

/// 示教状态 (0x2A1 byte 3)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[derive(Default)]
pub enum TeachStatus {
    #[default]
    Disabled = 0x00,
    StartRecording = 0x01,
    StopRecording = 0x02,
    ExecuteTrajectory = 0x03,
    PauseExecution = 0x04,
    ResumeExecution = 0x05,
    TerminateExecution = 0x06,
    MoveToStart = 0x07,
    Unknown = 0xFF,
}

/// 运动状态 (0x2A1 byte 4)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[derive(Default)]
pub enum MotionStatus {
    #[default]
    Reached = 0x00,
    NotReached = 0x01,
    Unknown = 0xFF,
}

impl From<u8> for CtrlMode {
    fn from(v: u8) -> Self {
        match v {
            0x00 => CtrlMode::Standby,
            0x01 => CtrlMode::CanCtrl,
            0x02 => CtrlMode::TeachingMode,
            0x03 => CtrlMode::EthernetControlMode,
            0x04 => CtrlMode::WifiControlMode,
            0x05 => CtrlMode::RemoteControlMode,
            0x06 => CtrlMode::LinkageTeachingInputMode,
            0x07 => CtrlMode::OfflineTrajectoryMode,
            _ => CtrlMode::Unknown,
        }
    }
}

impl From<u8> for ArmStatus {
    fn from(v: u8) -> Self {
        match v {
            0x00 => ArmStatus::Normal,
            0x01 => ArmStatus::EmergencyStop,
            0x02 => ArmStatus::NoSolution,
            0x03 => ArmStatus::SingularityPoint,
            0x04 => ArmStatus::TargetPosExceedsLimit,
            0x05 => ArmStatus::JointCommunicationErr,
            0x06 => ArmStatus::JointBrakeNotReleased,
            0x07 => ArmStatus::CollisionOccurred,
            0x08 => ArmStatus::OverspeedDuringTeachingDrag,
            0x09 => ArmStatus::JointStatusErr,
            0x0A => ArmStatus::OtherErr,
            0x0B => ArmStatus::TeachingRecord,
            0x0C => ArmStatus::TeachingExecution,
            0x0D => ArmStatus::TeachingPause,
            0x0E => ArmStatus::MainControllerNtcOverTemperature,
            0x0F => ArmStatus::ReleaseResistorNtcOverTemperature,
            _ => ArmStatus::Unknown,
        }
    }
}

impl From<u8> for ModeFeed {
    fn from(v: u8) -> Self {
        match v {
            0x00 => ModeFeed::MoveP,
            0x01 => ModeFeed::MoveJ,
            0x02 => ModeFeed::MoveL,
            0x03 => ModeFeed::MoveC,
            0x04 => ModeFeed::MoveM,
            0x05 => ModeFeed::MoveCpv,
            _ => ModeFeed::Unknown,
        }
    }
}

impl From<u8> for TeachStatus {
    fn from(v: u8) -> Self {
        match v {
            0x00 => TeachStatus::Disabled,
            0x01 => TeachStatus::StartRecording,
            0x02 => TeachStatus::StopRecording,
            0x03 => TeachStatus::ExecuteTrajectory,
            0x04 => TeachStatus::PauseExecution,
            0x05 => TeachStatus::ResumeExecution,
            0x06 => TeachStatus::TerminateExecution,
            0x07 => TeachStatus::MoveToStart,
            _ => TeachStatus::Unknown,
        }
    }
}

impl From<u8> for MotionStatus {
    fn from(v: u8) -> Self {
        match v {
            0x00 => MotionStatus::Reached,
            0x01 => MotionStatus::NotReached,
            _ => MotionStatus::Unknown,
        }
    }
}

/// 机械臂状态反馈 (0x2A1)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackStatus {
    pub ctrl_mode: CtrlMode,
    pub arm_status: ArmStatus,
    pub mode_feed: ModeFeed,
    pub teach_status: TeachStatus,
    pub motion_status: MotionStatus,
    pub trajectory_num: u8,
    /// 故障码 (uint16 big-endian, bytes 6-7)
    pub err_code: u16,
}

impl ArmMsgFeedbackStatus {
    /// bit0-5: 关节角度是否超限位
    pub fn joint_angle_limit(&self, idx: usize) -> bool {
        // idx 1..=6
        (self.err_code >> (7 + idx)) & 0x1 == 0x1
    }
    /// bits 0-5: joint communication status; bits 8-13: joint angle limits
    pub fn joint_communication_status(&self, idx: usize) -> bool {
        (self.err_code >> (idx - 1)) & 0x1 == 0x1
    }
}

/// 机械臂末端位姿反馈 (0x2A2/0x2A3/0x2A4), 单位 0.001 mm / 0.001 度
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackEndPose {
    pub x_axis: i32,
    pub y_axis: i32,
    pub z_axis: i32,
    pub rx_axis: i32,
    pub ry_axis: i32,
    pub rz_axis: i32,
}

/// 关节角度反馈 (0x2A5/0x2A6/0x2A7), 单位 0.001 度
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackJointStates {
    pub joint_1: i32,
    pub joint_2: i32,
    pub joint_3: i32,
    pub joint_4: i32,
    pub joint_5: i32,
    pub joint_6: i32,
}

/// 夹爪反馈状态位 (0x2A8 status_code)
#[derive(Debug, Clone, Copy, Default)]
pub struct GripperFocStatus {
    pub voltage_too_low: bool,
    pub motor_overheating: bool,
    pub driver_overcurrent: bool,
    pub driver_overheating: bool,
    pub sensor_status: bool,
    pub driver_error_status: bool,
    pub driver_enable_status: bool,
    pub homing_status: bool,
}

/// 夹爪反馈 (0x2A8)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackGripper {
    /// 行程, 单位 0.001 mm
    pub grippers_angle: i32,
    /// 力矩, 单位 0.001 N·m
    pub grippers_effort: i16,
    /// 状态码
    pub status_code: u8,
    pub foc_status: GripperFocStatus,
}

impl ArmMsgFeedbackGripper {
    pub fn set_status_code(&mut self, code: u8) {
        self.status_code = code;
        self.foc_status.voltage_too_low = code & (1 << 0) != 0;
        self.foc_status.motor_overheating = code & (1 << 1) != 0;
        self.foc_status.driver_overcurrent = code & (1 << 2) != 0;
        self.foc_status.driver_overheating = code & (1 << 3) != 0;
        self.foc_status.sensor_status = code & (1 << 4) != 0;
        self.foc_status.driver_error_status = code & (1 << 5) != 0;
        self.foc_status.driver_enable_status = code & (1 << 6) != 0;
        self.foc_status.homing_status = code & (1 << 7) != 0;
    }
}

/// 驱动器信息高速反馈 (0x251~0x256)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackHighSpd {
    pub can_id: u32,
    /// 单位 0.001 rad/s
    pub motor_speed: i16,
    /// 单位 0.001 A
    pub current: i16,
    /// 单位 rad
    pub pos: i32,
    /// 单位 0.001 N·m (换算得到)
    pub effort: f64,
}

impl ArmMsgFeedbackHighSpd {
    /// 扭矩换算系数。
    pub fn cal_effort(&mut self) -> f64 {
        const COEFF_1_3: f64 = 1.18125;
        const COEFF_4_6: f64 = 0.95844;
        self.effort = match self.can_id {
            0x251..=0x253 => f64::from(self.current) * COEFF_1_3,
            0x254..=0x256 => f64::from(self.current) * COEFF_4_6,
            _ => f64::from(self.current) * COEFF_1_3,
        };
        self.effort
    }
}

/// 驱动器低速反馈状态位 (0x261~0x266 foc_status_code)
#[derive(Debug, Clone, Copy, Default)]
pub struct LowSpdFocStatus {
    pub voltage_too_low: bool,
    pub motor_overheating: bool,
    pub driver_overcurrent: bool,
    pub driver_overheating: bool,
    pub collision_status: bool,
    pub driver_error_status: bool,
    pub driver_enable_status: bool,
    pub stall_status: bool,
}

/// 驱动器信息低速反馈 (0x261~0x266)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackLowSpd {
    pub can_id: u32,
    /// 单位 0.1 V
    pub vol: u16,
    /// 单位 ℃
    pub foc_temp: i16,
    /// 单位 ℃
    pub motor_temp: i8,
    pub foc_status_code: u8,
    pub foc_status: LowSpdFocStatus,
    /// 单位 0.001 A
    pub bus_current: u16,
}

impl ArmMsgFeedbackLowSpd {
    pub fn set_foc_status_code(&mut self, code: u8) {
        self.foc_status_code = code;
        self.foc_status.voltage_too_low = code & (1 << 0) != 0;
        self.foc_status.motor_overheating = code & (1 << 1) != 0;
        self.foc_status.driver_overcurrent = code & (1 << 2) != 0;
        self.foc_status.driver_overheating = code & (1 << 3) != 0;
        self.foc_status.collision_status = code & (1 << 4) != 0;
        self.foc_status.driver_error_status = code & (1 << 5) != 0;
        self.foc_status.driver_enable_status = code & (1 << 6) != 0;
        self.foc_status.stall_status = code & (1 << 7) != 0;
    }
}

/// 反馈当前电机限制角度/最大速度 (0x473)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd {
    pub motor_num: u8,
    /// 最大角度限制, 单位 0.1°
    pub max_angle_limit: i16,
    /// 最小角度限制, 单位 0.1°
    pub min_angle_limit: i16,
    /// 最大关节速度, 单位 0.001 rad/s
    pub max_joint_spd: u16,
}

/// 反馈当前末端速度/加速度参数 (0x478)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackCurrentEndVelAccParam {
    pub end_max_linear_vel: u16,
    pub end_max_angular_vel: u16,
    pub end_max_linear_acc: u16,
    pub end_max_angular_acc: u16,
}

/// 碰撞防护等级反馈 (0x47B)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackCrashProtectionRating {
    pub joint_1_protection_level: u8,
    pub joint_2_protection_level: u8,
    pub joint_3_protection_level: u8,
    pub joint_4_protection_level: u8,
    pub joint_5_protection_level: u8,
    pub joint_6_protection_level: u8,
}

/// 反馈当前电机最大加速度限制 (0x47C)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackCurrentMotorMaxAccLimit {
    pub joint_motor_num: u8,
    /// 单位 0.001 rad/s²
    pub max_joint_acc: u16,
}

/// 夹爪/示教器参数反馈 (0x47E)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackGripperTeachingPendantParam {
    pub teaching_range_per: u8,
    pub max_range_config: u8,
    pub teaching_friction: u8,
}

/// 设置指令应答反馈 (0x476)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackRespSetInstruction {
    /// `-1` is used as the "cleared" sentinel (see `ClearRespSetInstruction`).
    pub instruction_index: i16,
    pub is_set_zero_successfully: i16,
}

/// 反馈当前关节的末端速度/加速度 (0x481~0x486)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgFeedbackJointVelAcc {
    pub can_id: u32,
    pub end_linear_vel: u16,
    pub end_angular_vel: u16,
    pub end_linear_acc: u16,
    pub end_angular_acc: u16,
}
