//! Transmit (control) message data structures.

/// 机械臂运动控制指令1 (0x150)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgMotionCtrl1 {
    pub emergency_stop: u8,
    pub track_ctrl: u8,
    pub grag_teach_ctrl: u8,
}

/// 机械臂运动控制指令2 (0x151)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgMotionCtrl2 {
    pub ctrl_mode: u8,
    pub move_mode: u8,
    pub move_spd_rate_ctrl: u8,
    pub mit_mode: u8,
    pub residence_time: u8,
    pub installation_pos: u8,
}

/// 机械臂运动控制直角坐标系指令 (0x152/0x153/0x154)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgMotionCtrlCartesian {
    pub x_axis: i32,
    pub y_axis: i32,
    pub z_axis: i32,
    pub rx_axis: i32,
    pub ry_axis: i32,
    pub rz_axis: i32,
}

/// 关节控制指令 (0x155/0x156/0x157), 单位 0.001 度
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgJointCtrl {
    pub joint_1: i32,
    pub joint_2: i32,
    pub joint_3: i32,
    pub joint_4: i32,
    pub joint_5: i32,
    pub joint_6: i32,
}

/// MoveC 模式坐标序号更新指令 (0x158)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgCircularPatternCoordNumUpdateCtrl {
    pub instruction_num: u8,
}

/// 夹爪控制指令 (0x159)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgGripperCtrl {
    /// 行程, 单位 0.001 mm
    pub grippers_angle: i32,
    /// 力矩, 单位 0.001 N·m, 范围 0-5000
    pub grippers_effort: u16,
    /// 0x00 失能; 0x01 使能; 0x02 失能清除错误; 0x03 使能清除错误
    pub status_code: u8,
    /// 0x00 无效; 0xAE 设置零点
    pub set_zero: u8,
}

/// 关节 MIT 控制指令 (0x15A~0x15F)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgJointMitCtrl {
    /// 目标位置, 量化后 (16 bit)
    pub pos_ref: u16,
    /// 目标速度, 量化后 (12 bit)
    pub vel_ref: u16,
    /// 比例增益, 量化后 (12 bit)
    pub kp: u16,
    /// 微分增益, 量化后 (12 bit)
    pub kd: u16,
    /// 目标力矩, 量化后 (8 bit)
    pub t_ref: u8,
    /// 校验和 (4 bit)
    pub crc: u8,
}

/// 随动主从模式设置指令 (0x470)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgMasterSlaveModeConfig {
    pub linkage_config: u8,
    pub feedback_offset: u8,
    pub ctrl_offset: u8,
    pub linkage_offset: u8,
}

/// 电机使能/失能设置指令 (0x471)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgMotorEnableDisableConfig {
    pub motor_num: u8,
    /// 0x01 失能; 0x02 使能
    pub enable_flag: u8,
}

/// 查询电机角度/最大速度/最大加速度限制指令 (0x472)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgSearchMotorMaxAngleSpdAccLimit {
    pub motor_num: u8,
    /// 0x01 查询电机角度/最大速度; 0x02 查询电机最大加速度限制
    pub search_content: u8,
}

/// 电机角度限制/最大速度设置指令 (0x474)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgMotorAngleLimitMaxSpdSet {
    pub motor_num: u8,
    /// 最大角度限制, 单位 0.1°, 0x7FFF 为无效数值
    pub max_angle_limit: i16,
    /// 最小角度限制, 单位 0.1°, 0x7FFF 为无效数值
    pub min_angle_limit: i16,
    /// 最大关节速度, 单位 0.001 rad/s, 0x7FFF 为无效数值
    pub max_joint_spd: u16,
}

/// 关节设置指令 (0x475)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgJointConfig {
    pub joint_motor_num: u8,
    /// 设置当前位置为零点, 有效值 0xAE
    pub set_motor_current_pos_as_zero: u8,
    /// 加速度参数设置是否生效, 有效值 0xAE
    pub acc_param_config_is_effective_or_not: u8,
    /// 最大关节加速度, 单位 0.01 rad/s²
    pub max_joint_acc: u16,
    /// 清除关节错误代码, 有效值 0xAE
    pub clear_joint_err: u8,
}

/// 设置指令应答 (0x476)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgInstructionResponseConfig {
    pub instruction_index: u8,
    pub zero_config_success_flag: u8,
}

/// 机械臂参数查询与设置指令 (0x477)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgParamEnquiryAndConfig {
    pub param_enquiry: u8,
    pub param_setting: u8,
    pub data_feedback_0x48x: u8,
    pub end_load_param_setting_effective: u8,
    pub set_end_load: u8,
}

/// 末端速度/加速度参数设置指令 (0x479)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgEndVelAccParamConfig {
    pub end_max_linear_vel: u16,
    pub end_max_angular_vel: u16,
    pub end_max_linear_acc: u16,
    pub end_max_angular_acc: u16,
}

/// 碰撞防护等级设置指令 (0x47A)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgCrashProtectionRatingConfig {
    pub joint_1_protection_level: u8,
    pub joint_2_protection_level: u8,
    pub joint_3_protection_level: u8,
    pub joint_4_protection_level: u8,
    pub joint_5_protection_level: u8,
    pub joint_6_protection_level: u8,
}

/// 夹爪/示教器参数设置指令 (0x47D)
#[derive(Debug, Clone, Copy, Default)]
pub struct ArmMsgGripperTeachingPendantParamConfig {
    pub teaching_range_per: u8,
    pub max_range_config: u8,
    pub teaching_friction: u8,
}
