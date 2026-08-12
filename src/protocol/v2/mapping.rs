//! Mapping between CAN IDs and message types.

use crate::error::Error;

use super::can_id as id;
use super::msg_type::MsgType;

/// Map a CAN ID to its message type. Returns `None` for unknown IDs.
pub fn type_from_id(can_id: u32) -> Option<MsgType> {
    use MsgType::*;
    let t = match can_id {
        id::ARM_STATUS_FEEDBACK => StatusFeedback,
        id::ARM_END_POSE_FEEDBACK_1 => EndPoseFeedback1,
        id::ARM_END_POSE_FEEDBACK_2 => EndPoseFeedback2,
        id::ARM_END_POSE_FEEDBACK_3 => EndPoseFeedback3,
        id::ARM_JOINT_FEEDBACK_12 => JointFeedback12,
        id::ARM_JOINT_FEEDBACK_34 => JointFeedback34,
        id::ARM_JOINT_FEEDBACK_56 => JointFeedback56,
        id::ARM_GRIPPER_FEEDBACK => GripperFeedback,
        id::ARM_INFO_HIGH_SPD_FEEDBACK_1 => HighSpdFeedback1,
        id::ARM_INFO_HIGH_SPD_FEEDBACK_2 => HighSpdFeedback2,
        id::ARM_INFO_HIGH_SPD_FEEDBACK_3 => HighSpdFeedback3,
        id::ARM_INFO_HIGH_SPD_FEEDBACK_4 => HighSpdFeedback4,
        id::ARM_INFO_HIGH_SPD_FEEDBACK_5 => HighSpdFeedback5,
        id::ARM_INFO_HIGH_SPD_FEEDBACK_6 => HighSpdFeedback6,
        id::ARM_INFO_LOW_SPD_FEEDBACK_1 => LowSpdFeedback1,
        id::ARM_INFO_LOW_SPD_FEEDBACK_2 => LowSpdFeedback2,
        id::ARM_INFO_LOW_SPD_FEEDBACK_3 => LowSpdFeedback3,
        id::ARM_INFO_LOW_SPD_FEEDBACK_4 => LowSpdFeedback4,
        id::ARM_INFO_LOW_SPD_FEEDBACK_5 => LowSpdFeedback5,
        id::ARM_INFO_LOW_SPD_FEEDBACK_6 => LowSpdFeedback6,
        id::ARM_MOTION_CTRL_1 => MotionCtrl1,
        id::ARM_MOTION_CTRL_2 => MotionCtrl2,
        id::ARM_MOTION_CTRL_CARTESIAN_1 => MotionCtrlCartesian1,
        id::ARM_MOTION_CTRL_CARTESIAN_2 => MotionCtrlCartesian2,
        id::ARM_MOTION_CTRL_CARTESIAN_3 => MotionCtrlCartesian3,
        id::ARM_JOINT_CTRL_12 => JointCtrl12,
        id::ARM_JOINT_CTRL_34 => JointCtrl34,
        id::ARM_JOINT_CTRL_56 => JointCtrl56,
        id::ARM_CIRCULAR_PATTERN_COORD_NUM_UPDATE_CTRL => CircularPatternCoordNumUpdateCtrl,
        id::ARM_GRIPPER_CTRL => GripperCtrl,
        id::ARM_JOINT_MIT_CTRL_1 => JointMitCtrl1,
        id::ARM_JOINT_MIT_CTRL_2 => JointMitCtrl2,
        id::ARM_JOINT_MIT_CTRL_3 => JointMitCtrl3,
        id::ARM_JOINT_MIT_CTRL_4 => JointMitCtrl4,
        id::ARM_JOINT_MIT_CTRL_5 => JointMitCtrl5,
        id::ARM_JOINT_MIT_CTRL_6 => JointMitCtrl6,
        id::ARM_MASTER_SLAVE_MODE_CONFIG => MasterSlaveModeConfig,
        id::ARM_MOTOR_ENABLE_DISABLE_CONFIG => MotorEnableDisableConfig,
        id::ARM_SEARCH_MOTOR_MAX_SPD_ACC_LIMIT => SearchMotorMaxAngleSpdAccLimit,
        id::ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD => FeedbackCurrentMotorAngleLimitMaxSpd,
        id::ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET => MotorAngleLimitMaxSpdSet,
        id::ARM_JOINT_CONFIG => JointConfig,
        // 0x476 serves as both InstructionResponseConfig (tx) and
        // FeedbackRespSetInstruction (rx); the receive-side mapping wins for
        // id->type: the receive-side meaning wins for dual-use IDs.
        id::ARM_FEEDBACK_RESP_SET_INSTRUCTION => FeedbackRespSetInstruction,
        id::ARM_PARAM_ENQUIRY_AND_CONFIG => ParamEnquiryAndConfig,
        id::ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM => FeedbackCurrentEndVelAccParam,
        id::ARM_END_VEL_ACC_PARAM_CONFIG => EndVelAccParamConfig,
        id::ARM_CRASH_PROTECTION_RATING_CONFIG => CrashProtectionRatingConfig,
        id::ARM_CRASH_PROTECTION_RATING_FEEDBACK => CrashProtectionRatingFeedback,
        id::ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT => FeedbackCurrentMotorMaxAccLimit,
        id::ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG => GripperTeachingPendantParamConfig,
        id::ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK => GripperTeachingPendantParamFeedback,
        id::ARM_FEEDBACK_JOINT_VEL_ACC_1 => FeedbackJointVelAcc1,
        id::ARM_FEEDBACK_JOINT_VEL_ACC_2 => FeedbackJointVelAcc2,
        id::ARM_FEEDBACK_JOINT_VEL_ACC_3 => FeedbackJointVelAcc3,
        id::ARM_FEEDBACK_JOINT_VEL_ACC_4 => FeedbackJointVelAcc4,
        id::ARM_FEEDBACK_JOINT_VEL_ACC_5 => FeedbackJointVelAcc5,
        id::ARM_FEEDBACK_JOINT_VEL_ACC_6 => FeedbackJointVelAcc6,
        id::ARM_LIGHT_CTRL => LightCtrl,
        id::ARM_CAN_UPDATE_SILENT_MODE_CONFIG => CanUpdateSilentModeConfig,
        id::ARM_FIRMWARE_READ => FirmwareRead,
        _ => return None,
    };
    Some(t)
}

/// Map a message type to its CAN ID. Returns an error for types without an ID.
pub fn id_from_type(msg_type: MsgType) -> Result<u32, Error> {
    use MsgType::*;
    let id = match msg_type {
        StatusFeedback => id::ARM_STATUS_FEEDBACK,
        EndPoseFeedback1 => id::ARM_END_POSE_FEEDBACK_1,
        EndPoseFeedback2 => id::ARM_END_POSE_FEEDBACK_2,
        EndPoseFeedback3 => id::ARM_END_POSE_FEEDBACK_3,
        JointFeedback12 => id::ARM_JOINT_FEEDBACK_12,
        JointFeedback34 => id::ARM_JOINT_FEEDBACK_34,
        JointFeedback56 => id::ARM_JOINT_FEEDBACK_56,
        GripperFeedback => id::ARM_GRIPPER_FEEDBACK,
        HighSpdFeedback1 => id::ARM_INFO_HIGH_SPD_FEEDBACK_1,
        HighSpdFeedback2 => id::ARM_INFO_HIGH_SPD_FEEDBACK_2,
        HighSpdFeedback3 => id::ARM_INFO_HIGH_SPD_FEEDBACK_3,
        HighSpdFeedback4 => id::ARM_INFO_HIGH_SPD_FEEDBACK_4,
        HighSpdFeedback5 => id::ARM_INFO_HIGH_SPD_FEEDBACK_5,
        HighSpdFeedback6 => id::ARM_INFO_HIGH_SPD_FEEDBACK_6,
        LowSpdFeedback1 => id::ARM_INFO_LOW_SPD_FEEDBACK_1,
        LowSpdFeedback2 => id::ARM_INFO_LOW_SPD_FEEDBACK_2,
        LowSpdFeedback3 => id::ARM_INFO_LOW_SPD_FEEDBACK_3,
        LowSpdFeedback4 => id::ARM_INFO_LOW_SPD_FEEDBACK_4,
        LowSpdFeedback5 => id::ARM_INFO_LOW_SPD_FEEDBACK_5,
        LowSpdFeedback6 => id::ARM_INFO_LOW_SPD_FEEDBACK_6,
        MotionCtrl1 => id::ARM_MOTION_CTRL_1,
        MotionCtrl2 => id::ARM_MOTION_CTRL_2,
        MotionCtrlCartesian1 => id::ARM_MOTION_CTRL_CARTESIAN_1,
        MotionCtrlCartesian2 => id::ARM_MOTION_CTRL_CARTESIAN_2,
        MotionCtrlCartesian3 => id::ARM_MOTION_CTRL_CARTESIAN_3,
        JointCtrl12 => id::ARM_JOINT_CTRL_12,
        JointCtrl34 => id::ARM_JOINT_CTRL_34,
        JointCtrl56 => id::ARM_JOINT_CTRL_56,
        CircularPatternCoordNumUpdateCtrl => id::ARM_CIRCULAR_PATTERN_COORD_NUM_UPDATE_CTRL,
        GripperCtrl => id::ARM_GRIPPER_CTRL,
        JointMitCtrl1 => id::ARM_JOINT_MIT_CTRL_1,
        JointMitCtrl2 => id::ARM_JOINT_MIT_CTRL_2,
        JointMitCtrl3 => id::ARM_JOINT_MIT_CTRL_3,
        JointMitCtrl4 => id::ARM_JOINT_MIT_CTRL_4,
        JointMitCtrl5 => id::ARM_JOINT_MIT_CTRL_5,
        JointMitCtrl6 => id::ARM_JOINT_MIT_CTRL_6,
        MasterSlaveModeConfig => id::ARM_MASTER_SLAVE_MODE_CONFIG,
        MotorEnableDisableConfig => id::ARM_MOTOR_ENABLE_DISABLE_CONFIG,
        SearchMotorMaxAngleSpdAccLimit => id::ARM_SEARCH_MOTOR_MAX_SPD_ACC_LIMIT,
        FeedbackCurrentMotorAngleLimitMaxSpd => id::ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD,
        MotorAngleLimitMaxSpdSet => id::ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET,
        JointConfig => id::ARM_JOINT_CONFIG,
        InstructionResponseConfig => id::ARM_INSTRUCTION_RESPONSE_CONFIG,
        FeedbackRespSetInstruction => id::ARM_FEEDBACK_RESP_SET_INSTRUCTION,
        ParamEnquiryAndConfig => id::ARM_PARAM_ENQUIRY_AND_CONFIG,
        FeedbackCurrentEndVelAccParam => id::ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM,
        EndVelAccParamConfig => id::ARM_END_VEL_ACC_PARAM_CONFIG,
        CrashProtectionRatingConfig => id::ARM_CRASH_PROTECTION_RATING_CONFIG,
        CrashProtectionRatingFeedback => id::ARM_CRASH_PROTECTION_RATING_FEEDBACK,
        FeedbackCurrentMotorMaxAccLimit => id::ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT,
        GripperTeachingPendantParamConfig => id::ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG,
        GripperTeachingPendantParamFeedback => id::ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK,
        FeedbackJointVelAcc1 => id::ARM_FEEDBACK_JOINT_VEL_ACC_1,
        FeedbackJointVelAcc2 => id::ARM_FEEDBACK_JOINT_VEL_ACC_2,
        FeedbackJointVelAcc3 => id::ARM_FEEDBACK_JOINT_VEL_ACC_3,
        FeedbackJointVelAcc4 => id::ARM_FEEDBACK_JOINT_VEL_ACC_4,
        FeedbackJointVelAcc5 => id::ARM_FEEDBACK_JOINT_VEL_ACC_5,
        FeedbackJointVelAcc6 => id::ARM_FEEDBACK_JOINT_VEL_ACC_6,
        LightCtrl => id::ARM_LIGHT_CTRL,
        CanUpdateSilentModeConfig => id::ARM_CAN_UPDATE_SILENT_MODE_CONFIG,
        FirmwareRead => id::ARM_FIRMWARE_READ,
        Unknown => return Err(Error::UnknownId(0)),
    };
    Ok(id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_id_type() {
        for t in [
            MsgType::StatusFeedback,
            MsgType::JointFeedback12,
            MsgType::GripperCtrl,
            MsgType::JointMitCtrl6,
            MsgType::FirmwareRead,
        ] {
            let id = id_from_type(t).unwrap();
            assert_eq!(type_from_id(id), Some(t));
        }
    }

    #[test]
    fn unknown_id_is_none() {
        assert_eq!(type_from_id(0x777), None);
    }
}
