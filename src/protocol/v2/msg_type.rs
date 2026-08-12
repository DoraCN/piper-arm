//! Message type enumeration for the Piper V2 protocol.

/// Message types used to describe the kind of a CAN frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum MsgType {
    Unknown = 0x00,
    // feedback
    StatusFeedback,
    EndPoseFeedback1,
    EndPoseFeedback2,
    EndPoseFeedback3,
    JointFeedback12,
    JointFeedback34,
    JointFeedback56,
    GripperFeedback,
    HighSpdFeedback1,
    HighSpdFeedback2,
    HighSpdFeedback3,
    HighSpdFeedback4,
    HighSpdFeedback5,
    HighSpdFeedback6,
    LowSpdFeedback1,
    LowSpdFeedback2,
    LowSpdFeedback3,
    LowSpdFeedback4,
    LowSpdFeedback5,
    LowSpdFeedback6,
    // transmit
    MotionCtrl1,
    MotionCtrl2,
    MotionCtrlCartesian1,
    MotionCtrlCartesian2,
    MotionCtrlCartesian3,
    JointCtrl12,
    JointCtrl34,
    JointCtrl56,
    CircularPatternCoordNumUpdateCtrl,
    GripperCtrl,
    JointMitCtrl1,
    JointMitCtrl2,
    JointMitCtrl3,
    JointMitCtrl4,
    JointMitCtrl5,
    JointMitCtrl6,
    MasterSlaveModeConfig,
    MotorEnableDisableConfig,
    SearchMotorMaxAngleSpdAccLimit,
    FeedbackCurrentMotorAngleLimitMaxSpd,
    MotorAngleLimitMaxSpdSet,
    JointConfig,
    InstructionResponseConfig,
    FeedbackRespSetInstruction,
    ParamEnquiryAndConfig,
    FeedbackCurrentEndVelAccParam,
    EndVelAccParamConfig,
    CrashProtectionRatingConfig,
    CrashProtectionRatingFeedback,
    FeedbackCurrentMotorMaxAccLimit,
    GripperTeachingPendantParamConfig,
    GripperTeachingPendantParamFeedback,
    FeedbackJointVelAcc1,
    FeedbackJointVelAcc2,
    FeedbackJointVelAcc3,
    FeedbackJointVelAcc4,
    FeedbackJointVelAcc5,
    FeedbackJointVelAcc6,
    LightCtrl,
    CanUpdateSilentModeConfig,
    FirmwareRead,
}

impl MsgType {
    /// Return `true` if this type is a feedback (arm -> host) message.
    pub fn is_feedback(&self) -> bool {
        matches!(
            self,
            MsgType::StatusFeedback
                | MsgType::EndPoseFeedback1
                | MsgType::EndPoseFeedback2
                | MsgType::EndPoseFeedback3
                | MsgType::JointFeedback12
                | MsgType::JointFeedback34
                | MsgType::JointFeedback56
                | MsgType::GripperFeedback
                | MsgType::HighSpdFeedback1
                | MsgType::HighSpdFeedback2
                | MsgType::HighSpdFeedback3
                | MsgType::HighSpdFeedback4
                | MsgType::HighSpdFeedback5
                | MsgType::HighSpdFeedback6
                | MsgType::LowSpdFeedback1
                | MsgType::LowSpdFeedback2
                | MsgType::LowSpdFeedback3
                | MsgType::LowSpdFeedback4
                | MsgType::LowSpdFeedback5
                | MsgType::LowSpdFeedback6
                | MsgType::FeedbackCurrentMotorAngleLimitMaxSpd
                | MsgType::FeedbackRespSetInstruction
                | MsgType::FeedbackCurrentEndVelAccParam
                | MsgType::CrashProtectionRatingFeedback
                | MsgType::FeedbackCurrentMotorMaxAccLimit
                | MsgType::GripperTeachingPendantParamFeedback
                | MsgType::FeedbackJointVelAcc1
                | MsgType::FeedbackJointVelAcc2
                | MsgType::FeedbackJointVelAcc3
                | MsgType::FeedbackJointVelAcc4
                | MsgType::FeedbackJointVelAcc5
                | MsgType::FeedbackJointVelAcc6
                | MsgType::FirmwareRead
        )
    }
}
