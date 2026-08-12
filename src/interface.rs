//! High-level Piper interface.
//!
//! A background reader thread decodes incoming CAN frames and updates a
//! mutex-protected snapshot of the latest feedback state. Control methods
//! encode commands and send them over the CAN bus.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use crate::can::CanBus;
use crate::error::{Error, Result};
use crate::kinematics::PiperForwardKinematics;
use crate::param::PiperParamManager;
use crate::protocol::{ControlFrame, DecodedMessage};
use crate::protocol::v2::messages::*;
use crate::utils::fps::FpsCounter;

const CAN_READ_TIMEOUT: Duration = Duration::from_millis(50);
const FPS_TICK_INTERVAL: Duration = Duration::from_millis(100);
const IS_OK_ZERO_TICKS: usize = 5;

/// Shared state between the background threads and the public interface.
struct Inner {
    state: Mutex<LatestState>,
    stop: Arc<AtomicBool>,
    is_ok: Arc<AtomicBool>,
    firmware: Mutex<Vec<u8>>,
    param: Mutex<PiperParamManager>,
    feedback_fk: Mutex<[[f64; 6]; 6]>,
    ctrl_fk: Mutex<[[f64; 6]; 6]>,
    fps: Arc<FpsCounter>,
    zero_ticks: Arc<Mutex<usize>>,
    start_sdk_joint_limit: AtomicBool,
    start_sdk_gripper_limit: AtomicBool,
    filter_abnormal_data: AtomicBool,
    start_sdk_fk_cal: AtomicBool,
}

impl Inner {
    fn new() -> Self {
        let fps = FpsCounter::new();
        for name in [
            "CanMonitor",
            "ArmStatus",
            "ArmEndPose_XY",
            "ArmEndPose_ZRX",
            "ArmEndPose_RYRZ",
            "ArmJoint_12",
            "ArmJoint_34",
            "ArmJoint_56",
            "ArmGripper",
            "ArmJointCtrl_12",
            "ArmGripperCtrl",
            "ArmCtrlCode_151",
        ] {
            fps.add_variable(name);
        }
        for n in 1..=6 {
            fps.add_variable(&format!("ArmMotorDriverInfoHighSpd_{n}"));
            fps.add_variable(&format!("ArmMotorDriverInfoLowSpd_{n}"));
        }
        Self {
            state: Mutex::new(LatestState::default()),
            stop: Arc::new(AtomicBool::new(false)),
            is_ok: Arc::new(AtomicBool::new(true)),
            firmware: Mutex::new(Vec::new()),
            param: Mutex::new(PiperParamManager::new()),
            feedback_fk: Mutex::new([[0.0; 6]; 6]),
            ctrl_fk: Mutex::new([[0.0; 6]; 6]),
            fps: Arc::new(fps),
            zero_ticks: Arc::new(Mutex::new(0)),
            start_sdk_joint_limit: AtomicBool::new(false),
            start_sdk_gripper_limit: AtomicBool::new(false),
            filter_abnormal_data: AtomicBool::new(true),
            start_sdk_fk_cal: AtomicBool::new(false),
        }
    }
}

/// Latest feedback state snapshot.
#[derive(Debug, Clone)]
pub struct LatestState {
    pub time_stamp_status: f64,
    pub arm_status: ArmMsgFeedbackStatus,
    pub time_stamp_end_pose: f64,
    pub arm_end_pose: ArmMsgFeedbackEndPose,
    pub time_stamp_joint: f64,
    pub arm_joint: ArmMsgFeedbackJointStates,
    pub time_stamp_gripper: f64,
    pub arm_gripper: ArmMsgFeedbackGripper,
    pub motor_high: [ArmMsgFeedbackHighSpd; 6],
    pub motor_low: [ArmMsgFeedbackLowSpd; 6],
    pub current_motor_angle_limit: ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd,
    pub current_end_vel_acc: ArmMsgFeedbackCurrentEndVelAccParam,
    pub crash_protection: ArmMsgFeedbackCrashProtectionRating,
    pub gripper_teaching_param: ArmMsgFeedbackGripperTeachingPendantParam,
    pub current_motor_max_acc: ArmMsgFeedbackCurrentMotorMaxAccLimit,
    /// index 0 unused, 1..=6 filled.
    pub all_motor_angle_limit: [ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd; 7],
    pub all_motor_max_acc: [ArmMsgFeedbackCurrentMotorMaxAccLimit; 7],
    pub time_stamp_joint_ctrl: f64,
    pub joint_ctrl: ArmMsgJointCtrl,
    pub time_stamp_gripper_ctrl: f64,
    pub gripper_ctrl: ArmMsgGripperCtrl,
    pub time_stamp_ctrl_151: f64,
    pub ctrl_151: ArmMsgMotionCtrl2,
    pub resp_set_instruction: ArmMsgFeedbackRespSetInstruction,
    pub joint_vel_acc: [ArmMsgFeedbackJointVelAcc; 7],
}

impl Default for LatestState {
    fn default() -> Self {
        Self {
            time_stamp_status: 0.0,
            arm_status: ArmMsgFeedbackStatus::default(),
            time_stamp_end_pose: 0.0,
            arm_end_pose: ArmMsgFeedbackEndPose::default(),
            time_stamp_joint: 0.0,
            arm_joint: ArmMsgFeedbackJointStates::default(),
            time_stamp_gripper: 0.0,
            arm_gripper: ArmMsgFeedbackGripper::default(),
            motor_high: [ArmMsgFeedbackHighSpd::default(); 6],
            motor_low: [ArmMsgFeedbackLowSpd::default(); 6],
            current_motor_angle_limit: ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd::default(),
            current_end_vel_acc: ArmMsgFeedbackCurrentEndVelAccParam::default(),
            crash_protection: ArmMsgFeedbackCrashProtectionRating::default(),
            gripper_teaching_param: ArmMsgFeedbackGripperTeachingPendantParam::default(),
            current_motor_max_acc: ArmMsgFeedbackCurrentMotorMaxAccLimit::default(),
            all_motor_angle_limit: [ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd::default(); 7],
            all_motor_max_acc: [ArmMsgFeedbackCurrentMotorMaxAccLimit::default(); 7],
            time_stamp_joint_ctrl: 0.0,
            joint_ctrl: ArmMsgJointCtrl::default(),
            time_stamp_gripper_ctrl: 0.0,
            gripper_ctrl: ArmMsgGripperCtrl::default(),
            time_stamp_ctrl_151: 0.0,
            ctrl_151: ArmMsgMotionCtrl2::default(),
            resp_set_instruction: ArmMsgFeedbackRespSetInstruction::default(),
            joint_vel_acc: [ArmMsgFeedbackJointVelAcc::default(); 7],
        }
    }
}

/// The Piper robotic arm interface.
pub struct PiperInterface {
    inner: Arc<Inner>,
    can: Arc<dyn CanBus>,
    rx_thread: Mutex<Option<JoinHandle<()>>>,
    monitor_thread: Mutex<Option<JoinHandle<()>>>,
    can_name: String,
    connected: AtomicBool,
    fk: PiperForwardKinematics,
}

impl Clone for PiperInterface {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
            can: self.can.clone(),
            rx_thread: Mutex::new(None),
            monitor_thread: Mutex::new(None),
            can_name: self.can_name.clone(),
            connected: AtomicBool::new(self.connected.load(Ordering::SeqCst)),
            fk: self.fk.clone(),
        }
    }
}

impl PiperInterface {
    /// Create an interface over an existing CAN bus and start the background
    /// reader and monitor threads.
    pub fn new(can: Arc<dyn CanBus>) -> Result<Self> {
        let inner = Arc::new(Inner::new());
        let this = Self {
            inner: inner.clone(),
            can: can.clone(),
            rx_thread: Mutex::new(None),
            monitor_thread: Mutex::new(None),
            can_name: String::new(),
            connected: AtomicBool::new(true),
            fk: PiperForwardKinematics::new(true),
        };
        this.start_threads(inner);
        Ok(this)
    }

    /// Convenience constructor over a SocketCAN interface by name.
    pub fn open_socketcan(can_name: &str) -> Result<Self> {
        let bus = crate::can::socketcan::SocketCanBus::open_with_timeout(
            can_name,
            CAN_READ_TIMEOUT,
        )?;
        let mut this = Self::new(Arc::new(bus))?;
        this.can_name = can_name.to_string();
        Ok(this)
    }

    fn start_threads(&self, inner: Arc<Inner>) {
        let stop = Arc::clone(&inner.stop);
        let can = self.can.clone();
        let inner_reader = inner.clone();
        let fk = self.fk.clone();
        let reader = std::thread::Builder::new()
            .name("piper-read-can".into())
            .spawn(move || {
                loop {
                    if stop.load(Ordering::SeqCst) {
                        break;
                    }
                    match can.read_frame() {
                        Ok((id, data)) => {
                            if let Ok(Some(msg)) = crate::protocol::decode(id, &data) {
                                inner_reader.fps.increment("CanMonitor");
                                handle_message(&msg, &inner_reader, &fk);
                            }
                        }
                        Err(e) => {
                            if stop.load(Ordering::SeqCst) {
                                break;
                            }
                            match e {
                                Error::Io(ref io) if is_timeout(io) => {}
                                Error::NotConnected => {
                                    std::thread::sleep(Duration::from_millis(1));
                                }
                                _ => {
                                    log::warn!("piper read error: {e}");
                                    std::thread::sleep(Duration::from_millis(1));
                                }
                            }
                        }
                    }
                }
            })
            .map_err(|e| log::warn!("failed to spawn reader thread: {e}"))
            .ok();
        *self.rx_thread.lock().unwrap() = reader;

        let stop = Arc::clone(&inner.stop);
        let fps = Arc::clone(&inner.fps);
        let is_ok = Arc::clone(&inner.is_ok);
        let zero_ticks = Arc::clone(&inner.zero_ticks);
        let monitor = std::thread::Builder::new()
            .name("piper-can-monitor".into())
            .spawn(move || {
                while !stop.load(Ordering::SeqCst) {
                    std::thread::sleep(FPS_TICK_INTERVAL);
                    fps.tick();
                    let can_fps = fps.get_fps("CanMonitor");
                    let mut zt = zero_ticks.lock().unwrap();
                    if can_fps == 0.0 {
                        *zt += 1;
                    } else {
                        *zt = 0;
                    }
                    is_ok.store(*zt < IS_OK_ZERO_TICKS, Ordering::SeqCst);
                }
            })
            .map_err(|e| log::warn!("failed to spawn monitor thread: {e}"))
            .ok();
        *self.monitor_thread.lock().unwrap() = monitor;
    }

    // ------------------------------------------------------------------
    // Connection status
    // ------------------------------------------------------------------

    /// Stop the background threads (idempotent).
    pub fn disconnect(&self) {
        if !self.connected.swap(false, Ordering::SeqCst)
            && self.rx_thread.lock().unwrap().is_none()
        {
            return;
        }
        self.inner.stop.store(true, Ordering::SeqCst);
        if let Some(th) = self.rx_thread.lock().unwrap().take() {
            let _ = th.join();
        }
        if let Some(th) = self.monitor_thread.lock().unwrap().take() {
            let _ = th.join();
        }
    }

    /// Whether the interface is connected.
    pub fn get_connect_status(&self) -> bool {
        self.connected.load(Ordering::SeqCst)
    }

    /// Whether the CAN read stream is healthy.
    pub fn is_ok(&self) -> bool {
        self.inner.is_ok.load(Ordering::SeqCst)
    }

    /// The CAN interface name (if set).
    pub fn get_can_name(&self) -> &str {
        &self.can_name
    }

    /// Current CAN frame rate.
    pub fn get_can_fps(&self) -> f64 {
        self.inner.fps.get_fps("CanMonitor")
    }

    // ------------------------------------------------------------------
    // Feedback getters
    // ------------------------------------------------------------------

    /// Arm status feedback (0x2A1).
    pub fn get_arm_status(&self) -> LatestArmStatus {
        let s = self.inner.state.lock().unwrap();
        LatestArmStatus {
            time_stamp: s.time_stamp_status,
            msg: s.arm_status,
        }
    }

    /// End-effector pose feedback (0x2A2/0x2A3/0x2A4), units 0.001 mm / 0.001 deg.
    pub fn get_arm_end_pose(&self) -> LatestEndPose {
        let s = self.inner.state.lock().unwrap();
        LatestEndPose {
            time_stamp: s.time_stamp_end_pose,
            msg: s.arm_end_pose,
        }
    }

    /// Joint angle feedback (0x2A5/0x2A6/0x2A7), units 0.001 deg.
    pub fn get_arm_joint_msgs(&self) -> LatestJoint {
        let s = self.inner.state.lock().unwrap();
        LatestJoint {
            time_stamp: s.time_stamp_joint,
            msg: s.arm_joint,
        }
    }

    /// Gripper feedback (0x2A8).
    pub fn get_arm_gripper_msgs(&self) -> LatestGripper {
        let s = self.inner.state.lock().unwrap();
        LatestGripper {
            time_stamp: s.time_stamp_gripper,
            msg: s.arm_gripper,
        }
    }

    /// High-speed motor driver feedback (0x251..0x256), one entry per joint 1..6.
    pub fn get_arm_high_spd_info_msgs(&self) -> [ArmMsgFeedbackHighSpd; 6] {
        self.inner.state.lock().unwrap().motor_high
    }

    /// Alias of [`PiperInterface::get_arm_high_spd_info_msgs`].
    pub fn get_motor_states(&self) -> [ArmMsgFeedbackHighSpd; 6] {
        self.get_arm_high_spd_info_msgs()
    }

    /// Low-speed motor driver feedback (0x261..0x266), one entry per joint 1..6.
    pub fn get_arm_low_spd_info_msgs(&self) -> [ArmMsgFeedbackLowSpd; 6] {
        self.inner.state.lock().unwrap().motor_low
    }

    /// Alias of [`PiperInterface::get_arm_low_spd_info_msgs`].
    pub fn get_driver_states(&self) -> [ArmMsgFeedbackLowSpd; 6] {
        self.get_arm_low_spd_info_msgs()
    }

    /// Motor enable status for joints 1..6.
    pub fn get_arm_enable_status(&self) -> [bool; 6] {
        let s = self.inner.state.lock().unwrap();
        let mut out = [false; 6];
        for (i, m) in s.motor_low.iter().enumerate() {
            out[i] = m.foc_status.driver_enable_status;
        }
        out
    }

    /// Feedback current motor angle limit / max speed (0x473).
    pub fn get_current_motor_angle_limit_max_vel(
        &self,
    ) -> ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd {
        self.inner.state.lock().unwrap().current_motor_angle_limit
    }

    /// Feedback current end velocity/acceleration parameters (0x478).
    pub fn get_current_end_vel_acc_param(&self) -> ArmMsgFeedbackCurrentEndVelAccParam {
        self.inner.state.lock().unwrap().current_end_vel_acc
    }

    /// Feedback collision protection levels (0x47B).
    pub fn get_crash_protection_level_feedback(&self) -> ArmMsgFeedbackCrashProtectionRating {
        self.inner.state.lock().unwrap().crash_protection
    }

    /// Feedback gripper/teaching pendant parameters (0x47E).
    pub fn get_gripper_teaching_pendant_param_feedback(
        &self,
    ) -> ArmMsgFeedbackGripperTeachingPendantParam {
        self.inner.state.lock().unwrap().gripper_teaching_param
    }

    /// Feedback current motor maximum acceleration limit (0x47C).
    pub fn get_current_motor_max_acc_limit(&self) -> ArmMsgFeedbackCurrentMotorMaxAccLimit {
        self.inner.state.lock().unwrap().current_motor_max_acc
    }

    /// All motors' max acceleration limits (request via
    /// [`PiperInterface::search_all_motor_max_acc_limit`]).
    pub fn get_all_motor_max_acc_limit(&self) -> [ArmMsgFeedbackCurrentMotorMaxAccLimit; 7] {
        self.inner.state.lock().unwrap().all_motor_max_acc
    }

    /// All motors' angle limit / max speed (request via
    /// [`PiperInterface::search_all_motor_max_angle_spd`]).
    pub fn get_all_motor_angle_limit_max_spd(
        &self,
    ) -> [ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd; 7] {
        self.inner.state.lock().unwrap().all_motor_angle_limit
    }

    /// The joint control command read back from the main arm (0x155..0x157).
    pub fn get_arm_joint_ctrl(&self) -> LatestJointCtrl {
        let s = self.inner.state.lock().unwrap();
        LatestJointCtrl {
            time_stamp: s.time_stamp_joint_ctrl,
            msg: s.joint_ctrl,
        }
    }

    /// The gripper control command read back from the main arm (0x159).
    pub fn get_arm_gripper_ctrl(&self) -> LatestGripperCtrl {
        let s = self.inner.state.lock().unwrap();
        LatestGripperCtrl {
            time_stamp: s.time_stamp_gripper_ctrl,
            msg: s.gripper_ctrl,
        }
    }

    /// The 0x151 control command read back from the main arm.
    pub fn get_arm_ctrl_code_151(&self) -> LatestMotionCtrl2 {
        let s = self.inner.state.lock().unwrap();
        LatestMotionCtrl2 {
            time_stamp: s.time_stamp_ctrl_151,
            msg: s.ctrl_151,
        }
    }

    /// Alias of [`PiperInterface::get_arm_ctrl_code_151`].
    pub fn get_arm_mode_ctrl(&self) -> LatestMotionCtrl2 {
        self.get_arm_ctrl_code_151()
    }

    /// Instruction response feedback (0x476).
    pub fn get_resp_instruction(&self) -> ArmMsgFeedbackRespSetInstruction {
        self.inner.state.lock().unwrap().resp_set_instruction
    }

    /// Firmware version string, e.g. "S-V1.7-2". Returns `Err` when not yet
    /// available. Returns `Err` with `0x4AF` until the arm has replied.
    pub fn get_piper_firmware_version(&self) -> Result<String> {
        let fw = self.inner.firmware.lock().unwrap();
        let bytes = &fw[..];
        for i in 0..bytes.len().saturating_sub(1) {
            if bytes[i] == b'S' && bytes.get(i + 1) == Some(&b'-') && bytes.get(i + 2) == Some(&b'V')
            {
                let end = (i + 8).min(bytes.len());
                let s = String::from_utf8_lossy(&bytes[i..end]).into_owned();
                return Ok(s);
            }
        }
        Err(Error::Can("firmware version not found (0x4AF)".into()))
    }

    /// FK result of the feedback joint states (requires FK calc enabled).
    pub fn get_fk_feedback(&self) -> [[f64; 6]; 6] {
        *self.inner.feedback_fk.lock().unwrap()
    }

    /// FK result of the control joint commands (requires FK calc enabled).
    pub fn get_fk_control(&self) -> [[f64; 6]; 6] {
        *self.inner.ctrl_fk.lock().unwrap()
    }

    // ------------------------------------------------------------------
    // SDK config toggles
    // ------------------------------------------------------------------

    pub fn enable_fk_cal(&self) -> bool {
        self.inner.start_sdk_fk_cal.store(true, Ordering::SeqCst);
        true
    }

    pub fn disable_fk_cal(&self) -> bool {
        self.inner.start_sdk_fk_cal.store(false, Ordering::SeqCst);
        false
    }

    pub fn is_cal_fk(&self) -> bool {
        self.inner.start_sdk_fk_cal.load(Ordering::SeqCst)
    }

    pub fn enable_filter_abnormal_data(&self) -> bool {
        self.inner.filter_abnormal_data.store(true, Ordering::SeqCst);
        true
    }

    pub fn disable_filter_abnormal_data(&self) -> bool {
        self.inner.filter_abnormal_data.store(false, Ordering::SeqCst);
        false
    }

    pub fn is_filter_abnormal_data(&self) -> bool {
        self.inner.filter_abnormal_data.load(Ordering::SeqCst)
    }

    // ------------------------------------------------------------------
    // Control sends
    // ------------------------------------------------------------------

    fn send(&self, frame: &ControlFrame) -> Result<()> {
        self.can.send_frame(frame.id, &frame.data)
    }

    /// 机械臂运动控制指令1 (0x150).
    pub fn motion_ctrl_1(
        &self,
        emergency_stop: u8,
        track_ctrl: u8,
        grag_teach_ctrl: u8,
    ) -> Result<()> {
        let m = ArmMsgMotionCtrl1 {
            emergency_stop,
            track_ctrl,
            grag_teach_ctrl,
        };
        let frame = crate::protocol::encode_motion_ctrl_1(&m)?;
        self.send(&frame)
    }

    /// 快速急停 / 恢复 (0x150).
    pub fn emergency_stop(&self, emergency_stop: u8) -> Result<()> {
        self.motion_ctrl_1(emergency_stop, 0x00, 0x00)
    }

    /// 机械臂重置 (0x150), 会立刻失电落下并清除所有错误。
    pub fn reset_piper(&self) -> Result<()> {
        self.motion_ctrl_1(0x02, 0x00, 0x00)
    }

    /// 机械臂运动控制指令2 (0x151).
    #[allow(clippy::too_many_arguments)]
    pub fn motion_ctrl_2(
        &self,
        ctrl_mode: u8,
        move_mode: u8,
        move_spd_rate_ctrl: u8,
        mit_mode: u8,
        residence_time: u8,
        installation_pos: u8,
    ) -> Result<()> {
        let m = ArmMsgMotionCtrl2 {
            ctrl_mode,
            move_mode,
            move_spd_rate_ctrl,
            mit_mode,
            residence_time,
            installation_pos,
        };
        let frame = crate::protocol::encode_motion_ctrl_2(&m)?;
        self.send(&frame)
    }

    /// 模式控制 (0x151).
    pub fn mode_ctrl(
        &self,
        ctrl_mode: u8,
        move_mode: u8,
        move_spd_rate_ctrl: u8,
        mit_mode: u8,
    ) -> Result<()> {
        self.motion_ctrl_2(ctrl_mode, move_mode, move_spd_rate_ctrl, mit_mode, 0, 0)
    }

    /// 机械臂末端数值发送 (0x152/0x153/0x154), 单位 0.001 mm / 0.001 度。
    pub fn end_pose_ctrl(&self, x: i32, y: i32, z: i32, rx: i32, ry: i32, rz: i32) -> Result<()> {
        self.send(&crate::protocol::encode_cartesian_xy(x, y)?)?;
        self.send(&crate::protocol::encode_cartesian_zrx(z, rx)?)?;
        self.send(&crate::protocol::encode_cartesian_ryrz(ry, rz)?)?;
        Ok(())
    }

    /// 机械臂关节控制 (0x155/0x156/0x157), 单位 0.001 度。
    #[allow(clippy::too_many_arguments)]
    pub fn joint_ctrl(
        &self,
        joint_1: i32,
        joint_2: i32,
        joint_3: i32,
        joint_4: i32,
        joint_5: i32,
        joint_6: i32,
    ) -> Result<()> {
        let clamped =
            self.apply_sdk_joint_limit([joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]);
        self.send(&crate::protocol::encode_joint_ctrl_12(clamped[0], clamped[1])?)?;
        self.send(&crate::protocol::encode_joint_ctrl_34(clamped[2], clamped[3])?)?;
        self.send(&crate::protocol::encode_joint_ctrl_56(clamped[4], clamped[5])?)?;
        Ok(())
    }

    /// MoveC 模式坐标点更新指令 (0x158).
    pub fn move_c_axis_update_ctrl(&self, instruction_num: u8) -> Result<()> {
        let frame = crate::protocol::encode_circular_pattern(instruction_num)?;
        self.send(&frame)
    }

    /// 夹爪控制 (0x159).
    pub fn gripper_ctrl(
        &self,
        gripper_angle: i32,
        gripper_effort: u16,
        status_code: u8,
        set_zero: u8,
    ) -> Result<()> {
        let angle = self.apply_sdk_gripper_limit(gripper_angle);
        let m = ArmMsgGripperCtrl {
            grippers_angle: angle,
            grippers_effort: gripper_effort,
            status_code,
            set_zero,
        };
        let frame = crate::protocol::encode_gripper_ctrl(&m)?;
        self.send(&frame)
    }

    /// 随动主从模式设置指令 (0x470).
    pub fn master_slave_config(
        &self,
        linkage_config: u8,
        feedback_offset: u8,
        ctrl_offset: u8,
        linkage_offset: u8,
    ) -> Result<()> {
        let m = ArmMsgMasterSlaveModeConfig {
            linkage_config,
            feedback_offset,
            ctrl_offset,
            linkage_offset,
        };
        let frame = crate::protocol::encode_master_slave_config(&m)?;
        self.send(&frame)
    }

    /// 失能电机 (0x471).
    pub fn disable_arm(&self, motor_num: u8, enable_flag: u8) -> Result<()> {
        let m = ArmMsgMotorEnableDisableConfig {
            motor_num,
            enable_flag,
        };
        let frame = crate::protocol::encode_motor_enable_disable(&m)?;
        self.send(&frame)
    }

    /// 使能电机 (0x471).
    pub fn enable_arm(&self, motor_num: u8, enable_flag: u8) -> Result<()> {
        let m = ArmMsgMotorEnableDisableConfig {
            motor_num,
            enable_flag,
        };
        let frame = crate::protocol::encode_motor_enable_disable(&m)?;
        self.send(&frame)
    }

    /// 使能机械臂 (全部电机).
    pub fn enable_piper(&self) -> Result<bool> {
        let enable_list = self.get_arm_enable_status();
        self.enable_arm(7, 0x02)?;
        Ok(enable_list.iter().all(|&e| e))
    }

    /// 失能机械臂 (全部电机).
    pub fn disable_piper(&self) -> Result<bool> {
        let enable_list = self.get_arm_enable_status();
        self.disable_arm(7, 0x01)?;
        Ok(enable_list.iter().any(|&e| e))
    }

    /// 查询电机角度/最大速度/最大加速度限制 (0x472).
    pub fn search_motor_max_angle_spd_acc_limit(
        &self,
        motor_num: u8,
        search_content: u8,
    ) -> Result<()> {
        let m = ArmMsgSearchMotorMaxAngleSpdAccLimit {
            motor_num,
            search_content,
        };
        let frame = crate::protocol::encode_search_motor(&m)?;
        self.send(&frame)
    }

    /// 查询全部电机最大角度/最小角度/最大速度 (0x472).
    pub fn search_all_motor_max_angle_spd(&self) -> Result<()> {
        for n in 1..=6u8 {
            self.search_motor_max_angle_spd_acc_limit(n, 0x01)?;
        }
        Ok(())
    }

    /// 查询全部电机最大加速度限制 (0x472).
    pub fn search_all_motor_max_acc_limit(&self) -> Result<()> {
        for n in 1..=6u8 {
            self.search_motor_max_angle_spd_acc_limit(n, 0x02)?;
        }
        Ok(())
    }

    /// 电机角度限制/最大速度设置指令 (0x474).
    pub fn motor_angle_limit_max_spd_set(
        &self,
        motor_num: u8,
        max_angle_limit: i16,
        min_angle_limit: i16,
        max_joint_spd: u16,
    ) -> Result<()> {
        let m = ArmMsgMotorAngleLimitMaxSpdSet {
            motor_num,
            max_angle_limit,
            min_angle_limit,
            max_joint_spd,
        };
        let frame = crate::protocol::encode_motor_angle_limit_set(&m)?;
        self.send(&frame)
    }

    /// 电机最大速度设置指令 (0x474).
    pub fn motor_max_spd_set(&self, motor_num: u8, max_joint_spd: u16) -> Result<()> {
        self.motor_angle_limit_max_spd_set(motor_num, 0x7FFF, 0x7FFF, max_joint_spd)
    }

    /// 关节设置指令 (0x475).
    pub fn joint_config(
        &self,
        joint_num: u8,
        set_zero: u8,
        acc_param_effective: u8,
        max_joint_acc: u16,
        clear_err: u8,
    ) -> Result<()> {
        let m = ArmMsgJointConfig {
            joint_motor_num: joint_num,
            set_motor_current_pos_as_zero: set_zero,
            acc_param_config_is_effective_or_not: acc_param_effective,
            max_joint_acc,
            clear_joint_err: clear_err,
        };
        let frame = crate::protocol::encode_joint_config(&m)?;
        self.send(&frame)
    }

    /// 关节最大加速度设置指令 (0x475).
    pub fn joint_max_acc_config(&self, motor_num: u8, max_joint_acc: u16) -> Result<()> {
        self.joint_config(motor_num, 0, 0xAE, max_joint_acc, 0)
    }

    /// 机械臂参数查询与设置指令 (0x477).
    #[allow(clippy::too_many_arguments)]
    pub fn arm_param_enquiry_and_config(
        &self,
        param_enquiry: u8,
        param_setting: u8,
        data_feedback_0x48x: u8,
        end_load_param_setting_effective: u8,
        set_end_load: u8,
    ) -> Result<()> {
        let m = ArmMsgParamEnquiryAndConfig {
            param_enquiry,
            param_setting,
            data_feedback_0x48x,
            end_load_param_setting_effective,
            set_end_load,
        };
        let frame = crate::protocol::encode_param_enquiry(&m)?;
        self.send(&frame)
    }

    /// 末端速度/加速度参数设置指令 (0x479).
    pub fn end_spd_and_acc_param_set(
        &self,
        lin_vel: u16,
        ang_vel: u16,
        lin_acc: u16,
        ang_acc: u16,
    ) -> Result<()> {
        let m = ArmMsgEndVelAccParamConfig {
            end_max_linear_vel: lin_vel,
            end_max_angular_vel: ang_vel,
            end_max_linear_acc: lin_acc,
            end_max_angular_acc: ang_acc,
        };
        let frame = crate::protocol::encode_end_vel_acc_param(&m)?;
        self.send(&frame)
    }

    /// 碰撞防护等级设置指令 (0x47A).
    pub fn crash_protection_config(&self, levels: [u8; 6]) -> Result<()> {
        let m = ArmMsgCrashProtectionRatingConfig {
            joint_1_protection_level: levels[0],
            joint_2_protection_level: levels[1],
            joint_3_protection_level: levels[2],
            joint_4_protection_level: levels[3],
            joint_5_protection_level: levels[4],
            joint_6_protection_level: levels[5],
        };
        let frame = crate::protocol::encode_crash_protection(&m)?;
        self.send(&frame)
    }

    /// 夹爪/示教器参数设置指令 (0x47D).
    pub fn gripper_teaching_pendant_param_config(
        &self,
        teaching_range_per: u8,
        max_range_config: u8,
        teaching_friction: u8,
    ) -> Result<()> {
        let m = ArmMsgGripperTeachingPendantParamConfig {
            teaching_range_per,
            max_range_config,
            teaching_friction,
        };
        let frame = crate::protocol::encode_gripper_teaching_param(&m)?;
        self.send(&frame)
    }

    /// 发送 piper 机械臂固件版本查询指令 (0x4AF)。
    pub fn search_piper_firmware_version(&self) -> Result<()> {
        let frame = crate::protocol::encode_search_firmware()?;
        self.inner.firmware.lock().unwrap().clear();
        self.send(&frame)
    }

    /// 关节 MIT 控制指令 (0x15A~0x15F)。
    ///
    /// `pos_ref`, `vel_ref`, `kp`, `kd`, `t_ref` are physical values in the
    /// ranges documented by the protocol. This method quantizes them.
    #[allow(clippy::too_many_arguments)]
    pub fn joint_mit_ctrl(
        &self,
        motor_num: u8,
        pos_ref: f64,
        vel_ref: f64,
        kp: f64,
        kd: f64,
        t_ref: f64,
    ) -> Result<()> {
        let pos = crate::protocol::base::float_to_uint(pos_ref, -12.5, 12.5, 16);
        let vel = crate::protocol::base::float_to_uint(vel_ref, -45.0, 45.0, 12);
        let kp = crate::protocol::base::float_to_uint(kp, 0.0, 500.0, 12);
        let kd = crate::protocol::base::float_to_uint(kd, -5.0, 5.0, 12);
        let t = crate::protocol::base::float_to_uint(t_ref, -8.0, 8.0, 8);
        let m = ArmMsgJointMitCtrl {
            pos_ref: pos as u16,
            vel_ref: vel as u16,
            kp: kp as u16,
            kd: kd as u16,
            t_ref: t as u8,
            crc: 0,
        };
        let frame = crate::protocol::encode_joint_mit_ctrl(motor_num, &m)?;
        self.send(&frame)
    }

    /// 请求主臂回零指令 (0x191).
    pub fn req_master_arm_move_to_home(&self, mode: u8) -> Result<()> {
        let frame = crate::protocol::encode_req_master_arm_move_to_home(mode)?;
        self.send(&frame)
    }

    /// 设置指令应答 (0x476). Deprecated; kept for API parity and does nothing.
    pub fn set_instruction_response(&self, _instruction_index: u8, _zero_config_success_flag: u8) -> Result<()> {
        log::warn!("set_instruction_response is deprecated (since 0.5.0) and does nothing");
        Ok(())
    }

    /// 清除 SDK 保存的设置指令应答信息。
    pub fn clear_resp_set_instruction(&self) {
        let mut s = self.inner.state.lock().unwrap();
        s.resp_set_instruction = ArmMsgFeedbackRespSetInstruction {
            instruction_index: -1,
            is_set_zero_successfully: -1,
        };
    }

    /// PiperInit: 查询关节电机最大角度速度 / 最大加速度 / 固件版本。
    pub fn piper_init(&self) -> Result<()> {
        self.search_all_motor_max_angle_spd()?;
        self.search_all_motor_max_acc_limit()?;
        self.search_piper_firmware_version()?;
        Ok(())
    }

    // ------------------------------------------------------------------
    // SDK param helpers
    // ------------------------------------------------------------------

    pub fn get_sdk_joint_limit(&self, joint_name: &str) -> Result<(f64, f64)> {
        self.inner.param.lock().unwrap().get_joint_limit(joint_name)
    }

    pub fn get_sdk_gripper_range(&self) -> (f64, f64) {
        self.inner.param.lock().unwrap().get_gripper_range()
    }

    pub fn set_sdk_joint_limit(&self, joint_name: &str, min_val: f64, max_val: f64) -> Result<()> {
        self.inner
            .param
            .lock()
            .unwrap()
            .set_joint_limit(joint_name, min_val, max_val)
    }

    pub fn set_sdk_gripper_range(&self, min_val: f64, max_val: f64) -> Result<()> {
        self.inner
            .param
            .lock()
            .unwrap()
            .set_gripper_range(min_val, max_val)
    }

    /// Enable/disable the SDK joint angle limit clamping (feedback + control).
    pub fn set_sdk_joint_limit_enabled(&self, enabled: bool) {
        self.inner.start_sdk_joint_limit.store(enabled, Ordering::SeqCst);
    }

    /// Enable/disable the SDK gripper position limit clamping.
    pub fn set_sdk_gripper_limit_enabled(&self, enabled: bool) {
        self.inner
            .start_sdk_gripper_limit
            .store(enabled, Ordering::SeqCst);
    }

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------

    fn apply_sdk_joint_limit(&self, joints: [i32; 6]) -> [i32; 6] {
        if !self.inner.start_sdk_joint_limit.load(Ordering::SeqCst) {
            return joints;
        }
        let p = self.inner.param.lock().unwrap();
        let names = ["j1", "j2", "j3", "j4", "j5", "j6"];
        let mut out = joints;
        for (i, name) in names.iter().enumerate() {
            if let Ok((min, max)) = p.get_joint_limit(name) {
                let min = (min.to_degrees() * 1000.0).round() as i32;
                let max = (max.to_degrees() * 1000.0).round() as i32;
                out[i] = out[i].clamp(min, max);
            }
        }
        out
    }

    fn apply_sdk_gripper_limit(&self, value: i32) -> i32 {
        if !self.inner.start_sdk_gripper_limit.load(Ordering::SeqCst) {
            return value;
        }
        let (min, max) = self.inner.param.lock().unwrap().get_gripper_range();
        let min = (min * 1_000_000.0).round() as i32;
        let max = (max * 1_000_000.0).round() as i32;
        value.clamp(min, max)
    }
}

impl Drop for PiperInterface {
    fn drop(&mut self) {
        self.disconnect();
    }
}

fn is_timeout(e: &std::io::Error) -> bool {
    matches!(
        e.kind(),
        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
    )
}

fn handle_message(msg: &DecodedMessage, inner: &Inner, fk: &PiperForwardKinematics) {
    let mut s = inner.state.lock().unwrap();
    let now = now_secs();

    // Apply SDK/clamping helpers before storing.
    let joint_12 = match msg {
        DecodedMessage::JointFeedback12(a, b) => {
            let (a, b) = clamp_joints(
                [*a, *b],
                ["j1", "j2"],
                inner.start_sdk_joint_limit.load(Ordering::SeqCst),
                &inner.param,
            );
            if inner.filter_abnormal_data.load(Ordering::SeqCst)
                && (a.abs() > 3_000_000 || b.abs() > 3_000_000)
            {
                return;
            }
            inner.fps.increment("ArmJoint_12");
            Some((a, b))
        }
        _ => None,
    };
    let joint_34 = match msg {
        DecodedMessage::JointFeedback34(a, b) => {
            let (a, b) = clamp_joints(
                [*a, *b],
                ["j3", "j4"],
                inner.start_sdk_joint_limit.load(Ordering::SeqCst),
                &inner.param,
            );
            if inner.filter_abnormal_data.load(Ordering::SeqCst)
                && (a.abs() > 3_000_000 || b.abs() > 3_000_000)
            {
                return;
            }
            inner.fps.increment("ArmJoint_34");
            Some((a, b))
        }
        _ => None,
    };
    let joint_56 = match msg {
        DecodedMessage::JointFeedback56(a, b) => {
            let (a, b) = clamp_joints(
                [*a, *b],
                ["j5", "j6"],
                inner.start_sdk_joint_limit.load(Ordering::SeqCst),
                &inner.param,
            );
            if inner.filter_abnormal_data.load(Ordering::SeqCst)
                && (a.abs() > 3_000_000 || b.abs() > 3_000_000)
            {
                return;
            }
            inner.fps.increment("ArmJoint_56");
            Some((a, b))
        }
        _ => None,
    };

    match msg {
        DecodedMessage::StatusFeedback(m) => {
            inner.fps.increment("ArmStatus");
            s.time_stamp_status = now;
            s.arm_status = *m;
        }
        DecodedMessage::EndPoseXY(x, y) => {
            if inner.filter_abnormal_data.load(Ordering::SeqCst)
                && (x.abs() > 1_000_000 || y.abs() > 1_000_000)
            {
                return;
            }
            inner.fps.increment("ArmEndPose_XY");
            s.time_stamp_end_pose = now;
            s.arm_end_pose.x_axis = *x;
            s.arm_end_pose.y_axis = *y;
        }
        DecodedMessage::EndPoseZRX(z, rx) => {
            if inner.filter_abnormal_data.load(Ordering::SeqCst)
                && (z.abs() > 1_000_000 || rx.abs() > 361_000)
            {
                return;
            }
            inner.fps.increment("ArmEndPose_ZRX");
            s.time_stamp_end_pose = now;
            s.arm_end_pose.z_axis = *z;
            s.arm_end_pose.rx_axis = *rx;
        }
        DecodedMessage::EndPoseRYRZ(ry, rz) => {
            if inner.filter_abnormal_data.load(Ordering::SeqCst)
                && (ry.abs() > 361_000 || rz.abs() > 361_000)
            {
                return;
            }
            inner.fps.increment("ArmEndPose_RYRZ");
            s.time_stamp_end_pose = now;
            s.arm_end_pose.ry_axis = *ry;
            s.arm_end_pose.rz_axis = *rz;
        }
        DecodedMessage::JointFeedback12(_, _) => {
            if let Some((a, b)) = joint_12 {
                s.time_stamp_joint = now;
                s.arm_joint.joint_1 = a;
                s.arm_joint.joint_2 = b;
            }
        }
        DecodedMessage::JointFeedback34(_, _) => {
            if let Some((a, b)) = joint_34 {
                s.time_stamp_joint = now;
                s.arm_joint.joint_3 = a;
                s.arm_joint.joint_4 = b;
            }
        }
        DecodedMessage::JointFeedback56(_, _) => {
            if let Some((a, b)) = joint_56 {
                s.time_stamp_joint = now;
                s.arm_joint.joint_5 = a;
                s.arm_joint.joint_6 = b;
            }
        }
        DecodedMessage::GripperFeedback(m) => {
            let angle = clamp_gripper(
                m.grippers_angle,
                inner.start_sdk_gripper_limit.load(Ordering::SeqCst),
                &inner.param,
            );
            if inner.filter_abnormal_data.load(Ordering::SeqCst) && angle.abs() > 150_000 {
                return;
            }
            inner.fps.increment("ArmGripper");
            s.time_stamp_gripper = now;
            s.arm_gripper = *m;
            s.arm_gripper.grippers_angle = angle;
        }
        DecodedMessage::HighSpdFeedback(n, m) => {
            inner.fps.increment(&format!("ArmMotorDriverInfoHighSpd_{n}"));
            let mut m2 = *m;
            m2.cal_effort();
            s.motor_high[*n - 1] = m2;
        }
        DecodedMessage::LowSpdFeedback(n, m) => {
            inner.fps.increment(&format!("ArmMotorDriverInfoLowSpd_{n}"));
            s.motor_low[*n - 1] = *m;
        }
        DecodedMessage::FeedbackRespSetInstruction(m) => {
            s.resp_set_instruction = *m;
        }
        DecodedMessage::FeedbackCurrentMotorAngleLimitMaxSpd(m) => {
            s.current_motor_angle_limit = *m;
            let i = m.motor_num as usize;
            if (1..=6).contains(&i) {
                s.all_motor_angle_limit[i] = *m;
            }
        }
        DecodedMessage::FeedbackCurrentEndVelAccParam(m) => {
            s.current_end_vel_acc = *m;
        }
        DecodedMessage::CrashProtectionRatingFeedback(m) => {
            s.crash_protection = *m;
        }
        DecodedMessage::FeedbackCurrentMotorMaxAccLimit(m) => {
            s.current_motor_max_acc = *m;
            let i = m.joint_motor_num as usize;
            if (1..=6).contains(&i) {
                s.all_motor_max_acc[i] = *m;
            }
        }
        DecodedMessage::GripperTeachingPendantParamFeedback(m) => {
            s.gripper_teaching_param = *m;
        }
        DecodedMessage::FirmwareRead(data) => {
            inner.firmware.lock().unwrap().extend_from_slice(data);
        }
        DecodedMessage::JointCtrl(m) => {
            inner.fps.increment("ArmJointCtrl_12");
            s.time_stamp_joint_ctrl = now;
            s.joint_ctrl = *m;
        }
        DecodedMessage::GripperCtrl(m) => {
            inner.fps.increment("ArmGripperCtrl");
            s.time_stamp_gripper_ctrl = now;
            s.gripper_ctrl = *m;
        }
        DecodedMessage::MotionCtrl2(m) => {
            inner.fps.increment("ArmCtrlCode_151");
            s.time_stamp_ctrl_151 = now;
            s.ctrl_151 = *m;
        }
        DecodedMessage::JointVelAcc(n, m) => {
            s.joint_vel_acc[*n] = *m;
        }
    }

    if inner.start_sdk_fk_cal.load(Ordering::SeqCst) {
        let joints = [
            s.arm_joint.joint_1 as f64 / (1000.0 * fk.radian),
            s.arm_joint.joint_2 as f64 / (1000.0 * fk.radian),
            s.arm_joint.joint_3 as f64 / (1000.0 * fk.radian),
            s.arm_joint.joint_4 as f64 / (1000.0 * fk.radian),
            s.arm_joint.joint_5 as f64 / (1000.0 * fk.radian),
            s.arm_joint.joint_6 as f64 / (1000.0 * fk.radian),
        ];
        *inner.feedback_fk.lock().unwrap() = fk.cal_fk(&joints);
        let joints = [
            s.joint_ctrl.joint_1 as f64 / (1000.0 * fk.radian),
            s.joint_ctrl.joint_2 as f64 / (1000.0 * fk.radian),
            s.joint_ctrl.joint_3 as f64 / (1000.0 * fk.radian),
            s.joint_ctrl.joint_4 as f64 / (1000.0 * fk.radian),
            s.joint_ctrl.joint_5 as f64 / (1000.0 * fk.radian),
            s.joint_ctrl.joint_6 as f64 / (1000.0 * fk.radian),
        ];
        *inner.ctrl_fk.lock().unwrap() = fk.cal_fk(&joints);
    }
}

fn clamp_joints(
    joints: [i32; 2],
    names: [&str; 2],
    enabled: bool,
    param: &Mutex<PiperParamManager>,
) -> (i32, i32) {
    if !enabled {
        return (joints[0], joints[1]);
    }
    let p = param.lock().unwrap();
    let mut out = joints;
    for (i, name) in names.iter().enumerate() {
        if let Ok((min, max)) = p.get_joint_limit(name) {
            let min = (min.to_degrees() * 1000.0).round() as i32;
            let max = (max.to_degrees() * 1000.0).round() as i32;
            out[i] = out[i].clamp(min, max);
        }
    }
    (out[0], out[1])
}

fn clamp_gripper(value: i32, enabled: bool, param: &Mutex<PiperParamManager>) -> i32 {
    if !enabled {
        return value;
    }
    let (min, max) = param.lock().unwrap().get_gripper_range();
    let min = (min * 1_000_000.0).round() as i32;
    let max = (max * 1_000_000.0).round() as i32;
    value.clamp(min, max)
}

fn now_secs() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// Arm status feedback with a timestamp.
#[derive(Debug, Clone)]
pub struct LatestArmStatus {
    pub time_stamp: f64,
    pub msg: ArmMsgFeedbackStatus,
}

/// End-effector pose feedback with a timestamp.
#[derive(Debug, Clone)]
pub struct LatestEndPose {
    pub time_stamp: f64,
    pub msg: ArmMsgFeedbackEndPose,
}

/// Joint feedback with a timestamp.
#[derive(Debug, Clone)]
pub struct LatestJoint {
    pub time_stamp: f64,
    pub msg: ArmMsgFeedbackJointStates,
}

/// Gripper feedback with a timestamp.
#[derive(Debug, Clone)]
pub struct LatestGripper {
    pub time_stamp: f64,
    pub msg: ArmMsgFeedbackGripper,
}

/// Joint control read-back with a timestamp.
#[derive(Debug, Clone)]
pub struct LatestJointCtrl {
    pub time_stamp: f64,
    pub msg: ArmMsgJointCtrl,
}

/// Gripper control read-back with a timestamp.
#[derive(Debug, Clone)]
pub struct LatestGripperCtrl {
    pub time_stamp: f64,
    pub msg: ArmMsgGripperCtrl,
}

/// 0x151 control read-back with a timestamp.
#[derive(Debug, Clone)]
pub struct LatestMotionCtrl2 {
    pub time_stamp: f64,
    pub msg: ArmMsgMotionCtrl2,
}
