#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/hyundai_common.h"

#define HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(bus) \
  {0x1CF, bus, 8, .check_relay = false},  /* CRUISE_BUTTON */   \

#define HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  {0x50,  a_can, 16, .check_relay = (a_can) == 0},  /* LKAS */      \
  {0x2A4, a_can, 24, .check_relay = (a_can) == 0},  /* CAM_0x2A4 */ \

#define HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(a_can, e_can) \
  HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(e_can)                        \
  {0x110, a_can, 32, .check_relay = (a_can) == 0},  /* LKAS_ALT */  \
  {0x362, a_can, 32, .check_relay = (a_can) == 0},  /* CAM_0x362 */ \

#define HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(e_can)  \
  {0x12A, e_can, 16, .check_relay = (e_can) == 0},  /* LFA */            \
  {0x1E0, e_can, 16, .check_relay = (e_can) == 0},  /* LFAHDA_CLUSTER */ \
  {0xCB,  e_can, 24, .check_relay = (e_can) == 0},  /* LFA_ALT */ \

// HDA1 angle steering that actuates only through LFA_ALT (0xCB): openpilot does not send 0x12A,
// so the camera's 0x12A (lane/FCA/ESA status and warnings, no actuation) is forwarded to the car.
#define HYUNDAI_CANFD_LFA_ALT_STEERING_COMMON_TX_MSGS(e_can)  \
  {0x1E0, e_can, 16, .check_relay = (e_can) == 0},  /* LFAHDA_CLUSTER */ \
  {0xCB,  e_can, 24, .check_relay = (e_can) == 0},  /* LFA_ALT */ \

#define HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(e_can, longitudinal) \
  {0x1A0, e_can, 32, .check_relay = (longitudinal)},  /* SCC_CONTROL */ \

// *** Addresses checked in rx hook ***
// EV, ICE, HYBRID: ACCELERATOR (0x35), ACCELERATOR_BRAKE_ALT (0x100), ACCELERATOR_ALT (0x105)
#define HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                          \
  {.msg = {{0x35, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                  \
           {0x100, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true},                 \
           {0x105, (pt_bus), 32, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}}},               \
  {.msg = {{0x175, (pt_bus), 24, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xa0, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{0xea, (pt_bus), 24, 100U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(pt_bus)                                                                                            \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                                       \
  {.msg = {{0x1cf, (pt_bus), 8, 50U, .ignore_checksum = true, .max_counter = 0xfU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(pt_bus)                                                                                              \
  HYUNDAI_CANFD_COMMON_RX_CHECKS(pt_bus)                                                                                                         \
  {.msg = {{0x1aa, (pt_bus), 16, 50U, .ignore_checksum = true, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

// SCC_CONTROL (from ADAS unit or camera)
#define HYUNDAI_CANFD_SCC_ADDR_CHECK(scc_bus)                                                                            \
  {.msg = {{0x1a0, (scc_bus), 32, 50U, .max_counter = 0xffU, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

// CAMERA_HANDBACK lane-keeping window: the camera's own LFA_ICON (CCNC_0x161) and LKA_SysIndReq (LFA 0x12A) on bus 2, read in the rx hook.
// Optional: only read in the rx hook. A missing or late frame never disables controls (see safety_tick); it just closes the window (the camera stays blocked).
#define HYUNDAI_CANFD_HB_LKA_RX_CHECKS                                                                                                \
  {.msg = {{0x161, 2, 32, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .optional = true}, { 0 }, { 0 }}},  \
  {.msg = {{0x12a, 2, 16, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .optional = true}, { 0 }, { 0 }}}, \

static bool hyundai_canfd_alt_buttons = false;
static bool hyundai_canfd_angle_steering = false;
static bool hyundai_canfd_lka_steer_msg_alt = false;
static bool hyundai_ccnc = false;

static bool get_hyundai_ccnc(void) {
  return hyundai_ccnc;
}

static unsigned int hyundai_canfd_get_lka_addr(void) {
  return hyundai_canfd_lka_steer_msg_alt ? 0x110U : 0x50U;
}

static uint8_t hyundai_canfd_get_counter(const CANPacket_t *msg) {
  uint8_t ret = 0;
  if (GET_LEN(msg) == 8U) {
    ret = msg->data[1] >> 4;
  } else {
    ret = msg->data[2];
  }
  return ret;
}

static uint32_t hyundai_canfd_get_checksum(const CANPacket_t *msg) {
  uint32_t chksum = msg->data[0] | (msg->data[1] << 8);
  return chksum;
}

// CAMERA_HANDBACK (GV70 CCNC LFA_ALT camera SCC): openpilot owns lateral from the panda init (ignition) on: the camera's 0xCB (LFA_ALT) and
// 0xEA (MDPS to the camera) are blocked and openpilot's frames (also inactive ones) replace them, except inside the lane-keeping window below.
// The car's own LFA lane centering never gets through, also not after a Driving Assist (main) press that turns it on, which the driver did
// not ask for; the only way to steer is the LFA button (MADS). With openpilot longitudinal, 0x1A0 (SCC_CONTROL) belongs to the camera until
// openpilot takes that axis.
//   LKA NUDGE (lateral, openpilot owns the axis but neither controls_allowed nor controls_allowed_lateral): the camera's 0xCB (and the real MDPS
//                       0xEA for the camera) pass only inside the "lane-keeping window", else openpilot's inactive frames replace them. The car's
//                       lane-departure prevention steers through the same 0xCB with the camera's own LFA hidden. The forward hook sees no data,
//                       so the window is built from the camera's earlier frames (research/stock-lfa-ownership.md, routes 2d/2f/30):
//                         camera 0x161 LFA_ICON has been 0 for >= HYUNDAI_CANFD_HB_LKA_ICON_SETTLE_US (the LFA on/off blink lasts <= 1.3 s, a
//                         visible or blinking icon closes the window at once), AND
//                         camera 0x12A LKA_SysIndReq == 3 (lane departure) within HYUNDAI_CANFD_HB_LKA_HOLD_US.
//                       LFA lane centering never matches: its icon is 1/2 (or the 3,0,3,0 blink), and the frames that precede the icon on an
//                       LFA start have 0x12A LKA_SysIndReq 1/2. While controls_allowed or controls_allowed_lateral (on or paused) the camera is
//                       never forwarded. Openpilot's own 0xCB / 0xEA are rejected while the window is open (never two senders).
//   stock -> openpilot, longitudinal: the first openpilot frame that passes the TX checks while controls are allowed (0x1A0: ACCMode 1/2).
//   openpilot -> stock, longitudinal: the panda no longer allows it AND the driver acted (brake press, cancel, main button) within
//                       HYUNDAI_CANFD_HB_ACTION_WINDOW_US of that moment (before or after). Losing the permission any other way keeps the
//                       camera blocked until the next driver action.
// STOCK ACC LOCK: after a hand-back the camera sits in standby (main on, ACCMode 4) and its cruise key is a pause/resume toggle, so a CANCEL
// press makes the camera resume its own ACC at its stale set speed (route 2d 9:48.8, 11:01.8: 25 -> 49 mph). The panda cannot see the
// button in the forward hook (fwd runs before rx and only gets bus and address), so it acts on the camera's own state instead: a rising
// camera ACCMode 1/2 while controls are not allowed and openpilot does not own 0x1A0 is an engage the driver did not make for openpilot.
// From the next frame the camera's 0x1A0 is blocked and openpilot's 'off' frames (ACCMode not 1/2) replace it, until the camera is back
// in standby for HYUNDAI_CANFD_HB_STOCK_STANDBY_FRAMES frames or openpilot takes the axis. At most one camera frame (aReq about 0) leaks.
// While stock owns an axis, openpilot's frames for it are rejected (except the lock's 'off' frames), so the two never interleave.
static bool hyundai_canfd_handback = false;
static bool hyundai_canfd_hb_long_op = false;
static bool hyundai_canfd_hb_long_action = false;
static uint32_t hyundai_canfd_hb_long_action_ts = 0U;
static bool hyundai_canfd_hb_brake_prev = false;
static bool hyundai_canfd_hb_main_on_prev = false;
static bool hyundai_canfd_hb_cancel_prev = false;
static bool hyundai_canfd_hb_main_button_prev = false;
static bool hyundai_canfd_hb_cam_seen = false;      // a camera 0x1A0 was received since init (the first one is not an engage edge)
static bool hyundai_canfd_hb_cam_engaged = false;   // camera ACCMode 1 or 2
static bool hyundai_canfd_hb_stock_lock = false;
static uint8_t hyundai_canfd_hb_cam_standby_frames = 0U;
static const uint32_t HYUNDAI_CANFD_HB_ACTION_WINDOW_US = 10000000U;  // heartbeat mismatch exits MADS/controls ~3 s after openpilot disengages
static const uint8_t HYUNDAI_CANFD_HB_STOCK_STANDBY_FRAMES = 3U;
static const uint32_t HYUNDAI_CANFD_HB_LKA_ICON_SETTLE_US = 1500000U;  // camera LFA_ICON 0 for this long = LFA really off (blink <= 1.3 s)
static const uint32_t HYUNDAI_CANFD_HB_LKA_HOLD_US = 500000U;           // LKA_SysIndReq 3 is held this long (longest gap in 2d/2f/30: 0.33 s)
static const uint32_t HYUNDAI_CANFD_HB_LKA_STALE_US = 500000U;          // no camera 0x161 for this long: no window (the camera is blocked, controls stay on)
static bool hyundai_canfd_hb_cam_icon_zero = false;    // camera 0x161 LFA_ICON is 0 now
static uint32_t hyundai_canfd_hb_cam_icon_zero_ts = 0U;  // since when
static uint32_t hyundai_canfd_hb_cam_icon_ts = 0U;     // when the last camera 0x161 arrived (stale or missing: no window)
static bool hyundai_canfd_hb_cam_lka_seen = false;     // a camera 0x12A with LKA_SysIndReq 3 was seen since init
static uint32_t hyundai_canfd_hb_cam_lka_ts = 0U;      // the last one

static void hyundai_canfd_handback_note_action(uint32_t now) {
  hyundai_canfd_hb_long_action = true;
  hyundai_canfd_hb_long_action_ts = now;
}

static bool hyundai_canfd_handback_recent(bool valid, uint32_t ts, uint32_t now) {
  return valid && (safety_get_ts_elapsed(now, ts) <= HYUNDAI_CANFD_HB_ACTION_WINDOW_US);
}

static void hyundai_canfd_handback_step(void) {
  const uint32_t now = microsecond_timer_get();

  // a pause-key release granted controls_allowed but openpilot does not own 0x1A0 yet: the camera's own ACC must not drive in the meantime,
  // so the lock is on from the grant (before the camera's ACCMode rises, or while it is already engaged) until an accepted active openpilot frame
  if (hyundai_pause_resume_granted) {
    hyundai_pause_resume_granted = false;
    if (!hyundai_canfd_hb_long_op && controls_allowed) {
      hyundai_canfd_hb_stock_lock = true;
      hyundai_canfd_hb_cam_standby_frames = 0U;
    }
  }

  const bool brake_edge = brake_pressed && !hyundai_canfd_hb_brake_prev;
  const bool main_off_edge = hyundai_canfd_hb_main_on_prev && !acc_main_on;
  hyundai_canfd_hb_brake_prev = brake_pressed;
  hyundai_canfd_hb_main_on_prev = acc_main_on;

  if (brake_edge || main_off_edge) {
    hyundai_canfd_handback_note_action(now);
  }

  if (hyundai_canfd_hb_long_op && !controls_allowed && hyundai_canfd_handback_recent(hyundai_canfd_hb_long_action, hyundai_canfd_hb_long_action_ts, now)) {
    hyundai_canfd_hb_long_op = false;
    hyundai_canfd_hb_long_action = false;
  }
}

// the camera's own ACC state, from its 0x1A0 on bus 2
static void hyundai_canfd_handback_camera_acc(int acc_mode) {
  const bool engaged = (acc_mode == 1) || (acc_mode == 2);
  const bool rising = engaged && !hyundai_canfd_hb_cam_engaged && hyundai_canfd_hb_cam_seen;
  hyundai_canfd_hb_cam_engaged = engaged;
  hyundai_canfd_hb_cam_seen = true;

  if (rising && !hyundai_canfd_hb_long_op) {
    hyundai_canfd_hb_stock_lock = true;
    hyundai_canfd_hb_cam_standby_frames = 0U;
  }
  if (hyundai_canfd_hb_stock_lock) {
    if (engaged) {
      hyundai_canfd_hb_cam_standby_frames = 0U;
    } else {
      hyundai_canfd_hb_cam_standby_frames = (hyundai_canfd_hb_cam_standby_frames < 255U) ? (hyundai_canfd_hb_cam_standby_frames + 1U) : 255U;
      // while controls are allowed openpilot is about to take the axis (a camera standby must not reopen the gap)
      if ((hyundai_canfd_hb_cam_standby_frames >= HYUNDAI_CANFD_HB_STOCK_STANDBY_FRAMES) && !controls_allowed) {
        hyundai_canfd_hb_stock_lock = false;
      }
    }
  }
}

// the camera's own lane-keeping signals on bus 2: 0x161 LFA_ICON (224|4) and 0x12A LKA_SysIndReq (38|3)
static void hyundai_canfd_handback_camera_lka(const CANPacket_t *msg) {
  const uint32_t now = microsecond_timer_get();
  if (msg->addr == 0x161U) {
    const bool zero = (msg->data[28] & 0xFU) == 0U;
    const bool was_zero = hyundai_canfd_hb_cam_icon_zero && (safety_get_ts_elapsed(now, hyundai_canfd_hb_cam_icon_ts) <= HYUNDAI_CANFD_HB_LKA_STALE_US);
    if (zero && !was_zero) {
      hyundai_canfd_hb_cam_icon_zero_ts = now;
    }
    hyundai_canfd_hb_cam_icon_ts = now;
    hyundai_canfd_hb_cam_icon_zero = zero;
  } else if (msg->addr == 0x12AU) {
    const int ind = ((msg->data[4] >> 6) & 0x3U) | ((msg->data[5] & 0x1U) << 2);
    if (ind == 3) {
      hyundai_canfd_hb_cam_lka_seen = true;
      hyundai_canfd_hb_cam_lka_ts = now;
    }
  } else {
  }
}

// the camera's own lane-keeping nudge may steer: openpilot owns the axis but lateral is off (not even paused), and the window is open
static bool hyundai_canfd_handback_camera_lka_open(void) {
  const uint32_t now = microsecond_timer_get();
  return !(controls_allowed || controls_allowed_lateral) &&
         hyundai_canfd_hb_cam_icon_zero && (safety_get_ts_elapsed(now, hyundai_canfd_hb_cam_icon_ts) <= HYUNDAI_CANFD_HB_LKA_STALE_US) &&
         (safety_get_ts_elapsed(now, hyundai_canfd_hb_cam_icon_zero_ts) >= HYUNDAI_CANFD_HB_LKA_ICON_SETTLE_US) &&
         hyundai_canfd_hb_cam_lka_seen && (safety_get_ts_elapsed(now, hyundai_canfd_hb_cam_lka_ts) <= HYUNDAI_CANFD_HB_LKA_HOLD_US);
}

static void hyundai_canfd_handback_buttons(int cruise_button, bool main_button) {
  const bool cancel_edge = (cruise_button == HYUNDAI_BTN_CANCEL) && !hyundai_canfd_hb_cancel_prev;
  const bool main_edge = main_button && !hyundai_canfd_hb_main_button_prev;
  hyundai_canfd_hb_cancel_prev = (cruise_button == HYUNDAI_BTN_CANCEL);
  hyundai_canfd_hb_main_button_prev = main_button;
  if (hyundai_canfd_handback) {
    if (cancel_edge || main_edge) {
      hyundai_canfd_handback_note_action(microsecond_timer_get());
    }
    hyundai_canfd_handback_step();
  }
}

static void hyundai_canfd_rx_hook(const CANPacket_t *msg) {

  const unsigned pt_bus = hyundai_canfd_lka_steer_msg ? 1U : 0U;
  const unsigned int scc_bus = hyundai_camera_scc ? 2U : pt_bus;

  if (msg->bus == pt_bus) {
    // driver torque
    if (msg->addr == 0xeaU) {
      int torque_driver_new = ((msg->data[11] & 0x1fU) << 8U) | msg->data[10];
      torque_driver_new -= 4095;
      update_sample(&torque_driver, torque_driver_new);

      int angle_meas_new = (msg->data[17] << 8) | msg->data[16];
      angle_meas_new = to_signed(angle_meas_new, 16);
      update_sample(&angle_meas, angle_meas_new);
    }

    // cruise buttons
    const unsigned int button_addr = hyundai_canfd_alt_buttons ? 0x1aaU : 0x1cfU;
    if (msg->addr == button_addr) {
      bool main_button = false;
      int cruise_button = 0;
      if (msg->addr == 0x1cfU) {
        cruise_button = msg->data[2] & 0x7U;
        main_button = GET_BIT(msg, 19U);
        mads_button_press = GET_BIT(msg, 23U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      } else {
        cruise_button = (msg->data[4] >> 4) & 0x7U;
        main_button = GET_BIT(msg, 34U);
        mads_button_press = GET_BIT(msg, 39U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
      }
      hyundai_common_cruise_buttons_check(cruise_button, main_button);
      hyundai_canfd_handback_buttons(cruise_button, main_button);
    }

    // gas press, different for EV, hybrid, and ICE models
    if ((msg->addr == 0x35U) && hyundai_ev_gas_signal) {
      gas_pressed = msg->data[5] != 0U;
    } else if ((msg->addr == 0x105U) && hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(msg, 103U) || (msg->data[13] != 0U) || GET_BIT(msg, 112U);
    } else if ((msg->addr == 0x100U) && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal) {
      gas_pressed = GET_BIT(msg, 176U);
    } else {
    }

    // brake press
    if (msg->addr == 0x175U) {
      brake_pressed = GET_BIT(msg, 81U);
      if (brake_pressed && hyundai_long_main_button_engage) {
        hyundai_common_cancel_resume_disarm();  // a brake press during a pause-key hold cancels the resume, also when released again before the key
      }
    }

    // vehicle moving
    if (msg->addr == 0xa0U) {
      uint32_t fl = (GET_BYTES(msg, 8, 2)) & 0x3FFFU;
      uint32_t fr = (GET_BYTES(msg, 10, 2)) & 0x3FFFU;
      uint32_t rl = (GET_BYTES(msg, 12, 2)) & 0x3FFFU;
      uint32_t rr = (GET_BYTES(msg, 14, 2)) & 0x3FFFU;
      vehicle_moving = (fl > HYUNDAI_STANDSTILL_THRSLD) || (fr > HYUNDAI_STANDSTILL_THRSLD) ||
                       (rl > HYUNDAI_STANDSTILL_THRSLD) || (rr > HYUNDAI_STANDSTILL_THRSLD);

      // average of all 4 wheel speeds. Conversion: raw * 0.03125 / 3.6 = m/s
      UPDATE_VEHICLE_SPEED((fr + rr + rl + fl) / 4.0 * 0.03125 * KPH_TO_MS);
    }
  }

  if (msg->bus == scc_bus) {
    // cruise state
    if ((msg->addr == 0x1a0U) && !hyundai_longitudinal) {
      // 1=enabled, 2=driver override
      int cruise_status = ((msg->data[8] >> 4) & 0x7U);
      bool cruise_engaged = (cruise_status == 1) || (cruise_status == 2);
      hyundai_common_cruise_state_check(cruise_engaged);
      acc_main_on = GET_BIT(msg, 66U);
    }
    if ((msg->addr == 0x1a0U) && hyundai_longitudinal && hyundai_long_main_button_engage) {
      hyundai_common_camera_main_update(GET_BIT(msg, 66U));
      if (hyundai_canfd_handback) {
        hyundai_canfd_handback_camera_acc((msg->data[8] >> 4) & 0x7U);
      }
    }
  }

  if (hyundai_canfd_handback && (msg->bus == 2U)) {
    hyundai_canfd_handback_camera_lka(msg);
  }

  hyundai_common_reset_acc_main_on_mismatches();

  // sample the driver-action edges (brake, main off) on every whitelisted rx, not only when a frame is forwarded or sent
  if (hyundai_canfd_handback) {
    hyundai_canfd_handback_step();
  }
}

// CCNC LFA_ALT: the camera needs MDPS (0xEA) at all times, so pass the real one until openpilot sends its own copy
static bool hyundai_canfd_ccnc_mdps_tx = false;

static bool hyundai_canfd_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits HYUNDAI_CANFD_TORQUE_STEERING_LIMITS = {
    .max_torque = 270,
    .max_rt_delta = 112,
    .max_rate_up = 2,
    .max_rate_down = 3,
    .driver_torque_allowance = 250,
    .driver_torque_multiplier = 2,
    .type = TorqueDriverLimited,

    // the EPS faults when the steering angle is above a certain threshold for too long. to prevent this,
    // we allow setting torque actuation bit to 0 while maintaining the requested torque value for two consecutive frames
    .min_valid_request_frames = 89,
    .max_invalid_request_frames = 2,
    .min_valid_request_rt_interval = 810000,  // 810ms; a ~10% buffer on cutting every 90 frames
    .has_steer_req_tolerance = true,
  };

  const AngleSteeringLimits HYUNDAI_CANFD_ANGLE_STEERING_LIMITS = {
    .max_angle = 3600,
    .angle_deg_to_can = 10,
    .frequency = 100U,
  };

  // We need to find a middle ground between all the possible params or find a way to properly fingerprint.
  // HYUNDAI_IONIQ_5_PE: -0.0008688329819908074
  // KIA_EV6_2025: -0.000889804937754786
  // KIA_EV9: -0.0005410588125765342
  // GENESIS_GV80_2025: -0.0005685702046115589
  // HYUNDAI_SANTA_FE_HEV_5TH_GEN: -0.00059689759884299

  // IONIQ 5 PE values.
  // const AngleSteeringParams HYUNDAI_STEERING_PARAMS = {
  //   .slip_factor = -0.0008688329819908074,  // calc_slip_factor(VM)
  //   .steer_ratio = 14.26,
  //   .wheelbase = 2.97,
  // };

  // // GENESIS_GV80_2025 values. (values can be found on values.py)
  // const AngleSteeringParams HYUNDAI_STEERING_PARAMS = {
  //   .slip_factor = -0.0005685702046115589,  // calc_slip_factor(VM)
  //   .steer_ratio = 14.14,
  //   .wheelbase = 2.95,
  // };

  // HYUNDAI_SANTA_FE_HEV_5TH_GEN values. (values can be found on values.py)
  // const AngleSteeringParams HYUNDAI_STEERING_PARAMS = {
  //   .slip_factor = -0.00059689759884299,  // calc_slip_factor(VM)
  //   .steer_ratio = 13.72,
  //   .wheelbase = 2.81,
  // };

  // KIA_SPORTAGE_HEV_2026 values. (most conservative for now) (values can be found on values.py)
  const AngleSteeringParams HYUNDAI_STEERING_PARAMS = {
      .slip_factor = -0.0006085930193026732,  // calc_slip_factor(VM)
      .steer_ratio = 13.7,
      .wheelbase = 2.756,
    };


  bool tx = true;

  if ((msg->addr == 0xEAU) && (msg->bus == 2U)) {
    if (hyundai_canfd_handback) {
      // openpilot's MDPS copy only while it owns lateral; otherwise the real MDPS is forwarded and a second sender would interleave
      hyundai_canfd_handback_step();
      if (hyundai_canfd_handback_camera_lka_open()) {
        tx = false;
      }
    } else {
      hyundai_canfd_ccnc_mdps_tx = true;
    }
  }

  // HDA1 steering
  if ((msg->addr == 0xCBU) && hyundai_canfd_angle_steering) {
    const int lfa_angle_active = (msg->data[3] >> 4U);
    const bool steer_angle_req = lfa_angle_active == 2;

    int desired_angle = (((uint32_t)(msg->data[5] & 0x3FU)) << 8) | (uint32_t)msg->data[4];
    desired_angle = to_signed(desired_angle, 14);

    if (steer_angle_cmd_checks_vm(desired_angle, steer_angle_req, HYUNDAI_CANFD_ANGLE_STEERING_LIMITS, HYUNDAI_STEERING_PARAMS)) {
      tx = false;
    }

    if (hyundai_canfd_handback) {
      hyundai_canfd_handback_step();
      // openpilot owns lateral: its frames (also inactive) pass, except while the camera's lane-keeping nudge steers (one sender only)
      if (hyundai_canfd_handback_camera_lka_open()) {
        tx = false;
      }
    }
  }

  // steering
  const unsigned int steer_addr = (hyundai_canfd_lka_steer_msg && !hyundai_longitudinal) ? hyundai_canfd_get_lka_addr() : 0x12aU;
  if (msg->addr == steer_addr) {
    if (hyundai_canfd_angle_steering) {
      const int lkas_angle_active = (msg->data[9] >> 4U) & 0x3U;
      const bool steer_angle_req = lkas_angle_active != 1;

      int desired_angle = (msg->data[11] << 6U) | (msg->data[10] >> 2U);
      desired_angle = to_signed(desired_angle, 14);

      // ADAS_ACIAnglTqRedcGainVal: bit 96, 8 bits, unsigned. Raw 0-250 valid, 251-255 reserved.
      const uint8_t gain_raw = msg->data[12];
      bool gain_violation = gain_raw > 250U;
      if (!steer_angle_req && (gain_raw != 0U)) {
        gain_violation = true;
      }

      if (steer_angle_cmd_checks_vm(desired_angle, steer_angle_req, HYUNDAI_CANFD_ANGLE_STEERING_LIMITS, HYUNDAI_STEERING_PARAMS) || gain_violation) {
        tx = false;
      }
    } else {
      int desired_torque = (((msg->data[6] & 0xFU) << 7U) | (msg->data[5] >> 1U)) - 1024U;
      bool steer_req = GET_BIT(msg, 52U);

      if (steer_torque_cmd_checks(desired_torque, steer_req, HYUNDAI_CANFD_TORQUE_STEERING_LIMITS)) {
        tx = false;
      }
    }
  }

  // cruise buttons check
  if (msg->addr == 0x1cfU) {
    int button = msg->data[2] & 0x7U;
    bool is_cancel = (button == HYUNDAI_BTN_CANCEL);
    bool is_resume = (button == HYUNDAI_BTN_RESUME);
    bool is_set = (button == HYUNDAI_BTN_SET);

    bool allowed = (is_cancel && cruise_engaged_prev) || ((is_resume || is_set) && controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  // UDS: only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (((msg->addr == 0x730U) && hyundai_canfd_lka_steer_msg) || ((msg->addr == 0x7D0U) && !hyundai_camera_scc)) {
    if ((GET_BYTES(msg, 0, 4) != 0x00803E02U) || (GET_BYTES(msg, 4, 4) != 0x0U)) {
      tx = false;
    }
  }

  // ACCEL: safety check
  if (msg->addr == 0x1a0U) {
    int desired_accel_raw = (((msg->data[17] & 0x7U) << 8) | msg->data[16]) - 1023U;
    int desired_accel_val = ((msg->data[18] << 4) | (msg->data[17] >> 4)) - 1023U;

    bool violation = false;

    if (hyundai_longitudinal) {
      violation |= longitudinal_accel_checks(desired_accel_raw, HYUNDAI_LONG_LIMITS);
      violation |= longitudinal_accel_checks(desired_accel_val, HYUNDAI_LONG_LIMITS);
    } else {
      // only used to cancel on here
      const int acc_mode = (msg->data[8] >> 4) & 0x7U;
      if (acc_mode != 4) {
        violation = true;
      }

      if ((desired_accel_raw != 0) || (desired_accel_val != 0)) {
        violation = true;
      }
    }

    if (hyundai_canfd_handback && hyundai_longitudinal) {
      hyundai_canfd_handback_step();
      const int hb_acc_mode = (msg->data[8] >> 4) & 0x7U;
      if (!hyundai_canfd_hb_long_op) {
        if (!violation && ((hb_acc_mode == 1) || (hb_acc_mode == 2)) && controls_allowed) {
          hyundai_canfd_hb_long_op = true;
          hyundai_canfd_hb_long_action = false;
          hyundai_canfd_hb_stock_lock = false;
        } else if (hyundai_canfd_hb_stock_lock && !violation && (hb_acc_mode != 1) && (hb_acc_mode != 2)) {
          // stock ACC lock: 'off' frames replace the camera's, they cannot engage anything
        } else {
          violation = true;
        }
      }
    }

    if (violation) {
      tx = false;
    }

    if (!hyundai_long_main_button_engage) {  // there the camera's main state is the only source, nothing to compare
      acc_main_on_tx = GET_BIT(msg, 66U);
      hyundai_common_acc_main_on_sync();
    }
  }

  return tx;
}

static bool hyundai_canfd_fwd_hook(int bus_num, int addr) {
  if (hyundai_canfd_handback) {
    // block the camera's frame only while openpilot owns the axis (the car's own relay/static blocking is disabled for these addresses)
    hyundai_canfd_handback_step();
    if ((bus_num == 2) && (addr == 0xCB)) {
      return !hyundai_canfd_handback_camera_lka_open();
    }
    if ((bus_num == 0) && (addr == 0xEA)) {
      return !hyundai_canfd_handback_camera_lka_open();
    }
    if ((bus_num == 2) && (addr == 0x1A0) && hyundai_longitudinal) {
      // also blocked while the stock ACC lock is on: the camera's ACC engaged without a driver engage for openpilot
      return hyundai_canfd_hb_long_op || hyundai_canfd_hb_stock_lock;
    }
    return false;
  }

  // block the car's MDPS toward the camera only once openpilot sends its own copy (only allowed on the CCNC LFA_ALT list)
  return (bus_num == 0) && (addr == 0xEA) && hyundai_canfd_ccnc_mdps_tx;
}

static safety_config hyundai_canfd_init(uint16_t param) {
  const uint16_t HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT = 128;
  const uint16_t HYUNDAI_PARAM_CANFD_ALT_BUTTONS = 32;
  const uint16_t HYUNDAI_PARAM_CANFD_ANGLE_STEERING = 1024;
  const uint16_t HYUNDAI_PARAM_CCNC = 2048;
  const uint16_t HYUNDAI_PARAM_CANFD_LFA_ALT_STEERING = 4096;
  const uint16_t HYUNDAI_PARAM_LONG_MAIN_BUTTON_ENGAGE = 8192;
  const uint16_t HYUNDAI_PARAM_CAMERA_HANDBACK = 16384;

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(1)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(1, true)
    {0x51,  0, 32, .check_relay = false},  // ADRV_0x51
    {0x730, 1,  8, .check_relay = false},  // tester present for ADAS ECU disable
    {0x160, 1, 16, .check_relay = false},  // ADRV_0x160
    {0x1EA, 1, 32, .check_relay = false},  // ADRV_0x1ea
    {0x200, 1,  8, .check_relay = false},  // ADRV_0x200
    {0x345, 1,  8, .check_relay = false},  // ADRV_0x345
    {0x1DA, 1, 32, .check_relay = false},  // ADRV_0x1da
  };

  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false)
  };

  // ADRV_0x160 is checked for radar liveness
  static const CanMsg HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS[] = {
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2)
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, true)
    {0x160, 0, 16, .check_relay = true}, // ADRV_0x160
    {0x7D0, 0, 8, .check_relay = false},  // tester present for radar ECU disable
  };

  // ADRV_0x160 is checked for relay malfunction
#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)) \
    {0x160, 0, 16, .check_relay = (longitudinal)}, /* ADRV_0x160 */ \

#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_CCNC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)) \
    {0x161, 0, 32, .check_relay = true}, /* CCNC_0x161 */ \
    {0x162, 0, 32, .check_relay = true}, /* CCNC_0x162 */ \
    {0x7C4, 2, 8, .check_relay = true}, /* 0x7C4 */ \

#define HYUNDAI_CANFD_LFA_ALT_STEERING_CAMERA_SCC_TX_MSGS \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_ALT_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false) \
    {0x160, 0, 16, .check_relay = false}, /* ADRV_0x160 */ \

#define HYUNDAI_CANFD_LFA_ALT_STEERING_CAMERA_SCC_CCNC_TX_MSGS \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_ALT_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, false) \
    {0x161, 0, 32, .check_relay = true}, /* CCNC_0x161 */ \
    {0x162, 0, 32, .check_relay = true}, /* CCNC_0x162 */ \
    {0x7C4, 2, 8, .check_relay = true}, /* 0x7C4 */ \
    {0xEA, 2, 24, .check_relay = true, .disable_static_blocking = true}, /* MDPS copy to the camera */ \

// Longitudinal on top of LFA_ALT steering: openpilot sends SCC_CONTROL (the camera's is blocked) but still not 0x12A,
// so the camera's 0x12A keeps reaching the car. On CCNC we do not send ADRV_0x160 either, so the camera's FCA warning light
// frame is forwarded too; on non-CCNC ADRV_0x160 is ours (and relay-checked).
#define HYUNDAI_CANFD_LFA_ALT_STEERING_CAMERA_SCC_LONG_TX_MSGS \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_ALT_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, true) \
    {0x160, 0, 16, .check_relay = true}, /* ADRV_0x160 */ \

#define HYUNDAI_CANFD_LFA_ALT_STEERING_CAMERA_SCC_CCNC_LONG_TX_MSGS \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_ALT_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, true) \
    {0x161, 0, 32, .check_relay = true}, /* CCNC_0x161 */ \
    {0x162, 0, 32, .check_relay = true}, /* CCNC_0x162 */ \
    {0x7C4, 2, 8, .check_relay = true}, /* 0x7C4 */ \
    {0xEA, 2, 24, .check_relay = true, .disable_static_blocking = true}, /* MDPS copy to the camera */ \

// Camera hand-back: 0xCB and 0x1A0 are forwarded from the camera unless openpilot owns the axis (hyundai_canfd_fwd_hook), so static blocking is off.
#define HYUNDAI_CANFD_LFA_ALT_CCNC_HANDBACK_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    {0x1E0, 0, 16, .check_relay = true}, /* LFAHDA_CLUSTER */ \
    {0xCB, 0, 24, .check_relay = true, .disable_static_blocking = true}, /* LFA_ALT */ \
    {0x1A0, 0, 32, .check_relay = (longitudinal), .disable_static_blocking = true}, /* SCC_CONTROL */ \
    {0x161, 0, 32, .check_relay = true}, /* CCNC_0x161 */ \
    {0x162, 0, 32, .check_relay = true}, /* CCNC_0x162 */ \
    {0x7C4, 2, 8, .check_relay = true}, /* 0x7C4 */ \
    {0xEA, 2, 24, .check_relay = true, .disable_static_blocking = true}, /* MDPS copy to the camera */ \

  hyundai_common_init(param);
  hyundai_canfd_ccnc_mdps_tx = false;

  gen_crc_lookup_table_16(0x1021, hyundai_canfd_crc_lut);
  hyundai_canfd_alt_buttons = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ALT_BUTTONS);
  hyundai_canfd_angle_steering = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ANGLE_STEERING);
  // TODO: test this restriction
  hyundai_canfd_lka_steer_msg_alt = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT);
  hyundai_ccnc = GET_FLAG(param, HYUNDAI_PARAM_CCNC);
  // only meaningful for camera SCC, LFA angle steering (see the LFA steering checks below, with and without longitudinal)
  const bool lfa_alt_steering = hyundai_canfd_angle_steering && GET_FLAG(param, HYUNDAI_PARAM_CANFD_LFA_ALT_STEERING);
  // main cruise button engages/disengages openpilot longitudinal; only valid with the CCNC LFA_ALT camera-SCC long config
  hyundai_long_main_button_engage = GET_FLAG(param, HYUNDAI_PARAM_LONG_MAIN_BUTTON_ENGAGE) && lfa_alt_steering && hyundai_ccnc &&
                                    hyundai_camera_scc && hyundai_longitudinal && !hyundai_canfd_alt_buttons && !hyundai_canfd_lka_steer_msg;
  // the main button is the cruise button on this config, so main off ends cruise only
  mads_main_off_keeps_lateral = hyundai_long_main_button_engage;
  hyundai_canfd_handback = GET_FLAG(param, HYUNDAI_PARAM_CAMERA_HANDBACK) && lfa_alt_steering && hyundai_ccnc && hyundai_camera_scc &&
                           !hyundai_canfd_alt_buttons && !hyundai_canfd_lka_steer_msg;
  hyundai_canfd_hb_cam_icon_ts = 0U;
  hyundai_canfd_hb_cam_icon_zero = false;
  hyundai_canfd_hb_cam_icon_zero_ts = 0U;
  hyundai_canfd_hb_cam_lka_seen = false;
  hyundai_canfd_hb_cam_lka_ts = 0U;
  hyundai_canfd_hb_long_op = false;
  hyundai_canfd_hb_long_action = false;
  hyundai_canfd_hb_long_action_ts = 0U;
  hyundai_canfd_hb_brake_prev = false;
  hyundai_canfd_hb_main_on_prev = false;
  hyundai_canfd_hb_cancel_prev = false;
  hyundai_canfd_hb_main_button_prev = false;
  hyundai_canfd_hb_cam_seen = false;
  hyundai_canfd_hb_cam_engaged = false;
  hyundai_canfd_hb_stock_lock = false;
  hyundai_canfd_hb_cam_standby_frames = 0U;

  safety_config ret;
  if (hyundai_longitudinal) {
    if (hyundai_canfd_lka_steer_msg) {
      static RxCheck hyundai_canfd_lka_steer_msg_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
      };

      ret = BUILD_SAFETY_CFG(hyundai_canfd_lka_steer_msg_long_rx_checks, HYUNDAI_CANFD_LKA_STEER_MSG_LONG_TX_MSGS);

    } else {
      // Longitudinal checks for LFA steering
      static RxCheck hyundai_canfd_long_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
      };

      static RxCheck hyundai_canfd_alt_buttons_long_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(true)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_ccnc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_CCNC_TX_MSGS(true)
      };

      static CanMsg hyundai_canfd_lfa_alt_steering_camera_scc_long_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_ALT_STEERING_CAMERA_SCC_LONG_TX_MSGS
      };

      static CanMsg hyundai_canfd_lfa_alt_steering_camera_scc_ccnc_long_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_ALT_STEERING_CAMERA_SCC_CCNC_LONG_TX_MSGS
      };

      static CanMsg hyundai_canfd_lfa_alt_camera_scc_ccnc_handback_long_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_ALT_CCNC_HANDBACK_TX_MSGS(true)
      };

      // main-button engage follows the camera's own MainMode_ACC (bus 2 0x1A0), so that frame must be a checked rx message
      static RxCheck hyundai_canfd_long_camera_main_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static RxCheck hyundai_canfd_alt_buttons_long_camera_main_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static RxCheck hyundai_canfd_long_camera_main_handback_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
        HYUNDAI_CANFD_HB_LKA_RX_CHECKS
      };

      static RxCheck hyundai_canfd_long_handback_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_HB_LKA_RX_CHECKS
      };

      if (hyundai_canfd_handback) {
        if (hyundai_long_main_button_engage) {
          SET_RX_CHECKS(hyundai_canfd_long_camera_main_handback_rx_checks, ret);
        } else {
          SET_RX_CHECKS(hyundai_canfd_long_handback_rx_checks, ret);
        }
      } else if (hyundai_long_main_button_engage) {
        if (hyundai_canfd_alt_buttons) {
          SET_RX_CHECKS(hyundai_canfd_alt_buttons_long_camera_main_rx_checks, ret);
        } else {
          SET_RX_CHECKS(hyundai_canfd_long_camera_main_rx_checks, ret);
        }
      } else if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_long_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_long_rx_checks, ret);
      }

      if (hyundai_camera_scc) {
        if (lfa_alt_steering) {
          if (hyundai_canfd_handback) {
            SET_TX_MSGS(hyundai_canfd_lfa_alt_camera_scc_ccnc_handback_long_tx_msgs, ret);
          } else if (get_hyundai_ccnc()) {
            SET_TX_MSGS(hyundai_canfd_lfa_alt_steering_camera_scc_ccnc_long_tx_msgs, ret);
          } else {
            SET_TX_MSGS(hyundai_canfd_lfa_alt_steering_camera_scc_long_tx_msgs, ret);
          }
        } else if (get_hyundai_ccnc()) {
          SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_ccnc_tx_msgs, ret);
        } else {
          SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
        }
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_LONG_TX_MSGS, ret);
      }
    }

  } else {
    if (hyundai_canfd_lka_steer_msg) {
      // *** LKA steering checks ***
      // E-CAN is on bus 1, SCC messages are sent on cars with ADRV ECU.
      // Does not use the alt buttons message
      static RxCheck hyundai_canfd_lka_steer_msg_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(1)
      };

      SET_RX_CHECKS(hyundai_canfd_lka_steer_msg_rx_checks, ret);
      if (hyundai_canfd_lka_steer_msg_alt) {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS, ret);
      } else {
        SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS, ret);
      }

    } else if (!hyundai_camera_scc) {
      // Radar sends SCC messages on these cars instead of camera
      static RxCheck hyundai_canfd_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      static RxCheck hyundai_canfd_alt_buttons_radar_scc_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(0)
      };

      SET_TX_MSGS(HYUNDAI_CANFD_LFA_STEERING_TX_MSGS, ret);

      if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_radar_scc_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_radar_scc_rx_checks, ret);
      }

    } else {
      // *** LFA steering checks ***
      // Camera sends SCC messages on LFA steering cars.
      // Both button messages exist on some platforms, so we ensure we track the correct one using flag
      static RxCheck hyundai_canfd_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static RxCheck hyundai_canfd_alt_buttons_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
      };

      static RxCheck hyundai_canfd_handback_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
        HYUNDAI_CANFD_HB_LKA_RX_CHECKS
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(false)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_ccnc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_CCNC_TX_MSGS(false)
      };

      static CanMsg hyundai_canfd_lfa_alt_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_ALT_STEERING_CAMERA_SCC_TX_MSGS
      };

      static CanMsg hyundai_canfd_lfa_alt_steering_camera_scc_ccnc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_ALT_STEERING_CAMERA_SCC_CCNC_TX_MSGS
      };

      static CanMsg hyundai_canfd_lfa_alt_camera_scc_ccnc_handback_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_ALT_CCNC_HANDBACK_TX_MSGS(false)
      };

      if (lfa_alt_steering) {
        if (hyundai_canfd_handback) {
          SET_TX_MSGS(hyundai_canfd_lfa_alt_camera_scc_ccnc_handback_tx_msgs, ret);
        } else if (get_hyundai_ccnc()) {
          SET_TX_MSGS(hyundai_canfd_lfa_alt_steering_camera_scc_ccnc_tx_msgs, ret);
        } else {
          SET_TX_MSGS(hyundai_canfd_lfa_alt_steering_camera_scc_tx_msgs, ret);
        }
      } else if (get_hyundai_ccnc()) {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_ccnc_tx_msgs, ret);
      } else {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
      }

      if (hyundai_canfd_handback) {
        SET_RX_CHECKS(hyundai_canfd_handback_rx_checks, ret);
      } else if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_rx_checks, ret);
      }
    }
  }

  return ret;
}

const safety_hooks hyundai_canfd_hooks = {
  .init = hyundai_canfd_init,
  .rx = hyundai_canfd_rx_hook,
  .tx = hyundai_canfd_tx_hook,
  .fwd = hyundai_canfd_fwd_hook,
  .get_counter = hyundai_canfd_get_counter,
  .get_checksum = hyundai_canfd_get_checksum,
  .compute_checksum = hyundai_common_canfd_compute_checksum,
};
