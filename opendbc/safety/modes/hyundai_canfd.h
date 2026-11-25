#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/hyundai_common.h"
#include "opendbc/safety/modes/hyundai_canfd_angle_models.h"

#define HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(bus) \
  {0x1CF, bus, 8, .check_relay = false},  /* CRUISE_BUTTON */   \

#define HYUNDAI_CANFD_CRUISE_BUTTON_ALT_TX_MSGS(bus) \
  {0x1AA, bus, 16, .check_relay = false},  /* CRUISE_BUTTONS_ALT */ \

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
static bool hyundai_canfd_lfa_alt_steer = false;
static uint8_t hyundai_canfd_angle_model_id = HYUNDAI_ANGLE_MODEL_BASELINE;

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

// CAMERA_HANDBACK (GV70 CCNC LFA_ALT camera SCC, SP flag 256): openpilot owns lateral from the panda init on. The camera's 0xCB (LFA_ALT) and the
// real 0xEA (MDPS to the camera) are blocked and openpilot's frames, inactive ones too, replace them, so the car's own LFA lane centering never steers
// (also not after a Driving Assist press that turns it on). The one exception is the car's own lane-keeping nudge (LKA, lane-departure prevention),
// which steers through the same 0xCB while openpilot is not steering. Upstream blocks the camera's 0xCB statically; here the block is dynamic
// (.disable_static_blocking on 0xCB) so the nudge can pass. The camera's 0x1A0 (SCC_CONTROL) keeps upstream's static block: nothing of the
// longitudinal axis is handed back.
//   LKA NUDGE: the camera's 0xCB (and the real MDPS 0xEA for the camera) pass only inside the "lane-keeping window", else openpilot's frames go. The
//              forward hook sees no data, so the window is built from the camera's earlier frames (research/stock-lfa-ownership.md, routes 2d/2f/30):
//                camera 0x161 LFA_ICON has been 0 for >= HYUNDAI_CANFD_HB_LKA_ICON_SETTLE_US (the LFA on/off blink lasts <= 1.3 s, a visible or
//                blinking icon closes the window at once), AND
//                camera 0x12A LKA_SysIndReq == 3 (lane departure) within HYUNDAI_CANFD_HB_LKA_HOLD_US.
//              LFA lane centering never matches: its icon is 1/2 (or the 3,0,3,0 blink), and the frames that precede the icon on an LFA start have
//              0x12A LKA_SysIndReq 1/2. "Not steering": no accepted ACTIVE 0xCB from openpilot for >= HYUNDAI_CANFD_HB_OP_IDLE_US, whatever its
//              permission (route 48 T23: cruise on, steering off by the LFA button; openpilot stopped sending for the nudge, the camera stayed
//              blocked, and the car got no 0xCB for 0.85 s, EPS flag 0.53 s in). An accepted ACTIVE frame while the window is open is dropped and
//              closes it: the camera's frame of that cycle went out, openpilot's next frame takes over. Openpilot's own 0xCB / 0xEA are rejected
//              while the camera owns the slot (never two senders).
// Which 0xCB / MDPS 0xEA sender the car (and the camera) get is an explicit owner, so two senders never meet on the bus whatever the order of the
// frames and status messages (R1 on 3a8016aa: a status frame between the two senders, the 0.2 s boundary inside a cycle, the MDPS copy after the
// real one). A switch happens only after the old sender's frame was dropped or did not come: OURS -> TO_CAM (our frame dropped) -> CAM (the camera's
// next frame forwarded), or OURS -> CAM directly when none of our frames went out since the camera's last one (openpilot silent);
// CAM -> TO_OURS (our frame dropped) -> OURS (the camera's next frame blocked). At most one cycle without a 0xCB at a switch, never two senders.
// A camera that stops for HYUNDAI_CANFD_HB_LAT_SWITCH_US hands back without that frame. The MDPS copy follows OURS, the real MDPS follows CAM.
static bool hyundai_canfd_handback = false;
static const uint32_t HYUNDAI_CANFD_HB_LKA_ICON_SETTLE_US = 1500000U;  // camera LFA_ICON 0 for this long = LFA really off (blink <= 1.3 s)
static const uint32_t HYUNDAI_CANFD_HB_LKA_HOLD_US = 500000U;           // LKA_SysIndReq 3 is held this long (longest gap in 2d/2f/30: 0.33 s)
static const uint32_t HYUNDAI_CANFD_HB_LKA_STALE_US = 500000U;          // no camera 0x161 for this long: no window (the camera is blocked, controls stay on)
static bool hyundai_canfd_hb_cam_icon_zero = false;    // camera 0x161 LFA_ICON is 0 now
static uint32_t hyundai_canfd_hb_cam_icon_zero_ts = 0U;  // since when
static uint32_t hyundai_canfd_hb_cam_icon_ts = 0U;     // when the last camera 0x161 arrived (stale or missing: no window)
static bool hyundai_canfd_hb_cam_icon_settled = false;  // the icon has been 0 for HYUNDAI_CANFD_HB_LKA_ICON_SETTLE_US: latched, never compared across a timer wrap
static bool hyundai_canfd_hb_cam_lka_seen = false;     // a camera 0x12A with LKA_SysIndReq 3 less than HYUNDAI_CANFD_HB_LKA_HOLD_US ago (cleared when it expires)
static uint32_t hyundai_canfd_hb_cam_lka_ts = 0U;      // the last one
static const uint32_t HYUNDAI_CANFD_HB_OP_IDLE_US = 200000U;            // openpilot sent no accepted ACTIVE 0xCB for this long = it is not steering
static bool hyundai_canfd_hb_op_steer_seen = false;    // openpilot sent an accepted ACTIVE 0xCB less than HYUNDAI_CANFD_HB_OP_IDLE_US ago (cleared when it expires)
static uint32_t hyundai_canfd_hb_op_steer_ts = 0U;     // the last one
static const uint8_t HYUNDAI_CANFD_HB_LAT_OURS = 0U;
static const uint8_t HYUNDAI_CANFD_HB_LAT_TO_CAM = 1U;
static const uint8_t HYUNDAI_CANFD_HB_LAT_CAM = 2U;
static const uint8_t HYUNDAI_CANFD_HB_LAT_TO_OURS = 3U;
static const uint32_t HYUNDAI_CANFD_HB_LAT_SWITCH_US = 25000U;          // 2.5 cycles: the other sender's frame did not come
static uint8_t hyundai_canfd_hb_lat_state = 0U;         // HYUNDAI_CANFD_HB_LAT_*
static uint32_t hyundai_canfd_hb_lat_state_ts = 0U;     // when TO_CAM / TO_OURS started
static uint32_t hyundai_canfd_hb_cam_cb_ts = 0U;        // the camera's last forwarded 0xCB (CAM)
static bool hyundai_canfd_hb_op_cb_since_cam = false;  // our 0xCB reached the car since the camera's last 0xCB decision, less than
static uint32_t hyundai_canfd_hb_op_cb_ts = 0U;         // HYUNDAI_CANFD_HB_LAT_CYCLE_US ago: the camera's frame now would be a second sender
static const uint32_t HYUNDAI_CANFD_HB_LAT_CYCLE_US = 15000U;           // 1.5 cycles
static const uint32_t HYUNDAI_CANFD_HB_MDPS_SAME_CYCLE_US = 7000U;      // the real MDPS and our copy less than this apart = the same 10 ms cycle
static bool hyundai_canfd_hb_real_mdps_seen = false;   // the real MDPS (bus 0 -> 2) / our MDPS copy (bus 2) reached the camera less than
static uint32_t hyundai_canfd_hb_real_mdps_ts = 0U;     // HYUNDAI_CANFD_HB_MDPS_SAME_CYCLE_US ago: the other one must not follow in the same cycle
static bool hyundai_canfd_hb_op_mdps_seen = false;     // (the owner can switch between the two; R1 v2, fuzz)
static uint32_t hyundai_canfd_hb_op_mdps_ts = 0U;
static const uint32_t HYUNDAI_CANFD_HB_LAT_HOLD_US = 20000U;  // after the slot came back to openpilot, no switch to the camera for this long: two switches
static bool hyundai_canfd_hb_ours_hold = false;               // in a row would drop two frames in a row (fuzz: the nudge hold ends and a new nudge
static uint32_t hyundai_canfd_hb_ours_since_ts = 0U;          // frame arrives within one cycle)

// MDPS 0xEA to the camera: ours = openpilot's copy (TX bus 2), else the real one (forwarded 0 -> 2). True = it may pass
static bool hyundai_canfd_handback_mdps_ok(bool ours) {
  const uint32_t now = microsecond_timer_get();
  // expire both marks first: an old timestamp never looks recent again after the timer wraps (R1 v3)
  if (hyundai_canfd_hb_real_mdps_seen && (safety_get_ts_elapsed(now, hyundai_canfd_hb_real_mdps_ts) >= HYUNDAI_CANFD_HB_MDPS_SAME_CYCLE_US)) {
    hyundai_canfd_hb_real_mdps_seen = false;
  }
  if (hyundai_canfd_hb_op_mdps_seen && (safety_get_ts_elapsed(now, hyundai_canfd_hb_op_mdps_ts) >= HYUNDAI_CANFD_HB_MDPS_SAME_CYCLE_US)) {
    hyundai_canfd_hb_op_mdps_seen = false;
  }
  const bool other_recent = ours ? hyundai_canfd_hb_real_mdps_seen : hyundai_canfd_hb_op_mdps_seen;
  const bool owner = ours ? (hyundai_canfd_hb_lat_state == HYUNDAI_CANFD_HB_LAT_OURS) : (hyundai_canfd_hb_lat_state == HYUNDAI_CANFD_HB_LAT_CAM);
  const bool ok = owner && !other_recent;
  if (ok && ours) {
    hyundai_canfd_hb_op_mdps_seen = true;
    hyundai_canfd_hb_op_mdps_ts = now;
  } else if (ok) {
    hyundai_canfd_hb_real_mdps_seen = true;
    hyundai_canfd_hb_real_mdps_ts = now;
  } else {
  }
  return ok;
}

// the camera's own lane-keeping signals on bus 2: 0x161 LFA_ICON (224|4) and 0x12A LKA_SysIndReq (38|3)
// Every mark is either latched (settled) or cleared when it expires (icon, hold, openpilot steering), so no elapsed time is compared once it is older
// than the unsigned 32 bit microsecond timer can tell (2^32 us, ~71.6 min): an old event never looks recent again
static void hyundai_canfd_handback_camera_lka(const CANPacket_t *msg) {
  const uint32_t now = microsecond_timer_get();
  if (msg->addr == 0x161U) {
    const bool zero = (msg->data[28] & 0xFU) == 0U;
    const bool was_zero = hyundai_canfd_hb_cam_icon_zero && (safety_get_ts_elapsed(now, hyundai_canfd_hb_cam_icon_ts) <= HYUNDAI_CANFD_HB_LKA_STALE_US);
    if (zero && !was_zero) {
      hyundai_canfd_hb_cam_icon_zero_ts = now;
      hyundai_canfd_hb_cam_icon_settled = false;
    }
    if (zero && !hyundai_canfd_hb_cam_icon_settled && (safety_get_ts_elapsed(now, hyundai_canfd_hb_cam_icon_zero_ts) >= HYUNDAI_CANFD_HB_LKA_ICON_SETTLE_US)) {
      hyundai_canfd_hb_cam_icon_settled = true;
    }
    if (!zero) {
      hyundai_canfd_hb_cam_icon_settled = false;
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

// the camera's own lane-keeping nudge may steer: openpilot owns the axis but is not steering (no permission, or no accepted ACTIVE frame for
// HYUNDAI_CANFD_HB_OP_IDLE_US), and the window is open
static bool hyundai_canfd_handback_camera_lka_open(void) {
  const uint32_t now = microsecond_timer_get();
  if (hyundai_canfd_hb_op_steer_seen && (safety_get_ts_elapsed(now, hyundai_canfd_hb_op_steer_ts) >= HYUNDAI_CANFD_HB_OP_IDLE_US)) {
    hyundai_canfd_hb_op_steer_seen = false;  // expired
  }
  if (hyundai_canfd_hb_cam_icon_zero && (safety_get_ts_elapsed(now, hyundai_canfd_hb_cam_icon_ts) > HYUNDAI_CANFD_HB_LKA_STALE_US)) {
    hyundai_canfd_hb_cam_icon_zero = false;  // no camera 0x161 for that long: the icon state and its settle time are gone
    hyundai_canfd_hb_cam_icon_settled = false;
  }
  if (hyundai_canfd_hb_cam_lka_seen && (safety_get_ts_elapsed(now, hyundai_canfd_hb_cam_lka_ts) > HYUNDAI_CANFD_HB_LKA_HOLD_US)) {
    hyundai_canfd_hb_cam_lka_seen = false;  // the lane-departure hold is over
  }
  const bool op_steering = (controls_allowed || controls_allowed_lateral) && hyundai_canfd_hb_op_steer_seen;
  return !op_steering && hyundai_canfd_hb_cam_icon_zero && hyundai_canfd_hb_cam_icon_settled && hyundai_canfd_hb_cam_lka_seen;
}

static void hyundai_canfd_handback_lat_to_ours(void) {
  if (hyundai_canfd_hb_lat_state != HYUNDAI_CANFD_HB_LAT_OURS) {
    hyundai_canfd_hb_lat_state = HYUNDAI_CANFD_HB_LAT_OURS;
    hyundai_canfd_hb_ours_hold = true;
    hyundai_canfd_hb_ours_since_ts = microsecond_timer_get();
  }
}

// the camera may take the slot from openpilot's frame (which is then dropped): its window is open and the slot has been openpilot's for at least
// HYUNDAI_CANFD_HB_LAT_HOLD_US. Taking it while openpilot is silent drops nothing and needs no hold (R1 v3: else up to 30 ms without 0xCB)
static bool hyundai_canfd_handback_cam_may_take(bool eligible) {
  if (hyundai_canfd_hb_ours_hold && (safety_get_ts_elapsed(microsecond_timer_get(), hyundai_canfd_hb_ours_since_ts) >= HYUNDAI_CANFD_HB_LAT_HOLD_US)) {
    hyundai_canfd_hb_ours_hold = false;  // expired: an old timestamp never looks recent again after the timer wraps
  }
  return eligible && !hyundai_canfd_hb_ours_hold;
}

// our 0xCB: true = it may reach the car (the owner rules above), from the window decision made before this frame
static bool hyundai_canfd_handback_lat_tx_ok(bool eligible) {
  const uint32_t now = microsecond_timer_get();
  bool ok = false;
  if (hyundai_canfd_hb_lat_state == HYUNDAI_CANFD_HB_LAT_OURS) {
    if (hyundai_canfd_handback_cam_may_take(eligible)) {
      hyundai_canfd_hb_lat_state = HYUNDAI_CANFD_HB_LAT_TO_CAM;  // the camera's nudge: our frame is the one dropped at the switch
      hyundai_canfd_hb_lat_state_ts = now;
    } else {
      ok = true;
    }
  } else if (hyundai_canfd_hb_lat_state == HYUNDAI_CANFD_HB_LAT_CAM) {
    if (safety_get_ts_elapsed(now, hyundai_canfd_hb_cam_cb_ts) > HYUNDAI_CANFD_HB_LAT_SWITCH_US) {
      hyundai_canfd_handback_lat_to_ours();  // the camera stopped sending
      ok = true;
    } else if (!eligible) {
      hyundai_canfd_hb_lat_state = HYUNDAI_CANFD_HB_LAT_TO_OURS;  // openpilot steers (or the window closed): ours is the frame dropped at the switch
      hyundai_canfd_hb_lat_state_ts = now;
    } else {
    }
  } else {  // TO_CAM, TO_OURS: waiting for the camera's frame; it did not come
    if (safety_get_ts_elapsed(now, hyundai_canfd_hb_lat_state_ts) > HYUNDAI_CANFD_HB_LAT_SWITCH_US) {
      hyundai_canfd_handback_lat_to_ours();
      ok = true;
    }
  }
  return ok;
}

// the camera's 0xCB: true = forward it to the car
static bool hyundai_canfd_handback_lat_fwd_ok(void) {
  const uint32_t now = microsecond_timer_get();
  const bool eligible = hyundai_canfd_handback_camera_lka_open();
  bool ok = false;
  if ((hyundai_canfd_hb_lat_state == HYUNDAI_CANFD_HB_LAT_TO_CAM) || (hyundai_canfd_hb_lat_state == HYUNDAI_CANFD_HB_LAT_CAM)) {
    ok = eligible;
  } else if ((hyundai_canfd_hb_lat_state == HYUNDAI_CANFD_HB_LAT_OURS) &&
             !(hyundai_canfd_hb_op_cb_since_cam && (safety_get_ts_elapsed(now, hyundai_canfd_hb_op_cb_ts) <= HYUNDAI_CANFD_HB_LAT_CYCLE_US))) {
    ok = eligible;  // directly only when no frame of ours went out since the camera's last one (openpilot silent: nothing to drop, no hold)
  } else {
  }
  hyundai_canfd_hb_op_cb_since_cam = false;
  if (ok) {
    hyundai_canfd_hb_lat_state = HYUNDAI_CANFD_HB_LAT_CAM;
    hyundai_canfd_hb_cam_cb_ts = now;
  } else {
    hyundai_canfd_handback_lat_to_ours();  // TO_OURS ends here: this camera frame is the one dropped at the switch
  }
  return ok;
}

static void hyundai_canfd_rx_hook(const CANPacket_t *msg) {
  const unsigned pt_bus = hyundai_canfd_lka_steer_msg ? 1U : 0U;
  const unsigned int scc_bus = hyundai_camera_scc ? 2U : pt_bus;

  // driver torque
  if (msg_matches(msg, 0xeaU, pt_bus)) {
    int torque_driver_new = ((msg->data[11] & 0x1fU) << 8U) | msg->data[10];
    torque_driver_new -= 4095;
    update_sample(&torque_driver, torque_driver_new);

    int angle_meas_new = (msg->data[17] << 8) | msg->data[16];
    angle_meas_new = to_signed(angle_meas_new, 16);
    update_sample(&angle_meas, angle_meas_new);
  }

  // cruise buttons
  const unsigned int button_addr = hyundai_canfd_alt_buttons ? 0x1aaU : 0x1cfU;
  if (msg_matches(msg, button_addr, pt_bus)) {
    bool main_button = false;
    int cruise_button = 0;
    if (msg_matches(msg, 0x1cfU, pt_bus)) {
      cruise_button = msg->data[2] & 0x7U;
      // a long MAIN sends only NORMAL_CRUISE_MAIN_BTN (bit 21, GV70 CCNC): with main-button engage it is the same driver MAIN press
      main_button = GET_BIT(msg, 19U) || (hyundai_long_main_button_engage && GET_BIT(msg, 21U));
      mads_button_press = GET_BIT(msg, 23U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    } else {
      cruise_button = (msg->data[4] >> 4) & 0x7U;
      main_button = GET_BIT(msg, 34U);
      mads_button_press = GET_BIT(msg, 39U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }
    hyundai_common_cruise_buttons_check(cruise_button, main_button);
  }

  // gas press, different for EV, hybrid, and ICE models
  if (msg_matches(msg, 0x35U, pt_bus) && hyundai_ev_gas_signal) {
    gas_pressed = msg->data[5] != 0U;
  }
  if (msg_matches(msg, 0x105U, pt_bus) && hyundai_hybrid_gas_signal) {
    gas_pressed = ((msg->data[12] >> 7) | msg->data[13] | (msg->data[14] & 1U)) != 0U;
  }
  if (msg_matches(msg, 0x100U, pt_bus) && !hyundai_ev_gas_signal && !hyundai_hybrid_gas_signal) {
    gas_pressed = GET_BIT(msg, 176U);
  }

  // brake press
  if (msg_matches(msg, 0x175U, pt_bus)) {
    brake_pressed = GET_BIT(msg, 81U);
    if (brake_pressed && hyundai_long_main_button_engage) {
      hyundai_common_cancel_resume_disarm();  // a brake press during a pause-key hold cancels the resume, also when released again before the key
    }
  }

  // vehicle moving
  if (msg_matches(msg, 0xa0U, pt_bus)) {
    uint32_t fl = (GET_BYTES_LE(msg, 8, 2)) & 0x3FFFU;
    uint32_t fr = (GET_BYTES_LE(msg, 10, 2)) & 0x3FFFU;
    uint32_t rl = (GET_BYTES_LE(msg, 12, 2)) & 0x3FFFU;
    uint32_t rr = (GET_BYTES_LE(msg, 14, 2)) & 0x3FFFU;
    vehicle_moving = (fl > HYUNDAI_STANDSTILL_THRSLD) || (fr > HYUNDAI_STANDSTILL_THRSLD) ||
                     (rl > HYUNDAI_STANDSTILL_THRSLD) || (rr > HYUNDAI_STANDSTILL_THRSLD);

    // average of all 4 wheel speeds. Conversion: raw * 0.03125 / 3.6 = m/s
    UPDATE_VEHICLE_SPEED((fr + rr + rl + fl) / 4.0 * 0.03125 * KPH_TO_MS);
  }

  // cruise state
  if (msg_matches(msg, 0x1a0U, scc_bus) && !hyundai_longitudinal) {
    // 1=enabled, 2=driver override
    int cruise_status = ((msg->data[8] >> 4) & 0x7U);
    bool cruise_engaged = (cruise_status == 1) || (cruise_status == 2);
    hyundai_common_cruise_state_check(cruise_engaged);
    acc_main_on = GET_BIT(msg, 66U);
  }
  if (msg_matches(msg, 0x1a0U, scc_bus) && hyundai_longitudinal && hyundai_long_main_button_engage) {
    hyundai_common_camera_main_update(GET_BIT(msg, 66U));
  }

  if (hyundai_canfd_handback && (msg->bus == 2U)) {
    hyundai_canfd_handback_camera_lka(msg);
  }

  hyundai_common_reset_acc_main_on_mismatches();
}

// LFA_ALT on CCNC: the camera needs an MDPS (0xEA) at all times, so the car's real one is forwarded until openpilot
// sends its own copy; from then on only openpilot's copy reaches the camera
static bool hyundai_canfd_mdps_tx = false;

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

  // the MDPS faults when LFA_ALT requests more than ~175 deg
  const AngleSteeringLimits HYUNDAI_CANFD_ANGLE_STEERING_LIMITS_LFA_ALT = {
    .max_angle = 1750,
    .angle_deg_to_can = 10,
    .frequency = 100U,
  };

  // Per-vehicle angle steering params, resolved at init from safety_param_sp
  const AngleSteeringParams *HYUNDAI_STEERING_PARAMS = hyundai_get_angle_params(hyundai_canfd_angle_model_id);

  bool tx = true;

  if (hyundai_canfd_lfa_alt_steer && (msg->addr == 0xEAU) && (msg->bus == 2U)) {
    hyundai_canfd_mdps_tx = true;
    // with the hand-back openpilot's MDPS copy only goes to the camera while openpilot owns lateral; during the car's own nudge the real MDPS
    // is forwarded (CAM) and a second sender would interleave. At a switch neither: the camera may miss one MDPS frame, never get two
    if (hyundai_canfd_handback && !hyundai_canfd_handback_mdps_ok(true)) {
      tx = false;
    }
  }

  // steering
  unsigned int steer_addr = (hyundai_canfd_lka_steer_msg && !hyundai_longitudinal) ? hyundai_canfd_get_lka_addr() : 0x12aU;
  if (hyundai_canfd_lfa_alt_steer) {
    steer_addr = 0xcbU;
  }
  if (msg->addr == steer_addr) {
    if (hyundai_canfd_angle_steering) {
      bool steer_angle_req;
      int desired_angle;
      uint8_t gain_raw;
      bool violation = false;
      if (hyundai_canfd_lfa_alt_steer) {
        // LFA_ALT: ADAS_ActvACISta bit 24, ADAS_ActvACILvl2Sta bit 28, ADAS_StrAnglReqVal bit 32,
        // ADAS_ACIAnglTqRedcGainVal bit 48, FCA_ESA_ActvSta bit 56, FCA_ESA_TqBstGainVal bit 64
        steer_angle_req = (msg->data[3] >> 4U) == 2U;
        desired_angle = ((msg->data[5] & 0x3FU) << 8U) | msg->data[4];
        gain_raw = msg->data[6];

        // the MDPS also acts on the level 1 interface and the emergency steering boost, never requested
        if (((msg->data[3] & 0xFU) != 0U) || ((msg->data[7] & 0x3U) != 0U) || (msg->data[8] != 0U)) {
          violation = true;
        }
      } else {
        const int lkas_angle_active = (msg->data[9] >> 4U) & 0x3U;
        steer_angle_req = lkas_angle_active != 1;
        desired_angle = (msg->data[11] << 6U) | (msg->data[10] >> 2U);
        // ADAS_ACIAnglTqRedcGainVal: bit 96, 8 bits, unsigned
        gain_raw = msg->data[12];
      }
      desired_angle = to_signed(desired_angle, 14);

      // gain raw 0-250 valid, 251-255 reserved
      if (gain_raw > 250U) {
        violation = true;
      }
      if (!steer_angle_req && (gain_raw != 0U)) {
        violation = true;
      }

      const AngleSteeringLimits *angle_limits = hyundai_canfd_lfa_alt_steer ? &HYUNDAI_CANFD_ANGLE_STEERING_LIMITS_LFA_ALT : &HYUNDAI_CANFD_ANGLE_STEERING_LIMITS;
      if (steer_angle_cmd_checks_vm(desired_angle, steer_angle_req, *angle_limits, *HYUNDAI_STEERING_PARAMS) || violation) {
        tx = false;
      }

      if (hyundai_canfd_handback) {
        // one 0xCB sender (see HYUNDAI_CANFD_HB_LAT_*): openpilot's frames (also inactive) while it owns lateral, the camera's lane-departure nudge
        // while openpilot is not steering. An accepted ACTIVE frame (with permission) marks openpilot as steering: the window closes, and if the
        // camera owns the slot this frame is the one dropped at the switch. Never two senders; at most one cycle without a frame at a switch
        // (route 48: 0.85 s without a frame, the EPS flagged 0.53 s in)
        if (tx && steer_angle_req && (controls_allowed || controls_allowed_lateral)) {
          hyundai_canfd_hb_op_steer_seen = true;
          hyundai_canfd_hb_op_steer_ts = microsecond_timer_get();
        }
        if (!hyundai_canfd_handback_lat_tx_ok(hyundai_canfd_handback_camera_lka_open())) {
          tx = false;
        }
        if (tx) {
          hyundai_canfd_hb_op_cb_since_cam = true;
          hyundai_canfd_hb_op_cb_ts = microsecond_timer_get();
        }
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
  if (((msg->addr == 0x1cfU) || (msg->addr == 0x1aaU)) && !(hyundai_ccnc && hyundai_camera_scc && hyundai_longitudinal)) {
    int button = (msg->addr == 0x1cfU) ? (msg->data[2] & 0x7U) : ((msg->data[4] >> 4) & 0x7U);
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
    if (GET_BYTES_64_LE(msg, 0, 8) != 0x0000000000803E02ULL) {
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
    if ((bus_num == 2) && (addr == 0xCB)) {
      return !hyundai_canfd_handback_lat_fwd_ok();  // the camera's 0xCB: only the lane-keeping nudge (the static block is off for this address)
    }
    if ((bus_num == 0) && (addr == 0xEA)) {
      // the real MDPS reaches the camera until openpilot sends its own copy, then only while the camera's 0xCB steers
      if (!hyundai_canfd_mdps_tx) {
        // passes without an owner decision, but counts as the camera's MDPS of this cycle: our first copy must not follow it within the same cycle
        hyundai_canfd_hb_real_mdps_seen = true;
        hyundai_canfd_hb_real_mdps_ts = microsecond_timer_get();
        return false;
      }
      return !hyundai_canfd_handback_mdps_ok(false);
    }
    return false;
  }

  // the camera's 0xCB is blocked for good (the static block is off for this address so that the hand-back can open it), and the car's MDPS toward
  // the camera only once openpilot sends its own copy
  return ((bus_num == 2) && (addr == 0xCB) && hyundai_canfd_lfa_alt_steer) || ((bus_num == 0) && (addr == 0xEA) && hyundai_canfd_mdps_tx);
}

static safety_config hyundai_canfd_init(uint16_t param) {
  const uint16_t HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT = 128;
  const uint16_t HYUNDAI_PARAM_CANFD_ALT_BUTTONS = 32;

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(0, 1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_ALT_BUTTONS_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_CRUISE_BUTTON_ALT_TX_MSGS(1)
  };

  static const CanMsg HYUNDAI_CANFD_LKA_STEER_MSG_ALT_ALT_BUTTONS_TX_MSGS[] = {
    HYUNDAI_CANFD_LKA_STEER_MSG_ALT_COMMON_TX_MSGS(0, 1)
    HYUNDAI_CANFD_CRUISE_BUTTON_ALT_TX_MSGS(1)
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
    {0x160, 0, 16, .check_relay = true},  // ADRV_0x160
    {0x7D0, 0, 8, .check_relay = false},  // tester present for radar ECU disable
  };

  // ADRV_0x160 is checked for relay malfunction
#define HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(longitudinal) \
    HYUNDAI_CANFD_CRUISE_BUTTON_TX_MSGS(2) \
    HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0) \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal)) \
    {0x160, 0, 16, .check_relay = (longitudinal)}, /* ADRV_0x160 */ \

  // angle steering straight to the MDPS on LFA_ALT, the camera's LFA and LFAHDA_CLUSTER are forwarded
#define HYUNDAI_CANFD_LFA_ALT_STEERING_CAMERA_SCC_CCNC_TX_MSGS(longitudinal) \
    {0xCB,  0, 24, .check_relay = true, .disable_static_blocking = true}, /* LFA_ALT, blocked by the fwd hook (the lane-keeping nudge may pass) */ \
    HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, (longitudinal))          \
    {0x1CF, 2,  8, .check_relay = (longitudinal)},  /* CRUISE_BUTTONS */ \
    {0x161, 0, 32, .check_relay = true},            /* CCNC_0x161 */     \
    {0x162, 0, 32, .check_relay = true},            /* CCNC_0x162 */     \
    {0xEA,  2, 24, .check_relay = true, .disable_static_blocking = true}, /* MDPS, forwarded until openpilot sends its own */ \

  hyundai_common_init(param);

  hyundai_canfd_mdps_tx = false;
  gen_crc_lookup_table_16(0x1021, hyundai_canfd_crc_lut);
  hyundai_canfd_alt_buttons = GET_FLAG(param, HYUNDAI_PARAM_CANFD_ALT_BUTTONS);
  hyundai_canfd_angle_steering = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_CANFD_ANGLE_STEERING);
  hyundai_canfd_angle_model_id = hyundai_get_angle_model_id(current_safety_param_sp);
  // TODO: test this restriction
  hyundai_canfd_lka_steer_msg_alt = GET_FLAG(param, HYUNDAI_PARAM_CANFD_LKA_STEER_MSG_ALT);
  hyundai_canfd_lfa_alt_steer = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_CANFD_LFA_ALT_STEER) && hyundai_canfd_angle_steering &&
                                !hyundai_canfd_lka_steer_msg && hyundai_camera_scc && hyundai_ccnc;
  // main button engage: the camera's main state engages openpilot longitudinal; only valid with the CCNC LFA_ALT camera-SCC long config
  hyundai_long_main_button_engage = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_MAIN_BUTTON_ENGAGE) && hyundai_canfd_lfa_alt_steer &&
                                    hyundai_longitudinal && !hyundai_canfd_alt_buttons;
  // the main button is the cruise button on this config, so main off ends cruise only
  mads_main_off_keeps_lateral = hyundai_long_main_button_engage;
  // lateral hand-back: the camera's lane-keeping nudge passes while openpilot is not steering (see the CAMERA_HANDBACK description above)
  hyundai_canfd_handback = GET_FLAG(current_safety_param_sp, HYUNDAI_PARAM_SP_CAMERA_HANDBACK) && hyundai_canfd_lfa_alt_steer && !hyundai_canfd_alt_buttons;
  hyundai_canfd_hb_cam_icon_ts = 0U;
  hyundai_canfd_hb_cam_icon_zero = false;
  hyundai_canfd_hb_cam_icon_zero_ts = 0U;
  hyundai_canfd_hb_cam_icon_settled = false;
  hyundai_canfd_hb_cam_lka_seen = false;
  hyundai_canfd_hb_cam_lka_ts = 0U;
  hyundai_canfd_hb_op_steer_seen = false;
  hyundai_canfd_hb_op_steer_ts = 0U;
  hyundai_canfd_hb_lat_state = HYUNDAI_CANFD_HB_LAT_OURS;
  hyundai_canfd_hb_lat_state_ts = 0U;
  hyundai_canfd_hb_ours_hold = false;
  hyundai_canfd_hb_ours_since_ts = 0U;
  hyundai_canfd_hb_cam_cb_ts = 0U;
  hyundai_canfd_hb_op_cb_since_cam = false;
  hyundai_canfd_hb_op_cb_ts = 0U;
  hyundai_canfd_hb_real_mdps_seen = false;
  hyundai_canfd_hb_real_mdps_ts = 0U;
  hyundai_canfd_hb_op_mdps_seen = false;
  hyundai_canfd_hb_op_mdps_ts = 0U;

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
        HYUNDAI_CANFD_LFA_STEERING_COMMON_TX_MSGS(0)
        HYUNDAI_CANFD_SCC_CONTROL_COMMON_TX_MSGS(0, true)
        {0x1CF, 2,  8, .check_relay = true},  // CRUISE_BUTTONS
        {0x161, 0, 32, .check_relay = true},  // CCNC_0x161
        {0x162, 0, 32, .check_relay = true},  // CCNC_0x162
        {0xEA,  2, 24, .check_relay = true},  // MDPS
      };

      // main-button engage follows the camera's own MainMode_ACC (bus 2 0x1A0), so that frame must be a checked rx message
      static RxCheck hyundai_canfd_long_camera_main_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
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

      if (hyundai_canfd_handback && hyundai_long_main_button_engage) {
        SET_RX_CHECKS(hyundai_canfd_long_camera_main_handback_rx_checks, ret);
      } else if (hyundai_canfd_handback) {
        SET_RX_CHECKS(hyundai_canfd_long_handback_rx_checks, ret);
      } else if (hyundai_long_main_button_engage) {
        SET_RX_CHECKS(hyundai_canfd_long_camera_main_rx_checks, ret);
      } else if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_alt_buttons_long_rx_checks, ret);
      } else {
        SET_RX_CHECKS(hyundai_canfd_long_rx_checks, ret);
      }

      static CanMsg hyundai_canfd_lfa_alt_steering_camera_scc_ccnc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_ALT_STEERING_CAMERA_SCC_CCNC_TX_MSGS(true)
      };

      if (hyundai_camera_scc) {
        if (hyundai_canfd_lfa_alt_steer) {
          SET_TX_MSGS(hyundai_canfd_lfa_alt_steering_camera_scc_ccnc_tx_msgs, ret);
        } else if (hyundai_ccnc) {
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
      static RxCheck hyundai_canfd_lka_steer_msg_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(1)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(1)
      };

      static RxCheck hyundai_canfd_lka_steer_msg_alt_buttons_rx_checks[] = {
        HYUNDAI_CANFD_ALT_BUTTONS_RX_CHECKS(1)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(1)
      };

      if (hyundai_canfd_alt_buttons) {
        SET_RX_CHECKS(hyundai_canfd_lka_steer_msg_alt_buttons_rx_checks, ret);
        if (hyundai_canfd_lka_steer_msg_alt) {
          SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_ALT_ALT_BUTTONS_TX_MSGS, ret);
        } else {
          SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_ALT_BUTTONS_TX_MSGS, ret);
        }
      } else {
        SET_RX_CHECKS(hyundai_canfd_lka_steer_msg_rx_checks, ret);
        if (hyundai_canfd_lka_steer_msg_alt) {
          SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_ALT_TX_MSGS, ret);
        } else {
          SET_TX_MSGS(HYUNDAI_CANFD_LKA_STEER_MSG_TX_MSGS, ret);
        }
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

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(false)
      };

      static CanMsg hyundai_canfd_lfa_steering_camera_scc_ccnc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_STEERING_CAMERA_SCC_TX_MSGS(false)
        {0x161, 0, 32, .check_relay = true},  // CCNC_0x161
        {0x162, 0, 32, .check_relay = true},  // CCNC_0x162
        {0xEA,  2, 24, .check_relay = true},  // MDPS
      };

      static CanMsg hyundai_canfd_lfa_alt_steering_camera_scc_ccnc_tx_msgs[] = {
        HYUNDAI_CANFD_LFA_ALT_STEERING_CAMERA_SCC_CCNC_TX_MSGS(false)
      };

      if (hyundai_canfd_lfa_alt_steer) {
        SET_TX_MSGS(hyundai_canfd_lfa_alt_steering_camera_scc_ccnc_tx_msgs, ret);
      } else if (hyundai_ccnc) {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_ccnc_tx_msgs, ret);
      } else {
        SET_TX_MSGS(hyundai_canfd_lfa_steering_camera_scc_tx_msgs, ret);
      }

      static RxCheck hyundai_canfd_handback_rx_checks[] = {
        HYUNDAI_CANFD_STD_BUTTONS_RX_CHECKS(0)
        HYUNDAI_CANFD_SCC_ADDR_CHECK(2)
        HYUNDAI_CANFD_HB_LKA_RX_CHECKS
      };

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
