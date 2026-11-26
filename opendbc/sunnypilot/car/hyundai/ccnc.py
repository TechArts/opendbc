"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
from opendbc.car import DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.crc import CRC8J1850, mk_crc8_fun
from opendbc.car.hyundai.values import HyundaiFlags, Buttons
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

cruise_buttons_checksum = mk_crc8_fun(CRC8J1850, init_crc=0x97, xor_out=0x97)  # mk_crc8_fun xors init with xor_out: register starts at 0

CANCEL_BUTTON_DELAY_FRAMES = 10  # 100 ms
MAIN_PRESS_FRAMES = 12  # 120 ms
OVERRIDE_BLINK_FRAMES = 100
OVERRIDE_BLINK_ON_FRAMES = 60
STOP_DISTANCE_BP = [0.0, 5.0]  # m/s
STOP_DISTANCE_V = [3.5, 6.0]  # m
T_FOLLOW = {1: 1.25, 2: 1.45, 3: 1.75}  # s
TARGET_DISTANCE_MAX = 204.6  # m
LEAD_FILTER_RC = 0.5  # s
LANE_CHANGE_SPEED_MIN = 8.9408  # m/s (20 mph): below it openpilot does not change lanes, so the lane-change icons stay hidden

# CCNC_0x161 value tables (hyundai_canfd.dbc VAL_ 353)
LANELINE_GRAY, LANELINE_HIDDEN, LANELINE_WHITE, LANELINE_ORANGE, LANELINE_GREEN = 0, 1, 2, 4, 6
LCA_HIDDEN, LCA_GRAY, LCA_GREEN, LCA_WHITE = 0, 1, 2, 4
LCA_ARROW_SHOWN = 2  # the lane-change arrow value seen on the car; the value table lists 1 = VISIBLE
LANE_POSITION_CENTER = 15  # LANELINE_*_POSITION: 0..30 across the lane picture, 15 + 15 = centered
# steering angle / 4.5 (-15..15) -> LANELINE_CURVATURE (14..0 left, 31, 15..30 right; 15 straight)
LANELINE_CURVATURE = {i: (31 if i == -1 else 13 - abs(i + 15)) if i < 0 else 15 + i for i in range(-15, 16)}


def lane_line_positions(msg_1b5) -> tuple[int, int]:
  """The dash lane-line positions (left, right, summing to 30) from the camera's lane distances (FR_CMR_03_50ms): the lines slide across the
  picture while the car crosses a lane. Same mapping as royjr's ccnc-port."""
  left_raw, right_raw = msg_1b5["Info_LftLnPosVal"], msg_1b5["Info_RtLnPosVal"]
  scale_per_m = LANE_POSITION_CENTER / 1.7
  left = abs(int(round(LANE_POSITION_CENTER + (left_raw - 1.7) * scale_per_m)))
  right = abs(int(round(LANE_POSITION_CENTER + (right_raw - 1.7) * scale_per_m)))
  if msg_1b5["Info_LftLnQualSta"] not in (2, 3):
    left = 0
  if msg_1b5["Info_RtLnQualSta"] not in (2, 3):
    right = 0
  if left_raw == -2.0248375:
    left = 30 - right
  if right_raw == 2.0248375:
    right = 30 - left
  if left_raw == right_raw == 0:
    left = right = LANE_POSITION_CENTER
  elif left_raw == 0:
    left = 30 - right
  elif right_raw == 0:
    right = 30 - left
  left, right = (min(max(v, 0), 30) for v in (left, right))  # a lane over ~3.4 m away, or a "30 - other" branch, must stay inside the picture
  total = left + right
  if total == 0:
    return LANE_POSITION_CENTER, LANE_POSITION_CENTER
  left = round(left / total * 30)
  return left, 30 - left


def lateral_dash(lat_active, hud_control, out, msg_1b5) -> dict:
  """The 0x161 lateral fields while openpilot steers (lat_active) or pauses (grey wheel). Steering: one green wheel and the green center line;
  lane lines white as on the stock car, orange on a lane departure. A lane change (blinker on while steering at >= 20 mph, where openpilot
  changes lanes): both lines green and sliding across the picture with the camera's lane positions, the blinker side's lane-change icon green
  with its arrow. Lane-change icons otherwise white at >= 20 mph, hidden below; grey when the blind-spot sensor sees a car on that side. LKA
  icon hidden (the wheel says it all)."""
  lane_change_speed = out.vEgo >= LANE_CHANGE_SPEED_MIN
  lane_change = lat_active and lane_change_speed and (out.leftBlinker or out.rightBlinker)

  def line(visible, depart):
    if not visible:
      return LANELINE_HIDDEN
    return LANELINE_ORANGE if depart else LANELINE_GREEN if lane_change else LANELINE_WHITE

  def lca(side_blinker, side_blindspot):
    if not (lat_active and lane_change_speed):
      return LCA_HIDDEN
    return LCA_GRAY if side_blindspot else LCA_GREEN if side_blinker else LCA_WHITE

  values = {
    "LFA_ICON": 2 if lat_active else 1,
    "LKA_ICON": 0,
    "CENTERLINE": 1 if lat_active else 0,
    "LANELINE_LEFT": line(hud_control.leftLaneVisible, hud_control.leftLaneDepart),
    "LANELINE_RIGHT": line(hud_control.rightLaneVisible, hud_control.rightLaneDepart),
    "LANELINE_CURVATURE": (LANELINE_CURVATURE.get(max(-15, min(int(out.steeringAngleDeg / 4.5), 15)), 14)
                           if lat_active and not lane_change else LANE_POSITION_CENTER),
    "LCA_LEFT_ICON": lca(out.leftBlinker, out.leftBlindspot),
    "LCA_RIGHT_ICON": lca(out.rightBlinker, out.rightBlindspot),
    "LCA_LEFT_ARROW": LCA_ARROW_SHOWN if lane_change and out.leftBlinker else 0,
    "LCA_RIGHT_ARROW": LCA_ARROW_SHOWN if lane_change and out.rightBlinker else 0,
  }
  if lane_change and msg_1b5:
    values["LANELINE_LEFT_POSITION"], values["LANELINE_RIGHT_POSITION"] = lane_line_positions(msg_1b5)
  return values


def create_ccnc_mdps(packer, CAN, mdps):
  values = {k: v for k, v in mdps.items() if k != "COUNTER"}

  values.update({
    "MDPS_LkaToiActvSta": 0,
    "MDPS_LkaToiUnblSta": 1,
  })

  return packer.make_can_msg("MDPS", CAN.CAM, values)


def create_ccnc_0x161(packer, CAN, frame, msg_161, enabled, lat_active, paused, long_enabled, main_cruise_enabled, gas_override,
                      hud_control, set_speed, v_ego, lead_visible, show_distance_bars, out=None, msg_1b5=None):
  """out: CarState (blinkers, blind spots, speed, steering angle) for the lane-change picture; msg_1b5: the camera's lane positions."""
  values = msg_161.copy()

  if lat_active or paused:
    values.update(lateral_dash(lat_active, hud_control, out if out is not None else structs.CarState(), msg_1b5))
  else:
    values.update({"LKA_ICON": 3, "LFA_ICON": 0, "LANELINE_LEFT": LANELINE_GRAY, "LANELINE_RIGHT": LANELINE_GRAY, "CENTERLINE": 0,
                   "LCA_LEFT_ICON": LCA_HIDDEN, "LCA_RIGHT_ICON": LCA_HIDDEN})
  values["DAW_ICON"] = 0

  # suppress consider taking a break and its chime
  if values["ALERTS_2"] == 5:
    values.update({
      "ALERTS_2": 0,
      "SOUNDS_2": 0,
    })

  if long_enabled:
    override_blink = gas_override and frame % OVERRIDE_BLINK_FRAMES < OVERRIDE_BLINK_ON_FRAMES
    long_active = enabled and not gas_override
    hud_state = 2 if long_active else (3 if override_blink else 0) if enabled else 1 if main_cruise_enabled else 0
    spacing_state = 1 if long_active or (enabled and override_blink) else 0

    t_follow = T_FOLLOW.get(hud_control.leadDistanceBars, 1.45)
    stop_distance = float(np.interp(v_ego, STOP_DISTANCE_BP, STOP_DISTANCE_V))
    target_distance = min(TARGET_DISTANCE_MAX, max(0.0, t_follow * v_ego + stop_distance))

    if enabled and show_distance_bars and not values["ALERTS_3"]:
      values["ALERTS_3"] = 6 + hud_control.leadDistanceBars

    values.update({
      "SETSPEED": hud_state,
      "SETSPEED_HUD": hud_state,
      "SETSPEED_SPEED": set_speed if main_cruise_enabled else 255,
      "DISTANCE": hud_control.leadDistanceBars if enabled else 4 if main_cruise_enabled else 0,
      "DISTANCE_SPACING": spacing_state,
      "DISTANCE_LEAD": (2 if long_active else 1 if main_cruise_enabled else 0) if lead_visible else 0,
      "DISTANCE_CAR": 2 if enabled else 1 if main_cruise_enabled else 0,
      "SLA_ICON": 0,
      "NAV_ICON": 0,
      "TARGET": spacing_state,
      "TARGET_DISTANCE": target_distance if enabled else TARGET_DISTANCE_MAX,
    })

  return packer.make_can_msg("CCNC_0x161", CAN.ECAN, values)


def create_ccnc_0x162(packer, CAN, msg_162, enabled, long_enabled, main_cruise_enabled, gas_override, lead_visible, lead_distance):
  values = msg_162.copy()

  if long_enabled:
    values.update({
      "LEAD": (2 if enabled and not gas_override else 1 if main_cruise_enabled else 0) if lead_visible else 0,
      "LEAD_DISTANCE": min(TARGET_DISTANCE_MAX, max(0.0, lead_distance)) if lead_visible else TARGET_DISTANCE_MAX,
      "LEAD_LATERAL": 0,
    })

  return packer.make_can_msg("CCNC_0x162", CAN.ECAN, values)


def create_ccnc_buttons(packer, CAN, cruise_buttons_msg, main_btn):
  values = cruise_buttons_msg.copy()

  values.update({
    "ADAPTIVE_CRUISE_MAIN_BTN": 1 if main_btn else 0,
    "NORMAL_CRUISE_MAIN_BTN": 0,
  })

  dat = packer.make_can_msg("CRUISE_BUTTONS", CAN.CAM, values)[1]
  values["_CHECKSUM"] = cruise_buttons_checksum(dat[1:])

  return packer.make_can_msg("CRUISE_BUTTONS", CAN.CAM, values)


def create_ccnc_alt_buttons(packer, CAN, cruise_buttons_msg, cnt, btn):
  values = cruise_buttons_msg.copy()

  values.update({
    "COUNTER": cnt,
    "CRUISE_BUTTONS": btn,
  })

  return packer.make_can_msg("CRUISE_BUTTONS_ALT", CAN.ECAN, values)


class CcncCarController:
  def __init__(self, CP, CP_SP):
    self.ccnc_lfa_steering = bool(CP_SP.flags & HyundaiFlagsSP.CCNC and not (CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG))
    self.ccnc_lka_alt_buttons = bool(CP_SP.flags & HyundaiFlagsSP.CCNC and CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and
                                     CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS)
    self.lead_distance_filter = FirstOrderFilter(TARGET_DISTANCE_MAX, LEAD_FILTER_RC, 5 * DT_CTRL, initialized=False)
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0

  def create_alt_button_msgs(self, CS, CC):
    can_sends = []

    if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
      btn = None
      if CC.cruiseControl.cancel:
        if self.cancel_counter > CANCEL_BUTTON_DELAY_FRAMES:
          btn = Buttons.CANCEL
      elif CC.cruiseControl.resume:
        btn = Buttons.RES_ACCEL

      if btn is not None:
        for _ in range(10):
          can_sends.append(create_ccnc_alt_buttons(self.packer, self.CAN, CS.cruise_buttons_msg, CS.buttons_counter + 1, btn))
        self.last_button_frame = self.frame

    return can_sends

  @staticmethod
  def _get_set_speed(v_cruise_cluster, is_metric):
    unit_conv = 1 if is_metric else CV.KPH_TO_MPH
    max_speed = 145 if is_metric else 90
    min_speed = 40 if is_metric else 25
    set_speed = round(v_cruise_cluster * unit_conv)
    return min_speed if set_speed > max_speed else set_speed

  def update(self, CS, CC):
    can_sends = []

    if self.ccnc_lka_alt_buttons:
      return self.create_alt_button_msgs(CS, CC)

    if not self.ccnc_lfa_steering:
      return can_sends

    can_sends.append(create_ccnc_mdps(self.packer, self.CAN, CS.mdps))

    if self.CP.openpilotLongitudinalControl:
      if not CC.enabled or CC.hudControl.leadDistanceBars != self.lead_distance_bars_last:
        self.distance_bar_frame = self.frame
      self.lead_distance_bars_last = CC.hudControl.leadDistanceBars

      if self.frame % 2 == 0 and self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC:
        press_main = CS.cruise_info["MainMode_ACC"] and self.frame % (2 * MAIN_PRESS_FRAMES) < MAIN_PRESS_FRAMES
        can_sends.append(create_ccnc_buttons(self.packer, self.CAN, CS.cruise_buttons_msg, press_main))

      if self.frame % 5 == 0:
        if not self.lead_visible:
          self.lead_distance_filter.initialized = False
        elif self.lead_one.status:
          self.lead_distance_filter.update(self.lead_distance)

    if self.frame % 5 == 0:
      lat_active, paused, long_enabled_icons = self.dash_states(CS, CC)
      set_speed = self._get_set_speed(CS.out.vCruiseCluster, CS.is_metric)

      can_sends.append(create_ccnc_0x161(self.packer, self.CAN, self.frame, CS.msg_161, long_enabled_icons, lat_active, paused,
                                         self.CP.openpilotLongitudinalControl, CS.main_cruise_enabled, CC.cruiseControl.override,
                                         CC.hudControl, set_speed, CS.out.vEgo, self.lead_visible,
                                         self.frame - self.distance_bar_frame < 400, CS.out, CS.msg_1b5))
      can_sends.append(create_ccnc_0x162(self.packer, self.CAN, CS.msg_162, long_enabled_icons, self.CP.openpilotLongitudinalControl,
                                         CS.main_cruise_enabled, CC.cruiseControl.override, self.lead_visible,
                                         self.lead_distance_filter.x))

    return can_sends

  def dash_states(self, CS, CC):
    """(lat_active, lat_paused, cruise_shown) for the dash: what each axis is really doing, independent of the other. A steering fault is not
    a pause (wheel hidden, not grey), and a faulted ACC shows no engaged cruise."""
    lat_active = self.mads.lat_active if self.mads.enable_mads else CC.enabled
    paused = self.mads.enable_mads and self.mads.paused
    if not lat_active and (CS.out.steerFaultTemporary or CS.out.steerFaultPermanent):
      paused = False
    return lat_active, paused, bool(CC.enabled and not CS.out.accFaulted)
