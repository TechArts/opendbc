"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import copy
from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.car.hyundai.values import HyundaiFlags, CAR
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP


CAMERA_STALE_FRAMES = 30  # 0.3 s of CarState updates: strictly inside the panda's 0.5 s (HYUNDAI_CANFD_HB_LKA_STALE_US / _HOLD_US)
NAV_SPEED_UNIT_MPH = 2  # HU_Navi_ISLW_PE Navi_ISLW_SpdUnit: only 2 = mph (US car) is observed; every other unit is invalid until seen
NAV_STALE_FRAMES = 300  # 3 s of CarState updates: the head unit's send rate is not measured, so be lenient (the limit only drives a map-source fallback)
NAV_SPEED_NONE = (0, 255)  # Navi_ISLW_SpdLimit sentinels: no limit
CAMERA_SIGN_STOP = 8  # CCNC_0x162 SIGNS: the camera sees a stop sign ahead (route logs: on 41-73 m before the stop, ~4-9 s)
SWRC_STALE_FRAMES = 50  # 0.5 s of CarState updates: SWRC_01 arrives at ~5 Hz and on every change (route 87), so 0.5 s is > 2 periods
CUSTOM_BUTTON_PRESSED = 1  # SWRC_01 Audio_CustomSw3Sta while the programmable wheel button is held (route 87: only 0 and 1 seen)
ISLA_SIGN_SCHOOL_CROSSING = 1  # FR_CMR_02_100ms ISLA_AddtnlSign "School Crossing" (routes 74-8d: 1.5 % of moving time; ISLA_SchoolZone never set)


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.aBasis = 0.0
    self.msg_161 = {}
    self.msg_162 = {}
    self.msg_1b5 = {}  # the camera's lane positions (FR_CMR_03_50ms): the dash's lane lines move with them during a lane change
    self.mdps = {}
    self.cruise_buttons_msg = {}
    self.lfa_alt = {}
    self.cam_lfa_icon = None  # the camera's own LFA_ICON (0x161) and LKA_SysIndReq (0x12A), None unless fresh (CAMERA_HANDBACK lane-keeping window)
    self.cam_lka_ind = None
    self._camera_seen: dict[str, tuple[int, int]] = {}  # camera message -> (receive time, updates since it last changed)

  def _camera_fresh(self, name: str, ts_nanos: int, new_only: bool = False, stale_frames: int = CAMERA_STALE_FRAMES) -> bool:
    """True while a camera message keeps arriving: its receive time moved within the last CAMERA_STALE_FRAMES updates (never received: False). The
    panda lets the lane-keeping window expire after 0.5 s without 0x161 / 0x12A, so ours must be over strictly before. new_only: True only in the update
    where the message arrived (an event, for a signal whose hold LkaWindow counts itself: a cached value must not restart that hold)."""
    last_ts, age = self._camera_seen.get(name, (0, stale_frames + 1))
    age = 0 if ts_nanos != last_ts else min(age + 1, stale_frames + 1)
    self._camera_seen[name] = (ts_nanos, age)
    return ts_nanos != 0 and (age == 0 if new_only else age <= stale_frames)

  def update_nav_speed_limit(self, cp) -> float:
    """The head unit's navigation speed limit in m/s while HU_Navi_ISLW_PE keeps arriving (0 = none, stale, a unit other than mph, or 0/255)."""
    if not (self.CP_SP.flags & HyundaiFlagsSP.CCNC and not self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG):
      return 0.0
    msg = cp.vl["HU_Navi_ISLW_PE"]
    if not self._camera_fresh("HU_Navi_ISLW_PE", cp.ts_nanos["HU_Navi_ISLW_PE"]["Navi_ISLW_SpdLimit"],
                              stale_frames=NAV_STALE_FRAMES):
      return 0.0
    limit = msg["Navi_ISLW_SpdLimit"]
    unit = msg["Navi_ISLW_SpdUnit"]
    if limit in NAV_SPEED_NONE:
      return 0.0
    if unit == NAV_SPEED_UNIT_MPH:
      return limit * CV.MPH_TO_MS
    return 0.0

  def update_custom_button(self, cp) -> bool:
    """The programmable steering-wheel button (SWRC_01 Audio_CustomSw3Sta) is held, while SWRC_01 keeps arriving (stale: released, so a lost
    message can never look like a long press)."""
    if not (self.CP_SP.flags & HyundaiFlagsSP.CCNC and not self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG):
      return False
    if not self._camera_fresh("SWRC_01_200ms", cp.ts_nanos["SWRC_01_200ms"]["Audio_CustomSw3Sta"], stale_frames=SWRC_STALE_FRAMES):
      return False
    return cp.vl["SWRC_01_200ms"]["Audio_CustomSw3Sta"] == CUSTOM_BUTTON_PRESSED

  def update_school_sign(self, cp, cp_cam) -> bool:
    """The camera reports a "School Crossing" sign (FR_CMR_02_100ms ISLA_AddtnlSign == 1) while the message keeps arriving: one input of the
    speed-limit vote's school-zone rule (openpilot speed_limit/vote.py)."""
    if not (self.CP.flags & HyundaiFlags.CANFD and self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE and
            self.CP_SP.flags & HyundaiFlagsSP.CCNC):
      return False
    bus = cp if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG else cp_cam
    if not self._camera_fresh("FR_CMR_02_100ms", bus.ts_nanos["FR_CMR_02_100ms"]["ISLA_AddtnlSign"]):
      return False
    return bus.vl["FR_CMR_02_100ms"]["ISLA_AddtnlSign"] == ISLA_SIGN_SCHOOL_CROSSING

  def update_speed_limit(self, cp, cp_cam) -> float:
    speed_limit = 0

    if self.CP.flags & HyundaiFlags.CANFD:
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        bus = cp if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG else cp_cam
        speed_limit = bus.vl["FR_CMR_02_100ms"]["ISLW_SpdCluMainDis"]
    else:
      nav, cam = 0, 0
      if self.CP_SP.flags & HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE:
        nav = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]
      if self.CP_SP.flags & HyundaiFlagsSP.HAS_LKAS12:
        cam = cp_cam.vl["LKAS12"]["CF_Lkas_TsrSpeed_Display_Clu"]

      speed_limit = cam if cam not in (0, 255) else nav

    if speed_limit in (0, 255):
      speed_limit = 0

    return speed_limit

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser], speed_conv: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS13"]["aBasis"]

    if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC:
      cruise_msg = "LABEL11" if self.CP.flags & HyundaiFlags.EV else \
                   "E_CRUISE_CONTROL" if self.CP.flags & HyundaiFlags.HYBRID else \
                   "EMS16"
      cruise_available_sig = "CC_React" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_M"
      cruise_enabled_sig = "CC_ACT" if self.CP.flags & HyundaiFlags.EV else "CRUISE_LAMP_S"
      cruise_speed_msg = "E_EMS11" if self.CP.flags & HyundaiFlags.EV else \
                         "ELECT_GEAR" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "EMS_H12" if self.CP.carFingerprint == CAR.HYUNDAI_VENUE_2021_NON_SCC else \
                         "LVR12"
      cruise_speed_sig = "Cruise_Limit_Target" if self.CP.flags & HyundaiFlags.EV else \
                         "SLC_SET_SPEED" if self.CP.flags & HyundaiFlags.HYBRID else \
                         "SLD_VS" if self.CP.carFingerprint == CAR.HYUNDAI_VENUE_2021_NON_SCC else \
                         "CF_Lvr_CruiseSet"
      ret.cruiseState.available = cp.vl[cruise_msg][cruise_available_sig] != 0
      ret.cruiseState.enabled = cp.vl[cruise_msg][cruise_enabled_sig] != 0
      ret.cruiseState.speed = cp.vl[cruise_speed_msg][cruise_speed_sig] * speed_conv
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False

      if not self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_NO_FCA:
        cp_cruise = cp if self.CP_SP.flags & HyundaiFlagsSP.NON_SCC_RADAR_FCA else cp_cam

        aeb_src = "FCA11"
        aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
        aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src]["FCA_CmdAct"] != 0
        ret.stockFcw = aeb_warning and not aeb_braking
        ret.stockAeb = aeb_warning and aeb_braking

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_conv

  def update_canfd_ext(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser],
                       speed_factor: float) -> None:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    self.aBasis = cp.vl["TCS"]["aBasis"]

    if self.CP_SP.flags & HyundaiFlagsSP.CCNC and not self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
      self.msg_161 = copy.copy(cp_cam.vl["CCNC_0x161"])
      self.msg_162 = copy.copy(cp_cam.vl["CCNC_0x162"])
      fresh_1b5 = self._camera_fresh("FR_CMR_03_50ms", cp_cam.ts_nanos["FR_CMR_03_50ms"]["FR_CMR_AlvCnt3Val"])
      self.msg_1b5 = copy.copy(cp_cam.vl["FR_CMR_03_50ms"]) if fresh_1b5 else {}  # stale or never sent: the dash keeps the camera's positions
      # never copy before the real MDPS has arrived: the copy is re-sent to the camera, which must not get default values
      if cp.ts_nanos["MDPS"]["COUNTER"]:
        self.mdps = copy.copy(cp.vl["MDPS"])
      self.cruise_info = copy.copy(cp_cam.vl["SCC_CONTROL"])
      self.cruise_buttons_msg = copy.copy(cp.vl["CRUISE_BUTTONS"])
      if self.CP_SP.flags & HyundaiFlagsSP.CANFD_LFA_ALT_STEER:
        self.lfa_alt = copy.copy(cp_cam.vl["LFA_ALT"])
        if self.CP_SP.safetyParam & HyundaiSafetyFlagsSP.CAMERA_HANDBACK:
          # the camera's lane-keeping signals, also while the panda blocks its 0xCB: LFA_ICON (0x161) and LKA_SysIndReq (0x12A, 3 = lane departure).
          # The parser keeps the last value of a message that stopped: a signal is only passed on while its message is fresh (see _camera_fresh)
          lka = cp_cam.vl["LFA"]  # registers the message in the parser
          self.cam_lka_ind = lka["LKA_SysIndReq"] if self._camera_fresh("LFA", cp_cam.ts_nanos["LFA"]["COUNTER"], new_only=True) else None
          self.cam_lfa_icon = self.msg_161["LFA_ICON"] if self._camera_fresh("CCNC_0x161", cp_cam.ts_nanos["CCNC_0x161"]["COUNTER"]) else None

    if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS and self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
      self.cruise_buttons_msg = copy.copy(cp.vl[self.cruise_btns_msg_canfd])

    ret_sp.speedLimit = self.update_speed_limit(cp, cp_cam) * speed_factor

    # the camera's traffic-sign flag, only while 0x162 keeps arriving (the parser keeps the last value of a stopped message)
    ret_sp.cameraStopSign = bool(self.CP_SP.flags & HyundaiFlagsSP.CCNC and not self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG and
                                 self._camera_fresh("CCNC_0x162", cp_cam.ts_nanos["CCNC_0x162"]["COUNTER"]) and
                                 self.msg_162["SIGNS"] == CAMERA_SIGN_STOP)
    ret_sp.navSpeedLimit = self.update_nav_speed_limit(cp)
    ret_sp.customButton = self.update_custom_button(cp)
    ret_sp.speedLimitSchoolSign = self.update_school_sign(cp, cp_cam)
    # GV70 CCNC: a long MAIN is its own bit, held until the release; a short MAIN is a ~0.11 s ADAPTIVE_CRUISE_MAIN_BTN pulse (routes 94-9f)
    ret_sp.mainCruiseLong = bool(self.cruise_buttons_msg.get("NORMAL_CRUISE_MAIN_BTN", 0))
