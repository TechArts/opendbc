import unittest
from types import SimpleNamespace

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.carstate import CarState, MAIN_DESYNC_FRAMES
from opendbc.car.hyundai.hyundaicanfd import CanBus, ICON_OFF, ICON_PAUSED, ICON_ACTIVE
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.sunnypilot.car.hyundai.mads import MadsDataSP

DBC_NAME = "hyundai_canfd_generated"
ButtonType = structs.CarState.ButtonEvent.Type


class FakePacker:
  def make_can_msg(self, name, bus, values):
    return name, dict(values)


class FakeCAN:
  ECAN = 0


def _defaults(msg):
  return dict(CANParser(DBC_NAME, [(msg, 0)], 0).vl[msg])


def ccnc(lat=ICON_OFF, long=ICON_OFF, main=True, long_control=True, lead=False, bars=2, blinker=False, passthrough_lat=False, passthrough_long=False,
         msg_161=None, msg_162=None, **out_kw):
  hud = structs.CarControl.HUDControl(leadDistanceBars=bars, leadVisible=lead, leftLaneVisible=True, rightLaneVisible=True)
  out = structs.CarState(vEgo=20.0, vCruiseCluster=100.0, **out_kw)
  msgs = dict(hyundaicanfd.create_ccnc(FakePacker(), FakeCAN(), long_control, long, hud, blinker, False,
                                       msg_161 if msg_161 is not None else _defaults("CCNC_0x161"),
                                       msg_162 if msg_162 is not None else _defaults("CCNC_0x162"),
                                       _defaults("FR_CMR_03_50ms"), False, out, main, lat,
                                       passthrough_lat=passthrough_lat, passthrough_long=passthrough_long))
  return msgs["CCNC_0x161"], msgs["CCNC_0x162"]


class TestIconMapping(unittest.TestCase):
  def test_lateral(self):
    # state: (LFA_ICON, CENTERLINE, left lane line)  LFA_ICON 0 hidden / 1 gray / 2 green, lane line 0 gray / 2 white / 6 green
    for state, expected in ((ICON_OFF, (0, 0, 0)), (ICON_PAUSED, (1, 0, 2)), (ICON_ACTIVE, (2, 1, 2))):
      with self.subTest(state=state):
        m161, _ = ccnc(lat=state)
        self.assertEqual((m161["LFA_ICON"], m161["CENTERLINE"], m161["LANELINE_LEFT"]), expected)

  def test_lateral_green_lane_lines_only_when_active_with_blinker(self):
    self.assertEqual(ccnc(lat=ICON_ACTIVE, blinker=True)[0]["LANELINE_LEFT"], 6)
    self.assertEqual(ccnc(lat=ICON_PAUSED, blinker=True)[0]["LANELINE_LEFT"], 2)

  def test_longitudinal(self):
    # (state, main): SETSPEED, SETSPEED_HUD, DISTANCE, DISTANCE_SPACING, DISTANCE_CAR, DISTANCE_LEAD, LEAD
    keys = ("SETSPEED", "SETSPEED_HUD", "DISTANCE", "DISTANCE_SPACING", "DISTANCE_CAR", "DISTANCE_LEAD")
    table = {
      (ICON_ACTIVE, True): (2, 2, 2, 1, 2, 2, 2),
      (ICON_PAUSED, True): (1, 1, 2, 1, 1, 1, 1),  # gas override: gray
      (ICON_OFF, True): (1, 1, 0, 0, 0, 0, 1),     # main on, not engaged: gray standby, no gap bars
      (ICON_OFF, False): (1, 0, 0, 0, 0, 0, 0),    # main off: hidden
    }
    for (state, main), expected in table.items():
      with self.subTest(state=state, main=main):
        m161, m162 = ccnc(long=state, main=main, lead=True, bars=2)
        self.assertEqual(tuple(m161[k] for k in keys) + (m162["LEAD"],), expected)

  def test_gap_bars_follow_personality_only_while_engaged(self):
    for bars in (1, 2, 3):
      self.assertEqual(ccnc(long=ICON_ACTIVE, bars=bars)[0]["DISTANCE"], bars)
      self.assertEqual(ccnc(long=ICON_PAUSED, bars=bars)[0]["DISTANCE"], bars)
      self.assertEqual(ccnc(long=ICON_OFF, bars=bars)[0]["DISTANCE"], 0)

  def test_set_speed_number(self):
    self.assertEqual(ccnc(long=ICON_ACTIVE)[0]["SETSPEED_SPEED"], 62)  # 100 km/h in mph
    self.assertEqual(ccnc(long=ICON_ACTIVE, main=False)[0]["SETSPEED_SPEED"], 255)

  def test_axes_are_independent(self):
    for lat in (ICON_OFF, ICON_PAUSED, ICON_ACTIVE):
      for long in (ICON_OFF, ICON_PAUSED, ICON_ACTIVE):
        m161, _ = ccnc(lat=lat, long=long)
        self.assertEqual(m161["LFA_ICON"], lat)
        self.assertEqual(m161["SETSPEED_HUD"], 2 if long == ICON_ACTIVE else 1)


class TestPassthrough(unittest.TestCase):
  def _camera(self):
    m161 = _defaults("CCNC_0x161")
    m161.update({"LFA_ICON": 2, "CENTERLINE": 1, "LANELINE_LEFT": 6, "SETSPEED": 2, "SETSPEED_HUD": 2, "SETSPEED_SPEED": 33, "DISTANCE": 3,
                 "DISTANCE_CAR": 2, "DISTANCE_SPACING": 3, "DISTANCE_LEAD": 2})
    m162 = _defaults("CCNC_0x162")
    m162["LEAD"] = 4
    return m161, m162

  def test_camera_lateral_kept(self):
    m161, m162 = ccnc(passthrough_lat=True, msg_161=self._camera()[0], msg_162=self._camera()[1])
    self.assertEqual((m161["LFA_ICON"], m161["CENTERLINE"], m161["LANELINE_LEFT"]), (2, 1, 6))
    self.assertEqual(m161["SETSPEED_HUD"], 1)  # long is still ours

  def test_camera_cruise_kept(self):
    m161, m162 = ccnc(passthrough_long=True, msg_161=self._camera()[0], msg_162=self._camera()[1])
    self.assertEqual((m161["SETSPEED"], m161["SETSPEED_HUD"], m161["SETSPEED_SPEED"], m161["DISTANCE"], m161["DISTANCE_CAR"]), (2, 2, 33, 3, 2))
    self.assertEqual((m161["DISTANCE_SPACING"], m161["DISTANCE_LEAD"], m162["LEAD"]), (3, 2, 4))
    self.assertEqual(m161["LFA_ICON"], 0)  # lateral is still ours

  def test_fault_passthrough_kept(self):
    m161 = self._camera()[0]
    m161.update({"ALERTS_1": 3, "FAULT_SCC": 1})
    out, _ = ccnc(passthrough_lat=True, passthrough_long=True, msg_161=m161)
    self.assertEqual((out["ALERTS_1"], out["FAULT_SCC"]), (3, 1))


def _carcontroller():
  fingerprint = gen_empty_fingerprint()
  can = CanBus(None, fingerprint)
  fingerprint[can.ECAN] = {0x1CF: 8}
  fingerprint[can.CAM] = {0xCB: 24}
  cp = CarInterface.get_params(CAR.GENESIS_GV70_2ND_GEN, fingerprint, [], True, False, False)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.GENESIS_GV70_2ND_GEN)
  return cp, cp_sp, CarController(DBC[cp.carFingerprint], cp, cp_sp), CarState(cp, cp_sp)


class TestDashStates(unittest.TestCase):
  """Per-axis state from the real control state, independent of each other (MadsUnifiedEngagementMode=0: either axis alone)."""

  def setUp(self):
    self.cp, self.cp_sp, self.cc, self.cs = _carcontroller()
    self.cs.out = structs.CarState()
    self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    self.cs.cam_lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0}
    self.cs.msg_161, self.cs.msg_162, self.cs.msg_1b5 = (_defaults(m) for m in ("CCNC_0x161", "CCNC_0x162", "FR_CMR_03_50ms"))
    self.cs.main_cruise_enabled = True
    self.cc.tuning = SimpleNamespace(stopping=False, actual_accel=0.0, jerk_lower=0.5, jerk_upper=0.5)
    self.cc.frame = 5  # icon cadence

  def _states(self, enabled=False, lat_active=False, long_active=False, mads_enabled=False, steer_fault=False, acc_fault=False):
    cc = structs.CarControl(enabled=enabled, latActive=lat_active, longActive=long_active)
    self.cc.mads = MadsDataSP(True, lat_active, False, mads_enabled and not lat_active)
    self.cs.out.steerFaultTemporary = steer_fault
    self.cs.out.accFaulted = acc_fault
    return self.cc.dash_states(self.cs, cc)

  def test_nothing_engaged(self):
    self.assertEqual(self._states(), (ICON_OFF, ICON_OFF))

  def test_steering_only(self):
    self.assertEqual(self._states(lat_active=True, mads_enabled=True), (ICON_ACTIVE, ICON_OFF))

  def test_cruise_only(self):
    self.assertEqual(self._states(enabled=True, long_active=True), (ICON_OFF, ICON_ACTIVE))

  def test_both(self):
    self.assertEqual(self._states(enabled=True, lat_active=True, long_active=True, mads_enabled=True), (ICON_ACTIVE, ICON_ACTIVE))

  def test_lateral_pause_is_gray_and_does_not_touch_cruise(self):
    self.assertEqual(self._states(enabled=True, long_active=True, mads_enabled=True), (ICON_PAUSED, ICON_ACTIVE))

  def test_takeover_pause_is_gray_and_fault_is_off(self):
    self.cc.takeover.paused = True
    self.assertEqual(self._states(enabled=True, lat_active=True, long_active=True, mads_enabled=True), (ICON_PAUSED, ICON_ACTIVE))
    self.assertEqual(self._states(enabled=True, long_active=True, mads_enabled=True, steer_fault=True)[0], ICON_OFF)  # not latActive
    self.cc.takeover.paused = False
    self.assertEqual(self._states(enabled=True, lat_active=True, long_active=True, mads_enabled=True)[0], ICON_ACTIVE)

  def test_gas_override_is_gray_cruise_only(self):
    self.assertEqual(self._states(enabled=True, lat_active=True, mads_enabled=True), (ICON_ACTIVE, ICON_PAUSED))

  def test_steer_fault_turns_off_only_steering(self):
    self.assertEqual(self._states(enabled=True, long_active=True, mads_enabled=True, steer_fault=True), (ICON_OFF, ICON_ACTIVE))

  def test_acc_fault_turns_off_only_cruise(self):
    self.assertEqual(self._states(enabled=True, lat_active=True, mads_enabled=True, acc_fault=True), (ICON_ACTIVE, ICON_OFF))

  def test_error_disengaging_everything_turns_off_both(self):
    self.assertEqual(self._states(), (ICON_OFF, ICON_OFF))

  def _frame(self, enabled, lat_active, long_active, mads_enabled, steer_req=False):
    cc = structs.CarControl(enabled=enabled, latActive=lat_active, longActive=long_active)
    self._states(enabled, lat_active, long_active, mads_enabled)
    msgs = self.cc.create_canfd_msgs(steer_req, 0, 0.0, 0.0, False, structs.CarControl.HUDControl(), self.cs, cc)
    parser = CANParser(DBC_NAME, [("CCNC_0x161", 0)], 0)
    dat = next(m for m in msgs if m[0] == 0x161)
    parser.update([(0, [dat])])
    self.cc.frame += 5
    return parser.vl["CCNC_0x161"]

  def test_camera_icons_pass_through_until_openpilot_claims_an_axis(self):
    self.cs.msg_161.update({"LFA_ICON": 2, "SETSPEED_HUD": 2, "SETSPEED": 2, "SETSPEED_SPEED": 33})
    out = self._frame(False, False, False, False)  # nothing ours: stock LFA and stock cruise show
    self.assertEqual((out["LFA_ICON"], out["SETSPEED_HUD"], out["SETSPEED_SPEED"]), (2, 2, 33))

    self.cs.msg_161.update({"LFA_ICON": 2, "SETSPEED_HUD": 2})
    out = self._frame(True, True, False, True, steer_req=True)  # openpilot steers (active) and cruise engaged but overridden
    self.assertEqual((out["LFA_ICON"], out["SETSPEED_HUD"], out["SETSPEED"]), (2, 1, 1))

    self.cs.msg_161.update({"LFA_ICON": 2, "SETSPEED_HUD": 2})
    out = self._frame(False, False, False, False)  # we still own both (no driver action): off frames, not the camera's icons
    self.assertEqual((out["LFA_ICON"], out["SETSPEED_HUD"], out["DISTANCE"]), (0, 1, 0))


class TestMainDesync(unittest.TestCase):
  def setUp(self):
    _, _, _, self.cs = _carcontroller()
    self.cs.main_cruise_enabled = True
    self.cs.main_available = True

  def _run(self, camera_main, frames, pressed=False):
    for _ in range(frames):
      self.cs.update_main_desync(camera_main, pressed)

  def test_agreement(self):
    self._run(True, 5 * MAIN_DESYNC_FRAMES)
    self.assertFalse(self.cs.main_desync)

  def test_short_mismatch_after_press_ignored(self):
    self._run(False, MAIN_DESYNC_FRAMES)
    self.assertFalse(self.cs.main_desync)

  def test_persistent_mismatch_flagged_and_clears(self):
    self._run(False, MAIN_DESYNC_FRAMES + 1)
    self.assertTrue(self.cs.main_desync)
    self._run(True, 1)
    self.assertFalse(self.cs.main_desync)

  def test_unknown_camera_state_is_not_a_mismatch(self):
    self._run(None, 5 * MAIN_DESYNC_FRAMES)
    self.assertFalse(self.cs.main_desync)

  def test_set_res_blocked_on_desync_or_main_off_but_main_press_works(self):
    set_release = [structs.CarState.ButtonEvent(type=ButtonType.decelCruise, pressed=False)]
    self.assertTrue(self.cs.update_button_enable(set_release))
    self._run(False, MAIN_DESYNC_FRAMES + 1)
    self.assertFalse(self.cs.update_button_enable(set_release))
    self.cs.main_button_enable = True  # a main press engages at the current speed regardless
    self.assertTrue(self.cs.update_button_enable([]))
    self.cs.main_button_enable = False
    self._run(True, 1)
    self.cs.main_available = False
    self.assertFalse(self.cs.update_button_enable(set_release))


if __name__ == "__main__":
  unittest.main()
