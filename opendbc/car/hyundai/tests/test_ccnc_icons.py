import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.sunnypilot.car.hyundai.ccnc import CcncCarController, create_ccnc_0x161, create_ccnc_0x162, lane_line_positions
from opendbc.car.hyundai.handback import CameraHandback
from opendbc.sunnypilot.car.hyundai.mads import MadsDataSP

DBC_NAME = "hyundai_canfd_generated"

# CCNC_0x161 value tables: LFA_ICON 0 hidden / 1 grey / 2 green; SETSPEED / SETSPEED_HUD 0 hidden / 1 grey standby / 2 green
LFA_OFF, LFA_GREY, LFA_GREEN = 0, 1, 2


def _defaults(msg):
  return dict(CANParser(DBC_NAME, [(msg, 0)], 0).vl[msg])


def _carcontroller():
  fingerprint = gen_empty_fingerprint()
  can = CanBus(None, fingerprint)
  fingerprint[can.ECAN] = {0x1CF: 8}
  fingerprint[can.CAM] = {0xCB: 24, 0x161: 32, 0x162: 32}
  cp = CarInterface.get_params(CAR.GENESIS_GV70_2ND_GEN, fingerprint, [], True, False, False)
  cp_sp = CarInterface.get_params_sp(cp, CAR.GENESIS_GV70_2ND_GEN, fingerprint, [], True, False, False)
  return CarController(DBC[cp.carFingerprint], cp, cp_sp), CarState(cp, cp_sp)


class TestDashStates(unittest.TestCase):
  """Per axis, what the dash shows is what the car really does; the axes do not touch each other."""

  def setUp(self):
    self.cc, self.cs = _carcontroller()
    self.cs.out = structs.CarState(vEgo=20.0, vCruiseCluster=100.0)
    self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    self.cs.lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
    self.cs.cruise_info = {"COUNTER": 50, "MainMode_ACC": 1}
    self.cs.msg_161, self.cs.msg_162 = _defaults("CCNC_0x161"), _defaults("CCNC_0x162")
    self.cs.main_cruise_enabled = True
    self.cc.frame = 5  # icon cadence

  def _icons(self, enabled=False, lat_active=False, mads_enabled=True, steer_fault=False, acc_fault=False, override=False, main=True):
    cc = structs.CarControl(enabled=enabled, latActive=lat_active)
    cc.cruiseControl.override = override
    self.cc.mads = MadsDataSP(mads_enabled, lat_active, False, mads_enabled and not lat_active)
    self.cs.out.steerFaultTemporary = steer_fault
    self.cs.out.accFaulted = acc_fault
    self.cs.main_cruise_enabled = main
    msgs = CcncCarController.update(self.cc, self.cs, cc)
    parser = CANParser(DBC_NAME, [("CCNC_0x161", 0)], 0)
    dat = next(m for m in msgs if m[0] == 0x161)
    parser.update([(0, [dat])])
    return parser.vl["CCNC_0x161"]

  def test_steering_states(self):
    for kwargs, expected in (({}, LFA_GREY), ({"lat_active": True}, LFA_GREEN), ({"mads_enabled": False}, LFA_OFF)):
      with self.subTest(**kwargs):
        self.assertEqual(self._icons(**kwargs)["LFA_ICON"], expected)

  def test_steer_fault_hides_the_wheel_instead_of_a_grey_pause(self):
    self.assertEqual(self._icons(steer_fault=True)["LFA_ICON"], LFA_OFF)
    self.assertEqual(self._icons(lat_active=True, steer_fault=True)["LFA_ICON"], LFA_GREEN)

  def test_cruise_states(self):
    # (kwargs): SETSPEED_HUD
    for kwargs, expected in (({}, 1), ({"enabled": True}, 2), ({"enabled": True, "override": True}, 3), ({"main": False}, 0)):
      with self.subTest(**kwargs):
        self.assertEqual(self._icons(**kwargs)["SETSPEED_HUD"], expected)

  def test_acc_fault_hides_cruise_only(self):
    out = self._icons(enabled=True, lat_active=True, acc_fault=True)
    self.assertEqual(out["SETSPEED_HUD"], 1)  # standby: not engaged
    self.assertEqual(out["DISTANCE_CAR"], 1)
    self.assertEqual(out["LFA_ICON"], LFA_GREEN)  # steering untouched

  def test_axes_are_independent(self):
    for lat_active in (False, True):
      for enabled in (False, True):
        out = self._icons(enabled=enabled, lat_active=lat_active)
        self.assertEqual(out["LFA_ICON"], LFA_GREEN if lat_active else LFA_GREY)
        self.assertEqual(out["SETSPEED_HUD"], 2 if enabled else 1)


def _ccnc_0x161(lat_active=False, paused=False, passthrough_lat=False, camera_lfa_on=False, msg_161=None, out=None, msg_1b5=None, hud=None):
  """The 0x161 frame create_ccnc_0x161 builds (openpilot long, main cruise on, not engaged)."""
  fingerprint = gen_empty_fingerprint()
  can = CanBus(None, fingerprint)
  hud = hud or structs.CarControl.HUDControl(leadDistanceBars=2, leftLaneVisible=True, rightLaneVisible=True)
  msg = create_ccnc_0x161(CANPacker(DBC_NAME), can, 5, msg_161 if msg_161 is not None else _defaults("CCNC_0x161"), False, lat_active, paused, True,
                          True, False, hud, 62, 20.0, False, False, None, passthrough_lat, camera_lfa_on, out, msg_1b5)
  parser = CANParser(DBC_NAME, [("CCNC_0x161", 0)], msg[2])
  parser.update([(0, [msg])])
  return parser.vl["CCNC_0x161"]

class TestCameraFaults(unittest.TestCase):
  """F4: the camera's FAULT_* (0x162) and its alert (0x161) reach the dash whatever openpilot does, so a real fault is never hidden by our copies."""
  FAULTS = {"FAULT_SCC": 1, "FAULT_LFA": 2, "FAULT_LSS": 1, "FAULT_FCA": 1}

  def _162(self, enabled, lead_visible):
    can = CanBus(None, gen_empty_fingerprint())
    msg = create_ccnc_0x162(CANPacker(DBC_NAME), can, dict(_defaults("CCNC_0x162"), **self.FAULTS), enabled, True, True, False, lead_visible, 30.0)
    parser = CANParser(DBC_NAME, [("CCNC_0x162", 0)], msg[2])
    parser.update([(0, [msg])])
    return parser.vl["CCNC_0x162"]

  def test_faults_pass_through(self):
    for enabled in (False, True):
      for lead_visible in (False, True):
        out = self._162(enabled, lead_visible)
        self.assertEqual({k: out[k] for k in self.FAULTS}, self.FAULTS, (enabled, lead_visible))

  def test_camera_alert_passes_through(self):
    for kw in ({}, {"lat_active": True}, {"paused": True}, {"passthrough_lat": True}):
      out = _ccnc_0x161(msg_161=dict(_defaults("CCNC_0x161"), ALERTS_1=3), out=structs.CarState(vEgo=25.0), **kw)
      self.assertEqual(out["ALERTS_1"], 3, kw)


class TestLaneChangeDash(unittest.TestCase):
  """While openpilot steers: one green wheel + centre line, white lane lines and lane-change icons; a blinker turns the lines green and the
  blinker side's icon green with its arrow; a car in the blind spot turns that side grey; nothing for lane changes below 20 mph."""
  LANES = {"Info_LftLnPosVal": -1.7, "Info_RtLnPosVal": 1.7, "Info_LftLnQualSta": 3, "Info_RtLnQualSta": 3}

  def _out(self, v=25.0, **cs):
    return _ccnc_0x161(lat_active=True, out=structs.CarState(vEgo=v, **cs), msg_1b5=dict(self.LANES))

  def test_lane_keeping_shows_one_green_wheel_and_white_lines(self):
    out = self._out()
    self.assertEqual((out["LFA_ICON"], out["LKA_ICON"], out["CENTERLINE"]), (2, 0, 1))
    self.assertEqual((out["LANELINE_LEFT"], out["LANELINE_RIGHT"]), (2, 2))
    self.assertEqual((out["LCA_LEFT_ICON"], out["LCA_RIGHT_ICON"], out["LCA_LEFT_ARROW"], out["LCA_RIGHT_ARROW"]), (4, 4, 0, 0))

  def test_blinker_side_turns_green_with_its_arrow(self):
    out = self._out(leftBlinker=True)
    self.assertEqual((out["LANELINE_LEFT"], out["LANELINE_RIGHT"]), (6, 6))
    self.assertEqual((out["LCA_LEFT_ICON"], out["LCA_LEFT_ARROW"], out["LCA_RIGHT_ICON"], out["LCA_RIGHT_ARROW"]), (2, 2, 4, 0))

  def test_lane_lines_slide_with_the_camera_lanes_during_a_change(self):
    msg_1b5 = dict(self.LANES, Info_LftLnPosVal=-0.4, Info_RtLnPosVal=3.0)  # the car is near the left line, crossing it
    out = _ccnc_0x161(lat_active=True, out=structs.CarState(vEgo=25.0, leftBlinker=True), msg_1b5=msg_1b5)
    self.assertLess(out["LANELINE_LEFT_POSITION"], 15)
    self.assertEqual(out["LANELINE_LEFT_POSITION"] + out["LANELINE_RIGHT_POSITION"], 30)

  def test_blind_spot_side_is_grey(self):
    out = self._out(rightBlinker=True, rightBlindspot=True)
    self.assertEqual((out["LCA_RIGHT_ICON"], out["LCA_LEFT_ICON"]), (1, 4))

  def test_no_lane_change_picture_below_20_mph(self):
    out = self._out(v=5.0, leftBlinker=True)
    self.assertEqual((out["LCA_LEFT_ICON"], out["LCA_RIGHT_ICON"], out["LCA_LEFT_ARROW"]), (0, 0, 0))
    self.assertEqual(out["LANELINE_LEFT"], 2)  # white: no lane change, positions stay the camera's

  def test_lane_positions_stay_inside_the_picture(self):
    for left, right in ((0.0, 3.5), (-3.6, 0.0), (-4.0, 4.0)):
      with self.subTest(left=left, right=right):
        l, r = lane_line_positions(dict(self.LANES, Info_LftLnPosVal=left, Info_RtLnPosVal=right))
        self.assertTrue(0 <= l <= 30 and 0 <= r <= 30 and l + r == 30)

  def test_paused_is_a_grey_wheel_with_plain_lines(self):
    out = _ccnc_0x161(paused=True, out=structs.CarState(vEgo=25.0, leftBlinker=True))
    self.assertEqual((out["LFA_ICON"], out["CENTERLINE"], out["LANELINE_LEFT"], out["LCA_LEFT_ICON"], out["LCA_LEFT_ARROW"]), (1, 0, 2, 0, 0))

  def test_lane_departure_line_is_orange(self):
    hud = structs.CarControl.HUDControl(leftLaneVisible=True, rightLaneVisible=True, rightLaneDepart=True)
    out = _ccnc_0x161(lat_active=True, out=structs.CarState(vEgo=25.0), hud=hud)
    self.assertEqual((out["LANELINE_LEFT"], out["LANELINE_RIGHT"]), (2, 4))

  def test_hda_icon_is_the_cameras(self):
    cam = _defaults("CCNC_0x161")
    cam["HDA_ICON"] = 0
    self.assertEqual(_ccnc_0x161(lat_active=True, msg_161=cam, out=structs.CarState(vEgo=25.0))["HDA_ICON"], 0)


class TestCameraLfaIcon(unittest.TestCase):
  def test_blocked_camera_lfa_shows_grey_never_green(self):
    # openpilot steering is off but the car's own LFA is on (it cannot steer: the panda blocks its lane centering): grey wheel, no centre line
    out = _ccnc_0x161(camera_lfa_on=True)
    self.assertEqual((out["LFA_ICON"], out["CENTERLINE"]), (LFA_GREY, 0))
    out = _ccnc_0x161(camera_lfa_on=False)
    self.assertEqual((out["LFA_ICON"], out["CENTERLINE"]), (LFA_OFF, 0))

  def test_openpilot_state_wins_when_it_steers_or_pauses(self):
    self.assertEqual(_ccnc_0x161(paused=True, camera_lfa_on=True)["LFA_ICON"], LFA_GREY)
    self.assertEqual(_ccnc_0x161(lat_active=True, camera_lfa_on=True)["LFA_ICON"], LFA_GREEN)

  def test_lka_nudge_keeps_the_cameras_own_fields(self):
    # the camera's lane-keeping nudge steers (no frame of ours): its dash fields, LKA icon and warning included, pass through as received
    cam = _defaults("CCNC_0x161")
    cam.update({"LFA_ICON": 0, "LKA_ICON": 4, "LANELINE_LEFT": 4})
    out = _ccnc_0x161(passthrough_lat=True, camera_lfa_on=True, msg_161=cam)
    self.assertEqual((out["LFA_ICON"], out["LKA_ICON"], out["LANELINE_LEFT"]), (0, 4, 4))


class TestPassthrough(unittest.TestCase):
  """Lateral only: the cruise axis is never handed back (there is no longitudinal passthrough)."""

  def _camera(self):
    m161 = _defaults("CCNC_0x161")
    m161.update({"LFA_ICON": 2, "LKA_ICON": 4, "CENTERLINE": 1, "LANELINE_LEFT": 6, "LANELINE_RIGHT": 6, "LCA_LEFT_ICON": 2, "SETSPEED": 2,
                 "SETSPEED_HUD": 2, "SETSPEED_SPEED": 33, "DISTANCE": 3})
    return m161

  def test_camera_lateral_kept(self):
    out = _ccnc_0x161(passthrough_lat=True, msg_161=self._camera())
    self.assertEqual((out["LFA_ICON"], out["LKA_ICON"], out["CENTERLINE"], out["LANELINE_LEFT"], out["LANELINE_RIGHT"], out["LCA_LEFT_ICON"]),
                     (2, 4, 1, 6, 6, 2))
    self.assertEqual((out["SETSPEED_HUD"], out["SETSPEED"], out["DISTANCE"]), (1, 1, 4))  # cruise is still ours (main on, standby)

  def test_lateral_is_ours_without_passthrough(self):
    out = _ccnc_0x161(msg_161=self._camera())
    self.assertEqual((out["LFA_ICON"], out["LKA_ICON"], out["CENTERLINE"], out["LANELINE_LEFT"], out["LCA_LEFT_ICON"]), (0, 3, 0, 0, 0))

  def test_fault_passthrough_kept(self):
    cam = self._camera()
    cam.update({"ALERTS_1": 3, "ALERTS_2": 7})
    out = _ccnc_0x161(passthrough_lat=True, msg_161=cam)
    self.assertEqual((out["ALERTS_1"], out["ALERTS_2"]), (3, 7))


class TestHandbackDash(unittest.TestCase):
  """CcncCarController.update: the dash follows the hand-back state (the camera's nudge steers, or the car's blocked LFA is on)."""

  def setUp(self):
    self.cc, self.cs = _carcontroller()
    self.cs.out = structs.CarState(vEgo=20.0, vCruiseCluster=100.0)
    self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    self.cs.lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
    self.cs.cruise_info = {"COUNTER": 50, "MainMode_ACC": 1}
    cam = _defaults("CCNC_0x161")
    cam.update({"LFA_ICON": 0, "LKA_ICON": 4, "LANELINE_LEFT": 4})
    self.cs.msg_161, self.cs.msg_162 = cam, _defaults("CCNC_0x162")
    self.cs.main_cruise_enabled = True
    self.cc.frame = 5
    self.cc.mads = MadsDataSP(False, False, False, False)
    self.cc.handback = CameraHandback()

  def _out(self, send_lat, lfa_on=False):
    self.cc.send_lat = send_lat
    self.cc.handback.lka_window.lfa_on = lfa_on
    msgs = CcncCarController.update(self.cc, self.cs, structs.CarControl())
    parser = CANParser(DBC_NAME, [("CCNC_0x161", 0)], 0)
    parser.update([(0, [next(m for m in msgs if m[0] == 0x161)])])
    return parser.vl["CCNC_0x161"], {m[0] for m in msgs}

  def test_camera_nudge_passes_its_dash_fields_and_the_mdps_copy_stops(self):
    out, sent = self._out(send_lat=False)
    self.assertEqual((out["LFA_ICON"], out["LKA_ICON"], out["LANELINE_LEFT"]), (0, 4, 4))
    self.assertNotIn(0xEA, sent)

  def test_openpilot_sending_lateral_owns_the_fields_and_the_mdps_copy(self):
    out, sent = self._out(send_lat=True)
    self.assertEqual((out["LFA_ICON"], out["LKA_ICON"], out["LANELINE_LEFT"]), (0, 3, 0))
    self.assertIn(0xEA, sent)

  def test_blocked_cars_lfa_is_grey_while_openpilot_is_off(self):
    self.assertEqual(self._out(send_lat=True, lfa_on=True)[0]["LFA_ICON"], LFA_GREY)
    self.assertEqual(self._out(send_lat=True, lfa_on=False)[0]["LFA_ICON"], LFA_OFF)

  def test_cruise_axis_is_not_handed_back(self):
    self.cs.msg_161.update({"SETSPEED": 2, "SETSPEED_HUD": 2, "SETSPEED_SPEED": 33})
    out, _ = self._out(send_lat=False)  # the camera's nudge steers, the cruise icon is still openpilot's: main on, standby
    self.assertEqual((out["SETSPEED"], out["SETSPEED_HUD"]), (1, 1))


class TestMainForward(unittest.TestCase):
  """The camera's CRUISE_BUTTONS copy (main-button engage): a short MAIN (ADAPTIVE_CRUISE_MAIN_BTN pulse) passes unchanged; the car's long MAIN
  (NORMAL_CRUISE_MAIN_BTN, openpilot's speed-limit follow) never reaches the camera, which gets a short MAIN at its start instead, so it turns its
  cruise main on as for a short press and never enters its own limiter (route 94: a 1.47 s long MAIN started Manual Speed Limit Assist)."""
  setUp = TestHandbackDash.setUp

  def _camera_main(self, signal, held_frames, total_frames):
    self.cc.send_lat = True
    buttons = _defaults("CRUISE_BUTTONS")
    seen = []
    for i in range(total_frames):
      self.cc.frame = i
      self.cs.cruise_buttons_msg = dict(buttons, **{signal: 1 if i < held_frames else 0})
      msgs = CcncCarController.update(self.cc, self.cs, structs.CarControl())
      for m in msgs:
        if m[0] == 0x1CF and m[2] == self.cc.CAN.CAM:
          parser = CANParser(DBC_NAME, [("CRUISE_BUTTONS", 0)], m[2])
          parser.update([(0, [m])])
          vl = parser.vl["CRUISE_BUTTONS"]
          seen.append((i, vl["ADAPTIVE_CRUISE_MAIN_BTN"], vl["NORMAL_CRUISE_MAIN_BTN"]))
    return [i for i, a, _ in seen if a], [i for i, _, n in seen if n]

  def test_short_main_passes_unchanged(self):
    self.assertEqual(self._camera_main("ADAPTIVE_CRUISE_MAIN_BTN", 12, 40), ([0, 2, 4, 6, 8, 10], []))

  def test_long_main_reaches_the_camera_as_a_short_press(self):
    short, long = self._camera_main("NORMAL_CRUISE_MAIN_BTN", 147, 200)
    self.assertEqual(long, [])
    self.assertEqual(short, [0, 2, 4, 6, 8, 10])


if __name__ == "__main__":
  unittest.main()
