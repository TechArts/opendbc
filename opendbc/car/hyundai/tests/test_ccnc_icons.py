import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.sunnypilot.car.hyundai.carstate_ext import CarStateExt, LANE_POSITIONS_STALE_FRAMES
from opendbc.sunnypilot.car.hyundai.ccnc import CcncCarController, create_ccnc_0x161, create_ccnc_0x162, lane_line_positions
from opendbc.sunnypilot.car.hyundai.mads import MadsDataSP
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

DBC_NAME = "hyundai_canfd_generated"
CAR_MODEL = CAR.GENESIS_GV70_1ST_GEN

# CCNC_0x161 value table: LFA_ICON 0 hidden / 1 grey / 2 green
LFA_OFF, LFA_GREY, LFA_GREEN = 0, 1, 2


def _defaults(msg):
  return dict(CANParser(DBC_NAME, [(msg, 0)], 0).vl[msg])


def _ccnc_fingerprint():
  fingerprint = gen_empty_fingerprint()
  can = CanBus(None, fingerprint)
  fingerprint[can.ECAN] = {0x1CF: 8}
  fingerprint[can.CAM] = {0xCB: 24, 0x161: 32, 0x162: 32}
  return fingerprint


def _carcontroller():
  fingerprint = _ccnc_fingerprint()
  cp = CarInterface.get_params(CAR_MODEL, fingerprint, [], True, False, False)
  cp_sp = CarInterface.get_params_sp(cp, CAR_MODEL, fingerprint, [], True, False, False)
  assert cp_sp.flags & HyundaiFlagsSP.CCNC
  return CarController(DBC[cp.carFingerprint], cp, cp_sp), CarState(cp, cp_sp)


def _parse(msg, name):
  parser = CANParser(DBC_NAME, [(name, 0)], msg[2])
  parser.update([(0, [msg])])
  return parser.vl[name]


class TestDashStates(unittest.TestCase):
  """Per axis, what the dash shows is what the car really does; the axes do not touch each other."""

  def setUp(self):
    self.cc, self.cs = _carcontroller()
    self.cs.out = structs.CarState(vEgo=20.0, vCruiseCluster=100.0)
    self.cs.mdps = {}
    self.cs.cruise_info = {"MainMode_ACC": 1}
    self.cs.cruise_buttons_msg = _defaults("CRUISE_BUTTONS")
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
    return _parse(next(m for m in msgs if m[0] == 0x161), "CCNC_0x161")

  def test_steering_states(self):
    for kwargs, expected in (({}, LFA_GREY), ({"lat_active": True}, LFA_GREEN), ({"mads_enabled": False}, LFA_OFF)):
      with self.subTest(**kwargs):
        self.assertEqual(self._icons(**kwargs)["LFA_ICON"], expected)

  def test_steer_fault_hides_the_wheel_instead_of_a_grey_pause(self):
    self.assertEqual(self._icons(steer_fault=True)["LFA_ICON"], LFA_OFF)
    self.assertEqual(self._icons(lat_active=True, steer_fault=True)["LFA_ICON"], LFA_GREEN)

  def test_cruise_states(self):
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


def _ccnc_0x161(lat_active=False, paused=False, msg_161=None, out=None, msg_1b5=None, hud=None):
  """The 0x161 frame create_ccnc_0x161 builds (openpilot long, main cruise on, not engaged)."""
  can = CanBus(None, gen_empty_fingerprint())
  hud = hud or structs.CarControl.HUDControl(leadDistanceBars=2, leftLaneVisible=True, rightLaneVisible=True)
  msg = create_ccnc_0x161(CANPacker(DBC_NAME), can, 5, msg_161 if msg_161 is not None else _defaults("CCNC_0x161"), False, lat_active, paused,
                          True, True, False, hud, 62, 20.0, False, False, out, msg_1b5)
  return _parse(msg, "CCNC_0x161")


class TestCameraFaults(unittest.TestCase):
  """The camera's FAULT_* (0x162) and its alert (0x161) reach the dash whatever openpilot does."""
  FAULTS = {"FAULT_SCC": 1, "FAULT_LFA": 2, "FAULT_LSS": 1, "FAULT_FCA": 1}

  def test_faults_pass_through(self):
    can = CanBus(None, gen_empty_fingerprint())
    for enabled in (False, True):
      for lead_visible in (False, True):
        msg = create_ccnc_0x162(CANPacker(DBC_NAME), can, dict(_defaults("CCNC_0x162"), **self.FAULTS), enabled, True, True, False,
                                lead_visible, 30.0)
        out = _parse(msg, "CCNC_0x162")
        self.assertEqual({k: out[k] for k in self.FAULTS}, self.FAULTS, (enabled, lead_visible))

  def test_camera_alert_passes_through(self):
    for kw in ({}, {"lat_active": True}, {"paused": True}):
      out = _ccnc_0x161(msg_161=dict(_defaults("CCNC_0x161"), ALERTS_1=3), out=structs.CarState(vEgo=25.0), **kw)
      self.assertEqual(out["ALERTS_1"], 3, kw)


class TestLaneChangeDash(unittest.TestCase):
  """While openpilot steers: one green wheel + center line, white lane lines and lane-change icons; a blinker turns the lines green and the
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

  def test_lane_positions_stay_inside_the_picture(self):
    for left, right in ((0.0, 3.5), (-3.6, 0.0), (-4.0, 4.0)):
      with self.subTest(left=left, right=right):
        lft, rgt = lane_line_positions(dict(self.LANES, Info_LftLnPosVal=left, Info_RtLnPosVal=right))
        self.assertTrue(0 <= lft <= 30 and 0 <= rgt <= 30 and lft + rgt == 30)

  def test_paused_is_a_grey_wheel_with_plain_lines(self):
    out = _ccnc_0x161(paused=True, out=structs.CarState(vEgo=25.0, leftBlinker=True))
    self.assertEqual((out["LFA_ICON"], out["CENTERLINE"], out["LANELINE_LEFT"], out["LCA_LEFT_ICON"], out["LCA_LEFT_ARROW"]), (1, 0, 2, 0, 0))

  def test_off_hides_the_lateral_picture(self):
    out = _ccnc_0x161(out=structs.CarState(vEgo=25.0, leftBlinker=True))
    self.assertEqual((out["LFA_ICON"], out["LKA_ICON"], out["CENTERLINE"], out["LANELINE_LEFT"], out["LCA_LEFT_ICON"]), (0, 3, 0, 0, 0))

  def test_lane_departure_line_is_orange(self):
    hud = structs.CarControl.HUDControl(leftLaneVisible=True, rightLaneVisible=True, rightLaneDepart=True)
    out = _ccnc_0x161(lat_active=True, out=structs.CarState(vEgo=25.0), hud=hud)
    self.assertEqual((out["LANELINE_LEFT"], out["LANELINE_RIGHT"]), (2, 4))

  def test_hda_icon_is_the_cameras(self):
    cam = _defaults("CCNC_0x161")
    cam["HDA_ICON"] = 0
    self.assertEqual(_ccnc_0x161(lat_active=True, msg_161=cam, out=structs.CarState(vEgo=25.0))["HDA_ICON"], 0)


class FakeParser:
  """Stands in for the CANParsers: the messages CarStateExt.update_canfd_ext copies, and FR_CMR_03_50ms with a receive time."""
  def __init__(self):
    self.vl = {"TCS": {"aBasis": 0.0}, "MDPS": {}, "CRUISE_BUTTONS": {}, "CCNC_0x161": {}, "CCNC_0x162": {}, "SCC_CONTROL": {},
               "FR_CMR_03_50ms": dict(TestLaneChangeDash.LANES)}
    self.ts_nanos = {"FR_CMR_03_50ms": {"Info_LftLnPosVal": 0}}

  def send(self, t):
    self.ts_nanos["FR_CMR_03_50ms"]["Info_LftLnPosVal"] = t


class TestLanePositionsFreshness(unittest.TestCase):
  """The camera's lane positions are used only while FR_CMR_03_50ms keeps arriving."""

  def _ext(self):
    fingerprint = _ccnc_fingerprint()
    cp = CarInterface.get_params(CAR_MODEL, fingerprint, [], True, False, False)
    cp_sp = CarInterface.get_params_sp(cp, CAR_MODEL, fingerprint, [], True, False, False)
    return CarStateExt(cp, cp_sp)

  def _update(self, ext, parser):
    ext.update_canfd_ext(structs.CarState(), structs.CarStateSP(), {Bus.pt: parser, Bus.cam: parser}, 1.0)

  def test_never_received_is_empty(self):
    ext, cam = self._ext(), FakeParser()
    self._update(ext, cam)
    self.assertEqual(ext.msg_1b5, {})

  def test_stale_after_the_window(self):
    ext, cam = self._ext(), FakeParser()
    cam.send(1)
    self._update(ext, cam)
    self.assertEqual(ext.msg_1b5["Info_LftLnPosVal"], -1.7)
    for _ in range(LANE_POSITIONS_STALE_FRAMES):
      self._update(ext, cam)
    self.assertNotEqual(ext.msg_1b5, {})
    self._update(ext, cam)
    self.assertEqual(ext.msg_1b5, {})
    cam.send(2)
    self._update(ext, cam)
    self.assertNotEqual(ext.msg_1b5, {})


if __name__ == "__main__":
  unittest.main()
