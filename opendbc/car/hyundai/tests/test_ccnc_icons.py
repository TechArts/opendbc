import unittest

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.sunnypilot.car.hyundai.ccnc import CcncCarController
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


if __name__ == "__main__":
  unittest.main()
