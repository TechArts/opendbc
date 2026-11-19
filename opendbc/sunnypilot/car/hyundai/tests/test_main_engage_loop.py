"""
Controller -> simulated camera -> panda safety, closed loop for MAIN_BUTTON_ENGAGE: the driver's real MAIN press has to reach the camera through
the controller (the panda blocks the physical 0x1CF), the camera answers with its MainMode_ACC, and the panda and carstate engage from the pair.
"""

import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.hyundai.ccnc import CcncCarController

DBC_NAME = "hyundai_canfd_generated"
MAIN_STEP = 20_000  # us per controller frame pair


def _defaults(msg):
  return dict(CANParser(DBC_NAME, [(msg, 0)], 0).vl[msg])


class TestMainEngageLoop(unittest.TestCase):
  def setUp(self):
    fingerprint = gen_empty_fingerprint()
    can = CanBus(None, fingerprint)
    fingerprint[can.ECAN] = {0x1CF: 8}
    fingerprint[can.CAM] = {0xCB: 24, 0x161: 32, 0x162: 32}
    self.cp = CarInterface.get_params(CAR.GENESIS_GV70_2ND_GEN, fingerprint, [], True, False, False)
    self.cp_sp = CarInterface.get_params_sp(self.cp, CAR.GENESIS_GV70_2ND_GEN, fingerprint, [], True, False, False)
    self.cc = CarController(DBC[self.cp.carFingerprint], self.cp, self.cp_sp)
    self.cs = CarState(self.cp, self.cp_sp)
    self.cs.out = structs.CarState(vEgo=20.0, vCruiseCluster=100.0)
    self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    self.cs.lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
    self.cs.cruise_info = {"COUNTER": 50, "MainMode_ACC": 0}
    self.cs.msg_161, self.cs.msg_162 = _defaults("CCNC_0x161"), _defaults("CCNC_0x162")
    self.cs.cruise_buttons_msg = _defaults("CRUISE_BUTTONS")

    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.cp_sp.safetyParam)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, self.cp.safetyConfigs[-1].safetyParam)
    self.safety.init_tests()
    self.safety.set_timer(0)
    self.packer = CANPackerSafety(DBC_NAME)
    self.packer_car = CANPacker(DBC_NAME)

    self.cam_main = False
    self.cam_seen_main_btn = False
    self.t = 0
    self.frame = 0
    self.engage_requests = 0
    self.sent_main_pulses = 0

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def _camera_sends_main(self):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 2, {"MainMode_ACC": int(self.cam_main)}))

  def _step(self, driver_main: bool):
    """One controller frame pair: the driver's physical frame on bus 0, the controller's frames, the camera's reaction."""
    self.t += MAIN_STEP
    self.safety.set_timer(self.t)
    self.cs.cruise_buttons_msg = {**_defaults("CRUISE_BUTTONS"), "ADAPTIVE_CRUISE_MAIN_BTN": int(driver_main)}
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("CRUISE_BUTTONS", 0, {"ADAPTIVE_CRUISE_MAIN_BTN": int(driver_main)}))

    for _ in range(2):
      self.cc.frame = self.frame
      self.frame += 1
      for addr, dat, bus in CcncCarController.update(self.cc, self.cs, structs.CarControl()):
        if (addr, bus) != (0x1CF, 2):
          continue
        # the frame passes the panda tx checks and the (simulated) camera sees it
        self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)))
        parser = CANParser(DBC_NAME, [("CRUISE_BUTTONS", 0)], 2)
        parser.update([(self.t, [(addr, dat, bus)])])
        main_btn = bool(parser.vl["CRUISE_BUTTONS"]["ADAPTIVE_CRUISE_MAIN_BTN"])
        self.sent_main_pulses += main_btn and not driver_main
        if main_btn and not self.cam_seen_main_btn:
          self.cam_main = not self.cam_main  # the camera toggles its main state on a press
          self._camera_sends_main()
        self.cam_seen_main_btn = main_btn

    # carstate side of the same frame: the camera's main and the driver's press
    self.engage_requests += self.cs.update_camera_main(self.cam_main, driver_main)

  def _hold_main(self, frames):
    for _ in range(frames):
      self._step(True)
    self._step(False)

  def test_driver_press_reaches_the_camera_and_engages(self):
    self._hold_main(10)
    self.assertTrue(self.cam_main)
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertEqual(self.engage_requests, 1)  # carstate and panda agree

  def test_second_press_switches_the_camera_off(self):
    self._hold_main(10)
    self._hold_main(10)
    self.assertFalse(self.cam_main)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_acc_main_on())

  def test_no_synthetic_main_pulses(self):
    # camera main on after a press, driver idle: nothing is sent that would toggle it off
    self._hold_main(10)
    for _ in range(300):
      self._step(False)
    self.assertEqual(self.sent_main_pulses, 0)
    self.assertTrue(self.cam_main)

  def test_other_buttons_are_the_drivers(self):
    self.cs.cruise_buttons_msg = {**_defaults("CRUISE_BUTTONS"), "CRUISE_BUTTONS": 4}
    msgs = CcncCarController.update(self.cc, self.cs, structs.CarControl())
    dat = next(m[1] for m in msgs if m[0] == 0x1CF)
    parser = CANParser(DBC_NAME, [("CRUISE_BUTTONS", 0)], 2)
    parser.update([(1, [(0x1CF, dat, 2)])])
    self.assertEqual(parser.vl["CRUISE_BUTTONS"]["CRUISE_BUTTONS"], 4)


if __name__ == "__main__":
  unittest.main()
