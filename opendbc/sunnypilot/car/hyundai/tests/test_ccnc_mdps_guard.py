"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import unittest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car.hyundai.ccnc import CcncCarController
from opendbc.sunnypilot.car.hyundai.lfa_alt import LFA_ALT_MSG
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

CCNC = (0x161, 0x162)
MDPS = 0xEA


def get_params(cam_msgs):
  fingerprint = gen_empty_fingerprint()
  fingerprint[0][0x1cf] = 8
  fingerprint[2].update(dict.fromkeys(cam_msgs, 8))
  CP = CarInterface.get_params(CAR.KIA_SPORTAGE_2026, fingerprint, [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.KIA_SPORTAGE_2026, fingerprint, [], False, False, False)
  return CP, CP_SP


class TestCcncMdpsGuard(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("hyundai_canfd_generated")

  def _car_state(self, cam_msgs):
    CP, CP_SP = get_params(cam_msgs)
    CS = CarInterface.CarState(CP, CP_SP)
    return CP, CP_SP, CS, CS.get_can_parsers(CP, CP_SP)

  def _update(self, CS, can_parsers, frames, t):
    for cp in can_parsers.values():
      cp.update([(t, frames)])
    CS.update(can_parsers)

  def _mdps_frame(self, CP):
    return self.packer.make_can_msg("MDPS", CanBus(CP).ECAN, {"MDPS_EstStrAnglVal": 1.5, "COUNTER": 3})

  def test_lfa_alt_no_mdps_before_a_real_frame(self):
    CP, CP_SP, CS, can_parsers = self._car_state((LFA_ALT_MSG, *CCNC))
    assert CP_SP.flags & HyundaiFlagsSP.CANFD_LFA_ALT_STEER
    self._update(CS, can_parsers, [], int(1e7))
    self.assertEqual(CS.mdps, {})

    self._update(CS, can_parsers, [self._mdps_frame(CP)], int(2e7))
    self.assertEqual(CS.mdps["MDPS_EstStrAnglVal"], 1.5)

  def test_other_ccnc_copies_from_the_start(self):
    # without LFA_ALT the panda blocks the real MDPS toward the camera, so the copy is sent from the first frame
    CP, CP_SP, CS, can_parsers = self._car_state(CCNC)
    assert CP_SP.flags & HyundaiFlagsSP.CCNC and not CP_SP.flags & HyundaiFlagsSP.CANFD_LFA_ALT_STEER
    self._update(CS, can_parsers, [], int(1e7))
    self.assertNotEqual(CS.mdps, {})

  def test_controller_sends_copy_only_with_mdps(self):
    CP, CP_SP = get_params((LFA_ALT_MSG, *CCNC))
    controller = CcncCarController(CP, CP_SP)
    controller.CP = CP
    controller.packer = self.packer
    controller.CAN = CanBus(CP)
    controller.frame = 1

    def mdps_sends(mdps):
      CS = type("CS", (), {"mdps": mdps, "lfa_alt": {}, "msg_161": {}, "msg_162": {}})()
      return [s for s in controller.update(CS, structs.CarControl()) if s[0] == MDPS]

    self.assertEqual(mdps_sends({}), [])
    self.assertEqual(len(mdps_sends({"MDPS_EstStrAnglVal": 1.0, "COUNTER": 3})), 1)


if __name__ == "__main__":
  unittest.main()
