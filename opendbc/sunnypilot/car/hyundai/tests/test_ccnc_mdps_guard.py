import unittest
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.ccnc import CcncCarController
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


class FakePacker:
  def make_can_msg(self, name, bus, values):
    return name, dict(values), bus


class FakeCar(CcncCarController):
  def __init__(self):
    CP = structs.CarParams()
    CP.flags = int(HyundaiFlags.CANFD | HyundaiFlags.CANFD_CAMERA_SCC)
    CP_SP = structs.CarParamsSP()
    CP_SP.flags = int(HyundaiFlagsSP.CCNC | HyundaiFlagsSP.CANFD_LFA_ALT_STEER)
    CcncCarController.__init__(self, CP, CP_SP)
    self.CP = CP
    self.packer = FakePacker()
    self.CAN = SimpleNamespace(CAM=2, ECAN=0)
    self.frame = 1


class TestCcncMdpsGuard(unittest.TestCase):
  def _mdps_sends(self, mdps):
    CS = SimpleNamespace(mdps=mdps, lfa_alt={}, msg_161={}, msg_162={})
    sends = FakeCar().update(CS, structs.CarControl())
    return [s for s in sends if s[0] == "MDPS"]

  def test_no_copy_before_a_real_mdps_frame(self):
    self.assertEqual(self._mdps_sends({}), [])

  def test_copy_once_received(self):
    sends = self._mdps_sends({"MDPS_EstStrAnglVal": 1.0, "COUNTER": 3})
    self.assertEqual(len(sends), 1)
    self.assertNotIn("COUNTER", sends[0][1])


if __name__ == "__main__":
  unittest.main()
