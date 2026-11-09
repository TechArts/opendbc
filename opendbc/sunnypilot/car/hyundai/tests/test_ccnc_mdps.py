import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car.hyundai import hyundaicanfd

DBC = "hyundai_canfd_generated"


class FakeCAN:
  CAM = 2


class TestCcncMdpsCopy(unittest.TestCase):
  def _roundtrip(self, mdps):
    addr, dat, bus = hyundaicanfd.create_ccnc_mdps(CANPacker(DBC), FakeCAN, mdps)
    parser = CANParser(DBC, [("MDPS", 0)], bus)
    parser.update([(0, [(addr, dat, bus)])])
    return bus, parser.vl["MDPS"]

  def test_reports_aci_inactive_to_camera(self):
    # the real MDPS reports ACI Lv2 active while openpilot steers; the camera (not requesting) must see inactive
    bus, out = self._roundtrip({"MDPS_ADASAciActvSta_Lv2": 2, "MDPS_LkaToiActvSta": 1, "MDPS_LkaToiUnblSta": 0,
                                "MDPS_EstStrAnglVal": 12.5, "MDPS_ADAS_AciFltSig_Lv2": 4})
    self.assertEqual(bus, 2)
    self.assertEqual(out["MDPS_ADASAciActvSta_Lv2"], 1)
    self.assertEqual(out["MDPS_LkaToiActvSta"], 0)
    self.assertEqual(out["MDPS_LkaToiUnblSta"], 1)

  def test_passes_real_state_through(self):
    _, out = self._roundtrip({"MDPS_ADASAciActvSta_Lv2": 1, "MDPS_EstStrAnglVal": -42.0, "MDPS_ADAS_AciFltSig_Lv2": 2})
    self.assertAlmostEqual(out["MDPS_EstStrAnglVal"], -42.0, places=1)
    self.assertEqual(out["MDPS_ADAS_AciFltSig_Lv2"], 2)  # real faults stay visible to the camera


if __name__ == "__main__":
  unittest.main()
