import unittest
from types import SimpleNamespace

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.carstate_ext import CarStateExt, NAV_STALE_FRAMES
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


class FakeParser:
  """Stands in for the CANParser: one HU_Navi_ISLW_PE message with a receive time."""
  def __init__(self):
    self.vl = {"HU_Navi_ISLW_PE": {"Navi_ISLW_SpdLimit": 0, "Navi_ISLW_SpdUnit": 0}}
    self.ts_nanos = {"HU_Navi_ISLW_PE": {"Navi_ISLW_SpdLimit": 0}}
    self._t = 0

  def send(self, limit, unit):
    self._t += 10_000_000
    self.vl["HU_Navi_ISLW_PE"] = {"Navi_ISLW_SpdLimit": limit, "Navi_ISLW_SpdUnit": unit}
    self.ts_nanos["HU_Navi_ISLW_PE"]["Navi_ISLW_SpdLimit"] = self._t


def make(ccnc=True, lka_steer_msg=False):
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD | (HyundaiFlags.CANFD_LKA_STEER_MSG if lka_steer_msg else 0))
  CP_SP = SimpleNamespace(flags=HyundaiFlagsSP.CCNC if ccnc else 0)
  return CarStateExt(CP, CP_SP), FakeParser()


class TestNavSpeedLimit(unittest.TestCase):
  def test_mph(self):
    ext, cp = make()
    cp.send(35, 2)
    self.assertAlmostEqual(ext.update_nav_speed_limit(cp), 35 * CV.MPH_TO_MS)

  def test_kph(self):
    ext, cp = make()
    cp.send(50, 1)
    self.assertAlmostEqual(ext.update_nav_speed_limit(cp), 50 * CV.KPH_TO_MS)

  def test_sentinels_and_unknown_units(self):
    ext, cp = make()
    for limit, unit in ((0, 2), (255, 2), (255, 3), (35, 3), (35, 0)):
      cp.send(limit, unit)
      self.assertEqual(ext.update_nav_speed_limit(cp), 0.0, (limit, unit))

  def test_never_received(self):
    ext, cp = make()
    self.assertEqual(ext.update_nav_speed_limit(cp), 0.0)

  def test_stale_message(self):
    ext, cp = make()
    cp.send(35, 2)
    self.assertGreater(ext.update_nav_speed_limit(cp), 0.0)
    for _ in range(NAV_STALE_FRAMES):  # the parser keeps the last value; the receive time no longer moves
      self.assertGreater(ext.update_nav_speed_limit(cp), 0.0)
    self.assertEqual(ext.update_nav_speed_limit(cp), 0.0)
    cp.send(40, 2)  # arrives again
    self.assertAlmostEqual(ext.update_nav_speed_limit(cp), 40 * CV.MPH_TO_MS)

  def test_only_ccnc(self):
    for ccnc, lka in ((False, False), (True, True)):
      ext, cp = make(ccnc, lka)
      cp.send(35, 2)
      self.assertEqual(ext.update_nav_speed_limit(cp), 0.0)


if __name__ == "__main__":
  unittest.main()
