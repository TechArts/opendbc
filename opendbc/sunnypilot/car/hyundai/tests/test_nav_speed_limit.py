import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import HyundaiFlags, CAR
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

  def test_sentinels_and_unknown_units(self):
    ext, cp = make()
    for limit, unit in ((0, 2), (255, 2), (255, 3), (35, 3), (35, 1), (35, 0)):
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


class TestNavSpeedLimitCanValid(unittest.TestCase):
  """The optional message must never affect canValid."""

  def _parser(self):
    CP = CarInterface.get_params(CAR.GENESIS_GV70_2ND_GEN, gen_empty_fingerprint(), [], False, False, False)
    return CarState.get_can_parsers_canfd(None, CP)[Bus.pt]

  def _frame(self, packer, limit):
    return packer.make_can_msg("HU_Navi_ISLW_PE", 0, {"Navi_ISLW_SpdUnit": 2, "Navi_ISLW_SpdLimit": limit})

  def test_valid_when_never_received(self):
    cp = self._parser()
    for i in range(1, 400):
      cp.update([(i * 10_000_000, [])])
      self.assertTrue(cp.can_valid)

  def test_valid_when_it_stops(self):
    cp = self._parser()
    packer = CANPacker("hyundai_canfd_generated")
    t = 0
    for _ in range(20):
      t += 100_000_000
      cp.update([(t, [self._frame(packer, 35)])])
      self.assertTrue(cp.can_valid)
    self.assertEqual(cp.vl["HU_Navi_ISLW_PE"]["Navi_ISLW_SpdLimit"], 35)
    for _ in range(600):  # 60 s of silence
      t += 100_000_000
      cp.update([(t, [])])
      self.assertTrue(cp.can_valid)


if __name__ == "__main__":
  unittest.main()
