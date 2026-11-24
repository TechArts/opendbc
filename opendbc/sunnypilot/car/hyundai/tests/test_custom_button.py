import unittest
from types import SimpleNamespace

from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.carstate_ext import CarStateExt, SWRC_STALE_FRAMES
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


class FakeParser:
  """Stands in for the CANParser: one SWRC_01_200ms message with a receive time."""
  def __init__(self):
    self.vl = {"SWRC_01_200ms": {"Audio_CustomSw3Sta": 0}}
    self.ts_nanos = {"SWRC_01_200ms": {"Audio_CustomSw3Sta": 0}}
    self._t = 0

  def send(self, value):
    self._t += 200_000_000
    self.vl["SWRC_01_200ms"] = {"Audio_CustomSw3Sta": value}
    self.ts_nanos["SWRC_01_200ms"]["Audio_CustomSw3Sta"] = self._t


def make(ccnc=True, lka_steer_msg=False):
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD | (HyundaiFlags.CANFD_LKA_STEER_MSG if lka_steer_msg else 0))
  CP_SP = SimpleNamespace(flags=HyundaiFlagsSP.CCNC if ccnc else 0)
  return CarStateExt(CP, CP_SP), FakeParser()


class TestCustomButton(unittest.TestCase):
  def test_pressed_and_released(self):
    ext, cp = make()
    cp.send(1)
    self.assertTrue(ext.update_custom_button(cp))
    cp.send(0)
    self.assertFalse(ext.update_custom_button(cp))

  def test_never_received(self):
    ext, cp = make()
    self.assertFalse(ext.update_custom_button(cp))

  def test_stale_press_reads_released(self):
    """A message lost while the button is held must not look like a long press."""
    ext, cp = make()
    cp.send(1)
    for _ in range(SWRC_STALE_FRAMES + 1):
      self.assertTrue(ext.update_custom_button(cp))
    self.assertFalse(ext.update_custom_button(cp))

  def test_only_ccnc(self):
    for ccnc, lka in ((False, False), (True, True)):
      ext, cp = make(ccnc, lka)
      cp.send(1)
      self.assertFalse(ext.update_custom_button(cp))


if __name__ == "__main__":
  unittest.main()
