import unittest
from types import SimpleNamespace

from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.sunnypilot.car.hyundai.carstate_ext import CarStateExt, CAMERA_STALE_FRAMES, ISLA_SIGN_SCHOOL_CROSSING
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


class FakeCam:
  """Stands in for the camera-bus CANParser: one FR_CMR_02_100ms message with a receive time."""
  def __init__(self):
    self.vl = {"FR_CMR_02_100ms": {"ISLA_AddtnlSign": 0}}
    self.ts_nanos = {"FR_CMR_02_100ms": {"ISLA_AddtnlSign": 0}}
    self._t = 0

  def send(self, sign):
    self._t += 100_000_000
    self.vl["FR_CMR_02_100ms"] = {"ISLA_AddtnlSign": sign}
    self.ts_nanos["FR_CMR_02_100ms"]["ISLA_AddtnlSign"] = self._t


def make(flags_sp=HyundaiFlagsSP.CCNC | HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE):
  CP = SimpleNamespace(flags=HyundaiFlags.CANFD)
  return CarStateExt(CP, SimpleNamespace(flags=flags_sp)), FakeCam()


class TestSchoolSign(unittest.TestCase):
  """carStateSP.speedLimitSchoolSign: the camera's "School Crossing" sign, one input of the speed-limit vote's school-zone rule."""

  def test_school_crossing(self):
    ext, cam = make()
    cam.send(ISLA_SIGN_SCHOOL_CROSSING)
    self.assertTrue(ext.update_school_sign(None, cam))
    cam.send(0)
    self.assertFalse(ext.update_school_sign(None, cam))

  def test_stale_sign_reads_none(self):
    ext, cam = make()
    cam.send(ISLA_SIGN_SCHOOL_CROSSING)
    for _ in range(CAMERA_STALE_FRAMES + 1):
      self.assertTrue(ext.update_school_sign(None, cam))
    self.assertFalse(ext.update_school_sign(None, cam))

  def test_only_ccnc_with_speed_limits(self):
    for flags in (HyundaiFlagsSP.CCNC, HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE):
      ext, cam = make(flags)
      cam.send(ISLA_SIGN_SCHOOL_CROSSING)
      self.assertFalse(ext.update_school_sign(None, cam))


if __name__ == "__main__":
  unittest.main()
