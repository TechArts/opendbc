import unittest

from opendbc.car import structs
from opendbc.car.hyundai.carcontroller import compute_torque_reduction_gain, steer_feel_blend
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.car.interfaces import _initialize_custom_longitudinal_tuning


def settle(torque, v_ego, blend):
  gain = 0.0
  for _ in range(500):
    gain = compute_torque_reduction_gain(torque, v_ego, True, gain, blend)
  return gain


class _CI:
  @staticmethod
  def get_longitudinal_tuning_sp(CP, CP_SP):
    return CP_SP


class TestSteerFeel(unittest.TestCase):
  def _cp(self, fingerprint, flags):
    CP = structs.CarParams(brand="hyundai", carFingerprint=fingerprint, flags=int(flags))
    return CP

  def _cp_sp(self, setting):
    CP_SP = structs.CarParamsSP()
    _initialize_custom_longitudinal_tuning(_CI, self._cp(CAR.GENESIS_GV70_2ND_GEN, HyundaiFlags.SEND_LFA), CP_SP,
                                           {"HyundaiSteerFeel": str(setting)})
    return CP_SP

  def test_param_selects_blend(self):
    CP = self._cp(CAR.GENESIS_GV70_2ND_GEN, HyundaiFlags.SEND_LFA | HyundaiFlags.CANFD_ANGLE_STEERING)
    self.assertEqual([steer_feel_blend(CP, self._cp_sp(s)) for s in (0, 1, 2, 7)], [0.0, 0.5, 1.0, 0.0])

  def test_only_gv70_lfa_alt(self):
    stock = self._cp_sp(2)
    self.assertEqual(steer_feel_blend(self._cp(CAR.GENESIS_GV70_2ND_GEN, HyundaiFlags.CANFD_ANGLE_STEERING), stock), 0.0)
    self.assertEqual(steer_feel_blend(self._cp(CAR.KIA_SPORTAGE_HEV_2026, HyundaiFlags.SEND_LFA), stock), 0.0)
    self.assertTrue(stock.flags & HyundaiFlagsSP.STEER_FEEL_STOCK)

  def test_softer_against_driver_only_above_low_speed(self):
    for torque in (175, 250, 400):
      generic, halfway, stock = (settle(torque, 15.0, b) for b in (0.0, 0.5, 1.0))
      self.assertLess(stock, halfway)
      self.assertLess(halfway, generic)
      self.assertAlmostEqual(halfway, (generic + stock) / 2, delta=0.004)
    # low speed (stock LFA barely active there) and no-hands gain are unchanged
    for torque in (0, 175, 400):
      self.assertEqual(settle(torque, 3.0, 1.0), settle(torque, 3.0, 0.0))
    self.assertEqual(settle(0, 15.0, 1.0), settle(0, 15.0, 0.0))

  def test_zero_when_not_active(self):
    self.assertEqual(compute_torque_reduction_gain(300, 15.0, False, 0.0, 1.0), 0.0)


if __name__ == "__main__":
  unittest.main()
