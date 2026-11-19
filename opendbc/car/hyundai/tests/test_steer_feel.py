import unittest

from opendbc.car import structs
from opendbc.car.hyundai.carcontroller import compute_torque_reduction_gain
from opendbc.car.hyundai.steer_feel import steer_feel_blend
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.car.interfaces import _initialize_custom_longitudinal_tuning


def settle(torque, v_ego, blend, all_speeds=False, curve_hold=None):
  gain = 0.0
  for _ in range(500):
    gain = compute_torque_reduction_gain(torque, v_ego, True, gain, blend, all_speeds, curve_hold)
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
    CP = self._cp(CAR.GENESIS_GV70_2ND_GEN, HyundaiFlags.SEND_LFA)
    self.assertEqual([steer_feel_blend(CP, self._cp_sp(s)) for s in (0, 1, 2, 3, 7)],
                     [(0.0, False), (0.5, False), (1.0, False), (1.0, True), (0.0, False)])

  def test_only_gv70_lfa_alt(self):
    stock = self._cp_sp(2)
    self.assertEqual(steer_feel_blend(self._cp(CAR.GENESIS_GV70_2ND_GEN, 0), stock), (0.0, False))
    self.assertEqual(steer_feel_blend(self._cp(CAR.KIA_SPORTAGE_HEV_2026, HyundaiFlags.SEND_LFA), self._cp_sp(3)), (0.0, False))
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

  def test_all_speeds_matches_highway_feel_at_low_speed(self):
    # setting 3: pushing against openpilot at a crawl yields like it does at speed
    for v_ego in (0.0, 1.0, 3.0):
      for torque in (175, 250, 400):
        self.assertLess(settle(torque, v_ego, 1.0, True), settle(torque, v_ego, 0.0))
        self.assertAlmostEqual(settle(torque, v_ego, 1.0, True), settle(torque, 15.0, 1.0), delta=0.004)
      # no-hands gain is still the generic ceiling
      self.assertEqual(settle(0, v_ego, 1.0, True), settle(0, v_ego, 0.0))

  def test_gain_rises_fast_with_the_hand_off_and_slowly_with_a_hand_on(self):
    """Routes 60-63: after every pause the gain restarted at 0.004 per frame (0.74 s median to 0.3). Hand off (torque under 100): 0.02 per frame."""
    self.assertEqual(compute_torque_reduction_gain(0, 10.0, True, 0.0), 0.02)
    self.assertEqual(compute_torque_reduction_gain(99, 10.0, True, 0.0), 0.02)
    self.assertEqual(compute_torque_reduction_gain(100, 10.0, True, 0.0), 0.004)
    self.assertEqual(compute_torque_reduction_gain(300, 10.0, True, 0.0), 0.004)
    self.assertEqual(compute_torque_reduction_gain(0, 10.0, True, 0.0, 1.0, True), 0.02)  # steer feel: the target at 0 torque is the ceiling
    self.assertAlmostEqual(settle(0, 10.0, 0.0), 0.85, delta=0.004)  # the target is unchanged
    self.assertAlmostEqual(compute_torque_reduction_gain(0, 10.0, False, 0.5), 0.5 - 0.014, delta=0.004)  # the fall is unchanged
    self.assertAlmostEqual(compute_torque_reduction_gain(0, 10.0, True, 0.84), 0.85, delta=0.004)  # no overshoot of the target

  def test_zero_when_not_active(self):
    self.assertEqual(compute_torque_reduction_gain(300, 15.0, False, 0.0, 1.0), 0.0)

  def test_curve_hold_param(self):
    self.assertFalse(self._cp_sp(3).flags & HyundaiFlagsSP.CURVE_HOLD)
    CP_SP = structs.CarParamsSP()
    _initialize_custom_longitudinal_tuning(_CI, self._cp(CAR.GENESIS_GV70_2ND_GEN, HyundaiFlags.SEND_LFA), CP_SP,
                                           {"HyundaiSteerFeel": "3", "HyundaiCurveHold": "1"})
    self.assertTrue(CP_SP.flags & HyundaiFlagsSP.CURVE_HOLD)

  def test_curve_hold_floor_interpolates_with_lateral_demand(self):
    # v 16, 250 torque: stock curve 0.10, generic 0.60
    stock = settle(250, 16.0, 1.0, True)
    self.assertAlmostEqual(stock, 0.10, delta=0.004)
    self.assertEqual(settle(250, 16.0, 1.0, True, (0.2, False, False)), stock)  # below 0.3 m/s^2: stock
    self.assertEqual(settle(250, 16.0, 1.0, True, (0.3, False, False)), stock)
    self.assertAlmostEqual(settle(250, 16.0, 1.0, True, (1.0, False, False)), 0.30, delta=0.004)
    self.assertAlmostEqual(settle(250, 16.0, 1.0, True, (0.65, False, False)), 0.15, delta=0.004)
    self.assertAlmostEqual(settle(250, 16.0, 1.0, True, (2.0, False, False)), 0.55, delta=0.004)
    self.assertAlmostEqual(settle(250, 16.0, 1.0, True, (3.5, False, False)), 0.55, delta=0.004)

  def test_curve_hold_never_above_generic_curve_or_below_stock(self):
    for v in (8.0, 11.0, 16.0, 22.0):
      for torque in (0, 100, 150, 250, 300, 400, 550, 700):
        generic = settle(torque, v, 0.0)
        stock = settle(torque, v, 1.0, True)
        for a in (0.0, 0.5, 1.0, 1.5, 2.0, 4.0):
          held = settle(torque, v, 1.0, True, (a, False, False))
          self.assertLessEqual(held, max(generic, stock) + 0.004, (v, torque, a))
          self.assertGreaterEqual(held, stock - 0.004, (v, torque, a))
    # at 550 torque and 22 m/s the generic curve (0.41) is below the 0.55 floor, so it caps the floor
    self.assertAlmostEqual(settle(550, 22.0, 1.0, True, (2.0, False, False)), settle(550, 22.0, 0.0), delta=0.004)

  def test_curve_hold_off_when_opposing_paused_or_param_off(self):
    stock = settle(250, 16.0, 1.0, True)
    self.assertEqual(settle(250, 16.0, 1.0, True, (2.0, True, False)), stock)
    self.assertEqual(settle(250, 16.0, 1.0, True, (2.0, False, True)), stock)
    self.assertEqual(settle(250, 16.0, 1.0, True, None), stock)
    self.assertEqual(settle(250, 16.0, 0.0, False, (2.0, False, False)), settle(250, 16.0, 0.0))  # steer feel 0: generic is the cap
    self.assertEqual(compute_torque_reduction_gain(250, 16.0, False, 0.5, 1.0, True, (2.0, False, False)),
                     compute_torque_reduction_gain(250, 16.0, False, 0.5, 1.0, True))  # inactive: ramps down as before

  def test_curve_hold_keeps_no_hands_and_light_hand_gain(self):
    for torque in (0, 25, 50):
      self.assertEqual(settle(torque, 16.0, 1.0, True, (2.0, False, False)), settle(torque, 16.0, 1.0, True))

  def test_takeover_params_fill_the_car_params_sp(self):
    """The app reads CarParamsSP.hyundaiTakeover* / hyundaiResumeAngleRateX10 and the HyundaiTakeover*, HyundaiResumeAngleRate, HyundaiCurveHold params."""
    def fill(params, brand="hyundai"):
      CP_SP = structs.CarParamsSP()
      CP = structs.CarParams(brand=brand, carFingerprint=CAR.GENESIS_GV70_2ND_GEN, flags=int(HyundaiFlags.SEND_LFA))
      _initialize_custom_longitudinal_tuning(_CI, CP, CP_SP, params)
      return CP_SP

    defaults = fill({})
    self.assertEqual((defaults.hyundaiTakeoverTorque, defaults.hyundaiTakeoverReleaseTorque, defaults.hyundaiTakeoverHoldMs,
                      defaults.hyundaiResumeAngleRateX10), (150, 100, 500, 10))
    self.assertFalse(defaults.flags & HyundaiFlagsSP.CURVE_HOLD)

    custom = fill({"HyundaiTakeoverTorque": "0", "HyundaiTakeoverReleaseTorque": "80", "HyundaiTakeoverHold": "300",
                   "HyundaiResumeAngleRate": "5", "HyundaiCurveHold": "1"})
    self.assertEqual((custom.hyundaiTakeoverTorque, custom.hyundaiTakeoverReleaseTorque, custom.hyundaiTakeoverHoldMs,
                      custom.hyundaiResumeAngleRateX10), (0, 80, 300, 5))
    self.assertTrue(custom.flags & HyundaiFlagsSP.CURVE_HOLD)

    other = fill({"HyundaiTakeoverTorque": "99"}, brand="toyota")
    self.assertEqual((other.hyundaiTakeoverTorque, other.flags), (0, 0))

  def test_steer_feel_flags_do_not_collide_with_the_platform_flags(self):
    taken = (HyundaiFlagsSP.CCNC | HyundaiFlagsSP.CANFD_LFA_ALT_STEER | HyundaiFlagsSP.CANFD_ANGLE_STEERING | HyundaiFlagsSP.SPEED_LIMIT_AVAILABLE |
             HyundaiFlagsSP.HAS_LKAS12 | HyundaiFlagsSP.LONG_TUNING_DYNAMIC | HyundaiFlagsSP.LONG_TUNING_PREDICTIVE)
    for flag in (HyundaiFlagsSP.STEER_FEEL_HALFWAY, HyundaiFlagsSP.STEER_FEEL_STOCK, HyundaiFlagsSP.STEER_FEEL_ALL_SPEEDS, HyundaiFlagsSP.CURVE_HOLD):
      self.assertFalse(flag & taken)
      self.assertLess(int(flag), 2 ** 32)  # CarParamsSP.flags is a UInt32


if __name__ == "__main__":
  unittest.main()
