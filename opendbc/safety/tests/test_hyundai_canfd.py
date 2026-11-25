#!/usr/bin/env python3
from opendbc.testing import parameterized_class, parameterized
import unittest
import numpy as np

from opendbc.car.hyundai.carcontroller import ANGLE_SAFETY_BASELINE_MODEL
from opendbc.car.hyundai.values import HyundaiSafetyFlags, CAR, CarControllerParams
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from opendbc.sunnypilot.car.hyundai.values import ANGLE_MODEL_MASK, ANGLE_STEERING_MODEL_BY_CAR, encode_angle_model_id
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import away_round, round_speed, make_msg
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm, AngleSteeringLimitsVM
from opendbc.car.hyundai.interface import CarInterface

# All combinations of radar/camera-SCC and gas/hybrid/EV cars
ALL_GAS_EV_HYBRID_COMBOS = [
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "EXTRA_SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
]


def round_angle(angle_deg: float, can_offset=0):
  scaled = angle_deg / 0.1
  scaled += can_offset
  return int(scaled) * 0.1


class TestHyundaiCanfdBase(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  DBC = "hyundai_canfd_generated"
  SAFETY_MODEL = CarParams.SafetyModel.hyundaiCanfd

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  STANDSTILL_THRESHOLD = 0.375 * 0.03125  # 0.375 kph
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [270]

  MAX_RT_DELTA = 112

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  PT_BUS = 0
  SCC_BUS = 2
  STEER_BUS = 0
  STEER_MSG = ""
  GAS_MSG = ("", "")
  BUTTONS_TX_BUS = 1

  def _torque_driver_msg(self, torque):
    values = {"MDPS_StrTqSnsrVal": torque}
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"StrTqReqVal": torque, "ActToiSta": steer_req}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _speed_msg(self, speed):
    values = {f"WHL_Spd{pos}Val": speed * 3.6 for pos in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"DriverBraking": brake}
    return self.packer.make_can_msg_safety("TCS", self.PT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_safety(self.GAS_MSG[0], self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", bus, values)

  def _acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.PT_BUS, values)

  def _main_cruise_button_msg(self, enabled):
    return self._button_msg(0, enabled)


class TestHyundaiCanfdTorqueSteering(TestHyundaiCanfdBase, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdTorqueSteering":
      cls.packer = None
      raise unittest.SkipTest


def get_controller_params(car_name):
  CP = CarInterface.get_non_essential_params(car_name)
  CarInterface.get_non_essential_params_sp(CP, car_name)
  return CarControllerParams(CP)


class TestHyundaiCanfdAngleSteering(TestHyundaiCanfdBase, common.AngleSteeringSafetyTest):

  SAFETY_PARAM = 0
  SAFETY_PARAM_SP = HyundaiSafetyFlagsSP.CANFD_ANGLE_STEERING
  ANGLE_ACTIVE_SIG = "LKAS_ANGLE_ACTIVE"

  PLATFORMS = {str(platform): platform for platform in CAR if
               str(platform) in ANGLE_STEERING_MODEL_BY_CAR and not CarInterface.get_non_essential_params(str(platform)).dashcamOnly}

  # Angle control limits
  BASELINE_PANDA_ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    360,  # degrees (safe upper bound for command, allowing some margin)
    MAX_ANGLE_RATE=5  # comfort rate limit for angle commands, in degrees per frame.
  )

  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10
  ANGLE_SAFETY_THRESHOLD_PCT = -2.0  # Fail if difference is less than -2%

  # Hyundai uses get_max_angle_delta and get_max_angle for real lateral accel and jerk limits
  # TODO: integrate this into AngleSteeringSafetyTest
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 100  # Hz

  cnt_angle_cmd = 0

  def get_angle_limits(self, car_name):
    limits = get_controller_params(car_name)
    limits.ANGLE_LIMITS = self.BASELINE_PANDA_ANGLE_LIMITS
    return limits

  def get_baseline_limits(self):
    return self.get_angle_limits(ANGLE_SAFETY_BASELINE_MODEL)

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True, gain: float = 0.0):
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    values = {"ADAS_StrAnglReqVal": angle, self.ANGLE_ACTIVE_SIG: 2 if enabled else 1,
              "ADAS_ACIAnglTqRedcGainVal": gain}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _angle_meas_msg(self, angle: float):
    values = {"MDPS_EstStrAnglVal": angle}
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _get_steer_cmd_angle_max(self, speed):
    baseline_vm = self.get_vm(ANGLE_SAFETY_BASELINE_MODEL)
    return get_max_angle_vm(max(speed, 1), baseline_vm, self.get_baseline_limits())

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdAngleSteering":
      cls.packer = None
      raise unittest.SkipTest

  def setUp(self):
    libsafety_py.libsafety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    super().setUp()

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def get_vm(self, car_name):
    return VehicleModel(CarInterface.get_non_essential_params(car_name))

  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  def test_lateral_accel_limit(self):
    car_name = ANGLE_SAFETY_BASELINE_MODEL
    for speed in np.linspace(0, 40, 100):
      speed = round_speed(away_round(speed / 0.03125 * 3.6) * 0.03125 / 3.6)
      speed = max(speed, 1)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed

        # at limit (safety tolerance adds 1)
        angl = get_max_angle_vm(speed, self.get_vm(car_name), self.get_angle_limits(car_name))
        max_angle = round_angle(get_max_angle_vm(speed, self.get_vm(car_name), self.get_angle_limits(car_name)), 1) * sign
        max_angle = np.clip(max_angle, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)), f"{angl} -- {max_angle}")

        # above limit (offset 6 to reliably exceed C float tolerance)
        max_angle_raw = round_angle(get_max_angle_vm(speed, self.get_vm(car_name), self.get_angle_limits(car_name)), 6) * sign
        max_angle = np.clip(max_angle_raw, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self._tx(self._angle_cmd_msg(max_angle, True))

        # at low speeds max angle is above 360, so adding 1 has no effect
        should_tx = abs(max_angle_raw) >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(max_angle, True)), f"should_tx: {should_tx}, max_angle: {max_angle}, speed: {speed}")

  @parameterized("car_name", sorted(PLATFORMS))
  def test_lateral_jerk_limit(self, car_name):
    # car_name = ANGLE_SAFETY_BASELINE_MODEL
    angle_model_id = ANGLE_STEERING_MODEL_BY_CAR.get(car_name, 0)
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP | encode_angle_model_id(angle_model_id))
    self._reset_safety_hooks()

    for speed in np.linspace(0, 40, 100):
      speed = round_speed(away_round(speed / 0.03125 * 3.6) * 0.03125 / 3.6)
      speed = max(speed, 1)
      for sign in (-1, 1):  # (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        # Stay within limits
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.get_vm(car_name), self.get_angle_limits(car_name))) * sign
        sent = self._tx(self._angle_cmd_msg(max_angle_delta, True))
        self.assertTrue(sent)

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        # TODO-SP: Why do I need to set a can_offset so high to pass the tests and why tesla only does +1? and why does it seem to differ based on the baseline?
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.get_vm(car_name), self.get_angle_limits(car_name)), 6) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)), vars(self.get_angle_limits(car_name)))

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

  def test_rt_limits(self):
    # TODO: remove and check all safety modes
    if self.LATERAL_FREQUENCY == -1:
      raise unittest.SkipTest("No real time limits")

    # Angle safety enforces real time limits by checking the message send frequency in a 250ms time window
    self.safety.set_timer(0)
    self.safety.set_controls_allowed(True)
    max_rt_msgs = int(self.LATERAL_FREQUENCY * common.RT_INTERVAL / 1e6 * 1.2 + 1)  # 1.2x buffer

    for i in range(max_rt_msgs * 2):
      should_tx = i <= max_rt_msgs
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

    # One under RT interval should do nothing
    self.safety.set_timer(common.RT_INTERVAL - 1)
    for _ in range(5):
      self.assertFalse(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

    # Increment timer and send 1 message to reset RT window
    self.safety.set_timer(common.RT_INTERVAL)
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))
    for _ in range(5):
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True, increment_timer=False)))

  def test_torque_reduction_gain(self):
    # Valid gains when enabled
    for gain in [0.0, 0.5, 1.0]:
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True, gain=gain)),
                      f"gain={gain} should be allowed when enabled")

    # Reserved values (raw 251+) must fail even when enabled
    for gain in [1.004, 1.008, 1.02]:
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._tx(self._angle_cmd_msg(0, True, gain=gain)),
                       f"gain={gain} (reserved) should be blocked")

    # Non-zero gain when disabled must fail
    for gain in [0.004, 0.5, 1.0]:
      self.safety.set_controls_allowed(True)
      self.assertFalse(self._tx(self._angle_cmd_msg(0, False, gain=gain)),
                       f"gain={gain} should be blocked when disabled")

    # Zero gain when disabled must pass
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False, gain=0.0)))

  @parameterized("car_name", sorted(PLATFORMS))
  def test_max_steering_angle_safety(self, car_name):
    """
    Test that ensures the current car's max steering angles are never more than 2%
    lower than the baseline car across all test speeds.
    """
    baseline_car = ANGLE_SAFETY_BASELINE_MODEL
    baseline_vm = self.get_vm(baseline_car)
    current_vm = self.get_vm(car_name)

    for speed in np.linspace(1, 40, 10):
      baseline_max_angle = get_max_angle_vm(speed, baseline_vm, self.get_baseline_limits())
      current_max_angle = get_max_angle_vm(speed, current_vm, self.get_angle_limits(car_name))

      # Skip if both exceed STEER_ANGLE_MAX (only_relevant_angles logic)
      if current_max_angle > self.STEER_ANGLE_MAX and baseline_max_angle > self.STEER_ANGLE_MAX:
        continue

      # Calculate percentage difference
      if baseline_max_angle != 0:
        angle_diff_pct = ((current_max_angle - baseline_max_angle) / baseline_max_angle) * 100
      else:
        angle_diff_pct = 0

      # Assert that difference is not dangerously low
      self.assertTrue(
        angle_diff_pct >= self.ANGLE_SAFETY_THRESHOLD_PCT,
        f"{car_name} max steering angle at {speed:.1f} m/s [{current_max_angle:.2f}°] is {angle_diff_pct:.2f}% " +
        f"lower than baseline {baseline_car} ({current_max_angle:.2f}° vs {baseline_max_angle:.2f}°). " +
        f"Must be >= {self.ANGLE_SAFETY_THRESHOLD_PCT}% to ensure safety." +
        f"Consider updating the baseline model to be {car_name} (which will lower the threshold for ALL models). " +
        f"Slip Factor: {repr(calc_slip_factor(current_vm))}"
      )

  @parameterized("car_name", sorted(PLATFORMS))
  def test_max_steering_angle_delta_safety(self, car_name):
    """
    Test that ensures the current car's max steering angle deltas are never more than 2%
    lower than the baseline car across all test speeds.
    """
    baseline_car = ANGLE_SAFETY_BASELINE_MODEL
    baseline_vm = self.get_vm(baseline_car)
    baseline_limits = get_controller_params(baseline_car)
    current_vm = self.get_vm(car_name)
    current_limits = get_controller_params(car_name)

    for speed in np.linspace(1, 40, 10):
      baseline_max_delta = get_max_angle_delta_vm(speed, baseline_vm, baseline_limits)
      current_max_delta = get_max_angle_delta_vm(speed, current_vm, current_limits)

      # Calculate percentage difference
      if baseline_max_delta != 0:
        delta_diff_pct = ((current_max_delta - baseline_max_delta) / baseline_max_delta) * 100
      else:
        delta_diff_pct = 0

      # Assert that difference is not dangerously low
      self.assertTrue(
        delta_diff_pct >= self.ANGLE_SAFETY_THRESHOLD_PCT,
        f"{car_name} max steering angle delta at {speed:.1f} m/s is {delta_diff_pct:.2f}% " +
        f"lower than {baseline_car} ({current_max_delta:.4f} vs {baseline_max_delta:.4f} deg/frame). " +
        f"Must be >= {self.ANGLE_SAFETY_THRESHOLD_PCT}% to ensure safety." +
        f"Consider updating the baseline model to be {car_name} (which will lower the threshold for ALL models)." +
        f"Slip Factor: {repr(calc_slip_factor(current_vm))}"
      )

  @parameterized("car_name", sorted(PLATFORMS))
  def test_valid_angle_model_extraction(self, car_name):
    """
    Ensures that the safety layer correctly extracts and assigns the expected
    angle steering model ID from the safety_param_sp bitmask for every supported platform.
    """
    angle_model_id = ANGLE_STEERING_MODEL_BY_CAR.get(car_name, 0)
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP | encode_angle_model_id(angle_model_id))
    self._reset_safety_hooks()
    self.assertEqual(self.safety.get_hyundai_angle_model_id(), angle_model_id)

  def test_out_of_bounds_angle_model_fallback(self):
    """
    Verifies that passing an unknown or out of bounds angle model ID
    via safety_param_sp safely defaults to the baseline fallback model (ID 0).
    """
    model_count = len(ANGLE_STEERING_MODEL_BY_CAR)
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP | encode_angle_model_id(model_count + 1))
    self._reset_safety_hooks()
    self.assertEqual(self.safety.get_hyundai_angle_model_id(), 0)

  def test_angle_model_bitmask_isolation(self):
    """
    Tests that bits [0:10] and bit 15 in safety_param_sp are properly ignored.
    Verifies the shift and mask logic works regardless of other flags being set.
    """
    test_id = 2  # HYUNDAI_IONIQ_5_PE

    # 0x87FF sets every bit to 1 EXCEPT bits 11-14.
    noise_mask = 0x87FF
    param_with_noise = encode_angle_model_id(test_id) | noise_mask

    self.safety.set_current_safety_param_sp(param_with_noise)
    self._reset_safety_hooks()

    self.assertEqual(self.safety.get_hyundai_angle_model_id(), test_id)

  def test_angle_model_does_not_overlap_flags(self):
    flags = [v for k, v in vars(HyundaiSafetyFlagsSP).items() if k.isupper()]
    for model_id in range(ANGLE_MODEL_MASK + 1):
      param = encode_angle_model_id(model_id)
      self.assertLess(param | sum(flags), 2 ** 15)
      for flag in flags:
        self.assertFalse(param & flag)


class TestHyundaiCanfdLFASteeringBase(TestHyundaiCanfdTorqueSteering):

  TX_MSGS = [[0x12A, 0], [0x1A0, 1], [0x1CF, 0], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0)}  # LFA, LFAHDA_CLUSTER
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0]}

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2
  EXTRA_SAFETY_PARAM: int

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdLFASteering", "TestHyundaiCanfdLFASteeringAltButtons"):
      cls.packer = None
      raise unittest.SkipTest

  @property
  def SAFETY_PARAM(self):
    return self.EXTRA_SAFETY_PARAM
@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteering(TestHyundaiCanfdLFASteeringBase):
  pass
class TestHyundaiCanfdLFASteeringAltButtonsBase(TestHyundaiCanfdLFASteeringBase):

  EXTRA_SAFETY_PARAM: int

  @property
  def SAFETY_PARAM(self):
    return HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.EXTRA_SAFETY_PARAM

  def _button_msg(self, buttons, main_button=0, bus=1):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _acc_cancel_msg(self, cancel, accel=0):
    values = {"ACCMode": 4 if cancel else 0, "aReqRaw": accel, "aReqValue": accel}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_button_sends(self):
    """
      No button send allowed with alt buttons.
    """
    for enabled in (True, False):
      for btn in range(8):
        self.safety.set_controls_allowed(enabled)
        self.assertFalse(self._tx(self._button_msg(btn)))

  def test_acc_cancel(self):
    # FIXME: the CANFD_ALT_BUTTONS cars are the only ones that use SCC_CONTROL to cancel, why can't we use buttons?
    for enabled in (True, False):
      self.safety.set_controls_allowed(enabled)
      self.assertTrue(self._tx(self._acc_cancel_msg(True)))
      self.assertFalse(self._tx(self._acc_cancel_msg(True, accel=1)))
      self.assertFalse(self._tx(self._acc_cancel_msg(False)))
@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringAltButtons(TestHyundaiCanfdLFASteeringAltButtonsBase):
  pass


class TestHyundaiCanfdLKASteeringEV(TestHyundaiCanfdTorqueSteering):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}  # LKAS, CAM_0x2A4
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
# TODO: Handle ICE and HEV configurations once we see cars that use the new messages
class TestHyundaiCanfdLKASteeringAltEVBase(TestHyundaiCanfdBase):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}  # LKAS_ALT, CAM_0x362
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")


class TestHyundaiCanfdLKASteeringAltEVTorque(TestHyundaiCanfdLKASteeringAltEVBase, TestHyundaiCanfdTorqueSteering):
  pass


class TestHyundaiCanfdLKASteeringAltAngle(TestHyundaiCanfdAngleSteering):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED")

  # Angle steering does not use torque — override inherited torque tests
  def test_steer_safety_check(self):
    pass

  def test_non_realtime_limit_up(self):
    pass

  def test_steer_req_bit(self):
    pass

  def test_steer_req_bit_frames(self):
    pass

  def test_steer_req_bit_multi_invalid(self):
    pass

  def test_steer_req_bit_realtime(self):
    pass

  def test_against_torque_driver(self):
    pass

  def test_realtime_limits(self):
    pass


CAMERA_SCC_COMBOS = [c for c in ALL_GAS_EV_HYBRID_COMBOS if c["EXTRA_SAFETY_PARAM"] & HyundaiSafetyFlags.CAMERA_SCC]


class HyundaiCanfdLFAAltAngleBase(TestHyundaiCanfdLKASteeringAltAngle):

  TX_MSGS = [[0xCB, 0], [0x1A0, 0], [0x1CF, 2], [0x161, 0], [0x162, 0], [0xEA, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0xCB, 0x161, 0x162), 2: (0xEA,)}
  FWD_BLACKLISTED_ADDRS = {2: [0xCB, 0x161, 0x162]}  # 0xEA is forwarded until openpilot sends its own

  PT_BUS = 0
  SCC_BUS = 2
  STEER_MSG = "LFA_ALT"
  BUTTONS_TX_BUS = 2
  ANGLE_ACTIVE_SIG = "ADAS_ActvACILvl2Sta"
  SAFETY_PARAM_SP = HyundaiSafetyFlagsSP.CANFD_ANGLE_STEERING | HyundaiSafetyFlagsSP.CCNC | HyundaiSafetyFlagsSP.CANFD_LFA_ALT_STEER
  EXTRA_SAFETY_PARAM: int

  STEER_ANGLE_MAX = 175  # deg
  BASELINE_PANDA_ANGLE_LIMITS = AngleSteeringLimitsVM(175, MAX_ANGLE_RATE=5)

  @property
  def SAFETY_PARAM(self):
    return self.EXTRA_SAFETY_PARAM

  def test_lfa_alt_unused_signals(self):
    self.safety.set_controls_allowed(True)
    for sig in ("ADAS_ActvACISta", "FCA_ESA_ActvSta", "FCA_ESA_TqBstGainVal"):
      for active in (1, 2):
        ok = self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": active})
        bad = self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": active, sig: 0.004 if "Gain" in sig else 1})
        self.assertTrue(self._tx(ok))
        self.assertFalse(self._tx(bad), sig)

  def test_lfa_not_sendable(self):
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("LFA", 0, {})))
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("LFAHDA_CLUSTER", 0, {})))

  def test_mdps_forwarded_until_openpilot_sends_its_own(self):
    # the camera needs an MDPS at all times: the real one reaches it until our copy starts, then only ours
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0xEA))
    self.assertTrue(self._tx(make_msg(2, 0xEA, 24)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0xEA))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x12A))  # only 0xEA is affected
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.SAFETY_PARAM)  # a new init starts forwarding it again
    self.safety.init_tests()
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0xEA))

  def test_mdps_copy_only_on_camera_bus(self):
    self.assertFalse(self._tx(make_msg(0, 0xEA, 24)))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0xEA))

  def test_mdps_forward_stays_blocked_without_lfa_alt(self):
    # outside LFA_ALT the static block stays: the other CCNC lists never forward the real 0xEA
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP & ~HyundaiSafetyFlagsSP.CANFD_LFA_ALT_STEER)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.SAFETY_PARAM)
    self.safety.init_tests()
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0xEA))
    self._tx(make_msg(2, 0xEA, 24))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0xEA))

  def test_lfa_alt_requires_angle_ccnc(self):
    lfa_alt = self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": 1})
    for param, param_sp in ((self.SAFETY_PARAM, self.SAFETY_PARAM_SP & ~HyundaiSafetyFlagsSP.CCNC),
                            (self.SAFETY_PARAM, self.SAFETY_PARAM_SP & ~HyundaiSafetyFlagsSP.CANFD_ANGLE_STEERING),
                            (self.SAFETY_PARAM & ~HyundaiSafetyFlags.CAMERA_SCC, self.SAFETY_PARAM_SP),
                            (self.SAFETY_PARAM | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG, self.SAFETY_PARAM_SP)):
      self.safety.set_current_safety_param_sp(param_sp)
      self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, param)
      self.safety.init_tests()
      self.assertFalse(self._tx(lfa_alt), (param, param_sp))


@parameterized_class(CAMERA_SCC_COMBOS)
class TestHyundaiCanfdLFAAltAngle(HyundaiCanfdLFAAltAngleBase):

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFAAltAngle":
      cls.packer = None
      raise unittest.SkipTest


@parameterized_class(CAMERA_SCC_COMBOS)
class TestHyundaiCanfdLFAAltAngleLong(HyundaiLongitudinalBase, HyundaiCanfdLFAAltAngleBase):

  RELAY_MALFUNCTION_ADDRS = {0: (0xCB, 0x1A0, 0x161, 0x162), 2: (0x1CF, 0xEA)}
  FWD_BLACKLISTED_ADDRS = {0: [0x1CF], 2: [0xCB, 0x1A0, 0x161, 0x162]}

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFAAltAngleLong":
      cls.packer = None
      raise unittest.SkipTest

  @property
  def SAFETY_PARAM(self):
    return HyundaiSafetyFlags.LONG | self.EXTRA_SAFETY_PARAM

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {"aReqRaw": accel, "aReqValue": accel}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_tester_present_allowed(self, ecu_disable: bool = True):
    super().test_tester_present_allowed(ecu_disable=False)

  def test_button_sends(self):
    for btn in range(8):
      for main_btn in (0, 1):
        self.assertTrue(self._tx(self._button_msg(btn, main_button=main_btn, bus=self.BUTTONS_TX_BUS)))


class HyundaiCanfdLKASteeringAltButtonsMixin:
  def _button_msg(self, buttons, main_button=0, bus=None):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS if bus is None else bus, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)
class TestHyundaiCanfdLKASteeringAltButtonsHybrid(HyundaiCanfdLKASteeringAltButtonsMixin, TestHyundaiCanfdLKASteeringEV):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CANFD_ALT_BUTTONS

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x1AA, 1]]
  GAS_MSG = ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL")
class TestHyundaiCanfdLKASteeringAltAltButtonsHybrid(HyundaiCanfdLKASteeringAltButtonsMixin, TestHyundaiCanfdLKASteeringAltEVTorque):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT | \
                 HyundaiSafetyFlags.CANFD_ALT_BUTTONS

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0], [0x1AA, 1]]
  GAS_MSG = ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL")
class TestHyundaiCanfdLKASteeringLongEV(HyundaiLongitudinalBase, TestHyundaiCanfdLKASteeringEV):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4), 1: (0x1a0,)}  # LKAS, CAM_0x2A4, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)
# Tests longitudinal for ICE, hybrid, EV cars with LFA steering
class TestHyundaiCanfdLFASteeringLongBase(HyundaiLongitudinalBase, TestHyundaiCanfdLFASteeringBase):

  FWD_BLACKLISTED_ADDRS = {2: [0x12a, 0x1e0, 0x1a0, 0x160]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x1a0, 0x160)}  # LFA, LFAHDA_CLUSTER, SCC_CONTROL, ADRV_0x160

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  @property
  def SAFETY_PARAM(self):
    return HyundaiSafetyFlags.LONG | self.EXTRA_SAFETY_PARAM

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_tester_present_allowed(self, ecu_disable: bool = True):
    super().test_tester_present_allowed(ecu_disable=not self.EXTRA_SAFETY_PARAM & HyundaiSafetyFlags.CAMERA_SCC)
@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLong(TestHyundaiCanfdLFASteeringLongBase):
  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLong":
      raise unittest.SkipTest
@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLongAltButtons(TestHyundaiCanfdLFASteeringLongBase, TestHyundaiCanfdLFASteeringAltButtonsBase):
  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongAltButtons":
      raise unittest.SkipTest

  @property
  def SAFETY_PARAM(self):
    return HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.EXTRA_SAFETY_PARAM

  def test_acc_cancel(self):
    # Alt buttons does not use SCC_CONTROL to cancel if longitudinal
    pass


@parameterized_class(CAMERA_SCC_COMBOS)
class TestHyundaiCanfdLFASteeringCCNC(TestHyundaiCanfdLFASteeringBase):

  TX_MSGS = [[0x12A, 0], [0x1E0, 0], [0x1CF, 2], [0x161, 0], [0x162, 0], [0xEA, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x161, 0x162), 2: (0xEA,)}
  FWD_BLACKLISTED_ADDRS = {0: [0xEA], 2: [0x12A, 0x1E0, 0x161, 0x162]}

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFASteeringCCNC":
      cls.packer = None
      raise unittest.SkipTest

  def setUp(self):
    libsafety_py.libsafety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.CCNC)
    super().setUp()

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def test_ccnc_tx_msgs(self):
    for msg, bus in (("CCNC_0x161", 0), ("CCNC_0x162", 0), ("MDPS", 2)):
      self.assertTrue(self._tx(self.packer.make_can_msg_safety(msg, bus, {})))


@parameterized_class(CAMERA_SCC_COMBOS)
class TestHyundaiCanfdLFASteeringCCNCLong(TestHyundaiCanfdLFASteeringLongBase, TestHyundaiCanfdLFASteeringCCNC):

  TX_MSGS = [[0x12A, 0], [0x1E0, 0], [0x1CF, 2], [0x1A0, 0], [0x161, 0], [0x162, 0], [0xEA, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x1A0, 0x161, 0x162), 2: (0x1CF, 0xEA)}
  FWD_BLACKLISTED_ADDRS = {0: [0x1CF, 0xEA], 2: [0x12A, 0x1E0, 0x1A0, 0x161, 0x162]}

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFASteeringCCNCLong":
      cls.packer = None
      raise unittest.SkipTest

  def test_button_sends(self):
    for btn in range(8):
      for main_btn in (0, 1):
        self.assertTrue(self._tx(self._button_msg(btn, main_button=main_btn, bus=self.BUTTONS_TX_BUS)))


class TestHyundaiCanfdLongMainButtonEngage(unittest.TestCase):
  """CCNC LFA_ALT camera-SCC longitudinal: the camera's own MainMode_ACC (bus 2 0x1A0) is the main cruise state. Its rising edge with a main
  button press nearby engages openpilot longitudinal, its falling edge disengages. A press the camera ignores or a camera change without a
  press does nothing."""

  BASE = HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.LONG
  BASE_SP = HyundaiSafetyFlagsSP.CANFD_ANGLE_STEERING | HyundaiSafetyFlagsSP.CCNC | HyundaiSafetyFlagsSP.CANFD_LFA_ALT_STEER
  FLAG = HyundaiSafetyFlagsSP.MAIN_BUTTON_ENGAGE
  SET, RESUME, CANCEL = 2, 1, 4

  def setUp(self):
    self.packer = common.CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def _init(self, param=None, param_sp=None, toggleable=True):
    param = self.BASE if param is None else param
    param_sp = (self.BASE_SP | self.FLAG) if param_sp is None else param_sp
    if toggleable:
      param_sp |= HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE
    self.safety.set_current_safety_param_sp(int(param_sp))
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, int(param))
    self.safety.init_tests()
    self.safety.set_timer(0)
    self.cam = False

  def _button(self, buttons=0, main=0, long_main=0):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("CRUISE_BUTTONS", 0, {"CRUISE_BUTTONS": buttons, "ADAPTIVE_CRUISE_MAIN_BTN": main,
                                                                                      "NORMAL_CRUISE_MAIN_BTN": long_main}))

  def _brake(self, pressed):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("TCS", 0, {"DriverBraking": pressed}))

  def _camera_main(self, on):
    self.cam = on
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 2, {"MainMode_ACC": int(on)}))

  def _press_main(self, frames=3, camera=True):
    """A main press; the camera toggles its main state on the first frame of it (unless it ignores the press)."""
    for i in range(frames):
      self._button(main=1)
      if i == 0 and camera:
        self._camera_main(not self.cam)
    self._button(main=0)

  def _tap(self, btn, frames=3):
    for _ in range(frames):
      self._button(btn)
    self._button(0)

  def test_main_press_engages(self):
    self._init()
    self._button(main=0)
    self.assertFalse(self.safety.get_controls_allowed())
    self._button(main=1)  # a press alone does not change main: only the camera does
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed())
    self._camera_main(True)
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed())
    self._button(main=0)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_long_main_press_engages(self):
    # route 94: a long MAIN sends only NORMAL_CRUISE_MAIN_BTN; with main-button engage it is the driver's MAIN press
    self._init()
    self._button(long_main=1)
    self.assertFalse(self.safety.get_controls_allowed())
    self._camera_main(True)
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed())

  def test_long_main_bit_is_no_press_without_main_button_engage(self):
    self._init(param_sp=self.BASE_SP)
    self._button(long_main=1)
    self.assertFalse(self.safety.get_acc_main_on())  # with the flag off bit 21 is no MAIN, so it does not toggle main (review R1 2026-11-25)
    self._camera_main(True)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_camera_ignores_press(self):
    self._init()
    self._press_main(camera=False)  # e.g. brake held at low speed: the stock car does not engage either
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_timer(100_000_000)
    self._camera_main(True)  # much later, no press near it
    self.assertFalse(self.safety.get_acc_main_on())  # raw camera availability is not a driver-qualified main
    self.assertFalse(self.safety.get_controls_allowed())

  def test_camera_reacts_before_press_is_seen(self):
    self._init()
    self._camera_main(True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_timer(200_000)
    self._button(main=1)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_camera_main_without_press_does_not_arm(self):
    self._init()
    self._camera_main(True)
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_timer(2_000_000)
    self._button(main=1)  # a press long after: nothing to pair with
    self._button(main=0)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_stale_press_does_not_arm(self):
    self._init()
    self._button(main=1)
    self._button(main=0)
    self.safety.set_timer(2_000_000)
    self._camera_main(True)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_camera_main_falling_disengages_and_keeps_lateral(self):
    self._init()
    self.safety.set_mads_params(True, False, False)
    self._press_main()
    self.assertTrue(self.safety.get_controls_allowed() and self.safety.get_controls_allowed_lateral())
    self._camera_main(False)  # also without a press (e.g. the car switched ACC off)
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_main_does_not_toggle_by_press_alone(self):
    self._init()
    self._press_main(camera=False)
    self._press_main(camera=False)
    self.assertFalse(self.safety.get_acc_main_on())

  def test_second_main_press_disengages(self):
    self._init()
    self._press_main()
    self.assertTrue(self.safety.get_controls_allowed())
    self._press_main()  # camera: main off
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed())
    self._press_main()  # and again: engage
    self.assertTrue(self.safety.get_controls_allowed())

  def test_long_press_is_one_transition(self):
    self._init()
    self._press_main(frames=200)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_acc_main_on())

  def test_main_press_engages_at_standstill_and_moving(self):
    for speed in (0, 20):
      with self.subTest(speed=speed):
        self._init()
        self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, {f"WHL_Spd{p}Val": speed * 3.6 for p in ("FL", "FR", "RL", "RR")}))
        self._press_main()
        self.assertTrue(self.safety.get_controls_allowed())

  def test_without_flag_main_never_engages(self):
    self._init(param_sp=self.BASE_SP)
    self._press_main()
    self.assertTrue(self.safety.get_acc_main_on())  # main still toggles (MADS)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_flag_requires_ccnc_lfa_alt_long(self):
    # the flag alone, without the rest of the config, is ignored
    full = self.BASE_SP | self.FLAG
    for param, param_sp in ((self.BASE & ~HyundaiSafetyFlags.LONG, full), (self.BASE & ~HyundaiSafetyFlags.CAMERA_SCC, full),
                            (self.BASE, full & ~HyundaiSafetyFlagsSP.CCNC), (self.BASE, full & ~HyundaiSafetyFlagsSP.CANFD_LFA_ALT_STEER),
                            (self.BASE, full & ~HyundaiSafetyFlagsSP.CANFD_ANGLE_STEERING),
                            (self.BASE | HyundaiSafetyFlags.CANFD_ALT_BUTTONS, full), (self.BASE | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG, full)):
      with self.subTest(param=int(param), param_sp=int(param_sp)):
        self._init(param, param_sp)
        self._press_main()
        self.assertFalse(self.safety.get_controls_allowed())

  def test_toggleable_flag_irrelevant(self):
    self._init(toggleable=False)  # the camera, not a toggled flag, is the main state
    self._press_main()
    self.assertTrue(self.safety.get_controls_allowed())

  def test_cancel_still_disengages(self):
    self._init()
    self._press_main()
    self.assertTrue(self.safety.get_controls_allowed())
    self._button(self.CANCEL)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_acc_main_on())  # pause: main stays on

  def test_cancel_wins_over_simultaneous_main(self):
    self._init()
    self._button(self.CANCEL, main=1)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_still_disengages(self):
    self._init()
    self._press_main()
    self.assertTrue(self.safety.get_controls_allowed())
    self._brake(1)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_set_resume_without_flag_unchanged(self):
    for btn in (self.SET, self.RESUME):
      with self.subTest(btn=btn):
        self._init(param_sp=self.BASE_SP)
        self._button(btn)
        self.assertFalse(self.safety.get_controls_allowed())  # engages on release, not press
        self._button(0)
        self.assertTrue(self.safety.get_controls_allowed())
        self.assertFalse(self.safety.get_acc_main_on())

  def test_set_resume_with_main_off_does_not_arm(self):
    for btn in (self.SET, self.RESUME):
      with self.subTest(btn=btn):
        self._init()
        self._button(btn)
        self._button(0)
        self.assertFalse(self.safety.get_acc_main_on())
        self.assertFalse(self.safety.get_controls_allowed())
        self._press_main(camera=False)  # camera main still off after an ignored press
        self._button(btn)
        self._button(0)
        self.assertFalse(self.safety.get_controls_allowed())

  def test_set_resume_with_main_on_arms(self):
    for btn in (self.SET, self.RESUME):
      with self.subTest(btn=btn):
        self._init()
        self._press_main()
        self._button(self.CANCEL)  # main stays on, controls off
        self._button(0)
        self.assertTrue(self.safety.get_acc_main_on())
        self.assertFalse(self.safety.get_controls_allowed())
        self._button(btn)
        self._button(0)
        self.assertTrue(self.safety.get_controls_allowed())

  def test_set_after_main_off_does_not_rearm(self):
    self._init()
    self._press_main()  # engage
    self._press_main()  # main off
    self.assertFalse(self.safety.get_controls_allowed())
    self._button(self.SET)
    self._button(0)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_init_resets_flag(self):
    self._init()
    self._init(param_sp=self.BASE_SP)
    self._press_main()
    self.assertFalse(self.safety.get_controls_allowed())

  def _lateral_on(self, param_sp=None):
    self._init(param_sp=param_sp)
    self.safety.set_mads_params(True, False, False)
    self._press_main()  # main on: also requests lateral
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_main_off_keeps_lateral_with_flag(self):
    self._lateral_on()
    self._press_main()  # second press: main off ends cruise only
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_main_off_drops_lateral_without_flag(self):
    self._init(param_sp=self.BASE_SP)
    self.safety.set_mads_params(True, False, False)
    self._button(main=1)
    self._button(main=0)
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._button(main=1)
    self._button(main=0)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_other_lateral_exits_unchanged_with_flag(self):
    # heartbeat mismatch (sunnypilot ended lateral, e.g. LFA button) still exits after 3 checks
    self._lateral_on()
    self.safety.set_heartbeat_engaged_mads(True)
    for _ in range(5):
      self.safety.mads_heartbeat_engaged_check()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.safety.set_heartbeat_engaged_mads(False)
    for _ in range(3):
      self.safety.mads_heartbeat_engaged_check()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_flag_resets_on_init(self):
    self._lateral_on()
    self._init(param_sp=self.BASE_SP)
    self.safety.set_mads_params(True, False, False)
    self._button(main=1)
    self._button(main=0)
    self._button(main=1)
    self._button(main=0)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_openpilot_scc_control_not_part_of_main_state(self):
    # our own MainMode_ACC in SCC_CONTROL is not compared with the camera's: no desync exit in this config
    self._init()
    self.safety.set_mads_params(True, False, False)
    self._press_main()
    for _ in range(10):
      self.safety.safety_tx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 0, {"MainMode_ACC": 0}))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertEqual(0, self.safety.get_acc_main_on_mismatches())

  def _camera_acc(self, acc_mode, main=1):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 2, {"ACCMode": acc_mode, "MainMode_ACC": main}))
    self.cam = bool(main)

  def _scc_fwd(self):
    return self.safety.safety_fwd_hook(2, 0x1A0)

  def test_stock_acc_never_reaches_the_car(self):
    """F21/F23 (stock ACC lock-out): with openpilot longitudinal the camera's 0x1A0 is a statically blocked relay-checked message, so no ACC-active
    frame of the camera reaches the car in any state: before the engage, after it, on a pause or cancel press, when the camera engages by itself,
    after a brake or main off. Openpilot's own frames (ACCMode 0 when not engaged) are the only 0x1A0 the car sees."""
    for param_sp in (None, self.BASE_SP):  # with and without main-button engage
      with self.subTest(param_sp=param_sp):
        self._init(param_sp=param_sp)
        self.assertEqual(-1, self._scc_fwd())
        self._press_main()
        self.assertEqual(-1, self._scc_fwd())
        self._camera_acc(1)
        self.assertEqual(-1, self._scc_fwd())
        self._tap(self.CANCEL)
        self._camera_acc(1)  # the pause key is the camera's resume key too
        self.assertEqual(-1, self._scc_fwd())
        self._brake(1)
        self._camera_acc(2)
        self.assertEqual(-1, self._scc_fwd())
        self._camera_acc(4, main=0)
        self.assertEqual(-1, self._scc_fwd())

  def test_camera_acc_engaging_by_itself_does_not_enter_controls(self):
    self._init()
    self._camera_acc(4)  # main on, standby
    self._camera_acc(1)  # the camera engages without a driver press for openpilot
    self._camera_acc(1)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(-1, self._scc_fwd())

  def test_stock_acc_openpilot_off_frames_pass_safety(self):
    # the replacement frame for the camera's own: ACCMode 0, no acceleration, allowed without controls
    self._init()
    self.assertTrue(self.safety.safety_tx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 0, {"ACCMode": 0, "aReqRaw": 0, "aReqValue": 0})))
    self.assertFalse(self.safety.safety_tx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 0, {"ACCMode": 1, "aReqRaw": 1.0, "aReqValue": 1.0})))

  def test_pause_key_is_a_pause_resume_toggle(self):
    self._init()
    self._press_main()
    self._tap(self.CANCEL)  # pause: the release does not resume
    self.assertFalse(self.safety.get_controls_allowed())
    self._button(self.CANCEL)
    self.assertFalse(self.safety.get_controls_allowed())  # resume on release, not on press
    self._button(0)
    self.assertTrue(self.safety.get_controls_allowed())
    self._tap(self.CANCEL)  # and pause again
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pause_key_does_not_resume_with_main_off(self):
    self._init()
    self._tap(self.CANCEL)
    self.assertFalse(self.safety.get_controls_allowed())
    self._press_main()
    self._tap(self.CANCEL)
    self._button(self.CANCEL)
    self._camera_main(False)  # main goes off while the key is down
    self._button(0)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pause_key_does_not_resume_on_a_held_brake(self):
    self._init()
    self._press_main()
    self._tap(self.CANCEL)
    self._brake(1)  # held at standstill: no rising-edge or moving exit after this
    self._tap(self.CANCEL)
    self.assertFalse(self.safety.get_controls_allowed())
    self._brake(0)
    self._tap(self.CANCEL)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_pause_key_without_flag_only_cancels(self):
    self._init(param_sp=self.BASE_SP)
    self._press_main()  # main toggles (MADS), no engage
    self.assertTrue(self.safety.get_acc_main_on())
    self._tap(self.CANCEL)
    self.assertFalse(self.safety.get_controls_allowed())
    self._tap(self.CANCEL)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pause_key_resume_is_gated_like_the_stock_one(self):
    # a resume needs the press to have started with controls off: a press that started with controls on only exits
    self._init()
    self._press_main()
    self.assertTrue(self.safety.get_controls_allowed())
    for _ in range(3):
      self._button(self.CANCEL)
    self._button(0)
    self.assertFalse(self.safety.get_controls_allowed())

  # --- policy: during a pause-key hold any brake press, main change or lag fault disarms the pending resume; the release only pauses
  def _hold_resume_released(self, interrupt):
    self._init()
    self._press_main()
    self._tap(self.CANCEL)  # paused: main on, controls off
    self.assertFalse(self.safety.get_controls_allowed())
    for _ in range(7):
      self._button(self.CANCEL)
    interrupt()
    self._button(0)
    return self.safety.get_controls_allowed()

  def test_hold_brake_pressed_and_released_disarms(self):
    def interrupt():
      self._brake(1)
      self._brake(0)
    self.assertFalse(self._hold_resume_released(interrupt))
    # pause only: a fresh press starts clean
    for _ in range(3):
      self._button(self.CANCEL)
    self._button(0)
    self.assertTrue(self.safety.get_controls_allowed())

  def test_hold_main_off_then_on_disarms(self):
    def interrupt():
      self._camera_main(False)
      self._camera_main(True)
    self.assertFalse(self._hold_resume_released(interrupt))

  def test_hold_main_off_disarms(self):
    self.assertFalse(self._hold_resume_released(lambda: self._camera_main(False)))

  def test_hold_lag_fault_disarms(self):
    def interrupt():
      self.safety.set_timer(5_000_000)
      self.safety.safety_tick()
    self.assertFalse(self._hold_resume_released(interrupt))

  def test_hold_safety_reinit_disarms(self):
    # a lost heartbeat re-initializes the safety: the pending resume is gone with it
    self.assertFalse(self._hold_resume_released(lambda: self._init() or self.safety.set_controls_allowed(False)))

  def test_hold_without_fault_still_resumes(self):
    self.assertTrue(self._hold_resume_released(lambda: None))

  def test_camera_only_rise_grants_neither_lateral_nor_long(self):
    # an unsolicited camera MAIN rise (no driver press) is availability only: MADS must not see a main rising edge
    self._init()
    self.safety.set_mads_params(True, False, False)
    self._camera_main(True)
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_timer(5_000_000)
    self._tap(self.SET)  # SET/RES do not arm on a camera main nobody asked for
    self.assertFalse(self.safety.get_controls_allowed())

  def test_press_then_cancel_before_the_camera_answers(self):
    self._init()
    self.safety.set_mads_params(True, False, False)
    self._button(main=1)
    self._button(main=0)
    self._button(self.CANCEL)
    self._button(0)
    self.safety.set_timer(200_000)
    self._camera_main(True)  # the late answer
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_cancel_together_with_main_press_does_not_arm_a_late_answer(self):
    self._init()
    self._button(self.CANCEL, main=1)
    self._button(0)
    self._camera_main(True)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_camera_first_pairing_is_single_use(self):
    self._init()
    self._camera_main(True)
    self.safety.set_timer(100_000)
    self._button(main=1)  # pairs with the camera's rise
    self.assertTrue(self.safety.get_controls_allowed())
    self._button(main=0)
    self._camera_main(False)
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_timer(200_000)
    self._camera_main(True)  # fall and rise inside the window: the same press must not engage again
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_acc_main_on())

  def test_press_first_pairing_is_single_use(self):
    self._init()
    self._button(main=1)
    self._button(main=0)
    self._camera_main(True)
    self.assertTrue(self.safety.get_controls_allowed())
    self._camera_main(False)
    self.safety.set_timer(200_000)
    self._camera_main(True)
    self.assertFalse(self.safety.get_controls_allowed())


class TestHyundaiCanfdCameraHandback(unittest.TestCase):
  """CAMERA_HANDBACK (SP flag 256) on the CCNC LFA_ALT camera-SCC config: openpilot owns the camera's 0xCB (LFA_ALT) and the MDPS 0xEA copy to the
  camera from init on. The car's own lane-keeping nudge (LKA) passes through the forward hook while openpilot is not steering: camera 0x161 LFA_ICON 0
  for >= 1.5 s and camera 0x12A LKA_SysIndReq 3 within 0.5 s, openpilot with no accepted ACTIVE 0xCB for 0.2 s. One sender at a time (owner state
  machine OURS / TO_CAM / CAM / TO_OURS), never two senders, at most one cycle without a 0xCB at a switch. 0x1A0 keeps upstream's static block."""

  BASE = HyundaiSafetyFlags.CAMERA_SCC
  BASE_SP = HyundaiSafetyFlagsSP.CANFD_ANGLE_STEERING | HyundaiSafetyFlagsSP.CCNC | HyundaiSafetyFlagsSP.CANFD_LFA_ALT_STEER
  FLAG = HyundaiSafetyFlagsSP.CAMERA_HANDBACK
  LONG = False

  def setUp(self):
    self.packer = common.CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def _init(self, flag=True, base=None, base_sp=None, mdps_copy=True):
    param = (self.BASE if base is None else base) | (HyundaiSafetyFlags.LONG if self.LONG else 0)
    param_sp = (self.BASE_SP if base_sp is None else base_sp) | (self.FLAG if flag else 0)
    self.safety.set_current_safety_param_sp(int(param_sp))
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, int(param))
    self.safety.init_tests()
    self.safety.set_timer(0)
    if mdps_copy:
      self.assertTrue(self._mdps())  # openpilot's first MDPS copy: from here on the real MDPS is blocked while openpilot owns lateral

  def _cb(self, active):
    return self.safety.safety_tx_hook(self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": 2 if active else 1}))

  def _mdps(self):
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0xEA, 2, b"\x00" * 24))

  def _rx(self, addr, bus, length):
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, b"\x00" * length))

  # forwarded (the camera's frame reaches the car / the camera) vs blocked
  def _cb_fwd(self):
    return self.safety.safety_fwd_hook(2, 0xCB) == 0

  def _mdps_fwd(self):
    return self.safety.safety_fwd_hook(0, 0xEA) == 2

  def _scc_fwd(self):
    return self.safety.safety_fwd_hook(2, 0x1A0) == 0

  def _assert_scc_static(self):
    self.assertEqual(self.LONG, not self._scc_fwd())  # 0x1A0: upstream's static block (longitudinal), nothing handed back

  # --- the camera's lane-keeping signals: 0x161 LFA_ICON and 0x12A LKA_SysIndReq (research/stock-lfa-ownership.md) ---
  def _cam_icon(self, icon):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("CCNC_0x161", 2, {"LFA_ICON": icon}))

  def _cam_ind(self, ind):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("LFA", 2, {"LKA_SysIndReq": ind}))

  def _cam_run(self, t0, t1, icon, ind, step=10_000):
    """camera frames from t0 to t1 (us): 0x12A every step, 0x161 every 50 ms; leaves the timer at t1"""
    t = t0
    while t <= t1:
      self.safety.set_timer(t)
      self._cam_ind(ind)
      if (t - t0) % 50_000 == 0:
        self._cam_icon(icon)
      t += step
    self.safety.set_timer(t1)

  def _own_lat_then_off(self):
    """openpilot steers (MADS on), then the driver turns it off: openpilot keeps sending inactive frames"""
    self._init()
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._cb(True))
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self._cb_fwd())

  def _open_window_with_cruise_on(self):
    self._init()
    self.safety.set_controls_allowed(True)
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertTrue(self._cb_fwd())
    self.assertFalse(self._cb(False))

  def _sweep(self, t0, t1, icon_at, ind_at, active_at=lambda t: False, camera_first=True):
    """10 ms cycles, camera status every cycle (icon every 50 ms), then both senders in the given order, then both MDPS senders. Never two 0xCB or
    two MDPS frames in a cycle; never two cycles in a row without a 0xCB. Returns [(ours, camera)] per cycle."""
    out, prev_empty = [], False
    for t in range(t0, t1, 10_000):
      self.safety.set_timer(t)
      if t % 50_000 == 0:
        self._cam_icon(icon_at(t))
      self._cam_ind(ind_at(t))
      if camera_first:
        cam = self._cb_fwd()
        ours = self._cb(active_at(t))
      else:
        ours = self._cb(active_at(t))
        cam = self._cb_fwd()
      self.assertFalse(ours and cam, f"two 0xCB senders at t={t}")
      empty = not (ours or cam)
      self.assertFalse(empty and prev_empty, f"two cycles without 0xCB at t={t}")
      prev_empty = empty
      self.assertFalse(self._mdps() and self._mdps_fwd(), f"two MDPS senders at t={t}")
      out.append((ours, cam))
    return out

  # --- the flag: off by default, only with the CCNC LFA_ALT camera-SCC config ---
  def test_flag_off_unchanged(self):
    self._init(flag=False, mdps_copy=False)
    self.assertFalse(self._cb_fwd())  # statically blocked, openpilot always sends
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0xCB))
    self.assertTrue(self._mdps_fwd())  # the real MDPS until openpilot's copy appears
    self.assertTrue(self._mdps())
    self.assertFalse(self._mdps_fwd())
    # the camera's lane-keeping signals change nothing: its 0xCB stays blocked, openpilot's frames and its MDPS copy always go
    self.safety.set_controls_allowed(True)
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._mdps_fwd())
    for _ in range(3):
      self.assertTrue(self._cb(False))
      self.assertTrue(self._cb(True))
      self.assertTrue(self._mdps())
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._mdps_fwd())

  def test_flag_off_blocks_the_camera_always(self):
    self._init(flag=False)
    for t, icon, ind in ((0, 0, 3), (2_000_000, 0, 3), (5_000_000, 1, 1), (9_000_000, 2, 2), (12_000_000, 3, 3)):
      self._cam_run(t, t + 2_000_000, icon, ind)
      self.assertFalse(self._cb_fwd(), f"icon {icon} ind {ind}")
      self.assertFalse(self._mdps_fwd(), f"icon {icon} ind {ind}")

  def test_flag_needs_ccnc_lfa_alt_camera_scc(self):
    # outside the CCNC LFA_ALT camera-SCC config the flag does nothing: the camera's 0xCB / MDPS decisions equal the ones without it
    variants = (
      (self.BASE, self.BASE_SP & ~HyundaiSafetyFlagsSP.CCNC),
      (self.BASE, self.BASE_SP & ~HyundaiSafetyFlagsSP.CANFD_LFA_ALT_STEER),
      (self.BASE, self.BASE_SP & ~HyundaiSafetyFlagsSP.CANFD_ANGLE_STEERING),
      (self.BASE & ~HyundaiSafetyFlags.CAMERA_SCC, self.BASE_SP),
      (self.BASE | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG, self.BASE_SP),
      (self.BASE | HyundaiSafetyFlags.CANFD_ALT_BUTTONS, self.BASE_SP),
    )
    for base, base_sp in variants:
      with self.subTest(base=int(base), base_sp=int(base_sp)):
        results = []
        for flag in (False, True):
          self._init(flag=flag, base=base, base_sp=base_sp, mdps_copy=False)
          seen = [self.safety.safety_fwd_hook(2, 0xCB), self.safety.safety_fwd_hook(0, 0xEA)]
          self.safety.set_controls_allowed(True)
          self._cam_run(0, 2_000_000, 0, 3)
          seen += [self.safety.safety_fwd_hook(2, 0xCB), self.safety.safety_fwd_hook(0, 0xEA), self.safety.safety_fwd_hook(2, 0x12A)]
          results.append(seen)
        self.assertEqual(results[0], results[1])

  def test_startup_openpilot_owns_lateral(self):
    self._init(mdps_copy=False)
    self.assertFalse(self._cb_fwd())
    self.assertTrue(self._mdps_fwd())  # the camera needs an MDPS at all times: the real one until openpilot's copy starts
    self.assertFalse(self._mdps())  # our first copy in the same 10 ms cycle as the real one would be a second sender: dropped
    self.safety.set_timer(10_000)
    self.assertTrue(self._mdps())
    self.assertFalse(self._mdps_fwd())  # then only ours
    self._assert_scc_static()
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x12A))  # the camera's 0x12A is forwarded
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x161))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x162))

  def test_init_restarts_the_mdps_forward(self):
    self._init()
    self.assertFalse(self._mdps_fwd())
    self._init(mdps_copy=False)
    self.assertTrue(self._mdps_fwd())

  def test_mdps_copy_only_on_camera_bus(self):
    self._init(mdps_copy=False)
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0xEA, 0, b"\x00" * 24)))
    self.assertTrue(self._mdps_fwd())

  def test_lateral_frames_with_permission(self):
    self._init()
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._cb(True))  # no permission
    self.assertTrue(self._cb(False))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._cb(True))
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._mdps_fwd())
    self.assertTrue(self._mdps())
    self.assertTrue(self._cb(False))  # inactive frames continue: the car must not see a gap
    self._assert_scc_static()

  def test_mads_lateral_only(self):
    self._init()
    self.safety.set_controls_allowed_lateral(True)  # MADS: lateral permission without longitudinal
    self.assertTrue(self._cb(True))
    self.assertFalse(self._cb_fwd())
    self._assert_scc_static()

  # --- openpilot owns lateral from boot: the camera's 0xCB / MDPS never pass unless the nudge window is open ---
  def test_permission_loss_and_lfa_button_keep_the_camera_blocked(self):
    self._init()
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._cb(True))
    self.safety.set_mads_button_press(1)  # lane-assist button, then the heartbeat mismatch drops the permission ~3 s later
    self.assertFalse(self._cb_fwd())
    self.safety.set_timer(3_000_000)
    self.safety.set_controls_allowed_lateral(False)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())
    self.safety.set_timer(30_000_000)
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())
    self.assertTrue(self._cb(False))  # openpilot's inactive frames and its MDPS copy keep flowing
    self.assertTrue(self._mdps())

  def test_paused_inactive_frames_never_let_the_camera_through(self):
    self._init()
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._cb(True))
    for t in range(10_000, 510_000, 10_000):  # paused: inactive frames only, for a long time and across a permission change
      self.safety.set_timer(t)
      self.assertTrue(self._cb(False))
      self.assertFalse(self._cb_fwd() or self._mdps_fwd())
    self.safety.set_controls_allowed_lateral(False)
    self.safety.set_timer(10_000_000)
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())

  def test_inactive_frame_passes_from_boot(self):
    # no permission yet (MADS never on): openpilot's inactive frame and MDPS copy still replace the camera's, which would be LFA
    self._init()
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._mdps_fwd())
    self.assertTrue(self._cb(False))
    self.assertTrue(self._mdps())
    self._assert_scc_static()

  def test_lfa_blocked_from_boot(self):
    # nothing ever set a lateral permission: the camera's LFA (icon 1/2/blink, also with a lane departure) never passes, e.g. after a main press
    self._init()
    for n, (icon, ind) in enumerate(((2, 2), (1, 1), (2, 3), (3, 3), (1, 3))):
      with self.subTest(icon=icon, ind=ind):
        base = n * 10_000_000
        self._cam_run(base, base + 3_000_000, icon, ind)
        self.assertFalse(self._cb_fwd())
        self.assertFalse(self._mdps_fwd())
        self.assertTrue(self._cb(False))
        self.assertTrue(self._mdps())

  def test_lka_passes_from_boot(self):
    self._init()
    self._cam_run(0, 1_400_000, 0, 3)
    self.assertFalse(self._cb_fwd())  # icon 0 for less than the settle time
    self._cam_run(1_400_000, 1_600_000, 0, 3)
    self.assertTrue(self._cb_fwd())  # controls_allowed_lateral was never set
    self.assertTrue(self._mdps_fwd())
    self.assertFalse(self._cb(False))  # one sender
    self.assertFalse(self._mdps())
    self._assert_scc_static()

  def test_no_mixed_sender_from_boot(self):
    for camera_first in (True, False):
      with self.subTest(camera_first=camera_first):
        self._init()
        out = self._sweep(10_000, 6_000_000, lambda t: 2 if 2_000_000 <= t < 3_000_000 else 3 if 3_000_000 <= t < 3_050_000 else 0,
                          lambda t: 3 if t % 100_000 < 60_000 else 2, camera_first=camera_first)
        self.assertTrue(any(cam for _, cam in out))

  # --- the lane-keeping window ---
  def test_lka_frame_passes_with_mads_off(self):
    self._own_lat_then_off()
    self._cam_run(0, 1_400_000, 0, 3)
    self.assertFalse(self._cb_fwd())  # icon 0 for less than the settle time
    self._cam_run(1_400_000, 1_600_000, 0, 3)
    self.assertTrue(self._cb_fwd())  # LKA nudge: icon settled 0, lane departure
    self.assertTrue(self._mdps_fwd())  # the real MDPS follows the camera's frame
    self.assertFalse(self._cb(False))  # and openpilot's own frames are rejected: one sender
    self.assertFalse(self._mdps())
    self._assert_scc_static()

  def test_lfa_frame_blocked_with_mads_off(self):
    self._own_lat_then_off()
    for n, (icon, ind) in enumerate(((2, 2), (2, 1), (1, 1), (2, 3), (1, 3), (3, 3))):  # LFA on/steering/grey, even with a lane departure, or the blink
      with self.subTest(icon=icon, ind=ind):
        base = (n + 1) * 10_000_000
        self._cam_run(base + 2_000_000, base + 5_000_000, icon, ind)
        self.assertFalse(self._cb_fwd())
        self.assertFalse(self._mdps_fwd())
        self.assertTrue(self._cb(False))  # openpilot's inactive frame keeps the car supplied
        self.assertTrue(self._mdps())

  def test_lka_window_needs_icon_settled_and_lane_departure(self):
    self._own_lat_then_off()
    self._cam_run(0, 2_000_000, 0, 2)
    self.assertFalse(self._cb_fwd())  # icon settled but no lane departure: an LFA start has LKA_SysIndReq 1/2
    self._cam_run(2_000_000, 2_100_000, 0, 3)
    self.assertTrue(self._cb_fwd())
    # lane departure ends: the window holds 0.5 s, then closes
    self._cam_run(2_100_000, 2_500_000, 0, 2)
    self.assertTrue(self._cb_fwd())
    self._cam_run(2_500_000, 2_700_000, 0, 2)
    self.assertFalse(self._cb_fwd())
    self.assertTrue(self._cb(False))
    # the LFA on/off blink (3,0,3,0) restarts the settle time: ind 3 with icon 0 right after it does not pass
    self._cam_icon(3)
    self._cam_run(2_700_000, 3_900_000, 0, 3)
    self.assertFalse(self._cb_fwd())
    self._cam_run(3_900_000, 4_300_000, 0, 3)
    self.assertTrue(self._cb_fwd())
    # the camera's LFA turns on (icon 2): closed at once, no frame passes
    self.safety.set_timer(4_310_000)
    self._cam_icon(2)
    self.assertFalse(self._cb_fwd())

  def test_blink_closes_the_window_at_once(self):
    for icon in (1, 2, 3):
      with self.subTest(icon=icon):
        self._open_window_with_cruise_on()
        self.safety.set_timer(2_010_000)
        self._cam_icon(icon)
        self.assertFalse(self._cb_fwd())
        self.assertFalse(self._mdps_fwd())

  def test_lka_window_holds_the_lane_departure_for_half_a_second(self):
    self._own_lat_then_off()
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertTrue(self._cb_fwd())
    for t in range(2_010_000, 2_490_000, 10_000):  # 0x12A goes on with LKA_SysIndReq 0 (no more lane departure), 0x161 stays 0
      self.safety.set_timer(t)
      self._cam_ind(0)
      if t % 50_000 == 0:
        self._cam_icon(0)
      self.assertTrue(self._cb_fwd(), f"t={t}")
    self._cam_run(2_490_000, 2_600_000, 0, 0)
    self.assertFalse(self._cb_fwd())

  def test_stale_0x12a_closes_the_window(self):
    self._own_lat_then_off()
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertTrue(self._cb_fwd())
    for t in range(2_010_000, 2_600_000, 10_000):  # 0x161 keeps coming, 0x12A stops: the last lane departure ages out after 0.5 s
      self.safety.set_timer(t)
      if t % 50_000 == 0:
        self._cam_icon(0)
    self.assertFalse(self._cb_fwd())
    self.assertTrue(self._cb(False))

  def test_missing_icon_frames_close_the_window(self):
    self._init()
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertTrue(self._cb_fwd())
    for t in range(2_000_000, 2_700_000, 10_000):  # 0x12A keeps coming, 0x161 stops
      self.safety.set_timer(t)
      self._cam_ind(3)
    self.assertFalse(self._cb_fwd())
    self.assertTrue(self._cb(False))
    self._cam_icon(0)  # 0x161 is back: the run restarts, no window until it settled again
    self._cam_run(2_700_000, 4_000_000, 0, 3)
    self.assertFalse(self._cb_fwd())
    self._cam_run(4_000_000, 4_300_000, 0, 3)
    self.assertTrue(self._cb_fwd())

  def test_no_mixed_frame_leak_around_the_window(self):
    self._own_lat_then_off()
    # a long-settled icon 0, then an LFA start (icon 0 frames with LKA_SysIndReq 1, then the icon): never forwarded
    self._cam_run(0, 3_000_000, 0, 1)
    for t in range(3_000_000, 3_300_000, 10_000):
      self.safety.set_timer(t)
      self._cam_ind(1)
      self.assertFalse(self._cb_fwd())
      self.assertFalse(self._mdps_fwd())
    self._cam_icon(2)
    self._cam_run(3_300_000, 4_000_000, 2, 2)
    self.assertFalse(self._cb_fwd())
    # every state: either the camera's frame or openpilot's, never both
    for t in range(4_000_000, 7_000_000, 50_000):
      self.safety.set_timer(t)
      self._cam_icon(0 if t > 4_500_000 else 2)
      self._cam_ind(3 if t % 200_000 == 0 else 2)
      self.assertNotEqual(self._cb_fwd(), self._cb(False))

  def test_init_resets_the_window(self):
    self._init()
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertTrue(self._cb_fwd())
    self._init()
    self.safety.set_timer(2_000_000)
    self.assertFalse(self._cb_fwd())  # no camera frames seen since init: the camera stays blocked

  # --- openpilot steering closes the window ---
  def test_window_follows_openpilot_steering_not_its_permission(self):
    """Route 48 35:59-37:02 (T23): openpilot cruise on (controls_allowed), steering off by the LFA button, openpilot's frames inactive. The camera's
    nudge must reach the car: the window opens once openpilot has sent no accepted ACTIVE frame for 0.2 s, whatever the permission says, and stays
    shut while openpilot steers. Before, the window needed both permissions off, and openpilot (which had stopped sending for the nudge) left the car
    with no 0xCB for 0.85 s; the EPS flagged 0.53 s into each gap."""
    for allow in ("controls", "lateral"):
      with self.subTest(allow=allow):
        self._init()
        getattr(self.safety, f"set_controls_allowed{'_lateral' if allow == 'lateral' else ''}")(True)
        self.assertTrue(self._cb(True))  # openpilot steers at t=0
        self._cam_run(0, 1_600_000, 0, 3)
        self.assertTrue(self._cb_fwd())  # 1.6 s without an ACTIVE frame: the nudge passes, permission or not
        self.assertTrue(self._mdps_fwd())
        self.assertFalse(self._cb(False))  # one sender
        self.assertFalse(self._mdps())

  def test_openpilot_steering_keeps_the_window_shut(self):
    self._init()
    self.safety.set_controls_allowed(True)
    for t in range(0, 3_000_000, 10_000):  # an accepted ACTIVE frame every cycle: the camera's lane departure never passes
      self.safety.set_timer(t)
      if t % 50_000 == 0:
        self._cam_icon(0)
      self._cam_ind(3)
      self.assertFalse(self._cb_fwd(), f"t={t}")
      self.assertTrue(self._cb(True), f"t={t}")
    self.assertFalse(self._mdps_fwd())

  def test_rejected_active_frames_do_not_hold_the_window_shut(self):
    """An ACTIVE frame the panda rejects (no permission) is not openpilot steering: it must not close the window, or the car would get neither."""
    self._init()
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertTrue(self._cb_fwd())
    self.assertFalse(self._cb(True))  # no permission
    self.assertTrue(self._cb_fwd())

  def test_active_frame_outside_the_limits_does_not_take_the_axis(self):
    self._init()
    self.safety.set_controls_allowed(True)
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertTrue(self._cb_fwd())
    too_far = self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": 2, "ADAS_StrAnglReqVal": 300})
    self.assertFalse(self.safety.safety_tx_hook(too_far))  # refused as before and not openpilot steering: the nudge continues
    self.safety.set_timer(2_010_000)
    self.assertTrue(self._cb_fwd())

  def test_lka_window_opens_0_2_s_after_the_last_active_frame(self):
    self._init()
    self.safety.set_controls_allowed(True)
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertTrue(self._cb_fwd())    # the camera's nudge steers
    self.safety.set_timer(2_010_000)
    self.assertFalse(self._cb(True))   # openpilot steers: this frame is the one dropped at the switch
    self.assertFalse(self._cb_fwd())   # the camera's next frame is blocked: openpilot's now
    self.safety.set_timer(2_020_000)
    self.assertTrue(self._cb(True))    # last ACTIVE frame at 2.02 s
    self._cam_run(2_030_000, 2_210_000, 0, 3)
    self.assertFalse(self._cb_fwd())   # 0.19 s after the last ACTIVE frame: still openpilot's
    self.assertTrue(self._cb(False))
    self._cam_run(2_220_000, 2_230_000, 0, 3)
    self.assertFalse(self._cb(False))  # 0.21 s: openpilot is not steering; its frame is the one dropped at the switch
    self.assertTrue(self._cb_fwd())    # the camera's
    self.assertFalse(self._cb(False))

  def test_the_idle_boundary_inside_one_cycle_never_gives_two(self):
    """R1: openpilot's inactive frame at last ACTIVE + 199.9 ms, the camera's at + 200.1 ms in the same cycle: the camera takes over only after a
    cycle without openpilot's frame."""
    self._init()
    self.safety.set_controls_allowed(True)
    self._cam_run(0, 2_000_000, 0, 3)
    self.safety.set_timer(2_000_000)
    self.assertTrue(self._cb(True))
    self._cam_run(2_010_000, 2_190_000, 0, 3)
    self.safety.set_timer(2_199_900)
    self.assertTrue(self._cb(False))
    self.safety.set_timer(2_200_100)
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._mdps_fwd())

  def test_steering_timestamp_expires_across_a_timer_wrap(self):
    """R1 F2: after the 0.2 s idle the steering mark is retired, so 71.6 min later (one timer wrap) the old timestamp does not look recent again."""
    self._init()
    self.safety.set_controls_allowed(True)
    T = 10_000_000
    self.safety.set_timer(T)
    self.assertTrue(self._cb(True))
    self._cam_run(T + 10_000, T + 2_000_000, 0, 3)
    self.assertFalse(self._cb(False))
    self.assertTrue(self._cb_fwd())     # the camera's, mark retired
    self._cam_run(0xFFF00000, 0xFFFFFFFF, 0, 3)
    self._cam_run(0, T - 10_000, 0, 3)
    for t in range(T, T + 300_000, 10_000):
      self.safety.set_timer(t)
      self._cam_icon(0)
      self._cam_ind(3)
      self.assertTrue(self._cb_fwd(), f"t={t}")
      self.assertTrue(self._mdps_fwd(), f"t={t}")

  def test_no_mixed_sender_with_cruise_on_and_steering_off(self):
    """Every 10 ms with openpilot cruise on and steering off, both bus orders: never two senders, never two cycles without one (icon 0 / LFA /
    blink, lane departure on and off)."""
    for camera_first in (True, False):
      with self.subTest(camera_first=camera_first):
        self._init()
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._cb(True))
        out = self._sweep(10_000, 6_000_000, lambda t: 2 if 2_000_000 <= t < 3_000_000 else 3 if 3_000_000 <= t < 3_050_000 else 0,
                          lambda t: 3 if t % 100_000 < 60_000 else 2, camera_first=camera_first)
        self.assertTrue(any(cam for _, cam in out))  # the nudge actually passed

  # --- the hand-over cycles: one 0xCB per cycle, never two senders ---
  def test_takeover_handover_camera_first_in_the_cycle(self):
    """Openpilot starts steering inside the window and the camera's frame comes first in every cycle: the camera's frame of the hand-over cycle
    passes, ours is dropped and closes the window; from the next cycle only ours. Exactly one 0xCB per cycle."""
    self._open_window_with_cruise_on()
    self.safety.set_timer(2_010_000)
    self.assertTrue(self._cb_fwd())
    self.assertFalse(self._cb(True))  # dropped, the window closes
    for t in range(2_020_000, 2_300_000, 10_000):
      self.safety.set_timer(t)
      self._cam_ind(3)
      self.assertFalse(self._cb_fwd(), f"t={t}")
      self.assertTrue(self._cb(True), f"t={t}")

  def test_takeover_handover_ours_first_in_the_cycle(self):
    """Same, ours comes first: ours is dropped and closes the window, so the camera's frame of that cycle is blocked too: one empty cycle (a 20 ms gap
    at most; never two senders), then only ours."""
    self._open_window_with_cruise_on()
    self.safety.set_timer(2_010_000)
    self.assertFalse(self._cb(True))
    self.assertFalse(self._cb_fwd())  # the one empty cycle
    for t in range(2_020_000, 2_300_000, 10_000):
      self.safety.set_timer(t)
      self._cam_ind(3)
      self.assertTrue(self._cb(True), f"t={t}")
      self.assertFalse(self._cb_fwd(), f"t={t}")

  def test_camera_status_between_the_two_senders_never_gives_two(self):
    """R1 F1: the camera's 0xCB has passed this cycle, then a status frame closes the window (icon 1 / 2 / blink) before openpilot's frame:
    openpilot's frame (ACTIVE or not) is the one dropped at the switch, the camera's next frame is blocked, openpilot's next passes."""
    for icon in (1, 2, 3):
      for active in (True, False):
        with self.subTest(icon=icon, active=active):
          self._open_window_with_cruise_on()
          self.safety.set_timer(2_010_000)
          self.assertTrue(self._cb_fwd())
          self._cam_icon(icon)
          self.assertFalse(self._cb(active))
          self.assertFalse(self._mdps())
          self.safety.set_timer(2_020_000)
          self.assertFalse(self._cb_fwd())
          self.assertTrue(self._cb(active))

  def test_silent_openpilot_gets_the_nudge_at_once_after_a_switch_back(self):
    """R1 v3 F1: the camera steers the nudge at 100 Hz; the indicator stops (last 3 at 2.0 s), openpilot resumes inactive frames (its 0.3 s hold
    is shorter, dropped while the camera owns), the panda's 0.5 s hold lapses at the camera frame of 2.51 s (blocked, slot back to openpilot), a new
    indicator 3 comes right after and openpilot goes silent again: the camera's 2.52 s frame passes (one missing frame, not two)."""
    self._init()
    self.safety.set_controls_allowed(True)
    self._cam_run(0, 2_000_000, 0, 3)
    for t in range(2_000_000, 2_510_000, 10_000):
      self.safety.set_timer(t)
      if t % 50_000 == 0:
        self._cam_icon(0)
      if t > 2_000_000:
        self._cam_ind(2)
      self.assertTrue(self._cb_fwd(), f"t={t}")
      if t >= 2_310_000:
        self.assertFalse(self._cb(False), f"t={t}")  # openpilot's inactive frames while the camera owns: dropped
    self.safety.set_timer(2_510_000)
    self.assertFalse(self._cb_fwd())  # the panda's hold lapsed: blocked, the slot is openpilot's
    self.safety.set_timer(2_510_100)
    self._cam_ind(3)                  # a new nudge frame; openpilot goes silent (its window reopens)
    self.safety.set_timer(2_520_000)
    self.assertTrue(self._cb_fwd())   # nothing of openpilot's to drop: the camera's frame passes at once

  def test_the_camera_that_stops_hands_back_without_its_frame(self):
    self._open_window_with_cruise_on()
    self.safety.set_timer(2_010_000)
    self.assertTrue(self._cb_fwd())
    self.safety.set_timer(2_010_000 + 30_000)  # the camera sent nothing for more than 2.5 cycles: openpilot's frame goes
    self.assertTrue(self._cb(False))
    self.assertFalse(self._cb_fwd())

  # --- the MDPS copy follows the owner ---
  def test_mdps_copy_not_accepted_after_the_real_one_at_a_takeover(self):
    """R1: the camera's nudge steers and the real MDPS reaches the camera; openpilot takes over: its ACTIVE 0xCB is dropped, and its MDPS copy of
    the same cycle must be dropped too (the real one already went to the camera), from the next cycle only the copy."""
    self._open_window_with_cruise_on()
    self.safety.set_timer(2_010_000)
    self.assertTrue(self._cb_fwd())
    self.assertTrue(self._mdps_fwd())
    self.assertFalse(self._cb(True))
    self.assertFalse(self._mdps())
    self.safety.set_timer(2_020_000)
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._mdps_fwd())
    self.assertTrue(self._cb(True))
    self.assertTrue(self._mdps())

  def test_mdps_real_then_switch_then_copy_in_one_cycle(self):
    """R1 v2 / fuzz seed 0: the real MDPS passes while the camera owns the slot, an icon frame closes the window, the camera's 0xCB is blocked
    (owner back to openpilot), then openpilot's MDPS copy in the same cycle: dropped; the next cycle only the copy."""
    self._open_window_with_cruise_on()
    self.safety.set_timer(2_010_000)
    self.assertTrue(self._cb_fwd())
    self.safety.set_timer(2_020_000)
    self.assertTrue(self._mdps_fwd())
    self._cam_icon(2)
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._mdps())
    self.safety.set_timer(2_030_000)
    self.assertFalse(self._mdps_fwd())
    self.assertTrue(self._mdps())

  def test_mdps_marks_expire_across_a_timer_wrap(self):
    """R1 v3: the MDPS sender marks are retired once 7 ms passed (at the next MDPS frame, 100 Hz in the car), so one timer wrap later an old mark
    does not alias to 'recent' and block the other sender."""
    self._open_window_with_cruise_on()
    self.safety.set_timer(2_010_000)
    self.assertTrue(self._cb_fwd())
    self.assertTrue(self._mdps_fwd())  # real MDPS mark at 2.01 s
    self.safety.set_timer(2_030_000)
    self.assertFalse(self._mdps())     # the camera's slot: the copy is dropped; the 2.01 s mark is retired here
    self.safety.set_timer((2_010_000 + 0x100000000 + 1_000) & 0xFFFFFFFF)  # one wrap later: the old mark would alias to 1 ms
    self._cam_icon(2)
    self.assertFalse(self._cb_fwd())   # the window closed: the slot is openpilot's
    self.assertTrue(self._mdps())       # the copy passes: no stale real-MDPS mark

  def test_mdps_follows_the_0xcb_owner_in_both_orders(self):
    """R1: the camera gets the real MDPS only while its own 0xCB steers, openpilot's copy only while openpilot's does; at a switch it may miss one,
    never get two (route 48 data: real ACI state 1, the copy 2)."""
    for camera_first in (True, False):
      with self.subTest(camera_first=camera_first):
        self._init()
        self.safety.set_controls_allowed(True)
        self.assertTrue(self._cb(True))
        self._sweep(10_000, 6_000_000, lambda t: 0, lambda t: 3 if 2_000_000 <= t < 4_000_000 else 2,
                    active_at=lambda t: t < 1_500_000 or t >= 3_500_000, camera_first=camera_first)

  def test_random_bus_orders_never_give_two_senders(self):
    """Fuzz (seeded): random event order inside each 10 ms cycle (camera status, the camera's 0xCB, ours, both MDPS), timer jitter, dropped camera
    frames, permission flips, openpilot ACTIVE / inactive / silent, icon and nudge changes. Never two 0xCB or two MDPS in a cycle; never two cycles
    in a row without a 0xCB while both senders tried with valid frames."""
    import random
    for seed in range(20):
      rng = random.Random(seed)
      self._init()
      t = rng.randrange(0, 0xFFFFFFFF)
      icon, ind, allowed, mode, prev_empty = 0, 2, rng.random() < 0.5, 'silent', False
      for c in range(2000):
        t = (t + 10_000 + rng.randint(-1500, 1500)) & 0xFFFFFFFF
        allowed = (not allowed) if rng.random() < 0.003 else allowed
        mode = rng.choice(['active', 'inactive', 'silent']) if rng.random() < 0.01 else mode
        icon = rng.choice([0, 0, 0, 1, 2, 3]) if rng.random() < 0.004 else icon
        ind = rng.choice([2, 3, 3, 1]) if rng.random() < 0.02 else ind
        self.safety.set_controls_allowed(allowed)
        events = ['icon', 'ind', 'cam', 'ours', 'real_mdps', 'ours_mdps']
        rng.shuffle(events)
        cam_sends = rng.random() > 0.02
        got = dict.fromkeys(events, False)
        for e in events:
          self.safety.set_timer(t)
          t = (t + rng.randint(0, 600)) & 0xFFFFFFFF
          if e == 'icon' and (c % 5 == 0 or rng.random() < 0.01):
            self._cam_icon(icon)
          elif e == 'ind':
            self._cam_ind(ind)
          elif e == 'cam' and cam_sends:
            got[e] = self._cb_fwd()
          elif e == 'ours' and mode != 'silent':
            got[e] = bool(self._cb(mode == 'active'))
          elif e == 'real_mdps':
            got[e] = self._mdps_fwd()
          elif e == 'ours_mdps' and mode != 'silent':
            got[e] = bool(self._mdps())
        where = f"seed {seed} cycle {c} (mode {mode}, allowed {allowed}, icon {icon}, ind {ind}, order {events})"
        self.assertFalse(got['ours'] and got['cam'], "two 0xCB senders, " + where)
        self.assertFalse(got['ours_mdps'] and got['real_mdps'], "two MDPS senders, " + where)
        empty = cam_sends and (mode == 'inactive' or (mode == 'active' and allowed)) and not (got['ours'] or got['cam'])
        self.assertFalse(empty and prev_empty, "two cycles without 0xCB, " + where)
        prev_empty = empty

  # --- rx checks, relay, tx list ---
  def _feed_required(self, t0, t1):
    """the rx messages every config of this class needs (not the camera's lane-keeping signals), every 10 ms from t0 to t1"""
    for t in range(t0, t1, 10_000):
      self.safety.set_timer(t)
      for name, bus in (("ACCELERATOR_BRAKE_ALT", 0), ("TCS", 0), ("WHEEL_SPEEDS", 0), ("MDPS", 0), ("CRUISE_BUTTONS", 0), ("SCC_CONTROL", 2)):
        self.safety.safety_rx_hook(self.packer.make_can_msg_safety(name, bus, {}))

  def test_optional_rx_checks_never_disable_controls(self):
    for camera in ("never", "once", "stops"):
      with self.subTest(camera=camera):
        self._init()
        self._feed_required(0, 100_000)
        if camera != "never":
          self._cam_icon(0)
          self._cam_ind(3)
        self.assertTrue(self.safety.safety_config_valid())
        for second in range(1, 4):  # the camera's 0x161 / 0x12A are missing for seconds
          self._feed_required(second * 1_000_000, second * 1_000_000 + 100_000)
          if camera == "stops" and second == 1:
            self._cam_icon(1)
          self.safety.set_controls_allowed(True)
          self.safety.safety_tick()
          self.assertTrue(self.safety.get_controls_allowed(), f"second {second}")
          self.assertTrue(self.safety.safety_config_valid(), f"second {second}")
        self.assertFalse(self._cb_fwd())  # and the window is closed: the camera stays blocked

  def test_required_rx_checks_still_disable_controls(self):
    self._init()
    self._feed_required(0, 100_000)
    self.safety.set_timer(3_000_000)  # the required messages are gone (the optional ones are not what trips this)
    self.safety.set_controls_allowed(True)
    self.safety.safety_tick()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_relay_malfunction(self):
    for addr, bus, length, expected in ((0xCB, 0, 24, True), (0xCB, 2, 24, False), (0xEA, 2, 24, True), (0xEA, 0, 24, False),
                                        (0x1A0, 0, 32, self.LONG), (0x1A0, 2, 32, False), (0x161, 0, 32, True)):
      with self.subTest(addr=hex(addr), bus=bus):
        self._init()
        self._rx(addr, bus, length)
        self.assertEqual(expected, self.safety.get_relay_malfunction())

  def test_camera_frames_forwarded_do_not_trip_relay(self):
    self._init()
    for t in range(0, 4_000_000, 10_000):
      self.safety.set_timer(t)
      if t % 50_000 == 0:
        self._cam_icon(0)
      self._cam_ind(3)
      self._cb_fwd()
      self._mdps_fwd()
    self.assertFalse(self.safety.get_relay_malfunction())

  def test_relay_malfunction_blocks_all_forwarding(self):
    self._open_window_with_cruise_on()
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self._cb_fwd() or self._mdps_fwd() or self._scc_fwd())

  def test_camera_lane_keeping_frames_still_forwarded(self):
    self._init()
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x12A))  # LKA_SysIndReq / lane departure warnings: untouched
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x161))  # blocked as upstream does: openpilot sends its own
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x162))
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x12A))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x161))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x162))

  def test_openpilot_tx_checks_unchanged(self):
    self._init()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self.safety.safety_tx_hook(self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": 2, "ADAS_StrAnglReqVal": 300})))
    self.assertFalse(self._cb_fwd())  # openpilot owns lateral from init anyway
    for sig in ("ADAS_ActvACISta", "FCA_ESA_ActvSta"):
      self.assertFalse(self.safety.safety_tx_hook(self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": 2, sig: 1})), sig)
    self.assertTrue(self._cb(True))

  def test_tx_set(self):
    self._init()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x12A, 0, b"\x00" * 16)))  # the camera keeps 0x12A
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0xEA, 0, b"\x00" * 24)))  # wrong bus
    self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0xCB, 2, b"\x00" * 24)))  # wrong bus

  WRAP = 2 ** 32  # the panda's microsecond timer is unsigned 32 bit: ~71.6 min

  def _steady_icon_through_the_wrap(self, ind3, after=0):
    """camera 0x161 (icon 0) every 0.4 s for more than a timer wrap, 0x12A LKA_SysIndReq 3 whenever ind3(t) says so; every 0.4 s the forward decision of
    the camera's 0xCB is asked (the 0xCB path runs at 100 Hz on the car). Returns that decision at the first step `after` us past the wrap."""
    self._init()
    t = 0
    while True:
      self.safety.set_timer(t % self.WRAP)
      self._cam_icon(0)
      if ind3(t):
        self._cam_ind(3)
      decision = self._cb_fwd()
      if t >= self.WRAP + after:
        return decision
      t += 400_000

  def test_settled_icon_survives_the_timer_wrap(self):
    """R1 P1 4: the settle time (icon 0 for 1.5 s) was compared as an elapsed time from the first zero icon: ~71.6 min of steady zero icon later it
    wrapped and closed the nudge window for 1.5 s. It is latched now. A lane departure right after the wrap opens the window."""
    self.assertTrue(self._steady_icon_through_the_wrap(lambda t: t >= self.WRAP))

  def test_lane_departure_hold_does_not_revive_after_a_timer_wrap(self):
    """R1 P2 5: one LKA_SysIndReq 3 at 2.0 s, then none: its hold (0.5 s) was only compared as an elapsed time, so 2^32 us later the old event looked
    recent again and opened the window. It is cleared when the hold expires."""
    self.assertFalse(self._steady_icon_through_the_wrap(lambda t: 2_000_000 <= t < 2_400_000, after=2_000_000))


class TestHyundaiCanfdCameraHandbackLong(TestHyundaiCanfdCameraHandback):
  """The same with openpilot longitudinal: 0x1A0 stays upstream's static block, the lateral hand-back does not depend on it."""

  LONG = True


if __name__ == "__main__":
  unittest.main()
