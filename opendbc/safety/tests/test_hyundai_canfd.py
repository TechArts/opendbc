#!/usr/bin/env python3
from opendbc.testing import parameterized_class, parameterized
import unittest
import numpy as np

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from opendbc.car.hyundai.carcontroller import ANGLE_SAFETY_BASELINE_MODEL
from opendbc.car.hyundai.values import HyundaiSafetyFlags, CAR, HyundaiFlags, CarControllerParams
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, away_round, round_speed
from opendbc.safety.tests.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm, ISO_LATERAL_ACCEL, AngleSteeringLimits
from opendbc.car.hyundai.interface import CarInterface

# All combinations of radar/camera-SCC and gas/hybrid/EV cars
ALL_GAS_EV_HYBRID_COMBOS = [
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
]

ALL_GAS_EV_HYBRID_COMBOS_CCNC = [
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
]


def round_angle(angle_deg: float, can_offset=0):
  scaled = angle_deg / 0.1
  scaled += can_offset
  return int(scaled) * 0.1


class TestHyundaiCanfdBase(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

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

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE = 270

  MAX_RT_DELTA = 112
  RT_INTERVAL = 250000

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2
  MIN_VALID_STEERING_RT_INTERVAL = 810000  # a ~10% buffer, can send steer up to 110Hz

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdTorqueSteering":
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, 0)
    self.safety.init_tests()


class TestHyundaiCanfdAngleSteering(TestHyundaiCanfdBase, common.AngleSteeringSafetyTest):
  PLATFORMS = {str(platform): platform for platform in CAR if
               platform.config.flags & HyundaiFlags.CANFD_ANGLE_STEERING and not CarInterface.get_non_essential_params(str(platform)).dashcamOnly}

  # Angle control limits
  BASELINE_PANDA_ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    360,  # degrees (safe upper bound for command, allowing some margin)
    ([], []),
    ([], []),
    MAX_LATERAL_ACCEL=(ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * 0.06)),  # ~3.0 m/s^2
    MAX_LATERAL_JERK=(3.0 + (ACCELERATION_DUE_TO_GRAVITY * 0.06)),  # ~3.0 m/s^3,
    MAX_ANGLE_RATE=5  # comfort rate limit for angle commands, in degrees per frame.
  )

  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10
  ANGLE_SAFETY_THRESHOLD_PCT = -2.0  # Fail if difference is less than -2%

  # Panda safety has hardcoded 0.06 superelevation for road roll, we test safety with that.
  AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll lowers lateral acceleration
  MAX_LATERAL_ACCEL=(ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL))
  MAX_LATERAL_JERK=(3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL))

  # Hyundai uses get_max_angle_delta and get_max_angle for real lateral accel and jerk limits
  # TODO: integrate this into AngleSteeringSafetyTest
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 100  # Hz

  cnt_angle_cmd = 0

  def get_baseline_limits(self):
    limits = CarControllerParams(CarInterface.get_non_essential_params(ANGLE_SAFETY_BASELINE_MODEL))
    limits.ANGLE_LIMITS = self.BASELINE_PANDA_ANGLE_LIMITS
    return limits

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True, gain: float = 0.0):
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    values = {"ADAS_StrAnglReqVal": angle, "LKAS_ANGLE_ACTIVE": 2 if enabled else 1,
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
      cls.safety = None
      raise unittest.SkipTest

  def get_vm(self, car_name):
    return VehicleModel(CarInterface.get_non_essential_params(car_name))

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ANGLE_STEERING)
    self.safety.init_tests()

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
        angl = get_max_angle_vm(speed, self.get_vm(car_name), self.get_baseline_limits())
        max_angle = round_angle(get_max_angle_vm(speed, self.get_vm(car_name), self.get_baseline_limits()), 1) * sign
        max_angle = np.clip(max_angle, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)), f"{angl} -- {max_angle}")

        # above limit (offset 6 to reliably exceed C float tolerance)
        max_angle_raw = round_angle(get_max_angle_vm(speed, self.get_vm(car_name), self.get_baseline_limits()), 6) * sign
        max_angle = np.clip(max_angle_raw, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self._tx(self._angle_cmd_msg(max_angle, True))

        # at low speeds max angle is above 360, so adding 1 has no effect
        should_tx = abs(max_angle_raw) >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(max_angle, True)), f"should_tx: {should_tx}, max_angle: {max_angle}, speed: {speed}")

  def test_lateral_jerk_limit(self):
    car_name = ANGLE_SAFETY_BASELINE_MODEL
    for speed in np.linspace(0, 40, 100):
      speed = round_speed(away_round(speed / 0.03125 * 3.6) * 0.03125 / 3.6)
      speed = max(speed, 1)
      for sign in (-1, 1):  # (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)  # safety fudges the speed
        self._tx(self._angle_cmd_msg(0, True))

        # Stay within limits
        # Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.get_vm(car_name), self.get_baseline_limits())) * sign
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates
        # Up
        # TODO-SP: Why do I need to set a can_offset so high to pass the tests and why tesla only does +1? and why does it seem to differ based on the baseline?
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.get_vm(car_name), self.get_baseline_limits()), 6) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)), vars(self.get_baseline_limits()))

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
      current_max_angle = get_max_angle_vm(speed, current_vm, self.get_baseline_limits())

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
    baseline_limits = CarControllerParams(CarInterface.get_non_essential_params(baseline_car))
    current_vm = self.get_vm(car_name)
    current_limits = CarControllerParams(CarInterface.get_non_essential_params(car_name))

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


class TestHyundaiCanfdLFASteeringBase(TestHyundaiCanfdTorqueSteering):

  TX_MSGS = [[0x12A, 0], [0x1A0, 1], [0x1CF, 0], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0)}  # LFA, LFAHDA_CLUSTER
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0]}

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2
  SAFETY_PARAM: int

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdLFASteering", "TestHyundaiCanfdLFASteeringAltButtons"):
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.SAFETY_PARAM)
    self.safety.init_tests()


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteering(TestHyundaiCanfdLFASteeringBase):
  pass


class TestHyundaiCanfdLFASteeringAltButtonsBase(TestHyundaiCanfdLFASteeringBase):

  SAFETY_PARAM: int

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

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

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}  # LKAS, CAM_0x2A4
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()


# TODO: Handle ICE and HEV configurations once we see cars that use the new messages
class TestHyundaiCanfdLKASteeringAltEVBase(TestHyundaiCanfdBase):

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}  # LKAS_ALT, CAM_0x362
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringAltEVTorque(TestHyundaiCanfdLKASteeringAltEVBase, TestHyundaiCanfdTorqueSteering):

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringAltAngle(TestHyundaiCanfdAngleSteering):

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG |
                                 HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT | HyundaiSafetyFlags.CANFD_ANGLE_STEERING)
    self.safety.init_tests()

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


class TestHyundaiCanfdLKASteeringLongEV(HyundaiLongitudinalBase, TestHyundaiCanfdLKASteeringEV):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4), 1: (0x1a0,)}  # LKAS, CAM_0x2A4, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEER_MSG |
                                 HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

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

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongBase":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  # no knockout
  def test_tester_present_allowed(self):
    pass


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLong(TestHyundaiCanfdLFASteeringLongBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLong":
      cls.safety = None
      raise unittest.SkipTest


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLongAltButtons(TestHyundaiCanfdLFASteeringLongBase, TestHyundaiCanfdLFASteeringAltButtonsBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongAltButtons":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def test_acc_cancel(self):
    # Alt buttons does not use SCC_CONTROL to cancel if longitudinal
    pass


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS_CCNC)
class TestHyundaiCanfdLFASteeringCCNC(TestHyundaiCanfdLFASteeringBase):

  TX_MSGS = [[0x12A, 0], [0x1E0, 0], [0x1CF, 2], [0x7C4, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x161, 0x162), 2: (0x7C4,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0, 0x161, 0x162], 0: [0x7C4]}

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringCCNC":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CCNC | self.SAFETY_PARAM)
    self.safety.init_tests()

  def test_ccnc(self):
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("CCNC_0x161", self.STEER_BUS, {})))
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("CCNC_0x162", self.STEER_BUS, {})))

  def test_tx_hook_on_wrong_safety_mode(self):
    from opendbc.safety.tests.common import make_msg
    import importlib
    for test_name in ["TestElm327"]:
      mod = importlib.import_module("opendbc.safety.tests.test_" + test_name.replace("Test", "").lower())
      tx_list = [m for m in getattr(mod, test_name).TX_MSGS if m[0] != 0x7C4]  # skip overlapping 0x7C4 from Elm327
      for addr, bus in tx_list:
        if [addr, bus] not in self.TX_MSGS:
          self.assertFalse(self._tx(make_msg(bus, addr)), f"allowed TX {addr=:#x} {bus=}")


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS_CCNC)
class TestHyundaiCanfdLFASteeringLongCCNC(TestHyundaiCanfdLFASteeringLongBase):

  TX_MSGS = [[0x12A, 0], [0x1E0, 0], [0x1CF, 2], [0x7C4, 2], [0x1A0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x161, 0x162, 0x1A0), 2: (0x7C4,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0, 0x161, 0x162, 0x1A0], 0: [0x7C4]}

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongCCNC":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CCNC | HyundaiSafetyFlags.LONG | self.SAFETY_PARAM)
    self.safety.init_tests()

  def test_ccnc(self):
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("CCNC_0x161", self.STEER_BUS, {})))
    self.assertTrue(self._tx(self.packer.make_can_msg_safety("CCNC_0x162", self.STEER_BUS, {})))

  def test_tx_hook_on_wrong_safety_mode(self):
    from opendbc.safety.tests.common import make_msg
    import importlib
    for test_name in ["TestElm327"]:
      mod = importlib.import_module("opendbc.safety.tests.test_" + test_name.replace("Test", "").lower())
      tx_list = [m for m in getattr(mod, test_name).TX_MSGS if m[0] != 0x7C4]  # skip overlapping 0x7C4 from Elm327
      for addr, bus in tx_list:
        if [addr, bus] not in self.TX_MSGS:
          self.assertFalse(self._tx(make_msg(bus, addr)), f"allowed TX {addr=:#x} {bus=}")


class TestHyundaiCanfdLFAAltSteering(unittest.TestCase):
  """HDA1 angle steering through LFA_ALT (0xCB) only: the camera keeps 0x12A (status/warnings, no actuation)."""

  # CCNC variant (superset); read by common.test_tx_hook_on_wrong_safety_mode
  TX_MSGS = [[0x1CF, 2], [0x1E0, 0], [0xCB, 0], [0x1A0, 0], [0x160, 0], [0x161, 0], [0x162, 0], [0x7C4, 2], [0xEA, 2]]

  BASE = HyundaiSafetyFlags.CANFD_ANGLE_STEERING | HyundaiSafetyFlags.CAMERA_SCC
  FLAG = HyundaiSafetyFlags.CANFD_LFA_ALT_STEERING

  def _init(self, param):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, int(param))
    self.safety.init_tests()

  def _tx(self, addr, bus, length):
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, b"\x00" * length))

  def _rx(self, addr, bus, length):
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, b"\x00" * length))

  def test_camera_lfa_forwarded_and_not_sendable(self):
    for ccnc in (0, HyundaiSafetyFlags.CCNC):
      with self.subTest(ccnc=bool(ccnc)):
        self._init(self.BASE | ccnc | self.FLAG)
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x12A))
        self.assertFalse(self._tx(0x12A, 0, 16))
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0xCB))
        self.assertTrue(self._tx(0xCB, 0, 24))  # inactive, zero angle

  def test_relay_malfunction(self):
    for addr, length, expected in ((0x12A, 16, False), (0xCB, 24, True)):
      with self.subTest(addr=hex(addr)):
        self._init(self.BASE | HyundaiSafetyFlags.CCNC | self.FLAG)
        self._rx(addr, 0, length)
        self.assertEqual(expected, self.safety.get_relay_malfunction())

  def test_flag_requires_angle_steering(self):
    self._init(HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CCNC | self.FLAG)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x12A))

  def test_without_flag_unchanged(self):
    self._init(self.BASE | HyundaiSafetyFlags.CCNC)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x12A))

  def test_ccnc_mdps_copy_replaces_real_mdps(self):
    self._init(self.BASE | HyundaiSafetyFlags.CCNC | self.FLAG)
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0xEA))  # real MDPS reaches the camera until openpilot sends a copy
    self.assertTrue(self._tx(0xEA, 2, 24))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0xEA))  # then only openpilot's copy
    self._init(self.BASE | HyundaiSafetyFlags.CCNC | self.FLAG)
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0xEA))  # init resets

  def test_mdps_copy_not_allowed_elsewhere(self):
    for param in (self.BASE | self.FLAG, self.BASE | HyundaiSafetyFlags.CCNC):
      with self.subTest(param=int(param)):
        self._init(param)
        self.assertFalse(self._tx(0xEA, 2, 24))
        self.assertEqual(2, self.safety.safety_fwd_hook(0, 0xEA))

  def test_relay_malfunction_on_camera_mdps(self):
    self._init(self.BASE | HyundaiSafetyFlags.CCNC | self.FLAG)
    self._rx(0xEA, 2, 24)
    self.assertTrue(self.safety.get_relay_malfunction())


class TestHyundaiCanfdLFAAltSteeringLong(unittest.TestCase):
  """Openpilot longitudinal (SCC_CONTROL) on top of HDA1 LFA_ALT angle steering with camera SCC."""

  BASE = HyundaiSafetyFlags.CANFD_ANGLE_STEERING | HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_LFA_ALT_STEERING
  LONG = HyundaiSafetyFlags.LONG
  CCNC = HyundaiSafetyFlags.CCNC

  # superset of both variants; read by common.test_tx_hook_on_wrong_safety_mode
  TX_MSGS = [[0x1CF, 2], [0x1E0, 0], [0xCB, 0], [0x1A0, 0], [0x160, 0], [0x161, 0], [0x162, 0], [0x7C4, 2], [0xEA, 2]]

  # (addr, bus, length) openpilot may send; 0x1CF is gated on button state, covered separately
  TX_CCNC = {(0x1E0, 0, 16), (0xCB, 0, 24), (0x1A0, 0, 32), (0x161, 0, 32), (0x162, 0, 32), (0x7C4, 2, 8), (0xEA, 2, 24)}
  TX_NON_CCNC = {(0x1E0, 0, 16), (0xCB, 0, 24), (0x1A0, 0, 32), (0x160, 0, 16)}
  CANDIDATES = {(0x12A, 0, 16), (0x1CF, 2, 8), (0x160, 0, 16), (0x7D0, 0, 8), (0x51, 0, 32)} | TX_CCNC | TX_NON_CCNC

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety

  def _init(self, ccnc=True, long=True):
    param = self.BASE | (self.CCNC if ccnc else 0) | (self.LONG if long else 0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, int(param))
    self.safety.init_tests()

  def _tx_zero(self, addr, bus, length):
    if addr == 0x1A0:
      return self.safety.safety_tx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", bus, {"aReqRaw": 0, "aReqValue": 0}))
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, b"\x00" * length))

  def _rx(self, addr, bus, length):
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, b"\x00" * length))

  def test_tx_set(self):
    for ccnc, expected in ((True, self.TX_CCNC), (False, self.TX_NON_CCNC)):
      self._init(ccnc)
      for msg in self.CANDIDATES:
        with self.subTest(ccnc=ccnc, msg=tuple(map(hex, msg[:1])) + msg[1:]):
          self.assertEqual(msg in expected, self._tx_zero(*msg))

  def test_openpilot_never_sends_0x12a(self):
    for ccnc in (True, False):
      self._init(ccnc)
      self.assertFalse(self._tx_zero(0x12A, 0, 16))

  def test_camera_0x12a_forwarded_scc_control_blocked(self):
    for ccnc in (True, False):
      with self.subTest(ccnc=ccnc):
        self._init(ccnc)
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x12A))  # lane/FCA/ESA status from the camera still reaches the car
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1A0))  # the camera's SCC_CONTROL is replaced by ours
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0xCB))
        self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1E0))

  def test_ccnc_camera_frames(self):
    self._init(ccnc=True)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x161))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x162))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x160))  # not ours on CCNC: the camera's stock FCA warning light frame stays
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x7C4))
    self._init(ccnc=False)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x160))  # ours on non-CCNC

  def test_mdps_copy_in_long(self):
    self._init()
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0xEA))
    self.assertTrue(self._tx_zero(0xEA, 2, 24))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0xEA))
    self._init(ccnc=False)
    self.assertFalse(self._tx_zero(0xEA, 2, 24))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0xEA))

  def test_relay_malfunction(self):
    for ccnc, addr, length, expected in ((True, 0x1A0, 32, True), (True, 0x161, 32, True), (True, 0xCB, 24, True), (True, 0x12A, 16, False),
                                         (True, 0x160, 16, False), (False, 0x160, 16, True), (False, 0x1A0, 32, True)):
      with self.subTest(ccnc=ccnc, addr=hex(addr)):
        self._init(ccnc)
        self._rx(addr, 0, length)
        self.assertEqual(expected, self.safety.get_relay_malfunction())

  def test_camera_mdps_relay_malfunction(self):
    self._init()
    self._rx(0xEA, 2, 24)
    self.assertTrue(self.safety.get_relay_malfunction())

  def test_accel_limited_by_longitudinal_checks(self):
    self._init()
    self.assertTrue(self.safety.safety_tx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 0, {"aReqRaw": 0, "aReqValue": 0})))
    # controls not allowed: only the inactive accel value goes out
    self.assertFalse(self.safety.safety_tx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 0, {"aReqRaw": 1.0, "aReqValue": 1.0})))

  def test_cruise_buttons_still_gated(self):
    self._init()
    self.assertFalse(self._tx_zero(0x1CF, 2, 8))  # no button / main-button pulses are not allowed

  def test_default_is_stock_scc(self):
    # toggle off: openpilot may only cancel via SCC_CONTROL, the camera's SCC_CONTROL and 0x12A are forwarded, no MDPS copy until sent
    for ccnc in (True, False):
      with self.subTest(ccnc=ccnc):
        self._init(ccnc, long=False)
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x12A))
        self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1A0))
        self.assertFalse(self.safety.safety_tx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 0, {"aReqRaw": 1.0, "aReqValue": 1.0})))
        self.assertEqual(ccnc, self._tx_zero(0xEA, 2, 24))


class TestHyundaiCanfdLongMainButtonEngage(unittest.TestCase):
  """CCNC LFA_ALT camera-SCC longitudinal: the camera's own MainMode_ACC (bus 2 0x1A0) is the main cruise state. Its rising edge with a main
  button press nearby engages openpilot longitudinal, its falling edge disengages. A press the camera ignores or a camera change without a press does nothing."""

  # superset of the CCNC LFA_ALT long variants; read by common.test_tx_hook_on_wrong_safety_mode
  TX_MSGS = [[0x1CF, 2], [0x1E0, 0], [0xCB, 0], [0x1A0, 0], [0x160, 0], [0x161, 0], [0x162, 0], [0x7C4, 2], [0xEA, 2]]

  BASE = (HyundaiSafetyFlags.CANFD_ANGLE_STEERING | HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_LFA_ALT_STEERING |
          HyundaiSafetyFlags.CCNC | HyundaiSafetyFlags.LONG)
  FLAG = HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE
  SET, RESUME, CANCEL = 2, 1, 4

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety

  def _init(self, param=None, toggleable=True):
    param = self.BASE | self.FLAG if param is None else param
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE if toggleable else 0)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, int(param))
    self.safety.init_tests()
    self.safety.set_timer(0)
    self.cam = False

  def _camera_main(self, on):
    self.cam = on
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 2, {"MainMode_ACC": int(on)}))

  def _button(self, buttons=0, main=0):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("CRUISE_BUTTONS", 0, {"CRUISE_BUTTONS": buttons, "ADAPTIVE_CRUISE_MAIN_BTN": main}))

  def _press_main(self, frames=3, camera=True):
    """A main press; the camera toggles its main state on the first frame of it (unless it ignores the press)."""
    for i in range(frames):
      self._button(main=1)
      if i == 0 and camera:
        self._camera_main(not self.cam)
    self._button(main=0)

  def _brake(self, pressed):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("TCS", 0, {"DriverBraking": pressed}))

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

  def test_camera_ignores_press(self):
    self._init()
    self._press_main(camera=False)  # e.g. brake held at low speed: the stock car does not engage either
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_timer(100_000_000)
    self._camera_main(True)  # much later, no press near it
    self.assertTrue(self.safety.get_acc_main_on())
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
    self.assertTrue(self.safety.get_acc_main_on())
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
    self._init(self.BASE)
    self._press_main()
    self.assertTrue(self.safety.get_acc_main_on())  # main still toggles (MADS)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_flag_requires_ccnc_lfa_alt_long(self):
    # flag alone, without the rest of the config, is ignored
    for param in (self.BASE & ~HyundaiSafetyFlags.LONG, self.BASE & ~HyundaiSafetyFlags.CCNC, self.BASE & ~HyundaiSafetyFlags.CANFD_LFA_ALT_STEERING,
                  self.BASE & ~HyundaiSafetyFlags.CAMERA_SCC, self.BASE & ~HyundaiSafetyFlags.CANFD_ANGLE_STEERING):
      with self.subTest(param=int(param)):
        self._init(param | self.FLAG)
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
        self._init(self.BASE)
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
    self._init(self.BASE)
    self._press_main()
    self.assertFalse(self.safety.get_controls_allowed())

  def _lateral_on(self, param=None):
    self._init(param)
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
    self._init(self.BASE)
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
    self._init(self.BASE)
    self.safety.set_mads_params(True, False, False)
    self._button(main=1)
    self._button(main=0)
    self._button(main=1)
    self._button(main=0)
    self.assertFalse(self.safety.get_controls_allowed_lateral())


class TestHyundaiCanfdCameraHandback(unittest.TestCase):
  """CAMERA_HANDBACK: the camera's 0xCB / 0xEA (MDPS) / 0x1A0 are forwarded unless openpilot owns that axis. Lateral goes to openpilot on its first
  accepted 0xCB frame (active or inactive, with permission) and never comes back; longitudinal returns to the camera after a driver action (brake,
  cancel, main button), never because openpilot stalled; a camera ACC that engages without a driver engage for openpilot is locked out of the car."""

  BASE = HyundaiSafetyFlags.CANFD_ANGLE_STEERING | HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_LFA_ALT_STEERING | HyundaiSafetyFlags.CCNC
  FLAG = HyundaiSafetyFlags.CAMERA_HANDBACK
  LONG = HyundaiSafetyFlags.LONG
  MAIN = HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE
  CANCEL = 4
  S = 1_000_000  # us

  # superset of the variants; read by common.test_tx_hook_on_wrong_safety_mode
  TX_MSGS = [[0x1CF, 2], [0x1E0, 0], [0xCB, 0], [0x1A0, 0], [0x160, 0], [0x161, 0], [0x162, 0], [0x7C4, 2], [0xEA, 2]]

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety

  def _init(self, long=True, flag=True, base=None):
    param = (self.BASE if base is None else base) | (self.FLAG if flag else 0) | ((self.LONG | self.MAIN) if long else 0)
    self.safety.set_current_safety_param_sp(HyundaiSafetyFlagsSP.LONG_MAIN_CRUISE_TOGGLEABLE)
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, int(param))
    self.safety.init_tests()
    self.safety.set_timer(0)

  def _cb(self, active):
    return self.safety.safety_tx_hook(self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": 2 if active else 1}))

  def _mdps(self):
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0xEA, 2, b"\x00" * 24))

  def _scc(self, mode):
    return self.safety.safety_tx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 0, {"ACCMode": mode, "aReqRaw": 0, "aReqValue": 0}))

  def _rx(self, addr, bus, length):
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, b"\x00" * length))

  def _button(self, buttons=0, main=0):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("CRUISE_BUTTONS", 0, {"CRUISE_BUTTONS": buttons, "ADAPTIVE_CRUISE_MAIN_BTN": main}))

  def _brake(self, pressed):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("TCS", 0, {"DriverBraking": pressed}))

  def _camera_main(self, on):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 2, {"MainMode_ACC": on}))

  # forwarded (camera frame reaches the car / the camera) vs blocked
  def _cb_fwd(self):
    return self.safety.safety_fwd_hook(2, 0xCB) == 0

  def _mdps_fwd(self):
    return self.safety.safety_fwd_hook(0, 0xEA) == 2

  def _scc_fwd(self):
    return self.safety.safety_fwd_hook(2, 0x1A0) == 0

  def _engage_lat(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._cb(True))

  def _engage_long(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._scc(1))

  def test_flag_off_unchanged(self):
    self._init(flag=False)
    self.assertFalse(self._cb_fwd())  # statically blocked, openpilot always sends
    self.assertFalse(self._scc_fwd())
    self.assertTrue(self._mdps_fwd())  # real MDPS until openpilot's copy appears
    self.assertTrue(self._mdps())
    self.assertFalse(self._mdps_fwd())

  def test_flag_needs_ccnc_lfa_alt_camera_scc(self):
    for base in (self.BASE & ~HyundaiSafetyFlags.CCNC, self.BASE & ~HyundaiSafetyFlags.CANFD_LFA_ALT_STEERING,
                 self.BASE & ~HyundaiSafetyFlags.CAMERA_SCC):
      with self.subTest(base=int(base)):
        self._init(base=base)
        self.assertFalse(self._cb_fwd())

  def test_startup_stock_owns_everything(self):
    for long in (True, False):
      with self.subTest(long=long):
        self._init(long)
        self.assertTrue(self._cb_fwd())
        self.assertTrue(self._mdps_fwd())
        self.assertTrue(self._scc_fwd())
        self.assertTrue(self.safety.safety_fwd_hook(2, 0x12A) == 0)

  def test_openpilot_rejected_while_stock_owns(self):
    self._init()
    self.assertFalse(self._cb(False))  # no inactive frame, no MDPS copy: a second sender would interleave with the camera
    self.assertFalse(self._mdps())
    self.assertFalse(self._scc(0))
    self.assertFalse(self._cb(True))  # controls not allowed
    self.assertFalse(self._scc(1))
    self.assertTrue(self._cb_fwd() and self._scc_fwd() and self._mdps_fwd())

  def test_lateral_takeover_on_first_active_frame(self):
    self._init()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._cb_fwd())  # allowed, but openpilot has not sent yet: camera keeps the car supplied
    self.assertTrue(self._cb(True))
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._mdps_fwd())
    self.assertTrue(self._mdps())
    self.assertTrue(self._cb(False))  # inactive frames continue (the car must not see a gap)
    self.assertTrue(self._scc_fwd())  # longitudinal untouched

  def test_mads_lateral_only(self):
    self._init()
    self.safety.set_controls_allowed_lateral(True)  # MADS: lateral permission without longitudinal
    self.assertTrue(self._cb(True))
    self.assertFalse(self._cb_fwd())
    self.assertTrue(self._scc_fwd())
    self.assertFalse(self._scc(1))  # longitudinal is not allowed

  # --- lane-keeping (LKA) nudge window: camera 0x161 LFA_ICON + 0x12A LKA_SysIndReq (research/stock-lfa-ownership.md) ---
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
    """openpilot takes lateral (MADS on), then the driver turns it off"""
    self._init()
    self.safety.set_timer(0)
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._cb(True))
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self._cb_fwd())

  def test_lka_frame_passes_with_mads_off(self):
    self._own_lat_then_off()
    self._cam_run(0, 1_400_000, 0, 3)
    self.assertFalse(self._cb_fwd())  # icon 0 for less than the settle time
    self._cam_run(1_400_000, 1_600_000, 0, 3)
    self.assertTrue(self._cb_fwd())  # LKA nudge: icon settled 0, lane departure
    self.assertTrue(self._mdps_fwd())  # the real MDPS follows the camera's frame
    self.assertFalse(self._cb(False))  # and openpilot's own frames are rejected: one sender
    self.assertFalse(self._mdps())
    self.assertTrue(self._scc_fwd())  # longitudinal untouched

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

  def test_lka_frames_blocked_while_mads_on_or_paused(self):
    self._own_lat_then_off()
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertTrue(self._cb_fwd())
    self.safety.set_controls_allowed_lateral(True)  # MADS on, and also paused (the panda sees no difference: the permission stays)
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._mdps_fwd())
    self.assertTrue(self._cb(False))
    self.assertTrue(self._cb(True))
    self.safety.set_controls_allowed_lateral(False)
    self.safety.set_controls_allowed(True)  # controls (ACC + steering) allowed
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._mdps_fwd())
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._cb_fwd())  # lateral is off again: the nudge may steer

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

  def test_lka_window_resets_at_init(self):
    self._own_lat_then_off()
    self._cam_run(0, 2_000_000, 0, 3)
    self.assertTrue(self._cb_fwd())
    self._init()
    self.safety.set_timer(2_000_000)
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._cb(True))
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self._cb_fwd())  # no camera frames seen since init

  def test_stock_owns_before_openpilot_takes_lateral(self):
    self._init()
    self._cam_run(0, 2_000_000, 2, 2)
    self.assertTrue(self._cb_fwd())  # before the first openpilot frame the camera is untouched (LFA included)

  def test_long_takeover_on_first_active_frame(self):
    self._init()
    self.assertFalse(self._scc(0))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._scc_fwd())
    self.assertFalse(self._scc(0))  # an 'off' frame cannot take the axis
    self.assertTrue(self._scc(1))
    self.assertFalse(self._scc_fwd())
    self.assertTrue(self._scc(0))  # off frames pass once openpilot owns it
    self.assertTrue(self._cb_fwd())

  def test_non_long_scc_always_forwarded(self):
    self._init(long=False)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._scc_fwd())
    self.assertFalse(self._scc(1))  # openpilot may only cancel through 0x1A0 without longitudinal
    self.assertTrue(self._cb(True))
    self.assertTrue(self._scc_fwd())

  def test_no_timeout_while_openpilot_owns(self):
    self._init()
    self._engage_lat()
    self._engage_long()
    self.safety.set_timer(100 * self.S)
    self.assertFalse(self._cb_fwd() or self._mdps_fwd() or self._scc_fwd())

  def _lfa_button(self):
    self.safety.set_mads_button_press(1)

  def test_brake_hands_back_longitudinal_only(self):
    self._init()
    self._engage_lat()
    self._engage_long()
    self._brake(1)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._scc_fwd())
    # steering stays with openpilot: a brake edge is not an explicit steering disable, and its trailing frames keep flowing
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())
    self.assertTrue(self._cb(False) and self._mdps())
    self.assertFalse(self._scc(0))  # the camera owns longitudinal again

  def test_cancel_hands_back_longitudinal_only(self):
    self._init()
    self._engage_lat()
    self._engage_long()
    self._button(self.CANCEL)
    self.assertTrue(self._scc_fwd())
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())

  def test_main_button_off_hands_back_longitudinal_only(self):
    self._init()
    self._button(main=1)  # Driving Assist press, the camera turns main on: controls allowed
    self._camera_main(1)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._cb(True) and self._scc(1))
    self._button(main=0)
    self.assertFalse(self._cb_fwd() or self._scc_fwd())
    self._button(main=1)  # second press: the camera turns main off
    self._camera_main(0)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._scc_fwd())
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())

  def test_lfa_button_never_hands_lateral_back(self):
    # the camera's lane keeping (also enabled again by the Driving Assist/main press or by lane keeping assist) must not steer after the driver's
    # LFA press: openpilot keeps sending its inactive frames until the next panda init
    self._init()
    self._engage_lat()
    self._engage_long()
    self._brake(1)
    self._brake(0)
    self.assertFalse(self._cb_fwd())
    self._lfa_button()
    self.safety.set_controls_allowed_lateral(False)
    self.safety.set_timer(30 * self.S)
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())
    self.assertTrue(self._cb(False) and self._mdps())  # our inactive frames and the MDPS copy keep flowing
    for _ in range(3):
      self._button(self.CANCEL)
      self._button(main=1)
      self._camera_main(1)
      self._button(main=0)
      self._lfa_button()
      self.assertFalse(self._cb_fwd() or self._mdps_fwd())

  def test_mads_button_then_permission_loss_keeps_lateral(self):
    self._init()
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._cb(True))
    self.safety.set_mads_button_press(1)  # lane-assist button, then the heartbeat mismatch drops the permission ~3 s later
    self.assertFalse(self._cb_fwd())
    self.safety.set_timer(3 * self.S)
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())

  def test_inactive_frame_takes_lateral_under_permission(self):
    # LFA on while the driver pushes hard: the takeover filter sends inactive frames (request 0), the camera's request must not reach the EPS
    self._init()
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._cb_fwd())  # openpilot has not sent yet
    self.assertTrue(self._cb(False))
    self.assertFalse(self._cb_fwd())
    self.assertFalse(self._mdps_fwd())
    self.assertTrue(self._mdps())
    self.assertTrue(self._scc_fwd())  # longitudinal untouched

  def test_inactive_frame_rejected_without_permission(self):
    self._init()
    self.assertFalse(self._cb(False))
    self.assertTrue(self._cb_fwd() and self._mdps_fwd())
    self.safety.set_controls_allowed_lateral(True)
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self._cb(False))
    self.assertTrue(self._cb_fwd())

  def test_takeover_pause_never_lets_camera_through(self):
    self._init()
    self._engage_lat()
    for _ in range(50):  # paused: inactive frames only, for a long time and across a permission change
      self.assertTrue(self._cb(False))
      self.assertFalse(self._cb_fwd() or self._mdps_fwd())
    self.safety.set_controls_allowed_lateral(False)
    self.safety.set_timer(10 * self.S)
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())

  def test_heartbeat_loss_keeps_camera_blocked_until_driver_action(self):
    self._init()
    self._engage_lat()
    self._engage_long()
    self.safety.set_controls_allowed(False)  # permission lost with no driver action (heartbeat mismatch / lag / stalled openpilot)
    self.safety.set_timer(30 * self.S)
    self.assertFalse(self._cb_fwd() or self._mdps_fwd() or self._scc_fwd())
    self._brake(1)  # the next brake releases longitudinal only
    self.assertTrue(self._scc_fwd())
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())
    self._lfa_button()  # lateral stays with openpilot
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())

  def test_brake_while_steering_never_hands_lateral_back(self):
    self._init()
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._cb(True))
    self._brake(1)  # MADS continues through the brake, also when the permission is lost soon after
    self.safety.set_timer(4 * self.S)
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self._cb_fwd())

  def test_reengage_after_longitudinal_handback(self):
    self._init()
    self._engage_lat()
    self._engage_long()
    self._brake(1)
    self._brake(0)
    self.assertTrue(self._scc_fwd())
    self.assertFalse(self._cb_fwd() or self._mdps_fwd())
    self._engage_long()
    self.assertFalse(self._cb_fwd() or self._mdps_fwd() or self._scc_fwd())

  # --- stock ACC lock: the camera's own ACC engaged without a driver engage for openpilot (CANCEL tap in standby, route 2d 9:48.8 / 11:01.8)
  def _camera_acc(self, mode, main=1):
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 2, {"MainMode_ACC": main, "ACCMode": mode}))

  def _standby_after_brake(self):
    """openpilot engaged (lateral and longitudinal), driver brakes: longitudinal goes back to the camera, which sits in standby with main on."""
    self._init()
    self._button(main=1)
    self._camera_main(1)
    self._button(main=0)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._cb(True) and self._scc(1))
    self._camera_acc(1)  # camera engaged along with openpilot
    self._brake(1)
    self._brake(0)
    self.assertTrue(self._scc_fwd())
    self.assertFalse(self.safety.get_controls_allowed())
    self._camera_acc(4)  # camera: standby
    self.assertTrue(self._scc_fwd())

  def _cancel_tap(self):
    for _ in range(7):
      self._button(self.CANCEL)
    self._button(0)

  def test_cancel_tap_in_standby_is_locked_out(self):
    self._standby_after_brake()
    self._cancel_tap()
    self.assertFalse(self.safety.get_controls_allowed())  # the panda sees a cancel: nothing of ours is engaged
    self._camera_acc(1)  # the camera resumed its own ACC at the stale set speed
    self.assertFalse(self._scc_fwd())  # from the next frame the car does not see the camera's 0x1A0
    self.assertTrue(self._scc(0))  # openpilot's 'off' frames replace it
    self.assertFalse(self._scc(1))  # but cannot engage anything
    self.assertFalse(self._scc(2))
    for _ in range(20):
      self._camera_acc(1)
      self.assertFalse(self._scc_fwd())
      self.assertTrue(self._scc(0))

  def test_stock_lock_ends_when_camera_is_back_in_standby(self):
    self._standby_after_brake()
    self._cancel_tap()
    self._camera_acc(1)
    self.assertFalse(self._scc_fwd())
    self._camera_acc(4)
    self._camera_acc(4)
    self.assertFalse(self._scc_fwd())
    self._camera_acc(1)  # a blip resets the count
    self._camera_acc(4)
    self._camera_acc(4)
    self.assertFalse(self._scc_fwd())
    self._camera_acc(4)
    self.assertTrue(self._scc_fwd())  # camera frames flow again (standby, nothing to drive)
    self.assertFalse(self._scc(0))  # and openpilot's frames are rejected as before

  def test_stock_lock_ends_when_camera_main_goes_off(self):
    self._standby_after_brake()
    self._cancel_tap()
    self._camera_acc(1)
    for _ in range(3):
      self._camera_acc(0, main=0)
    self.assertTrue(self._scc_fwd())

  def test_explicit_engage_is_not_locked(self):
    # SET/RES press with main on: controls allowed at the press, the camera engages along: openpilot takes over, no lock
    self._standby_after_brake()
    self._button(2)
    self._button(0)
    self.assertTrue(self.safety.get_controls_allowed())
    self._camera_acc(1)
    self.assertTrue(self._scc_fwd())  # camera still supplies the car until openpilot's first engaged frame
    self.assertFalse(self._scc(0))
    self.assertTrue(self._scc(1))
    self.assertFalse(self._scc_fwd())

  def test_lock_releases_to_openpilot_on_explicit_engage(self):
    self._standby_after_brake()
    self._cancel_tap()
    self._camera_acc(1)
    self.assertFalse(self._scc_fwd())
    self._button(2)  # now the driver engages on purpose
    self._button(0)
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self._scc(1))
    self._camera_acc(4)
    self._camera_acc(4)
    self._camera_acc(4)
    self.assertFalse(self._scc_fwd())  # openpilot owns it, the lock is gone but the ownership stays
    self.assertTrue(self._scc(1))

  def test_first_camera_frame_is_not_an_engage_edge(self):
    self._init()
    self._camera_acc(1)  # openpilot restarted while the stock ACC was already driving: no lock, the camera keeps the car
    self.assertTrue(self._scc_fwd())
    self._camera_acc(1)
    self.assertTrue(self._scc_fwd())

  def test_stock_lock_needs_camera_main_engage_config(self):
    self._init()
    self.safety.set_controls_allowed(False)
    self._camera_acc(4)
    self._camera_acc(1)
    self.assertFalse(self._scc_fwd())
    self._init(long=False)
    self._camera_acc(4)
    self._camera_acc(1)
    self.assertTrue(self._scc_fwd())  # without openpilot longitudinal 0x1A0 is the camera's alone

  def test_cancel_button_still_reaches_camera_when_it_is_engaged(self):
    # the stock pause key must keep working on a camera ACC that is engaged (the lock only blocks its 0x1A0 output)
    self._standby_after_brake()
    self._cancel_tap()
    self._camera_acc(1)
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x1CF))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x1CF))


  def test_init_resets_ownership(self):
    self._init()
    self._engage_lat()
    self._init()
    self.assertTrue(self._cb_fwd() and self._mdps_fwd())

  def test_relay_malfunction(self):
    for long in (True, False):
      for addr, bus, length, expected in ((0xCB, 0, 24, True), (0xCB, 2, 24, False), (0xEA, 2, 24, True), (0xEA, 0, 24, False),
                                          (0x1A0, 0, 32, long), (0x1A0, 2, 32, False), (0x161, 0, 32, True)):
        with self.subTest(long=long, addr=hex(addr), bus=bus):
          self._init(long)
          self._rx(addr, bus, length)
          self.assertEqual(expected, self.safety.get_relay_malfunction())

  def test_camera_frames_forwarded_do_not_trip_relay(self):
    self._init()
    for _ in range(10):
      self.assertTrue(self._cb_fwd() and self._scc_fwd())
    self.assertFalse(self.safety.get_relay_malfunction())

  def test_relay_malfunction_blocks_all_forwarding(self):
    self._init()
    self.safety.set_relay_malfunction(True)
    self.assertFalse(self._cb_fwd() or self._mdps_fwd() or self._scc_fwd())

  def test_openpilot_checks_unchanged(self):
    self._init()
    self.safety.set_controls_allowed(True)
    # steering request far from the measured angle is still refused, and does not take the axis
    self.assertFalse(self.safety.safety_tx_hook(self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": 2, "ADAS_StrAnglReqVal": 300})))
    self.assertTrue(self._cb_fwd())
    # accel above the limit is refused
    self.assertFalse(self.safety.safety_tx_hook(self.packer.make_can_msg_safety("SCC_CONTROL", 0, {"ACCMode": 1, "aReqRaw": 3.5, "aReqValue": 3.5})))
    self.assertTrue(self._scc_fwd())

  def test_tx_set(self):
    for long in (True, False):
      self._init(long)
      self.safety.set_controls_allowed(True)
      self.assertTrue(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x1E0, 0, b"\x00" * 16)))
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0x12A, 0, b"\x00" * 16)))  # camera keeps 0x12A
      self.assertFalse(self.safety.safety_tx_hook(libsafety_py.make_CANPacket(0xEA, 0, b"\x00" * 24)))  # wrong bus


if __name__ == "__main__":
  unittest.main()
