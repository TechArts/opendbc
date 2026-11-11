"""Mirror of the panda's per-frame angle acceptance for 0xCB / 0x12A (opendbc/safety/lateral.h steer_angle_cmd_checks_vm, params of
hyundai_canfd.h HYUNDAI_STEERING_PARAMS): speed = min of the recent samples - 1 m/s (at least 1), integer 0.1 deg units, +1 count tolerance.
Used by the controller to send inactive at the measured angle instead of a frame the panda would reject."""
import math
from collections import deque

EARTH_G = 9.81
AVERAGE_ROAD_ROLL = 0.06
MAX_LATERAL_ACCEL = 3.0 + EARTH_G * AVERAGE_ROAD_ROLL
MAX_LATERAL_JERK = 3.0 + EARTH_G * AVERAGE_ROAD_ROLL
SLIP_FACTOR = -0.0006085930193026732
STEER_RATIO = 13.7
WHEELBASE = 2.756
RAD_TO_DEG = 57.29577951308232
DEG_TO_CAN = 10
FREQUENCY = 100
SPEED_SAMPLES = 6  # MAX_SAMPLE_VALS
# the panda's speed samples are integer mm/s of the mean of four 0.03125 kph wheel counts (hyundai_canfd.h:329, UPDATE_VEHICLE_SPEED), quantised below
# the same way; its limits are float32. Err on the side of inactive: a slightly higher speed (smaller limits) and one count less than the panda's
# own +1 tolerance.
SPEED_MARGIN = 0.005  # m/s on top of the exact quantisation below (float32 in the panda)
COUNT_MARGIN = 1


def _angle_from_curvature(curvature: float, speed: float) -> float:
  curvature_factor = 1.0 / (1.0 - SLIP_FACTOR * speed * speed) / WHEELBASE
  return curvature * STEER_RATIO / curvature_factor * RAD_TO_DEG


def _to_can(angle: float) -> int:
  """The CAN packer's quantisation of a signal with factor 0.1: floor(x / 0.1 + 0.5), not Python's banker's round()."""
  return math.floor(angle / 0.1 + 0.5)


def limits_can(speed_min: float) -> tuple[int, int]:
  """(max step per frame, max |angle|) in CAN units for the minimum of the recent speed samples (m/s)."""
  speed = max(speed_min - 1.0, 1.0)
  step = _angle_from_curvature(MAX_LATERAL_JERK / (speed * speed), speed) / FREQUENCY
  max_angle = _angle_from_curvature(MAX_LATERAL_ACCEL / (speed * speed), speed)
  return int(step * DEG_TO_CAN + 1.0), int(max_angle * DEG_TO_CAN + 1.0)


class PandaAngleCheck:
  def __init__(self):
    self.speeds: deque[float] = deque(maxlen=SPEED_SAMPLES)

  def add_speed(self, v_ego_raw: float):
    """v_ego_raw is the mean of the four wheel counts * 0.03125 kph in m/s (a multiple of a quarter count); stored like the panda's sample."""
    mean_counts = round(v_ego_raw * 3.6 / 0.03125 * 4) / 4
    self.speeds.append(math.floor(mean_counts * 0.03125 / 3.6 * 1000.0 + 0.5) / 1000.0)

  def accepts(self, request: float, last: float) -> bool:
    """Would the panda accept an ACTIVE frame with this request, after a frame with angle `last` (both deg)? Never True for a frame it rejects;
    False for at most the few counts at the very edge of its limits that it would accept."""
    step, max_angle = limits_can((min(self.speeds) if self.speeds else 0.0) + SPEED_MARGIN)
    request_can, last_can = _to_can(request), _to_can(last)
    return abs(request_can - last_can) <= step - COUNT_MARGIN and abs(request_can) <= max_angle - COUNT_MARGIN
