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
# Backstop only (the controller's own step cap in takeover.limit() is what keeps normal frames far inside the panda's limits). The panda's speed is
# the minimum of its last six RX samples, which are not aligned with our control frames (CAN batches). Smaller speeds give larger limits, so the
# backstop evaluates at the largest recent speed + 1 m/s (the panda subtracts 1 m/s): tighter than the panda for any sample history. Speeds are
# quantised like the panda's (hyundai_canfd.h:329, integer mm/s of the mean of four 0.03125 kph counts); its limits are float32, one count of margin.
SPEED_MARGIN = 1.0  # m/s
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
    """Would the panda accept an ACTIVE frame with this request, after a frame with angle `last` (both deg)? Conservative backstop: True only if the panda
    accepts it for any speed samples up to the largest recent one."""
    step, max_angle = limits_can((max(self.speeds) if self.speeds else 0.0) + SPEED_MARGIN)
    request_can, last_can = _to_can(request), _to_can(last)
    return abs(request_can - last_can) <= step - COUNT_MARGIN and abs(request_can) <= max_angle - COUNT_MARGIN

  def clamp(self, request: float, last: float, max_step_deg: float = 1e9) -> float | None:
    """The request moved toward `last` to the largest step (and within the largest angle) the backstop accepts, in CAN counts; the request itself when
    it is accepted already; max_step_deg bounds the step too (a caller's own cap). None only when no angle is both within the step of `last` and within the angle limit."""
    step, max_angle = limits_can((max(self.speeds) if self.speeds else 0.0) + SPEED_MARGIN)
    step, max_angle = min(step - COUNT_MARGIN, int(max_step_deg * DEG_TO_CAN)), max_angle - COUNT_MARGIN
    last_can = _to_can(last)
    low, high = max(last_can - step, -max_angle), min(last_can + step, max_angle)
    if low > high:
      return None
    request_can = _to_can(request)
    if low <= request_can <= high:
      return request
    return (low if request_can < low else high) / DEG_TO_CAN
