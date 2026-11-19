"""Mirror of the panda's per-frame angle acceptance for 0xCB / 0x12A (opendbc/safety/lateral.h steer_angle_cmd_checks_vm with the platform's angle
model from hyundai_canfd_angle_models.h): speed = min of the recent samples - 1 m/s (at least 1), integer 0.1 deg units, +1 count tolerance.
Used by the controller to send inactive at the measured angle instead of a frame the panda would reject.

The model is the platform's own (slip factor of the vehicle model, steer ratio, wheelbase: the values the panda table is generated from), so the mirror
follows the panda for every platform; test_takeover.TestPandaAngleCheck compares it with the compiled panda and with the header's table."""
import math
from collections import deque

from opendbc.car.lateral import MAX_LATERAL_ACCEL, MAX_LATERAL_JERK

RAD_TO_DEG = 57.29577951308232
DEG_TO_CAN = 10
FREQUENCY = 100
SPEED_SAMPLES = 6  # MAX_SAMPLE_VALS
# Backstop only (the controller's own step cap in takeover.limit() is what keeps normal frames far inside the panda's limits). The panda's speed is
# the minimum of its last six RX samples, which are not aligned with our control frames (CAN batches). Smaller speeds give larger limits, so the
# backstop evaluates at the largest recent speed + 1 m/s (the panda subtracts 1 m/s): tighter than the panda for any sample history. Speeds are
# quantised like the panda's (hyundai_canfd.h, integer mm/s of the mean of four 0.03125 kph counts); its limits are float32, one count of margin.
SPEED_MARGIN = 1.0  # m/s
COUNT_MARGIN = 1


def _to_can(angle: float) -> int:
  """The CAN packer's quantisation of a signal with factor 0.1: floor(x / 0.1 + 0.5), not Python's banker's round()."""
  return math.floor(angle / 0.1 + 0.5)


class PandaAngleCheck:
  def __init__(self, slip_factor: float, steer_ratio: float, wheelbase: float):
    self.slip_factor = slip_factor
    self.steer_ratio = steer_ratio
    self.wheelbase = wheelbase
    self.speeds: deque[float] = deque(maxlen=SPEED_SAMPLES)

  def _angle_from_curvature(self, curvature: float, speed: float) -> float:
    curvature_factor = 1.0 / (1.0 - self.slip_factor * speed * speed) / self.wheelbase
    return curvature * self.steer_ratio / curvature_factor * RAD_TO_DEG

  def limits_can(self, speed_min: float) -> tuple[int, int]:
    """(max step per frame, max |angle|) in CAN units for the minimum of the recent speed samples (m/s)."""
    speed = max(speed_min - 1.0, 1.0)
    step = self._angle_from_curvature(MAX_LATERAL_JERK / (speed * speed), speed) / FREQUENCY
    max_angle = self._angle_from_curvature(MAX_LATERAL_ACCEL / (speed * speed), speed)
    return int(step * DEG_TO_CAN + 1.0), int(max_angle * DEG_TO_CAN + 1.0)

  def add_speed(self, v_ego_raw: float):
    """v_ego_raw is the mean of the four wheel counts * 0.03125 kph in m/s (a multiple of a quarter count); stored like the panda's sample."""
    mean_counts = round(v_ego_raw * 3.6 / 0.03125 * 4) / 4
    self.speeds.append(math.floor(mean_counts * 0.03125 / 3.6 * 1000.0 + 0.5) / 1000.0)

  def accepts(self, request: float, last: float) -> bool:
    """Would the panda accept an ACTIVE frame with this request, after a frame with angle `last` (both deg)? Conservative backstop: True only if the panda
    accepts it for any speed samples up to the largest recent one."""
    step, max_angle = self.limits_can((max(self.speeds) if self.speeds else 0.0) + SPEED_MARGIN)
    request_can, last_can = _to_can(request), _to_can(last)
    return abs(request_can - last_can) <= step - COUNT_MARGIN and abs(request_can) <= max_angle - COUNT_MARGIN

  def window(self, last: float, max_step_deg: float = 1e9) -> tuple[float, float] | None:
    """The angles (deg) the backstop accepts after a frame with angle `last`: within its step (and the caller's max_step_deg) of `last`, and within its
    angle limit, in CAN counts (multiples of 0.1 deg). None when no angle is."""
    step, max_angle = self.limits_can((max(self.speeds) if self.speeds else 0.0) + SPEED_MARGIN)
    step, max_angle = min(step - COUNT_MARGIN, int(max_step_deg * DEG_TO_CAN)), max_angle - COUNT_MARGIN
    last_can = _to_can(last)
    low, high = max(last_can - step, -max_angle), min(last_can + step, max_angle)
    if low > high:
      return None
    return low / DEG_TO_CAN, high / DEG_TO_CAN
