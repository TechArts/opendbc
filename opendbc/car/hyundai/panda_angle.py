"""Mirror of the panda's per-frame angle acceptance for 0xCB / 0x12A (opendbc/safety/lateral.h steer_angle_cmd_checks_vm, params of
hyundai_canfd.h HYUNDAI_STEERING_PARAMS): speed = min of the recent samples - 1 m/s (at least 1), integer 0.1 deg units, +1 count tolerance.
Used by the controller to send inactive at the measured angle instead of a frame the panda would reject."""
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


def _angle_from_curvature(curvature: float, speed: float) -> float:
  curvature_factor = 1.0 / (1.0 - SLIP_FACTOR * speed * speed) / WHEELBASE
  return curvature * STEER_RATIO / curvature_factor * RAD_TO_DEG


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
    self.speeds.append(v_ego_raw)

  def accepts(self, request: float, last: float) -> bool:
    """Would the panda accept an ACTIVE frame with this request, after a frame with angle `last` (both deg)?"""
    step, max_angle = limits_can(min(self.speeds) if self.speeds else 0.0)
    request_can, last_can = round(request * DEG_TO_CAN), round(last * DEG_TO_CAN)
    return abs(request_can - last_can) <= step and abs(request_can) <= max_angle
