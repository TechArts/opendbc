"""GV70 angle steering: a driver takeover (wheel) is a pause, not a fight.

While paused the controller sends 0xCB INACTIVE at the measured angle with gain 0 (no counter-force, no angle error for the EPS to fault on).
When the driver lets go the request is blended back to the model: angle step limited, gain ramped by the controller, and never further than
CONVERGE_DEG from the measured angle. Also pauses on a temporary EPS fault and when the low-speed, large-angle cap keeps binding.
"""
import numpy as np

from opendbc.car import DT_CTRL

ENTER_FRAMES = 3  # |torque| above the enter threshold for this many frames starts a pause
RESUME_FRAMES = int(1.0 / DT_CTRL)  # angle step limit after a pause
FAULT_FRAMES = int(1.0 / DT_CTRL)  # stay paused this long after steerFaultTemporary clears
CONVERGE_DEG = 5.0  # request stays this close to the measured angle while resuming

# low-speed guard: both short EPS faults had a request 19-29 deg ahead of the wheel at 174-182 deg and 3-4 m/s
GUARD_SPEED = 5.0  # m/s
GUARD_ANGLE = 150.0  # deg
GUARD_MAX_ERROR = 10.0  # deg, |request - measured|
GUARD_MAX_ANGLE = 170.0  # deg, |request|
GUARD_FRAMES = int(0.5 / DT_CTRL)  # the cap must bind this long before it pauses steering


class TakeoverFilter:
  def __init__(self, enter_torque: int, release_torque: int, hold_ms: int, resume_rate_x10: int):
    """Units are those of the params: torque units, ms, deg per frame x 10. enter_torque 0 turns the filter off."""
    self.enter_torque = enter_torque
    self.release_torque = release_torque
    self.hold_frames = int(hold_ms / 1000.0 / DT_CTRL)
    self.resume_rate = resume_rate_x10 / 10.0
    self.paused = False
    self.resuming = False
    self.grab_frames = 0
    self.release_frames = 0
    self.resume_frames = 0
    self.fault_frames = 0
    self.bind_frames = 0

  @property
  def enabled(self) -> bool:
    return self.enter_torque > 0

  def _reset(self):
    self.paused = False
    self.resuming = False
    self.grab_frames = self.release_frames = self.resume_frames = self.bind_frames = 0

  @staticmethod
  def guard_active(angle: float, v_ego: float) -> bool:
    return v_ego < GUARD_SPEED and abs(angle) > GUARD_ANGLE

  def update(self, torque: float, pressed: bool, fault: bool, v_ego: float, angle: float, desired: float, lat_active: bool) -> bool:
    """Once per frame, with the model's (filtered) desired angle. Returns True while steering is paused."""
    if not self.enabled:
      return False

    self.fault_frames = FAULT_FRAMES if fault else max(0, self.fault_frames - 1)
    if not lat_active:
      self._reset()
      return False

    binds = self.guard_active(angle, v_ego) and (abs(desired - angle) > GUARD_MAX_ERROR or abs(desired) > GUARD_MAX_ANGLE)
    self.bind_frames = self.bind_frames + 1 if binds else 0
    self.grab_frames = self.grab_frames + 1 if abs(torque) >= self.enter_torque else 0

    if not self.paused:
      if pressed or self.grab_frames >= ENTER_FRAMES or self.fault_frames > 0 or self.bind_frames > GUARD_FRAMES:
        self.paused = True
        self.resuming = False
        self.release_frames = 0
    else:
      self.release_frames = self.release_frames + 1 if (abs(torque) < self.release_torque and not pressed) else 0
      if self.release_frames >= self.hold_frames and self.fault_frames == 0 and not binds:
        self.paused = False
        self.resuming = True
        self.resume_frames = RESUME_FRAMES

    if self.resuming:
      self.resume_frames = max(0, self.resume_frames - 1)
      if self.resume_frames == 0 and abs(desired - angle) <= CONVERGE_DEG:
        self.resuming = False
    return self.paused

  def limit(self, request: float, last: float, angle: float, v_ego: float) -> float:
    """Applies the resume blend and the low-speed cap to a request that already passed the vehicle-model limits."""
    if not self.enabled:
      return request
    if self.resuming:
      request = float(np.clip(request, last - self.resume_rate, last + self.resume_rate))
      request = float(np.clip(request, angle - CONVERGE_DEG, angle + CONVERGE_DEG))
    if self.guard_active(angle, v_ego):
      request = float(np.clip(request, angle - GUARD_MAX_ERROR, angle + GUARD_MAX_ERROR))
      request = float(np.clip(request, -GUARD_MAX_ANGLE, GUARD_MAX_ANGLE))
    return request
