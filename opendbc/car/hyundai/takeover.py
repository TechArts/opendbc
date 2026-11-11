"""GV70 angle steering: a driver takeover (wheel) is a pause, not a fight.

While paused the controller sends 0xCB INACTIVE at the measured angle with gain 0 (no counter-force, no angle error for the EPS to fault on).
When the driver lets go the request is blended back to the model: angle step limited (never more than the resume rate per frame, whatever the
wheel does; only the low-speed guard's own clips run after it), gain ramped by the controller, and kept within CONVERGE_DEG of the measured angle while the wheel moves slower than the resume rate.
Lateral never starts (engage, blinker pause end, takeover release) while the wheel still moves faster than that: it stays paused until it calms.
Also pauses on a temporary EPS fault and when the low-speed, large-angle cap keeps binding.
"""
import math
from collections import deque

import numpy as np

from opendbc.car import DT_CTRL

ENTER_FRAMES = 3  # |torque| above the enter threshold for this many frames starts a pause
RESUME_FRAMES = int(1.0 / DT_CTRL)  # angle step limit after a pause
FAULT_FRAMES = int(1.0 / DT_CTRL)  # stay paused this long after steerFaultTemporary clears
CONVERGE_DEG = 5.0  # request stays this close to the measured angle while resuming
WHEEL_RATE_FRAMES = 5  # frames the wheel's angular speed is measured over

# low-speed guard: both short EPS faults had a request 19-29 deg ahead of the wheel at 174-182 deg and 3-4 m/s
GUARD_SPEED = 5.0  # m/s
GUARD_ANGLE = 150.0  # deg
GUARD_MAX_ERROR = 10.0  # deg, |request - measured|
GUARD_MAX_ANGLE = 170.0  # deg, |request|
GUARD_FRAMES = int(0.5 / DT_CTRL)  # the cap must bind this long before it pauses steering

# curve-aware mode (HyundaiCurveHold, research/curve-hold.md Q2/Q5): in a curve only a pull against the turn, or a push deeper than the model
# asks, is a grab. A hand that holds the wheel along the turn (129 + 103 * a_lat torque once paused) is not.
CURVE_A_MIN = 0.3  # m/s^2, lateral load from which the wheel counts as in a curve
CURVE_DES_MIN = 2.0  # deg, and the model asks for at least this much angle
OPPOSE_RELEASE = 100  # opposing torque (curve sign) that blocks the release and drops the gain floor
ALONG_BASE = 300  # beyond-push clause: along-turn torque >= ALONG_BASE + ALONG_PER_A * a ...
ALONG_PER_A = 150
BEYOND_DEG = 3.0  # ... and the wheel at least this far past the model's angle in the push direction


class TakeoverFilter:
  def __init__(self, enter_torque: int, release_torque: int, hold_ms: int, resume_rate_x10: int,
               angle_to_lat_accel: float = 0.0, curve_hold: bool = False):
    """Units are those of the params: torque units, ms, deg per frame x 10. enter_torque 0 turns the filter off.
    angle_to_lat_accel is 1 / (steerRatio * wheelbase * 180 / pi) in (m/s^2 per (m/s)^2 per deg); curve_hold turns on the curve-aware rules."""
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
    self.lat_active_last = False
    self.angle_hist: deque[float] = deque(maxlen=WHEEL_RATE_FRAMES + 1)
    self.k = angle_to_lat_accel
    self.curve_hold = curve_hold and angle_to_lat_accel > 0.0
    self.hold_load = 0.0  # v^2 * |desired| * k, the model's lateral demand (gain floor input)
    self.opposing = False  # in a curve and the hand pulls against the turn by OPPOSE_RELEASE or more

  @property
  def enabled(self) -> bool:
    return self.enter_torque > 0

  @property
  def hold_state(self) -> tuple[float, bool, bool] | None:
    """Input of the curve-hold gain floor: (lateral demand, hand opposes the turn, paused or resuming); None unless curve hold applies."""
    if not (self.curve_hold and self.enabled):
      return None
    return self.hold_load, self.opposing, self.paused or self.resuming

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

    self.angle_hist.append(angle)
    wheel_rate = abs(angle - self.angle_hist[0]) / max(1, len(self.angle_hist) - 1)  # deg per frame
    # a wheel that still moves faster than the resume rate (route 34 3:25.3: the driver unwinding a turn at 1.5-2.3 deg per frame as the blinker pause
    # ended) cannot be blended with: a request limited to the rate falls behind it and fights, one that follows it exceeds the rate
    fast = self.resume_rate > 0 and wheel_rate > self.resume_rate

    self.fault_frames = FAULT_FRAMES if fault else max(0, self.fault_frames - 1)
    if not lat_active:
      self._reset()
      self.lat_active_last = False
      return False
    if not self.lat_active_last:
      # lateral just came back (engage, blinker pause, MADS pause, fault): blend in like after a takeover, never snap to the model
      self.resuming = True
      self.resume_frames = RESUME_FRAMES
      if fast:
        self.paused = True
        self.resuming = False
        self.release_frames = 0
    self.lat_active_last = True

    binds = self.guard_active(angle, v_ego) and (abs(desired - angle) > GUARD_MAX_ERROR or abs(desired) > GUARD_MAX_ANGLE)
    self.bind_frames = self.bind_frames + 1 if binds else 0

    turn, load, beyond = 0, 0.0, False
    self.hold_load, self.opposing = 0.0, False
    if self.curve_hold:
      self.hold_load = v_ego * v_ego * abs(desired) * self.k
      load = v_ego * v_ego * max(abs(desired), abs(angle)) * self.k
      if abs(desired) >= CURVE_DES_MIN and load >= CURVE_A_MIN:
        turn = int(math.copysign(1, desired))
    along = torque * turn  # > 0: along the requested turn (assisting), < 0: opposing
    self.opposing = turn != 0 and along <= -OPPOSE_RELEASE

    if turn == 0:
      grab = abs(torque) >= self.enter_torque
      press = pressed
    else:
      beyond = along >= ALONG_BASE + ALONG_PER_A * load and (angle - desired) * turn >= BEYOND_DEG
      grab = along <= -self.enter_torque or beyond
      press = pressed and along < 0  # steeringPressed from along-turn torque is the driver holding the curve
    self.grab_frames = self.grab_frames + 1 if grab else 0

    if not self.paused:
      if press or self.grab_frames >= ENTER_FRAMES or self.fault_frames > 0 or self.bind_frames > GUARD_FRAMES:
        self.paused = True
        self.resuming = False
        self.release_frames = 0
    else:
      if turn == 0:
        calm = abs(torque) < self.release_torque and not pressed
      else:
        calm = along > -self.release_torque and not beyond  # however hard the hand holds along the turn (data: 270-490 at 0.5-2 m/s^2)
      calm = calm and not fast
      self.release_frames = self.release_frames + 1 if calm else 0
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
      request = float(np.clip(request, angle - CONVERGE_DEG, angle + CONVERGE_DEG))
      request = float(np.clip(request, last - self.resume_rate, last + self.resume_rate))  # last, so the step cap holds whatever the wheel does
    if self.guard_active(angle, v_ego):
      request = float(np.clip(request, angle - GUARD_MAX_ERROR, angle + GUARD_MAX_ERROR))
      request = float(np.clip(request, -GUARD_MAX_ANGLE, GUARD_MAX_ANGLE))
    return request
