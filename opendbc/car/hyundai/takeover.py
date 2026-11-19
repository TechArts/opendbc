"""GV70 angle steering: a driver takeover (wheel) is a pause, not a fight.

While paused the controller sends 0xCB INACTIVE at the measured angle with gain 0 (no counter-force, no angle error for the EPS to fault on).
When the driver lets go the request is blended back to the model: angle step limited (never more than the resume rate per frame, whatever the
wheel does; the low-speed guard's clips and the lead band that run after it stay within that step), gain ramped by the controller, and kept
within CONVERGE_DEG (CONVERGE_FREE_DEG outside the low-speed guard region) of the measured angle while the wheel moves slower than the resume rate.
Lateral never starts (engage, blinker pause end, takeover release) while the wheel still moves faster than that: it stays paused until it calms.
Also pauses on a temporary EPS fault and when the low-speed, large-angle cap keeps binding. Below 6 m/s at large angles the request is also kept from
running ahead of a lagging wheel (lead_band).
"""
import math
from collections import deque

import numpy as np

from opendbc.car import DT_CTRL

ENTER_FRAMES = 3  # |torque| above the enter threshold for this many frames starts a pause
RESUME_FRAMES = int(1.0 / DT_CTRL)  # angle step limit after a pause
FAULT_FRAMES = int(1.0 / DT_CTRL)  # stay paused this long after steerFaultTemporary clears
CONVERGE_DEG = 5.0  # request stays this close to the measured angle while resuming inside the low-speed guard region (step_capped)
# Outside the guard region the resume window is wider: routes 60-63 (hand off, 0-10 m/s, model asks 15 deg or more): while resuming the wheel trailed the
# model by 24 deg below 5 m/s and 9 deg at 5-10 m/s (2.5 and -3 deg when not resuming), the request held 5 deg ahead of the wheel and the EPS turned the
# wheel at 39 deg/s at most with an error of 3-6 deg (69-84 deg/s at 6-12 deg, gain 0.5 or more), less than the model's 50-100 deg/s turn-in.
CONVERGE_FREE_DEG = 12.0
# The window narrows from CONVERGE_FREE_DEG to CONVERGE_DEG over the CONVERGE_TAPER_* distance to the guard region (wheel up to STEP_CAP_ANGLE, speed
# down to STEP_CAP_SPEED), so a resumed request never leads the wheel by more than the window inside the region allows (+- CONVERGE_DEG, last +- 1 deg)
# on the frame the region starts. The widest narrowing per frame (7 deg / 40 deg x the wheel's deg per frame) stays below the resume rate.
CONVERGE_TAPER_ANGLE = 40.0  # deg
CONVERGE_TAPER_SPEED = 3.0  # m/s
# the taper ends this far before the region, so the rate-limited band (self.band) has reached CONVERGE_DEG when it starts
CONVERGE_MARGIN_ANGLE = 10.0  # deg
CONVERGE_MARGIN_SPEED = 0.5  # m/s
WHEEL_RATE_FRAMES = 5  # frames the wheel's angular speed is measured over

# low-speed guard: both short EPS faults had a request 19-29 deg ahead of the wheel at 174-182 deg and 3-4 m/s
GUARD_SPEED = 5.0  # m/s
GUARD_ANGLE = 150.0  # deg
GUARD_MAX_ERROR = 10.0  # deg, |request - measured|
GUARD_MAX_ANGLE = 170.0  # deg, |request|
GUARD_FRAMES = int(0.5 / DT_CTRL)  # the cap must bind this long before it pauses steering
STEP_CAP_DEG = 1.0  # deg per frame, the most any ACTIVE request moves in the low-speed guard region, see limit()
STEP_CAP_SPEED = 6.0  # m/s
STEP_CAP_ANGLE = 140.0  # deg

# route 42 13:12.5-13:13.6 (3.2 m/s, hand off): the model's turn tightened faster than the EPS turned the wheel (about 1 deg per frame), the request
# ran 11-47 deg ahead (wheel 68 -> 144 deg) and the EPS faulted (AciFltSig_Lv2 = 2, then the camera's FAULT_*) as the wheel passed 140, before the
# guard's band (> GUARD_ANGLE) applied. Below LEAD_SPEED, once the wheel or the request is past LEAD_ANGLE, the request is moved toward the wheel +-
# LEAD_MAX_ERROR within what may be sent each frame (lead_band), from either side: while the wheel moves no faster than that allowance, a lead cannot
# build up and one that exists shrinks. Hands-off turns on routes 40-42 led by at most 7 deg, so the band binds only when request - wheel exceeds it.
LEAD_SPEED = STEP_CAP_SPEED  # m/s
LEAD_ANGLE = 90.0  # deg, |wheel| or |request|
LEAD_MAX_ERROR = GUARD_MAX_ERROR  # deg, |request - measured|

# curve-aware mode (HyundaiCurveHold, research/curve-hold.md Q2/Q5): in a curve only a pull against the turn, or a push deeper than the model
# asks, is a grab. A hand that holds the wheel along the turn (129 + 103 * a_lat torque once paused) is not.
CURVE_A_MIN = 0.3  # m/s^2, lateral load from which the wheel counts as in a curve
CURVE_DES_MIN = 2.0  # deg, and the model asks for at least this much angle
OPPOSE_RELEASE = 100  # opposing torque (curve sign) that blocks the release and drops the gain floor
ALONG_BASE = 300  # beyond-push clause: along-turn torque >= ALONG_BASE + ALONG_PER_A * a ...
ALONG_PER_A = 150
BEYOND_DEG = 3.0  # ... and the wheel at least this far past the model's angle in the push direction

# Route 3a 1:31.3, 1:32.6, 1:35.1, 1:47.1 (6-7 mph, wheel 200-310 deg, hand torque 250-520 along the turn): the curve rule above let ACTIVE frames
# (gain 0.03, request = wheel) through under a holding hand and the EPS faulted (AciFltSig_Lv2 = 2, then the camera's FAULT_*), 4 of 4 times.
# In the low-speed, large-angle region (step_capped) the straight-road rule applies whatever the curve: any hand on the wheel is a pause,
# released only by torque below the release value and no steeringPressed for the hold time. An EPS fault in the region latches the same rule
# until the hand has been off for the hold time (across lateral-inactive frames too), so there is no fixed-time retry under a holding hand, even if
# the wheel unwinds out of the region or latActive drops with the fault.


class TakeoverFilter:
  def __init__(self, enter_torque: int, release_torque: int, hold_ms: int, resume_rate_x10: int,
               angle_to_lat_accel: float = 0.0, curve_hold: bool = False, max_angle: float = math.inf, max_rate: float = math.inf):
    """Units are those of the params: torque units, ms, deg per frame x 10. enter_torque 0 turns the filter off.
    angle_to_lat_accel is 1 / (steerRatio * wheelbase * 180 / pi) in (m/s^2 per (m/s)^2 per deg); curve_hold turns on the curve-aware rules.
    max_angle is the largest angle the EPS accepts as a request (STEER_ANGLE_MAX, 175 deg on LFA_ALT), max_rate the largest step per frame it takes
    (LFA_ALT 2.2 deg): the resume rate never exceeds it, whatever HyundaiResumeAngleRate says."""
    self.enter_torque = enter_torque
    self.max_angle = max_angle
    self.release_torque = release_torque
    self.hold_frames = int(hold_ms / 1000.0 / DT_CTRL)
    self.resume_rate = min(resume_rate_x10 / 10.0, max_rate)
    self.paused = False
    self.resuming = False
    self.grab_frames = 0
    self.release_frames = 0
    self.resume_frames = 0
    self.fault_frames = 0
    self.strict = False  # an EPS fault in the guard region latched the straight-road rule until the hand has been off for the hold time
    self.strict_calm_frames = 0
    self.bind_frames = 0
    self.lat_active_last = False
    self.band = CONVERGE_DEG  # the resume window now (wheel +- this): converge_deg, narrowing no faster than the request can follow, see update()
    self.unsendable_frames = 0  # consecutive frames lateral was on without a sendable ACTIVE request, from the controller
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
    self.grab_frames = self.release_frames = self.resume_frames = self.bind_frames = self.unsendable_frames = 0

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
    # the strict latch lives across inactive frames (controlsd drops latActive on the fault itself, test_fault_guard): set by a fault in the guard
    # region; while set, steering is paused (below) whatever `paused` did across inactive frames, and it clears only with lateral active and the hand
    # off (|torque| < release, not pressed) for the full hold time with the fault gone
    if fault and self.step_capped(angle, v_ego):
      self.strict = True
    if not lat_active:
      self._reset()
      self.lat_active_last = False
      self.strict_calm_frames = 0
      return False
    self.strict_calm_frames = self.strict_calm_frames + 1 if abs(torque) < self.release_torque and not pressed else 0
    if self.strict and self.strict_calm_frames >= self.hold_frames and self.fault_frames == 0:
      self.strict = False
    in_region = self.step_capped(angle, v_ego)
    started = not self.lat_active_last
    if started:
      # lateral just came back (engage, blinker pause, MADS pause, fault): blend in like after a takeover, never snap to the model. Inside the guard
      # region it does not start at all (see the release below): it starts paused, so the controller sends inactive at the measured angle
      self.resuming = True
      self.resume_frames = RESUME_FRAMES
      if fast or in_region:
        self.paused = True
        self.resuming = False
        self.release_frames = 0
    self.lat_active_last = True

    # the resume window follows converge_deg, but its edge never retreats faster than the request can follow: the request moves at most the resume
    # rate per frame while the wheel (the band's centre) moves too, so the width narrows by at most the resume rate less the wheel's rate (a hard edge
    # that retreated faster left the request outside the band and the guard region's window empty: wheel 174 - 0.9 deg per frame, 7 - 0.05 m/s per
    # frame). Widening is immediate. The same band applies inside the region (window()), so it never jumps at the region's edge.
    target = self.converge_deg(angle, v_ego)
    if self.resuming and not started:
      step_wheel = abs(angle - self.angle_hist[-2]) if len(self.angle_hist) > 1 else 0.0
      self.band = target if target >= self.band else max(target, self.band - max(0.0, self.resume_rate - max(wheel_rate, step_wheel)))
    else:
      self.band = target

    binds = self.guard_active(angle, v_ego) and (abs(desired - angle) > GUARD_MAX_ERROR or abs(desired) > self.guard_max_angle(angle))
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
    if self.strict or self.step_capped(angle, v_ego):
      turn = 0  # route 3a: a hand on the wheel in the guard region is never "holding the curve"

    if turn == 0:
      grab = abs(torque) >= self.enter_torque
      press = pressed
    else:
      beyond = along >= ALONG_BASE + ALONG_PER_A * load and (angle - desired) * turn >= BEYOND_DEG
      grab = along <= -self.enter_torque or beyond
      press = pressed and along < 0  # steeringPressed from along-turn torque is the driver holding the curve
    self.grab_frames = self.grab_frames + 1 if grab else 0

    if not self.paused:
      if (press or self.strict or self.grab_frames >= ENTER_FRAMES or self.fault_frames > 0 or self.bind_frames > GUARD_FRAMES or
          self.unsendable_frames > GUARD_FRAMES):
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
      # route 45 7:46.7 (0.94 m/s, wheel -341 deg, light hand): a release inside the guard region gave 14 ACTIVE frames at gain 0.03, request 3 deg from
      # the wheel, and the EPS faulted (as on route 3a). No ACTIVE re-entry inside the region: a pause holds until the wheel is back under
      # STEP_CAP_ANGLE or the speed is STEP_CAP_SPEED or more (steering that is already ACTIVE when the wheel enters the region continues)
      if self.release_frames >= self.hold_frames and self.fault_frames == 0 and not binds and not in_region:
        self.paused = False
        self.resuming = True
        self.resume_frames = RESUME_FRAMES

    if self.resuming:
      self.resume_frames = max(0, self.resume_frames - 1)
      if self.resume_frames == 0 and abs(desired - angle) <= (CONVERGE_DEG if in_region else self.band):
        self.resuming = False
    return self.paused

  def guard_max_angle(self, angle: float) -> float:
    """|request| cap of the guard. Never below the wheel itself: a wheel already past GUARD_MAX_ANGLE (a tight turn, route 39 29:19.6, -216 deg)
    must not be pulled to the cap, that is a step of tens of degrees the panda rejects frame after frame. The request still never goes outward.
    Never above max_angle either: the EPS faults on a request beyond it (upstream LFA_ALT, 175 deg), so a wheel past max_angle + GUARD_MAX_ERROR
    leaves no request inside the guard band: inactive."""
    return min(max(GUARD_MAX_ANGLE, abs(angle)), self.max_angle)

  @classmethod
  def converge_deg(cls, angle: float, v_ego: float) -> float:
    """Resume window (wheel +- this): CONVERGE_DEG in the guard region (step_capped), CONVERGE_FREE_DEG away from it by either the wheel angle or the
    speed, linear in between: the region needs both a large angle and a low speed, so the nearer of the two margins sets the width."""
    far = max((STEP_CAP_ANGLE - CONVERGE_MARGIN_ANGLE - abs(angle)) / CONVERGE_TAPER_ANGLE,
              (v_ego - STEP_CAP_SPEED - CONVERGE_MARGIN_SPEED) / CONVERGE_TAPER_SPEED)
    return CONVERGE_DEG + (CONVERGE_FREE_DEG - CONVERGE_DEG) * min(max(far, 0.0), 1.0)

  @staticmethod
  def step_capped(angle: float, v_ego: float) -> bool:
    return v_ego < STEP_CAP_SPEED and abs(angle) > STEP_CAP_ANGLE

  def window(self, last: float, angle: float, v_ego: float) -> tuple[float, float]:
    """Inside the guard region: the intersection (lo, hi), empty when lo > hi, of every constraint on the request: the guard band (wheel +-10 deg,
    |request| <= guard_max_angle), the resume window (wheel +-CONVERGE_DEG, always, and last +-resume rate) and the step cap (last +-STEP_CAP_DEG).

    The step cap holds whatever the source of the change (guard clips, resume, tracking). Proof that the panda accepts it whatever its speed history is
    (lateral.h, steer_angle_cmd_checks_vm): the step limit is max_angle_delta = jerk / v^2 * k / 100 Hz with v = min of the panda's last six
    speed samples - 1 m/s, decreasing in v. Here our speed is < STEP_CAP_SPEED (6 m/s); the panda's samples are the same wheel speeds, at most 1 m/s
    above ours (100 Hz samples, a car below 6 m/s cannot change 1 m/s between two reads), so v < 6 m/s: at v = 6 m/s the limit is 2.6 deg per frame
    (26 CAN counts in the GV70 angle model, 23 in the baseline model it replaced; PandaAngleCheck.limits_can(7.0), checked against libsafety)
    > STEP_CAP_DEG = 1 deg, and it only grows below. The panda reads the
    previous frame we sent, which is `last` (the measured angle after an inactive frame)."""
    lo, hi = max(last - STEP_CAP_DEG, -self.max_angle), min(last + STEP_CAP_DEG, self.max_angle)
    if self.guard_active(angle, v_ego):
      cap = self.guard_max_angle(angle)
      lo, hi = max(lo, angle - GUARD_MAX_ERROR, -cap), min(hi, angle + GUARD_MAX_ERROR, cap)
    if self.resuming:
      lo, hi = max(lo, angle - CONVERGE_DEG, last - self.resume_rate), min(hi, angle + CONVERGE_DEG, last + self.resume_rate)  # hard, whatever self.band
    return lo, hi

  @staticmethod
  def lead_band(request: float, lo: float, hi: float, angle: float, v_ego: float) -> float:
    """The request within [lo, hi] (what may be sent this frame) and, below LEAD_SPEED once the wheel or the request is past LEAD_ANGLE, as close to
    the wheel +- LEAD_MAX_ERROR as [lo, hi] allows: inside the band where they overlap, else at the edge of [lo, hi] nearest to it. A request already
    outside the band converges by what [lo, hi] permits (gradual, not an instant cap), and the band never empties [lo, hi]."""
    limited = float(np.clip(request, lo, hi))
    if v_ego >= LEAD_SPEED or max(abs(angle), abs(request)) <= LEAD_ANGLE:
      return limited
    band_lo, band_hi = np.clip((angle - LEAD_MAX_ERROR, angle + LEAD_MAX_ERROR), lo, hi)  # no overlap: both at the edge of [lo, hi] nearest the band
    return float(np.clip(limited, band_lo, band_hi))

  def limit(self, request: float, last: float, angle: float, v_ego: float, backstop: tuple[float, float] | None = (-math.inf, math.inf)) -> float | None:
    """See _limit. An empty window while resuming near or in the guard region (resume window narrower than CONVERGE_FREE_DEG) sends inactive and latches the
    pause like a takeover: no ACTIVE frame follows before the hands-off hold, and none inside the region at all (the release needs the wheel out of it)."""
    out = self._limit(request, last, angle, v_ego, backstop)
    if out is None:
      self.latch_empty(angle, v_ego)
    return out

  def latch_empty(self, angle: float, v_ego: float):
    """No ACTIVE request exists this frame (the controller sends inactive): while resuming near or in the region that is a takeover-like pause. The
    controller calls it for the vehicle-model limiter's None too."""
    if self.enabled and self.resuming and (self.step_capped(angle, v_ego) or self.converge_deg(angle, v_ego) < CONVERGE_FREE_DEG):
      self.paused = True
      self.resuming = False
      self.release_frames = 0

  def _limit(self, request: float, last: float, angle: float, v_ego: float, backstop: tuple[float, float] | None = (-math.inf, math.inf)) -> float | None:
    """Applies the low-speed cap, the lead band and the resume blend to a request that already passed the vehicle-model limits.

    `backstop` is the window the controller computes for every frame from what the upstream vehicle model (jerk and lateral accel), the EPS rate limit
    and the panda's own check accept after `last`; the result is always inside it and inside +-max_angle (the 175 deg EPS limit), whatever the region.
    Outside the guard region (step_capped false): the resume blend (wheel +-CONVERGE_FREE_DEG, last +- the resume rate), then the lead band within what may
    be sent: anything between last and that request, or a step from last of the resume rate while resuming, else STEP_CAP_DEG (accepted by the panda
    below 6 m/s, see window()). Nothing changes at LEAD_SPEED and above or within LEAD_ANGLE. Inside: the request moved to the nearest point of window()
    intersected with `backstop`, then the lead band within that. None: the intersection is empty (a wheel that moves against the band, a step the panda or
    the vehicle model would not accept, the resume window out of reach): send inactive at the measured angle, the next frame starts from there. The lead
    band never makes it empty."""
    if not self.enabled:
      return request
    if backstop is None:
      return None
    lo, hi = max(backstop[0], -self.max_angle), min(backstop[1], self.max_angle)
    if not self.step_capped(angle, v_ego):
      step = STEP_CAP_DEG
      if self.resuming:
        # the step from last (resume rate, inside the vehicle-model / EPS / panda window) is hard whatever the wheel does; the convergence to the wheel
        # only moves the request inside it
        lo, hi = max(lo, last - self.resume_rate), min(hi, last + self.resume_rate)
        step = self.resume_rate
        if lo > hi:
          return None
        request = float(np.clip(np.clip(request, angle - self.band, angle + self.band), lo, hi))
      lo, hi = max(lo, min(request, last - step)), min(hi, max(request, last + step))
      if lo > hi:
        return None
      return self.lead_band(request, lo, hi, angle, v_ego)
    wlo, whi = self.window(last, angle, v_ego)
    lo, hi = max(lo, wlo), min(hi, whi)
    if lo > hi:
      return None
    return self.lead_band(request, lo, hi, angle, v_ego)

  def note_unsendable(self, unsendable: bool):
    """Once per frame by the controller: True when lateral is on but no ACTIVE request exists (inactive frame sent instead)."""
    self.unsendable_frames = self.unsendable_frames + 1 if unsendable else 0
