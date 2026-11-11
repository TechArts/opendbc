import unittest
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.hyundai.takeover import TakeoverFilter, ENTER_FRAMES, RESUME_FRAMES, FAULT_FRAMES, GUARD_FRAMES, CONVERGE_DEG
from opendbc.car.hyundai.tests.gv70 import dbc_defaults, gv70_controller
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP


def make(enter=150, release=100, hold_ms=500, rate=10):
  return TakeoverFilter(enter, release, hold_ms, rate)


def run(f, frames, torque=0.0, pressed=False, fault=False, v=15.0, angle=10.0, desired=10.0, active=True):
  paused = []
  for _ in range(frames):
    paused.append(f.update(torque, pressed, fault, v, angle, desired, active))
  return paused


class TestTakeoverFilter(unittest.TestCase):
  def test_enter_needs_three_frames(self):
    f = make()
    self.assertFalse(any(run(f, ENTER_FRAMES - 1, torque=200)))
    run(f, 1, torque=100)  # counter resets below the threshold
    self.assertFalse(any(run(f, ENTER_FRAMES - 1, torque=-200)))
    self.assertTrue(run(f, 1, torque=-200)[-1])

  def test_below_threshold_never_pauses(self):
    self.assertFalse(any(run(make(), 500, torque=149)))

  def test_steering_pressed_pauses_at_once(self):
    self.assertTrue(run(make(), 1, torque=0, pressed=True)[0])

  def test_pause_also_at_standstill(self):
    self.assertTrue(run(make(), ENTER_FRAMES, torque=200, v=0.0, angle=0.0, desired=0.0)[-1])

  def test_exit_hysteresis_and_hold(self):
    f = make()
    run(f, 10, torque=300)
    self.assertTrue(f.paused)
    self.assertTrue(all(run(f, 200, torque=120)))  # between release and enter: stays paused
    self.assertTrue(all(run(f, 49, torque=50)))  # 0.5 s hold not complete
    run(f, 1, torque=120)  # a touch above release restarts the hold
    self.assertTrue(all(run(f, 49, torque=50)))
    self.assertFalse(run(f, 1, torque=50)[-1])
    self.assertTrue(f.resuming)

  def test_regrab_during_hold_keeps_pause(self):
    f = make()
    run(f, 10, torque=300)
    run(f, 40, torque=0)
    self.assertTrue(all(run(f, 10, torque=300)))
    self.assertTrue(all(run(f, 49, torque=0)))

  def test_setting_zero_is_off(self):
    f = make(enter=0)
    self.assertFalse(f.enabled)
    self.assertFalse(any(run(f, 100, torque=500, pressed=True, fault=True, v=2.0, angle=175.0, desired=215.0)))
    self.assertEqual(f.limit(215.0, 170.0, 175.0, 2.0), 215.0)

  def test_not_lat_active_resets(self):
    f = make()
    run(f, 10, torque=300)
    self.assertFalse(run(f, 1, torque=300, active=False)[0])
    self.assertFalse(f.paused or f.resuming)

  def test_any_lateral_resume_blends_in(self):
    # e.g. the end of a blinker pause at low speed: the request starts at the wheel and moves at most 1 deg per frame
    f = make()
    run(f, 20, active=False, angle=20.0)
    f.update(0.0, False, False, 3.0, 20.0, 60.0, True)
    self.assertTrue(f.resuming)
    self.assertEqual(f.limit(60.0, 20.0, 20.0, 3.0), 21.0)

  def _unwinding(self, f, frames, start, rate, active=True):
    """the wheel moves rate deg per frame from start (the driver finishing a turn); returns the paused flags and the last angle"""
    out, angle = [], start
    for _ in range(frames):
      out.append(f.update(0.0, False, False, 6.7, angle, angle + 20.0, active))
      angle += rate
    return out, angle

  def test_resume_with_a_fast_wheel_stays_paused_until_it_calms(self):
    # route 34 3:25.3: the blinker pause ended while the wheel unwound at 1.5-2.3 deg per frame at 15 mph
    f = make()
    self._unwinding(f, 20, -150.0, 2.3, active=False)
    paused, angle = self._unwinding(f, 40, -104.0, 2.0)
    self.assertTrue(all(paused))  # never an active request against a wheel that moves faster than the blend
    self.assertFalse(f.resuming)
    paused, angle = self._unwinding(f, 60, angle, 0.2)  # the wheel calms: released after the 0.5 s hold, then blended in
    self.assertTrue(paused[0] and not paused[-1])
    self.assertTrue(f.resuming)

  def test_resume_with_a_slow_wheel_is_unchanged(self):
    f = make()
    self._unwinding(f, 20, -50.0, 0.5, active=False)
    paused, _ = self._unwinding(f, 5, -40.0, 0.5)
    self.assertFalse(any(paused))
    self.assertTrue(f.resuming)

  def test_request_step_never_exceeds_the_resume_rate_even_when_the_wheel_moves_fast(self):
    # the wheel starts slow (resume accepted), then speeds up: the convergence limit must not drag the request along at the wheel's speed
    f = make()
    self._unwinding(f, 20, -86.0, 0.2, active=False)
    last, angle = -82.0, -82.0
    steps = []
    for _ in range(40):
      f.update(0.0, False, False, 6.7, angle, angle + 30.0, True)
      if f.paused:
        break
      req = f.limit(angle + 30.0, last, angle, 6.7)
      steps.append(abs(req - last))
      last, angle = req, angle + 1.9
    self.assertTrue(steps and max(steps) <= 1.0 + 1e-9, steps)

  def test_resume_ramp_and_convergence(self):
    f = make()
    run(f, 10, torque=300)
    run(f, 50, torque=0)
    self.assertFalse(f.paused)
    measured, last = 10.0, 10.0
    desired = 60.0  # the model is far from where the driver left the wheel
    for _ in range(RESUME_FRAMES):
      f.update(0.0, False, False, 15.0, measured, desired, True)
      req = f.limit(desired, last, measured, 15.0)
      self.assertLessEqual(abs(req - last), 1.0 + 1e-9)
      self.assertLessEqual(abs(req - measured), CONVERGE_DEG + 1e-9)
      last = req
    self.assertTrue(f.resuming)  # model still more than 5 deg away from the wheel
    f.update(0.0, False, False, 15.0, 58.0, desired, True)
    self.assertFalse(f.resuming)
    self.assertEqual(f.limit(desired, 58.0, 58.0, 15.0), desired)

  def test_resume_rate_param(self):
    f = make(rate=5)
    run(f, 10, torque=300)
    run(f, 50, torque=0)
    self.assertEqual(f.limit(50.0, 10.0, 10.0, 15.0), 10.5)

  def test_fault_guard(self):
    f = make()
    run(f, 1, fault=True, active=False)  # controlsd drops latActive during the fault
    run(f, 5, fault=False, active=False)
    paused = run(f, FAULT_FRAMES + 60, fault=False)
    self.assertTrue(all(paused[:FAULT_FRAMES - 5 - 1]))  # one second after the flag cleared
    self.assertFalse(paused[-1])
    self.assertTrue(f.resuming)

  def test_fault_while_active_pauses_even_without_torque(self):
    f = make()
    self.assertTrue(run(f, 1, fault=True)[0])
    self.assertTrue(all(run(f, FAULT_FRAMES - 1, fault=False)))


class TestLowSpeedGuard(unittest.TestCase):
  def test_cap(self):
    f = make()
    self.assertEqual(f.limit(215.0, 176.0, 175.0, 4.0), 175.0)  # wheel already past the cap: the request stays at the wheel, never pulled inward
    self.assertEqual(f.limit(160.0, 165.5, 175.0, 4.0), 165.0)  # error cap
    self.assertEqual(f.limit(-215.0, -176.0, -175.0, 4.0), -175.0)
    self.assertEqual(f.limit(168.0, 167.5, 160.0, 4.0), 168.0)
    self.assertEqual(f.limit(175.0, 169.5, 160.0, 4.0), 170.0)  # wheel within the cap: the cap applies
    self.assertEqual(f.limit(175.0, 160.0, 160.0, 4.0), 161.0)  # and no step above 1 deg per frame in the guard region
    self.assertIsNone(f.limit(175.0, 140.0, 160.0, 4.0))  # the capped step cannot reach the guard's band: inactive
    self.assertEqual(f.limit(175.0, 160.0, 160.0, 6.0), 175.0)  # above the cap speed
    self.assertEqual(f.limit(215.0, 176.0, 175.0, 6.0), 215.0)  # fast enough
    self.assertEqual(f.limit(120.0, 100.0, 100.0, 4.0), 120.0)  # small angle

  def test_cap_binding_pauses_after_half_second_and_stays_until_released(self):
    f = make()
    paused = run(f, GUARD_FRAMES + 5, v=4.0, angle=175.0, desired=205.0)
    self.assertFalse(paused[GUARD_FRAMES - 1])
    self.assertTrue(paused[-1])
    self.assertTrue(all(run(f, 200, v=4.0, angle=175.0, desired=205.0)))  # still binding: stay paused, no flapping
    self.assertFalse(run(f, 60, v=4.0, angle=175.0, desired=168.0)[-1])

  def test_no_pause_when_cap_does_not_bind(self):
    self.assertFalse(any(run(make(), 300, v=4.0, angle=175.0, desired=168.0)))

  def test_short_binding_does_not_pause(self):
    f = make()
    run(f, GUARD_FRAMES - 5, v=4.0, angle=175.0, desired=205.0)
    self.assertFalse(any(run(f, 100, v=4.0, angle=175.0, desired=170.0)))


def _carcontroller(takeover_torque, curve_hold=False):
  def tweak_sp(cp_sp):
    cp_sp.hyundaiTakeoverTorque = takeover_torque
    cp_sp.hyundaiTakeoverReleaseTorque, cp_sp.hyundaiTakeoverHoldMs, cp_sp.hyundaiResumeAngleRateX10 = 100, 500, 10
    cp_sp.flags |= HyundaiFlagsSP.STEER_FEEL_STOCK | HyundaiFlagsSP.STEER_FEEL_ALL_SPEEDS
    if curve_hold:
      cp_sp.flags |= HyundaiFlagsSP.CURVE_HOLD
  _, _, cc, cs = gv70_controller(tweak_sp=tweak_sp)
  return cc, cs


class TestControllerTakeover(unittest.TestCase):
  def _update(self, cc, cs, torque, angle_cmd=30.0, angle=5.0):
    cs.out = structs.CarState(vEgo=10.0, vEgoRaw=10.0, steeringTorque=torque, steeringAngleDeg=angle)
    cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    cs.cam_lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
    cs.msg_161, cs.msg_162, cs.msg_1b5 = (dbc_defaults(m) for m in ("CCNC_0x161", "CCNC_0x162", "FR_CMR_03_50ms"))
    cs.main_cruise_enabled = True
    cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0}
    cc.tuning = SimpleNamespace(stopping=False, actual_accel=0.0, jerk_lower=0.5, jerk_upper=0.5)
    CC = structs.CarControl(latActive=True)
    CC.actuators.steeringAngleDeg = angle_cmd
    act, _ = cc.update(CC.as_reader(), structs.CarControlSP(), cs, 0)
    return act

  def test_pause_sends_measured_angle_and_zero_gain(self):
    cc, cs = _carcontroller(150)
    for _ in range(20):
      act = self._update(cc, cs, 0.0)
    self.assertGreater(act.torqueOutputCan, 0)
    for _ in range(ENTER_FRAMES + 1):
      act = self._update(cc, cs, 300.0)
    self.assertTrue(cc.takeover.paused)
    self.assertEqual(act.torqueOutputCan, 0)
    self.assertEqual(act.steeringAngleDeg, 5.0)

  def test_setting_zero_does_not_pause(self):
    cc, cs = _carcontroller(0)
    for _ in range(20):
      act = self._update(cc, cs, 300.0)
    self.assertFalse(cc.takeover.paused)
    self.assertGreater(act.torqueOutputCan, 0)

  def test_curve_hold_flag_holds_along_turn_push_in_controller(self):
    # a = 10^2 * 30 deg * k = 1.2 m/s^2, wheel on the model, 250 torque along the turn: off pauses (today), on stays active at the gain floor
    outputs = {}
    for on in (False, True):
      cc, cs = _carcontroller(150, curve_hold=on)
      for _ in range(400):
        act = self._update(cc, cs, 250.0, angle_cmd=30.0, angle=30.0)
      outputs[on] = (cc.takeover.paused, act.torqueOutputCan)
    self.assertEqual(outputs[False], (True, 0))
    self.assertFalse(outputs[True][0])
    self.assertGreater(outputs[True][1], 0)


K = 1.0 / (15.2 * 2.875 * 57.2958)  # deg -> m/s^2 per (m/s)^2, as the controller computes it


def make_curve(**kw):
  return TakeoverFilter(150, 100, 500, 10, K, curve_hold=True, **kw)


class TestCurveTakeover(unittest.TestCase):
  """v 15 m/s, model and wheel at 17 deg: a = 1.53 m/s^2, a turn in the positive direction (along = torque)."""

  def curve(self, f, frames, torque, **kw):
    kw.setdefault("angle", 17.0)
    kw.setdefault("desired", 17.0)
    return run(f, frames, torque=torque, **kw)

  def test_along_turn_hold_never_pauses_also_with_pressed(self):
    for sign in (1, -1):
      f = make_curve()
      self.assertFalse(any(self.curve(f, 100, 250 * sign, angle=17.0 * sign, desired=17.0 * sign)))
      self.assertFalse(any(self.curve(f, 100, 250 * sign, pressed=True, angle=17.0 * sign, desired=17.0 * sign)))

  def test_opposing_grab_pauses_by_frame_three(self):
    for sign in (1, -1):
      f = make_curve()
      paused = self.curve(f, ENTER_FRAMES, -160 * sign, angle=17.0 * sign, desired=17.0 * sign)
      self.assertEqual(paused, [False, False, True])

  def test_opposing_pressed_pauses_at_once(self):
    self.assertTrue(self.curve(make_curve(), 1, -200, pressed=True)[0])

  def test_guard_region_hand_along_the_turn_pauses_and_releases_like_straight(self):
    """Route 3a: v 2.68 m/s, wheel and model 250: a = 0.7 m/s^2, a turn. In the region any torque or steeringPressed pauses, along the turn too."""
    for sign in (1, -1):
      a = 250.0 * sign
      kw = dict(v=2.68, angle=a, desired=a)
      f = make_curve()
      self.assertTrue(run(f, 1, torque=0.0, pressed=True, **kw)[0])
      f = make_curve()
      self.assertTrue(run(f, ENTER_FRAMES, torque=250 * sign, **kw)[-1])  # along-turn torque from the 3rd frame, unpressed
      hold = f.hold_frames
      self.assertTrue(all(run(f, hold + 50, torque=300 * sign, pressed=True, **kw)))
      self.assertTrue(all(run(f, hold - 1, torque=0.0, **kw)))
      self.assertFalse(run(f, 1, torque=0.0, **kw)[-1])
    f = make_curve()  # the same hand above the region's speed: not a grab (F22)
    self.assertFalse(any(run(f, 100, torque=300, pressed=True, v=8.0, angle=250.0, desired=250.0)))

  def test_random_sequences_after_a_fault_no_active_frame_until_a_release_hold(self):
    """State machine: random latActive toggles, fault flags, torque 0-600, pressed, wheel in/out of the region. After a fault in the region the filter
    returns paused (no ACTIVE frame) every active frame until lateral has been active with |torque| < release and not pressed for hold frames
    with the fault gone."""
    import random
    for seed in range(300):
      rng = random.Random(seed)
      f = make_curve()
      latched, calm = False, 0
      angle, v = 250.0, 2.68
      for i in range(3000):
        if rng.random() < 0.02:
          angle = rng.choice((250.0, 120.0, 100.0, -250.0, -120.0))
          v = rng.choice((2.68, 2.68, 5.0, 8.0))
        tq = rng.choice((0.0, 0.0, 50.0, 99.0, 120.0, 200.0, 300.0, 600.0)) * rng.choice((1, -1))
        pr = rng.random() < 0.2
        act = rng.random() < (0.9 if i % 400 < 300 else 0.5)
        fault = rng.random() < (0.01 if latched else 0.003)
        if rng.random() < 0.01:
          fault, tq, pr = False, 0.0, False  # a long hand-off now and then
        region = v < 6.0 and abs(angle) > 140.0
        if fault and region:
          latched = True
        paused = f.update(tq, pr, fault, v, angle, angle, act)
        if not act:
          calm = 0
          continue
        calm = calm + 1 if abs(tq) < 100 and not pr else 0
        if latched and calm >= f.hold_frames and f.fault_frames == 0:
          latched = False
          continue
        if latched:
          self.assertTrue(paused, f"seed {seed} frame {i}: ACTIVE under the latch")

  def test_fault_in_the_region_latches_the_straight_release_until_the_hand_is_off(self):
    f = make_curve()
    kw = dict(angle=250.0, desired=250.0, v=2.68)
    run(f, 20, **kw)
    run(f, FAULT_FRAMES, torque=300, pressed=True, fault=True, active=False, **kw)  # controlsd drops latActive on the fault itself (R1Guard)
    self.assertTrue(f.strict)
    out = dict(angle=120.0, desired=120.0, v=2.68)  # wheel unwound out of the region but still a curve for F22 (a = 0.34), the hand still holds
    self.assertTrue(all(run(f, 5 * FAULT_FRAMES, torque=300, pressed=True, **out)))
    self.assertTrue(all(run(f, f.hold_frames - 1, **out)))
    self.assertFalse(run(f, 1, **out)[-1])
    self.assertFalse(f.strict)
    h = make_curve()  # the latch survives lateral toggling, and is not stuck when the hand stays off
    run(h, FAULT_FRAMES, torque=300, pressed=True, fault=True, active=False, **kw)
    run(h, 1, torque=300, pressed=True, active=False, **out)
    self.assertTrue(h.strict)
    run(h, h.hold_frames + FAULT_FRAMES + 2, active=False, **out)
    self.assertTrue(h.strict)  # a hand-off time spent inactive does not count
    self.assertTrue(all(run(h, h.hold_frames - 1, **out)))
    self.assertTrue(h.strict)
    self.assertFalse(run(h, 3, **out)[-1])  # strict clears at the hold frame, the pause itself one frame later
    self.assertFalse(h.strict)
    # R1Guard round 8: latch set while inactive, then a torque between release (100) and enter (150), unpressed, fault gone: still no ACTIVE
    for tq in (120.0, 200.0):
      k = make_curve()
      run(k, FAULT_FRAMES, torque=300, pressed=True, fault=True, active=False, **kw)
      run(k, FAULT_FRAMES + 5, torque=tq, active=False, **out)
      self.assertTrue(k.strict)
      self.assertEqual(k.paused, False)  # `paused` did not survive the inactive frames: strict alone gates
      self.assertTrue(all(run(k, 200, torque=tq, **out)), f"torque {tq}: an ACTIVE frame under a hand")
      self.assertTrue(k.strict)
      self.assertTrue(all(run(k, k.hold_frames - 1, **out)))
      self.assertFalse(run(k, 1, **out)[-1])
      self.assertFalse(k.strict)
    g = make_curve()  # a fault outside the region does not latch anything
    run(g, 5, torque=300, fault=True, v=15.0, angle=17.0, desired=17.0)
    self.assertFalse(g.strict)

  def test_straight_is_unchanged(self):
    for kw in (dict(angle=1.0, desired=1.0), dict(angle=17.0, desired=1.0, v=0.5)):
      f = make_curve()
      self.assertEqual(self.curve(f, ENTER_FRAMES, 150, **kw), [False, False, True])  # along or opposing: symmetric
      self.assertTrue(self.curve(make_curve(), 1, 0, pressed=True, **kw)[0])

  def test_disabled_flag_is_todays_behaviour(self):
    f = TakeoverFilter(150, 100, 500, 10, K, curve_hold=False)
    self.assertTrue(self.curve(f, ENTER_FRAMES, 250)[-1])

  def test_beyond_push_pauses_only_with_the_wheel_past_the_model(self):
    # a = 1.53 -> along-turn threshold 300 + 150 * 1.53 = 530
    self.assertFalse(any(self.curve(make_curve(), 50, 700, angle=17.0, desired=17.0)))
    self.assertFalse(any(self.curve(make_curve(), 50, 700, angle=19.0, desired=17.0)))  # 2 deg: not beyond
    self.assertFalse(any(self.curve(make_curve(), 50, 500, angle=20.5, desired=17.0)))  # beyond but below the torque bar
    self.assertTrue(self.curve(make_curve(), ENTER_FRAMES, 700, angle=20.5, desired=17.0)[-1])
    self.assertTrue(self.curve(make_curve(), ENTER_FRAMES, -700, angle=-20.5, desired=-17.0)[-1])
    self.assertFalse(any(self.curve(make_curve(), 50, 700, angle=13.5, desired=17.0)))  # wheel short of the model, not past it

  def test_curve_release_while_holding_along_the_turn(self):
    # latched pause at a = 1.75 (v 15, 19.5 deg) with 490 along-turn torque, the hand holds the curve: ends after the hold time, also pressed
    f = make_curve()
    self.curve(f, 10, -300, angle=19.5, desired=19.5)  # opposing grab
    self.assertTrue(f.paused)
    self.assertTrue(all(self.curve(f, 49, 490, pressed=True, angle=19.5, desired=19.5)))
    self.assertFalse(self.curve(f, 1, 490, pressed=True, angle=19.5, desired=19.5)[-1])
    self.assertTrue(f.resuming)

  def test_curve_release_blocked_by_opposing(self):
    f = make_curve()
    self.curve(f, 10, -300)
    self.assertTrue(all(self.curve(f, 200, -120)))
    self.assertTrue(all(self.curve(f, 49, 0)))
    self.curve(f, 1, -120)  # restarts the hold
    self.assertTrue(all(self.curve(f, 49, 0)))
    self.assertFalse(self.curve(f, 1, 0)[-1])

  def test_curve_release_blocked_by_beyond_push(self):
    f = make_curve()
    self.curve(f, 10, -300)
    self.assertTrue(all(self.curve(f, 200, 700, angle=20.5, desired=17.0)))  # along >= 530 and wheel 3.5 deg past the model
    self.assertFalse(self.curve(f, 50, 700, angle=17.0, desired=17.0)[-1])  # wheel back on the model: releases

  def test_exposes_load_and_opposing_for_the_gain(self):
    f = make_curve()
    self.curve(f, 1, 0)
    self.assertAlmostEqual(f.hold_load, 15.0 ** 2 * 17.0 * K)
    self.assertFalse(f.opposing)
    self.curve(f, 1, -100)
    self.assertTrue(f.opposing)
    self.curve(f, 1, 250)
    self.assertFalse(f.opposing)
    g = make_curve()
    self.curve(g, 1, -150, angle=1.0, desired=1.0)
    self.assertFalse(g.opposing)  # straight: no curve sign


class TestBlockedResumeRoute39(unittest.TestCase):
  """Route 39 29:19.6: the blinker pause ends at 4.76 m/s with the wheel at -216 deg (past the guard's 170 deg). The guard pulled the request to
  -170, 46 deg from the wheel; the panda rejected all 50 frames (no 0xCB reached the car for 0.52 s), the EPS faulted, the icon was green.
  Closed loop against the real panda safety (libsafety)."""

  V, WHEEL, MODEL = 4.76, -216.0, -253.0
  ADDR = 0xCB

  def setUp(self):
    from opendbc.safety.tests.common import CANPackerSafety
    from opendbc.safety.tests.libsafety import libsafety_py
    self.libsafety_py = libsafety_py
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.cc, self.cs = _carcontroller(150, curve_hold=True)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, int(self.cc.CP.safetyConfigs[-1].safetyParam))
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)
    self.t = 0

  def _frame(self, lat_active, angle, model=None, v=None, wheel_counts=None, extra_counts=(), v_ego=None, torque=0.0, pressed=False, fault=False):
    """One controller cycle into the real panda. Returns (0xCB frames, accepted, active, icon, request angle). extra_counts: wheel-speed RX frames
    the panda gets before this cycle's one, which the controller never reads."""
    v = self.V if v is None else v
    self.t += 10000
    self.safety.set_timer(self.t)
    wheels = {f"WHL_Spd{p}Val": v * 3.6 for p in ("FL", "FR", "RL", "RR")}
    if wheel_counts is not None:  # raw 0.03125 kph counts per wheel as on the bus; vEgoRaw is their mean, as the car state computes it
      wheels = {f"WHL_Spd{p}Val": c * 0.03125 for p, c in zip(("FL", "FR", "RL", "RR"), wheel_counts)}
      v = sum(wheel_counts) / 4 * 0.03125 / 3.6
    for extra in extra_counts:
      self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, {f"WHL_Spd{p}Val": c * 0.03125 for p, c in zip(("FL", "FR", "RL", "RR"), extra)}))
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, wheels))
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("MDPS", 0, {"MDPS_EstStrAnglVal": angle}))
    self.cs.out = structs.CarState(vEgo=v if v_ego is None else v_ego, vEgoRaw=v, steeringTorque=torque, steeringPressed=pressed,
                                 steerFaultTemporary=fault, steeringAngleDeg=angle)
    self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    self.cs.cam_lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
    self.cs.msg_161, self.cs.msg_162, self.cs.msg_1b5 = (dbc_defaults(m) for m in ("CCNC_0x161", "CCNC_0x162", "FR_CMR_03_50ms"))
    self.cs.main_cruise_enabled = True
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0}
    self.cc.tuning = SimpleNamespace(stopping=False, actual_accel=0.0, jerk_lower=0.5, jerk_upper=0.5)
    CC = structs.CarControl(latActive=lat_active, enabled=lat_active)
    CC.actuators.steeringAngleDeg = self.MODEL if model is None else model
    act, sends = self.cc.update(CC.as_reader(), structs.CarControlSP(), self.cs, 0)
    cb = [s for s in sends if s[0] == self.ADDR]
    accepted = [bool(self.safety.safety_tx_hook(self.libsafety_py.make_CANPacket(s[0], s[2], s[1]))) for s in cb]
    active = [(bytes(s[1])[3] >> 4) == 2 for s in cb]
    return cb, accepted, active, self.cc.dash_states(self.cs, CC.as_reader())[0], act.steeringAngleDeg

  def _replay(self, wheel_at, wheel0=None, model=None, v=None, frames=200):
    """Blinker pause (100 frames inactive), then lateral comes back; wheel_at(i) is the measured angle at resume frame i.
    Every frame: one 0xCB, accepted, green icon only with an ACTIVE frame. Returns [(active, request angle, icon)] of the resume frames."""
    from opendbc.car.hyundai import hyundaicanfd
    wheel0 = self.WHEEL if wheel0 is None else wheel0
    for _ in range(100):
      cb, ok, active, _, _ = self._frame(False, wheel0, model, v)
      self.assertEqual((len(cb), ok, active), (1, [True], [False]))
    out = []
    for i in range(frames):
      cb, ok, active, icon, req = self._frame(True, wheel_at(i), model, v)
      self.assertEqual(len(cb), 1, f"frame {i}: TX gap")
      self.assertEqual(ok, [True], f"frame {i}: the panda rejected the frame")
      if icon == hyundaicanfd.ICON_ACTIVE:
        self.assertEqual(active, [True], f"frame {i}: icon green without an ACTIVE frame")
      out.append((active[0], req, icon))
    return out

  def _assert_resume_steps(self, out):
    """Between consecutive ACTIVE frames of the resume (RESUME_FRAMES) the request never moves more than the resume rate (1 deg/frame)."""
    for i in range(1, RESUME_FRAMES):
      if out[i - 1][0] and out[i][0]:
        self.assertLessEqual(abs(out[i][1] - out[i - 1][1]), 1.0 + 1e-6, f"frame {i}: resume step")

  def test_resume_at_minus_216_never_rejected_no_gap_icon_follows_the_frames(self):
    out = self._replay(lambda i: self.WHEEL)
    self.assertGreater(sum(a for a, _, _ in out), 20)  # ACTIVE is actually delivered, not an all-INACTIVE pass
    self._assert_resume_steps(out)

  def test_resume_with_a_slowly_unwinding_wheel(self):
    out = self._replay(lambda i: self.WHEEL + 0.5 * i)
    self.assertGreater(sum(a for a, _, _ in out), 20)
    self._assert_resume_steps(out)

  def test_resume_with_a_fast_wheel(self):
    out = self._replay(lambda i: self.WHEEL + 1.3 * i)
    self._assert_resume_steps(out)  # 1.3 deg/frame trajectory: steps stay <= 1 deg between ACTIVE frames (inactive where none exists)

  def test_guard_boundaries_both_signs(self):
    """150 / 170 deg wheel and 5 m/s, positive and negative: resume slowly unwinding, model 1.1x the wheel; all frames accepted, steps <= 1 deg."""
    for sign in (1, -1):
      for wheel in (149.0, 151.0, 169.0, 171.0, 216.0):
        for v in (4.9, 5.1):
          with self.subTest(wheel=sign * wheel, v=v):
            self.setUp()
            out = self._replay(lambda i: sign * (wheel - 0.3 * i), wheel0=sign * wheel, model=sign * wheel * 1.1, v=v)
            self.assertGreater(sum(a for a, _, _ in out), 20)
            self._assert_resume_steps(out)

  def test_unsendable_wheel_pauses_with_a_grey_icon(self):
    """Wheel +-350, model 345 at 4.9 m/s: the model limits leave no legal ACTIVE command. Inactive frames only (one per cycle, accepted), then an
    explicit pause after the guard time, grey the whole time."""
    from opendbc.car.hyundai import hyundaicanfd
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.setUp()
        paused_at = None
        for _ in range(100):
          self._frame(False, sign * 350.0, sign * 345.0, 4.9)
        for i in range(120):
          cb, ok, active, icon, _ = self._frame(True, sign * 350.0, sign * 345.0, 4.9)
          self.assertEqual((len(cb), ok, active), (1, [True], [False]))
          self.assertEqual(icon, hyundaicanfd.ICON_PAUSED)
          if self.cc.takeover.paused and paused_at is None:
            paused_at = i
        self.assertIsNotNone(paused_at)
        self.assertGreater(paused_at, GUARD_FRAMES)
        self.assertLess(paused_at, GUARD_FRAMES + 5)

  def test_guard_step_cap_never_exceeds_one_degree_and_tracking_delivers_active(self):
    """last 146, wheel 160 (a contrived gap): the capped step cannot reach the guard's band, so inactive (accepted) and the next frame starts at the wheel;
    then tracking a model that moves 0.5 deg per frame delivers ACTIVE every frame, steps <= 1 deg."""
    self._replay(lambda i: 160.0, wheel0=160.0, model=160.0, frames=150)
    self.assertFalse(self.cc.takeover.resuming)
    self.cc.apply_angle_last = 146.0
    self.safety.set_desired_angle_last(1460)
    cb, ok, active, icon, req = self._frame(True, 160.0, 160.0)
    self.assertEqual((ok, active), ([True], [False]))
    prev = None
    for i in range(100):
      cb, ok, active, icon, req = self._frame(True, 160.0 + 0.5 * i, 160.0 + 0.5 * i + 3.0)
      self.assertEqual((len(cb), ok, active), (1, [True], [True]), f"frame {i}")
      if prev is not None:
        self.assertLessEqual(abs(req - prev), 1.0 + 1e-6)
      prev = req

  def test_random_rx_out_of_step_with_controller_updates(self):
    """Random wheel / model / speed walks (rising and falling speed below 6 m/s) with 0-5 extra RX wheel-speed frames the controller never reads
    between updates: no rejected frame, no gap, ACTIVE steps <= 1 deg, green only with ACTIVE, ACTIVE is delivered."""
    import random
    for seed in range(8):
      rng = random.Random(seed)
      self.setUp()
      sign = rng.choice((1, -1))
      wheel = sign * rng.uniform(145.0, 250.0)
      counts = rng.randint(300, 640)
      for _ in range(100):
        self._frame(False, wheel, wheel, wheel_counts=(counts,) * 4)
      prev, n_active = None, 0
      drift = rng.choice((-3, 0, 3))
      for i in range(300):
        counts = max(250, min(660, counts + drift + rng.randint(-4, 4)))
        extra = [(max(250, min(660, counts + rng.randint(-8, 8))),) * 4 for _ in range(rng.randint(0, 5))]
        wheel += rng.uniform(-0.8, 0.8)
        wheel = sign * max(145.0, abs(wheel))
        cb, ok, active, icon, req = self._frame(True, wheel, wheel + rng.uniform(-15, 15), wheel_counts=(counts,) * 4, extra_counts=extra)
        self.assertEqual((len(cb), ok), (1, [True]), f"seed {seed} frame {i}")
        self.assertFalse(icon == 2 and not active[0], f"seed {seed} frame {i}: green without ACTIVE")
        if active[0] and prev is not None:
          self.assertLessEqual(abs(req - prev), 1.0 + 1e-6, f"seed {seed} frame {i}")
        prev = req if active[0] else None
        n_active += active[0]
      self.assertGreater(n_active, 30, f"seed {seed}")


  def test_quantised_wheel_speed_repro(self):
    counts = (567, 567, 567, 566)
    for _ in range(100):
      self._frame(False, 160.0, 160.0, wheel_counts=counts)
    for i in range(150):
      self._frame(True, 160.0, 160.0, wheel_counts=counts)
    self.cc.apply_angle_last = 146.0
    self.safety.set_desired_angle_last(1460)
    cb, ok, active, icon, req = self._frame(True, 161.2, 161.2, wheel_counts=counts)
    self.assertEqual((len(cb), ok), (1, [True]))  # accepted: R1Guard's repro sent ACTIVE 151.2 and got rejected with a green icon
    self.assertFalse(icon == 2 and not active[0])

  def test_speed_history_out_of_step_with_the_panda_samples(self):
    """R1Guard: warm at 530 counts per wheel, then the panda gets six samples at 565 while the controller reads only the latest once: our history
    still holds the old low speeds, the panda's minimum is the new one. last 146, wheel 161.9: ACTIVE must not be sent unless accepted."""
    for _ in range(100):
      self._frame(False, 160.0, 160.0, wheel_counts=(530,) * 4)
    for _ in range(150):
      self._frame(True, 160.0, 160.0, wheel_counts=(530,) * 4)
    for _ in range(6):
      self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, {f"WHL_Spd{p}Val": 565 * 0.03125 for p in ("FL", "FR", "RL", "RR")}))
    self.t += 60000
    self.cc.apply_angle_last = 146.0
    self.safety.set_desired_angle_last(1460)
    cb, ok, active, icon, req = self._frame(True, 161.9, 161.9, wheel_counts=(565,) * 4)
    self.assertEqual((len(cb), ok), (1, [True]))
    self.assertFalse(icon == 2 and not active[0])

  def test_outside_the_guard_region_the_commanded_angle_is_the_vehicle_model_chain(self):
    """Randomized: wheel <= 140 deg or v >= 6 m/s, no takeover torque. After the resume blend the controller's request is exactly what c8b769bd
    computed: the two vehicle-model limiter passes from the last request, nothing of the guard-region step cap or backstop on top."""
    import random
    from opendbc.car.lateral import apply_steer_angle_limits_vm
    from opendbc.car.hyundai.carcontroller import ANGLE_SAFETY_BASELINE_MODEL
    for seed in range(6):
      rng = random.Random(seed)
      self.setUp()
      v = rng.choice((3.0, 8.0, 18.0, 19.3, 25.0))
      wheel_max = 140.0 if v < 6.0 else 250.0
      counts = (round(v * 3.6 / 0.03125),) * 4
      v = sum(counts) / 4 * 0.03125 / 3.6
      wheel, last = rng.uniform(-wheel_max, wheel_max), None
      for _ in range(100):
        self._frame(False, wheel, wheel, wheel_counts=counts)
      for i in range(300):
        wheel = max(-wheel_max, min(wheel_max, wheel + rng.uniform(-0.6, 0.6)))
        model = max(-300.0, min(300.0, wheel + rng.uniform(-12, 12)))
        last = self.cc.apply_angle_last
        filtered_before = self.cc.angle_filter.x
        cb, ok, active, icon, req = self._frame(True, wheel, model, wheel_counts=counts)
        self.assertEqual((len(cb), ok), (1, [True]), f"seed {seed} frame {i}")
        if not self.cc.takeover.resuming and not self.cc.takeover.paused:
          desired = self.cc.angle_filter.x
          ref = apply_steer_angle_limits_vm(desired, last, v, wheel, True, self.cc.params, self.cc.VM)
          if self.cc.CP.carFingerprint != ANGLE_SAFETY_BASELINE_MODEL:
            ref = apply_steer_angle_limits_vm(ref or desired, last, v, wheel, True, self.cc.params, self.cc.BASELINE_VM)
          if ref is not None:
            self.assertAlmostEqual(req, ref, places=4, msg=f"seed {seed} frame {i}")
          self.assertFalse(self.cc.takeover.unsendable_frames and ref is not None)

  def test_highway_speed_delivers_active_and_converges(self):
    """R1Guard round 4: wheel 0, model 10 at 18 and 19.3 m/s, takeover off and on: ACTIVE every frame, never rejected, the request converges. Outside the
    low-speed guard region no backstop runs."""
    for torque in (0, 150):
      for v in (18.0, 19.3):
        with self.subTest(takeover=torque, v=v):
          self.setUp()
          self.cc, self.cs = _carcontroller(torque, curve_hold=True)
          self.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, int(self.cc.CP.safetyConfigs[-1].safetyParam))
          self.safety.init_tests()
          self.safety.set_controls_allowed(True)
          self.safety.set_controls_allowed_lateral(True)
          counts = (round(v * 3.6 / 0.03125),) * 4
          for _ in range(100):
            self._frame(False, 0.0, 10.0, wheel_counts=counts)
          n_active, req = 0, 0.0
          for i in range(400):
            cb, ok, active, icon, req = self._frame(True, 0.0, 10.0, wheel_counts=counts)
            self.assertEqual((len(cb), ok), (1, [True]), f"frame {i}")
            n_active += active[0]
          self.assertGreaterEqual(n_active, 390)
          self.assertGreater(req, 4.9 if torque else 9.9)

  def test_backstop_window(self):
    """The backstop's window from `last`: the largest accepted steps, within its angle limit, None when nothing fits."""
    from opendbc.car.hyundai.panda_angle import PandaAngleCheck, limits_can
    check = PandaAngleCheck()
    for _ in range(6):
      check.add_speed(18.0)
    step, _ = limits_can(19.0)
    lo, hi = check.window(0.0)
    self.assertAlmostEqual(hi, (step - 1) / 10)
    self.assertAlmostEqual(lo, -(step - 1) / 10)
    self.assertEqual(check.window(0.0, 0.1), (-0.1, 0.1))  # the caller's own cap
    self.assertIsNone(check.window(400.0))  # last beyond the angle limit by more than a step: nothing fits

  def test_limit_intersects_guard_resume_step_and_backstop(self):
    """R1Guard round 5: last 189.64, wheel 199.6, model 189.64 at 4.9 m/s; the backstop's angle limit (about 188.6) would put the request 11 deg from
    the wheel (guard max 10): the intersection is empty, so inactive. A resume rate of 0.5 deg bounds the step too."""
    f = TakeoverFilter(150, 100, 500, 5)  # resume rate 0.5 deg per frame
    self.assertIsNone(f.limit(189.64, 189.64, 199.6, 4.9, backstop=(180.0, 188.6)))
    self.assertEqual(f.limit(189.64, 189.64, 199.6, 4.9, backstop=(180.0, 190.2)), 189.64)  # already inside
    self.assertEqual(f.limit(195.0, 189.64, 199.6, 4.9, backstop=(180.0, 190.2)), 190.2)  # nearest point of the intersection
    f.resuming = True
    self.assertIsNone(f.limit(195.0, 189.64, 199.6, 4.9, backstop=(180.0, 200.0)))  # resume window (last +- 0.5) and wheel +- 5 do not meet
    self.assertEqual(f.limit(195.0, 189.64, 190.0, 4.9, backstop=(180.0, 200.0)), 190.0)  # the guard's cap |request| <= |wheel| binds, inside the 0.5 window
    self.assertEqual(f.limit(180.0, 189.64, 190.0, 4.9, backstop=(180.0, 200.0)), 189.14)  # the 0.5 resume step, tighter than the 1 deg cap
    self.assertIsNone(f.limit(195.0, 189.64, 199.6, 4.9, backstop=None))

  def test_every_accepted_active_frame_satisfies_every_constraint(self):
    """Property over random inputs, filtered speed != raw speed, panda samples out of step: any ACTIVE frame the controller sends is inside the guard
    band, the cap, the resume window, the 1 deg step cap and the panda's real limits (libsafety accepts it)."""
    import random
    for seed in range(6):
      rng = random.Random(100 + seed)
      self.setUp()
      sign = rng.choice((1, -1))
      wheel = sign * rng.uniform(150.0, 240.0)
      counts = rng.randint(300, 750)
      for _ in range(100):
        self._frame(False, wheel, wheel, wheel_counts=(counts,) * 4)
      last = None
      for i in range(250):
        counts = max(250, min(760, counts + rng.randint(-5, 5)))
        extra = [(max(250, min(760, counts + rng.randint(-10, 10))),) * 4 for _ in range(rng.randint(0, 4))]
        wheel = sign * max(145.0, abs(wheel + rng.uniform(-0.9, 0.9)))
        last = self.cc.apply_angle_last
        v_filtered = rng.choice((4.9, 5.4, 5.95))
        cb, ok, active, icon, req = self._frame(True, wheel, wheel + rng.uniform(-15, 15), wheel_counts=(counts,) * 4, extra_counts=extra,
                                                 v_ego=v_filtered)  # filtered speed differs from vEgoRaw
        self.assertEqual((len(cb), ok), (1, [True]), f"seed {seed} frame {i}")
        if active[0]:
          if v_filtered < 5.0:  # the guard itself
            self.assertLessEqual(abs(req - wheel), 10.0 + 1e-3, f"seed {seed} frame {i}: guard band")
            self.assertLessEqual(abs(req), max(170.0, abs(wheel)) + 1e-3, f"seed {seed} frame {i}: guard cap")
          self.assertLessEqual(abs(req - last), 1.0 + 1e-3, f"seed {seed} frame {i}: step cap")
          if self.cc.takeover.resuming:  # R1Guard round 6 nit: the resume window too (limit() saw resuming whenever it is still set after update())
            self.assertLessEqual(abs(req - last), self.cc.takeover.resume_rate + 1e-3, f"seed {seed} frame {i}: resume rate")
            self.assertLessEqual(abs(req - wheel), CONVERGE_DEG + 1e-3, f"seed {seed} frame {i}: resume convergence")

  def test_sendable_turn_is_not_paused(self):
    """Negative counterpart: wheel 300 at 4.9 m/s has a legal ACTIVE command, lateral stays on without a pause."""
    out = self._replay(lambda i: 300.0, wheel0=300.0, model=300.0, v=4.9, frames=150)
    self.assertFalse(self.cc.takeover.paused)
    self.assertGreater(sum(a for a, _, _ in out), 20)

  # --- route 3a: a hand on the wheel in the guard region (v < 6 m/s, |wheel| > 140 deg) is a pause, never "holding the curve" ---

  def _warm(self, wheel, v, frames=150, model=None):
    model = wheel if model is None else model
    counts = (round(v * 3.6 / 0.03125),) * 4
    for _ in range(100):
      self._frame(False, wheel, model, wheel_counts=counts)
    for _ in range(frames):
      self._frame(True, wheel, model, wheel_counts=counts)
    return counts

  def test_route_3a_1_31_3_hand_along_the_turn_pauses_without_active_frames(self):
    """1:31.3: 6 mph (2.68 m/s), wheel 250 deg, curve hold on, torque 250-520 along the turn with steeringPressed. Before: ACTIVE for 13-14 frames
    (gain 0.03, request = wheel), then the EPS faulted. Now: paused within the frame, INACTIVE at the measured angle, grey icon, no ACTIVE frame."""
    from opendbc.car.hyundai import hyundaicanfd
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.setUp()
        wheel, v = sign * 250.0, 2.68
        counts = self._warm(wheel, v)
        self.assertFalse(self.cc.takeover.paused)
        for i, tq in enumerate([250.0, 300.0, 361.0, 450.0, 520.0] * 40):
          cb, ok, active, icon, req = self._frame(True, wheel, wheel, wheel_counts=counts, torque=sign * tq, pressed=True)
          self.assertEqual((len(cb), ok, active), (1, [True], [False]), f"frame {i}")
          self.assertEqual(icon, hyundaicanfd.ICON_PAUSED, f"frame {i}")
          self.assertAlmostEqual(req, wheel, places=4, msg=f"frame {i}: INACTIVE at the measured angle")
          self.assertTrue(self.cc.takeover.paused)
          self.assertEqual(self.cc.apply_torque_last, 0)

  def test_route_3a_after_the_eps_flag_no_retry_while_the_hand_holds(self):
    """The flag (steerFaultTemporary 61 frames) in the region, the hand still holds along the turn, also after the wheel unwinds out of the region:
    no ACTIVE frame for 3 s; once the hand is off for the hold time (and the fault 1 s) steering comes back."""
    from opendbc.car.hyundai import hyundaicanfd
    self.setUp()
    wheel, v = 250.0, 2.68
    counts = self._warm(wheel, v)
    for i in range(61):
      cb, ok, active, icon, _ = self._frame(False, wheel, wheel, wheel_counts=counts, torque=361.0, pressed=True, fault=True)
      self.assertEqual((ok, active), ([True], [False]))
    self.assertTrue(self.cc.takeover.strict)
    unwind = 250.0
    for i in range(300):  # hand on, wheel unwinds from 250 to 120 deg (out of the region, still a curve for F22)
      unwind = max(120.0, unwind - 0.7)
      cb, ok, active, icon, _ = self._frame(True, unwind, unwind, wheel_counts=counts, torque=300.0, pressed=True)
      self.assertEqual((ok, active, icon), ([True], [False], hyundaicanfd.ICON_PAUSED), f"frame {i}")
    self.assertTrue(self.cc.takeover.paused)
    n_active = 0
    for i in range(250):  # hand off
      cb, ok, active, icon, _ = self._frame(True, unwind, unwind, wheel_counts=counts)
      self.assertEqual(ok, [True])
      n_active += active[0]
    self.assertFalse(self.cc.takeover.paused)
    self.assertFalse(self.cc.takeover.strict)
    self.assertGreater(n_active, 100)

  def test_inactive_beyond_360_is_clipped_because_the_panda_demands_it(self):
    """Route 3a 1:28.3-1:29.7: wheel 362-409 deg, INACTIVE request fixed at 360.0 (49 deg off). The panda clamps the INACTIVE band to
    clamp(measured, 360) +- 0.1 deg, so the clip is what it accepts; the unclipped measured angle is rejected (and would reset its angle state)."""
    from opendbc.car.hyundai import hyundaicanfd
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.setUp()
        for wheel in (361.0, 380.0, 409.0):
          cb, ok, active, icon, req = self._frame(False, sign * wheel, sign * wheel, 4.9)
          self.assertEqual((len(cb), ok, active), (1, [True], [False]))
          self.assertEqual(req, sign * 360.0)
        raw = hyundaicanfd.create_steering_messages(self.cc.packer, self.cc.CP, self.cc.CAN, False, False, 0, sign * 409.0, 0, 0)
        msg = next(m for m in raw if m[0] == self.ADDR)
        self.assertFalse(self.safety.safety_tx_hook(self.libsafety_py.make_CANPacket(msg[0], msg[2], msg[1])))

  def test_outside_the_region_the_along_turn_hold_is_unchanged(self):
    """Above 6 m/s, or a wheel within 140 deg, the F22 along-turn exemption still holds: same controller, hand 300 along the turn, pressed."""
    for v, wheel in ((8.0, 250.0), (2.68, 120.0)):
      with self.subTest(v=v, wheel=wheel):
        self.setUp()
        counts = self._warm(wheel, v)
        for i in range(200):
          self._frame(True, wheel, wheel, wheel_counts=counts, torque=300.0, pressed=True)
        self.assertFalse(self.cc.takeover.paused)


class TestPandaAngleCheck(unittest.TestCase):
  """The controller's mirror of the panda acceptance (panda_angle.py) against the real libsafety."""

  def setUp(self):
    from opendbc.car.hyundai.values import HyundaiSafetyFlags
    from opendbc.safety.tests.common import CANPackerSafety
    from opendbc.safety.tests.libsafety import libsafety_py
    self.libsafety_py = libsafety_py
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, int(HyundaiSafetyFlags.CANFD_ANGLE_STEERING | HyundaiSafetyFlags.CAMERA_SCC |
                                                                                  HyundaiSafetyFlags.CANFD_LFA_ALT_STEERING | HyundaiSafetyFlags.CCNC))
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)
    self.t = 0

  def _panda(self, v, last, request):
    for _ in range(6):
      self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, {f"WHL_Spd{p}Val": v * 3.6 for p in ("FL", "FR", "RL", "RR")}))
    self.t = (self.t + 300_000) % 2**32  # more than the 250 ms real-time window apart: that check starts clean
    self.safety.set_timer(self.t)
    self.safety.set_desired_angle_last(round(last * 10))
    msg = self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": 2, "ADAS_StrAnglReqVal": request})
    return bool(self.safety.safety_tx_hook(msg))

  def test_mirror_never_accepts_what_the_panda_rejects(self):
    """Fine speed grid, half-count requests, both signs: the backstop never accepts a frame the panda rejects."""
    from opendbc.car.hyundai.panda_angle import PandaAngleCheck, limits_can
    for v in [0.5, 2.0, 4.76, 4.9197049, 6.0, 10.0] + [4.5 + 0.0137 * k for k in range(40)]:
      step, max_angle = limits_can(v)
      for last in (0.0, 146.0, -216.0, 300.0):
        check = PandaAngleCheck()
        for _ in range(6):
          check.add_speed(v)
        for d in range(-step - 4, step + 5):
          for frac in (0.0, 0.05, 0.03, 0.07):  # half counts too: the packer rounds half up, Python's round() to even
            request = last + d / 10 + frac
            if abs(request * 10) > max_angle + 4:
              continue
            with self.subTest(v=v, last=last, d=d, frac=frac):
              self.assertFalse(check.accepts(request, last) and not self._panda(v, last, request))

  def test_random_parity_mirror_accept_implies_panda_accept(self):
    """Thousands of random cases with the panda's speed samples out of step with the controller's reads (rising and falling speed): the mirror sees a
    random subset of 12 recent samples that always holds the newest, the panda's minimum is over the last six. Mirror accepts => libsafety accepts."""
    import random
    from opendbc.car.hyundai.panda_angle import PandaAngleCheck, limits_can
    rng = random.Random(1111)
    accepted = 0
    for _ in range(4000):
      c = rng.randint(100, 900)
      samples = []
      drift = rng.choice((-12, -4, 0, 4, 12))  # counts per sample: falling, flat or rising
      for _ in range(12):
        c = max(0, min(1000, c + drift + rng.randint(-2, 2)))
        samples.append([c] * 4)
      for s in samples:
        s[rng.randrange(4)] += rng.choice((0, 0, -1, 1))
      speeds = [sum(s) / 4 * 0.03125 / 3.6 for s in samples]
      check = PandaAngleCheck()
      for k in sorted(rng.sample(range(11), rng.randint(0, 6)) + [11]):
        check.add_speed(speeds[k])
      last = rng.uniform(-340.0, 340.0)
      step, _ = limits_can(min(speeds[6:]))
      request = max(-359.0, min(359.0, last + rng.choice((-1, 1)) * rng.uniform(0.0, 1.3) * step / 10))
      if check.accepts(request, last):
        accepted += 1
        for s in samples[6:]:
          self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, {f"WHL_Spd{p}Val": x * 0.03125 for p, x in zip(("FL", "FR", "RL", "RR"), s)}))
        self.t = (self.t + 300_000) % 2**32
        self.safety.set_timer(self.t)
        self.safety.set_desired_angle_last(round(last * 10))
        self.assertTrue(self.safety.safety_tx_hook(self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": 2, "ADAS_StrAnglReqVal": request})),
                        f"samples={samples[6:]} last={last} request={request}")
    self.assertGreater(accepted, 200)  # the test does exercise accepted frames

  def test_backstop_accepts_small_steps_and_rejects_big_ones(self):
    """last 146 at 4.76 m/s: a 1 deg step (the controller's cap in the guard region) passes; a 24 deg step does not, for the backstop and the panda."""
    from opendbc.car.hyundai.panda_angle import PandaAngleCheck
    check = PandaAngleCheck()
    for _ in range(6):
      check.add_speed(4.76)
    self.assertTrue(check.accepts(147.0, 146.0))
    self.assertTrue(self._panda(4.76, 146.0, 147.0))
    self.assertFalse(check.accepts(170.0, 146.0))
    self.assertFalse(self._panda(4.76, 146.0, 170.0))

  def test_one_degree_step_is_always_legal_below_six_m_s(self):
    """The proof in takeover.limit(): the panda's step limit at its worst plausible speed (ours + 1 m/s, below 7 m/s sample, 6 m/s after its -1 m/s
    fudge) is above the 1 deg cap. Checked against libsafety over the whole range, both signs."""
    from opendbc.car.hyundai.takeover import STEP_CAP_DEG, STEP_CAP_SPEED
    for v in [0.0, 1.0, 3.0, 4.9, 5.9, STEP_CAP_SPEED - 0.01, STEP_CAP_SPEED + 0.99]:
      for last in (-200.0, -146.0, 0.0, 146.0, 150.0, 199.0):  # within the panda's lateral accel limit (220 deg at its 6 m/s)
        for sign in (1, -1):
          with self.subTest(v=v, last=last, sign=sign):
            self.assertTrue(self._panda(v, last, last + sign * STEP_CAP_DEG))




if __name__ == "__main__":
  unittest.main()
