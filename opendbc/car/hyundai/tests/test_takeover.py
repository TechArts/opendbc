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
    self.assertEqual(f.limit(160.0, 176.0, 175.0, 4.0), 165.0)  # error cap
    self.assertEqual(f.limit(-215.0, -176.0, -175.0, 4.0), -175.0)
    self.assertEqual(f.limit(168.0, 160.0, 160.0, 4.0), 168.0)
    self.assertEqual(f.limit(175.0, 160.0, 160.0, 4.0), 170.0)  # wheel within the cap: the cap applies
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

  def _frame(self, lat_active, angle):
    self.t += 10000
    self.safety.set_timer(self.t)
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, {f"WHL_Spd{p}Val": self.V * 3.6 for p in ("FL", "FR", "RL", "RR")}))
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("MDPS", 0, {"MDPS_EstStrAnglVal": angle}))
    self.cs.out = structs.CarState(vEgo=self.V, vEgoRaw=self.V, steeringTorque=0.0, steeringAngleDeg=angle)
    self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    self.cs.cam_lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
    self.cs.msg_161, self.cs.msg_162, self.cs.msg_1b5 = (dbc_defaults(m) for m in ("CCNC_0x161", "CCNC_0x162", "FR_CMR_03_50ms"))
    self.cs.main_cruise_enabled = True
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0}
    self.cc.tuning = SimpleNamespace(stopping=False, actual_accel=0.0, jerk_lower=0.5, jerk_upper=0.5)
    CC = structs.CarControl(latActive=lat_active, enabled=lat_active)
    CC.actuators.steeringAngleDeg = self.MODEL
    _, sends = self.cc.update(CC.as_reader(), structs.CarControlSP(), self.cs, 0)
    cb = [s for s in sends if s[0] == self.ADDR]
    accepted = [bool(self.safety.safety_tx_hook(self.libsafety_py.make_CANPacket(s[0], s[2], s[1]))) for s in cb]
    active = [(bytes(s[1])[3] >> 4) == 2 for s in cb]
    return cb, accepted, active, self.cc.dash_states(self.cs, CC.as_reader())[0]

  def _replay(self, wheel_at):
    """Blinker pause (100 frames inactive), then lateral comes back; wheel_at(i) is the measured angle at resume frame i."""
    from opendbc.car.hyundai import hyundaicanfd
    for _ in range(100):
      cb, ok, active, _ = self._frame(False, self.WHEEL)
      self.assertEqual((len(cb), ok, active), (1, [True], [False]))
    for i in range(200):
      cb, ok, active, icon = self._frame(True, wheel_at(i))
      self.assertEqual(len(cb), 1, f"frame {i}: TX gap")
      self.assertEqual(ok, [True], f"frame {i}: the panda rejected the frame")
      if icon == hyundaicanfd.ICON_ACTIVE:
        self.assertEqual(active, [True], f"frame {i}: icon green without an ACTIVE frame")

  def test_resume_at_minus_216_never_rejected_no_gap_icon_follows_the_frames(self):
    self._replay(lambda i: self.WHEEL)

  def test_resume_with_a_slowly_unwinding_wheel(self):
    self._replay(lambda i: self.WHEEL + 0.5 * i)

  def test_resume_with_a_fast_wheel(self):
    self._replay(lambda i: self.WHEEL + 1.3 * i)


if __name__ == "__main__":
  unittest.main()
