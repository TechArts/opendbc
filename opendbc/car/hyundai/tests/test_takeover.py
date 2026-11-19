import re
import unittest
from pathlib import Path
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.hyundai.panda_angle import PandaAngleCheck
from opendbc.car.hyundai.takeover import TakeoverFilter, ENTER_FRAMES, RESUME_FRAMES, FAULT_FRAMES, GUARD_FRAMES, CONVERGE_DEG, CONVERGE_FREE_DEG
from opendbc.car.hyundai.tests.gv70 import car_params, dbc_defaults, gv70_controller
from opendbc.car.vehicle_model import VehicleModel, calc_slip_factor
from opendbc.sunnypilot.car.hyundai.values import HyundaiAngleSteeringModel, HyundaiFlagsSP, HyundaiSafetyFlagsSP

ICON_OFF, ICON_PAUSED, ICON_ACTIVE = 0, 1, 2  # CCNC_0x161 LFA_ICON: hidden / grey / green
EPS_ANGLE_MAX = 175.0  # LFA_ALT: the EPS faults on a request beyond it (lfa_alt.py LFA_ALT_ANGLE_LIMITS)
MODELS_H = Path(__file__).parents[3] / "safety" / "modes" / "hyundai_canfd_angle_models.h"


def _panda_angle_check():
  """The mirror of the panda's angle model for the GV70, as the controller builds it."""
  cp, _ = car_params()
  return PandaAngleCheck(calc_slip_factor(VehicleModel(cp)), cp.steerRatio, cp.wheelbase)


def _wheel_speeds(counts):
  """WHEEL_SPEEDS signals from raw 0.03125 kph counts (FL, FR, RL, RR)."""
  return {f"WHL_Spd{p}Val": c * 0.03125 for p, c in zip(("FL", "FR", "RL", "RR"), counts, strict=True)}


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
      self.assertLessEqual(abs(req - measured), CONVERGE_FREE_DEG + 1e-9)
      last = req
    self.assertTrue(f.resuming)  # model still more than 12 deg away from the wheel
    f.update(0.0, False, False, 15.0, 58.0, desired, True)
    self.assertFalse(f.resuming)
    self.assertEqual(f.limit(desired, 58.0, 58.0, 15.0), desired)

  def test_resume_window_is_wider_outside_the_guard_region(self):
    """Routes 60-63: a 5 deg leash held the request at wheel + 5 while the model turned in at 50-100 deg per second. Outside the guard region the
    request may lead the wheel by CONVERGE_FREE_DEG while resuming; inside (below 6 m/s past 140 deg) it stays at CONVERGE_DEG."""
    self.assertEqual((CONVERGE_DEG, CONVERGE_FREE_DEG), (5.0, 12.0))
    f = make()
    run(f, 10, torque=300)
    run(f, 50, torque=0)
    last, measured = 20.0, 20.0
    for _ in range(40):
      f.update(0.0, False, False, 10.0, measured, 80.0, True)
      last = f.limit(80.0, last, measured, 10.0)
    self.assertEqual(last - measured, CONVERGE_FREE_DEG)
    self.assertEqual(TakeoverFilter.converge_deg(20.0, 10.0), CONVERGE_FREE_DEG)
    # the taper ends 10 deg / 0.5 m/s before the region and starts 40 deg / 3 m/s before that
    self.assertEqual(TakeoverFilter.converge_deg(90.0, 5.9), CONVERGE_FREE_DEG)
    self.assertEqual(TakeoverFilter.converge_deg(141.0, 5.9), CONVERGE_DEG)
    self.assertEqual(TakeoverFilter.converge_deg(130.0, 5.9), CONVERGE_DEG)
    self.assertEqual(TakeoverFilter.converge_deg(150.0, 6.5), CONVERGE_DEG)
    self.assertEqual(TakeoverFilter.converge_deg(110.0, 5.0), CONVERGE_DEG + 7.0 * 0.5)  # halfway to the region by angle
    self.assertEqual(TakeoverFilter.converge_deg(150.0, 8.0), CONVERGE_DEG + 7.0 * 0.5)  # halfway by speed
    self.assertEqual(TakeoverFilter.converge_deg(150.0, 9.5), CONVERGE_FREE_DEG)

  def test_resume_ends_when_the_model_is_within_the_free_window(self):
    f = make()
    run(f, 10, torque=300)
    run(f, 50, torque=0)
    for _ in range(RESUME_FRAMES):
      f.update(0.0, False, False, 10.0, 30.0, 40.0, True)
    self.assertFalse(f.resuming)  # 10 deg away, inside CONVERGE_FREE_DEG
    g = make()
    run(g, 10, torque=300)
    run(g, 50, torque=0)
    for _ in range(RESUME_FRAMES):
      g.update(0.0, False, False, 10.0, 30.0, 45.0, True)
    self.assertTrue(g.resuming)  # 15 deg away

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
    self.assertEqual(f.limit(85.0, 80.0, 80.0, 4.0), 85.0)  # small angle (wheel and request within the lead band's LEAD_ANGLE)

  def test_lead_band_keeps_a_low_speed_request_within_10_deg_of_the_wheel(self):
    """Route 42 13:13.6 (3.2 m/s, hand off): the request ran 11-47 deg ahead of a lagging wheel (68 -> 144 deg) and the EPS faulted as the wheel passed
    140, before the guard's band (> 150 deg) applied. Below 6 m/s, once the wheel or the request is past 90 deg, the request stays within 10 deg of the
    wheel; one already further out comes back by at most 1 deg per frame (a step the panda accepts), never an inactive frame."""
    f = make()
    self.assertEqual(f.limit(140.0, 137.0, 128.0, 3.2), 138.0)  # 13:13.4 shape: wheel 128, request 140: wheel + 10
    self.assertEqual(f.limit(-140.0, -137.0, -128.0, 3.2), -138.0)
    self.assertEqual(f.limit(95.0, 85.0, 80.0, 3.2), 90.0)  # the request alone past 90 deg is enough
    self.assertEqual(f.limit(130.0, 125.0, 100.0, 3.2), 124.0)  # last 25 deg ahead: back by 1 deg per frame, not to the band in one step
    self.assertEqual(f.limit(115.0, 125.0, 100.0, 3.2), 115.0)  # a request the vehicle-model limits already brought further back is kept
    self.assertEqual(f.limit(191.0, 190.0, 144.0, 3.2), 189.0)  # in the step-capped region, 46 deg ahead: same, not inactive
    self.assertEqual(f.limit(60.0, 70.0, 100.0, 3.2), 71.0)  # the wheel 30 deg ahead (driver turned further): the request catches up by 1 deg per frame
    self.assertEqual(f.limit(88.0, 84.0, 80.0, 3.2), 88.0)  # both within 90 deg: unchanged
    self.assertEqual(f.limit(140.0, 137.0, 128.0, 6.0), 140.0)  # at 6 m/s and above: unchanged
    self.assertEqual(f.limit(108.0, 105.0, 100.0, 3.2), 108.0)  # within 10 deg of the wheel: unchanged

  def test_lead_band_never_empties_the_quantised_window(self):
    """R1 (fb39adef): an off-grid last request (133.04) with the panda backstop on the 0.1 deg grid ([132.0, 134.0]) and the band unreachable must
    still give an ACTIVE request at the reachable edge, not an inactive frame; both rounding directions."""
    f = make()
    self.assertEqual(f.limit(140.0, 133.04, 145.0, 3.0, (132.0, 134.0)), 134.0)
    self.assertEqual(f.limit(150.0, 157.06, 145.0, 3.0, (156.1, 158.1)), 156.1)

  def test_lead_band_keeps_a_slower_resume_rate(self):
    """R1 (fb39adef): resuming at 0.5 deg per frame, wheel 100, last 80: the band moves the request toward the wheel by the resume rate, not 1 deg."""
    f = make(rate=5)
    run(f, 1, v=3.0, angle=100.0, desired=100.0, active=False)
    run(f, 1, v=3.0, angle=100.0, desired=100.0)
    self.assertTrue(f.resuming)
    self.assertEqual(f.limit(100.0, 80.0, 100.0, 3.0), 80.5)
    self.assertEqual(f.limit(60.0, 80.0, 100.0, 3.0), 80.5)  # a request the other way still only steps 0.5 toward the wheel

  def test_a_pause_inside_the_guard_region_holds_until_the_wheel_or_speed_leaves_it(self):
    """Route 45 7:46.7 (0.94 m/s, wheel -341 deg, hand 70-187, not pressed): after a takeover pause openpilot resumed ACTIVE (gain 0.03, request 3 deg
    from the wheel) and the EPS faulted. Inside the region (v < 6 m/s, |wheel| > 140 deg) a pause never releases, however long the hand is off; it
    releases after the hold time once the wheel is back under 140 deg or the speed is 6 m/s or more."""
    hold = 50
    for sign in (1, -1):
      with self.subTest(sign=sign):
        a = -341.0 * sign
        f = make()
        self.assertTrue(run(f, ENTER_FRAMES, torque=300.0, v=0.94, angle=a, desired=a)[-1])
        self.assertTrue(all(run(f, 600, torque=70.0, v=0.94, angle=a, desired=a)))  # calm, but still in the region
        self.assertTrue(all(run(f, 600, torque=0.0, v=5.9, angle=a, desired=a)))  # hand off, still below 6 m/s
        self.assertFalse(run(f, hold + 5, torque=0.0, v=0.94, angle=-130.0 * sign, desired=-130.0 * sign)[-1])  # unwound below 140: releases
        f = make()
        run(f, ENTER_FRAMES, torque=300.0, v=0.94, angle=a, desired=a)
        self.assertFalse(run(f, hold + 5, torque=0.0, v=6.0, angle=a, desired=a)[-1])  # 6 m/s: outside the region, releases

  def test_lateral_coming_back_inside_the_guard_region_starts_paused(self):
    """Engage, blinker-pause end or MADS resume inside the region: no ACTIVE start there (paused, so the controller sends inactive at the wheel);
    outside the region it starts with the usual resume blend."""
    for sign in (1, -1):
      with self.subTest(sign=sign):
        a = 216.0 * sign
        f = make()
        run(f, 1, v=4.76, angle=a, desired=a, active=False)
        self.assertTrue(all(run(f, 300, v=4.76, angle=a, desired=a)))
        self.assertFalse(run(f, 60, v=4.76, angle=120.0 * sign, desired=120.0 * sign)[-1])
        f = make()
        run(f, 1, v=4.76, angle=120.0 * sign, desired=120.0 * sign, active=False)
        self.assertFalse(run(f, 1, v=4.76, angle=120.0 * sign, desired=120.0 * sign)[0])
        self.assertTrue(f.resuming)

  def test_guard_cap_never_exceeds_the_eps_limit(self):
    """The EPS faults on a request past 175 deg (LFA_ALT): the guard cap is the lower of the wheel and the EPS limit, so a wheel more than the guard band
    past the limit leaves no request in the intersection (inactive) instead of one 10+ deg behind the wheel."""
    f = TakeoverFilter(150, 100, 500, 10, max_angle=EPS_ANGLE_MAX)
    for sign in (1, -1):
      self.assertEqual(f.limit(sign * 215.0, sign * 172.0, sign * 172.0, 4.0), sign * 172.0)  # the guard cap is the wheel (past 170 deg), below the limit
      self.assertEqual(f.limit(sign * 215.0, sign * 175.0, sign * 180.0, 4.0), sign * 175.0)  # band [170, 190] and the limit: 175 (was up to 176 at the wheel)
      self.assertEqual(f.limit(sign * 190.0, sign * 175.0, sign * 185.0, 4.0), sign * 175.0)  # the cap is reached at wheel + 10
      self.assertIsNone(f.limit(sign * 175.0, sign * 175.0, sign * 190.0, 4.0))  # band [180, 200] and the cap meet nowhere
      self.assertIsNone(f.limit(sign * 215.0, sign * 176.0, sign * 216.0, 4.0))
    self.assertEqual(TakeoverFilter(150, 100, 500, 10).limit(215.0, 185.5, 186.0, 4.0), 186.0)  # no EPS limit given: the guard alone
    self.assertIsNone(f.limit(215.0, 185.5, 186.0, 4.0))  # the same wheel with the limit: nothing to send

  def test_cap_binding_pauses_after_half_second_and_stays_until_released(self):
    f = make()
    run(f, 1, v=4.0, angle=130.0, desired=130.0)  # lateral on outside the region, then the wheel turns in while ACTIVE
    paused = run(f, GUARD_FRAMES + 5, v=4.0, angle=175.0, desired=205.0)
    self.assertFalse(paused[GUARD_FRAMES - 1])
    self.assertTrue(paused[-1])
    self.assertTrue(all(run(f, 200, v=4.0, angle=175.0, desired=205.0)))  # still binding: stay paused, no flapping
    self.assertTrue(all(run(f, 60, v=4.0, angle=175.0, desired=168.0)))  # no longer binding, but no ACTIVE re-entry inside the region
    self.assertFalse(run(f, 60, v=4.0, angle=130.0, desired=130.0)[-1])  # unwound below 140 deg: released

  def test_no_pause_when_cap_does_not_bind(self):
    f = make()
    run(f, 1, v=4.0, angle=130.0, desired=130.0)
    self.assertFalse(any(run(f, 300, v=4.0, angle=175.0, desired=168.0)))

  def test_short_binding_does_not_pause(self):
    f = make()
    run(f, 1, v=4.0, angle=130.0, desired=130.0)
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
    cs.lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
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

  def test_guard_region_hand_along_the_turn_pauses_like_straight_and_holds_in_the_region(self):
    """Route 3a: v 2.68 m/s, wheel and model 250: a = 0.7 m/s^2, a turn. In the region any torque or steeringPressed pauses, along the turn too;
    the pause then holds while the wheel is in the region (route 45) and releases like straight once it is back under 140 deg."""
    for sign in (1, -1):
      a = 250.0 * sign
      kw = dict(v=2.68, angle=a, desired=a)
      out = dict(v=2.68, angle=120.0 * sign, desired=120.0 * sign)
      f = make_curve()
      run(f, 1, **out)  # lateral on outside the region, then the wheel turns in while ACTIVE
      self.assertTrue(run(f, 1, torque=0.0, pressed=True, **kw)[0])
      f = make_curve()
      run(f, 1, **out)
      self.assertFalse(run(f, ENTER_FRAMES - 1, torque=250 * sign, **kw)[-1])
      self.assertTrue(run(f, 1, torque=250 * sign, **kw)[-1])  # along-turn torque for ENTER_FRAMES, unpressed: a grab in the region
      hold = f.hold_frames
      self.assertTrue(all(run(f, hold + 50, torque=300 * sign, pressed=True, **kw)))
      self.assertTrue(all(run(f, hold + 50, torque=0.0, **kw)))  # hand off, still in the region: held
      self.assertTrue(all(run(f, 5, torque=0.0, **out)))  # back under 140; the 130 deg jump reads as a fast wheel for the 5-frame rate window
      self.assertTrue(all(run(f, hold - 1, torque=0.0, **out)))  # then the straight-road hold time
      self.assertFalse(run(f, 1, torque=0.0, **out)[-1])
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
    self._start_safety()
    self.t = 0

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def _start_safety(self):
    """The panda in the controller's own configuration (SP flags with the angle model id, and the car's param), lateral allowed."""
    self.safety.set_current_safety_param_sp(self.cc.CP_SP.safetyParam)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, int(self.cc.CP.safetyConfigs[-1].safetyParam))
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)

  def _frame(self, lat_active, angle, model=None, v=None, wheel_counts=None, extra_counts=(), v_ego=None, torque=0.0, pressed=False, fault=False):
    """One controller cycle into the real panda. Returns (0xCB frames, accepted, active, icon, request angle). extra_counts: wheel-speed RX frames
    the panda gets before this cycle's one, which the controller never reads."""
    v = self.V if v is None else v
    self.t += 10000
    self.safety.set_timer(self.t)
    wheels = {f"WHL_Spd{p}Val": v * 3.6 for p in ("FL", "FR", "RL", "RR")}
    if wheel_counts is not None:  # raw 0.03125 kph counts per wheel as on the bus; vEgoRaw is their mean, as the car state computes it
      wheels = _wheel_speeds(wheel_counts)
      v = sum(wheel_counts) / 4 * 0.03125 / 3.6
    for extra in extra_counts:
      self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, _wheel_speeds(extra)))
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, wheels))
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("MDPS", 0, {"MDPS_EstStrAnglVal": angle}))
    self.cs.out = structs.CarState(vEgo=v if v_ego is None else v_ego, vEgoRaw=v, steeringTorque=torque, steeringPressed=pressed,
                                 steerFaultTemporary=fault, steeringAngleDeg=angle)
    self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    self.cs.lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
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
    lat_active_icon, lat_paused_icon, _ = self.cc.dash_states(self.cs, CC.as_reader())
    icon = ICON_ACTIVE if lat_active_icon else ICON_PAUSED if lat_paused_icon else ICON_OFF
    return cb, accepted, active, icon, act.steeringAngleDeg

  def _replay(self, wheel_at, wheel0=None, model=None, v=None, frames=200):
    """Blinker pause (100 frames inactive), then lateral comes back; wheel_at(i) is the measured angle at resume frame i.
    Every frame: one 0xCB, accepted, green icon only with an ACTIVE frame. Returns [(active, request angle, icon)] of the resume frames."""
    wheel0 = self.WHEEL if wheel0 is None else wheel0
    for _ in range(100):
      cb, ok, active, _, _ = self._frame(False, wheel0, model, v)
      self.assertEqual((len(cb), ok, active), (1, [True], [False]))
    out = []
    for i in range(frames):
      cb, ok, active, icon, req = self._frame(True, wheel_at(i), model, v)
      self.assertEqual(len(cb), 1, f"frame {i}: TX gap")
      self.assertEqual(ok, [True], f"frame {i}: the panda rejected the frame")
      if icon == ICON_ACTIVE:
        self.assertEqual(active, [True], f"frame {i}: icon green without an ACTIVE frame")
      out.append((active[0], req, icon))
    return out

  def _assert_resume_steps(self, out):
    """Between consecutive ACTIVE frames of the resume (RESUME_FRAMES) the request never moves more than the resume rate (1 deg/frame)."""
    for i in range(1, RESUME_FRAMES):
      if out[i - 1][0] and out[i][0]:
        self.assertLessEqual(abs(out[i][1] - out[i - 1][1]), 1.0 + 1e-6, f"frame {i}: resume step")

  def _assert_active_steps(self, out):
    """Between any two consecutive ACTIVE frames the request never moves more than 1 deg."""
    for i in range(1, len(out)):
      if out[i - 1][0] and out[i][0]:
        self.assertLessEqual(abs(out[i][1] - out[i - 1][1]), 1.0 + 1e-6, f"frame {i}: ACTIVE step")

  def test_resume_at_minus_216_stays_inactive_until_the_wheel_leaves_the_region(self):
    """Route 39 29:19.6: the blinker pause ends at 4.76 m/s with the wheel at -216 deg. Lateral does not start inside the region (route 45): every
    frame inactive, accepted, grey; once the driver unwinds below 140 deg steering comes back, ACTIVE steps <= 1 deg."""
    def wheel_at(i):
      return self.WHEEL if i < 100 else min(-130.0, self.WHEEL + 1.0 * (i - 100))
    out = self._replay(wheel_at, frames=300)
    for i, (active, _, _) in enumerate(out):
      if abs(wheel_at(i)) > 140.0:
        self.assertFalse(active, f"frame {i}: ACTIVE inside the region")
    self.assertGreater(sum(a for a, _, _ in out), 20)  # ACTIVE is actually delivered once out of the region
    self._assert_active_steps(out)

  def _resume_into_the_region(self, sign, wheel_at, v_at, model, wheel0, v0, frames=300, may_pause=False):
    """Blinker pause, then a resume outside the region (wheel0, v0) that runs into it: the wheel and the speed per frame. Every frame: one 0xCB, accepted.
    ACTIVE in the region while resuming is always within CONVERGE_DEG of the wheel. Once ACTIVE: no inactive frame and no gain reset, unless may_pause:
    then an inactive frame must be the latched pause (inactive for the whole hold, no ACTIVE start in the region). Returns [(active, wheel, v, gain)]."""
    for _ in range(100):
      cb, ok, active, _, _ = self._frame(False, wheel0, model, v0)
      self.assertEqual((len(cb), ok, active), (1, [True], [False]))
    out, started, inactive_run, paused_once = [], False, 0, False
    for i in range(frames):
      w, v = wheel_at(i), v_at(i)
      cb, ok, active, _, req = self._frame(True, w, model, v)
      self.assertEqual((len(cb), ok), (1, [True]), f"frame {i}: TX gap or rejected frame")
      gain = self.cc.apply_torque_last
      in_region = abs(w) > 140.0 and v < 6.0
      if active[0] and in_region and self.cc.takeover.resuming:
        self.assertLessEqual(abs(req - w), CONVERGE_DEG + 1e-3, f"sign {sign} frame {i}: request {req:.2f}, {abs(req - w):.2f} deg from the wheel")
      if started and not active[0]:
        self.assertTrue(may_pause, f"sign {sign} frame {i} wheel {w:.1f} v {v:.2f}: inactive after ACTIVE (gain was reset)")
        paused_once = True
      if active[0]:
        if started and paused_once:
          self.assertGreaterEqual(inactive_run, GUARD_FRAMES, f"frame {i}: ACTIVE again after {inactive_run} inactive frames")
        if not started:
          self.assertFalse(in_region, f"frame {i}: ACTIVE started inside the region")
        started = True
        inactive_run = 0
        self.assertGreater(gain, 0.0, f"frame {i}: gain reset")
      else:
        inactive_run += 1
      out.append((active[0], w, v, gain))
    self.assertTrue(started)
    return out

  def test_resume_crossing_into_the_region_on_the_wheel_angle_keeps_steering(self):
    """3.2 m/s, resuming at 100 deg with the model far ahead, the wheel unwinds toward 175: it crosses 140 deg ACTIVE, both signs, slow and fast wheel."""
    for sign in (1, -1):
      for rate in (0.2, 0.5, 0.9):
        self.setUp()
        out = self._resume_into_the_region(sign, lambda i, sign=sign, rate=rate: sign * min(100.0 + rate * i, 175.0), lambda i: 3.2, sign * 175.0,
                                          sign * 100.0, 3.2)
        self.assertTrue(any(abs(w) > 140.0 for _, w, _, _ in out) and out[-1][0])

  def test_resume_crossing_into_the_region_on_the_speed_keeps_steering(self):
    """Wheel 150 deg, resuming at 7 m/s (outside the region, inside the lateral acceleration cap), the car slows through 6 m/s:
    both signs, slow and fast braking."""
    for sign in (1, -1):
      for decel in (0.005, 0.02, 0.05):
        self.setUp()
        out = self._resume_into_the_region(sign, lambda i, sign=sign: sign * (150.0 + 0.02 * i), lambda i, decel=decel: max(7.0 - decel * i, 4.0), sign * 160.0,
                                          sign * 150.0, 7.0, frames=600)
        self.assertTrue(any(v < 6.0 for _, _, v, _ in out) and out[-1][0])

  def test_resume_braking_and_unwinding_into_the_region(self):
    """R1 re-review: wheel 174 - 0.9 deg per frame, speed 7 - 0.05 m/s per frame, model 175. The band edge used to retreat faster than the 1 deg resume
    rate. Inside the region the bound is a hard +-5 deg around the wheel: where the rate-limited band (it cannot narrow faster than the request follows
    the wheel) is still wider at entry the resume takes the latched pause instead (hold, no ACTIVE start in the region). Realistic braking keeps steering."""
    for sign in (1, -1):
      for decel in (0.01, 0.02):  # 1 and 2 m/s^2
        self.setUp()
        out = self._resume_into_the_region(sign, lambda i, sign=sign: sign * max(174.0 - 0.9 * i, 100.0), lambda i, decel=decel: max(7.0 - decel * i, 3.0),
                                           sign * 175.0, sign * 174.0, 7.0, frames=150)
        self.assertTrue(all(a for a, _, _, _ in out))
      for decel in (0.05, 0.06):  # 5 and 6 m/s^2: the reviewer's counterexample and beyond
        self.setUp()
        self._resume_into_the_region(sign, lambda i, sign=sign: sign * max(174.0 - 0.9 * i, 100.0), lambda i, decel=decel: max(7.0 - decel * i, 3.0),
                                     sign * 175.0, sign * 174.0, 7.0, frames=250, may_pause=True)

  def test_vehicle_model_limit_while_resuming_near_the_region_latches_the_pause(self):
    """150 deg at 8 m/s is above the lateral acceleration cap (the vehicle model leaves no request, limit() is not reached): a resume there takes the
    same latched pause, inactive for the whole hold."""
    for sign in (1, -1):
      self.setUp()
      for _ in range(100):
        self._frame(False, sign * 150.0, sign * 160.0, 8.0)
      runs = []
      for _i in range(60):
        cb, ok, active, _, _ = self._frame(True, sign * 150.0, sign * 160.0, 8.0)
        self.assertEqual((len(cb), ok), (1, [True]))
        runs.append(active[0])
      self.assertTrue(self.cc.takeover.paused)
      self.assertFalse(any(runs[:GUARD_FRAMES]), "ACTIVE before the hold ended")

  def test_the_region_bound_is_a_hard_5_deg_whatever_the_band(self):
    """R1: inside the guard region (under 6 m/s, past 140 deg) a resume request stays within CONVERGE_DEG of the wheel even while the rate-limited band
    outside it is still wider; a last request further out cannot be inside it: no ACTIVE request, the pause is latched."""
    f = TakeoverFilter(150, 100, 500, 10)
    f.resuming, f.band = True, 8.0
    self.assertEqual(f.limit(160.0, 154.5, 150.0, 5.0), 155.0)  # wheel + 5, not + 8
    f.resuming, f.band = True, 8.0
    self.assertIsNone(f.limit(160.0, 157.5, 150.0, 5.0))  # last is 7.5 from the wheel: nothing within 1 deg of it is within 5 deg of the wheel
    self.assertTrue(f.paused and not f.resuming)

  def test_forced_inactive_while_resuming_near_the_region_latches_the_pause(self):
    """An empty window while resuming near the region is a takeover-like pause: inactive for at least the hold, no ACTIVE inside the region."""
    f = TakeoverFilter(150, 100, 500, 10)
    f.resuming = True
    self.assertIsNone(f.limit(160.0, 190.0, 150.0, 5.5, backstop=(0.0, 175.0)))  # last 190 is outside the window of any request
    self.assertTrue(f.paused and not f.resuming)
    g = TakeoverFilter(150, 100, 500, 10)
    g.resuming = True
    self.assertIsNone(g.limit(160.0, 190.0, 50.0, 12.0, backstop=(0.0, 175.0)))  # far from the region: unchanged, no latch
    self.assertFalse(g.paused)

  def test_random_trajectories_near_the_region_never_reenter_ungated(self):
    """Random wheel (at most 0.95 deg per frame) and speed (up to 0.06 m/s per frame) near the boundary, resuming, through libsafety: every cycle one
    accepted 0xCB; an inactive cycle after ACTIVE near the region (past 135 deg, under 6.5 m/s) is followed by at least the hold (500 ms) of inactive, and no
    ACTIVE start inside the region."""
    import random
    for seed in range(40):
      rng = random.Random(500 + seed)
      self.setUp()
      sign = rng.choice((1, -1))
      wheel, v = rng.uniform(140.0, 178.0), rng.uniform(5.5, 7.0)
      drift_w, drift_v = rng.uniform(-0.9, 0.3), rng.uniform(-0.06, 0.01)  # a trajectory that tends toward the region, plus noise
      for _ in range(100):
        self._frame(False, sign * wheel, sign * wheel, v)
      prev_active, inactive_run, started = False, 10 ** 6, False
      for i in range(300):
        wheel = min(max(wheel + min(max(drift_w + rng.uniform(-0.05, 0.05), -0.95), 0.95), 100.0), 178.0)
        v = min(max(v + drift_v + rng.uniform(-0.005, 0.005), 3.0), 9.0)
        model = wheel + rng.uniform(-8.0, 8.0)
        cb, ok, active, _, req = self._frame(True, sign * wheel, sign * min(model, 175.0), v)
        self.assertEqual((len(cb), ok), (1, [True]), f"seed {seed} frame {i}")
        if active[0] and wheel > 140.0 and v < 6.0 and self.cc.takeover.resuming:
          self.assertLessEqual(abs(req - sign * wheel), CONVERGE_DEG + 1e-3, f"seed {seed} frame {i}: request {abs(req - sign * wheel):.2f} deg from the wheel")
        if active[0] and not prev_active:
          self.assertFalse(wheel > 140.0 and v < 6.0, f"seed {seed} frame {i}: ACTIVE started inside the region")
          if started and wheel > 135.0 and v < 6.5:  # near the region (the lateral acceleration cap does not bind below 7 m/s)
            self.assertGreaterEqual(inactive_run, GUARD_FRAMES, f"seed {seed} frame {i}: ACTIVE again after {inactive_run} inactive frames")
        started = started or active[0]
        inactive_run = 0 if active[0] else inactive_run + 1
        prev_active = active[0]

  def test_resume_with_a_slowly_unwinding_wheel(self):
    out = self._replay(lambda i: self.WHEEL + 0.5 * i)
    self.assertGreater(sum(a for a, _, _ in out), 20)
    self._assert_resume_steps(out)

  def test_resume_with_a_fast_wheel(self):
    out = self._replay(lambda i: self.WHEEL + 1.3 * i)
    self._assert_resume_steps(out)  # 1.3 deg/frame trajectory: steps stay <= 1 deg between ACTIVE frames (inactive where none exists)

  def test_guard_boundaries_both_signs(self):
    """150 / 170 deg wheel and 5 m/s, positive and negative: resume slowly unwinding, model 1.1x the wheel; all frames accepted, no ACTIVE while the
    wheel is in the region (below 6 m/s), steps <= 1 deg."""
    for sign in (1, -1):
      for wheel in (149.0, 151.0, 169.0, 171.0, 216.0):
        for v in (4.9, 5.1):
          with self.subTest(wheel=sign * wheel, v=v):
            self.setUp()
            out = self._replay(lambda i, sign=sign, wheel=wheel: sign * (wheel - 0.3 * i), wheel0=sign * wheel, model=sign * wheel * 1.1, v=v, frames=400)
            for i, (active, _, _) in enumerate(out):
              if wheel - 0.3 * i > 140.0:
                self.assertFalse(active, f"frame {i}: ACTIVE inside the region")
            self.assertGreater(sum(a for a, _, _ in out), 20)
            self._assert_active_steps(out)

  def test_unsendable_wheel_pauses_with_a_grey_icon(self):
    """Steering ACTIVE at 170 deg (4.9 m/s), then the wheel is at +-250 with the model at 245: past the EPS limit plus the guard band, so the model
    limits leave no legal ACTIVE command. Inactive frames only (one per cycle, accepted), then an explicit pause after the guard time, grey the whole time."""
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.setUp()
        self._warm(sign * 170.0, 4.9)
        self.assertFalse(self.cc.takeover.paused)
        paused_at = None
        for i in range(120):
          cb, ok, active, icon, _ = self._frame(True, sign * 250.0, sign * 245.0, 4.9)
          self.assertEqual((len(cb), ok, active), (1, [True], [False]))
          self.assertEqual(icon, ICON_PAUSED)
          if self.cc.takeover.paused and paused_at is None:
            paused_at = i
        self.assertIsNotNone(paused_at)
        self.assertGreaterEqual(paused_at, GUARD_FRAMES)
        self.assertLess(paused_at, GUARD_FRAMES + 5)

  def test_guard_step_cap_never_exceeds_one_degree_and_tracking_delivers_active(self):
    """last 146, wheel 160 (a contrived gap): the capped step cannot reach the guard's band, so inactive (accepted) and the next frame starts at the wheel;
    then tracking a model that moves 0.5 deg per frame delivers ACTIVE every frame, steps <= 1 deg."""
    self._warm(160.0, self.V)
    self.assertFalse(self.cc.takeover.resuming)
    self.cc.apply_angle_last = 146.0
    self.safety.set_desired_angle_last(1460)
    cb, ok, active, icon, req = self._frame(True, 160.0, 160.0)
    self.assertEqual((ok, active), ([True], [False]))
    prev = None
    for i in range(100):
      cb, ok, active, icon, req = self._frame(True, 160.0 + 0.15 * i, 160.0 + 0.15 * i + 3.0)
      self.assertEqual((len(cb), ok, active), (1, [True], [True]), f"frame {i}")
      if prev is not None:
        self.assertLessEqual(abs(req - prev), 1.0 + 1e-6)
      prev = req

  def test_random_rx_out_of_step_with_controller_updates(self):
    """Random wheel / model / speed walks (rising and falling speed below 6 m/s) with 0-5 extra RX wheel-speed frames the controller never reads
    between updates: no rejected frame, no gap, ACTIVE steps <= 1 deg, green only with ACTIVE, ACTIVE is delivered. Lateral comes on at 130 deg and
    the wheel turns into the region while ACTIVE (lateral never starts inside it)."""
    import random
    for seed in range(8):
      rng = random.Random(seed)
      self.setUp()
      sign = rng.choice((1, -1))
      wheel = sign * rng.uniform(145.0, 175.0)
      counts = rng.randint(300, 640)
      w = sign * 130.0
      for _ in range(100):
        self._frame(False, w, w, wheel_counts=(counts,) * 4)
      for _ in range(150):
        self._frame(True, w, w, wheel_counts=(counts,) * 4)
      while w != wheel:
        w += max(-0.5, min(0.5, wheel - w))
        self._frame(True, w, w, wheel_counts=(counts,) * 4)
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

  def _enter(self, wheel, counts, v_ego=None, model=None, frames=150):
    """Lateral on and settled at (wheel, model) at the given raw wheel counts. Lateral never starts inside the guard region, so a target there is
    entered while ACTIVE: lateral comes on at 130 deg, then the wheel (and the model, same offset) turns in at 0.5 deg per frame, then 50 frames."""
    model = wheel if model is None else model
    v = sum(counts) / len(counts) * 0.03125 / 3.6
    inside = self.cc.takeover.step_capped(wheel, v)
    w = (130.0 if wheel > 0 else -130.0) if inside else wheel
    for _ in range(100):
      self._frame(False, w, w + model - wheel, wheel_counts=counts, v_ego=v_ego)
    for _ in range(frames):
      self._frame(True, w, w + model - wheel, wheel_counts=counts, v_ego=v_ego)
    if inside:
      while w != wheel:
        w += max(-0.5, min(0.5, wheel - w))
        self._frame(True, w, w + model - wheel, wheel_counts=counts, v_ego=v_ego)
      for _ in range(50):
        self._frame(True, wheel, model, wheel_counts=counts, v_ego=v_ego)
    self.assertFalse(self.cc.takeover.paused)  # the ACTIVE path decides the next frames, not a pause

  def test_quantised_wheel_speed_repro(self):
    counts = (567, 567, 567, 566)
    self._enter(160.0, counts)
    self.cc.apply_angle_last = 146.0
    self.safety.set_desired_angle_last(1460)
    cb, ok, active, icon, req = self._frame(True, 161.2, 161.2, wheel_counts=counts)
    self.assertEqual((len(cb), ok), (1, [True]))  # accepted: R1Guard's repro sent ACTIVE 151.2 and got rejected with a green icon
    self.assertFalse(icon == 2 and not active[0])

  def test_speed_history_out_of_step_with_the_panda_samples(self):
    """R1Guard: warm at 530 counts per wheel, then the panda gets six samples at 565 while the controller reads only the latest once: our history
    still holds the old low speeds, the panda's minimum is the new one. last 146, wheel 161.9: ACTIVE must not be sent unless accepted."""
    self._enter(160.0, (530,) * 4)
    for _ in range(6):
      self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, {f"WHL_Spd{p}Val": 565 * 0.03125 for p in ("FL", "FR", "RL", "RR")}))
    self.t += 60000
    self.cc.apply_angle_last = 146.0
    self.safety.set_desired_angle_last(1460)
    cb, ok, active, icon, req = self._frame(True, 161.9, 161.9, wheel_counts=(565,) * 4)
    self.assertEqual((len(cb), ok), (1, [True]))
    self.assertFalse(icon == 2 and not active[0])

  def test_outside_the_guard_region_the_commanded_angle_is_the_vehicle_model_chain(self):
    """Randomized: wheel and model within LEAD_ANGLE (90 deg) below 6 m/s, or v >= 6 m/s, no takeover torque. After the resume blend the controller's
    request is exactly what the vehicle model limiter computes from the last request (the panda checks the same model, hyundai_canfd_angle_models.h):
    nothing of the guard-region step cap, lead band or backstop on top."""
    import random
    from opendbc.car.lateral import apply_steer_angle_limits_vm
    from opendbc.car.hyundai.carcontroller import ANGLE_SAFETY_BASELINE_MODEL  # the platform's own panda model: no baseline re-limit
    self.assertNotEqual(self.cc.CP.carFingerprint, ANGLE_SAFETY_BASELINE_MODEL)
    for seed in range(6):
      rng = random.Random(seed)
      self.setUp()
      v = rng.choice((3.0, 8.0, 18.0, 19.3, 25.0))
      wheel_max = 78.0 if v < 6.0 else 250.0  # below 6 m/s: model = wheel +- 12 stays within 90 deg
      counts = (round(v * 3.6 / 0.03125),) * 4
      v = sum(counts) / 4 * 0.03125 / 3.6
      wheel, last = rng.uniform(-wheel_max, wheel_max), None
      for _ in range(100):
        self._frame(False, wheel, wheel, wheel_counts=counts)
      for i in range(300):
        wheel = max(-wheel_max, min(wheel_max, wheel + rng.uniform(-0.6, 0.6)))
        model = max(-300.0, min(300.0, wheel + rng.uniform(-12, 12)))
        last = self.cc.apply_angle_last
        cb, ok, active, icon, req = self._frame(True, wheel, model, wheel_counts=counts)
        self.assertEqual((len(cb), ok), (1, [True]), f"seed {seed} frame {i}")
        if not self.cc.takeover.resuming and not self.cc.takeover.paused:
          desired = self.cc.angle_filter.x
          ref = apply_steer_angle_limits_vm(desired, last, v, wheel, True, self.cc.params, self.cc.VM)
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
          self._start_safety()
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
    check = _panda_angle_check()
    for _ in range(6):
      check.add_speed(18.0)
    step, _ = check.limits_can(19.0)
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
    self.assertTrue(f.paused and not f.resuming)  # an empty window while resuming in the region latches the pause
    f.paused, f.resuming = False, True
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
      wheel = sign * rng.uniform(150.0, 165.0)
      counts = rng.randint(300, 750)
      self._enter(wheel, (counts,) * 4)
      last, n_active = None, 0
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
            self.assertLessEqual(abs(req), min(max(170.0, abs(wheel)), EPS_ANGLE_MAX) + 1e-3, f"seed {seed} frame {i}: guard cap")
          self.assertLessEqual(abs(req - last), 1.0 + 1e-3, f"seed {seed} frame {i}: step cap")
          if self.cc.takeover.resuming:  # R1Guard round 6 nit: the resume window too (limit() saw resuming whenever it is still set after update())
            self.assertLessEqual(abs(req - last), self.cc.takeover.resume_rate + 1e-3, f"seed {seed} frame {i}: resume rate")
            self.assertLessEqual(abs(req - wheel), CONVERGE_DEG + 1e-3, f"seed {seed} frame {i}: resume convergence")
          n_active += 1
      self.assertGreater(n_active, 20, f"seed {seed}: the ACTIVE constraints were actually checked")

  def test_sendable_turn_is_not_paused(self):
    """Negative counterpart: steering ACTIVE into a wheel of 175 deg at 4.9 m/s has a legal ACTIVE command, lateral stays on without a pause."""
    counts = self._warm(175.0, 4.9)
    n_active = sum(self._frame(True, 175.0, 175.0, wheel_counts=counts)[2][0] for _ in range(150))
    self.assertFalse(self.cc.takeover.paused)
    self.assertGreater(n_active, 20)

  # --- route 3a: a hand on the wheel in the guard region (v < 6 m/s, |wheel| > 140 deg) is a pause, never "holding the curve" ---

  def _warm(self, wheel, v, frames=150, model=None):
    """_enter at the wheel counts for speed v; returns the counts."""
    counts = (round(v * 3.6 / 0.03125),) * 4
    self._enter(wheel, counts, model=model, frames=frames)
    return counts

  def test_route_3a_1_31_3_hand_along_the_turn_pauses_without_active_frames(self):
    """1:31.3: 6 mph (2.68 m/s), wheel 250 deg (170 here: the EPS takes no request past 175 deg), curve hold on, torque 250-520 along the turn with
    steeringPressed. Before: ACTIVE for 13-14 frames (gain 0.03, request = wheel), then the EPS faulted. Now: paused within the frame, INACTIVE at the
    measured angle, grey icon, no ACTIVE frame."""
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.setUp()
        wheel, v = sign * 170.0, 2.68
        counts = self._warm(wheel, v)
        self.assertFalse(self.cc.takeover.paused)
        for i, tq in enumerate([250.0, 300.0, 361.0, 450.0, 520.0] * 40):
          cb, ok, active, icon, req = self._frame(True, wheel, wheel, wheel_counts=counts, torque=sign * tq, pressed=True)
          self.assertEqual((len(cb), ok, active), (1, [True], [False]), f"frame {i}")
          self.assertEqual(icon, ICON_PAUSED, f"frame {i}")
          self.assertAlmostEqual(req, wheel, places=4, msg=f"frame {i}: INACTIVE at the measured angle")
          self.assertTrue(self.cc.takeover.paused)
          self.assertEqual(self.cc.apply_torque_last, 0)

  def test_route_45_7_46_7_no_active_re_entry_at_minus_341(self):
    """Route 45 7:46.7: 0.94 m/s (2.1 mph), wheel -341 deg (-170 here: the EPS takes no request past 175 deg), the driver's hand 70-187 below the
    takeover torque, not pressed. Before: after the pause the controller resumed ACTIVE for 14 frames (gain 0.03, request 3 deg from the wheel) and the
    EPS faulted. Now, closed loop with the real panda: after the takeover no ACTIVE frame while the wheel is in the region (every frame accepted,
    grey), and steering comes back once it is unwound below 140 deg, ACTIVE steps <= 1 deg."""
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.setUp()
        wheel, v = sign * -170.0, 0.94
        counts = self._warm(wheel, v)
        self.assertFalse(self.cc.takeover.paused)
        for _ in range(10):  # the driver takes the wheel
          self._frame(True, wheel, wheel, wheel_counts=counts, torque=sign * -300.0, pressed=True)
        self.assertTrue(self.cc.takeover.paused)
        for i in range(300):  # light hand, then hand off, still at -341
          cb, ok, active, icon, _ = self._frame(True, wheel, wheel, wheel_counts=counts, torque=sign * -70.0 if i < 150 else 0.0)
          self.assertEqual((len(cb), ok, active, icon), (1, [True], [False], ICON_PAUSED), f"frame {i}")
        out, w = [], wheel
        for _ in range(400):  # the driver unwinds at 1 deg per frame to 120 deg
          w = (1.0 if w > 0 else -1.0) * max(120.0, abs(w) - 1.0)
          cb, ok, active, icon, req = self._frame(True, w, w, wheel_counts=counts)
          self.assertEqual((len(cb), ok), (1, [True]))
          if abs(w) > 140.0:
            self.assertFalse(active[0], f"ACTIVE at {w:.0f} deg")
          out.append((active[0], req, icon))
        self.assertGreater(sum(a for a, _, _ in out), 100)
        self._assert_active_steps(out)

  def test_route_3a_after_the_eps_flag_no_retry_while_the_hand_holds(self):
    """The flag (steerFaultTemporary 61 frames) in the region, the hand still holds along the turn, also after the wheel unwinds out of the region:
    no ACTIVE frame for 3 s; once the hand is off for the hold time (and the fault 1 s) steering comes back."""
    self.setUp()
    wheel, v = 170.0, 2.68
    counts = self._warm(wheel, v)
    for _ in range(61):
      cb, ok, active, icon, _ = self._frame(False, wheel, wheel, wheel_counts=counts, torque=361.0, pressed=True, fault=True)
      self.assertEqual((ok, active), ([True], [False]))
    self.assertTrue(self.cc.takeover.strict)
    unwind = 170.0
    for i in range(300):  # hand on, wheel unwinds from 170 to 120 deg (out of the region, still a curve for F22)
      unwind = max(120.0, unwind - 0.7)
      cb, ok, active, icon, _ = self._frame(True, unwind, unwind, wheel_counts=counts, torque=300.0, pressed=True)
      self.assertEqual((ok, active, icon), ([True], [False], ICON_PAUSED), f"frame {i}")
    self.assertTrue(self.cc.takeover.paused)
    n_active = 0
    for _ in range(250):  # hand off
      cb, ok, active, icon, _ = self._frame(True, unwind, unwind, wheel_counts=counts)
      self.assertEqual(ok, [True])
      n_active += active[0]
    self.assertFalse(self.cc.takeover.paused)
    self.assertFalse(self.cc.takeover.strict)
    self.assertGreater(n_active, 100)

  def test_inactive_beyond_the_eps_limit_is_clipped_because_the_panda_demands_it(self):
    """A wheel beyond the EPS limit (route 3a 1:28.3-1:29.7 was 362-409 deg; the LFA_ALT limit is 175 deg): the INACTIVE request is fixed at 175.0. The
    panda clamps the INACTIVE band to clamp(measured, 175) +- 0.1 deg, so the clip is what it accepts; the unclipped measured angle is rejected (and
    would reset its angle state)."""
    from opendbc.sunnypilot.car.hyundai.lfa_alt import create_lfa_alt
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.setUp()
        for wheel in (181.0, 220.0, 409.0):
          cb, ok, active, icon, req = self._frame(False, sign * wheel, sign * wheel, 4.9)
          self.assertEqual((len(cb), ok, active), (1, [True], [False]))
          self.assertEqual(req, sign * EPS_ANGLE_MAX)
        msg = create_lfa_alt(self.cc.packer, self.cc.CAN, False, sign * 409.0, 0)
        self.assertFalse(self.safety.safety_tx_hook(self.libsafety_py.make_CANPacket(msg[0], msg[2], msg[1])))

  def test_a_wheel_past_the_eps_limit_never_gets_an_active_request(self):
    """Route 45's -341 deg, route 3a's 250 deg: with the request capped at 175 deg the guard band (wheel +- 10) leaves no request past 185 deg, so
    every frame is inactive at 175.0 (accepted), grey, and the unsendable counter pauses steering; ACTIVE starts again below the region."""
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.setUp()
        counts = self._warm(sign * 170.0, 2.68)
        n_active = 0
        for i in range(200):
          wheel = sign * min(341.0, 170.0 + 2.0 * i)
          cb, ok, active, icon, req = self._frame(True, wheel, wheel, wheel_counts=counts)
          self.assertEqual((len(cb), ok), (1, [True]))
          if abs(wheel) > 190.0:
            self.assertEqual((active, icon, req), ([False], ICON_PAUSED, sign * EPS_ANGLE_MAX), f"frame {i}")
          n_active += active[0]
        self.assertTrue(self.cc.takeover.paused)
        self.assertGreater(n_active, 5)

  def test_outside_the_region_the_along_turn_hold_is_unchanged(self):
    """Above 6 m/s, or a wheel within 140 deg, the F22 along-turn exemption still holds: same controller, hand 300 along the turn, pressed."""
    for v, wheel in ((7.0, 170.0), (2.68, 120.0)):  # 7 m/s: inside the lateral acceleration cap (the cap at 8 m/s is below 170 deg)
      with self.subTest(v=v, wheel=wheel):
        self.setUp()
        counts = self._warm(wheel, v)
        for _i in range(200):
          self._frame(True, wheel, wheel, wheel_counts=counts, torque=300.0, pressed=True)
        self.assertFalse(self.cc.takeover.paused)

  def test_route_42_13_13_a_lagging_wheel_never_falls_more_than_10_deg_behind_the_request(self):
    """Route 42 13:12.5-13:13.6 shape, closed loop against the real panda: 3.2 m/s, hand off, the model's turn tightens 1.5 deg per frame from 60 to
    165 deg while the EPS turns the wheel at most 1 deg per frame toward our request. Before: the request ran ahead (47 deg at the wheel's 144 deg in
    the log), and the guard's band then left no sendable request. Now every frame: one 0xCB, accepted, ACTIVE, the request at most 11 deg ahead of
    the wheel (the band plus one frame of wheel travel), and the wheel still reaches the model's angle."""
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.setUp()
        v, wheel = 3.2, sign * 60.0
        counts = self._warm(wheel, v)
        model = wheel
        for i in range(200):
          model = sign * min(165.0, abs(model) + 1.5)
          cb, ok, active, _, req = self._frame(True, wheel, model, wheel_counts=counts)
          self.assertEqual((len(cb), ok, active), (1, [True], [True]), f"frame {i}: wheel {wheel:.1f}")
          self.assertLessEqual(abs(req - wheel), 11.0, f"frame {i}: request {req:.1f}, wheel {wheel:.1f}")
          wheel += max(-1.0, min(1.0, req - wheel))
        self.assertAlmostEqual(wheel, model, delta=1.0)

  def test_resume_outside_the_guard_region_stays_inside_the_vehicle_model_window(self):
    """R1 P1: 30 m/s, the model asks 20 deg more than the wheel, the wheel moves 0.8 deg per frame (under the 1 deg 'fast' threshold) while lateral resumes.
    The convergence clip (wheel +- 5) used to undo the vehicle model's 0.2 deg step: the request ran ahead, the panda rejected it, reset its reference
    to the measured angle and Python kept the rejected request. Every frame must be one accepted 0xCB, the request must keep stepping within the model."""
    counts = (round(30.0 * 3.6 / 0.03125),) * 4
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.setUp()
        for _ in range(100):
          self._frame(False, 0.0, 0.0, wheel_counts=counts)
        n_active, prev = 0, None
        for i in range(300):
          wheel = sign * min(8.0, 0.8 * (i + 1))
          cb, ok, active, icon, req = self._frame(True, wheel, wheel + sign * 20.0, wheel_counts=counts)
          self.assertEqual((len(cb), ok), (1, [True]), f"frame {i}: wheel {wheel:.1f}")
          if active[0] and prev is not None:
            self.assertLessEqual(abs(req - prev), 0.3, f"frame {i}")
          prev = req if active[0] else None
          n_active += active[0]
        self.assertGreater(n_active, 100)

  def test_eps_angle_limit_holds_in_every_window_up_to_six_m_s(self):
    """R1 P1: the 175 deg cap used to apply only in the guard band (below 5 m/s); at 5.5 m/s, wheel 186, last 175 the step window returned 176 and the
    request could ratchet outward. Filter level, then closed loop against the panda."""
    f = TakeoverFilter(150, 100, 500, 10, max_angle=EPS_ANGLE_MAX)
    for sign in (1, -1):
      for v in (3.0, 5.0, 5.5, 5.99):
        self.assertLessEqual(abs(f.limit(sign * 190.0, sign * 175.0, sign * 186.0, v) or 0.0), EPS_ANGLE_MAX, f"v {v}")
      f.resuming = True
      self.assertLessEqual(abs(f.limit(sign * 190.0, sign * 175.0, sign * 186.0, 5.5) or 0.0), EPS_ANGLE_MAX)
      f.resuming = False
    for sign in (1, -1):
      with self.subTest(sign=sign):
        self.setUp()
        counts = self._warm(sign * 170.0, 5.5)
        for i in range(60):
          cb, ok, active, icon, req = self._frame(True, sign * 186.0, sign * 190.0, wheel_counts=counts)
          self.assertEqual((len(cb), ok), (1, [True]), f"frame {i}")
          self.assertLessEqual(abs(req), EPS_ANGLE_MAX, f"frame {i}")

  def test_resume_rate_never_exceeds_the_eps_rate(self):
    """R1 P1: HyundaiResumeAngleRate above 22 (2.2 deg per frame) must not let the resume step past what the EPS takes."""
    self.assertEqual(TakeoverFilter(150, 100, 500, 50, max_rate=2.2).resume_rate, 2.2)
    self.assertEqual(TakeoverFilter(150, 100, 500, 10, max_rate=2.2).resume_rate, 1.0)

    def tweak(cp_sp):
      cp_sp.hyundaiTakeoverTorque, cp_sp.hyundaiTakeoverReleaseTorque, cp_sp.hyundaiTakeoverHoldMs, cp_sp.hyundaiResumeAngleRateX10 = 150, 100, 500, 90
    _, _, cc, _ = gv70_controller(tweak_sp=tweak)
    self.assertEqual(cc.takeover.resume_rate, cc.params.ANGLE_LIMITS.MAX_ANGLE_RATE)


class TestNudgeResumeRoute55(unittest.TestCase):
  """Route 55 3:16.9-3:19.0 (T24), closed loop against the real panda (libsafety, CAMERA_HANDBACK): 14.3 m/s, openpilot steering with lateral
  permission, cruise off. The driver holds the wheel (takeover pause), the car's lane-departure nudge comes, so the camera's 0xCB reaches the car and
  ours is not sent; the driver lets go during the nudge and openpilot steers again while the nudge window is open. On the car the first ACTIVE frame
  was 1.0 deg from the last frame sent, the panda rejected all 87 ACTIVE frames and no 0xCB reached the car for 0.54 s (EPS flag). The fix is the
  controller's lat_resync: the first ACTIVE frame after an unsent cycle goes out as one inactive frame at the measured angle."""

  V = 14.3
  camera_first = False

  def setUp(self):
    from opendbc.safety.tests.common import CANPackerSafety
    from opendbc.safety.tests.libsafety import libsafety_py
    self.libsafety_py = libsafety_py
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.cc, self.cs = _carcontroller(150, curve_hold=True)
    self.assertTrue(self.cc.CP_SP.safetyParam & HyundaiSafetyFlagsSP.CAMERA_HANDBACK)
    self.safety.set_current_safety_param_sp(self.cc.CP_SP.safetyParam)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, int(self.cc.CP.safetyConfigs[-1].safetyParam))
    self.safety.init_tests()
    self.safety.set_controls_allowed_lateral(True)
    self.t = 0

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def _cycle(self, wheel, model, torque, ind, camera_first):
    """One 10 ms cycle: the camera's status, the speed and the wheel reach the panda, the controller sends, the panda decides (the camera's 0xCB on the
    bus before ours or after). Returns (ours ACTIVE and accepted, any 0xCB reached the car)."""
    self.t += 10_000
    self.safety.set_timer(self.t)
    if self.t % 50_000 == 0:
      self.safety.safety_rx_hook(self.packer.make_can_msg_safety("CCNC_0x161", 2, {"LFA_ICON": 0}))
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("LFA", 2, {"LKA_SysIndReq": ind}))
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, {f"WHL_Spd{p}Val": self.V * 3.6 for p in ("FL", "FR", "RL", "RR")}))
    self.safety.safety_rx_hook(self.packer.make_can_msg_safety("MDPS", 0, {"MDPS_EstStrAnglVal": wheel}))
    self.cs.out = structs.CarState(vEgo=self.V, vEgoRaw=self.V, steeringTorque=torque, steeringPressed=abs(torque) > 100, steeringAngleDeg=wheel)
    self.cs.cam_lfa_icon, self.cs.cam_lka_ind = 0, ind
    self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    self.cs.lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 1}
    self.cs.msg_161, self.cs.msg_162, self.cs.msg_1b5 = (dbc_defaults(m) for m in ("CCNC_0x161", "CCNC_0x162", "FR_CMR_03_50ms"))
    self.cs.main_cruise_enabled = False
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0}
    self.cc.tuning = SimpleNamespace(stopping=False, actual_accel=0.0, jerk_lower=0.5, jerk_upper=0.5)
    CC = structs.CarControl(latActive=True, enabled=False)
    CC.actuators.steeringAngleDeg = model
    camera = camera_first and self.safety.safety_fwd_hook(2, 0xCB) == 0
    _, sends = self.cc.update(CC.as_reader(), structs.CarControlSP(), self.cs, 0)
    cb = [s for s in sends if s[0] == 0xCB]
    ok = [bool(self.safety.safety_tx_hook(self.libsafety_py.make_CANPacket(s[0], s[2], s[1]))) for s in cb]
    if not camera_first:
      camera = self.safety.safety_fwd_hook(2, 0xCB) == 0
    active_ok = any(o and (bytes(s[1])[3] >> 4) == 2 for s, o in zip(cb, ok, strict=True))
    return active_ok, camera or any(ok)

  def _run(self, release_s, wheel_rate):
    """2 s steering at 5.3 deg, 0.5 s held (torque 300), then the nudge (1.0 s) during which the driver lets go after release_s and the camera moves
    the wheel at wheel_rate deg/s, then 1.5 s with the model at -3.5 deg. Returns (longest run of cycles without a 0xCB at the car after the nudge
    starts, accepted ACTIVE frames in the 1.0 s after the nudge ends)."""
    wheel, gap, longest, active_after = 5.3, 0, 0, 0
    phases = [(200, 5.3, 0.0, 2), (50, 5.3, 300.0, 2)]
    phases += [(1, None, 300.0 if i < release_s * 100 else 0.0, 3) for i in range(100)]
    phases += [(150, -3.5, 0.0, 2)]
    cycle = 0
    for n, model, torque, ind in phases:
      for _ in range(n):
        if ind == 3:
          wheel += wheel_rate / 100
        active_ok, reached = self._cycle(wheel, wheel if model is None else model, torque, ind, self.camera_first)
        cycle += 1
        if cycle > 250:
          gap = 0 if reached else gap + 1
          longest = max(longest, gap)
        if 350 < cycle <= 450:
          active_after += active_ok
    return longest, active_after

  def test_route_55_resume_during_the_nudge(self):
    """The driver lets go 0.3-0.7 s into the nudge while the camera moves the wheel (0 to -6 deg/s): the car never goes two cycles without a 0xCB,
    and openpilot steers again after the takeover hold (> 0.5 s of accepted ACTIVE frames in the 1.0 s after the nudge), in both bus orders."""
    for camera_first in (False, True):
      for release_s in (0.3, 0.5, 0.7):
        for wheel_rate in (0.0, -3.0, -6.0):
          with self.subTest(camera_first=camera_first, release_s=release_s, wheel_rate=wheel_rate):
            self.setUp()
            self.camera_first = camera_first
            longest, active_after = self._run(release_s, wheel_rate)
            self.assertLessEqual(longest, 1, "cycles in a row without a 0xCB at the car")
            self.assertGreater(active_after, 50, "accepted ACTIVE frames in the 1.0 s after the nudge")

  def test_first_active_frame_after_an_unsent_cycle_is_one_inactive_frame(self):
    """The controller side of the fix: after a cycle without our 0xCB (the camera's nudge owned the slot) the next ACTIVE request is replaced by one
    inactive frame at the measured angle (lat_resync), then ACTIVE frames follow."""
    kinds = []
    for step in range(6):
      self.cc.lat_sent = step != 0  # step 0 follows an unsent cycle
      self.cs.out = structs.CarState(vEgo=self.V, vEgoRaw=self.V, steeringAngleDeg=5.3)
      self.cs.cam_lfa_icon, self.cs.cam_lka_ind = 0, 2
      self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
      self.cs.lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 1}
      self.cs.msg_161, self.cs.msg_162 = dbc_defaults("CCNC_0x161"), dbc_defaults("CCNC_0x162")
      self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0}
      self.cc.tuning = SimpleNamespace(stopping=False, actual_accel=0.0, jerk_lower=0.5, jerk_upper=0.5)
      self.cc.frame = 2 + step * 2  # off the icon cadence
      CC = structs.CarControl(latActive=True, enabled=False)
      CC.actuators.steeringAngleDeg = 5.3
      _, sends = self.cc.update(CC.as_reader(), structs.CarControlSP(), self.cs, 0)
      cb = [s for s in sends if s[0] == 0xCB]
      self.assertEqual(len(cb), 1)
      kinds.append(bytes(cb[0][1])[3] >> 4)
      self.assertEqual(self.cc.lat_resync, step == 0)
    self.assertEqual(kinds[0], 1)  # inactive (ADAS_ActvACILvl2Sta 1), one frame
    self.assertEqual(set(kinds[1:]), {2})  # ACTIVE after that


class TestPandaAngleCheck(unittest.TestCase):
  """The controller's mirror of the panda acceptance (panda_angle.py) against the real libsafety, in the GV70's own angle model."""

  def setUp(self):
    from opendbc.safety.tests.common import CANPackerSafety
    from opendbc.safety.tests.libsafety import libsafety_py
    self.libsafety_py = libsafety_py
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    cp, cp_sp = car_params()
    self.safety.set_current_safety_param_sp(cp_sp.safetyParam)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, int(cp.safetyConfigs[-1].safetyParam))
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)
    self.t = 0

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def test_mirror_model_is_the_panda_model_of_the_platform(self):
    """The mirror's slip factor, steer ratio and wheelbase are the GV70 row of the panda's table (angle model 11)."""
    cp, cp_sp = car_params()
    model_id = (cp_sp.safetyParam >> 11) & 0xF
    self.assertEqual(model_id, HyundaiAngleSteeringModel.GENESIS_GV70_2ND_GEN)
    table = MODELS_H.read_text()
    row = re.search(r"\[HYUNDAI_ANGLE_MODEL_GENESIS_GV70_2ND_GEN\] = \{\s*\.slip_factor = ([-\d.e]+),\s*\.steer_ratio = ([\d.]+),\s*"
                    + r"\.wheelbase = ([\d.]+),", table)
    self.assertIsNotNone(row)
    check = _panda_angle_check()
    # the panda's table is float32, CarParams too: equal to float32 precision
    self.assertAlmostEqual(check.slip_factor, float(row.group(1)), places=8)
    self.assertAlmostEqual(check.steer_ratio, float(row.group(2)), places=6)
    self.assertAlmostEqual(check.wheelbase, float(row.group(3)), places=6)

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
    probe = _panda_angle_check()
    for v in [0.5, 2.0, 4.76, 4.9197049, 6.0, 10.0] + [4.5 + 0.0137 * k for k in range(40)]:
      step, max_angle = probe.limits_can(v)
      for last in (0.0, 146.0, -216.0, 300.0):
        check = _panda_angle_check()
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
    probe = _panda_angle_check()
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
      check = _panda_angle_check()
      for k in sorted(rng.sample(range(11), rng.randint(0, 6)) + [11]):
        check.add_speed(speeds[k])
      last = rng.uniform(-340.0, 340.0)
      step, _ = probe.limits_can(min(speeds[6:]))
      request = max(-359.0, min(359.0, last + rng.choice((-1, 1)) * rng.uniform(0.0, 1.3) * step / 10))
      if check.accepts(request, last):
        accepted += 1
        for s in samples[6:]:
          self.safety.safety_rx_hook(self.packer.make_can_msg_safety("WHEEL_SPEEDS", 0, _wheel_speeds(s)))
        self.t = (self.t + 300_000) % 2**32
        self.safety.set_timer(self.t)
        self.safety.set_desired_angle_last(round(last * 10))
        self.assertTrue(self.safety.safety_tx_hook(self.packer.make_can_msg_safety("LFA_ALT", 0, {"ADAS_ActvACILvl2Sta": 2, "ADAS_StrAnglReqVal": request})),
                        f"samples={samples[6:]} last={last} request={request}")
    self.assertGreater(accepted, 200)  # the test does exercise accepted frames

  def test_backstop_accepts_small_steps_and_rejects_big_ones(self):
    """last 146 at 4.76 m/s: a 1 deg step (the controller's cap in the guard region) passes; a 24 deg step does not, for the backstop and the panda."""
    check = _panda_angle_check()
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
