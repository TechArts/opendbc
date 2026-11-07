import unittest
from types import SimpleNamespace

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.takeover import TakeoverFilter, ENTER_FRAMES, RESUME_FRAMES, FAULT_FRAMES, GUARD_FRAMES, CONVERGE_DEG
from opendbc.car.hyundai.values import CAR, DBC


def _defaults(msg):
  return dict(CANParser("hyundai_canfd_generated", [(msg, 0)], 0).vl[msg])


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
    self.assertEqual(f.limit(215.0, 176.0, 175.0, 4.0), 170.0)
    self.assertEqual(f.limit(160.0, 176.0, 175.0, 4.0), 165.0)  # error cap
    self.assertEqual(f.limit(-215.0, -176.0, -175.0, 4.0), -170.0)
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


def _carcontroller(takeover_torque):
  fingerprint = gen_empty_fingerprint()
  can = CanBus(None, fingerprint)
  fingerprint[can.ECAN] = {0x1CF: 8}
  fingerprint[can.CAM] = {0xCB: 24}
  cp = CarInterface.get_params(CAR.GENESIS_GV70_2ND_GEN, fingerprint, [], True, False, False)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.GENESIS_GV70_2ND_GEN)
  cp_sp.hyundaiTakeoverTorque = takeover_torque
  cp_sp.hyundaiTakeoverReleaseTorque, cp_sp.hyundaiTakeoverHoldMs, cp_sp.hyundaiResumeAngleRateX10 = 100, 500, 10
  return CarController(DBC[cp.carFingerprint], cp, cp_sp), CarState(cp, cp_sp)


class TestControllerTakeover(unittest.TestCase):
  def _update(self, cc, cs, torque, angle_cmd=30.0):
    cs.out = structs.CarState(vEgo=10.0, vEgoRaw=10.0, steeringTorque=torque, steeringAngleDeg=5.0)
    cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    cs.cam_lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
    cs.msg_161, cs.msg_162, cs.msg_1b5 = (_defaults(m) for m in ("CCNC_0x161", "CCNC_0x162", "FR_CMR_03_50ms"))
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


if __name__ == "__main__":
  unittest.main()
