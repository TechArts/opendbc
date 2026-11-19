import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai import handback
from opendbc.car.hyundai.handback import CameraHandback, CounterFollower, LkaWindow, LKA_ICON_SETTLE_FRAMES, LKA_HOLD_FRAMES, LAT_IDLE_FRAMES, \
                                          LFA_PRESS_BLINK_FRAMES
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.gv70 import DBC_NAME, dbc_defaults, gv70_controller
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car.hyundai.carstate_ext import CAMERA_STALE_FRAMES
from opendbc.sunnypilot.car.hyundai.ccnc import CcncCarController
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type
CAMERA_HANDBACK = HyundaiSafetyFlagsSP.CAMERA_HANDBACK


class TestLkaWindow(unittest.TestCase):
  def _run(self, w, icon, ind, start, frames):
    open_ = False
    for f in range(start, start + frames):
      open_ = w.update(icon, ind, f)
    return open_

  def test_nudge_needs_settled_icon_and_lane_departure(self):
    w = LkaWindow()
    self.assertFalse(w.update(None, None, 0))  # nothing received yet
    self.assertFalse(self._run(w, 0, 3, 1, LKA_ICON_SETTLE_FRAMES - 2))  # icon 0 for less than the settle time
    self.assertTrue(self._run(w, 0, 3, 1 + LKA_ICON_SETTLE_FRAMES - 2, 5))
    w = LkaWindow()
    self.assertFalse(self._run(w, 0, 2, 0, 3 * LKA_ICON_SETTLE_FRAMES))  # settled, but no lane departure (an LFA start has LKA_SysIndReq 1/2)

  def test_lfa_never_opens(self):
    for icon, ind in ((2, 3), (1, 3), (2, 2), (3, 3)):
      with self.subTest(icon=icon, ind=ind):
        self.assertFalse(self._run(LkaWindow(), icon, ind, 0, 3 * LKA_ICON_SETTLE_FRAMES))

  def test_blink_restarts_settle_time(self):
    w = LkaWindow()
    self._run(w, 0, 3, 0, 3 * LKA_ICON_SETTLE_FRAMES)
    w.update(3, 3, 1000)  # the off-press blink 3,0,3,0
    self.assertFalse(self._run(w, 0, 3, 1001, LKA_ICON_SETTLE_FRAMES - 5))
    self.assertTrue(self._run(w, 0, 3, 1001 + LKA_ICON_SETTLE_FRAMES - 5, 10))

  def test_window_closes_after_hold(self):
    w = LkaWindow()
    self._run(w, 0, 3, 0, 3 * LKA_ICON_SETTLE_FRAMES)
    base = 3 * LKA_ICON_SETTLE_FRAMES
    self.assertTrue(self._run(w, 0, 2, base, LKA_HOLD_FRAMES))
    self.assertFalse(self._run(w, 0, 2, base + LKA_HOLD_FRAMES, 5))

  def test_lfa_on_is_settled_state(self):
    w = LkaWindow()
    self.assertFalse(w.lfa_on)
    w.update(2, 1, 0)
    self.assertTrue(w.lfa_on)
    for icon, f in ((3, 1), (0, 2), (3, 3), (0, 4)):  # the off blink alone does not flip it before the icon settles at 0
      w.update(icon, 1, f)
    self.assertTrue(w.lfa_on)
    self._run(w, 0, 1, 5, LKA_ICON_SETTLE_FRAMES)
    self.assertFalse(w.lfa_on)
    w.update(1, 1, 500)  # grey: on, not steering
    self.assertTrue(w.lfa_on)
    for icon, f in ((3, 501), (0, 502), (3, 503), (1, 504)):  # lane lost blink 3,0,3,1: stays on
      w.update(icon, 1, f)
    self.assertTrue(w.lfa_on)

  def test_lfa_off_press_clears_lfa_on_at_once(self):
    # route 34 5:46.5: LFA button press at t, the camera still shows 2 for 0.25 s, blinks 3,0,3,0, settles 0 1.7 s later: ours was grey for 2.5 s
    w = LkaWindow()
    w.update(2, 1, 0)
    self.assertTrue(w.lfa_on)
    w.lfa_pressed(10)
    self.assertFalse(w.lfa_on)
    for f in range(10, 10 + 12):  # the camera still shows its old icon
      w.update(2, 1, f)
      self.assertFalse(w.lfa_on)
    for icon, f in ((3, 40), (0, 70), (3, 90), (0, 120)):
      w.update(icon, 1, f)
      self.assertFalse(w.lfa_on)
    self._run(w, 0, 1, 121, LKA_ICON_SETTLE_FRAMES)
    self.assertFalse(w.lfa_on)

  def test_lfa_press_that_the_camera_ignores_or_turns_on_again(self):
    w = LkaWindow()
    w.update(1, 1, 0)
    w.lfa_pressed(5)
    self.assertFalse(w.lfa_on)
    for f in range(5, 5 + LFA_PRESS_BLINK_FRAMES + 10):  # no blink within the window: the press changed nothing, the camera's LFA is still on
      w.update(1, 1, f)
    self.assertTrue(w.lfa_on)
    w.lfa_pressed(200)  # off blink, then on again (another press)
    w.update(3, 1, 210)
    w.update(0, 1, 230)
    self.assertFalse(w.lfa_on)
    w.update(1, 1, 260)
    self.assertTrue(w.lfa_on)

  def test_lfa_press_with_lfa_off_changes_nothing(self):
    w = LkaWindow()
    w.lfa_pressed(5)  # turning on: not a state we show
    self.assertFalse(w.lfa_on)
    w.update(3, 1, 10)
    w.update(1, 1, 40)
    self.assertTrue(w.lfa_on)


class TestCarStateCameraLka(unittest.TestCase):
  def test_camera_icon_and_lka_indicator_are_read(self):
    # recorded camera frames (route 2f): 0x12A with LKA_SysIndReq 3 (lane departure), 0x161 with LFA_ICON 2
    f12a = bytes.fromhex("9c0b0158c00000000082080004000000")
    f161 = bytes.fromhex("3a93b1c000000000c1fff2c803000040000000000000000000ff000002000000")
    cp, cp_sp, _, cs = gv70_controller()
    self.assertTrue(cp_sp.safetyParam & CAMERA_HANDBACK)
    parsers = cs.get_can_parsers(cp, cp_sp)
    self.assertEqual((cs.cam_lfa_icon, cs.cam_lka_ind), (None, None))
    for t in (10_000_000, 20_000_000):  # the first update registers the messages
      parsers[Bus.cam].update([(t, [(0x12A, f12a, 2), (0x161, f161, 2)])])
      cs.update(parsers)
    self.assertEqual((cs.cam_lfa_icon, cs.cam_lka_ind), (2, 3))

  def test_signals_of_a_camera_that_stopped_are_not_passed_on(self):
    """R1 P1 3: the parser keeps the last value of a message that stopped (its receive time only says 'ever received'). A stale LKA_SysIndReq 3 / icon 0
    kept the controller silent for the camera while the panda (0.5 s expiry) already blocked it again: nobody steered. They go None after 0.3 s."""
    packer = CANPacker(DBC_NAME)  # frames with running counters and checksums, like the camera's: the parser drops repeats of one frame

    def frames():
      return [packer.make_can_msg("LFA", 2, {"LKA_SysIndReq": 3}), packer.make_can_msg("CCNC_0x161", 2, {"LFA_ICON": 2})]
    cp, cp_sp, _, cs = gv70_controller()
    parsers = cs.get_can_parsers(cp, cp_sp)
    t = 0
    inds = []
    for _ in range(10):
      t += 10_000_000
      parsers[Bus.cam].update([(t, frames())])
      cs.update(parsers)
      inds.append(cs.cam_lka_ind)
    self.assertEqual(cs.cam_lfa_icon, 2)  # the icon is a state, fresh while it keeps arriving
    self.assertIn(3, inds)  # the indicator is an event: it is only passed on in the update its frame arrived in (see the dropout test below)
    seen = []
    for _ in range(40):  # the camera's frames stop, the car state keeps updating every 10 ms
      t += 10_000_000
      parsers[Bus.cam].update([(t, [])])
      cs.update(parsers)
      seen.append((cs.cam_lfa_icon, cs.cam_lka_ind))
    # the parser stamps a message a few updates late, which only makes the window end earlier: fresh right after the last frame, stale for good by
    # CAMERA_STALE_FRAMES updates after it
    self.assertEqual(seen[0][0], 2)
    self.assertEqual(seen[CAMERA_STALE_FRAMES:], [(None, None)] * (40 - CAMERA_STALE_FRAMES))
    self.assertLess(CAMERA_STALE_FRAMES * 10_000, 500_000)  # strictly inside the panda's 0.5 s
    for _ in range(8):  # and back when it resumes
      t += 10_000_000
      parsers[Bus.cam].update([(t, frames())])
      cs.update(parsers)
    self.assertEqual(cs.cam_lfa_icon, 2)

  def test_not_read_without_the_flag(self):
    cp, cp_sp, _, cs = gv70_controller(tweak_sp=lambda sp: setattr(sp, "safetyParam", sp.safetyParam & ~CAMERA_HANDBACK))
    parsers = cs.get_can_parsers(cp, cp_sp)
    f161 = bytes.fromhex("3a93b1c000000000c1fff2c803000040000000000000000000ff000002000000")
    for t in (10_000_000, 20_000_000):
      parsers[Bus.cam].update([(t, [(0x161, f161, 2)])])
      cs.update(parsers)
    self.assertEqual((cs.cam_lfa_icon, cs.cam_lka_ind), (None, None))


class TestCounterFollower(unittest.TestCase):
  def test_continues_camera_counter(self):
    c = CounterFollower(1 << 16)
    self.assertIsNone(c.next(None))
    self.assertEqual(c.next(10), 10)  # replaces the camera frame that carried 10
    self.assertEqual(c.next(11), 11)
    self.assertEqual(c.next(11), 12)  # no new camera frame yet: keep counting
    self.assertEqual(c.next(13), 13)  # resync to the camera
    self.assertEqual(CounterFollower(256).next(255), 255)
    c = CounterFollower(256)
    c.next(255)
    self.assertEqual(c.next(255), 0)


class TestCarControllerHandback(unittest.TestCase):
  def setUp(self):
    self.cp, self.cp_sp, self.cc, self.cs = gv70_controller()
    self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    self.cs.lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0}
    self.cs.out = structs.CarState()
    self.cc.frame = 2
    self.cc.tuning = SimpleNamespace(stopping=False, actual_accel=0.0, jerk_lower=0.5, jerk_upper=0.5)
    self.hud = structs.CarControl.HUDControl()

  def _control(self, steer_req=False, enabled=False, lat_active=None):
    cc = structs.CarControl()
    cc.enabled = enabled
    cc.latActive = steer_req if lat_active is None else lat_active
    return cc

  def _send(self, cc, steer_req, apply_torque=0):
    """create_canfd_msgs (0xCB) and the MDPS copy (CcncCarController.update), as CarController.update builds them for one frame."""
    msgs = self.cc.create_canfd_msgs(steer_req, apply_torque, 0.0, 0.0, False, self.hud, self.cs, cc)
    msgs += CcncCarController.update(self.cc, self.cs, cc)
    self.cc.frame += 2
    self._last = {m[0]: m[1] for m in msgs}
    return {m[0] for m in msgs}

  def _msgs(self, steer_req=False, enabled=False, buttons=(), frame=None, lat_active=None):
    if frame is not None:
      self.cc.frame = frame
    if self.cc.frame % 5 == 0:
      self.cc.frame += 2  # off the icon cadence (needs real CCNC frames)
    self.cs.out.buttonEvents = [structs.CarState.ButtonEvent(type=t, pressed=True) for t in buttons]
    return self._send(self._control(steer_req, enabled, lat_active), steer_req, 1 if steer_req else 0)

  def _nudge(self, icon, ind, lat_active=False, seconds=2.2):
    """feed the camera's icon / LKA_SysIndReq for a while; returns what the last call sent"""
    sent = set()
    self.cs.cam_lfa_icon, self.cs.cam_lka_ind = icon, ind
    for _ in range(int(seconds * 50)):
      sent = self._msgs(steer_req=False, lat_active=lat_active)
    return sent

  def test_lka_nudge_sends_nothing_with_mads_off(self):
    self._msgs(steer_req=True)  # openpilot took lateral, then MADS went off
    sent = self._nudge(0, 3)
    self.assertFalse(sent & {0xCB, 0xEA})  # the panda forwards the camera's frame and the real MDPS

  def test_inactive_frames_when_not_a_lka_nudge(self):
    self._msgs(steer_req=True)
    for icon, ind in ((2, 3), (1, 3), (2, 2), (0, 2), (3, 3)):  # LFA, or no lane departure
      with self.subTest(icon=icon, ind=ind):
        sent = self._nudge(icon, ind)
        self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})

  def test_lka_nudge_waits_until_openpilot_has_stopped_steering(self):
    """The panda opens its window 0.2 s after openpilot's last ACTIVE frame; we go silent only after 0.3 s, so the car always has a 0xCB. A takeover
    pause (MADS on, no request) is not steering: the nudge may steer; an ACTIVE request sends at once (the panda drops it and closes its window)."""
    self.cs.cam_lfa_icon, self.cs.cam_lka_ind = 0, 2
    for _ in range(100):  # 2 s: icon settled, openpilot steering, no lane departure
      self._msgs(steer_req=True)
    self.cs.cam_lka_ind = 3
    start = self.cc.frame
    while self.cc.frame - start <= LAT_IDLE_FRAMES - 2:  # openpilot's request ends with the nudge: within 0.3 s we keep sending
      self.assertEqual(self._msgs(steer_req=False, lat_active=True) & {0xCB, 0xEA}, {0xCB, 0xEA})
    for _ in range(5):
      self._msgs(steer_req=False, lat_active=True)
    self.assertFalse(self._msgs(steer_req=False, lat_active=True) & {0xCB, 0xEA})  # paused, 0.3 s idle: the nudge steers
    self.assertEqual(self._msgs(steer_req=True, lat_active=True) & {0xCB, 0xEA}, {0xCB, 0xEA})  # openpilot steers again: at once

  def test_lka_nudge_with_openpilot_cruise_on_sends_nothing(self):
    """Route 48 35:59-37:02 (T23): openpilot cruise on, steering off by the LFA button, the car's nudge: silent (the panda forwards the camera)."""
    self._msgs(steer_req=True, enabled=True)
    self.cs.cam_lfa_icon, self.cs.cam_lka_ind = 0, 3
    for _ in range(110):
      sent = self._msgs(steer_req=False, enabled=True, lat_active=False)
    self.assertFalse(sent & {0xCB, 0xEA})
    self.assertIn(0x1A0, sent)  # cruise stays openpilot's

  def test_openpilot_owns_lateral_from_boot(self):
    self.assertEqual(self._nudge(2, 3) & {0xCB, 0xEA}, {0xCB, 0xEA})  # the car's LFA (icon 2): ours replaces it, also with a lane departure
    self.assertEqual(self._nudge(2, 2) & {0xCB, 0xEA}, {0xCB, 0xEA})

  def test_lka_nudge_from_boot_sends_nothing(self):
    self.assertFalse(self._nudge(0, 3) & {0xCB, 0xEA})  # MADS never used, the car's own nudge may steer

  def test_flag_set(self):
    self.assertTrue(self.cp_sp.safetyParam & CAMERA_HANDBACK)
    self.assertIsInstance(self.cc.handback, CameraHandback)

  def test_lateral_and_mdps_while_active_then_inactive_frames(self):
    sent = self._msgs(steer_req=True)
    self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})
    sent = self._msgs(steer_req=False)  # inactive: still sending 'off' frames
    self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})

  def test_lfa_button_never_ends_lateral(self):
    # the panda keeps lateral for good, and so does openpilot: inactive frames only, never the camera's
    self._msgs(steer_req=True, enabled=True)
    self._msgs(steer_req=False, enabled=False, buttons=(ButtonType.lkas,))
    for _ in range(5):
      sent = self._msgs()
      self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})

  def _lfa_alt(self, dat):
    parser = CANParser(DBC_NAME, [("LFA_ALT", 0)], self.cc.CAN.ECAN)
    parser.update([(0, [(0xCB, dat, self.cc.CAN.ECAN)])])
    return parser.vl["LFA_ALT"]

  def test_takeover_pause_sends_inactive_frames(self):
    # LFA on while the driver pushes hard: MADS allows steering (latActive) but the takeover filter sends no request; the frame is inactive
    sent = self._msgs(lat_active=True, steer_req=False)
    self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})
    out = self._lfa_alt(self._last[0xCB])
    self.assertEqual(out["ADAS_ActvACILvl2Sta"], 1)  # inactive
    self.assertEqual(out["ADAS_ACIAnglTqRedcGainVal"], 0)

  def test_inactive_frames_from_boot_while_mads_never_used(self):
    for _ in range(10):
      sent = self._msgs(lat_active=False)
      self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})

  def _lfa_icon(self, buttons=()):
    """the LFA_ICON of the 0x161 frame we send (the icon cadence needs frame % 5 == 0 and real CCNC frames)"""
    self.cc.frame = (self.cc.frame // 5 + 1) * 5
    self.cs.out.buttonEvents = [structs.CarState.ButtonEvent(type=t, pressed=True) for t in buttons]
    cc = structs.CarControl()
    self.cc.create_canfd_msgs(False, 0, 0.0, 0.0, False, self.hud, self.cs, cc)
    msgs = CcncCarController.update(self.cc, self.cs, cc)
    self.cc.frame += 1
    parser = CANParser(DBC_NAME, [("CCNC_0x161", 0)], 0)
    parser.update([(0, [(m[0], m[1], 0) for m in msgs if m[0] == 0x161])])
    return parser.vl["CCNC_0x161"]["LFA_ICON"]

  def test_icon_off_at_the_lfa_off_press_not_after_the_blink(self):
    self.cs.msg_161, self.cs.msg_162 = dbc_defaults("CCNC_0x161"), dbc_defaults("CCNC_0x162")
    self.cs.cam_lfa_icon, self.cs.cam_lka_ind = 2, 1
    for _ in range(4):
      self.assertEqual(1, self._lfa_icon())  # MADS off, the car's LFA on and green: ours grey (the real state of the car's LFA)
    self.assertEqual(0, self._lfa_icon((ButtonType.lkas,)))  # LFA button: ours is off at once
    for icon in (3, 0, 3, 0):  # the camera blinks, then settles at 0
      self.cs.cam_lfa_icon = icon
      self.assertEqual(0, self._lfa_icon())

  def _control_msgs(self, lat_active=True, enabled=False):
    cc = structs.CarControl(enabled=enabled, latActive=lat_active)
    msgs = self.cc.create_canfd_msgs(True, 1, 0.0, 0.0, False, self.hud, self.cs, cc)
    msgs += CcncCarController.update(self.cc, self.cs, cc)
    return {m[0]: m[1] for m in msgs}

  def test_counter_follows_camera(self):
    by_addr = self._control_msgs()
    parser = CANParser(DBC_NAME, [("LFA_ALT", 0)], self.cc.CAN.ECAN)
    parser.update([(0, [(0xCB, by_addr[0xCB], self.cc.CAN.ECAN)])])
    self.assertEqual(parser.vl["LFA_ALT"]["COUNTER"], 100)  # the camera's 0xCB counter, then it counts on
    by_addr = self._control_msgs()
    parser.update([(0, [(0xCB, by_addr[0xCB], self.cc.CAN.ECAN)])])
    self.assertEqual(parser.vl["LFA_ALT"]["COUNTER"], 101)
    self.cs.lfa_alt = {"COUNTER": 7, "ADAS_ActvACILvl2Sta": 2}
    by_addr = self._control_msgs()
    parser.update([(0, [(0xCB, by_addr[0xCB], self.cc.CAN.ECAN)])])
    self.assertEqual(parser.vl["LFA_ALT"]["COUNTER"], 7)

  def test_mdps_echo_value(self):
    msgs = self._control_msgs()
    parser = CANParser(DBC_NAME, [("MDPS", 0)], self.cc.CAN.CAM)
    parser.update([(0, [(0xEA, msgs[0xEA], self.cc.CAN.CAM)])])
    self.assertEqual(parser.vl["MDPS"]["MDPS_ADASAciActvSta_Lv2"], 2)  # the camera asks for ACI (ACTIVE): echoed
    self.cs.lfa_alt = {"COUNTER": 101, "ADAS_ActvACILvl2Sta": 1}
    msgs = self._control_msgs()
    parser.update([(0, [(0xEA, msgs[0xEA], self.cc.CAN.CAM)])])
    self.assertEqual(parser.vl["MDPS"]["MDPS_ADASAciActvSta_Lv2"], 1)


class TestLongitudinalNotHandedBack(unittest.TestCase):
  """Design difference to the old fork: there is no longitudinal hand-back; the camera's 0x1A0 keeps upstream's static block while openpilot has long."""

  def _controller(self, flag):
    def tweak(sp):
      if not flag:
        sp.safetyParam &= ~CAMERA_HANDBACK
    cp, cp_sp, cc, cs = gv70_controller(tweak_sp=tweak)
    self.assertEqual(bool(cp_sp.safetyParam & CAMERA_HANDBACK), flag)
    cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    cs.lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
    cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0}
    cs.out = structs.CarState()
    cc.tuning = SimpleNamespace(stopping=False, actual_accel=0.0, jerk_lower=0.5, jerk_upper=0.5)
    return cc, cs

  def _scc(self, cc, cs, frame, enabled, icon, ind):
    cs.cam_lfa_icon, cs.cam_lka_ind = icon, ind
    cc.frame = frame
    ctl = structs.CarControl(enabled=enabled, latActive=False)
    msgs = cc.create_canfd_msgs(False, 0, 0.0, 0.5 if enabled else 0.0, False, structs.CarControl.HUDControl(), cs, ctl)
    return next((bytes(m[1]) for m in msgs if m[0] == 0x1A0), None)

  def test_scc_control_is_sent_exactly_as_without_the_flag(self):
    with_flag, plain = self._controller(True), self._controller(False)
    for enabled in (False, True):
      for icon, ind in ((None, None), (0, 3), (0, 2), (2, 3), (3, 3)):
        for frame in range(2, 600, 2):  # past the settle time: the lane-keeping window is open for (0, 3)
          if frame % 5 == 0:
            continue
          expected = self._scc(*plain, frame, enabled, icon, ind)
          self.assertIsNotNone(expected)
          self.assertEqual(self._scc(*with_flag, frame, enabled, icon, ind), expected, (enabled, icon, ind, frame))

  def test_handback_module_has_no_longitudinal_state(self):
    # CameraHandback.update answers one question, lateral; its state is the lane-keeping window, the lateral counter and the last ACTIVE frame
    self.assertEqual(sorted(vars(CameraHandback())), ["lfa_alt_counter", "lka_window", "steer_frame"])
    self.assertEqual({n for n in dir(handback) if "long" in n.lower() or "scc" in n.lower()}, set())


class TestLateralSenderWithPanda(unittest.TestCase):
  """The car controller and the real panda (libsafety) on the same camera frames, every 10 ms: never two 0xCB or MDPS senders; a missing 0xCB only
  at a switch of sender, never two cycles in a row, in both bus orders. Route 48 (T23): 0.85 s with neither, the EPS faulted 0.53 s in."""
  _allow = None

  def setUp(self):
    from opendbc.safety.tests.common import CANPackerSafety
    from opendbc.safety.tests.libsafety import libsafety_py
    self.libsafety_py = libsafety_py
    self.spacker = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.cp, self.cp_sp, self.cc, self.cs = gv70_controller()
    self.safety.set_current_safety_param_sp(self.cp_sp.safetyParam)
    self.safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, int(self.cp.safetyConfigs[-1].safetyParam))
    self.safety.init_tests()
    self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    self.cs.lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 1}
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0}
    self.cs.msg_161, self.cs.msg_162 = dbc_defaults("CCNC_0x161"), dbc_defaults("CCNC_0x162")
    self.cs.out = structs.CarState()
    self.cs.out.vCruiseCluster = 40.0
    self.cc.frame = 0
    self.cc.tuning = SimpleNamespace(stopping=False, actual_accel=0.0, jerk_lower=0.5, jerk_upper=0.5)
    self.hud = structs.CarControl.HUDControl()
    self.t = 0
    self._panda_icon = 0

  def tearDown(self):
    self.safety.set_current_safety_param_sp(0)

  def _cycle(self, icon, ind, steer_req, lat_active, enabled, camera_first, icon_between=None):
    """One 10 ms cycle: the camera's status reaches the panda, the controller sends (0xCB, then its MDPS copy), the panda decides; the camera's 0xCB
    is on the bus before ours (camera_first) or after; icon_between: a camera icon frame between the camera's 0xCB and ours (R1 F1). The controller
    sees the icon the panda had at the end of the previous cycle (CAN latency). Returns (ours 0xCB, camera 0xCB, ours MDPS, real MDPS) as they reach
    the car / the camera."""
    self.t += 10_000
    self.safety.set_timer(self.t)
    self.cs.cam_lfa_icon, self.cs.cam_lka_ind = self._panda_icon, ind
    if self.t % 50_000 == 0:
      self.safety.safety_rx_hook(self.spacker.make_can_msg_safety("CCNC_0x161", 2, {"LFA_ICON": icon}))
      self._panda_icon = icon
    self.safety.safety_rx_hook(self.spacker.make_can_msg_safety("LFA", 2, {"LKA_SysIndReq": ind}))
    cc = structs.CarControl()
    cc.enabled, cc.latActive = enabled, lat_active
    msgs = self.cc.create_canfd_msgs(steer_req, 1 if steer_req else 0, 0.0, 0.0, False, self.hud, self.cs, cc)
    msgs += CcncCarController.update(self.cc, self.cs, cc)
    self.cc.frame += 1
    camera = camera_first and self.safety.safety_fwd_hook(2, 0xCB) == 0
    real_mdps = camera_first and self.safety.safety_fwd_hook(0, 0xEA) == 2
    if icon_between is not None:
      self.safety.safety_rx_hook(self.spacker.make_can_msg_safety("CCNC_0x161", 2, {"LFA_ICON": icon_between}))
      self._panda_icon = icon_between
    tx = {a: [bool(self.safety.safety_tx_hook(self.libsafety_py.make_CANPacket(m[0], m[2], m[1]))) for m in msgs if m[0] == a] for a in (0xCB, 0xEA)}
    if not camera_first:
      camera = self.safety.safety_fwd_hook(2, 0xCB) == 0
      real_mdps = self.safety.safety_fwd_hook(0, 0xEA) == 2
    return sum(tx[0xCB]), int(camera), sum(tx[0xEA]), int(real_mdps)

  def _run(self, phases, icon_between_at=None):
    """phases: (seconds, icon, ind, steer_req, lat_active, enabled). In both bus orders: never two 0xCB or two MDPS in a cycle; a cycle without a
    0xCB only at a switch of sender (the sender after it differs from the one before) and never two in a row."""
    for camera_first in (False, True):
      self.setUp()
      if self._allow:
        getattr(self.safety, self._allow)(True)
      last, gap_from, camera_cycles = 'ours', None, 0
      for seconds, icon, ind, steer_req, lat_active, enabled in phases:
        for _ in range(int(seconds * 100)):
          between = icon_between_at(self.t + 10_000) if icon_between_at else None
          ours, cam, ours_mdps, real_mdps = self._cycle(icon, ind, steer_req, lat_active, enabled, camera_first, between)
          where = f"t={self.t} (camera first: {camera_first})"
          self.assertLessEqual(ours + cam, 1, f"two 0xCB senders at {where}")
          self.assertLessEqual(ours_mdps + real_mdps, 1, f"two MDPS senders at {where}")
          camera_cycles += cam
          if ours + cam == 0:
            self.assertIsNone(gap_from, f"two cycles without 0xCB at {where}")
            gap_from = last
          else:
            now = 'ours' if ours else 'cam'
            if gap_from is not None:
              self.assertNotEqual(now, gap_from, f"a cycle without 0xCB before {where}, not at a switch")
            last, gap_from = now, None
      self.assertGreater(camera_cycles, 50, f"the camera's nudge never reached the car (camera first: {camera_first})")

  def test_route_48_cruise_on_steering_off_nudge(self):
    """Openpilot cruise on (panda controls_allowed), steering on, then off by the LFA button, the car's nudge comes and goes, then openpilot steers
    again in the middle of a nudge."""
    self._allow = "set_controls_allowed"
    self._run([
      (2.0, 0, 2, True, True, True),    # openpilot steers, the camera's icon settles at 0, no lane departure
      (1.5, 0, 2, False, False, True),  # LFA off: steering off, cruise on
      (2.0, 0, 3, False, False, True),  # the car's nudge (route 48 35:59)
      (1.5, 0, 2, False, False, True),  # nudge ends
      (1.0, 0, 3, False, False, True),  # again
      (1.0, 0, 3, True, True, True),    # openpilot steers again mid-nudge
      (1.0, 0, 2, True, True, True),
    ])

  def test_from_boot_and_with_the_cars_lfa(self):
    """No permission (cruise off), the car's LFA (icon 2) and its blink, nudges: the same invariant."""
    self._run([
      (2.0, 0, 3, False, False, False),
      (1.0, 2, 2, False, False, False),
      (0.3, 3, 3, False, False, False),
      (2.0, 0, 3, False, False, False),
      (1.0, 0, 1, False, False, False),
    ])

  def test_takeover_pause_with_permission_then_resume(self):
    """MADS on (lateral permission), a takeover pause (no request) during a nudge, then the request returns."""
    self._allow = "set_controls_allowed_lateral"
    self._run([
      (2.0, 0, 2, True, True, False),
      (1.0, 0, 3, False, True, False),
      (1.0, 0, 3, True, True, False),
      (1.0, 0, 2, True, True, False),
    ])

  def test_camera_icon_between_the_senders(self):
    """R1 F1 for the whole pipeline: while the camera's nudge steers, an icon frame (LFA 2, blink 3, standby 1) arrives between the camera's 0xCB
    and ours, once each, in both bus orders; then openpilot steers again."""
    self._allow = "set_controls_allowed"
    between = {4_000_000: 2, 6_000_000: 3, 8_000_000: 1}
    self._run([
      (2.0, 0, 2, True, True, True),
      (1.5, 0, 2, False, False, True),
      (6.0, 0, 3, False, False, True),
      (1.0, 0, 3, True, True, True),
    ], icon_between_at=lambda t: between.get(t))


class TestOtherConfigsUnchanged(unittest.TestCase):
  def test_flag_only_for_gv70_ccnc_lfa_alt(self):
    for candidate in (CAR.KIA_EV6, CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_SANTA_FE_HEV_5TH_GEN, CAR.GENESIS_GV70_1ST_GEN):
      fingerprint = gen_empty_fingerprint()
      fingerprint[CanBus(None, fingerprint).CAM] = {0xCB: 24}
      cp = CarInterface.get_params(candidate, fingerprint, [], True, False, False)
      cp_sp = CarInterface.get_params_sp(cp, candidate, fingerprint, [], True, False, False)
      self.assertFalse(cp_sp.safetyParam & CAMERA_HANDBACK, candidate)

  def test_gv70_without_lfa_alt_or_ccnc_has_no_handback(self):
    fingerprint = gen_empty_fingerprint()
    cp = CarInterface.get_params(CAR.GENESIS_GV70_2ND_GEN, fingerprint, [], True, False, False)
    cp_sp = CarInterface.get_params_sp(cp, CAR.GENESIS_GV70_2ND_GEN, fingerprint, [], True, False, False)
    self.assertFalse(cp_sp.safetyParam & CAMERA_HANDBACK)


class TestSelectiveLkaDropoutWithPanda(unittest.TestCase):
  """R1 P2: only 0x12A stops after reporting LKA_SysIndReq 3 while the camera's 0x161 zero icon keeps arriving. The real CarState (parser, freshness), the
  hand-back and the real panda on the same frames, every 10 ms. A cached indicator used to restart LkaWindow's own 0.3 s hold for 0.3 s more, so the
  controller stayed silent ~0.6 s while the panda blocked the camera again after 0.5 s: ~10 cycles with no 0xCB sender."""

  def tearDown(self):
    from opendbc.safety.tests.libsafety import libsafety_py
    libsafety_py.libsafety.set_current_safety_param_sp(0)

  def test_no_gap_when_only_0x12a_stops(self):
    from opendbc.safety.tests.common import CANPackerSafety
    from opendbc.safety.tests.libsafety import libsafety_py
    from opendbc.sunnypilot.car.hyundai.lfa_alt import create_lfa_alt
    cp, cp_sp, cc, cs = gv70_controller()
    safety, spacker, camera = libsafety_py.libsafety, CANPackerSafety(DBC_NAME), CANPacker(DBC_NAME)
    safety.set_current_safety_param_sp(cp_sp.safetyParam)
    safety.set_safety_hooks(structs.CarParams.SafetyModel.hyundaiCanfd, int(cp.safetyConfigs[-1].safetyParam))
    safety.init_tests()
    cs.out = structs.CarState()
    parsers = cs.get_can_parsers(cp, cp_sp)
    drop_us = 3_000_000
    gap, longest, camera_cycles = 0, 0, 0
    for frame in range(1, 600):
      t = frame * 10_000
      safety.set_timer(t)
      msgs = []
      if t % 50_000 == 0:
        msgs.append(("CCNC_0x161", {"LFA_ICON": 0}))
      if t < drop_us:
        msgs.append(("LFA", {"LKA_SysIndReq": 3}))
      for name, values in msgs:
        safety.safety_rx_hook(spacker.make_can_msg_safety(name, 2, values))
      parsers[Bus.cam].update([(t * 1000, [camera.make_can_msg(name, 2, values) for name, values in msgs])])
      cs.update(parsers)
      send_lat = cc.handback.update(cs, frame, False)
      camera_passes = safety.safety_fwd_hook(2, 0xCB) == 0
      ours = False
      if send_lat:
        addr, dat, bus = create_lfa_alt(cc.packer, cc.CAN, False, 0.0, 0.0)
        ours = bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)))
      camera_cycles += camera_passes
      gap = gap + 1 if not (camera_passes or ours) else 0
      longest = max(longest, gap)
    self.assertGreater(camera_cycles, 100, "the nudge never reached the car")
    self.assertLessEqual(longest, 2, "cycles in a row with neither sender")


if __name__ == "__main__":
  unittest.main()
