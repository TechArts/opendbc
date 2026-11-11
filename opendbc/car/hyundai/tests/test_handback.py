import unittest
from types import SimpleNamespace

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.handback import AxisOwner, CounterFollower, LkaWindow, TAIL_FRAMES, RECENT_ACTION_FRAMES, LKA_ICON_SETTLE_FRAMES, LKA_HOLD_FRAMES
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.gv70 import DBC_NAME, dbc_defaults, gv70_controller
from opendbc.car.hyundai.values import CAR, HyundaiSafetyFlags

ButtonType = structs.CarState.ButtonEvent.Type


class FakeCAN:
  CAM = 2


class TestAxisOwner(unittest.TestCase):
  def test_silent_until_active(self):
    o = AxisOwner()
    self.assertFalse(any(o.update(False, False, f) for f in range(500)))

  def test_keeps_sending_off_frames_without_driver_action(self):
    o = AxisOwner()
    self.assertTrue(o.update(True, False, 0))
    # openpilot disengages on its own: the camera stays blocked in the panda, so keep the car supplied indefinitely
    self.assertTrue(all(o.update(False, False, f) for f in range(1, 10 * TAIL_FRAMES, 50)))

  def test_driver_action_ends_after_tail(self):
    o = AxisOwner()
    o.update(True, False, 0)
    self.assertTrue(o.update(False, True, 100))  # brake / cancel: still sends for the tail (panda permission lingers ~3 s)
    self.assertTrue(o.update(False, False, 100 + TAIL_FRAMES))
    self.assertFalse(o.update(False, False, 100 + TAIL_FRAMES + 1))
    self.assertFalse(o.update(False, False, 100 + TAIL_FRAMES + 2))

  def test_action_just_before_inactive_counts(self):
    o = AxisOwner()
    o.update(True, False, 0)
    o.update(True, True, 100)  # brake while still active (MADS keeps steering)
    self.assertTrue(o.update(True, False, 150))
    self.assertTrue(o.update(False, False, 100 + RECENT_ACTION_FRAMES))
    self.assertFalse(o.update(False, False, 100 + RECENT_ACTION_FRAMES + TAIL_FRAMES + 1))

  def test_stale_action_does_not_count(self):
    o = AxisOwner()
    o.update(True, False, 0)
    o.update(True, True, 10)
    self.assertTrue(o.update(True, False, 10 + RECENT_ACTION_FRAMES + 5))
    self.assertTrue(o.update(False, False, 10 + RECENT_ACTION_FRAMES + 6))
    self.assertTrue(o.update(False, False, 10 + RECENT_ACTION_FRAMES + 6 + 2 * TAIL_FRAMES))

  def test_reengage_cancels_release(self):
    o = AxisOwner()
    o.update(True, False, 0)
    o.update(False, True, 10)
    self.assertTrue(o.update(True, False, 20))
    self.assertTrue(o.update(False, False, 20 + 2 * TAIL_FRAMES))


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
    for f in range(5, 5 + 160):  # no blink within 1.5 s: the press changed nothing, the camera's LFA is still on
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
    parsers = cs.get_can_parsers(cp, cp_sp)
    for t in (10_000_000, 20_000_000):  # the first update registers the messages
      parsers[Bus.cam].update([(t, [(0x12A, f12a, 2), (0x161, f161, 2)])])
      cs.update(parsers)
    self.assertEqual((cs.cam_lfa_icon, cs.cam_lka_ind), (2, 3))


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


class TestMdpsEcho(unittest.TestCase):
  def _out(self, mdps, camera_aci):
    addr, dat, bus = hyundaicanfd.create_ccnc_mdps(CANPacker(DBC_NAME), FakeCAN, mdps, camera_aci)
    parser = CANParser(DBC_NAME, [("MDPS", 0)], bus)
    parser.update([(0, [(addr, dat, bus)])])
    return bus, parser.vl["MDPS"]

  def test_echoes_camera_request(self):
    real = {"MDPS_ADASAciActvSta_Lv2": 1, "MDPS_EstStrAnglVal": 12.5, "MDPS_ADAS_AciFltSig_Lv2": 4, "MDPS_LkaToiActvSta": 1}
    for camera, expected in ((2, 2), (1, 1), (0, 1)):
      with self.subTest(camera=camera):
        bus, out = self._out({**real, "MDPS_ADASAciActvSta_Lv2": 2 if camera != 2 else 1}, camera)
        self.assertEqual(bus, 2)
        self.assertEqual(out["MDPS_ADASAciActvSta_Lv2"], expected)
        self.assertAlmostEqual(out["MDPS_EstStrAnglVal"], 12.5, places=1)  # everything else is the real MDPS
        self.assertEqual(out["MDPS_ADAS_AciFltSig_Lv2"], 4)
        self.assertEqual(out["MDPS_LkaToiActvSta"], 1)


class TestCarControllerHandback(unittest.TestCase):
  def setUp(self):
    self.cp, self.cp_sp, self.cc, self.cs = gv70_controller()
    self.packer = CANPacker(DBC_NAME)
    self.cs.mdps = {"MDPS_ADASAciActvSta_Lv2": 1}
    self.cs.cam_lfa_alt = {"COUNTER": 100, "ADAS_ActvACILvl2Sta": 2}
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0}
    self.cs.out = structs.CarState()
    self.cc.frame = 2
    self.cc.tuning = SimpleNamespace(stopping=False, actual_accel=0.0, jerk_lower=0.5, jerk_upper=0.5)
    self.hud = structs.CarControl.HUDControl()

  def _msgs(self, steer_req=False, enabled=False, brake=False, buttons=(), frame=None, lat_active=None):
    if frame is not None:
      self.cc.frame = frame
    if self.cc.frame % 5 == 0:
      self.cc.frame += 2  # off the icon cadence (needs real CCNC frames)
    self.cs.out.brakePressed = brake
    self.cs.out.buttonEvents = [structs.CarState.ButtonEvent(type=t, pressed=True) for t in buttons]
    cc = structs.CarControl()
    cc.enabled = enabled
    cc.latActive = steer_req if lat_active is None else lat_active
    msgs = self.cc.create_canfd_msgs(steer_req, 0, 0.0, 0.0, False, self.hud, self.cs, cc)
    self.cc.frame += 2
    self._last_cb = next((m[1] for m in msgs if m[0] == 0xCB), None)
    self._last_scc = next((m[1] for m in msgs if m[0] == 0x1A0), None)
    return {m[0] for m in msgs}

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

  def test_lka_nudge_not_allowed_while_mads_on_or_paused(self):
    self._msgs(steer_req=True)
    self._nudge(0, 3)
    sent = self._msgs(steer_req=False, lat_active=True)  # MADS on, request paused by a takeover
    self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})
    sent = self._msgs(steer_req=True, lat_active=True)
    self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})

  def test_openpilot_owns_lateral_from_boot(self):
    self.assertEqual(self._nudge(2, 3) & {0xCB, 0xEA}, {0xCB, 0xEA})  # the car's LFA (icon 2): ours replaces it, also with a lane departure
    self.assertEqual(self._nudge(2, 2) & {0xCB, 0xEA}, {0xCB, 0xEA})

  def test_lka_nudge_from_boot_sends_nothing(self):
    self.assertFalse(self._nudge(0, 3) & {0xCB, 0xEA})  # MADS never used, the car's own nudge may steer

  def test_flag_set(self):
    self.assertTrue(self.cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CAMERA_HANDBACK)
    self.assertTrue(self.cc.handback)

  def test_only_lateral_frames_when_not_controlling(self):
    for frame in (f for f in range(2, 40, 2) if f % 5):  # off the 5-frame icon cadence (needs real CCNC frames)
      sent = self._msgs(frame=frame)
      self.assertEqual(sent & {0xCB, 0xEA, 0x1A0}, {0xCB, 0xEA})  # inactive lateral from boot, longitudinal stays with the camera

  def test_lateral_and_mdps_only_while_lat_active_then_off_frames(self):
    sent = self._msgs(steer_req=True)
    self.assertEqual(sent & {0xCB, 0xEA, 0x1A0}, {0xCB, 0xEA})
    sent = self._msgs(steer_req=False)  # inactive: still sending 'off' frames
    self.assertEqual(sent & {0xCB, 0xEA, 0x1A0}, {0xCB, 0xEA})

  def test_long_only_while_enabled(self):
    sent = self._msgs(enabled=True)
    self.assertIn(0x1A0, sent)
    sent = self._msgs(enabled=False)  # disengaged by openpilot alone: 'off' frames continue
    self.assertIn(0x1A0, sent)

  def test_brake_ends_longitudinal_tail_only(self):
    self._msgs(steer_req=True, enabled=True)
    self._msgs(steer_req=False, enabled=False, brake=True)
    self._msgs(brake=False, frame=2 + TAIL_FRAMES + 10)
    sent = self._msgs()
    self.assertNotIn(0x1A0, sent)
    self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})  # steering is not handed back by a brake or a takeover

  def test_cancel_and_main_do_not_end_lateral(self):
    self._msgs(steer_req=True, enabled=True)
    self._msgs(steer_req=False, enabled=False, buttons=(ButtonType.cancel, ButtonType.mainCruise))
    self._msgs(frame=2 + TAIL_FRAMES + 10)
    sent = self._msgs()
    self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})

  def test_lfa_button_never_ends_lateral(self):
    # the panda keeps lateral for good, and so does openpilot: inactive frames only, never the camera's
    self._msgs(steer_req=True, enabled=True)
    self._msgs(steer_req=False, enabled=False, buttons=(ButtonType.lkas,))
    self._msgs(frame=2 + 10 * TAIL_FRAMES)
    for _ in range(5):
      sent = self._msgs()
      self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})

  def test_takeover_pause_sends_inactive_frames_and_takes_axis(self):
    # LFA on while the driver pushes hard: MADS allows steering (latActive) but the takeover filter sends no request; the first frame is inactive
    sent = self._msgs(lat_active=True, steer_req=False)
    self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})
    dat = self._last_cb
    parser = CANParser(DBC_NAME, [("LFA_ALT", 0)], 0)
    parser.update([(0, [(0xCB, dat, 0)])])
    self.assertEqual(parser.vl["LFA_ALT"]["ADAS_ActvACILvl2Sta"], 1)  # inactive
    self.assertEqual(parser.vl["LFA_ALT"]["ADAS_ACIAnglTqRedcGainVal"], 0)

  def test_inactive_frames_from_boot_while_mads_never_used(self):
    for _ in range(10):
      sent = self._msgs(lat_active=False)
      self.assertEqual(sent & {0xCB, 0xEA}, {0xCB, 0xEA})

  def _lfa_icon(self, buttons=()):
    """the LFA_ICON of the 0x161 frame we send (the icon cadence needs frame % 5 == 0 and real CCNC frames)"""
    self.cc.frame = (self.cc.frame // 5 + 1) * 5
    self.cs.out.buttonEvents = [structs.CarState.ButtonEvent(type=t, pressed=True) for t in buttons]
    cc = structs.CarControl()
    msgs = self.cc.create_canfd_msgs(False, 0, 0.0, 0.0, False, self.hud, self.cs, cc)
    self.cc.frame += 1
    parser = CANParser(DBC_NAME, [("CCNC_0x161", 0)], 0)
    parser.update([(0, [(m[0], m[1], 0) for m in msgs if m[0] == 0x161])])
    return parser.vl["CCNC_0x161"]["LFA_ICON"]

  def test_icon_off_at_the_lfa_off_press_not_after_the_blink(self):
    msg_161 = dbc_defaults("CCNC_0x161")
    self.cs.msg_161, self.cs.msg_162, self.cs.msg_1b5 = msg_161, dict(dbc_defaults("CCNC_0x162")), dict(dbc_defaults("FR_CMR_03_50ms"))
    self.cs.cam_lfa_icon, self.cs.cam_lka_ind = 2, 1
    for _ in range(4):
      self.assertEqual(1, self._lfa_icon())  # MADS off, the car's LFA on and green: ours grey (the real state of the car's LFA)
    self.assertEqual(0, self._lfa_icon((ButtonType.lkas,)))  # LFA button: ours is off at once
    self.cs.cam_lfa_icon = 3  # the camera blinks, then settles at 0
    for icon in (3, 0, 3, 0):
      self.cs.cam_lfa_icon = icon
      self.assertEqual(0, self._lfa_icon())

  def _cruise_icon(self, enabled=False):
    """(SETSPEED, SETSPEED_HUD, DISTANCE, SETSPEED_SPEED) of the 0x161 frame we send, main cruise on"""
    self.cs.main_cruise_enabled = True
    self.cs.out.cruiseState.available = True
    self.cc.frame = (self.cc.frame // 5 + 1) * 5
    self.cs.out.buttonEvents = []
    cc = structs.CarControl()
    cc.enabled = enabled
    cc.longActive = enabled
    msgs = self.cc.create_canfd_msgs(False, 0, 0.0, 0.0, False, self.hud, self.cs, cc)
    self.cc.frame += 1
    parser = CANParser(DBC_NAME, [("CCNC_0x161", 0)], 0)
    parser.update([(0, [(m[0], m[1], 0) for m in msgs if m[0] == 0x161])])
    v = parser.vl["CCNC_0x161"]
    return v["SETSPEED"], v["SETSPEED_HUD"], v["DISTANCE"], v["SETSPEED_SPEED"]

  def _dash_fixture(self):
    self.cs.msg_161, self.cs.msg_162, self.cs.msg_1b5 = dbc_defaults("CCNC_0x161"), dbc_defaults("CCNC_0x162"), dbc_defaults("FR_CMR_03_50ms")
    self.cs.cam_lfa_icon, self.cs.cam_lka_ind = 0, 1
    self.cs.out.vCruiseCluster = 40.0

  def test_cruise_icon_off_during_tail_and_stock_lock(self):
    # route 34 17:43.4 / 18:05.2: after a brake disengage and while the stock ACC is locked out we send 0x1A0 'off' frames; the cruise icon is off then
    self._dash_fixture()
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0, "ACCMode": 4}
    self.cs.msg_161.update({"SETSPEED": 1, "SETSPEED_HUD": 1, "DISTANCE": 4, "SETSPEED_SPEED": 35})
    self.assertEqual(self._cruise_icon(), (1, 1, 4, 35))  # the camera owns the axis: its own standby look is passed through
    self._msgs(steer_req=True, enabled=True)  # openpilot engaged and owner
    self._msgs(steer_req=False, enabled=False, brake=True)  # brake: disengaged, the tail starts
    self.assertEqual(self._cruise_icon(), (0, 0, 0, 255))
    self.cs.cruise_info["ACCMode"] = 1  # camera ACC engaged on its own (lock): off as well
    self.cc.handback.long_owner.owner = False
    self.cc.handback.long_owner.release_frame = None
    self.assertEqual(self._cruise_icon(), (0, 0, 0, 255))
    self.assertEqual(self._cruise_icon(enabled=True)[:2], (2, 2))  # engaged by openpilot: green

  def test_stock_acc_lock_sends_off_frames(self):
    # camera ACC engaged (ACCMode 1) while openpilot is not: 0x1A0 'off' frames, also if openpilot never owned the axis
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0, "ACCMode": 1}
    sent = self._msgs(enabled=False)
    self.assertIn(0x1A0, sent)
    dat = self._last_scc
    parser = CANParser(DBC_NAME, [("SCC_CONTROL", 0)], 0)
    parser.update([(0, [(0x1A0, dat, 0)])])
    self.assertEqual(parser.vl["SCC_CONTROL"]["ACCMode"], 0)
    self.assertEqual(parser.vl["SCC_CONTROL"]["aReqValue"], 0)
    self.cs.cruise_info["ACCMode"] = 4  # camera back in standby: silent again
    self.assertNotIn(0x1A0, self._msgs(enabled=False, frame=2 + 5000))

  def _step(self, event=None, available=True):
    """one controller frame (off the icon cadence), with an optional CANCEL press (True) / release (False) event; returns (sent addresses, 0x1A0 data)"""
    if self.cc.frame % 5 == 0:
      self.cc.frame += 2
    self.cs.out.cruiseState.available = available
    self.cs.out.buttonEvents = [] if event is None else [structs.CarState.ButtonEvent(type=ButtonType.cancel, pressed=event)]
    cc = structs.CarControl()
    cc.enabled = False
    msgs = self.cc.create_canfd_msgs(False, 0, 0.0, 0.0, False, self.hud, self.cs, cc)
    self.cc.frame += 2
    return {m[0] for m in msgs}, next((m[1] for m in msgs if m[0] == 0x1A0), None)

  def test_cancel_press_in_standby_sends_off_frames_until_settled(self):
    # the panda blocks the camera's 0x1A0 from the CANCEL press (the camera resumes its ACC on the release): our off frames must replace it from the
    # press on, while the key is down and until 0.8 s after the release; then silent again (the camera is back to standby)
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0, "ACCMode": 4}
    self.assertNotIn(0x1A0, self._step()[0])
    sent, dat = self._step(True)
    self.assertIn(0x1A0, sent)
    parser = CANParser(DBC_NAME, [("SCC_CONTROL", 0)], 0)
    parser.update([(0, [(0x1A0, dat, 0)])])
    self.assertEqual(parser.vl["SCC_CONTROL"]["ACCMode"], 0)  # an 'off' frame, nothing engages
    self.assertEqual(parser.vl["SCC_CONTROL"]["aReqValue"], 0)
    for _ in range(40):  # the key is held 1.6 s
      self.assertIn(0x1A0, self._step()[0])
    self.assertIn(0x1A0, self._step(False)[0])
    for _ in range(25):  # about 0.6 s (the cadence skips add frames): the camera resumes in this window
      self.assertIn(0x1A0, self._step()[0])
    for _ in range(30):  # the 0.8 s settle (80 frames) is over
      sent = self._step()[0]
    self.assertNotIn(0x1A0, sent)  # settled

  def test_cancel_press_without_camera_main_or_while_owner_sends_nothing(self):
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0, "ACCMode": 4}
    self.assertNotIn(0x1A0, self._step(True, available=False)[0])  # camera main off: the panda does not arm either
    self._step(False)
    self._msgs(enabled=True)  # openpilot owns the axis: the press is its own cancel, no lock bookkeeping
    self.assertTrue(self.cc.handback.long_owner.owner)
    self.cs.out.cruiseState.available = True
    self._msgs(enabled=False, buttons=(ButtonType.cancel,))
    self.assertFalse(self.cc.handback.cancel_armed)

  def test_stock_acc_not_locked_while_openpilot_engaged(self):
    self.cs.cruise_info = {"COUNTER": 50, "ACC_ObjDist": 0, "ACC_ObjRelSpd": 0, "ACCMode": 1}
    self._msgs(enabled=True)
    dat = self._last_scc
    parser = CANParser(DBC_NAME, [("SCC_CONTROL", 0)], 0)
    parser.update([(0, [(0x1A0, dat, 0)])])
    self.assertEqual(parser.vl["SCC_CONTROL"]["ACCMode"], 1)

  def test_counter_follows_camera(self):
    msgs = self.cc.create_canfd_msgs(True, 0, 0.0, 0.0, False, self.hud, self.cs, SimpleNamespace(
      enabled=True, latActive=True, cruiseControl=SimpleNamespace(override=False), hudControl=self.hud, leftBlinker=False, rightBlinker=False))
    by_addr = {m[0]: m[1] for m in msgs}
    parser = CANParser(DBC_NAME, [("LFA_ALT", 0), ("SCC_CONTROL", 0)], 0)
    parser.update([(0, [(0xCB, by_addr[0xCB], 0), (0x1A0, by_addr[0x1A0], 0)])])
    self.assertEqual(parser.vl["LFA_ALT"]["COUNTER"], 100)
    self.assertEqual(parser.vl["SCC_CONTROL"]["COUNTER"], 50)

  def test_mdps_echo_value(self):
    msgs = self.cc.create_canfd_msgs(True, 0, 0.0, 0.0, False, self.hud, self.cs, SimpleNamespace(
      enabled=False, latActive=True, cruiseControl=SimpleNamespace(override=False), hudControl=self.hud, leftBlinker=False, rightBlinker=False))
    dat = next(m[1] for m in msgs if m[0] == 0xEA)
    parser = CANParser(DBC_NAME, [("MDPS", 0)], 2)
    parser.update([(0, [(0xEA, dat, 2)])])
    self.assertEqual(parser.vl["MDPS"]["MDPS_ADASAciActvSta_Lv2"], 2)  # camera asks for ACI: echoed
    self.cs.cam_lfa_alt = {"COUNTER": 101, "ADAS_ActvACILvl2Sta": 1}
    msgs = self.cc.create_canfd_msgs(True, 0, 0.0, 0.0, False, self.hud, self.cs, SimpleNamespace(
      enabled=False, latActive=True, cruiseControl=SimpleNamespace(override=False), hudControl=self.hud, leftBlinker=False, rightBlinker=False))
    dat = next(m[1] for m in msgs if m[0] == 0xEA)
    parser.update([(0, [(0xEA, dat, 2)])])
    self.assertEqual(parser.vl["MDPS"]["MDPS_ADASAciActvSta_Lv2"], 1)


class TestOtherConfigsUnchanged(unittest.TestCase):
  def test_flag_only_for_gv70_ccnc_lfa_alt(self):
    for candidate in (CAR.KIA_EV6, CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_SANTA_FE_HEV_5TH_GEN, CAR.GENESIS_GV70_1ST_GEN):
      fingerprint = gen_empty_fingerprint()
      fingerprint[CanBus(None, fingerprint).CAM] = {0xCB: 24}
      cp = CarInterface.get_params(candidate, fingerprint, [], True, False, False)
      self.assertFalse(cp.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CAMERA_HANDBACK, candidate)

  def test_legacy_mdps_copy_without_flag(self):
    addr, dat, bus = hyundaicanfd.create_ccnc_mdps(CANPacker(DBC_NAME), FakeCAN, {"MDPS_ADASAciActvSta_Lv2": 2})
    parser = CANParser(DBC_NAME, [("MDPS", 0)], bus)
    parser.update([(0, [(addr, dat, bus)])])
    self.assertEqual(parser.vl["MDPS"]["MDPS_ADASAciActvSta_Lv2"], 1)


if __name__ == "__main__":
  unittest.main()
