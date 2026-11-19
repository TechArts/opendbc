import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai.carstate import CarState, MAIN_PRESS_FRAMES
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.sunnypilot.car.hyundai.values import HyundaiSafetyFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type
FLAG = HyundaiSafetyFlagsSP.MAIN_BUTTON_ENGAGE


def _cp(candidate, alpha_long, lfa_alt=True, ccnc=True):
  fingerprint = gen_empty_fingerprint()
  can = CanBus(None, fingerprint)
  fingerprint[can.ECAN] = {0x1CF: 8}
  fingerprint[can.CAM] = {}
  if lfa_alt:
    fingerprint[can.CAM][0xCB] = 24
  if ccnc:
    fingerprint[can.CAM].update({0x161: 32, 0x162: 32})
  cp = CarInterface.get_params(candidate, fingerprint, [], alpha_long, False, False)
  return cp, CarInterface.get_params_sp(cp, candidate, fingerprint, [], alpha_long, False, False)


def _flag(cp_sp):
  return bool(cp_sp.safetyParam & FLAG)


class TestMainButtonEngage(unittest.TestCase):
  def test_flag_only_for_long_ccnc_lfa_alt(self):
    cp, cp_sp = _cp(CAR.GENESIS_GV70_2ND_GEN, True)
    assert cp.openpilotLongitudinalControl
    assert _flag(cp_sp)

    # stock longitudinal, no LFA_ALT, no CCNC: unchanged
    assert not _flag(_cp(CAR.GENESIS_GV70_2ND_GEN, False)[1])
    assert not _flag(_cp(CAR.GENESIS_GV70_2ND_GEN, True, lfa_alt=False)[1])
    assert not _flag(_cp(CAR.GENESIS_GV70_2ND_GEN, True, ccnc=False)[1])
    for candidate in (CAR.KIA_EV6, CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_SONATA_2024):
      assert not _flag(_cp(candidate, True, lfa_alt=False, ccnc=False)[1])

  def test_flag_is_the_reserved_bit(self):
    assert FLAG == 32
    _, cp_sp = _cp(CAR.GENESIS_GV70_2ND_GEN, True)
    assert cp_sp.safetyParam & FLAG
    assert not _cp(CAR.GENESIS_GV70_2ND_GEN, False)[1].safetyParam & FLAG

  def _carstate(self, alpha_long=True):
    cp, cp_sp = _cp(CAR.GENESIS_GV70_2ND_GEN, alpha_long)
    return CarState(cp, cp_sp)

  def test_carstate_gate(self):
    assert self._carstate().long_main_button_engage
    assert not self._carstate(alpha_long=False).long_main_button_engage

  def _update(self, alpha_long, camera_main, main_btn):
    cp, cp_sp = _cp(CAR.GENESIS_GV70_2ND_GEN, alpha_long)
    cs = CarState(cp, cp_sp)
    parsers = cs.get_can_parsers(cp, cp_sp)
    packer = CANPacker(DBC[CAR.GENESIS_GV70_2ND_GEN][Bus.pt])
    can = CanBus(cp)
    ret, engage = None, False
    for i in range(3):
      ts = (i + 1) * 10_000_000
      parsers[Bus.pt].update([(ts, [packer.make_can_msg("CRUISE_BUTTONS", can.ECAN, {"ADAPTIVE_CRUISE_MAIN_BTN": main_btn})])])
      parsers[Bus.cam].update([(ts, [packer.make_can_msg("SCC_CONTROL", can.CAM, {"MainMode_ACC": camera_main})])])
      ret, _ = cs.update(parsers)
      engage = engage or cs.main_button_enable
    return cs, ret, engage

  def test_camera_main_is_the_cruise_available_state(self):
    cs, ret, engage = self._update(True, camera_main=1, main_btn=1)
    assert ret.cruiseState.available and cs.main_cruise_enabled
    assert engage  # the camera's main rose with a main press
    _, ret, engage = self._update(True, camera_main=0, main_btn=0)
    assert not ret.cruiseState.available and not engage

  def test_camera_main_without_press_does_not_engage(self):
    _, ret, engage = self._update(True, camera_main=1, main_btn=0)
    assert not ret.cruiseState.available  # raw camera availability is not the panda's driver-qualified main
    assert not engage

  def test_unpaired_camera_rise_then_pause_release_gives_no_enable(self):
    cp, cp_sp = _cp(CAR.GENESIS_GV70_2ND_GEN, True)
    cs = CarState(cp, cp_sp)
    parsers = cs.get_can_parsers(cp, cp_sp)
    packer = CANPacker(DBC[CAR.GENESIS_GV70_2ND_GEN][Bus.pt])
    can = CanBus(cp)

    def frame(i, camera_main, main_btn, cruise_btn):
      ts = (i + 1) * 10_000_000
      parsers[Bus.pt].update([(ts, [packer.make_can_msg("CRUISE_BUTTONS", can.ECAN, {"ADAPTIVE_CRUISE_MAIN_BTN": main_btn, "CRUISE_BUTTONS": cruise_btn})])])
      parsers[Bus.cam].update([(ts, [packer.make_can_msg("SCC_CONTROL", can.CAM, {"MainMode_ACC": camera_main})])])
      ret, _ = cs.update(parsers)
      return ret, cs.update_button_enable(ret.buttonEvents)

    for i in range(3):
      ret, enable = frame(i, 1, 0, 0)  # the camera's main rises by itself
    assert not ret.cruiseState.available
    ret, enable = frame(3, 1, 0, 4)  # pause key down
    assert not ret.cruiseState.available
    ret, enable = frame(4, 1, 0, 0)  # and released
    assert not ret.cruiseState.available and not enable

  def test_paired_press_publishes_available(self):
    _, ret, engage = self._update(True, camera_main=1, main_btn=1)
    assert engage and ret.cruiseState.available

  def test_stock_long_unaffected(self):
    _, _, engage = self._update(False, camera_main=1, main_btn=1)
    assert not engage


class TestCameraMain(unittest.TestCase):
  """The camera's MainMode_ACC is the main cruise state; a main press only pairs with its rising edge into an engage request."""

  def setUp(self):
    cp, cp_sp = _cp(CAR.GENESIS_GV70_2ND_GEN, True)
    self.cs = CarState(cp, cp_sp)

  def _step(self, camera_main, pressed=False, frames=1):
    out = False
    for _ in range(frames):
      out = self.cs.update_camera_main(camera_main, pressed)
    return out

  def test_press_then_camera_rises_engages(self):
    self.assertFalse(self._step(False, pressed=True))
    self.assertFalse(self._step(False, frames=10))
    self.assertTrue(self._step(True))
    self.assertFalse(self._step(True, frames=5))  # one request per edge

  def test_camera_rises_then_press_engages(self):
    self.assertFalse(self._step(True))
    self.assertFalse(self._step(True, frames=10))
    self.assertTrue(self._step(True, pressed=True))

  def test_same_frame_engages(self):
    self.assertTrue(self._step(True, pressed=True))

  def test_camera_ignores_press(self):
    self._step(False, pressed=True)
    self.assertFalse(self._step(False, frames=5 * MAIN_PRESS_FRAMES))
    self.assertFalse(self.cs.camera_main)

  def test_camera_main_without_press_does_not_engage(self):
    self.assertFalse(self._step(True, frames=5))
    self.assertTrue(self.cs.camera_main)
    self._step(True, frames=MAIN_PRESS_FRAMES + 5)
    self.assertFalse(self._step(True, pressed=True))  # a press long after the rise has nothing to pair with

  def test_stale_press_does_not_engage(self):
    self._step(False, pressed=True)
    self._step(False, frames=MAIN_PRESS_FRAMES + 5)
    self.assertFalse(self._step(True))

  def test_falling_edge_is_available_false_and_no_engage(self):
    self._step(True, pressed=True)
    self.assertFalse(self._step(False, pressed=True))
    self.assertFalse(self.cs.camera_main)

  def test_press_then_cancel_then_late_camera_rise_engages_nothing(self):
    self._step(False, pressed=True)
    self.assertFalse(self.cs.update_camera_main(False, False, cancel_pressed=True))
    self.assertFalse(self._step(True))  # the late answer
    self.assertFalse(self.cs.main_qualified)

  def test_cancel_with_main_press_does_not_pair(self):
    self.assertFalse(self.cs.update_camera_main(True, True, cancel_pressed=True))

  def test_pairing_is_single_use(self):
    self.assertTrue(self._step(True, pressed=True))
    self.assertFalse(self._step(False))
    self.assertFalse(self._step(True))  # fall and rise inside the window: the old press is spent
    self.assertFalse(self.cs.main_qualified)

  def test_camera_first_pairing_is_single_use(self):
    self.assertFalse(self._step(True))
    self.assertTrue(self._step(True, pressed=True))
    self.assertFalse(self._step(False))
    self.assertFalse(self._step(True))

  def test_camera_only_rise_is_not_a_qualified_main(self):
    self._step(True, frames=5)
    self.assertFalse(self.cs.main_qualified)
    set_release = [structs.CarState.ButtonEvent(type=ButtonType.decelCruise, pressed=False)]
    self.assertFalse(self.cs.update_button_enable(set_release))

  def test_unknown_camera_state_keeps_last(self):
    self._step(True, pressed=True)
    self.assertFalse(self._step(None, frames=5))
    self.assertTrue(self.cs.camera_main)

  def test_set_res_needs_camera_main(self):
    set_release = [structs.CarState.ButtonEvent(type=ButtonType.decelCruise, pressed=False)]
    self.cs.main_qualified = True
    self.assertTrue(self.cs.update_button_enable(set_release))
    self.assertFalse(self.cs.update_button_enable([structs.CarState.ButtonEvent(type=ButtonType.accelCruise, pressed=True)]))
    self.cs.main_qualified = False
    self.assertFalse(self.cs.update_button_enable(set_release))
    self.cs.main_button_enable = True  # an engage request still goes through
    self.assertTrue(self.cs.update_button_enable([]))


if __name__ == "__main__":
  unittest.main()
