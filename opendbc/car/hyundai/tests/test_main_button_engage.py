import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiSafetyFlags

ButtonType = structs.CarState.ButtonEvent.Type
FLAG = HyundaiSafetyFlags.LONG_MAIN_BUTTON_ENGAGE


def _cp(candidate, alpha_long, lfa_alt=True):
  fingerprint = gen_empty_fingerprint()
  can = CanBus(None, fingerprint)
  fingerprint[can.ECAN] = {0x1CF: 8}
  if lfa_alt:
    fingerprint[can.CAM] = {0xCB: 24}
  return CarInterface.get_params(candidate, fingerprint, [], alpha_long, False, False)


def _flag(cp):
  return bool(cp.safetyConfigs[-1].safetyParam & FLAG)


class TestMainButtonEngage(unittest.TestCase):
  def test_flag_only_for_long_ccnc_lfa_alt(self):
    gv70_long = _cp(CAR.GENESIS_GV70_2ND_GEN, True)
    assert gv70_long.openpilotLongitudinalControl
    assert gv70_long.safetyConfigs[-1].safetyParam == 7180 | FLAG

    # stock longitudinal, no LFA_ALT, other CAN FD cars: unchanged
    assert not _flag(_cp(CAR.GENESIS_GV70_2ND_GEN, False))
    assert _cp(CAR.GENESIS_GV70_2ND_GEN, False).safetyConfigs[-1].safetyParam == 7176
    assert not _flag(_cp(CAR.GENESIS_GV70_2ND_GEN, True, lfa_alt=False))
    for candidate in (CAR.KIA_EV6, CAR.HYUNDAI_IONIQ_5, CAR.HYUNDAI_SONATA_HEV_2024):
      assert not _flag(_cp(candidate, True))

  def _carstate(self, alpha_long=True):
    cp = _cp(CAR.GENESIS_GV70_2ND_GEN, alpha_long)
    cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.GENESIS_GV70_2ND_GEN)
    return CarState(cp, cp_sp)

  @staticmethod
  def _main_event(pressed):
    return [structs.CarState.ButtonEvent(type=ButtonType.mainCruise, pressed=pressed)]

  def test_carstate_gate(self):
    assert self._carstate().long_main_button_engage
    assert not self._carstate(alpha_long=False).long_main_button_engage

  def test_enable_request_logic(self):
    # mirrors update_canfd(): main_button_enable = gate and cruiseState.available and a main press event
    cs = self._carstate()
    assert not cs.update_button_enable(self._main_event(True))  # not computed by update_canfd yet
    cs.main_button_enable = True
    assert cs.update_button_enable([])
    cs.main_button_enable = False
    assert not cs.update_button_enable(self._main_event(False))

  def test_set_resume_enable_unchanged(self):
    cs = self._carstate()
    for btn, expected in ((ButtonType.accelCruise, True), (ButtonType.decelCruise, True), (ButtonType.setCruise, False)):
      assert cs.update_button_enable([structs.CarState.ButtonEvent(type=btn, pressed=False)]) == expected
    assert not cs.update_button_enable([structs.CarState.ButtonEvent(type=ButtonType.accelCruise, pressed=True)])
