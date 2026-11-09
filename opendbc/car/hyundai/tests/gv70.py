"""Shared fixtures for the GV70 (CCNC LFA_ALT camera SCC) car tests."""
from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC

DBC_NAME = "hyundai_canfd_generated"


def dbc_defaults(msg):
  return dict(CANParser(DBC_NAME, [(msg, 0)], 0).vl[msg])


def car_params(candidate=CAR.GENESIS_GV70_2ND_GEN, alpha_long=True, lfa_alt=True):
  fingerprint = gen_empty_fingerprint()
  can = CanBus(None, fingerprint)
  fingerprint[can.ECAN] = {0x1CF: 8}
  if lfa_alt:
    fingerprint[can.CAM] = {0xCB: 24}
  return CarInterface.get_params(candidate, fingerprint, [], alpha_long, False, False)


def gv70_controller(alpha_long=True, tweak_sp=None):
  """(CP, CP_SP, CarController, CarState); tweak_sp(CP_SP) may change the params before the controller is built."""
  cp = car_params(alpha_long=alpha_long)
  cp_sp = CarInterface.get_non_essential_params_sp(cp, CAR.GENESIS_GV70_2ND_GEN)
  if tweak_sp is not None:
    tweak_sp(cp_sp)
  return cp, cp_sp, CarController(DBC[cp.carFingerprint], cp, cp_sp), CarState(cp, cp_sp)
