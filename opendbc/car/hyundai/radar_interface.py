import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.values import DBC, HyundaiFlags

from opendbc.sunnypilot.car.hyundai.radar_interface_ext import RadarInterfaceExt

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

# MRR35 radar on CAN FD: 32 tracks in 0x3A5-0x3C4 on ACAN (bus 1), ~20 Hz
MRR35_START_ADDR = 0x3A5
MRR35_MSG_COUNT = 32
MRR35_VALID_STATES = (3, 4)  # measured, coasted

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  if CP.flags & HyundaiFlags.CANFD_MRR35_RADAR:
    messages = [(f"RADAR_TRACK_{addr:x}", 20) for addr in range(MRR35_START_ADDR, MRR35_START_ADDR + MRR35_MSG_COUNT)]
  else:
    messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


class RadarInterface(RadarInterfaceBase, RadarInterfaceExt):
  def __init__(self, CP, CP_SP):
    RadarInterfaceBase.__init__(self, CP, CP_SP)
    RadarInterfaceExt.__init__(self, CP, CP_SP)
    self.updated_messages = set()
    self.mrr35 = bool(CP.flags & HyundaiFlags.CANFD_MRR35_RADAR)
    self.trigger_msg = MRR35_START_ADDR + MRR35_MSG_COUNT - 1 if self.mrr35 else RADAR_START_ADDR + RADAR_MSG_COUNT - 1
    self.track_id = 0

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

    if self.rcp is None:
      self.initialize_radar_ext(self.trigger_msg)

  def update(self, can_strings):
    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    if self.mrr35:
      return self._update_mrr35(ret)

    if self.use_radar_interface_ext:
      return self.update_ext(ret)

    for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[addr].measured = True
        self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']
        self.pts[addr].aRel = msg['REL_ACCEL']
        self.pts[addr].yvRel = float('nan')

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret

  def _update_mrr35(self, ret):
    for addr in range(MRR35_START_ADDR, MRR35_START_ADDR + MRR35_MSG_COUNT):
      msg = self.rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if msg['STATE'] in MRR35_VALID_STATES:
        if addr not in self.pts:
          self.pts[addr] = structs.RadarData.RadarPoint()
          self.pts[addr].trackId = self.track_id
          self.track_id += 1

        self.pts[addr].measured = msg['STATE'] == 3
        self.pts[addr].dRel = msg['LONG_DIST']
        self.pts[addr].yRel = msg['LAT_DIST']  # sign as in sunnypilot ccnc-port-radar-tracks, not verified against a lead
        self.pts[addr].vRel = msg['REL_SPEED']
        self.pts[addr].aRel = msg['REL_ACCEL']
        self.pts[addr].yvRel = msg['REL_LAT_SPEED']
      else:
        self.pts.pop(addr, None)

    ret.points = list(self.pts.values())
    return ret
