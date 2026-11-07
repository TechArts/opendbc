import unittest

from opendbc.can import CANPacker
from opendbc.car import CanData, Bus, gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import MRR35_START_ADDR, MRR35_MSG_COUNT
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags

RADAR_BUS = 1


class TestMrr35RadarInterface(unittest.TestCase):
  def setUp(self):
    fp = gen_empty_fingerprint()
    self.CP = CarInterface.get_params(CAR.GENESIS_GV70_2ND_GEN, fp, [], False, False, False)
    self.CP_SP = CarInterface.get_params_sp(self.CP, CAR.GENESIS_GV70_2ND_GEN, fp, [], False, False, False)
    self.radar = CarInterface.RadarInterface(self.CP, self.CP_SP)
    self.packer = CANPacker(DBC[CAR.GENESIS_GV70_2ND_GEN][Bus.radar])
    self.counter = 0

  def _cycle(self, tracks: dict[int, dict]):
    frames = []
    for i in range(MRR35_MSG_COUNT):
      addr = MRR35_START_ADDR + i
      values = {"COUNTER": self.counter, **tracks.get(i, {"STATE": 0})}
      frames.append(CanData(*self.packer.make_can_msg(f"RADAR_TRACK_{addr:x}", RADAR_BUS, values)))
    self.counter = (self.counter + 1) % 256
    return self.radar.update([(int(self.counter * 5e7), frames)])

  def test_platform_uses_radar_tracks(self):
    assert self.CP.flags & HyundaiFlags.CANFD_MRR35_RADAR
    assert not self.CP.radarUnavailable
    assert self.radar.rcp.bus == RADAR_BUS

  def test_only_measured_and_coasted_tracks_are_published(self):
    base = {"LONG_DIST": 30.0, "LAT_DIST": 1.0, "REL_SPEED": -2.0}
    tracks = {0: {"STATE": 3, **base}, 1: {"STATE": 4, **base}, 2: {"STATE": 2, **base}, 3: {"STATE": 1, **base}, 4: {"STATE": 7, **base}}
    for _ in range(5):
      rr = self._cycle(tracks)
    assert rr is not None and not rr.errors.canError
    assert len(rr.points) == 2
    assert sorted(p.measured for p in rr.points) == [False, True]  # coasted track is not "measured"
    p = rr.points[0]
    assert abs(p.dRel - 30.0) < 0.1 and abs(p.yRel - 1.0) < 0.1 and abs(p.vRel + 2.0) < 0.02

  def test_track_id_is_stable_and_dropped_when_empty(self):
    track = {0: {"STATE": 3, "LONG_DIST": 20.0}}
    for _ in range(3):
      rr = self._cycle(track)
    tid = rr.points[0].trackId
    rr = self._cycle(track)
    assert rr.points[0].trackId == tid
    rr = self._cycle({})
    assert len(rr.points) == 0


if __name__ == "__main__":
  unittest.main()
