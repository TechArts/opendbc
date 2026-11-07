import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.hyundai import hyundaicanfd
from opendbc.sunnypilot.car.hyundai.cluster_alert import ClusterAlertCarController, CLUSTER_ALERT_HOLD_FRAMES

ClusterAlert = structs.CarControlSP.ClusterAlert
DBC = "hyundai_canfd_generated"

FAULT_VALUES = {"FAULT_FSS": 1, "FAULT_FCA": 2, "FAULT_LSS": 1, "FAULT_SCC": 1, "FAULT_LFA": 1, "FAULT_HDA": 2, "FAULT_DAS": 1, "FAULT_ESS": 1}


class FakePacker:
  def make_can_msg(self, name, bus, values):
    return name, dict(values)


class FakeCAN:
  ECAN = 0


CLUSTER_FIELDS = [f"ALERTS_{i}" for i in range(1, 6)] + [f"SOUNDS_{i}" for i in range(1, 5)] + list(FAULT_VALUES)


def stock_frames(**overrides):
  msg_161 = {"ALERTS_1": 0, "ALERTS_2": 0, "ALERTS_3": 0, "ALERTS_4": 0, "ALERTS_5": 0, "SOUNDS_1": 0, "SOUNDS_2": 0, "SOUNDS_3": 0,
             "SOUNDS_4": 0, "LFA_ICON": 0, **FAULT_VALUES}
  msg_162 = {"VIBRATE": 0}
  msg_161.update(overrides)
  return msg_161, msg_162


def run_ccnc(msg_161, msg_162, cluster_alert, long_control=False):
  msg_1b5 = {"Longitudinal_Distance": 0}
  msgs = hyundaicanfd.create_ccnc(FakePacker(), FakeCAN(), long_control, False, structs.CarControl.HUDControl(), False, False, msg_161, msg_162,
                                  msg_1b5, False, structs.CarState(), False, False, cluster_alert)
  return dict(msgs)["CCNC_0x161"], dict(msgs)["CCNC_0x162"]


class TestCcncClusterAlert(unittest.TestCase):
  def test_red_sets_text_chime_vibrate(self):
    out_161, out_162 = run_ccnc(*stock_frames(), ClusterAlert.red)
    self.assertEqual((out_161["ALERTS_3"], out_161["SOUNDS_3"], out_162["VIBRATE"]), (18, 5, 1))
    self.assertEqual((out_161["ALERTS_5"], out_161["SOUNDS_2"]), (0, 0))

  def test_green_sets_text_chime_vibrate(self):
    out_161, out_162 = run_ccnc(*stock_frames(), ClusterAlert.green)
    self.assertEqual((out_161["ALERTS_5"], out_161["SOUNDS_2"], out_162["VIBRATE"]), (11, 2, 1))
    self.assertEqual((out_161["ALERTS_3"], out_161["SOUNDS_3"]), (0, 0))

  def test_none_leaves_frames_untouched(self):
    for alert in (ClusterAlert.none, None):
      out_161, out_162 = run_ccnc(*stock_frames(), alert)
      self.assertEqual((out_161["ALERTS_3"], out_161["ALERTS_5"], out_162["VIBRATE"]), (0, 0, 0))

  def test_stock_alert_wins(self):
    any_alert = [{"ALERTS_1": 1}, {"ALERTS_1": 11}, {"ALERTS_1": 22}, {"ALERTS_2": 7}, {"ALERTS_2": 8}, {"ALERTS_2": 9}, {"ALERTS_2": 14},
                 {"ALERTS_2": 21}]
    cases = {
      ClusterAlert.red: any_alert + [{"ALERTS_3": 20}, {"SOUNDS_3": 3}],
      ClusterAlert.green: any_alert + [{"ALERTS_5": 3}, {"SOUNDS_2": 3}],
    }
    for alert, overrides_list in cases.items():
      for overrides in overrides_list:
        msg_161, msg_162 = stock_frames(**overrides)
        before = {k: msg_161[k] for k in CLUSTER_FIELDS}
        out_161, out_162 = run_ccnc(msg_161, msg_162, alert)
        self.assertEqual({k: out_161[k] for k in CLUSTER_FIELDS}, before, (alert, overrides))
        self.assertEqual(out_162["VIBRATE"], 0, (alert, overrides))

  def test_unrelated_stock_alert_does_not_block(self):
    # hands-on-wheel text lives in ALERTS_2 and is not a collision alert
    out_161, _ = run_ccnc(*stock_frames(ALERTS_2=1), ClusterAlert.red)
    self.assertEqual((out_161["ALERTS_2"], out_161["ALERTS_3"]), (1, 18))

  def test_faults_untouched(self):
    for alert in (ClusterAlert.red, ClusterAlert.green):
      out_161, _ = run_ccnc(*stock_frames(), alert)
      for k, v in FAULT_VALUES.items():
        self.assertEqual(out_161[k], v)

  def test_existing_suppression_frees_target_field(self):
    # with openpilot long the stock ALERTS_3 == 1 (set speed adjusting) is stripped, so our cue may use the field
    out_161, _ = run_ccnc(*stock_frames(ALERTS_3=1), ClusterAlert.red, long_control=True)
    self.assertEqual(out_161["ALERTS_3"], 18)

  def test_values_survive_dbc_packing(self):
    packer = CANPacker(DBC)
    out_161, out_162 = run_ccnc(*stock_frames(), ClusterAlert.red)
    parser = CANParser(DBC, [("CCNC_0x161", 0), ("CCNC_0x162", 0)], 0)
    for name, vals in (("CCNC_0x161", out_161), ("CCNC_0x162", out_162)):
      addr, dat, bus = packer.make_can_msg(name, 0, {k: v for k, v in vals.items() if k not in FAULT_VALUES})
      parser.update([(0, [(addr, dat, bus)])])
    self.assertEqual((parser.vl["CCNC_0x161"]["ALERTS_3"], parser.vl["CCNC_0x161"]["SOUNDS_3"], parser.vl["CCNC_0x162"]["VIBRATE"]), (18, 5, 1))
    out_161, out_162 = run_ccnc(*stock_frames(), ClusterAlert.green)
    for name, vals in (("CCNC_0x161", out_161), ("CCNC_0x162", out_162)):
      addr, dat, bus = packer.make_can_msg(name, 0, {k: v for k, v in vals.items() if k not in FAULT_VALUES})
      parser.update([(0, [(addr, dat, bus)])])
    self.assertEqual((parser.vl["CCNC_0x161"]["ALERTS_5"], parser.vl["CCNC_0x161"]["SOUNDS_2"]), (11, 2))


class TestClusterAlertLatch(unittest.TestCase):
  @staticmethod
  def cc_sp(alert):
    c = structs.CarControlSP()
    c.clusterAlert = alert
    return c

  def test_hold_then_clear(self):
    ctrl = ClusterAlertCarController()
    ctrl.update_cluster_alert(self.cc_sp(ClusterAlert.red), 100)
    self.assertEqual(ctrl.cluster_alert, ClusterAlert.red)
    for f in (101, 100 + CLUSTER_ALERT_HOLD_FRAMES - 1):  # request was one-shot, latch holds
      ctrl.update_cluster_alert(self.cc_sp(ClusterAlert.none), f)
      self.assertEqual(ctrl.cluster_alert, ClusterAlert.red)
    ctrl.update_cluster_alert(self.cc_sp(ClusterAlert.none), 100 + CLUSTER_ALERT_HOLD_FRAMES)
    self.assertEqual(ctrl.cluster_alert, ClusterAlert.none)

  def test_new_request_during_hold_is_ignored(self):
    ctrl = ClusterAlertCarController()
    ctrl.update_cluster_alert(self.cc_sp(ClusterAlert.red), 0)
    ctrl.update_cluster_alert(self.cc_sp(ClusterAlert.green), 10)
    self.assertEqual(ctrl.cluster_alert, ClusterAlert.red)


if __name__ == "__main__":
  unittest.main()
