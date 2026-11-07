"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import DT_CTRL, structs

CLUSTER_ALERT_HOLD_FRAMES = int(2.0 / DT_CTRL)

ClusterAlert = structs.CarControlSP.ClusterAlert


class ClusterAlertCarController:
  """Latches a one-shot red/green-light request from CarControlSP so the cluster shows it for CLUSTER_ALERT_HOLD_FRAMES."""

  def __init__(self):
    self.cluster_alert = ClusterAlert.none
    self.cluster_alert_end_frame = -1

  def update_cluster_alert(self, CC_SP: structs.CarControlSP, frame: int) -> None:
    if frame >= self.cluster_alert_end_frame:
      self.cluster_alert = ClusterAlert.none
      if CC_SP.clusterAlert != ClusterAlert.none:
        self.cluster_alert = ClusterAlert(CC_SP.clusterAlert)
        self.cluster_alert_end_frame = frame + CLUSTER_ALERT_HOLD_FRAMES
