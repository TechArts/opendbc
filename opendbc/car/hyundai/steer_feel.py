"""GV70 CCNC LFA_ALT steer feel: how much steering force (ADAS_ACIAnglTqRedcGainVal in 0xCB) the car allows while the driver pushes.

The stock camera's own gain vs driver torque, bin medians (placed at bin midpoints) from route 00000002 above 5 m/s. The generic curve
held 3-6x more gain against the driver. Experimental, chosen per drive with the HyundaiSteerFeel param (0 generic, 1 halfway, 2 stock
above 6 m/s, 3 stock at every speed). Below 4 m/s there is no stock data (the stock LFA is barely active there), so 1 and 2 keep the
generic curve at low speed; 3 uses the stock curve there too.

HyundaiCurveHold (research/curve-hold.md Q3): in a curve the gain never falls below a share of the model's lateral demand
a = v^2 * |desired| * k.
"""
import numpy as np

from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP

LFA_ALT_GAIN_TORQUE_BP = [25, 75, 125, 175, 250, 400]
LFA_ALT_GAIN_V = [0.85, 0.55, 0.25, 0.15, 0.10, 0.07]
LFA_ALT_GAIN_SPEED_BP = [4, 6]  # m/s: blend from the generic curve to the stock-matched one

CURVE_HOLD_A_BP = [0.3, 1.0, 2.0]  # m/s^2
CURVE_HOLD_FLOOR = [0.0, 0.30, 0.55]


def steer_feel_blend(CP, CP_SP) -> tuple[float, bool]:
  """Returns (weight of the stock curve, whether it applies at every speed)."""
  if CP.carFingerprint != CAR.GENESIS_GV70_2ND_GEN or not CP.flags & HyundaiFlags.SEND_LFA:
    return 0.0, False
  all_speeds = bool(CP_SP.flags & HyundaiFlagsSP.STEER_FEEL_ALL_SPEEDS)
  if CP_SP.flags & HyundaiFlagsSP.STEER_FEEL_STOCK:
    return 1.0, all_speeds
  if CP_SP.flags & HyundaiFlagsSP.STEER_FEEL_HALFWAY:
    return 0.5, all_speeds
  return 0.0, False


def apply_steer_feel(generic, ceiling, steering_torque, v_ego, stock_blend, all_speeds, curve_hold):
  """Moves the generic gain target towards the stock curve and applies the curve-hold floor.

  curve_hold: None (off) or (lateral demand a_hold in m/s^2, hand opposes the turn, filter paused or resuming)."""
  if stock_blend <= 0.0:
    return generic
  # first point at the ceiling, so the no-hands gain is unchanged at any speed (the ceiling is 0.85 above 1.5 m/s)
  stock = min(np.interp(abs(steering_torque), LFA_ALT_GAIN_TORQUE_BP, [ceiling, *LFA_ALT_GAIN_V[1:]]), ceiling)
  w = stock_blend * (1.0 if all_speeds else np.interp(v_ego, LFA_ALT_GAIN_SPEED_BP, [0.0, 1.0]))
  target = (1.0 - w) * generic + w * stock
  if curve_hold is not None:
    a_hold, opposing, paused = curve_hold
    if not opposing and not paused:
      target = max(target, min(float(np.interp(a_hold, CURVE_HOLD_A_BP, CURVE_HOLD_FLOOR)), float(generic)))
  return target
