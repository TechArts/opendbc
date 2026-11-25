"""Closed-loop replay of a recorded GV70 drive through the CURRENT opendbc code: the recorded car and camera CAN (buses 0 and 2) go to the CarState
parser and to the panda's rx hook (libsafety), the recorded carControl (latActive, desired angle, ...) goes to the real CarController, and what the
controller sends goes through the panda's tx hook; the camera's 0xCB goes through the panda's forward hook. Nothing of the old controller is replayed.

  PYTHONPATH=$PWD ~/workspace/comma/opendbc/.venv/bin/python -m opendbc.car.hyundai.tests.route_replay_gv70 \\
      <route dir prefix> <route time a> <route time b> [--pad S] [--warm N]
  e.g. ... ~/workspace/comma/routes/00000055--9a3f24b784 3:16.9 3:19.0

Route time is logMonoTime - the first carState of the route (the analysis tools' clock); times are mm:ss or seconds. Whole segments from (a - pad) to (b + 5 s)
are read, frames only from the moment the recorded panda was in hyundaiCanfd (its bus 2 carries stock frames before); the first --warm cycles run with
latActive forced off so the controller and the panda start in sync (a cold start mid-turn has no last request).

What is replayed and what is not:
- The panda's controls_allowed / controls_allowed_lateral follow the recorded pandaStates (the MADS and cruise state machine is not replayed).
- The panda's clock is the recorded logMonoTime made monotonic (events of different processes interleave out of order; a clock that steps back
  makes the panda's marks look expired: the first open-loop replay never opened the nudge window because of it).
- The wheel, the speeds and the camera follow the recorded drive: the plant is open loop, our requests do not move the wheel. What a replay shows is
  what the new code would have SENT and what the new panda would have PASSED for the same car and camera behaviour.
- The controller starts from the recorded carControl of that moment: its latActive and the model's desired angle (steeringAngleDeg).

Output per window: the longest time without any 0xCB reaching the car (ours accepted at any level, or the camera's forwarded), our ACTIVE frames
rejected by the panda (runs), the camera's 0xCB passed / seen, the controller's pause, and what the recorded drive's car saw (bus 128)."""
import argparse
import io
import os
import sys
from collections import Counter
from types import SimpleNamespace

import pickle
import subprocess

CEREAL = os.environ.get("GV70_REPLAY_CEREAL", os.path.expanduser(
  "~/Library/Mobile Documents/com~apple~CloudDocs/Personal/Notes/Personal/RedLightAlert/tools/analysis"))  # holds cereal/log.capnp
SAFETY_HYUNDAI_CANFD = 28  # opendbc/safety/declarations.h; importing the schema from structs would load a second capnp schema
EPS_ANGLE_MAX, GUARD_BAND = 175.0, 10.0  # LFA_ALT: the EPS faults on a request beyond it; takeover.GUARD_MAX_ERROR
GAP_MS_ALERT = 25.0  # a cycle is 10 ms: more than this without a 0xCB at the car is a gap worth listing
CAN_LEAD_US = 20000  # a CAN batch's logMonoTime may run ahead of the controller's; more than two cycles is a logging artefact, not time

CACHE = os.environ.get("GV70_REPLAY_CACHE", os.path.expanduser("~/workspace/comma/out/replay-cache"))


class _CI:
  @staticmethod
  def get_longitudinal_tuning_sp(CP, CP_SP):
    return CP_SP


def parse_time(s):
  if ":" in s:
    m, sec = s.split(":")
    return int(m) * 60 + float(sec)
  return float(s)


def mmss(t):
  return f"{int(t // 60)}:{t % 60:04.1f}"


def decode_segment(path, out):
  """Runs in its own interpreter (the cereal schema and opendbc's car.capnp cannot be loaded into one): rlog.zst -> pickle of what the replay reads.
  ('can', ns, [(addr, src, dat)]), ('cc', ns, dict), ('ccsp', ns, dict), ('ps', ns, (allowed, lateral)), ('cs', ns), ('init', params), ('cp', long)."""
  import capnp
  import zstandard
  capnp.remove_import_hook()
  log = capnp.load(f"{CEREAL}/cereal/log.capnp", imports=[f"{CEREAL}/cereal", f"{CEREAL}/cereal/include"])
  raw = zstandard.ZstdDecompressor().stream_reader(io.BytesIO(open(path, 'rb').read())).read()
  rows = []
  for ev in log.Event.read_multiple_bytes(raw):
    w, ns = ev.which(), ev.logMonoTime
    if w == 'can':
      rows.append(('can', ns, [(f.address, f.src, bytes(f.dat)) for f in ev.can if f.src <= 2 or (f.src == 128 and f.address == 0xcb)]))
    elif w == 'carControl':
      c, a, h, k = ev.carControl, ev.carControl.actuators, ev.carControl.hudControl, ev.carControl.cruiseControl
      rows.append(('cc', ns, dict(enabled=c.enabled, latActive=c.latActive, longActive=c.longActive, steeringAngleDeg=a.steeringAngleDeg, accel=a.accel,
                                  torque=a.torque, curvature=a.curvature, cancel=k.cancel, resume=k.resume, override=k.override, setSpeed=h.setSpeed,
                                  speedVisible=h.speedVisible, leadDistanceBars=int(h.leadDistanceBars))))
    elif w == 'carControlSP':
      m = ev.carControlSP.mads
      rows.append(('ccsp', ns, dict(available=m.available, enabled=m.enabled, active=m.active)))
    elif w == 'pandaStates':
      p = ev.pandaStates[0]
      rows.append(('ps', ns, (bool(p.controlsAllowed), bool(p.controlsAllowedLateral), str(p.safetyModel))))
    elif w == 'carState':
      rows.append(('cs', ns, None))
    elif w == 'initData':
      rows.append(('init', ns, {p.key: bytes(p.value).decode(errors="replace") for p in ev.initData.params.entries}))
    elif w == 'carParams':
      rows.append(('cp', ns, dict(long=bool(ev.carParams.openpilotLongitudinalControl), flags=int(ev.carParams.flags),
                                  safety=[int(c.safetyParam) for c in ev.carParams.safetyConfigs])))
  pickle.dump(rows, open(out, 'wb'))


def load_segment(prefix, seg):
  src = f"{prefix}--{seg}/rlog.zst"
  os.makedirs(CACHE, exist_ok=True)
  out = os.path.join(CACHE, f"{os.path.basename(prefix)}--{seg}.pkl")
  if not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(src):
    subprocess.run([sys.executable, os.path.abspath(__file__), "--decode", src, out], check=True)
  return pickle.load(open(out, 'rb'))


def segment_count(prefix):
  n = 0
  while os.path.exists(f"{prefix}--{n}/rlog.zst"):
    n += 1
  return n


def fingerprint_of(rows, seconds=10.0):
  """card.py's fingerprint: {bus: {address: length}} of the first frames of the drive (buses 0-2)."""
  from opendbc.car import gen_empty_fingerprint
  fp, t_first = gen_empty_fingerprint(), None
  for kind, ns, val in rows:
    if kind != 'can':
      continue
    t_first = ns if t_first is None else t_first
    if ns - t_first > seconds * 1e9:
      break
    for addr, src, dat in val:
      if src <= 2:
        fp[src][addr] = len(dat)
  return fp


def make_carcontroller(params, fingerprint, alpha_long):
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.tests.gv70 import car_params
  from opendbc.sunnypilot.car.interfaces import _initialize_custom_longitudinal_tuning
  cp, cp_sp = car_params(alpha_long=alpha_long, fingerprint=fingerprint)
  _initialize_custom_longitudinal_tuning(_CI, cp, cp_sp, params)
  return cp, cp_sp, CarInterface(cp, cp_sp)


def fill_carcontrol(d, force_lat_off):
  from opendbc.car import structs
  CC = structs.CarControl()
  CC.enabled = bool(d["enabled"])
  CC.latActive = bool(d["latActive"]) and not force_lat_off
  CC.longActive = bool(d["longActive"])
  CC.actuators.steeringAngleDeg = float(d["steeringAngleDeg"])
  CC.actuators.accel = float(d["accel"])
  CC.actuators.torque = float(d["torque"])
  CC.actuators.curvature = float(d["curvature"])
  CC.cruiseControl.cancel = bool(d["cancel"])
  CC.cruiseControl.resume = bool(d["resume"])
  CC.cruiseControl.override = bool(d["override"])
  CC.hudControl.setSpeed = float(d["setSpeed"])
  CC.hudControl.speedVisible = bool(d["speedVisible"])
  CC.hudControl.leadDistanceBars = int(d["leadDistanceBars"])
  return CC


def fill_carcontrol_sp(d):
  from opendbc.car import structs
  CC_SP = structs.CarControlSP()
  if d is not None:
    CC_SP.mads.available = bool(d["available"])
    CC_SP.mads.enabled = bool(d["enabled"])
    CC_SP.mads.active = bool(d["active"])
  return CC_SP


def runs_of(times, step=0.05):
  out = []
  for t in times:
    if out and t - out[-1][1] <= step:
      out[-1][1] = t
      out[-1][2] += 1
    else:
      out.append([t, t, 1])
  return out


def replay(prefix, t_a, t_b, pad=40.0, warm=100, to_panda=True, tx_filter=None):
  from opendbc.car import CanData
  from opendbc.safety.tests.libsafety import libsafety_py
  n = segment_count(prefix)
  if not n:
    raise SystemExit(f"no segments under {prefix}")
  seg0 = load_segment(prefix, 0)
  t0 = next(ns for k, ns, _ in seg0 if k == 'cs') / 1e9
  params = next((v for k, _, v in seg0 if k == 'init'), {})
  rec_cp = next((v for k, _, v in seg0 if k == 'cp'), None)
  alpha_long = rec_cp["long"] if rec_cp else True
  first, last = max(0, int((t_a - pad) // 60)), min(n - 1, int((t_b + 5.0) // 60))
  cp, cp_sp, CI = make_carcontroller(params, fingerprint_of(seg0), alpha_long)
  cc, cs = CI.CC, CI.CS
  print(f"CarParams: flags {cp.flags}, safetyParam {int(cp.safetyConfigs[-1].safetyParam)}, SP safetyParam {cp_sp.safetyParam}; recorded (old build): {rec_cp}")

  S = libsafety_py.libsafety
  S.set_current_safety_param_sp(cp_sp.safetyParam)
  S.set_safety_hooks(SAFETY_HYUNDAI_CANFD, int(cp.safetyConfigs[-1].safetyParam))
  S.init_tests()
  S.set_current_safety_param_sp(cp_sp.safetyParam)

  rec = SimpleNamespace(cb_ours=[], cb_cam=[], cb_car=[], rej=[], cyc=[], tx_rejected=Counter(), tx_total=Counter(), ev=[])
  live = False
  pending, cc_sp_src, cycle, timer = [], None, 0, 0  # timer: the panda's clock never runs backwards; logMonoTime of events of different processes does
  cc_us = None  # the last controller cycle's time: a CAN batch may move the clock at most CAN_LEAD_US past it (route 60 19:14.5: one batch
  #               stamped 0.65 s ahead froze the clock for 0.7 s, and the panda's 250 ms frame count rejected ~50 good frames)
  for seg in range(first, last + 1):
    for kind, ns, val in load_segment(prefix, seg):
      t = ns / 1e9 - t0
      if t > t_b + 5.0:
        break
      if kind == 'ps':
        live = val[2] == 'hyundaiCanfd'
        S.set_controls_allowed(val[0] and live)
        S.set_controls_allowed_lateral(val[1] and live)
        continue
      if not live:  # the recorded panda was not in the car's safety mode yet (ignition, startup): its bus 2 still carries frames the mode never sees
        continue
      if kind == 'ccsp':
        cc_sp_src = val
      elif kind == 'can':
        timer = max(timer, ns // 1000 if cc_us is None else min(ns // 1000, cc_us + CAN_LEAD_US))
        S.set_timer(timer & 0xFFFFFFFF)
        frames = []
        for addr, src, dat in val:
          if src == 128:
            rec.cb_car.append(t)
            continue
          S.safety_rx_hook(libsafety_py.make_CANPacket(addr, src, dat))
          frames.append(CanData(addr, dat, src))
          if addr == 0xcb and src == 2:
            fwd = S.safety_fwd_hook(2, 0xcb)
            rec.cb_cam.append(t if fwd != -1 else -t)  # negative: seen, blocked
            rec.ev.append((t, 'cam', 'pass' if fwd != -1 else 'block'))
        pending.append((ns, frames))
      elif kind == 'cc':
        cc_us = ns // 1000
        timer = max(timer, cc_us)
        S.set_timer(timer & 0xFFFFFFFF)
        CI.update(pending)
        pending = []
        CC = fill_carcontrol(val, force_lat_off=cycle < warm)
        _, sends = CI.apply(CC.as_reader(), fill_carcontrol_sp(cc_sp_src), ns)
        cycle += 1
        sent_active = False
        for addr, dat, bus in sends:
          ok = bool(S.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat))) if to_panda and (tx_filter is None or tx_filter(addr, bytes(dat))) else True
          rec.tx_total[addr] += 1
          if addr == 0xea:
            rec.ev.append((t, 'mdps', 'ok' if ok else 'REJECT'))
          if not ok:
            rec.tx_rejected[addr] += 1
          if addr == 0xcb:
            lvl2 = bytes(dat)[3] >> 4
            rec.ev.append((t, 'ours', f"lvl{lvl2} {'ok' if ok else 'REJECT'}"))
            if ok:
              rec.cb_ours.append((t, lvl2))
            if lvl2 == 2:
              sent_active = True
              if not ok:
                rec.rej.append(t)
        rec.cyc.append((t, CC.latActive, sent_active, bool(cc.takeover.paused), bool(cc.send_lat), float(cs.out.steeringAngleDeg), float(cs.out.vEgo)))
  return t0, rec


def longest_gap(times, a, b):
  """Longest gap between consecutive frames that starts inside [a, b] (the frame after it may be later than b): (start, length s)."""
  best = (None, 0.0)
  for x, y in zip(times, times[1:], strict=False):
    if a <= x <= b and y - x > best[1]:
      best = (x, y - x)
  return best


def window_report(name, rec, a, b):
  ours = sorted(t for t, _ in rec.cb_ours)
  passed = sorted(t for t in rec.cb_cam if t > 0)
  reaching = sorted(ours + passed)
  car = sorted(rec.cb_car)
  gap_t, gap = longest_gap(reaching, a, b)
  car_t, car_gap = longest_gap(car, a, b)
  in_w = lambda t: a <= abs(t) <= b  # noqa: E731
  rej = [t for t in rec.rej if in_w(t)]
  # a rejected ACTIVE frame while the camera's frame passed within 30 ms is the panda's hand-over (the window closes on it); any other is a lost frame
  other = [t for t in rej if not any(abs(t - c) <= 0.03 for c in passed if abs(t - c) <= 0.03)]
  cyc = [c for c in rec.cyc if in_w(c[0])]
  stuck = run = 0
  for _, lat, sent, paused, send, _, _ in cyc:  # latActive, nothing sent, no pause, controller silent
    run = run + 1 if (lat and not sent and not paused and not send) else 0
    stuck = max(stuck, run)
  cam_pass = [t for t in passed if in_w(t)]
  cam_all = [t for t in rec.cb_cam if in_w(t)]
  mdps_rej = sum(1 for t, k, i in rec.ev if k == 'mdps' and 'REJECT' in i and in_w(t))
  gap_at = f" (at {mmss(gap_t)})" if gap > GAP_MS_ALERT / 1000 else ""
  car_at = f" (at {mmss(car_t)})" if car_gap > GAP_MS_ALERT / 1000 else ""
  rej_text = ""
  if rej:
    runs_text = f": runs {', '.join(f'{mmss(x[0])}+{x[2]}' for x in runs_of(other)[:8])}" if other else ""
    rej_text = f" ({len(rej) - len(other)} at a hand-over, {len(other)} other{runs_text})"
  wheel = [abs(c[5]) for c in cyc]
  past_limit = EPS_ANGLE_MAX + GUARD_BAND
  past = sum(1 for c in cyc if c[2] and abs(c[5]) > past_limit)
  print(f"== {name}  {mmss(a)}-{mmss(b)}")
  print(f"   longest time without a 0xCB at the car: {1000 * gap:.0f} ms{gap_at}; the recorded car (old build): {1000 * car_gap:.0f} ms{car_at}")
  print(f"   ours: ACTIVE sent {sum(1 for c in cyc if c[2])}, ACTIVE rejected {len(rej)}{rej_text}; MDPS copy rejected {mdps_rej}")
  print(f"   camera 0xCB passed {len(cam_pass)} of {len(cam_all)}")
  print(f"   wheel: max {max(wheel, default=0):.0f} deg; ACTIVE sent with the wheel past {past_limit:.0f} deg (EPS limit + guard band): {past}")
  lat_on, paused, silent = (sum(1 for c in cyc if pred(c)) for pred in (lambda c: c[1], lambda c: c[3], lambda c: not c[4]))
  print(f"   cycles {len(cyc)}: latActive {lat_on}, takeover paused {paused}, controller silent {silent}, "
        + f"stuck (latActive, nothing sent, no pause, silent) longest {stuck}")
  return SimpleNamespace(gap_ms=1000 * gap, car_gap_ms=1000 * car_gap, rej=len(rej), rej_other=len(other), cam_pass=len(cam_pass), cam_all=len(cam_all),
                         stuck=stuck)


def main():
  if len(sys.argv) == 4 and sys.argv[1] == "--decode":
    return decode_segment(sys.argv[2], sys.argv[3])
  ap = argparse.ArgumentParser()
  ap.add_argument("prefix")
  ap.add_argument("a")
  ap.add_argument("b")
  ap.add_argument("--pad", type=float, default=40.0)
  ap.add_argument("--warm", type=int, default=100)
  ap.add_argument("--name", default="")
  args = ap.parse_args()
  a, b = parse_time(args.a), parse_time(args.b)
  _, rec = replay(args.prefix, a, b, args.pad, args.warm)
  window_report(args.name or os.path.basename(args.prefix), rec, a, b)
  rejected = {hex(k): f"{v}/{rec.tx_total[k]}" for k, v in rec.tx_rejected.items()}
  print("   tx rejected / sent by address (whole replay):", rejected or "none")


if __name__ == "__main__":
  sys.exit(main())
