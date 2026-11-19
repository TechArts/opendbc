"""
Camera hand-back (GV70 CCNC LFA_ALT camera SCC), lateral only: which side sends the steering frames, mirroring the panda's ownership rules.

Lateral (LFA_ALT 0xCB + MDPS 0xEA) is openpilot's from panda init: the panda blocks the camera's frames from the start and ours replace them, active
or inactive, so the car's own LFA lane centering never steers. The only exception is the car's own lane-keeping nudge (LKA, lane-departure
prevention) while openpilot steering is off (LkaWindow): the panda then forwards the camera's frames and we send nothing. The camera's 0x1A0
(SCC_CONTROL) keeps upstream's static block while openpilot has longitudinal, nothing of it is handed back.
"""
from opendbc.car import DT_CTRL, structs

# The panda's lane-keeping window (hyundai_canfd.h): camera LFA_ICON 0 for >= 1.5 s and LKA_SysIndReq 3 within 0.5 s, while openpilot is not steering
# (no accepted ACTIVE 0xCB for 0.2 s, whatever its permission); the panda's sender state machine decides frame by frame and drops our frames while
# the camera's are forwarded. Ours is a strict subset (a later start, an earlier end, as seen through the CAN latency, and 0.3 s without an ACTIVE
# frame), so whenever we go silent for the camera, the panda forwards its frame; the car misses at most one 0xCB at a switch of sender (10 ms) and
# never gets two. Route 48 (T23): our silence keyed on CC.latActive while the panda still required both permissions off; with openpilot cruise on
# the car got no 0xCB for 0.85 s and the EPS faulted 0.53 s in, 5 times.
LKA_ICON_SETTLE_FRAMES = int(1.7 / DT_CTRL)  # panda 1.5 s (LFA's on/off blink lasts <= 1.3 s)
LKA_HOLD_FRAMES = int(0.3 / DT_CTRL)         # panda 0.5 s (longest LKA_SysIndReq gap inside a nudge: 0.33 s, seen once)
LKA_ICON_LFA = (1, 2)                        # camera LFA_ICON: grey (on, not steering) / green (on, steering)
LFA_PRESS_BLINK_FRAMES = int(1.5 / DT_CTRL)  # an LFA button press makes the camera's icon blink within 0.3 s (route 34: 0.25 s); no blink in this time: ignored
LAT_IDLE_FRAMES = int(0.3 / DT_CTRL)         # panda 0.2 s (HYUNDAI_CANFD_HB_OP_IDLE_US) without an accepted ACTIVE frame = openpilot not steering


class LkaWindow:
  """Tells the car's own lane-keeping nudge (LKA, lane-departure prevention: camera LFA_ICON 0, LKA_SysIndReq 3) from its LFA lane centering
  (LFA_ICON 1/2, or the 3,0,3,0 blink around an off press): only the nudge may steer once openpilot steering is off."""
  def __init__(self):
    self.icon_zero_since: int | None = None
    self.lka_frame: int | None = None
    self.lfa_on = False  # the camera's own LFA, settled (a blink does not flip it)
    self.press_frame: int | None = None  # an LFA button press with the camera's LFA on: it goes off, the icon must not wait for the blink to settle
    self.blink_seen = False

  def lfa_pressed(self, frame: int) -> None:
    """The LFA button (MADS) was pressed while the camera's LFA is on: that press turns it off (the icon blinks 3,0,3,0 for 1-2 s, then 0, which
    only counts as off after the settle time). The truth for the dash is off from the press itself."""
    if self.lfa_on:
      self.lfa_on = False
      self.press_frame = frame
      self.blink_seen = False

  def update(self, icon: int | None, ind: int | None, frame: int) -> bool:
    """icon: camera LFA_ICON (0x161), ind: camera LKA_SysIndReq (0x12A), None before the first frame. True while the camera's nudge may steer."""
    if icon is None or icon != 0:
      self.icon_zero_since = None
    elif self.icon_zero_since is None:
      self.icon_zero_since = frame
    settled = self.icon_zero_since is not None and frame - self.icon_zero_since >= LKA_ICON_SETTLE_FRAMES
    if self.press_frame is not None:
      self.blink_seen = self.blink_seen or (icon is not None and icon not in LKA_ICON_LFA)
      if icon in LKA_ICON_LFA and (self.blink_seen or frame - self.press_frame > LFA_PRESS_BLINK_FRAMES):
        self.press_frame = None  # blinked and is on again (an LFA on press), or never blinked (the press changed nothing)
      elif settled:
        self.press_frame = None
    if icon in LKA_ICON_LFA:
      self.lfa_on = self.press_frame is None
    elif settled:
      self.lfa_on = False
    if ind == 3:
      self.lka_frame = frame
    return settled and self.lka_frame is not None and frame - self.lka_frame <= LKA_HOLD_FRAMES


class CounterFollower:
  """Counter for frames that replace the camera's: continue the camera's own counter so the receiver sees no jump at either hand-over."""
  def __init__(self, modulus: int):
    self.modulus = modulus
    self.last_camera: int | None = None
    self.last_out: int | None = None

  def next(self, camera_counter: int | None) -> int | None:
    if camera_counter is None:
      return None  # nothing to follow yet: the packer's own counter
    camera_counter = int(camera_counter)
    if self.last_out is None or camera_counter != self.last_camera:
      self.last_out = camera_counter
    else:
      self.last_out = (self.last_out + 1) % self.modulus
    self.last_camera = camera_counter
    return self.last_out


class CameraHandback:
  """Per-frame hand-back state of the car controller: whether openpilot sends the lateral frames this frame, and the counter it continues."""
  def __init__(self):
    self.lka_window = LkaWindow()
    self.lfa_alt_counter = CounterFollower(1 << 8)  # LFA_ALT COUNTER is 8 bits
    self.steer_frame: int | None = None  # the last frame we sent an ACTIVE 0xCB (openpilot steering)

  def update(self, CS, frame: int, steer_req: bool = False) -> bool:
    """Returns whether openpilot sends lateral (0xCB and the MDPS copy) this frame. steer_req: this frame's 0xCB would be ACTIVE.

    Always, from the start (inactive frames too: the camera's own request, LFA lane centering included, must not reach the EPS, also after a main press
    that turns the car's LFA on), except while the car's own lane-keeping nudge steers with openpilot not steering (no ACTIVE frame for LAT_IDLE_FRAMES,
    cruise on or off): then the panda forwards the camera's 0xCB / MDPS and rejects ours, so we send nothing (the panda decides the same, see LkaWindow).
    An ACTIVE request sends at once: the panda drops it and closes its window."""
    if any(be.type == structs.CarState.ButtonEvent.Type.lkas and be.pressed for be in CS.out.buttonEvents):
      self.lka_window.lfa_pressed(frame)
    lka_nudge = self.lka_window.update(CS.cam_lfa_icon, CS.cam_lka_ind, frame)
    if steer_req:
      self.steer_frame = frame
    not_steering = self.steer_frame is None or frame - self.steer_frame > LAT_IDLE_FRAMES
    return not (lka_nudge and not_steering)
