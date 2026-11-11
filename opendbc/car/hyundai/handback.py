"""
Camera hand-back (GV70 CCNC LFA_ALT camera SCC): which side sends an axis, mirroring the panda's ownership rules.

Lateral (LFA_ALT 0xCB + MDPS 0xEA) is openpilot's from panda init: the panda blocks the camera's frames from the start and ours replace them,
active or inactive, so the car's own LFA lane centering never steers. The only exception is the car's own lane-keeping nudge while openpilot
steering is off (LkaWindow): the panda then forwards the camera's frames and we send nothing.

Longitudinal (SCC_CONTROL 0x1A0) belongs to the camera until openpilot takes it. The panda blocks the camera's frame from the first accepted
openpilot frame until the axis is no longer allowed AND the driver acted. openpilot therefore:
  - sends the axis only while it is active, and keeps sending 'off' frames after that (the car must not see a gap) until a driver
    action, and stops TAIL_FRAMES after the driver action: the panda hands the axis back at the first moment the permission is gone after the
    action (the heartbeat mismatch takes ~3 s), and rejects any frame of ours from then on, so a long tail is harmless and a short one is not.
Without a driver action it keeps sending 'off' frames: the car sees a benign off state, the stock camera does not take over unannounced.
"""
from opendbc.car import DT_CTRL, structs

TAIL_FRAMES = int(6.0 / DT_CTRL)         # > the panda's ~3 s heartbeat mismatch + margin
RECENT_ACTION_FRAMES = int(1.0 / DT_CTRL)  # a driver action this close before the axis went inactive is its reason
CANCEL_SETTLE_FRAMES = int(0.8 / DT_CTRL)  # panda HYUNDAI_CANFD_HB_CANCEL_SETTLE_US 0.6 s after the last CANCEL frame, plus our CAN latency


class AxisOwner:
  def __init__(self):
    self.owner = False
    self.active_prev = False
    self.release_frame: int | None = None
    self.last_action_frame: int | None = None

  def update(self, active: bool, driver_action: bool, frame: int) -> bool:
    """Returns True if openpilot should send this axis' frames now."""
    if driver_action:
      if self.owner and not active and self.release_frame is None:
        self.release_frame = frame
      self.last_action_frame = frame

    if active:
      self.owner = True
      self.release_frame = None
    elif self.owner and self.active_prev and self.last_action_frame is not None and frame - self.last_action_frame <= RECENT_ACTION_FRAMES:
      self.release_frame = frame  # went inactive right after a driver action (brake, cancel, main/MADS button)

    if self.owner and self.release_frame is not None and frame - self.release_frame > TAIL_FRAMES:
      self.owner = False
      self.release_frame = None

    self.active_prev = active
    return self.owner


# The panda's lane-keeping window (hyundai_canfd.h): camera LFA_ICON 0 for >= 1.5 s and LKA_SysIndReq 3 within 0.5 s. Ours is a strict subset
# (a later start, an earlier end, as seen through the CAN latency), so whenever we think the camera steers, the panda really forwards its frame,
# and the car never sees a gap in 0xCB; the panda drops our inactive frames while its window is open, so the two never interleave.
LKA_ICON_SETTLE_FRAMES = int(1.7 / DT_CTRL)  # panda 1.5 s (LFA's on/off blink lasts <= 1.3 s)
LKA_HOLD_FRAMES = int(0.3 / DT_CTRL)         # panda 0.5 s (longest LKA_SysIndReq gap inside a nudge: 0.33 s, seen once)
LKA_ICON_LFA = (1, 2)                        # camera LFA_ICON: grey (on, not steering) / green (on, steering)
LFA_PRESS_BLINK_FRAMES = int(1.5 / DT_CTRL)  # after an LFA button press the camera's icon blinks within 0.3 s (route 34: 0.25 s); no blink in this time = it ignored it


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
  """Per-frame hand-back state of the car controller: which axes openpilot sends this frame, and the counters it continues."""
  def __init__(self):
    self.long_owner = AxisOwner()
    self.lka_window = LkaWindow()
    self.lfa_alt_counter = CounterFollower(1 << 16)
    self.scc_counter = CounterFollower(1 << 16)
    self.prev_brake_pressed = False
    self.cancel_armed = False                  # a CANCEL press started with the camera main on and no openpilot ownership: the panda locks 0x1A0
    self.cancel_down = False
    self.cancel_release_frame: int | None = None

  def cancel_lock(self, CS, owner: bool, frame: int) -> bool:
    """Mirror of the panda's CANCEL pre-arm (hyundai_canfd.h): from the CANCEL press until CANCEL_SETTLE_FRAMES after its release the camera's 0x1A0 is
    blocked (the camera resumes its own ACC on the release), so our 'off' frames must already be there, or the car sees no 0x1A0."""
    for be in CS.out.buttonEvents:
      if be.type == structs.CarState.ButtonEvent.Type.cancel:
        self.cancel_down = be.pressed
        if be.pressed:
          self.cancel_armed = bool(CS.out.cruiseState.available) and not owner
        else:
          self.cancel_release_frame = frame
    if not self.cancel_armed:
      return False
    if self.cancel_down:
      return True
    if self.cancel_release_frame is not None and frame - self.cancel_release_frame <= CANCEL_SETTLE_FRAMES:
      return True
    self.cancel_armed = False
    return False

  def update(self, CC, CS, openpilot_long: bool, frame: int) -> tuple[bool, bool]:
    """Returns (send lateral, send longitudinal) for this frame."""
    button_type = structs.CarState.ButtonEvent.Type
    brake_edge = CS.out.brakePressed and not self.prev_brake_pressed
    self.prev_brake_pressed = CS.out.brakePressed
    pressed = [be.type for be in CS.out.buttonEvents if be.pressed]  # a list: the capnp enum values do not hash consistently
    driver_cruise = brake_edge or button_type.cancel in pressed or button_type.mainCruise in pressed
    # lateral: always ours from the start (inactive frames too: the camera's own request, LFA lane centering included, must not reach the EPS,
    # also after a main press that turns the car's LFA on), except while the car's own lane-keeping nudge (LKA) steers with openpilot
    # steering off: then the panda forwards the camera's 0xCB / MDPS and rejects ours, so we send nothing (the panda decides the same, see LkaWindow)
    if any(be.type == structs.CarState.ButtonEvent.Type.lkas and be.pressed for be in CS.out.buttonEvents):
      self.lka_window.lfa_pressed(frame)
    lka_nudge = self.lka_window.update(CS.cam_lfa_icon, CS.cam_lka_ind, frame)
    send_lat = not (lka_nudge and not CC.latActive)
    send_long = self.long_owner.update(bool(CC.enabled and openpilot_long), driver_cruise, frame)
    # the camera's own ACC engaged while openpilot is not: the panda locks it out of the car and takes our 'off' frames instead
    stock_acc_lock = bool(openpilot_long and not CC.enabled and CS.cruise_info.get("ACCMode", 0) in (1, 2))
    cancel_lock = self.cancel_lock(CS, self.long_owner.owner, frame) and bool(openpilot_long and not CC.enabled)  # (always consumes the events)
    return send_lat, send_long or stock_acc_lock or cancel_lock
