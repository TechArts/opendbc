MAIN_PRESS_FRAMES = 50  # 0.5 s at 100 Hz: a main press and the camera's main change this close belong together


class CameraMain:
  """The camera's own MainMode_ACC (readable even while the panda blocks its SCC_CONTROL) is the car's main cruise state, the single source
  of truth shared with the panda."""
  def __init__(self):
    self.on = False
    self.press_age = self.rise_age = MAIN_PRESS_FRAMES + 1

  def update(self, camera_main: bool | None, main_pressed: bool) -> bool:
    """Returns the engage request: a rising edge of the camera's main with a main press within MAIN_PRESS_FRAMES (either order). A press the
    camera ignores (e.g. brake held at low speed) or a camera change without a press engages nothing."""
    if camera_main is None:
      camera_main = self.on  # not seen yet: keep the last state
    rising = camera_main and not self.on
    self.on = camera_main
    self.press_age = 0 if main_pressed else min(self.press_age + 1, MAIN_PRESS_FRAMES + 1)
    self.rise_age = 0 if rising else min(self.rise_age + 1, MAIN_PRESS_FRAMES + 1)
    return camera_main and ((rising and self.press_age <= MAIN_PRESS_FRAMES) or (main_pressed and self.rise_age <= MAIN_PRESS_FRAMES))
