# PFEIFER - VTSC

# Acknowledgements:
# Past versions of this code were based on the move-fast teams vtsc
# implementation. (https://github.com/move-fast/openpilot) Huge thanks to them
# for their initial implementation. I also used sunnypilot as a reference.
# (https://github.com/sunnyhaibin/sunnypilot) Big thanks for sunny's amazing work

import numpy as np
from time import time
from openpilot.common.params import Params
params = Params()

TARGET_LAT_A = 2.3 # m/s^2
MIN_TARGET_V = 5 # m/s
HOLD_TIME = 1.0 # s

class VisionTurnController():
  def __init__(self):
    self.op_enabled = False
    self.gas_pressed = False
    self.last_params_update = 0
    self.enabled = params.get_bool("TurnVisionControl")
    self.v_target = MIN_TARGET_V
    self.v_target_time = time()


  @property
  def active(self):
    return self.op_enabled and not self.gas_pressed and self.enabled

  def update_params(self):
    t = time()
    if t > self.last_params_update + 5.0:
      self.enabled = params.get_bool("TurnVisionControl")
      self.last_params_update = t

  def update(self, op_enabled, v_ego, sm):
    self.update_params()
    self.op_enabled = op_enabled
    self.gas_pressed = sm['carState'].gasPressed

    rate_plan = np.array(np.abs(sm['modelV2'].orientationRate.z))
    vel_plan = np.array(sm['modelV2'].velocity.x)

    if rate_plan.size == 0 or vel_plan.size == 0:
      return

    # get the maximum lat accel from the model
    predicted_lat_accels = rate_plan * vel_plan
    self.max_pred_lat_acc = np.amax(predicted_lat_accels)

    # get the maximum curve based on the current velocity
    v_ego = max(v_ego, 0.1) # ensure a value greater than 0 for calculations
    max_curve = self.max_pred_lat_acc / (v_ego**2)

    # Get the target velocity for the maximum curve
    v_target = (TARGET_LAT_A / max_curve) ** 0.5
    v_target = max(v_target, MIN_TARGET_V)

    # only set if lower than current target or if we are past the hold time
    if v_target < self.v_target or self.v_target_time + HOLD_TIME < time():
      self.v_target = v_target
      self.v_target_time = time()


vtsc = VisionTurnController()
