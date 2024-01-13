# PFEIFER - DLP
# Acknowledgements:
# This dynamic lane planner implementation is largely a copy of the sunnypilot
# DLP implementation with some features removed. Some minor additional code
# simplification has been performed as well.
# https://github.com/sunnyhaibin/sunnypilot/blob/dev-c3/selfdrive/controls/lib/lane_planner.py

import numpy as np
from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.numpy_fast import interp
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params
params = Params()

LaneChangeState = log.LateralPlan.LaneChangeState

TRAJECTORY_SIZE = 33


class LanePlanner:
  def __init__(self, DH):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(3.7, 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = 3.7

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.dynamic_lane_profile_status_buffer = False

    self._frame = 0

    self.DH = DH

    self.t_idxs = np.arange(TRAJECTORY_SIZE)

    try:
      self.enabled = params.get_bool("DynamicLanePlanner")
    except:
      self.enabled = False

  def use_lane_planner(self, v_ego):
    if not self.enabled:
      return False
    if v_ego < 4.5:
      return False

    lll_prob = self.lll_prob
    rll_prob = self.rll_prob
    # Turn off lanes during lane change
    if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
      lll_prob *= self.DH.lane_change_ll_prob
      rll_prob *= self.DH.lane_change_ll_prob

    # laneless while lane change in progress
    if self.DH.lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
      return False
    # only while lane change is off
    elif self.DH.lane_change_state == LaneChangeState.off:
      # laneline probability too low, we switch to laneless mode
      if (lll_prob + rll_prob) / 2 < 0.3:
        self.dynamic_lane_profile_status_buffer = True
      if (lll_prob + rll_prob) / 2 > 0.5:
        self.dynamic_lane_profile_status_buffer = False
      if self.dynamic_lane_profile_status_buffer:  # in buffer mode, always laneless
        return False
    return True

  def parse_model(self, md):
    if len(md.orientation.x) == TRAJECTORY_SIZE:
      self.t_idxs = np.array(md.position.t)

    lane_lines = md.laneLines
    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y)
      self.rll_y = np.array(lane_lines[2].y)
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, v_ego, path_xyz):
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = interp(t_check * v_ego, self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    self.lane_width_estimate.update(current_lane_width)
    speed_lane_width = interp(v_ego, [0., 31.], [2.8, 3.5])
    self.lane_width = self.lane_width_certainty.x * self.lane_width_estimate.x + \
                      (1 - self.lane_width_certainty.x) * speed_lane_width

    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    self.d_prob = l_prob + r_prob - l_prob * r_prob
    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    safe_idxs = np.isfinite(self.ll_t)
    if safe_idxs[0]:
      lane_path_y_interp = np.interp(self.t_idxs, self.ll_t[safe_idxs], lane_path_y[safe_idxs])
      path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
    return path_xyz
