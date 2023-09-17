import math
import numpy as np
from cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.numpy_fast import interp
from openpilot.common.realtime import DT_MDL
from openpilot.system.swaglog import cloudlog


TRAJECTORY_SIZE = 33
# camera offset is meters from center car to camera
# model path is in the frame of the camera
PATH_OFFSET = 0.00
CAMERA_OFFSET = 0.04

MIN_LANE_DISTANCE = 2.6
MAX_LANE_DISTANCE = 4.0
KEEP_MIN_DISTANCE_FROM_LANE = 1.35
KEEP_MAX_DISTANCE_FROM_LANE = 2.0

def clamp(num, min_value, max_value):
  # weird broken case, do something reasonable
  if min_value > num > max_value:
    return (min_value + max_value) * 0.5
  # ok, basic min/max below
  if num < min_value:
    return min_value
  if num > max_value:
    return max_value
  return num

def sigmoid(x, scale=1, offset=0):
  return (1 / (1 + math.exp(x*scale))) + offset

def lerp(start, end, t):
  t = clamp(t, 0.0, 1.0)
  return (start * (1.0 - t)) + (end * t)

class LanePlanner:
  def __init__(self):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.le_y = np.zeros((TRAJECTORY_SIZE,))
    self.re_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(3.2, 9.95, DT_MDL)
    self.lane_width = 3.2
    self.lane_change_multiplier = 1

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.camera_offset = -CAMERA_OFFSET
    self.path_offset = -PATH_OFFSET

  def parse_model(self, md):
    lane_lines = md.laneLines
    edges = md.roadEdges

    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y) + self.camera_offset
      self.rll_y = np.array(lane_lines[2].y) + self.camera_offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    if len(edges[0].t) == TRAJECTORY_SIZE:
      self.le_y = np.array(edges[0].y) + md.roadEdgeStds[0] * 0.4
      self.re_y = np.array(edges[1].y) - md.roadEdgeStds[1] * 0.4
    else:
      self.le_y = self.lll_y
      self.re_y = self.rll_y

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, v_ego, path_t, path_xyz, vcurv):
    # how visible is each lane?
    l_vis = (self.lll_prob * 0.5 + 0.5) * interp(self.lll_std, [0, 0.9], [1.0, 0.0])
    r_vis = (self.rll_prob * 0.5 + 0.5) * interp(self.rll_std, [0, 0.9], [1.0, 0.0])
    lane_trust = max(l_vis, r_vis)
    # make sure we have something with lanelines to work with
    # otherwise, we will default to laneless, unfortunately
    if lane_trust > 0.025:
      # which lane are we closer to?
      distance = self.rll_y[0] - self.lll_y[0]
      right_ratio = self.rll_y[0] / distance
      # give preference of lane by visibility and closeness, but make sure it is non-zero
      l_prob = right_ratio * l_vis
      r_prob = (1.0 - right_ratio) * r_vis
      total_prob = l_prob + r_prob
      l_prob = l_prob / total_prob
      r_prob = r_prob / total_prob

      # Find current lanewidth
      current_lane_width = clamp(abs(min(self.rll_y[0], self.re_y[0]) - max(self.lll_y[0], self.le_y[0])), MIN_LANE_DISTANCE, MAX_LANE_DISTANCE)
      self.lane_width_estimate.update(current_lane_width)
      self.lane_width = self.lane_width_estimate.x

      # how much are we centered in our lane right now?
      starting_centering = (self.rll_y[0] + self.lll_y[0]) * 0.5
      # go through all points in our lanes...
      for index in range(len(self.lll_y)):
        # get the raw lane width for this point
        lane_width = self.rll_y[index] - self.lll_y[index]
        use_min_lane_distance = min(lane_width * 0.5, KEEP_MIN_DISTANCE_FROM_LANE)
        # how much do we trust this? we want to be seeing both pretty well
        width_trust = min(l_vis, r_vis)
        final_lane_width = lerp(self.lane_width, lane_width, width_trust)
        # ok, get ideal point from each lane
        ideal_left = self.lll_y[index] + final_lane_width * 0.5
        ideal_right = self.rll_y[index] - final_lane_width * 0.5
        # make sure these points are not going too close or through the other lane
        ideal_left = clamp(ideal_left, self.lll_y[index] + use_min_lane_distance, self.rll_y[index] - use_min_lane_distance)
        ideal_right = clamp(ideal_right, self.lll_y[index] + use_min_lane_distance, self.rll_y[index] - use_min_lane_distance)
        # merge them to get an ideal center point, based on which value we want to prefer
        ideal_point = lerp(ideal_left, ideal_right, r_prob)
        # how much room do we have at this point to wiggle within the lane?
        wiggle_room = final_lane_width * 0.5 - use_min_lane_distance
        # how much do we want to shift at this point for upcoming and/or immediate curve?
        shift = clamp(0.7 * sigmoid(vcurv, 3.0, -0.5), -wiggle_room, wiggle_room) if wiggle_room > 0.0 else 0.0
        # if we are shifted exactly how much we want, this should add to 0
        shift_diff = starting_centering + shift
        # so, if it was off, apply some post-shift to shift us further to correct our starting centering
        shift += shift_diff
        # apply that shift to our ideal point
        ideal_point += shift
        # finally do a sanity check that this point is still within the lane markings and our min/max values
        if abs(self.lll_y[index] - ideal_point) > abs(self.rll_y[index] - ideal_point):
          # point is closer to the right lane, apply right min lane distance clamp
          # while letting us path a little closer to the left lane (which is further away)
          ideal_point = clamp(ideal_point, self.lll_y[index] + use_min_lane_distance * 0.5, self.rll_y[index] - use_min_lane_distance)
        else:
          # point is closer to the left lane, apply left min lane distance clamp
          # while letting us path a little closer to the right lane (which is further away)
          ideal_point = clamp(ideal_point, self.lll_y[index] + use_min_lane_distance, self.rll_y[index] - use_min_lane_distance * 0.5)
        # apply a max distance away from our preferred lane
        if l_prob > r_prob:
          ideal_point = min(ideal_point, self.lll_y[index] + KEEP_MAX_DISTANCE_FROM_LANE)
        else:
          ideal_point = max(ideal_point, self.rll_y[index] - KEEP_MAX_DISTANCE_FROM_LANE)
        # add it to our ultimate path!
        self.ultimate_path[index] = ideal_point

      # do we want to mix in the model path a little bit if lanelines are going south?
      final_ultimate_path_mix = self.lane_change_multiplier * clamp(lane_trust * 1.4, 0.0, 1.0)

      safe_idxs = np.isfinite(self.ll_t)
      if safe_idxs[0]:
        path_xyz[:,1] = final_ultimate_path_mix * np.interp(path_t, self.ll_t[safe_idxs], self.ultimate_path[safe_idxs]) + (1 - final_ultimate_path_mix) * path_xyz[:,1]
    # apply camera offset after everything
    path_xyz[:, 1] += CAMERA_OFFSET

    return path_xyz
