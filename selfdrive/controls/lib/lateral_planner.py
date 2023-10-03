import numpy as np
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.lane_planner import LanePlanner
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, MIN_SPEED, get_speed_error
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log

TRAJECTORY_SIZE = 33
CAMERA_OFFSET = 0.04

class LateralPlanner:
  def __init__(self, CP, debug=False, use_lanelines=True):
    self.use_lanelines = use_lanelines
    self.LP = LanePlanner()
    self.DH = DesireHelper()

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros((TRAJECTORY_SIZE,))
    self.v_plan = np.zeros((TRAJECTORY_SIZE,))
    self.x_sol = np.zeros((TRAJECTORY_SIZE, 4), dtype=np.float32)
    self.v_ego = 0.0
    self.l_lane_change_prob = 0.0
    self.r_lane_change_prob = 0.0
    self.d_path_w_lines_xyz = np.zeros((TRAJECTORY_SIZE, 3))

    self.debug_mode = debug

    self.dynamic_lane_profile = Params().get_bool("dynamic_lane")
    self.dynamic_lane_profile_status = False
    self.dynamic_lane_profile_status_buffer = False

    self.standstill_elapsed = 0.0
    self.standstill = False

  def update(self, sm):
    self.dynamic_lane_profile = Params().get_bool("dynamic_lane")
    self.standstill = sm['carState'].standstill
    # TODO: do something for 0 speed
    # TODO: is a small first order filter needed here?
    v_ego_car = sm['carState'].vEgo

    # Parse model predictions
    md = sm['modelV2']
    self.LP.parse_model(md)
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE and len(md.lateralPlannerSolution.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.plan_yaw = np.array(md.orientation.z)
      self.plan_yaw_rate = np.array(md.orientationRate.z)
      self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
      car_speed = np.linalg.norm(self.velocity_xyz, axis=1) - get_speed_error(md, v_ego_car)
      self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
      self.v_ego = self.v_plan[0]
      # YOLO e2e planning
      self.x_sol = np.column_stack([md.lateralPlannerSolution.x, md.lateralPlannerSolution.y, md.lateralPlannerSolution.yaw, md.lateralPlannerSolution.yawRate])

    # Lane change logic
    lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, md)

    # Turn off lanes during lane change
    if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
      self.LP.lll_prob *= self.DH.lane_change_ll_prob
      self.LP.rll_prob *= self.DH.lane_change_ll_prob

    low_speed = v_ego_car < 10 * CV.MPH_TO_MS

    if not self.get_dynamic_lane_profile() and not low_speed:
      d_path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)
      self.dynamic_lane_profile_status = False
    else:
      d_path_xyz = self.path_xyz
      self.dynamic_lane_profile_status = True
    self.path_xyz = d_path_xyz

  def get_dynamic_lane_profile(self):
    if not self.dynamic_lane_profile:
      return True
    else:
      # only while lane change is off
      if self.DH.lane_change_state == log.LateralPlan.LaneChangeState.off:
        # laneline probability too low, we switch to laneless mode
        if (self.LP.lll_prob + self.LP.rll_prob) / 2 < 0.3:
          self.dynamic_lane_profile_status_buffer = True
        if (self.LP.lll_prob + self.LP.rll_prob) / 2 > 0.5:
          self.dynamic_lane_profile_status_buffer = False
        if self.dynamic_lane_profile_status_buffer:  # in buffer mode, always laneless
          return True
    return False

  def publish(self, sm, pm):
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.laneWidth = float(self.LP.lane_width)
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.psis = self.x_sol[0:CONTROL_N, 2].tolist()

    lateralPlan.curvatures = (self.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(0) for _ in range(CONTROL_N-1)] # TODO: unused
    lateralPlan.lProb = float(self.LP.lll_prob)
    lateralPlan.rProb = float(self.LP.rll_prob)
    lateralPlan.dProb = float(self.LP.d_prob)


    lateralPlan.mpcSolutionValid = bool(1)
    lateralPlan.solverExecutionTime = 0.0
    if self.debug_mode:
      lateralPlan.solverState = log.LateralPlan.SolverState.new_message()
      lateralPlan.solverState.x = self.x_sol.tolist()

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = self.use_lanelines
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction

    lateralPlan.dPathWLinesX = [float(x) for x in self.d_path_w_lines_xyz[:, 0]]
    lateralPlan.dPathWLinesY = [float(y) for y in self.d_path_w_lines_xyz[:, 1]]

    if self.standstill:
      self.standstill_elapsed += DT_MDL
    else:
      self.standstill_elapsed = 0.0
    lateralPlan.standstillElapsed = int(self.standstill_elapsed)

    lateralPlan.dynamicLaneProfile = True

    pm.send('lateralPlan', plan_send)
