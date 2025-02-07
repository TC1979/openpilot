#!/usr/bin/env python3
import os
import time
import numpy as np
from cereal import log
from opendbc.car.interfaces import ACCEL_MIN
from opendbc.car.toyota.values import ToyotaFlags
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU

if __name__ == '__main__':  # generating code
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

SOURCES = ['lead0', 'lead1', 'cruise', 'e2e']

X_DIM = 3
U_DIM = 1
PARAM_DIM = 7
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'
CITY_SPEED_LIMIT = 25  # ~55mph
CRUISING_SPEED = 5     # ~11mph


# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
COMFORT_BRAKE = 2.5
# STOP_DISTANCE = 6.0

def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.relaxed:
    return 1.0
  elif personality==log.LongitudinalPersonality.standard:
    return 1.0
  elif personality==log.LongitudinalPersonality.aggressive:
    return 0.3
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.relaxed:
    return 1.8
  elif personality==log.LongitudinalPersonality.standard:
    return 1.3
  elif personality==log.LongitudinalPersonality.aggressive:
    return 0.8
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_dynamic_follow(v_ego, personality=log.LongitudinalPersonality.standard):
  # The Dynamic follow function is adjusted by Marc(cgw1968-5779)
  if personality==log.LongitudinalPersonality.relaxed:
    x_vel =  [0.,  3.0, 3.01, 10., 10.01, 15., 27.7]
    y_dist = [1.0, 1.0, 1.2,  1.2, 1.45,  1.6, 1.80]
  elif personality==log.LongitudinalPersonality.standard:
    x_vel =  [0.,  3.0, 3.01, 10., 10.01, 15., 27.7]
    y_dist = [0.8, 0.8, 1.0,  1.0, 1.2,   1.4, 1.45]
  elif personality==log.LongitudinalPersonality.aggressive:
    x_vel =  [0.,   3.0, 3.01, 10., 10.01, 15., 27.7]
    y_dist = [0.75, 0.75, 0.8, 0.8, 0.9,   0.9, 1.05]
  else:
    raise NotImplementedError("Dynamic Follow personality not supported")
  return np.interp(v_ego, x_vel, y_dist)


def get_STOP_DISTANCE(personality=log.LongitudinalPersonality.standard):
  if personality==log.LongitudinalPersonality.relaxed:
    return 4.5
  elif personality==log.LongitudinalPersonality.standard:
    return 4.0
  elif personality==log.LongitudinalPersonality.aggressive:
    return 3.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_stopped_equivalence_factor(v_lead, v_ego):
  v_diff_offset_max = 12  # Max additional offset distance
  delta_speed = v_lead - v_ego  # Relative speed of lead vs. ego

  v_diff_offset = np.zeros_like(delta_speed)  # Ensures proper shape

  mask = delta_speed > 0  # Only apply logic when lead is pulling away

  if np.any(mask):
    # 🔧 **Stronger Low-Speed Acceleration Scaling**
    scaling_factor = np.interp(v_ego, [0, 1, 3, 5, 9, 22], [2.0, 2.0, 0.95, 0.86, 0.82, 0.82])
    v_diff_offset[mask] = delta_speed[mask] * scaling_factor
    v_diff_offset = np.clip(v_diff_offset, 0, v_diff_offset_max)

    # 🔧 **Reduce Ego Speed Scaling Effect at Low Speeds**
    ego_scaling = np.interp(v_ego, [0, 1, 3, 5, 20], [1.5, 1.3, 1.1, 1.0, 1.0])
    v_diff_offset *= ego_scaling

  stopping_distance = (v_lead**2) / (2 * COMFORT_BRAKE) + v_diff_offset
  return stopping_distance

def get_safe_obstacle_distance(v_ego, t_follow, stop_distance=None):
  if stop_distance is None:
    stop_distance = get_STOP_DISTANCE()
  return (v_ego**2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + stop_distance

def desired_follow_distance(v_ego, v_lead, t_follow=None, stop_distance=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  if stop_distance is None:
    stop_distance = get_STOP_DISTANCE()
  return get_safe_obstacle_distance(v_ego, t_follow, stop_distance) - get_stopped_equivalence_factor(v_lead, v_ego)


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # set up states & controls
  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  stop_distance = SX.sym('stop_distance')
  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow, lead_danger_factor, stop_distance)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  prev_a = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]
  stop_distance = ocp.model.p[6]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow, stop_distance)

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep. This obstacle can be a lead car
  # or other object. In e2e mode we can use x_position targets as a cost
  # instead.
  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - prev_a,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints on speed, acceleration and desired distance to
  # the obstacle, which is treated as a slack constraint so it
  # behaves like an asymmetrical cost.
  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, get_T_FOLLOW(), LEAD_DANGER_FACTOR, get_STOP_DISTANCE()])


  # We put all constraint cost weights to 0 and only set them at runtime
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  # The HPIPM solver can give decent solutions even when it is stopped early
  # Which is critical for our purpose where compute time is strictly bounded
  # We use HPIPM in the SPEED_ABS mode, which ensures fastest runtime. This
  # does not cause issues since the problem is well bounded.
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  # More iterations take too much time and less lead to inaccurate convergence in
  # some situations. Ideally we would run just 1 iteration to ensure fixed runtime.
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, CP, mode='acc', dt=DT_MDL):
    self.CP = CP
    self.braking_offset = 1
    self.mode = mode
    self.dt = dt
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = SOURCES[2]

  def reset(self):
    # self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.solver.reset()
    # self.solver.options_set('print_level', 2)
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)
    self.yref = np.zeros((N+1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N,1))
    self.params = np.zeros((N+1, PARAM_DIM))
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))
    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # TODO don't hardcode A_CHANGE_COST idx
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    # Setting the slice without the copy make the array not contiguous,
    # causing issues with the C interface.
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    # Set L2 slack cost on lower bound constraints
    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard, v_lead0=0, v_lead1=0, distance_to_lead=100):
    jerk_factor = get_jerk_factor(personality)
    jerk_factor /= np.mean(self.braking_offset)
    v_ego = self.x0[1]

    # Default values for dynamic scaling factors
    j_ego_v_ego = 1
    a_change_v_ego = 1
    v_ego_bps = [0, 10]
    v_lead_speed = max(v_lead0, v_lead1)  # Get the highest lead speed
    # v_gap = v_lead_speed - v_ego  # Speed difference
    gap_factor = np.clip(distance_to_lead / 20, 0.5, 1.0)  # Reduce acceleration if too close to lead
    # Prevent ego from accelerating too much if lead is slow
    if (v_lead0 - v_ego >= 0) and (v_lead1 - v_ego >= 0) and v_lead_speed > 10:
      j_ego_v_ego = np.interp(v_ego, v_ego_bps, [0.10, 1.0])
      a_change_v_ego = np.interp(v_ego, v_ego_bps, [0.10, 1.0]) * gap_factor  # Reduce if too close

      # If following distance is small, reduce acceleration force more aggressively
      if distance_to_lead < 15:  # If too close, further reduce acceleration
        a_change_v_ego *= 0.4  # More aggressive reduction when close
        j_ego_v_ego *= 0.5    # Even further reduction in jerk when close

      # Keep the following distance above a threshold
      if distance_to_lead < 10:  # If we are really close
        a_change_v_ego = max(a_change_v_ego * 0.3, 0.2)  # Do not allow full throttle when too close to lead

    if self.mode == 'acc':
      a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0
      cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST, jerk_factor * a_change_cost * a_change_v_ego, jerk_factor * J_EGO_COST * j_ego_v_ego]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else 0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost * a_change_v_ego, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:  # probably only helps if v < v_prev
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      # Fake a fast lead car, so mpc can keep running in the same mode
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    # MPC will not converge if immediate crash is expected
    # Clip lead distance to what is still possible to brake for
    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = np.clip(x_lead, min_x_lead, 1e8)
    v_lead = np.clip(v_lead, 0.0, 1e8)
    a_lead = np.clip(a_lead, -10., 5.)
    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
    return lead_xv

  def set_accel_limits(self, min_a, max_a):
    # TODO this sets a max accel limit, but the minimum limit is only for cruise decel
    # needs refactor
    self.cruise_min_a = min_a
    self.max_a = max_a

  def update(self, radarstate, v_cruise, x, v, a, j, personality=log.LongitudinalPersonality.standard, dynamic_follow=False):
    t_follow = get_T_FOLLOW(personality)
    v_ego = self.x0[1]
    t_follow = get_T_FOLLOW(personality) if not dynamic_follow else get_dynamic_follow(v_ego, personality)
    if hasattr(self, 'braking_offset'):
      t_follow /= self.braking_offset
    stop_distance = get_STOP_DISTANCE(personality) * np.clip(1.0 / (0.5 * self.braking_offset + 0.5), 0.95, 1.05)
    if not (self.CP.flags & ToyotaFlags.SMART_DSU):
      stop_distance += 0.5

    if Params().get_bool("ToyotaTune"):
      stop_distance += 0.5

    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)
    lead = radarstate.leadOne

    # 常量定義
    BRAKING_THRESHOLDS = {
      'LEAD_DISTANCE': {'MIN': 5.0, 'MAX': 40.0, 'CRITICAL': 10.0},
      'SPEED': {'LOW': 5.0, 'HIGH': 12.0, 'MAX': 20.0},
      'BRAKING': {'MIN_OFFSET': 1.2, 'MAX_OFFSET': 3.0, 'RELATIVE_SPEED': 1.5},
      'SAFETY': {'MIN_FACTOR': 0.7, 'MAX_FACTOR': 1.8, 'MIN_MODEL_PROB': 0.75}
    }

    SMOOTHING_PARAMS = {
      'BASE_FACTOR': 0.3, 'SPEED_SCALE': 0.07, 'REL_SPEED_SCALE': 0.07,
      'MIN_SMOOTH': 0.05, 'MAX_SMOOTH': 0.4, 'COMFORT_SCALE': 0.2
    }

    lead_distance = lead_xv_0[0, 0]
    v_lead = lead_xv_0[0, 1]
    rel_speed = v_lead - v_ego
    lead_prob = getattr(lead, "modelProb", 1.0)

    self.smoother_braking = (lead.status and self.mode == 'acc' and
                             BRAKING_THRESHOLDS['LEAD_DISTANCE']['MIN'] < lead_distance <= BRAKING_THRESHOLDS['LEAD_DISTANCE']['MAX'] and
                             v_ego < BRAKING_THRESHOLDS['SPEED']['MAX'] and
                             abs(rel_speed) < BRAKING_THRESHOLDS['BRAKING']['RELATIVE_SPEED'] and
                             lead_prob > BRAKING_THRESHOLDS['SAFETY']['MIN_MODEL_PROB'])

    if self.smoother_braking:
      # 速度影響因子
      speed_scale = np.clip((v_ego - BRAKING_THRESHOLDS['SPEED']['LOW']) /
                            (BRAKING_THRESHOLDS['SPEED']['HIGH'] - BRAKING_THRESHOLDS['SPEED']['LOW']), 0, 1)

      # 相對速度影響因子
      relative_speed_factor = np.clip(rel_speed / BRAKING_THRESHOLDS['BRAKING']['RELATIVE_SPEED'], -1, 1)

      # 臨界距離修正
      if lead_distance < BRAKING_THRESHOLDS['LEAD_DISTANCE']['CRITICAL']:
        critical_ratio = (BRAKING_THRESHOLDS['LEAD_DISTANCE']['CRITICAL'] - lead_distance) / BRAKING_THRESHOLDS['LEAD_DISTANCE']['CRITICAL']
        critical_factor = np.tanh(critical_ratio * 5) * 0.15
        safety_factor = 1 + critical_factor * (1 - speed_scale * 0.7) * (1 + abs(relative_speed_factor) * 0.06)
      else:
        safety_factor = 1.0

      # 保持合理的跟車距離
      follow_distance_ratio = lead_distance / max(v_ego * min(t_follow, 2.0), 1.0)
      safety_factor = 0.9 * safety_factor + 0.1 * np.clip(follow_distance_ratio, BRAKING_THRESHOLDS['SAFETY']['MIN_FACTOR'], BRAKING_THRESHOLDS['SAFETY']['MAX_FACTOR'])

      # 舒適制動調整
      comfort_brake_adjustment = COMFORT_BRAKE * (1 + SMOOTHING_PARAMS['COMFORT_SCALE'] * abs(relative_speed_factor))

      # 動態平滑因子
      dynamic_smooth_factor = np.clip(SMOOTHING_PARAMS['BASE_FACTOR'] * (1 - 0.3 * (speed_scale + abs(relative_speed_factor))),
                                      SMOOTHING_PARAMS['MIN_SMOOTH'], SMOOTHING_PARAMS['MAX_SMOOTH'])

      # 更新剎車補償
      new_braking_offset = np.clip(self.braking_offset * safety_factor,
                             BRAKING_THRESHOLDS['BRAKING']['MIN_OFFSET'],
                             BRAKING_THRESHOLDS['BRAKING']['MAX_OFFSET'])
      self.braking_offset = np.clip((1 - dynamic_smooth_factor) * self.braking_offset +
                              dynamic_smooth_factor * new_braking_offset,
                              1.2, 2.5)

      t_follow = np.clip(t_follow / (0.3 * self.braking_offset + 0.7), 0.95 * t_follow, 1.1 * t_follow)

    # To estimate a safe distance from a moving lead, we calculate how much stopping
    # distance that lead needs as a minimum. We can add that to the current distance
    # and then treat that as a stopped car/obstacle at this new distance.
    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1], v_ego)
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1], v_ego)

    self.params[:,0] = ACCEL_MIN
    # negative accel constraint causes problems because negative speed is not allowed
    self.params[:,1] = max(0.0, self.max_a)

    # Update in ACC mode or ACC/e2e blend
    if self.mode == 'acc':
      self.params[:,5] = LEAD_DANGER_FACTOR

      # Fake an obstacle for cruise, this ensures smooth acceleration to set speed
      # when the leads are no factor.
      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      # TODO does this make sense when max_a is negative?
      v_upper = v_ego + (T_IDXS * self.max_a * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                                 v_lower,
                                 v_upper)
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow, stop_distance)
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
      self.source = SOURCES[np.argmin(x_obstacles[0])]

      # These are not used in ACC mode
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

    elif self.mode == 'blended':
      self.params[:,5] = 1.0

      x_obstacles = np.column_stack([lead_0_obstacle,
                                     lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1,0] < x_and_cruise[1,1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    self.yref[:,1] = x
    self.yref[:,2] = v
    self.yref[:,3] = a
    self.yref[:,5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.prev_a)
    self.params[:,4] = self.params[:,4] = np.clip(t_follow / self.braking_offset, 0.8 * t_follow, 1.2 * t_follow)
    self.params[:,6] = stop_distance

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    # Check if it got within lead comfort range
    # TODO This should be done cleaner
    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow, stop_distance))- self.x_sol[:,0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow, stop_distance))- self.x_sol[:,0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = 'lead1'

  def run(self):
    # t0 = time.monotonic()
    # reset = 0
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    # qp_iter = self.solver.get_stats('statistics')[-1][-1] # SQP_RTI specific
    # print(f"long_mpc timings: tot {self.solve_time:.2e}, qp {self.time_qp_solution:.2e}, lin {self.time_linearization:.2e}, \
    # integrator {self.time_integrator:.2e}, qp_iter {qp_iter}")
    # res = self.solver.get_residuals()
    # print(f"long_mpc residuals: {res[0]:.2e}, {res[1]:.2e}, {res[2]:.2e}, {res[3]:.2e}")
    # self.solver.print_statistics()

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.prev_a = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()
      # reset = 1
    # print(f"long_mpc timings: total internal {self.solve_time:.2e}, external: {(time.monotonic() - t0):.2e} qp {self.time_qp_solution:.2e}, \
    # lin {self.time_linearization:.2e} qp_iter {qp_iter}, reset {reset}")


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
