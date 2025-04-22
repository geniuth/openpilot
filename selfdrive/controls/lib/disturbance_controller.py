import numpy as np
import math

from collections import deque
from openpilot.common.pid import PIDController
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_CURVATURE

ALPHA_MIN = 0.004
ALPHA_MAX = 0.4

class DisturbanceController:
  def __init__(self, CP):
    self.lowpass_filtered_prev = 0.0
    self.alpha_prev = ALPHA_MAX
    self.pid = PIDController(1, 0, k_f=1, pos_limit=MAX_CURVATURE, neg_limit=-MAX_CURVATURE)
    self.length_des_curv_hist = int(round(CP.steerActuatorDelay / DT_CTRL)) + 1
    self.desired_curvature_hist = deque([0.0], maxlen=self.length_des_curv_hist)

  def reset(self):
    self.lowpass_filtered_prev = 0.0
    self.alpha_prev = ALPHA_MAX
    self.pid.reset()
    self.desired_curvature_hist.clear()
    self.desired_curvature_hist.append(0.0)

  def compute_dynamic_alpha(self, desired_curvature_hist, alpha_prev, dt=DT_CTRL, A=0.02, n=2.0, beta=3.0, k=2.0):
    if len(desired_curvature_hist) < self.length_des_curv_hist:
      return ALPHA_MAX
    d_desired = abs(desired_curvature_hist[0] - desired_curvature_hist[1]) / dt
    alpha_reactive = d_desired**n / (k * A) if A > 0 else 0.0
    alpha = np.clip(alpha_prev * np.exp(-beta * dt) + alpha_reactive, ALPHA_MIN, ALPHA_MAX)
    return alpha

  def lowpass_filter(self, current_value, lowpass_filtered_prev, alpha):
    alpha = min(alpha, ALPHA_MAX)
    if alpha >= ALPHA_MAX * 0.9:
      reset_factor = (alpha - ALPHA_MIN) / (ALPHA_MAX - ALPHA_MIN)
      lowpass_filtered = (1 - reset_factor) * lowpass_filtered_prev + reset_factor * current_value
    else:
      lowpass_filtered = (1 - alpha) * lowpass_filtered_prev + alpha * current_value
    return lowpass_filtered

  def highpass_filter(self, current_value, lowpass_value):
    return current_value - lowpass_value

  def compensate(self, CS, VM, params, calibrated_pose, desired_curvature):
    if calibrated_pose is None or CS.vEgo < 0.1:
      return desired_curvature

    steering_angle_without_offset = math.radians(CS.steeringAngleDeg - params.angleOffsetDeg)
    actual_curvature = -VM.calc_curvature_3dof(calibrated_pose.acceleration.y, calibrated_pose.acceleration.x,
                                               calibrated_pose.angular_velocity.yaw, CS.vEgo, steering_angle_without_offset, 0.)
    
    alpha = self.compute_dynamic_alpha(self.desired_curvature_hist, self.alpha_prev)
    reaction = self.lowpass_filter(actual_curvature, self.lowpass_filtered_prev, alpha)
    disturbance = self.highpass_filter(actual_curvature, reaction)

    error = -disturbance
    output_curvature = self.pid.update(error, feedforward=desired_curvature, speed=CS.vEgo)

    self.desired_curvature_hist.append(desired_curvature)
    self.alpha_prev = alpha
    self.lowpass_filtered_prev = reaction
    
    return float(output_curvature)
