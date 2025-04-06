import numpy as np
import math

from openpilot.common.realtime import DT_CTRL

ALPHA_MIN = 0.004
ALPHA_MAX = 0.4

class DisturbanceController:
  def __init__(self):
    self.lowpass_filtered = 0.0
    self.alpha_prev = ALPHA_MIN
    self.desired_curvature_prev = 0.0
    self.pid = PIDController(1, 1, k_f=0, pos_limit=0.2, neg_limit=-0.2)

  def reset(self):
    self.lowpass_filtered = 0.0
    self.alpha_prev = ALPHA_MIN
    self.desired_curvature_prev = 0.0
    self.pid.reset()

  def compute_dynamic_alpha(self, desired_curvature, dt=DT_CTRL, A=0.02, n=2.0, beta=3.0, k=2.0):
    d_desired = abs(desired_curvature - self.desired_curvature_prev) / dt
    alpha_reactive = d_desired**n / (k * A) if A > 0 else 0.0
    alpha = np.clip(self.alpha_prev * np.exp(-beta * dt) + alpha_reactive, ALPHA_MIN, ALPHA_MAX)
    self.alpha_prev = alpha
    self.desired_curvature_prev = desired_curvature
    return alpha

  def lowpass_filter(self, current_value, alpha):
    alpha = min(alpha, ALPHA_MAX)
    if alpha >= ALPHA_MAX * 0.9:
      reset_factor = (alpha - ALPHA_MIN) / (ALPHA_MAX - ALPHA_MIN)
      self.lowpass_filtered = (1 - reset_factor) * self.lowpass_filtered + reset_factor * current_value
    else:
      self.lowpass_filtered = (1 - alpha) * self.lowpass_filtered + alpha * current_value
    return self.lowpass_filtered

  def highpass_filter(self, current_value, lowpass_value):
    return current_value - lowpass_value

  def compensate(self, desired_curvature, actual_curvature):
    alpha = self.compute_dynamic_alpha(desired_curvature)
    reaction = self.lowpass_filter(actual_curvature, alpha)
    disturbance = self.highpass_filter(actual_curvature, reaction)

    error = desired_curvature - disturbance
    output_curvature = self.pid.update(error, feedforward=desired_curvature, speed=CS.vEgo)
    
    return output_curvature
