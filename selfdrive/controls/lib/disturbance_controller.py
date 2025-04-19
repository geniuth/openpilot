import numpy as np
import math

from collections import deque
from openpilot.common.pid import PIDController
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_CURVATURE

ALPHA_MIN = 0.004  # baseline LP rate
ALPHA_MAX = 0.4    # fastest LP rate

# Disturbance Observer (wind lateral force)
OBS_TAU = 0.20     # [s] filter constant (1st order LP on Fy_hat)
OBS_K   = 7.0      # observer gain (>> 1) – higher -> faster -> noisier

# Band pass (≈ 0.5 ... 15 Hz) for dynamic alpha
BP_FC_HP = 0.5      # high pass corner [Hz]
BP_ALPHA_HP = (1.0 / (2.0 * math.pi * BP_FC_HP * DT_CTRL))
BP_ALPHA_HP = BP_ALPHA_HP / (1.0 + BP_ALPHA_HP)  # pre warp for 1st order
KE_ENERGY  = 0.25   # scaling from |ay_bp| to alpha boost

# PID gains
PID_KP = 1.0
PID_KI = 0.0 #0.05  # small I to cancel steady wind offset
PID_KF = 0.0

FF_GAIN = 0.2

class DisturbanceController:
  """Wind disturbance compensator using
  * 3DoF curvature estimate
  * 1st order Disturbance Observer (Fy_hat)
  * adaptive LP/HP separation with band pass energy
  """
  
  def __init__(self, CP):
    self.lowpass_filtered = 0.0
    self.alpha_prev = ALPHA_MIN
    self.desired_curvature_prev = 0.0
    self.pid = PIDController(PID_KP, PID_KI, k_f=PID_KF, pos_limit=MAX_CURVATURE, neg_limit=-MAX_CURVATURE)
    self.reaction_hist = deque([0.0], maxlen=int(round(CP.steerActuatorDelay / DT_CTRL)) + 1) # Actuator delay compensation
    self.Fy_hat = 0.0  # Disturbance observer state: estimated lateral wind force [N]
    self.ay_hp  = 0.0  # Band pass filter states (simple 1st order HP + LP energy)
    self.ay_prev = 0.0

  def reset(self):
    self.lowpass_filtered = 0.0
    self.alpha_prev = ALPHA_MIN
    self.desired_curvature_prev = 0.0
    self.pid.reset()
    self.reaction_hist.clear()
    self.reaction_hist.append(0.0)
    self.Fy_hat = 0.0
    self.ay_hp  = 0.0
    self.ay_prev = 0.0

  def _update_bandpass_energy(self, ay_meas):
    """High pass filter to isolate wind böe frequency content (≥ 0.5 Hz)."""
    # 1st order HP: y[n] = alpha*(y[n_1] + x[n] - x[n_1])
    self.ay_hp = BP_ALPHA_HP * (self.ay_hp + ay_meas - self.ay_prev)
    self.ay_prev = ay_meas
    return abs(self.ay_hp)

  def _compute_dynamic_alpha(self, energy, dt=DT_CTRL):
    # baseline exponential decay
    alpha = self.alpha_prev * math.exp(-3.0 * dt)
    # energy based boost
    alpha += KE_ENERGY * energy
    alpha = float(np.clip(alpha, ALPHA_MIN, ALPHA_MAX))
    self.alpha_prev = alpha
    return alpha

  def _lowpass_filter(self, current_value, alpha):
    if alpha >= ALPHA_MAX * 0.9:
      reset_factor = (alpha - ALPHA_MIN) / (ALPHA_MAX - ALPHA_MIN)
      self.lowpass_filtered = (1.0 - reset_factor) * self.lowpass_filtered + reset_factor * current_value
    else:
      self.lowpass_filtered = (1.0 - alpha) * self.lowpass_filtered + alpha * current_value
    return self.lowpass_filtered

  @staticmethod
  def _highpass_filter(current_value, lowpass_value):
    return current_value - lowpass_value

  def compensate(self, CS, VM, params, calibrated_pose, desired_curvature):
    """Return curvature command with wind compensation."""

    if calibrated_pose is None:
      return desired_curvature

    v_ego = CS.vEgo
    if v_ego < 0.1:
      return desired_curvature

    # Build actual curvature from 3DoF inverse model
    steering_angle_wo_offset = math.radians(CS.steeringAngleDeg - params.angleOffsetDeg)
    ay_meas = calibrated_pose.acceleration.y
    ay_long = calibrated_pose.acceleration.x
    yaw_rate = calibrated_pose.angular_velocity.yaw

    actual_curvature = -VM.calc_curvature_3dof(ay_meas, ay_long, yaw_rate,
                                               v_ego, steering_angle_wo_offset, 0.0)

    # Disturbance observer (1st order) -> ay_wind_est
    ay_cmd = desired_curvature * v_ego * v_ego
    ay_wind = ay_meas - ay_cmd

    # Fy_hat dynamics: F = -F/tau + k*(m*ay_wind)
    m = VM.m
    self.Fy_hat += DT_CTRL * (-self.Fy_hat / OBS_TAU + OBS_K * (m * ay_wind))
    ay_wind_est = self.Fy_hat / m

    # immediate feed forward curvature correction
    curv_ff = -ay_wind_est / (v_ego * v_ego + 1e-3) * FF_GAIN
    desired_curvature_ff = desired_curvature + curv_ff

    # LP/HP separation with adaptive alpha (band pass energy)
    energy = self._update_bandpass_energy(ay_meas)
    alpha = self._compute_dynamic_alpha(energy)

    reaction = self._lowpass_filter(actual_curvature, alpha)
    self.reaction_hist.append(reaction)
    disturbance = self._highpass_filter(actual_curvature, reaction)

    # compensate actuator delay (use earliest lp value in deque)
    reaction_delayed = self.reaction_hist[0]

    # PID – track curvature with disturbance rejection
    error = desired_curvature_ff - (reaction_delayed + disturbance)
    output_curvature = self.pid.update(error, feedforward=desired_curvature_ff, speed=v_ego)

    return float(output_curvature)
