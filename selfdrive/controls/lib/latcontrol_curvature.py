#import math
#import numpy as np
#
#from cereal import log
#from opendbc.car.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
#from openpilot.selfdrive.controls.lib.latcontrol import LatControl
#from openpilot.common.pid import PIDController
#
#
#class LatControlCurvature(LatControl):
#  def __init__(self, CP, CI):
#    super().__init__(CP, CI)
#    self.curvature_params = CP.lateralTuning.curvature.as_builder()
#    self.pid = PIDController(self.curvature_params.kp, self.curvature_params.ki,
#                             k_f=self.curvature_params.kf, pos_limit=self.curvature_max, neg_limit=-self.curvature_max)
#
#  def update(self, active, CS, VM, params, steer_limited_by_controls, desired_curvature, calibrated_pose, curvature_limited):
#    pid_log = log.ControlsState.LateralCurvatureState.new_message()
#    if not active:
#      output_curvature = 0.0
#      pid_log.active = False
#    else:
#      actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
#      #roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
#
#      assert calibrated_pose is not None
#      actual_curvature_pose = calibrated_pose.angular_velocity.yaw / CS.vEgo
#      actual_curvature = np.interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_pose])
#
#      pid_log.error = float(desired_curvature - actual_curvature)
#
#      freeze_integrator = steer_limited_by_controls or CS.steeringPressed or CS.vEgo < 5
#      output_curvature = self.pid.update(pid_log.error, feedforward=desired_curvature, speed=CS.vEgo, freeze_integrator=freeze_integrator)
#
#      pid_log.active = True
#      pid_log.p = float(self.pid.p)
#      pid_log.i = float(self.pid.i)
#      pid_log.d = float(self.pid.d)
#      pid_log.f = float(self.pid.f)
#      pid_log.output = float(output_curvature)
#      pid_log.actualCurvature = float(actual_curvature)
#      pid_log.desiredCurvature = float(desired_curvature)
#      pid_log.saturated = bool(self._check_saturation(self.curvature_max - abs(output_curvature) < 1e-3, CS, steer_limited_by_controls, curvature_limited))
#
#    return 0.0, 0.0, pid_log
