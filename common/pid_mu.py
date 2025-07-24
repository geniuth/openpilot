from openpilot.common.pid import PIDController
import numpy as np

class MultiplicativeUnwindPID(PIDController):
  def update(self, error, error_rate=0.0, speed=0.0, override=False, feedforward=0., freeze_integrator=False):
    self.speed = speed

    self.p = float(error) * self.k_p
    self.f = feedforward * self.k_f
    self.d = error_rate * self.k_d

    if override:
      # Multiplikativer Abbau von i
      self.i *= (1.0 - self.i_unwind_rate)
      if abs(self.i) < 1e-10:
        self.i = 0.0
    else:
      if not freeze_integrator:
        self.i += error * self.k_i * self.i_rate

        # Clip i to prevent exceeding control limits
        control_no_i = self.p + self.d + self.f
        control_no_i = np.clip(control_no_i, self.neg_limit, self.pos_limit)
        self.i = np.clip(self.i, self.neg_limit - control_no_i, self.pos_limit - control_no_i)

    control = self.p + self.i + self.d + self.f
    self.control = np.clip(control, self.neg_limit, self.pos_limit)
    return self.control
