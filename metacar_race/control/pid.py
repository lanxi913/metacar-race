# -*- coding: utf-8 -*-
"""一个够用的 PID，带输出限幅和积分限幅。

纯跟踪给转向、速度环给油门都能用。用法：

    pid = PID(kp=0.8, ki=0.05, kd=0.1, out_min=-1.0, out_max=1.0)
    u = pid(目标值 - 当前值, dt)
"""


class PID:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, out_min=-1.0, out_max=1.0,
                 i_min=-1.0, i_max=1.0):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_min = float(out_min)
        self.out_max = float(out_max)
        self.i_min = float(i_min)
        self.i_max = float(i_max)
        self._integral = 0.0
        self._prev_err = None

    def reset(self):
        self._integral = 0.0
        self._prev_err = None

    def __call__(self, err, dt):
        if dt is None or dt <= 0.0:
            dt = 1e-3

        # 积分限幅：防止长时间偏差把积分项撑爆
        self._integral += err * dt
        self._integral = max(self.i_min, min(self.i_max, self._integral))

        if self._prev_err is None:
            derivative = 0.0
        else:
            derivative = (err - self._prev_err) / dt
        self._prev_err = err

        out = self.kp * err + self.ki * self._integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, out))
