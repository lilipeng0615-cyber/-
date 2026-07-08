"""离散 PID 控制器。

位置式实现,带积分限幅(抗饱和)和输出限幅。
与 UI 无关,可单独导入测试。
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class PID:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0

    out_min: float = -1e9
    out_max: float = 1e9

    # 内部状态
    _integral: float = field(default=0.0, init=False)
    _prev_error: float = field(default=0.0, init=False)
    _initialized: bool = field(default=False, init=False)

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialized = False

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        """执行一步控制,返回控制量 u。dt 为采样周期(秒)。"""
        if dt <= 0.0:
            return self._clamp(self.kp * (setpoint - measurement))

        error = setpoint - measurement

        # 微分项用第一次调用时不算,避免启动冲击
        if not self._initialized:
            self._prev_error = error
            self._initialized = True

        derivative = (error - self._prev_error) / dt

        # 积分项预累加
        integral_candidate = self._integral + error * dt
        u_unclamped = (
            self.kp * error
            + self.ki * integral_candidate
            + self.kd * derivative
        )

        u = self._clamp(u_unclamped)

        # 抗积分饱和:仅当输出未饱和,或积分方向有助于退出饱和时才累加
        if u == u_unclamped or (u_unclamped - u) * error < 0:
            self._integral = integral_candidate

        self._prev_error = error
        return u

    def _clamp(self, value: float) -> float:
        return max(self.out_min, min(self.out_max, value))
