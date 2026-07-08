"""继电反馈自整定(Relay feedback + Ziegler-Nichols)。

原理:
  用一个继电器(bang-bang)代替 PID 驱动对象,使系统进入
  稳定的极限环振荡。测出振荡周期 Tu 和幅值,推出临界增益 Ku:
        Ku = 4d / (pi * a)
  其中 d 是继电器幅值,a 是振荡幅值。
  再用 Ziegler-Nichols 经验公式换算 PID 参数。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
import math


class TuneState(Enum):
    IDLE = auto()
    RUNNING = auto()
    DONE = auto()
    FAILED = auto()


@dataclass
class RelayAutoTuner:
    setpoint: float = 0.0
    relay_amp: float = 1.0     # 继电器幅值 d
    hysteresis: float = 0.05   # 迟滞,抗噪声
    target_cycles: int = 6     # 采集多少个完整周期后判定收敛
    bias: float = 0.0          # 工作点偏置 u0,使输出绕设定值上下振荡

    state: TuneState = field(default=TuneState.IDLE, init=False)
    _relay_sign: int = field(default=1, init=False)
    _t: float = field(default=0.0, init=False)
    _last_cross_t: float = field(default=0.0, init=False)
    _periods: list = field(default_factory=list, init=False)
    _peaks: list = field(default_factory=list, init=False)
    _troughs: list = field(default_factory=list, init=False)
    _cur_max: float = field(default=-1e18, init=False)
    _cur_min: float = field(default=1e18, init=False)

    ku: float = field(default=0.0, init=False)
    tu: float = field(default=0.0, init=False)

    def start(self, setpoint: float, bias: float = 0.0) -> None:
        self.setpoint = setpoint
        self.bias = bias
        self.state = TuneState.RUNNING
        self._relay_sign = 1
        self._t = 0.0
        self._last_cross_t = 0.0
        self._periods.clear()
        self._peaks.clear()
        self._troughs.clear()
        self._cur_max = -1e18
        self._cur_min = 1e18

    def update(self, measurement: float, dt: float) -> float:
        """返回继电器输出的控制量。整定完成后 state 变为 DONE。"""
        if self.state != TuneState.RUNNING:
            return 0.0

        self._t += dt
        error = self.setpoint - measurement

        # 跟踪当前半周期的极值
        self._cur_max = max(self._cur_max, measurement)
        self._cur_min = min(self._cur_min, measurement)

        # 带迟滞的继电器切换
        switched = False
        if self._relay_sign > 0 and error < -self.hysteresis:
            self._relay_sign = -1
            switched = True
            self._peaks.append(self._cur_max)
            self._cur_max = -1e18
        elif self._relay_sign < 0 and error > self.hysteresis:
            self._relay_sign = 1
            switched = True
            self._troughs.append(self._cur_min)
            self._cur_min = 1e18

        if switched:
            # 一个完整周期 = 两次同向切换之间,这里用相邻切换间隔的 2 倍
            if self._last_cross_t > 0.0:
                half_period = self._t - self._last_cross_t
                self._periods.append(2.0 * half_period)
            self._last_cross_t = self._t
            self._check_convergence()

        return self.bias + self._relay_sign * self.relay_amp

    def _check_convergence(self) -> None:
        if len(self._periods) < self.target_cycles:
            return
        # 取后半段稳定的周期与幅值求平均
        recent = self._periods[-self.target_cycles // 2:]
        self.tu = sum(recent) / len(recent)

        if not self._peaks or not self._troughs:
            self.state = TuneState.FAILED
            return
        amp = (
            sum(self._peaks[-3:]) / len(self._peaks[-3:])
            - sum(self._troughs[-3:]) / len(self._troughs[-3:])
        ) / 2.0
        if amp <= 1e-6 or self.tu <= 1e-6:
            self.state = TuneState.FAILED
            return

        self.ku = (4.0 * self.relay_amp) / (math.pi * amp)
        self.state = TuneState.DONE

    def compute_gains(self, rule: str = "classic_pid") -> dict:
        """按 Ziegler-Nichols 表把 Ku/Tu 换算成 PID 参数。"""
        if self.state != TuneState.DONE:
            raise RuntimeError("自整定尚未完成,无法计算参数")

        ku, tu = self.ku, self.tu
        rules = {
            # rule: (Kp, Ti, Td) 系数
            "classic_pid": (0.6 * ku, 0.5 * tu, 0.125 * tu),
            "pessen":      (0.7 * ku, 0.4 * tu, 0.15 * tu),
            "some_overshoot": (0.33 * ku, 0.5 * tu, 0.33 * tu),
            "no_overshoot":   (0.2 * ku, 0.5 * tu, 0.33 * tu),
            "pi":          (0.45 * ku, 0.83 * tu, 0.0),
            "p":           (0.5 * ku, 0.0, 0.0),
        }
        kp, ti, td = rules[rule]
        ki = kp / ti if ti > 0 else 0.0
        kd = kp * td
        return {"kp": kp, "ki": ki, "kd": kd, "ku": ku, "tu": tu, "rule": rule}
