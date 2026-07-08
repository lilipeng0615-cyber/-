"""虚拟被控对象(仿真模型)。

用于没有真实硬件时测试上位机。默认是一个带纯滞后的
二阶惯性环节,比较接近电机转速 / 温度这类常见对象。
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field


@dataclass
class SecondOrderPlant:
    """二阶惯性 + 纯滞后对象。

    连续模型近似:
        G(s) = K / ((tau1*s + 1)(tau2*s + 1)) * e^{-L s}
    离散化用简单前向欧拉,对仿真够用。
    """

    gain: float = 1.0        # 稳态增益 K
    tau1: float = 0.8        # 时间常数 1 (秒)
    tau2: float = 0.4        # 时间常数 2 (秒)
    delay: float = 0.1       # 纯滞后 L (秒)
    noise: float = 0.0       # 输出高斯噪声标准差

    _y1: float = field(default=0.0, init=False)
    _y2: float = field(default=0.0, init=False)
    _delay_buf: deque = field(default=None, init=False)
    _dt: float = field(default=0.0, init=False)

    def reset(self) -> None:
        self._y1 = 0.0
        self._y2 = 0.0
        self._delay_buf = None
        self._dt = 0.0

    def _ensure_buffer(self, dt: float) -> None:
        if self._delay_buf is None or dt != self._dt:
            n = max(1, int(round(self.delay / dt))) if self.delay > 0 else 0
            self._delay_buf = deque([0.0] * n, maxlen=n) if n > 0 else deque(maxlen=0)
            self._dt = dt

    def step(self, u: float, dt: float) -> float:
        """输入控制量 u,推进 dt 秒,返回被控量 y。"""
        self._ensure_buffer(dt)

        # 纯滞后:输入先进队列
        if self._delay_buf.maxlen and self._delay_buf.maxlen > 0:
            self._delay_buf.append(u)
            u_eff = self._delay_buf[0]
        else:
            u_eff = u

        # 两级一阶惯性串联(前向欧拉)
        self._y1 += dt / self.tau1 * (self.gain * u_eff - self._y1)
        self._y2 += dt / self.tau2 * (self._y1 - self._y2)

        y = self._y2
        if self.noise > 0.0:
            import random
            y += random.gauss(0.0, self.noise)
        return y
