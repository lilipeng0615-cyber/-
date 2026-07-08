"""采集线程与自整定回路。

AcquisitionWorker 跑在后台 QThread 里:
  - 持续从 Transport 读数据行,解析成 DataPoint,通过信号发给界面
  - 承载继电反馈自整定回路:读 measurement -> 算继电器输出 ->
    用 O 命令直接下发,收敛后算出 PID 参数并发信号

界面层只连接信号槽,不直接碰串口和算法,避免阻塞 UI 线程。
"""
from __future__ import annotations

from PySide6.QtCore import QThread, Signal

from comms import protocol
from comms.transport import Transport
from core.autotune import RelayAutoTuner, TuneState


class AcquisitionWorker(QThread):
    """后台采集 + 自整定线程。"""

    # 收到一条上行采样: (t_ms, setpoint, measurement, output)
    data_received = Signal(int, float, float, float)
    # 自整定状态提示文本
    tune_status = Signal(str)
    # 自整定完成: 参数字典 {kp,ki,kd,ku,tu,rule}
    tune_done = Signal(dict)
    # 自整定失败
    tune_failed = Signal(str)
    # 连接断开/异常
    error = Signal(str)

    def __init__(self, transport: Transport, parent=None):
        super().__init__(parent)
        self._transport = transport
        self._alive = False

        # 自整定相关
        self._tuner: RelayAutoTuner | None = None
        self._tuning = False
        self._tune_setpoint = 0.0
        self._tune_rule = "classic_pid"
        self._last_t_ms: int | None = None

    # ---- 生命周期 ----

    def stop(self) -> None:
        self._alive = False

    def run(self) -> None:
        self._alive = True
        while self._alive:
            try:
                line = self._transport.readline(timeout=0.1)
            except Exception as exc:  # noqa: BLE001
                self.error.emit(f"读取异常: {exc}")
                break
            if line is None:
                continue

            dp = protocol.decode_line(line)
            if dp is None:
                continue

            self.data_received.emit(dp.t_ms, dp.setpoint, dp.measurement, dp.output)

            if self._tuning and self._tuner is not None:
                self._step_tuning(dp)

    # ---- 自整定 ----

    def start_autotune(
        self,
        setpoint: float,
        relay_amp: float,
        hysteresis: float,
        bias: float,
        rule: str = "classic_pid",
    ) -> None:
        """启动继电反馈自整定。

        bias 为工作点偏置 u0,使输出绕设定值上下振荡(≈ setpoint/被控增益)。
        """
        self._tuner = RelayAutoTuner(
            relay_amp=relay_amp,
            hysteresis=hysteresis,
        )
        self._tuner.start(setpoint, bias=bias)
        self._tune_setpoint = setpoint
        self._tune_rule = rule
        self._last_t_ms = None
        self._tuning = True

        # 切到直接输出模式,给一个初始偏置输出
        self._transport.write(protocol.encode_set_target(setpoint))
        self._transport.write(protocol.encode_mode(2))
        self._transport.write(protocol.encode_output(bias))
        self.tune_status.emit("自整定中: 等待极限环振荡...")

    def cancel_autotune(self) -> None:
        if self._tuning:
            self._tuning = False
            self._transport.write(protocol.encode_output(0.0))
            self._transport.write(protocol.encode_mode(0))
            self.tune_status.emit("自整定已取消")

    def _step_tuning(self, dp) -> None:
        tuner = self._tuner
        assert tuner is not None

        # 用上行时间戳算真实 dt,回退到标称值
        if self._last_t_ms is None:
            dt = 0.02
        else:
            dt = max(1e-4, (dp.t_ms - self._last_t_ms) / 1000.0)
        self._last_t_ms = dp.t_ms

        u = tuner.update(dp.measurement, dt)
        self._transport.write(protocol.encode_output(u))

        if tuner.state == TuneState.DONE:
            self._tuning = False
            self._transport.write(protocol.encode_output(0.0))
            self._transport.write(protocol.encode_mode(0))
            try:
                gains = tuner.compute_gains(self._tune_rule)
            except Exception as exc:  # noqa: BLE001
                self.tune_failed.emit(f"参数计算失败: {exc}")
                return
            self.tune_done.emit(gains)
        elif tuner.state == TuneState.FAILED:
            self._tuning = False
            self._transport.write(protocol.encode_output(0.0))
            self._transport.write(protocol.encode_mode(0))
            self.tune_failed.emit("未能形成稳定振荡,请调整继电幅值/迟滞后重试")
