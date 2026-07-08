"""串口传输层。

定义统一的 Transport 接口,底下两种实现:
  - SerialTransport:真实 UART(pyserial)
  - VirtualTransport:虚拟回环,内部跑 PID + 被控对象仿真,
    对上层表现得跟真串口一模一样(收发同样的文本协议)。

上层(UI / 采集线程)只依赖 Transport 接口,切换真实/仿真
只需换一个实现,其余代码不动。
"""
from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from typing import Optional

from core.pid import PID
from core.plant import SecondOrderPlant
from comms import protocol


class Transport(ABC):
    """收发字节的抽象传输通道。"""

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def write(self, data: bytes) -> None: ...

    @abstractmethod
    def readline(self, timeout: float = 0.1) -> Optional[str]:
        """读一行文本(不含换行),无数据返回 None。"""
        ...

    @property
    @abstractmethod
    def is_open(self) -> bool: ...


class SerialTransport(Transport):
    """基于 pyserial 的真实串口实现。"""

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._ser = None

    def open(self) -> None:
        import serial  # 延迟导入,虚拟模式下无需安装 pyserial
        self._ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def write(self, data: bytes) -> None:
        if self._ser is not None:
            self._ser.write(data)

    def readline(self, timeout: float = 0.1) -> Optional[str]:
        if self._ser is None:
            return None
        self._ser.timeout = timeout
        raw = self._ser.readline()
        if not raw:
            return None
        try:
            return raw.decode("ascii", errors="ignore").strip()
        except Exception:
            return None

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open


class VirtualTransport(Transport):
    """虚拟回环:内部以固定步长运行 PID + 被控对象仿真。

    模拟一块跑着 PID 的单片机:接收 S/T/R/M 命令,
    以 sample_dt 周期吐出 D 数据行。所有交互都走文本协议,
    因此上层无法区分它和真串口。
    """

    def __init__(self, sample_dt: float = 0.02, plant: Optional[SecondOrderPlant] = None):
        self.sample_dt = sample_dt
        self._plant = plant or SecondOrderPlant()
        self._pid = PID(kp=1.0, ki=0.0, kd=0.0, out_min=-10.0, out_max=10.0)

        self._setpoint = 0.0
        self._mode = 0            # 0=停止 1=PID 2=直接输出
        self._direct_u = 0.0      # 模式 2 下的直接输出量
        self._measurement = 0.0
        self._t0 = 0.0

        self._rx_queue: list[str] = []       # 待上层读取的数据行
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._alive = False

    def open(self) -> None:
        self._alive = True
        self._t0 = time.time()
        self._plant.reset()
        self._pid.reset()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._alive = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _run(self) -> None:
        next_t = time.time()
        while self._alive:
            now = time.time()
            if now < next_t:
                time.sleep(min(0.005, next_t - now))
                continue
            next_t += self.sample_dt

            if self._mode == 1:
                u = self._pid.update(self._setpoint, self._measurement, self.sample_dt)
                self._measurement = self._plant.step(u, self.sample_dt)
            elif self._mode == 2:
                u = self._direct_u
                self._measurement = self._plant.step(u, self.sample_dt)
            else:
                u = 0.0

            t_ms = int((time.time() - self._t0) * 1000)
            line = f"D,{t_ms},{self._setpoint:.6f},{self._measurement:.6f},{u:.6f}"
            with self._lock:
                self._rx_queue.append(line)
                # 防止上层不读时无限堆积
                if len(self._rx_queue) > 5000:
                    self._rx_queue = self._rx_queue[-2500:]

    def write(self, data: bytes) -> None:
        """解析下行命令并作用到内部仿真。"""
        try:
            text = data.decode("ascii", errors="ignore")
        except Exception:
            return
        for line in text.splitlines():
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            cmd = parts[0]
            try:
                if cmd == "S" and len(parts) == 4:
                    self._pid.set_gains(float(parts[1]), float(parts[2]), float(parts[3]))
                elif cmd == "T" and len(parts) == 2:
                    self._setpoint = float(parts[1])
                elif cmd == "R":
                    self._pid.reset()
                    self._plant.reset()
                    self._measurement = 0.0
                elif cmd == "M" and len(parts) == 2:
                    m = int(parts[1])
                    self._mode = m if m in (0, 1, 2) else 0
                elif cmd == "O" and len(parts) == 2:
                    self._direct_u = float(parts[1])
            except (ValueError, IndexError):
                continue

    def readline(self, timeout: float = 0.1) -> Optional[str]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._rx_queue:
                    return self._rx_queue.pop(0)
            time.sleep(0.002)
        return None

    @property
    def is_open(self) -> bool:
        return self._alive
