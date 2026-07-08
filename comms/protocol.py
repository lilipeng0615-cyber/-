"""上位机 <-> 下位机通信协议(编解码)。

采用简单的文本行协议,便于调试和移植到单片机:
  上行(下位机 -> 上位机),每个采样点一行:
      D,<t_ms>,<setpoint>,<measurement>,<output>\n
  下行(上位机 -> 下位机)命令:
      S,<kp>,<ki>,<kd>\n        设置 PID 参数
      T,<setpoint>\n            设置目标值
      R\n                       复位/清零
      M,<0|1|2>\n               模式: 0=手动/停止 1=PID运行 2=直接输出
      O,<u>\n                   直接给定输出量(自整定继电反馈用,需模式2)

文本协议对单片机端只需 sscanf/sprintf,零依赖。若追求带宽/抗噪,
后续可换成带 CRC 的二进制帧,这里保留 encode/decode 边界不变即可。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class DataPoint:
    """一条上行采样数据。"""
    t_ms: int
    setpoint: float
    measurement: float
    output: float


def encode_set_gains(kp: float, ki: float, kd: float) -> bytes:
    return f"S,{kp:.6f},{ki:.6f},{kd:.6f}\n".encode("ascii")


def encode_set_target(setpoint: float) -> bytes:
    return f"T,{setpoint:.6f}\n".encode("ascii")


def encode_reset() -> bytes:
    return b"R\n"


def encode_mode(mode: int) -> bytes:
    """0=停止 1=PID运行 2=直接输出。兼容旧调用:传 bool 也可。"""
    m = int(bool(mode)) if isinstance(mode, bool) else int(mode)
    return f"M,{m}\n".encode("ascii")


def encode_output(u: float) -> bytes:
    """直接给定输出量(继电反馈自整定用,需先切到模式 2)。"""
    return f"O,{u:.6f}\n".encode("ascii")


def decode_line(line: str) -> Optional[DataPoint]:
    """解析一行上行数据。非数据行(空行/命令回显/异常)返回 None。"""
    line = line.strip()
    if not line or not line.startswith("D,"):
        return None
    parts = line.split(",")
    if len(parts) != 5:
        return None
    try:
        return DataPoint(
            t_ms=int(parts[1]),
            setpoint=float(parts[2]),
            measurement=float(parts[3]),
            output=float(parts[4]),
        )
    except (ValueError, IndexError):
        return None
