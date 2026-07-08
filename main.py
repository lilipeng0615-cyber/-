"""PID 自动调参上位机 —— 程序入口。

运行:
    cd pid_tuner
    python main.py

默认进入"虚拟仿真"模式,无需硬件即可体验:
连接后可手动下发 PID 参数、启动闭环,或一键继电反馈自整定。
接真实单片机时,连接模式选"真实串口"并选择端口/波特率即可。
"""
from __future__ import annotations

import sys
import os

# 允许直接 python main.py 运行(把项目根加入 sys.path)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ui.main_window import main

if __name__ == "__main__":
    main()
