"""主窗口。

布局:
  左侧  实时曲线(setpoint / measurement / output)
  右侧  连接、PID 参数、目标值、自整定面板

界面只通过 Transport 接口和 AcquisitionWorker 信号与底层交互,
真实串口 / 虚拟仿真 只在连接时二选一。
"""
from __future__ import annotations

import csv
import json
from collections import deque
from pathlib import Path

import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from comms import protocol
from comms.transport import SerialTransport, VirtualTransport
from ui.worker import AcquisitionWorker

MAX_POINTS = 2000  # 曲线保留的最大采样点数


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID 自动调参上位机")
        self.resize(1100, 640)

        self._transport = None
        self._worker: AcquisitionWorker | None = None

        # 曲线数据缓冲
        self._t = deque(maxlen=MAX_POINTS)
        self._sp = deque(maxlen=MAX_POINTS)
        self._meas = deque(maxlen=MAX_POINTS)
        self._out = deque(maxlen=MAX_POINTS)

        self._build_ui()

        # 定时刷新曲线,避免每个数据点都重绘
        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._refresh_plot)
        self._plot_timer.start(50)

    # ---- UI 构建 ----

    def _build_ui(self) -> None:
        central = QWidget()
        root = QHBoxLayout(central)

        # 左:曲线
        self._plot = pg.PlotWidget()
        self._plot.setBackground("w")
        self._plot.showGrid(x=True, y=True, alpha=0.3)
        self._plot.addLegend()
        self._plot.setLabel("bottom", "时间", units="s")
        self._curve_sp = self._plot.plot(pen=pg.mkPen("#888", width=2, style=Qt.DashLine), name="设定值")
        self._curve_meas = self._plot.plot(pen=pg.mkPen("#c0392b", width=2), name="测量值")
        self._curve_out = self._plot.plot(pen=pg.mkPen("#2980b9", width=1), name="输出")
        root.addWidget(self._plot, stretch=3)

        # 右:控制面板
        panel = QVBoxLayout()
        panel.addWidget(self._build_conn_group())
        panel.addWidget(self._build_pid_group())
        panel.addWidget(self._build_tune_group())
        panel.addStretch(1)
        self._status = QLabel("未连接")
        self._status.setWordWrap(True)
        panel.addWidget(self._status)

        panel_w = QWidget()
        panel_w.setLayout(panel)
        panel_w.setFixedWidth(320)
        root.addWidget(panel_w, stretch=0)

        self.setCentralWidget(central)

    def _build_conn_group(self) -> QGroupBox:
        box = QGroupBox("连接")
        form = QFormLayout(box)

        self._conn_mode = QComboBox()
        self._conn_mode.addItems(["虚拟仿真", "真实串口"])
        self._conn_mode.currentIndexChanged.connect(self._on_conn_mode_changed)
        form.addRow("模式", self._conn_mode)

        self._port_combo = QComboBox()
        self._port_combo.setEnabled(False)
        self._refresh_ports()
        form.addRow("串口", self._port_combo)

        self._refresh_btn = QPushButton("刷新串口")
        self._refresh_btn.setEnabled(False)
        self._refresh_btn.clicked.connect(self._refresh_ports)
        form.addRow(self._refresh_btn)

        self._baud_combo = QComboBox()
        self._baud_combo.addItems(["9600", "57600", "115200", "230400", "460800"])
        self._baud_combo.setCurrentText("115200")
        self._baud_combo.setEnabled(False)
        form.addRow("波特率", self._baud_combo)

        self._connect_btn = QPushButton("连接")
        self._connect_btn.clicked.connect(self._toggle_connection)
        form.addRow(self._connect_btn)

        return box

    def _build_pid_group(self) -> QGroupBox:
        box = QGroupBox("PID 参数 / 运行")
        form = QFormLayout(box)

        self._kp = self._make_spin(0.0, 1000.0, 0.01, 1.0)
        self._ki = self._make_spin(0.0, 1000.0, 0.01, 0.0)
        self._kd = self._make_spin(0.0, 1000.0, 0.01, 0.0)
        self._setpoint = self._make_spin(-1000.0, 1000.0, 0.1, 1.0)
        form.addRow("Kp", self._kp)
        form.addRow("Ki", self._ki)
        form.addRow("Kd", self._kd)
        form.addRow("目标值", self._setpoint)

        btns = QHBoxLayout()
        self._send_btn = QPushButton("下发参数")
        self._send_btn.clicked.connect(self._send_gains)
        self._run_btn = QPushButton("启动")
        self._run_btn.setCheckable(True)
        self._run_btn.toggled.connect(self._toggle_run)
        self._reset_btn = QPushButton("复位")
        self._reset_btn.clicked.connect(self._reset)
        btns.addWidget(self._send_btn)
        btns.addWidget(self._run_btn)
        btns.addWidget(self._reset_btn)
        wrap = QWidget()
        wrap.setLayout(btns)
        form.addRow(wrap)

        file_btns = QHBoxLayout()
        self._save_profile_btn = QPushButton("保存参数")
        self._save_profile_btn.clicked.connect(self._save_profile)
        self._load_profile_btn = QPushButton("加载参数")
        self._load_profile_btn.clicked.connect(self._load_profile)
        self._export_btn = QPushButton("导出CSV")
        self._export_btn.clicked.connect(self._export_csv)
        file_btns.addWidget(self._save_profile_btn)
        file_btns.addWidget(self._load_profile_btn)
        file_btns.addWidget(self._export_btn)
        file_wrap = QWidget()
        file_wrap.setLayout(file_btns)
        form.addRow(file_wrap)

        self._set_controls_enabled(False)
        return box

    def _build_tune_group(self) -> QGroupBox:
        box = QGroupBox("自整定 (继电反馈 + Ziegler-Nichols)")
        form = QFormLayout(box)

        self._relay_amp = self._make_spin(0.01, 100.0, 0.1, 1.0)
        self._hyst = self._make_spin(0.0, 10.0, 0.01, 0.05)
        self._bias = self._make_spin(-100.0, 100.0, 0.1, 1.0)
        form.addRow("继电幅值 d", self._relay_amp)
        form.addRow("迟滞", self._hyst)
        form.addRow("工作点偏置 u0", self._bias)

        self._rule_combo = QComboBox()
        self._rule_combo.addItems(
            ["classic_pid", "pessen", "some_overshoot", "no_overshoot", "pi", "p"]
        )
        form.addRow("整定规则", self._rule_combo)

        self._tune_btn = QPushButton("开始自整定")
        self._tune_btn.setCheckable(True)
        self._tune_btn.toggled.connect(self._toggle_autotune)
        form.addRow(self._tune_btn)

        self._tune_result = QLabel("Ku / Tu: —")
        self._tune_result.setWordWrap(True)
        form.addRow(self._tune_result)

        self._tune_group = box
        box.setEnabled(False)
        return box

    @staticmethod
    def _make_spin(lo, hi, step, val) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(4)
        s.setValue(val)
        return s

    # ---- 连接管理 ----

    def _refresh_ports(self) -> None:
        current = self._port_combo.currentText() if hasattr(self, "_port_combo") else ""
        self._port_combo.clear()
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
        except Exception:
            ports = []
        self._port_combo.addItems(ports or ["(无可用串口)"])
        if current in ports:
            self._port_combo.setCurrentText(current)

    def _on_conn_mode_changed(self, idx: int) -> None:
        real = idx == 1
        self._port_combo.setEnabled(real)
        self._baud_combo.setEnabled(real)
        self._refresh_btn.setEnabled(real)
        if real:
            self._refresh_ports()

    def _toggle_connection(self) -> None:
        if self._transport is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        try:
            if self._conn_mode.currentIndex() == 1:
                port = self._port_combo.currentText()
                if not port or port.startswith("("):
                    QMessageBox.warning(self, "连接", "没有可用串口")
                    return
                baud = int(self._baud_combo.currentText())
                self._transport = SerialTransport(port, baud)
            else:
                self._transport = VirtualTransport()
            self._transport.open()
        except Exception as exc:  # noqa: BLE001
            self._transport = None
            QMessageBox.critical(self, "连接失败", str(exc))
            return

        self._worker = AcquisitionWorker(self._transport)
        self._worker.data_received.connect(self._on_data)
        self._worker.tune_status.connect(self._on_tune_status)
        self._worker.tune_done.connect(self._on_tune_done)
        self._worker.tune_failed.connect(self._on_tune_failed)
        self._worker.error.connect(self._on_worker_error)
        self._worker.start()

        self._connect_btn.setText("断开")
        self._conn_mode.setEnabled(False)
        self._set_controls_enabled(True)
        self._tune_group.setEnabled(True)
        self._status.setText("已连接")
        self._clear_data()

    def _disconnect(self) -> None:
        if self._worker is not None:
            self._worker.stop()
            self._worker.wait(1000)
            self._worker = None
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:
                pass
            self._transport = None
        self._connect_btn.setText("连接")
        self._conn_mode.setEnabled(True)
        self._set_controls_enabled(False)
        self._tune_group.setEnabled(False)
        self._run_btn.setChecked(False)
        self._tune_btn.setChecked(False)
        self._status.setText("未连接")

    def _set_controls_enabled(self, on: bool) -> None:
        for w in (self._send_btn, self._run_btn, self._reset_btn):
            w.setEnabled(on)

    # ---- PID 操作 ----

    def _send_gains(self) -> None:
        if self._transport is None:
            return
        self._transport.write(
            protocol.encode_set_gains(self._kp.value(), self._ki.value(), self._kd.value())
        )
        self._transport.write(protocol.encode_set_target(self._setpoint.value()))
        self._status.setText("已下发参数")

    def _toggle_run(self, on: bool) -> None:
        if self._transport is None:
            return
        if on:
            self._send_gains()
            self._transport.write(protocol.encode_mode(1))
            self._run_btn.setText("停止")
            self._status.setText("运行中")
        else:
            self._transport.write(protocol.encode_mode(0))
            self._run_btn.setText("启动")
            self._status.setText("已停止")

    def _reset(self) -> None:
        if self._transport is None:
            return
        self._transport.write(protocol.encode_reset())
        self._clear_data()
        self._status.setText("已复位")

    # ---- 参数预设与数据导出 ----

    def _profile_data(self) -> dict:
        return {
            "kp": self._kp.value(),
            "ki": self._ki.value(),
            "kd": self._kd.value(),
            "setpoint": self._setpoint.value(),
            "relay_amp": self._relay_amp.value(),
            "hysteresis": self._hyst.value(),
            "bias": self._bias.value(),
            "rule": self._rule_combo.currentText(),
        }

    def _apply_profile(self, data: dict) -> None:
        self._kp.setValue(float(data.get("kp", self._kp.value())))
        self._ki.setValue(float(data.get("ki", self._ki.value())))
        self._kd.setValue(float(data.get("kd", self._kd.value())))
        self._setpoint.setValue(float(data.get("setpoint", self._setpoint.value())))
        self._relay_amp.setValue(float(data.get("relay_amp", self._relay_amp.value())))
        self._hyst.setValue(float(data.get("hysteresis", self._hyst.value())))
        self._bias.setValue(float(data.get("bias", self._bias.value())))
        rule = str(data.get("rule", self._rule_combo.currentText()))
        idx = self._rule_combo.findText(rule)
        if idx >= 0:
            self._rule_combo.setCurrentIndex(idx)

    def _save_profile(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self,
            "保存 PID 参数预设",
            str(Path.home() / "pid_profile.json"),
            "PID 参数预设 (*.json);;JSON 文件 (*.json)",
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self._profile_data(), f, ensure_ascii=False, indent=2)
        except OSError as exc:
            QMessageBox.critical(self, "保存失败", str(exc))
            return
        self._status.setText(f"参数预设已保存: {path}")

    def _load_profile(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "加载 PID 参数预设",
            str(Path.home()),
            "PID 参数预设 (*.json);;JSON 文件 (*.json)",
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self._apply_profile(data)
        except (OSError, ValueError, TypeError) as exc:
            QMessageBox.critical(self, "加载失败", str(exc))
            return
        self._status.setText(f"参数预设已加载: {path}")

    def _export_csv(self) -> None:
        if not self._t:
            QMessageBox.information(self, "导出CSV", "当前还没有可导出的曲线数据")
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "导出曲线数据",
            str(Path.home() / "pid_curve.csv"),
            "CSV 文件 (*.csv)",
        )
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "setpoint", "measurement", "output"])
                writer.writerows(zip(self._t, self._sp, self._meas, self._out))
        except OSError as exc:
            QMessageBox.critical(self, "导出失败", str(exc))
            return
        self._status.setText(f"已导出 {len(self._t)} 个采样点: {path}")

    # ---- 自整定 ----

    def _toggle_autotune(self, on: bool) -> None:
        if self._transport is None or self._worker is None:
            return
        if on:
            self._run_btn.setChecked(False)  # 自整定与手动运行互斥
            self._clear_data()
            self._worker.start_autotune(
                setpoint=self._setpoint.value(),
                relay_amp=self._relay_amp.value(),
                hysteresis=self._hyst.value(),
                bias=self._bias.value(),
                rule=self._rule_combo.currentText(),
            )
            self._tune_btn.setText("取消自整定")
        else:
            self._worker.cancel_autotune()
            self._tune_btn.setText("开始自整定")

    def _on_tune_status(self, text: str) -> None:
        self._status.setText(text)

    def _on_tune_done(self, gains: dict) -> None:
        self._kp.setValue(gains["kp"])
        self._ki.setValue(gains["ki"])
        self._kd.setValue(gains["kd"])
        self._tune_result.setText(
            f"Ku={gains['ku']:.4f}  Tu={gains['tu']:.4f}\n"
            f"Kp={gains['kp']:.4f} Ki={gains['ki']:.4f} Kd={gains['kd']:.4f}"
        )
        self._tune_btn.setChecked(False)
        self._tune_btn.setText("开始自整定")
        self._status.setText("自整定完成,参数已填入。可下发并启动验证。")

    def _on_tune_failed(self, msg: str) -> None:
        self._tune_btn.setChecked(False)
        self._tune_btn.setText("开始自整定")
        QMessageBox.warning(self, "自整定失败", msg)
        self._status.setText("自整定失败")

    def _on_worker_error(self, msg: str) -> None:
        self._status.setText(f"错误: {msg}")

    # ---- 数据与绘图 ----

    def _on_data(self, t_ms: int, sp: float, meas: float, out: float) -> None:
        self._t.append(t_ms / 1000.0)
        self._sp.append(sp)
        self._meas.append(meas)
        self._out.append(out)

    def _clear_data(self) -> None:
        for buf in (self._t, self._sp, self._meas, self._out):
            buf.clear()

    def _refresh_plot(self) -> None:
        if not self._t:
            return
        t = list(self._t)
        self._curve_sp.setData(t, list(self._sp))
        self._curve_meas.setData(t, list(self._meas))
        self._curve_out.setData(t, list(self._out))

    def closeEvent(self, event) -> None:
        self._disconnect()
        super().closeEvent(event)


def main() -> None:
    app = QApplication.instance() or QApplication([])
    win = MainWindow()
    win.show()
    app.exec()


if __name__ == "__main__":
    main()
