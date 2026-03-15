from __future__ import annotations

# pyright: reportMissingTypeStubs=false, reportUnknownMemberType=false, reportUnknownArgumentType=false, reportUnknownVariableType=false, reportUnknownLambdaType=false

import csv
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, TextIO

import pyqtgraph as _pg  # type: ignore
import serial
from PySide6.QtCore import QThread, QTimer, Qt, Signal, Slot
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from serial.tools import list_ports

pg: Any = _pg


@dataclass
class TelemetryEvent:
    kind: str
    values: dict[str, Any]
    raw_line: str


class TelemetryParser:
    def __init__(self) -> None:
        self.patterns: list[tuple[str, re.Pattern[str]]] = [
            (
                "battery",
                re.compile(r"Battery Raw:\s*([-\d.]+)\s*V\s*\|\s*Filtered:\s*([-\d.]+)\s*V"),
            ),
            (
                "imu",
                re.compile(
                    r"Angle\(XYZ\):\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\|\s*"
                    r"Gyro\(XYZ\):\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\|\s*"
                    r"Acc\(XYZ\):\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)"
                ),
            ),
            (
                "encoder",
                re.compile(
                    r"Encoder1 Angle:\s*([-\d.]+)\s*Vel:\s*([-\d.]+)\s*\|\s*"
                    r"Encoder2 Angle:\s*([-\d.]+)\s*Vel:\s*([-\d.]+)"
                ),
            ),
            (
                "motor_basic",
                re.compile(
                    r"Torque\(V\) Target=([-\d.]+)\s*\|\s*"
                    r"M1 Angle:\s*([-\d.]+)\s*Vel:\s*([-\d.]+)\s*\|\s*"
                    r"M2 Angle:\s*([-\d.]+)\s*Vel:\s*([-\d.]+)"
                ),
            ),
            (
                "foc_result",
                re.compile(r"FOC Result -> M1:(-?\d+)\s*M2:(-?\d+)\s*\|\s*Ready:(YES|NO)"),
            ),
            (
                "foc_status",
                re.compile(r"FOC Ready:\s*(YES|NO)"),
            ),
            (
                "angle_loop",
                re.compile(r"Angle Target:\s*([-\d.]+)\s*\|\s*M1:\s*([-\d.]+)\s*\|\s*M2:\s*([-\d.]+)"),
            ),
            (
                "velocity_loop",
                re.compile(r"Velocity Target:\s*([-\d.]+)\s*\|\s*M1:\s*([-\d.]+)\s*\|\s*M2:\s*([-\d.]+)"),
            ),
            (
                "torque_sweep",
                re.compile(
                    r"Stage:(\d+)\s*\|\s*M1 Target:([-\d.]+)\s*Vel:([-\d.]+)\s*\|\s*"
                    r"M2 Target:([-\d.]+)\s*Vel:([-\d.]+)"
                ),
            ),
            (
                "openloop",
                re.compile(r"OpenLoop Stage:(\d+)\s*\|\s*M1 Cmd:([-\d.]+)\s*\|\s*M2 Cmd:([-\d.]+)"),
            ),
            (
                "manual_openloop",
                re.compile(r"Manual(?:\s+Command\s+Applied)?\s+OpenLoop\s*\|\s*M1 Cmd:([-\d.]+)\s*\|\s*M2 Cmd:([-\d.]+)"),
            ),
            (
                "test_start",
                re.compile(r">>> Starting Test\s*(\d+)\s*<<<"),
            ),
        ]

    def parse(self, line: str) -> Optional[TelemetryEvent]:
        text = line.strip()
        for kind, pattern in self.patterns:
            match = pattern.search(text)
            if not match:
                continue

            if kind == "battery":
                return TelemetryEvent(
                    kind,
                    {
                        "battery_raw": float(match.group(1)),
                        "battery_filtered": float(match.group(2)),
                    },
                    text,
                )

            if kind == "imu":
                return TelemetryEvent(
                    kind,
                    {
                        "angle_x": float(match.group(1)),
                        "angle_y": float(match.group(2)),
                        "angle_z": float(match.group(3)),
                        "gyro_x": float(match.group(4)),
                        "gyro_y": float(match.group(5)),
                        "gyro_z": float(match.group(6)),
                        "acc_x": float(match.group(7)),
                        "acc_y": float(match.group(8)),
                        "acc_z": float(match.group(9)),
                    },
                    text,
                )

            if kind == "encoder":
                return TelemetryEvent(
                    kind,
                    {
                        "encoder1_angle": float(match.group(1)),
                        "encoder1_velocity": float(match.group(2)),
                        "encoder2_angle": float(match.group(3)),
                        "encoder2_velocity": float(match.group(4)),
                    },
                    text,
                )

            if kind == "motor_basic":
                return TelemetryEvent(
                    kind,
                    {
                        "torque_target": float(match.group(1)),
                        "m1_angle": float(match.group(2)),
                        "m1_velocity": float(match.group(3)),
                        "m2_angle": float(match.group(4)),
                        "m2_velocity": float(match.group(5)),
                    },
                    text,
                )

            if kind == "foc_result":
                ready = match.group(3) == "YES"
                return TelemetryEvent(
                    kind,
                    {
                        "foc_m1_result": int(match.group(1)),
                        "foc_m2_result": int(match.group(2)),
                        "foc_ready": ready,
                    },
                    text,
                )

            if kind == "foc_status":
                return TelemetryEvent(kind, {"foc_ready": match.group(1) == "YES"}, text)

            if kind == "angle_loop":
                return TelemetryEvent(
                    kind,
                    {
                        "angle_target": float(match.group(1)),
                        "m1_angle": float(match.group(2)),
                        "m2_angle": float(match.group(3)),
                    },
                    text,
                )

            if kind == "velocity_loop":
                return TelemetryEvent(
                    kind,
                    {
                        "velocity_target": float(match.group(1)),
                        "m1_velocity": float(match.group(2)),
                        "m2_velocity": float(match.group(3)),
                    },
                    text,
                )

            if kind == "torque_sweep":
                return TelemetryEvent(
                    kind,
                    {
                        "stage": int(match.group(1)),
                        "m1_target": float(match.group(2)),
                        "m1_velocity": float(match.group(3)),
                        "m2_target": float(match.group(4)),
                        "m2_velocity": float(match.group(5)),
                    },
                    text,
                )

            if kind == "openloop":
                return TelemetryEvent(
                    kind,
                    {
                        "stage": int(match.group(1)),
                        "openloop_m1_cmd": float(match.group(2)),
                        "openloop_m2_cmd": float(match.group(3)),
                    },
                    text,
                )

            if kind == "manual_openloop":
                return TelemetryEvent(
                    kind,
                    {
                        "openloop_m1_cmd": float(match.group(1)),
                        "openloop_m2_cmd": float(match.group(2)),
                    },
                    text,
                )

            if kind == "test_start":
                return TelemetryEvent(kind, {"current_test": int(match.group(1))}, text)

        return None


class SerialWorker(QThread):
    line_received = Signal(str)
    status_changed = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, port: str, baudrate: int) -> None:
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self._serial: Optional[serial.Serial] = None
        self._running = True
        self._write_lock = threading.Lock()

    def run(self) -> None:
        try:
            serial_port = serial.Serial(self.port, self.baudrate, timeout=0.2)
            self._serial = serial_port
            self.status_changed.emit(f"已连接: {self.port} @ {self.baudrate}")
            while self._running:
                try:
                    raw = serial_port.readline()
                except serial.SerialException as exc:
                    self.error_occurred.emit(f"串口读取失败: {exc}")
                    break

                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    self.line_received.emit(line)
        except serial.SerialException as exc:
            self.error_occurred.emit(f"无法打开串口: {exc}")
        finally:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.close()
                except serial.SerialException:
                    pass
            self.status_changed.emit("串口已断开")

    def write_text(self, text: str) -> None:
        if not text:
            return
        with self._write_lock:
            if self._serial and self._serial.is_open:
                self._serial.write(text.encode("utf-8"))

    def stop(self) -> None:
        self._running = False
        with self._write_lock:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.close()
                except serial.SerialException:
                    pass


class MainWindow(QMainWindow):
    CSV_FIELDS = [
        "host_time",
        "msg_type",
        "current_test",
        "raw_line",
        "battery_raw",
        "battery_filtered",
        "angle_x",
        "angle_y",
        "angle_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "acc_x",
        "acc_y",
        "acc_z",
        "encoder1_angle",
        "encoder1_velocity",
        "encoder2_angle",
        "encoder2_velocity",
        "torque_target",
        "m1_angle",
        "m1_velocity",
        "m2_angle",
        "m2_velocity",
        "angle_target",
        "velocity_target",
        "stage",
        "m1_target",
        "m2_target",
        "openloop_m1_cmd",
        "openloop_m2_cmd",
        "manual_check_m1f",
        "manual_check_m1r",
        "manual_check_m2f",
        "manual_check_m2r",
        "manual_check_summary",
        "last_manual_command",
        "foc_m1_result",
        "foc_m2_result",
        "foc_ready",
    ]

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("轮腿机器人测试上位机")
        self.resize(1480, 920)

        self.parser = TelemetryParser()
        self.worker: Optional[SerialWorker] = None
        self.series: dict[str, dict[str, deque[float]]] = {}
        self.current_test = "-"
        self.start_time = time.perf_counter()
        self.csv_file: Optional[TextIO] = None
        self.csv_writer: Any = None

        pg.setConfigOptions(antialias=True)

        self.port_combo = QComboBox()
        self.baud_spin = QSpinBox()
        self.connect_button = QPushButton("连接串口")
        self.refresh_button = QPushButton("刷新串口")
        self.record_button = QPushButton("开始录制 CSV")
        self.stop_record_button = QPushButton("停止录制")
        self.send_input = QLineEdit()
        self.send_button = QPushButton("发送自定义命令")
        self.raw_log = QPlainTextEdit()
        self.status_label = QLabel("未连接")
        self.current_test_label = QLabel("-")
        self.foc_label = QLabel("未知")
        self.latest_line_label = QLabel("-")
        self.metric_labels: dict[str, QLabel] = {}
        self.curves: dict[str, Any] = {}
        self.direction_check_combos: dict[str, QComboBox] = {}
        self.direction_check_commands: dict[str, str] = {
            "manual_check_m1f": "M1F",
            "manual_check_m1r": "M1R",
            "manual_check_m2f": "M2F",
            "manual_check_m2r": "M2R",
        }
        self.direction_check_titles: dict[str, str] = {
            "manual_check_m1f": "M1 正转",
            "manual_check_m1r": "M1 反转",
            "manual_check_m2f": "M2 正转",
            "manual_check_m2r": "M2 反转",
        }
        self.manual_summary_label = QLabel("未开始")
        self.last_manual_command_label = QLabel("-")

        self._build_ui()
        self._refresh_ports()

        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self._refresh_plots)
        self.plot_timer.start(200)

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)

        root = QHBoxLayout(central)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        root.addWidget(splitter)

        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)
        control_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        serial_group = QGroupBox("串口")
        serial_form = QFormLayout(serial_group)
        self.baud_spin.setRange(1200, 3000000)
        self.baud_spin.setValue(115200)
        serial_form.addRow("端口", self.port_combo)
        serial_form.addRow("波特率", self.baud_spin)

        serial_buttons = QHBoxLayout()
        serial_buttons.addWidget(self.refresh_button)
        serial_buttons.addWidget(self.connect_button)
        serial_form.addRow(serial_buttons)

        status_group = QGroupBox("状态")
        status_form = QFormLayout(status_group)
        status_form.addRow("连接状态", self.status_label)
        status_form.addRow("当前测试", self.current_test_label)
        status_form.addRow("FOC状态", self.foc_label)
        status_form.addRow("最新串口行", self.latest_line_label)

        commands_group = QGroupBox("测试命令")
        commands_layout = QGridLayout(commands_group)
        button_specs = [
            ("0 停止", "0"),
            ("1 电池", "1"),
            ("2 IMU", "2"),
            ("3 舵机", "3"),
            ("4 编码器", "4"),
            ("5 FOC基础", "5"),
            ("6 FOC校准", "6"),
            ("7 位置闭环", "7"),
            ("8 速度闭环", "8"),
            ("9 力矩扫描", "9"),
            ("10 开环转动", "10"),
            ("11 手动模式", "11"),
            ("H 帮助", "h"),
        ]
        for index, (title, command) in enumerate(button_specs):
            button = QPushButton(title)
            button.clicked.connect(lambda _checked=False, cmd=command: self._send_command(cmd))
            commands_layout.addWidget(button, index // 2, index % 2)

        custom_group = QGroupBox("自定义命令")
        custom_layout = QHBoxLayout(custom_group)
        self.send_input.setPlaceholderText("例如: h、10、11、M1F")
        custom_layout.addWidget(self.send_input)
        custom_layout.addWidget(self.send_button)

        manual_group = self._build_manual_verification_group()

        record_group = QGroupBox("录制")
        record_layout = QHBoxLayout(record_group)
        record_layout.addWidget(self.record_button)
        record_layout.addWidget(self.stop_record_button)
        self.stop_record_button.setEnabled(False)

        metrics_group = QGroupBox("关键指标")
        metrics_form = QFormLayout(metrics_group)
        for name in [
            "battery_filtered",
            "angle_x",
            "angle_y",
            "angle_z",
            "gyro_y",
            "acc_z",
            "encoder1_angle",
            "encoder2_angle",
            "m1_angle",
            "m1_velocity",
            "m2_angle",
            "m2_velocity",
            "torque_target",
            "angle_target",
            "velocity_target",
            "openloop_m1_cmd",
            "openloop_m2_cmd",
            "stage",
        ]:
            label = QLabel("-")
            self.metric_labels[name] = label
            metrics_form.addRow(name, label)

        control_layout.addWidget(serial_group)
        control_layout.addWidget(status_group)
        control_layout.addWidget(commands_group)
        control_layout.addWidget(custom_group)
        control_layout.addWidget(manual_group)
        control_layout.addWidget(record_group)
        control_layout.addWidget(metrics_group)

        plots_tabs = QTabWidget()
        plots_tabs.addTab(self._build_overview_tab(), "实时曲线")
        plots_tabs.addTab(self._build_detail_tab(), "细节曲线")
        plots_tabs.addTab(self._build_raw_log_tab(), "原始日志")

        splitter.addWidget(control_panel)
        splitter.addWidget(plots_tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([320, 1120])

        self.refresh_button.clicked.connect(self._refresh_ports)
        self.connect_button.clicked.connect(self._toggle_connection)
        self.send_button.clicked.connect(self._send_custom_command)
        self.record_button.clicked.connect(self._start_recording)
        self.stop_record_button.clicked.connect(self._stop_recording)
        self._update_direction_summary()

    def _build_manual_verification_group(self) -> QGroupBox:
        group = QGroupBox("单电机手动控制 / 方向确认")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("动作"), 0, 0)
        layout.addWidget(QLabel("执行"), 0, 1)
        layout.addWidget(QLabel("判定"), 0, 2)

        direction_keys = [
            "manual_check_m1f",
            "manual_check_m1r",
            "manual_check_m2f",
            "manual_check_m2r",
        ]
        for row, key in enumerate(direction_keys, start=1):
            title = self.direction_check_titles[key]
            command = self.direction_check_commands[key]
            layout.addWidget(QLabel(title), row, 0)

            action_button = QPushButton("执行")
            action_button.clicked.connect(lambda _checked=False, cmd=command: self._send_manual_verification_command(cmd))
            layout.addWidget(action_button, row, 1)

            combo = QComboBox()
            combo.addItems(["未测", "正确", "错误"])
            combo.currentTextChanged.connect(lambda _text, state_key=key: self._update_direction_summary())
            layout.addWidget(combo, row, 2)
            self.direction_check_combos[key] = combo

        stop_specs = [
            ("M1 停止", "M1S"),
            ("M2 停止", "M2S"),
            ("全部停止", "MS"),
        ]
        for index, (title, command) in enumerate(stop_specs):
            button = QPushButton(title)
            button.clicked.connect(lambda _checked=False, cmd=command: self._send_manual_verification_command(cmd))
            layout.addWidget(button, 5, index)

        reset_button = QPushButton("清空判定")
        reset_button.clicked.connect(self._reset_direction_checks)
        layout.addWidget(reset_button, 6, 0)

        layout.addWidget(QLabel("最近手动命令"), 6, 1)
        layout.addWidget(self.last_manual_command_label, 6, 2)
        layout.addWidget(QLabel("方向确认结果"), 7, 0)
        layout.addWidget(self.manual_summary_label, 7, 1, 1, 2)

        return group

    def _send_manual_verification_command(self, command: str) -> None:
        if self._send_command(command):
            self.last_manual_command_label.setText(command)

    def _reset_direction_checks(self) -> None:
        for combo in self.direction_check_combos.values():
            combo.setCurrentText("未测")
        self.last_manual_command_label.setText("-")
        self._update_direction_summary()

    def _update_direction_summary(self) -> None:
        states = [combo.currentText() for combo in self.direction_check_combos.values()]
        correct = sum(1 for state in states if state == "正确")
        failed = sum(1 for state in states if state == "错误")
        untested = sum(1 for state in states if state == "未测")

        if failed > 0:
            summary = f"FAIL（正确 {correct}/4，错误 {failed}，未测 {untested}）"
        elif correct == len(states) and states:
            summary = "PASS（4/4 方向确认正确）"
        else:
            summary = f"进行中（正确 {correct}/4，未测 {untested}）"

        self.manual_summary_label.setText(summary)

    def _get_direction_check_snapshot(self) -> dict[str, str]:
        snapshot = {key: combo.currentText() for key, combo in self.direction_check_combos.items()}
        snapshot["manual_check_summary"] = self.manual_summary_label.text()
        snapshot["last_manual_command"] = self.last_manual_command_label.text()
        return snapshot

    def _build_overview_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        self.battery_plot = self._create_plot_widget("电池电压", "时间 (s)", "电压 (V)")
        self.imu_angle_plot = self._create_plot_widget("IMU 姿态角", "时间 (s)", "角度")
        self.motor_velocity_plot = self._create_plot_widget("电机速度", "时间 (s)", "速度")

        self.curves["battery_raw"] = self.battery_plot.plot(pen=pg.mkPen("#f39c12", width=2), name="Raw")
        self.curves["battery_filtered"] = self.battery_plot.plot(pen=pg.mkPen("#2ecc71", width=2), name="Filtered")
        self.curves["angle_x"] = self.imu_angle_plot.plot(pen=pg.mkPen("#e74c3c", width=2), name="Angle X")
        self.curves["angle_y"] = self.imu_angle_plot.plot(pen=pg.mkPen("#3498db", width=2), name="Angle Y")
        self.curves["angle_z"] = self.imu_angle_plot.plot(pen=pg.mkPen("#9b59b6", width=2), name="Angle Z")
        self.curves["m1_velocity"] = self.motor_velocity_plot.plot(pen=pg.mkPen("#16a085", width=2), name="M1 Vel")
        self.curves["m2_velocity"] = self.motor_velocity_plot.plot(pen=pg.mkPen("#c0392b", width=2), name="M2 Vel")
        self.curves["velocity_target"] = self.motor_velocity_plot.plot(pen=pg.mkPen("#7f8c8d", width=2, style=Qt.PenStyle.DashLine), name="Vel Target")
        self.curves["openloop_m1_cmd"] = self.motor_velocity_plot.plot(pen=pg.mkPen("#00d2d3", width=2), name="OpenLoop M1")
        self.curves["openloop_m2_cmd"] = self.motor_velocity_plot.plot(pen=pg.mkPen("#ff9f43", width=2), name="OpenLoop M2")

        layout.addWidget(self.battery_plot)
        layout.addWidget(self.imu_angle_plot)
        layout.addWidget(self.motor_velocity_plot)
        return page

    def _build_detail_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        self.imu_gyro_plot = self._create_plot_widget("IMU 陀螺仪", "时间 (s)", "Gyro")
        self.imu_acc_plot = self._create_plot_widget("IMU 加速度", "时间 (s)", "Acc")
        self.position_plot = self._create_plot_widget("电机位置闭环 / 角度", "时间 (s)", "角度")
        self.encoder_plot = self._create_plot_widget("编码器角度", "时间 (s)", "角度")

        self.curves["gyro_x"] = self.imu_gyro_plot.plot(pen=pg.mkPen("#ff6b6b", width=2), name="Gyro X")
        self.curves["gyro_y"] = self.imu_gyro_plot.plot(pen=pg.mkPen("#54a0ff", width=2), name="Gyro Y")
        self.curves["gyro_z"] = self.imu_gyro_plot.plot(pen=pg.mkPen("#1dd1a1", width=2), name="Gyro Z")
        self.curves["acc_x"] = self.imu_acc_plot.plot(pen=pg.mkPen("#e67e22", width=2), name="Acc X")
        self.curves["acc_y"] = self.imu_acc_plot.plot(pen=pg.mkPen("#2980b9", width=2), name="Acc Y")
        self.curves["acc_z"] = self.imu_acc_plot.plot(pen=pg.mkPen("#8e44ad", width=2), name="Acc Z")
        self.curves["m1_angle"] = self.position_plot.plot(pen=pg.mkPen("#27ae60", width=2), name="M1 Angle")
        self.curves["m2_angle"] = self.position_plot.plot(pen=pg.mkPen("#e74c3c", width=2), name="M2 Angle")
        self.curves["angle_target"] = self.position_plot.plot(pen=pg.mkPen("#34495e", width=2, style=Qt.PenStyle.DashLine), name="Angle Target")
        self.curves["encoder1_angle"] = self.encoder_plot.plot(pen=pg.mkPen("#f1c40f", width=2), name="Encoder1")
        self.curves["encoder2_angle"] = self.encoder_plot.plot(pen=pg.mkPen("#2d98da", width=2), name="Encoder2")

        layout.addWidget(self.imu_gyro_plot)
        layout.addWidget(self.imu_acc_plot)
        layout.addWidget(self.position_plot)
        layout.addWidget(self.encoder_plot)
        return page

    def _build_raw_log_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        self.raw_log.setReadOnly(True)
        self.raw_log.setMaximumBlockCount(2000)
        layout.addWidget(self.raw_log)
        return page

    def _create_plot_widget(self, title: str, bottom: str, left: str) -> Any:
        plot = pg.PlotWidget()
        plot.setBackground("#111827")
        plot.showGrid(x=True, y=True, alpha=0.25)
        plot.addLegend(offset=(10, 10))
        plot.setTitle(title)
        plot.setLabel("bottom", bottom)
        plot.setLabel("left", left)
        return plot

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentText()
        ports = [port.device for port in list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current and current in ports:
            self.port_combo.setCurrentText(current)

    def _toggle_connection(self) -> None:
        if self.worker and self.worker.isRunning():
            self._disconnect_serial()
            return

        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "未选择串口", "请先选择一个串口。")
            return

        self.worker = SerialWorker(port, self.baud_spin.value())
        self.worker.line_received.connect(self._handle_serial_line)
        self.worker.status_changed.connect(self._update_status)
        self.worker.error_occurred.connect(self._handle_serial_error)
        self.worker.finished.connect(self._on_worker_finished)
        self.worker.start()
        self.connect_button.setText("断开串口")

    def _disconnect_serial(self) -> None:
        if self.worker:
            self.worker.stop()
            self.worker.wait(1000)
        self.connect_button.setText("连接串口")

    @Slot()
    def _on_worker_finished(self) -> None:
        self.connect_button.setText("连接串口")

    @Slot(str)
    def _update_status(self, message: str) -> None:
        self.status_label.setText(message)
        self.statusBar().showMessage(message, 5000)

    @Slot(str)
    def _handle_serial_error(self, message: str) -> None:
        self.status_label.setText(message)
        self.statusBar().showMessage(message, 8000)
        QMessageBox.warning(self, "串口错误", message)
        self.connect_button.setText("连接串口")

    def _send_command(self, command: str) -> bool:
        if not self.worker or not self.worker.isRunning():
            QMessageBox.information(self, "尚未连接", "请先连接串口再发送命令。")
            return False
        self.worker.write_text(f"{command}\n")
        self.statusBar().showMessage(f"已发送命令: {command}", 2000)
        return True

    def _send_custom_command(self) -> None:
        text = self.send_input.text().strip()
        if not text:
            return
        self._send_command(text)
        self.send_input.clear()

    @Slot(str)
    def _handle_serial_line(self, line: str) -> None:
        self.latest_line_label.setText(line[:60] + ("..." if len(line) > 60 else ""))
        self.raw_log.appendPlainText(line)

        if line == "All tests stopped.":
            self.current_test = "0"
            self.current_test_label.setText("0")

        event = self.parser.parse(line)
        if not event:
            return

        if event.kind == "test_start":
            self.current_test = str(event.values["current_test"])
            self.current_test_label.setText(self.current_test)
        else:
            self._update_metrics(event)
            self._write_csv(event)

    def _update_metrics(self, event: TelemetryEvent) -> None:
        now = time.perf_counter() - self.start_time
        values = event.values

        if "foc_ready" in values:
            self.foc_label.setText("YES" if values["foc_ready"] else "NO")

        for key, value in values.items():
            if key in self.metric_labels:
                if isinstance(value, float):
                    self.metric_labels[key].setText(f"{value:.3f}")
                else:
                    self.metric_labels[key].setText(str(value))
            if isinstance(value, (int, float, bool)):
                self._append_series(key, now, float(value))

    def _append_series(self, name: str, x: float, y: float) -> None:
        if name not in self.series:
            self.series[name] = {
                "x": deque(maxlen=600),
                "y": deque(maxlen=600),
            }
        self.series[name]["x"].append(x)
        self.series[name]["y"].append(y)

    def _refresh_plots(self) -> None:
        for name, curve in self.curves.items():
            if name not in self.series:
                continue
            x = list(self.series[name]["x"])
            y = list(self.series[name]["y"])
            curve.setData(x, y)

    def _start_recording(self) -> None:
        logs_dir = Path(__file__).resolve().parent / "logs"
        logs_dir.mkdir(exist_ok=True)
        default_name = logs_dir / f"wheel_leg_test_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        file_name, _ = QFileDialog.getSaveFileName(
            self,
            "保存 CSV 日志",
            str(default_name),
            "CSV Files (*.csv)",
        )
        if not file_name:
            return

        self.csv_file = open(file_name, "w", newline="", encoding="utf-8-sig")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.CSV_FIELDS)
        self.csv_writer.writeheader()
        self.csv_file.flush()
        self.record_button.setEnabled(False)
        self.stop_record_button.setEnabled(True)
        self.statusBar().showMessage(f"开始录制: {file_name}", 5000)

    def _stop_recording(self) -> None:
        if self.csv_file:
            self.csv_file.close()
        self.csv_file = None
        self.csv_writer = None
        self.record_button.setEnabled(True)
        self.stop_record_button.setEnabled(False)
        self.statusBar().showMessage("CSV 录制已停止", 3000)

    def _write_csv(self, event: TelemetryEvent) -> None:
        if not self.csv_writer or not self.csv_file:
            return

        row = {field: "" for field in self.CSV_FIELDS}
        row["host_time"] = f"{time.time():.3f}"
        row["msg_type"] = event.kind
        row["current_test"] = self.current_test
        row["raw_line"] = event.raw_line
        row.update(self._get_direction_check_snapshot())
        for key, value in event.values.items():
            if key in row:
                row[key] = value
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def closeEvent(self, event: QCloseEvent) -> None:
        self._stop_recording()
        self._disconnect_serial()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
