"""PySide6 主窗口：视频显示 + 控制面板 + 状态栏。

拓扑：PC 作 TCP 服务器监听 8888；ESP32 作 TCP 客户端主动连接。
PC 的 IP 在 ESP32 固件中配置，双方无需在上位机填写 IP。
"""

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QPixmap, QFont
from PySide6.QtWidgets import (
    QCheckBox, QDialog, QDoubleSpinBox, QFormLayout,
    QHBoxLayout, QLabel, QMainWindow, QPushButton,
    QSplitter, QVBoxLayout, QWidget, QSizePolicy
)

import config
from video_worker import VideoWorker


class VideoLabel(QLabel):
    """按比例缩放绘制视频帧的黑底标签。"""

    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet("background-color:#101010;")
        self._pm = None
        self.setText("等待启动 …")

    def set_frame(self, qimg):
        if qimg is None:
            return
        self._pm = QPixmap.fromImage(qimg)
        self.update()

    def paintEvent(self, _):
        super().paintEvent(_)
        if self._pm is None:
            return
        from PySide6.QtGui import QPainter
        p = QPainter(self)
        scaled = self._pm.scaled(self.size(), Qt.KeepAspectRatio,
                                 Qt.SmoothTransformation)
        x = (self.width() - scaled.width()) // 2
        y = (self.height() - scaled.height()) // 2
        p.drawPixmap(x, y, scaled)
        p.end()


class SettingsDialog(QDialog):
    """解算参数设置对话框：Kp、Yaw/Pitch 行程范围。"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("解算参数设置")
        self.setMinimumWidth(320)

        form = QFormLayout(self)
        form.setLabelAlignment(Qt.AlignRight)

        self.sp_kpyaw = _dspn(0.0, 20.0, 0.5, config.KP_YAW)
        self.sp_kppitch = _dspn(0.0, 20.0, 0.5, config.KP_PITCH)
        form.addRow("Kp Yaw", self.sp_kpyaw)
        form.addRow("Kp Pitch", self.sp_kppitch)

        # [优化] 新增 PD 控制参数
        self.sp_kdyaw = _dspn(0.0, 10.0, 0.1, config.KD_YAW)
        self.sp_kdpitch = _dspn(0.0, 10.0, 0.1, config.KD_PITCH)
        form.addRow("Kd Yaw (微分)", self.sp_kdyaw)
        form.addRow("Kd Pitch (微分)", self.sp_kdpitch)

        self.sp_deadzone = _dspn(0.0, 0.5, 0.01, config.DEADZONE)
        self.sp_smoothing = _dspn(0.01, 1.0, 0.05, config.OUTPUT_SMOOTHING)
        self.sp_maxdelta = _dspn(0.5, 30.0, 0.5, config.MAX_DELTA_PER_FRAME)
        form.addRow("死区阈值", self.sp_deadzone)
        form.addRow("输出平滑", self.sp_smoothing)
        form.addRow("最大增量(°/帧)", self.sp_maxdelta)

        self.sp_ymin = _dspn(0.0, 720.0, 5.0, config.YAW_MIN)
        self.sp_ymax = _dspn(0.0, 720.0, 5.0, config.YAW_MAX)
        self.sp_pmin = _dspn(0.0, 180.0, 1.0, config.PITCH_MIN)
        self.sp_pmax = _dspn(0.0, 180.0, 1.0, config.PITCH_MAX)
        form.addRow("Yaw 最小", self.sp_ymin)
        form.addRow("Yaw 最大", self.sp_ymax)
        form.addRow("Pitch 最小", self.sp_pmin)
        form.addRow("Pitch 最大", self.sp_pmax)

        btns = QHBoxLayout()
        self.btn_ok = QPushButton("确定")
        self.btn_cancel = QPushButton("取消")
        btns.addStretch()
        btns.addWidget(self.btn_ok)
        btns.addWidget(self.btn_cancel)
        form.addRow(btns)

        self.btn_ok.clicked.connect(self.accept)
        self.btn_cancel.clicked.connect(self.reject)

    def get_params(self):
        return dict(
            kp_yaw=self.sp_kpyaw.value(),
            kp_pitch=self.sp_kppitch.value(),
            kd_yaw=self.sp_kdyaw.value(),
            kd_pitch=self.sp_kdpitch.value(),
            deadzone=self.sp_deadzone.value(),
            output_smoothing=self.sp_smoothing.value(),
            max_delta=self.sp_maxdelta.value(),
            yaw_min=self.sp_ymin.value(),
            yaw_max=self.sp_ymax.value(),
            pitch_min=self.sp_pmin.value(),
            pitch_max=self.sp_pmax.value(),
        )

    def set_params(self, p):
        if "kp_yaw" in p:
            self.sp_kpyaw.setValue(p["kp_yaw"])
        if "kp_pitch" in p:
            self.sp_kppitch.setValue(p["kp_pitch"])
        if "kd_yaw" in p:
            self.sp_kdyaw.setValue(p["kd_yaw"])
        if "kd_pitch" in p:
            self.sp_kdpitch.setValue(p["kd_pitch"])
        if "deadzone" in p:
            self.sp_deadzone.setValue(p["deadzone"])
        if "output_smoothing" in p:
            self.sp_smoothing.setValue(p["output_smoothing"])
        if "max_delta" in p:
            self.sp_maxdelta.setValue(p["max_delta"])
        if "yaw_min" in p:
            self.sp_ymin.setValue(p["yaw_min"])
        if "yaw_max" in p:
            self.sp_ymax.setValue(p["yaw_max"])
        if "pitch_min" in p:
            self.sp_pmin.setValue(p["pitch_min"])
        if "pitch_max" in p:
            self.sp_pmax.setValue(p["pitch_max"])


def _dspn(lo, hi, step, val):
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setSingleStep(step)
    s.setDecimals(2)
    s.setValue(val)
    return s


class MainWindow(QMainWindow):
    params_changed = Signal(object)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("人脸跟踪云台 - 上位机")
        self.resize(1080, 640)

        self.worker = None
        self.thread = None
        self._params = dict(
            kp_yaw=config.KP_YAW, kp_pitch=config.KP_PITCH,
            kd_yaw=config.KD_YAW, kd_pitch=config.KD_PITCH,
            deadzone=config.DEADZONE,
            output_smoothing=config.OUTPUT_SMOOTHING,
            max_delta=config.MAX_DELTA_PER_FRAME,
            yaw_min=config.YAW_MIN, yaw_max=config.YAW_MAX,
            pitch_min=config.PITCH_MIN, pitch_max=config.PITCH_MAX,
        )

        self._build_ui()
        self._connect_signals()

    # ---------- UI ----------
    def _build_ui(self):
        central = QWidget()
        root = QHBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 6)

        splitter = QSplitter(Qt.Horizontal)
        self.video_label = VideoLabel()
        splitter.addWidget(self.video_label)
        splitter.addWidget(self._build_panel())
        splitter.setStretchFactor(0, 8)
        splitter.setStretchFactor(1, 3)
        root.addWidget(splitter)
        self.setCentralWidget(central)

        self.status = self.statusBar()
        self.lbl_fps = QLabel("FPS --")
        self.lbl_face = QLabel("人脸: --")
        self.lbl_angle = QLabel("Yaw -- / Pitch --")
        self.lbl_offset = QLabel("dx -- / dy --")
        self.lbl_conn = QLabel("ESP32: 未连接")
        for w in (self.lbl_fps, self.lbl_face, self.lbl_angle,
                  self.lbl_offset, self.lbl_conn):
            w.setFont(QFont("Consolas", 9))
            self.status.addWidget(w)

    def _build_panel(self):
        box = QWidget()
        form = QFormLayout(box)
        form.setLabelAlignment(Qt.AlignRight)

        # 控制
        self.chk_send = QCheckBox("自动下发控制指令")
        self.chk_send.setChecked(config.SEND_CONTROL_DEFAULT)
        form.addRow("", self.chk_send)

        # 显示标注开关
        self.chk_annotate = QCheckBox("显示人脸标注")
        self.chk_annotate.setChecked(True)
        form.addRow("", self.chk_annotate)

        # 人脸检测阈值
        self.sp_threshold = QDoubleSpinBox()
        self.sp_threshold.setRange(0.0, 1.0)
        self.sp_threshold.setSingleStep(0.05)
        self.sp_threshold.setDecimals(2)
        self.sp_threshold.setValue(config.MIN_DETECTION_CONFIDENCE)
        form.addRow("检测阈值", self.sp_threshold)

        # 启动 / 停止
        row1 = QHBoxLayout()
        self.btn_start = QPushButton("启动")
        self.btn_stop = QPushButton("停止")
        self.btn_stop.setEnabled(False)
        row1.addWidget(self.btn_start)
        row1.addWidget(self.btn_stop)
        fw1 = QWidget()
        fw1.setLayout(row1)
        form.addRow(fw1)

        # 舵机复位 / 设置 / 获取校准
        row2 = QHBoxLayout()
        self.btn_reset = QPushButton("舵机复位")
        self.btn_settings = QPushButton("设置")
        self.btn_calib = QPushButton("获取舵机校准")
        row2.addWidget(self.btn_reset)
        row2.addWidget(self.btn_settings)
        row2.addWidget(self.btn_calib)
        fw2 = QWidget()
        fw2.setLayout(row2)
        form.addRow(fw2)

        return box

    # ---------- 信号 ----------
    def _connect_signals(self):
        self.btn_start.clicked.connect(self.start)
        self.btn_stop.clicked.connect(self.stop)
        self.btn_reset.clicked.connect(self.on_reset_clicked)
        self.btn_settings.clicked.connect(self.on_settings_clicked)
        self.btn_calib.clicked.connect(self.on_fetch_calib_clicked)

    def _collect_params(self):
        return dict(self._params)

    def _emit_params(self):
        p = self._collect_params()
        self.params_changed.emit(p)
        if self.worker is not None:
            self.worker.apply_params(p)

    def on_settings_clicked(self):
        dlg = SettingsDialog(self)
        dlg.set_params(self._params)
        if dlg.exec() == QDialog.Accepted:
            self._params = dlg.get_params()
            self._emit_params()

    # ---------- 启停 ----------
    def start(self):
        if self.worker is not None:
            return
        worker = VideoWorker()
        worker.set_send_control(self.chk_send.isChecked())
        worker.set_show_annotation(self.chk_annotate.isChecked())
        worker.set_detect_threshold(self.sp_threshold.value())

        th = QThread()
        worker.moveToThread(th)
        th.started.connect(worker.run)
        worker.frame_ready.connect(self.on_frame)
        worker.client_connected.connect(self.on_client_connected)
        worker.error.connect(self.on_error)
        worker.stopped.connect(self.on_stopped)
        self.params_changed.connect(worker.apply_params)
        self.chk_send.toggled.connect(
            lambda on: self.worker.set_send_control(on) if self.worker else None)
        self.chk_annotate.toggled.connect(
            lambda on: self.worker.set_show_annotation(on) if self.worker else None)
        self.sp_threshold.valueChanged.connect(
            lambda v: self.worker.set_detect_threshold(v) if self.worker else None)

        worker.apply_params(self._collect_params())

        self.worker = worker
        self.thread = th
        th.start()

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.video_label.setText("启动中 … 等待 ESP32 连接")

    def stop(self):
        if self.worker is not None:
            self.worker.stop()

    def on_client_connected(self, connected: bool):
        if connected:
            self.lbl_conn.setText("ESP32: 已连接")
            self.status.showMessage("ESP32 已连接", 3000)
        else:
            self.lbl_conn.setText("ESP32: 未连接")
            self.status.showMessage("ESP32 已断开", 3000)

    def on_reset_clicked(self):
        if self.worker is not None:
            self.worker.reset_angles()
        try:
            tx = self._get_direct_tx()
            if tx.send_reset():
                self.status.showMessage("已下发舵机复位指令", 3000)
            else:
                self.status.showMessage("复位指令发送失败（ESP32未连接）", 5000)
        except Exception as e:
            self.status.showMessage("复位失败: " + str(e), 5000)

    def _get_direct_tx(self):
        if self.worker is None or self.worker._tx is None or not self.worker._tx.has_client:
            raise RuntimeError("ESP32 尚未连接，请先启动并等待连接")
        return self.worker._tx

    def on_fetch_calib_clicked(self):
        try:
            tx = self._get_direct_tx()
            calib = tx.query_calibration(timeout=0.6)
        except Exception as e:
            self.status.showMessage("获取校准失败: " + str(e), 5000)
            return
        if calib is None:
            self.status.showMessage("未收到校准响应", 5000)
            return
        self._params.update(
            yaw_min=calib["yaw_min"], yaw_max=calib["yaw_max"],
            pitch_min=calib["pitch_min"], pitch_max=calib["pitch_max"],
        )
        self._emit_params()
        # 保存校准到 PC 端 JSON 文件
        import json
        from pathlib import Path
        calib_path = Path(__file__).parent / "calib.json"
        try:
            with open(calib_path, "w", encoding="utf-8") as f:
                json.dump(calib, f, indent=2, ensure_ascii=False)
            self.status.showMessage(
                f"已同步并保存校准: Yaw[{calib['yaw_min']:.1f},{calib['yaw_max']:.1f}] "
                f"Pitch[{calib['pitch_min']:.1f},{calib['pitch_max']:.1f}] -> {calib_path.name}", 5000)
        except OSError as e:
            self.status.showMessage(
                f"校准已同步(但保存文件失败: {e})", 5000)

    def on_frame(self, data):
        if data.get("qimage") is not None:
            self.video_label.set_frame(data["qimage"])
        face = data["face"]
        self.lbl_fps.setText(f"FPS {data['fps']:.1f}")
        self.lbl_face.setText(
            f"人脸: {'是' if face else '否'}" +
            (f" ({data['score']:.2f})" if face else ""))
        self.lbl_angle.setText(f"Yaw {data['yaw']:.1f} / Pitch {data['pitch']:.1f}")
        self.lbl_offset.setText(f"dx {data['dx']:+.2f} / dy {data['dy']:+.2f}")

    def on_error(self, msg):
        self.status.showMessage("错误: " + msg, 8000)

    def on_stopped(self):
        if self.thread is not None:
            self.thread.quit()
            self.thread.wait()
            self.thread = None
        self.worker = None
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.video_label.setText("已停止")
        self.lbl_conn.setText("ESP32: 未连接")

    def closeEvent(self, ev):
        self.stop()
        if self.thread is not None:
            self.thread.quit()
            self.thread.wait(3000)
        ev.accept()
