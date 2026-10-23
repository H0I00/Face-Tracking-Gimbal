"""视频处理线程：UDP 取流 -> 人脸检测 -> 角度解算 -> TCP 下发指令 -> 发信号刷新界面。

ESP32 通过 UDP 发送视频分片，通过 TCP 连接接收控制指令。
"""

import time

import cv2
import mediapipe as mp
import numpy as np
from PySide6.QtCore import QObject, QThread, Signal, Slot
from PySide6.QtGui import QImage

import config
import protocol
from angle_solver import AngleSolver
from tcp_control_sender import TcpControlSender
from udp_video_receiver import UdpVideoReceiver


def _load_model_buffer() -> bytes:
    """读模型为 bytes，规避 mediapipe C 层对 CJK 路径 fopen 失败的问题。"""
    path = config.MODEL_PATH
    try:
        with open(path, "rb") as f:
            return f.read()
    except FileNotFoundError:
        raise RuntimeError(
            f"未找到人脸检测模型: {path}\n请从 {config.MODEL_URL} 下载放到该路径。")


class VideoWorker(QObject):
    frame_ready = Signal(object)
    client_connected = Signal(bool)     # ESP32 TCP 连接状态变化
    error = Signal(str)
    stopped = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.solver = AngleSolver()
        self._send_control = config.SEND_CONTROL_DEFAULT
        self._show_annotation = True
        self._detect_threshold = config.MIN_DETECTION_CONFIDENCE

        self._running = False
        self._rx = None       # UdpVideoReceiver
        self._tx = None       # TcpControlSender
        self._detector = None
        self._last_ts = 0
        self._fps_t0 = 0.0
        self._fps_n = 0
        self._fps = 0.0

        self._blank_t0 = 0.0
        self._was_connected = False
        # [修复] 角度指令限速：避免TCP卡顿期间积压大量旧指令
        self._last_sent_yaw = None
        self._last_sent_pitch = None
        self._last_send_t = 0.0

    @Slot(bool)
    def set_send_control(self, on: bool):
        self._send_control = bool(on)

    @Slot(bool)
    def set_show_annotation(self, on: bool):
        self._show_annotation = bool(on)

    @Slot(float)
    def set_detect_threshold(self, val: float):
        self._detect_threshold = float(val)

    @Slot(object)
    def apply_params(self, params: dict):
        self.update_solver(**params)

    def update_solver(self, kp_yaw=None, kp_pitch=None,
                      kd_yaw=None, kd_pitch=None,
                      deadzone=None, output_smoothing=None, max_delta=None,
                      d_clamp=None, gain_boost_threshold=None,
                      gain_boost_factor=None,
                      yaw_min=None, yaw_max=None, pitch_min=None, pitch_max=None):
        if kp_yaw is not None:
            self.solver.kp_yaw = float(kp_yaw)
        if kp_pitch is not None:
            self.solver.kp_pitch = float(kp_pitch)
        if kd_yaw is not None:
            self.solver.kd_yaw = float(kd_yaw)
        if kd_pitch is not None:
            self.solver.kd_pitch = float(kd_pitch)
        if deadzone is not None:
            self.solver.deadzone = float(deadzone)
        if output_smoothing is not None:
            self.solver.output_smoothing = float(output_smoothing)
        if max_delta is not None:
            self.solver.max_delta = float(max_delta)
        if d_clamp is not None:
            self.solver.d_clamp = float(d_clamp)
        if gain_boost_threshold is not None:
            self.solver.gain_boost_threshold = float(gain_boost_threshold)
        if gain_boost_factor is not None:
            self.solver.gain_boost_factor = float(gain_boost_factor)
        if yaw_min is not None:
            self.solver.yaw_min = float(yaw_min)
        if yaw_max is not None:
            self.solver.yaw_max = float(yaw_max)
        if pitch_min is not None:
            self.solver.pitch_min = float(pitch_min)
        if pitch_max is not None:
            self.solver.pitch_max = float(pitch_max)

    def reset_angles(self):
        self.solver.reset()

    @Slot()
    def stop(self):
        self._running = False

    # ---------- 线程主循环 ----------
    @Slot()
    def run(self):
        try:
            self._detector = self._make_detector()
        except Exception as e:
            self.error.emit(str(e))
            self.stopped.emit()
            return

        try:
            self._rx = UdpVideoReceiver(config.UDP_VIDEO_PORT)
        except OSError as e:
            self.error.emit(f"无法绑定 UDP 端口 {config.UDP_VIDEO_PORT}: {e}")
            self._cleanup()
            self.stopped.emit()
            return

        # [优化] 启动后台收包线程：人脸检测期间持续排空 socket，不丢帧
        self._rx.start()

        try:
            self._tx = TcpControlSender(config.TCP_PORT)
        except OSError as e:
            self.error.emit(f"无法绑定 TCP 端口 {config.TCP_PORT}: {e}")
            self._cleanup()
            self.stopped.emit()
            return

        self._running = True
        self._fps_t0 = time.monotonic()

        interval = 1.0 / config.TARGET_FPS
        while self._running:
            t0 = time.monotonic()

            jpeg = self._rx.get_latest()
            connected = self._tx.poll()

            # 连接状态变化通知主界面
            if connected != self._was_connected:
                self._was_connected = connected
                self.client_connected.emit(connected)

            if jpeg is None:
                QThread.msleep(5)
                self._emit_status_blank()
                continue

            frame_bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8),
                                     cv2.IMREAD_COLOR)
            if frame_bgr is None:
                continue

            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]

            # 人脸检测降采样
            detect_w, detect_h = w, h
            rgb_detect = rgb
            if config.DETECT_MAX_DIM and max(w, h) > config.DETECT_MAX_DIM:
                scale = config.DETECT_MAX_DIM / float(max(w, h))
                detect_w, detect_h = int(w * scale), int(h * scale)
                rgb_detect = cv2.resize(rgb, (detect_w, detect_h),
                                        interpolation=cv2.INTER_AREA)
            mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_detect)

            ts = self._next_ts()
            try:
                result = self._detector.detect_for_video(mp_img, ts)
            except Exception:
                result = None

            face_cx = face_cy = None
            score = 0.0
            if result is not None and result.detections:
                # 按用户设置的阈值过滤检测结果
                valid = [d for d in result.detections
                         if d.categories and (d.categories[0].score or 0.0) >= self._detect_threshold]
                if valid:
                    best = max(valid, key=self._box_area)
                    bb = best.bounding_box
                    sx = w / float(detect_w)
                    sy = h / float(detect_h)
                    face_cx = (bb.origin_x + bb.width / 2.0) * sx
                    face_cy = (bb.origin_y + bb.height / 2.0) * sy
                    if best.categories:
                        score = best.categories[0].score or 0.0

            if face_cx is not None:
                yaw, pitch, dx, dy = self.solver.update(face_cx, face_cy, w, h)
                if self._send_control:
                    self._maybe_send_angles(yaw, pitch)
            else:
                yaw, pitch = self.solver.hold()
                dx = dy = 0.0

            annotated = self._annotate(rgb, w, h, face_cx, face_cy, score, yaw, pitch)
            self._emit(annotated, w, h, face_cx is not None, score, yaw, pitch, dx, dy)

            dt = time.monotonic() - t0
            if dt < interval:
                QThread.msleep(int((interval - dt) * 1000))

        self._cleanup()
        self.stopped.emit()

    # ---------- 内部方法 ----------
    def _maybe_send_angles(self, yaw: float, pitch: float):
        """限速发送角度指令。
        只有在以下两个条件同时满足时才发送：
          1. 距上次发送超过 MIN_SEND_INTERVAL 秒
          2. 角度变化超过 MIN_SEND_DELTA 度
        这样可以避免 TCP 卡顿期间积压大量旧角度指令导致卡顿恢复后舵机猛转。
        """
        now = time.monotonic()
        if now - self._last_send_t < config.CMD_MIN_INTERVAL:
            return
        if (self._last_sent_yaw is not None and
                abs(yaw - self._last_sent_yaw) < config.CMD_MIN_DELTA and
                abs(pitch - self._last_sent_pitch) < config.CMD_MIN_DELTA):
            return
        if self._tx.send_angles(yaw, pitch):
            self._last_sent_yaw = yaw
            self._last_sent_pitch = pitch
            self._last_send_t = now

    @staticmethod
    def _box_area(det):
        bb = det.bounding_box
        return max(0, bb.width) * max(0, bb.height)

    def _make_detector(self):
        from mediapipe.tasks import python
        from mediapipe.tasks.python import vision
        buf = _load_model_buffer()
        opts = vision.FaceDetectorOptions(
            base_options=python.BaseOptions(model_asset_buffer=buf),
            running_mode=vision.RunningMode.VIDEO,
            min_detection_confidence=config.MIN_DETECTION_CONFIDENCE,
        )
        return vision.FaceDetector.create_from_options(opts)

    def _next_ts(self):
        now = int(time.monotonic_ns() // 1_000_000)
        if now <= self._last_ts:
            now = self._last_ts + 1
        self._last_ts = now
        return now

    def _annotate(self, rgb, w, h, fcx, fcy, score, yaw, pitch):
        out = rgb
        if not self._show_annotation:
            return out
        cx, cy = w / 2.0, h / 2.0
        cv2.drawMarker(out, (int(cx), int(cy)), (60, 120, 255),
                       markerType=cv2.MARKER_CROSS, markerSize=24, thickness=1)
        if fcx is not None:
            cv2.rectangle(out,
                          (int(fcx) - 4, int(fcy) - 4),
                          (int(fcx) + 4, int(fcy) + 4),
                          (0, 255, 0), -1)
            cv2.line(out, (int(cx), int(cy)), (int(fcx), int(fcy)),
                     (255, 200, 0), 1)
            cv2.putText(out, f"face {score:.2f}", (8, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        else:
            cv2.putText(out, "no face", (8, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        cv2.putText(out, f"Y {yaw:.1f}  P {pitch:.1f}", (8, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return out

    def _emit(self, rgb, w, h, face, score, yaw, pitch, dx, dy):
        self._fps_n += 1
        now = time.monotonic()
        if now - self._fps_t0 >= 0.5:
            self._fps = self._fps_n / (now - self._fps_t0)
            self._fps_n = 0
            self._fps_t0 = now
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        self.frame_ready.emit({
            "qimage": qimg, "w": w, "h": h,
            "fps": self._fps, "face": face, "score": score,
            "yaw": yaw, "pitch": pitch, "dx": dx, "dy": dy,
        })

    def _emit_status_blank(self):
        now = time.monotonic()
        if now - self._blank_t0 < 0.5:
            return
        self._blank_t0 = now
        self._fps = 0.0
        self.frame_ready.emit({
            "qimage": None, "w": 0, "h": 0, "fps": 0.0,
            "face": False, "score": 0.0,
            "yaw": self.solver.yaw, "pitch": self.solver.pitch,
            "dx": 0.0, "dy": 0.0,
        })

    def _cleanup(self):
        if self._rx is not None:
            self._rx.close()
            self._rx = None
        if self._tx is not None:
            self._tx.close()
            self._tx = None
        if self._detector is not None:
            try:
                self._detector.close()
            except Exception:
                pass
            self._detector = None
