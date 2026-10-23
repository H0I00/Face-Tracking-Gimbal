"""人脸偏移 -> 云台目标角度的 PD 控制器（自适应版 v3）。

约定：
    图像坐标系原点在左上角，x 向右增、y 向下增。
    dx = (face_cx - W/2)/(W/2)   ->  面部相对画面中心的归一化水平偏移
    dy = (face_cy - H/2)/(H/2)   ->  归一化垂直偏移

控制方向：
    dx > 0 (脸在右) -> yaw 增大（向右追）
    dy > 0 (脸在下) -> pitch 增大（向下追）
无面部时保持当前角度。

优化清单 v3：
  [1] PD 控制：微分项抑制振荡
  [2] 死区：微小偏移不更新
  [3] EMA 输出平滑
  [4] 最大增量限制
  [5] D 项钳位：d_dx/d_dy 限制在 d_clamp 内
  [6] 首帧跳过 D 项
  [7] 非线性增益放大：偏移超过阈值时 P 增益放大，大偏移→大角度修正→快速追上
"""

import config


class AngleSolver:
    def __init__(self,
                 kp_yaw=config.KP_YAW, kp_pitch=config.KP_PITCH,
                 kd_yaw=config.KD_YAW, kd_pitch=config.KD_PITCH,
                 deadzone=config.DEADZONE,
                 output_smoothing=config.OUTPUT_SMOOTHING,
                 max_delta=config.MAX_DELTA_PER_FRAME,
                 d_clamp=config.D_CLAMP,
                 gain_boost_threshold=config.GAIN_BOOST_THRESHOLD,
                 gain_boost_factor=config.GAIN_BOOST_FACTOR,
                 yaw_min=config.YAW_MIN, yaw_max=config.YAW_MAX,
                 pitch_min=config.PITCH_MIN, pitch_max=config.PITCH_MAX,
                 yaw_init=config.YAW_INIT, pitch_init=config.PITCH_INIT):
        self.kp_yaw = float(kp_yaw)
        self.kp_pitch = float(kp_pitch)
        self.kd_yaw = float(kd_yaw)
        self.kd_pitch = float(kd_pitch)
        self.deadzone = float(deadzone)
        self.output_smoothing = float(output_smoothing)
        self.max_delta = float(max_delta)
        self.d_clamp = float(d_clamp)
        self.gain_boost_threshold = float(gain_boost_threshold)
        self.gain_boost_factor = float(gain_boost_factor)
        self.yaw_min = float(yaw_min)
        self.yaw_max = float(yaw_max)
        self.pitch_min = float(pitch_min)
        self.pitch_max = float(pitch_max)
        self.yaw = float(yaw_init)
        self.pitch = float(pitch_init)
        self._smooth_yaw = float(yaw_init)
        self._smooth_pitch = float(pitch_init)
        self._prev_dx = 0.0
        self._prev_dy = 0.0
        # [修复] 标记是否处于无脸保持状态，用于恢复首帧跳过 D 项
        self._was_holding = False

    @staticmethod
    def _clamp(v, lo, hi):
        return lo if v < lo else hi if v > hi else v

    def _boost(self, offset):
        """非线性增益放大：偏移超过阈值时放大，大偏移→大角度修正。"""
        abs_off = abs(offset)
        if abs_off <= self.gain_boost_threshold:
            return offset
        sign = 1.0 if offset > 0 else -1.0
        excess = abs_off - self.gain_boost_threshold
        return sign * (self.gain_boost_threshold + excess * self.gain_boost_factor)

    def reset(self):
        self.yaw = (self.yaw_min + self.yaw_max) / 2
        self.pitch = (self.pitch_min + self.pitch_max) / 2
        self._smooth_yaw = self.yaw
        self._smooth_pitch = self.pitch
        self._prev_dx = 0.0
        self._prev_dy = 0.0
        self._was_holding = False

    def update(self, face_cx, face_cy, img_w, img_h):
        """输入面部中心像素坐标，更新并返回 (yaw, pitch, dx, dy)。"""
        if img_w <= 0 or img_h <= 0:
            return self._smooth_yaw, self._smooth_pitch, 0.0, 0.0

        dx = (face_cx - img_w / 2.0) / (img_w / 2.0)
        dy = (face_cy - img_h / 2.0) / (img_h / 2.0)

        # 死区
        if abs(dx) < self.deadzone and abs(dy) < self.deadzone:
            self._prev_dx = dx
            self._prev_dy = dy
            self._was_holding = False
            return self._smooth_yaw, self._smooth_pitch, dx, dy

        # [修复] 从无脸恢复的首帧：跳过 D 项，防止 d_dx = dx - 0 激增
        if self._was_holding:
            d_dx = 0.0
            d_dy = 0.0
            self._was_holding = False
        else:
            # [修复] D 项钳位：限制偏移变化率，防止通信卡顿后 D 项爆炸
            d_dx = self._clamp(dx - self._prev_dx, -self.d_clamp, self.d_clamp)
            d_dy = self._clamp(dy - self._prev_dy, -self.d_clamp, self.d_clamp)

        self._prev_dx = dx
        self._prev_dy = dy

        # 非线性增益放大：大偏移→更大的 P 项输出→快速追上
        dx_eff = self._boost(dx)
        dy_eff = self._boost(dy)

        # PD 控制
        delta_yaw = self.kp_yaw * dx_eff + self.kd_yaw * d_dx
        delta_pitch = self.kp_pitch * dy_eff + self.kd_pitch * d_dy

        # 最大增量限制
        delta_yaw = self._clamp(delta_yaw, -self.max_delta, self.max_delta)
        delta_pitch = self._clamp(delta_pitch, -self.max_delta, self.max_delta)

        self.yaw = self._clamp(self.yaw + delta_yaw,
                               self.yaw_min, self.yaw_max)
        self.pitch = self._clamp(self.pitch + delta_pitch,
                                 self.pitch_min, self.pitch_max)

        # EMA 输出平滑
        alpha = self.output_smoothing
        self._smooth_yaw += alpha * (self.yaw - self._smooth_yaw)
        self._smooth_pitch += alpha * (self.pitch - self._smooth_pitch)

        return self._smooth_yaw, self._smooth_pitch, dx, dy

    def hold(self):
        """无面部时调用，角度不变。平滑值继续收敛到当前角度。"""
        alpha = self.output_smoothing
        self._smooth_yaw += alpha * (self.yaw - self._smooth_yaw)
        self._smooth_pitch += alpha * (self.pitch - self._smooth_pitch)
        self._was_holding = True
        self._prev_dx = 0.0
        self._prev_dy = 0.0
        return self._smooth_yaw, self._smooth_pitch
