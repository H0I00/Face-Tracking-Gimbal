"""上位机默认配置。GUI 里大部分参数可实时修改，这里仅给出初值。

优化说明：
  - 角度解算从纯 P 控制升级为 PD 控制（加微分项抑制振荡）
  - 新增死区、EMA 平滑、最大增量限制参数
"""

from pathlib import Path

# ---------- 网络 ----------
TCP_PORT = 8888                # TCP 控制连接端口
UDP_VIDEO_PORT = 8889          # UDP 视频接收端口

# ---------- 视觉 ----------
MODEL_PATH = str(Path(__file__).parent / "models" / "blaze_face_short_range.tflite")
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/face_detector/"
             "blaze_face_short_range/float16/latest/blaze_face_short_range.tflite")
MIN_DETECTION_CONFIDENCE = 0.5
DETECT_MAX_DIM = 320                   # 人脸检测降采样最大边长（0=不缩放）
TARGET_FPS = 30                        # 处理帧率上限

# ---------- 角度解算（PD 控制 + 非线性增益 + 平滑） ----------
KP_YAW = 3.0                          # 比例增益 Yaw
KP_PITCH = 2.0                        # 比例增益 Pitch
KD_YAW = 0.6                          # 微分增益 Yaw（抑制振荡）
KD_PITCH = 0.4                        # 微分增益 Pitch
DEADZONE = 0.05                       # 归一化死区阈值
OUTPUT_SMOOTHING = 0.45               # 输出 EMA：低=目标角渐变更缓，配合 ESP32 低 EMA 实现连续滑行
MAX_DELTA_PER_FRAME = 12.0            # 单帧最大角度增量(度)，放大以支持非线性增益的大角度修正
D_CLAMP = 0.3                         # D 项偏移变化率钳位（归一化），防止卡顿后 D 项爆炸
GAIN_BOOST_THRESHOLD = 0.2            # 归一化偏移超过此值时启动增益放大
GAIN_BOOST_FACTOR = 2.5               # 超过阈值部分的增益放大倍数：大偏移→大角度修正
YAW_MIN, YAW_MAX = 0.0, 180.0
PITCH_MIN, PITCH_MAX = 0.0, 90.0
YAW_INIT = (YAW_MIN + YAW_MAX) / 2
PITCH_INIT = (PITCH_MIN + PITCH_MAX) / 2

# ---------- 控制 ----------
SEND_CONTROL_DEFAULT = True
CMD_MIN_INTERVAL = 0.03   # 角度指令最小发送间隔(秒)，约33Hz
CMD_MIN_DELTA    = 0.15   # 角度变化小于此值(度)时不重复发送
