# ESP32 Face Tracking Gimbal

ESP32-S3 face tracking gimbal with OV2640 video streaming, dual SG90 pan-tilt servos, and a PySide6 desktop face detector.

本项目使用 ESP32-S3-N16R8 控制 OV2640 摄像头采集图像并硬件压缩为 JPEG，经 Wi-Fi 以 UDP 分片上行到 Windows 上位机。上位机使用 MediaPipe BlazeFace 进行人脸检测，经 PD 算法解算出 Yaw/Pitch 目标角度，通过 TCP 下发给 ESP32，驱动双 SG90 舵机云台平滑跟随人脸。本地七向按键支持手动微调与行程校准，校准结果持久化到 LittleFS。

## Features

- OV2640 DVP 接口采集 VGA（640×480）JPEG 帧，硬件压缩，三缓冲。
- 视频走 UDP 单播分片（端口 8889），无反压，丢帧不卡顿，永远不拖慢摄像头采集。
- 控制走 TCP（端口 8888），ESP32 主动连接 PC，NoDelay + Keepalive 快速检测死连接。
- MediaPipe BlazeFace 短距人脸检测（TFLite float16，320 降采样），按置信度阈值过滤并选最大人脸。
- PD 角度解算：微分项抑制振荡，非线性增益放大让大偏移快速追上，死区与 EMA 输出平滑。
- ESP32 端 50Hz 独立舵机任务：自适应 EMA（大误差快冲、小误差微调）+ 单步限幅防猛转。
- TCP 卡顿积压时角度指令只应用最新一条，不产生恢复后的猛转。
- 七向按键本地手动微调、SET/MID 两角点校准，范围存 LittleFS `/calib.json` 重启生效。
- 有源蜂鸣器状态提示（连接成功、校准节点、断线告警）。
- PySide6 上位机实时画面、人脸标注、偏移量与角度显示，解算参数在线可调。

## Repository Layout

```text
.
├── ov2640_psram_test/              # ESP32 Arduino 固件
│   └── ov2640_psram_test.ino       # 采集/传输/舵机/按键/校准/蜂鸣器
├── upper_pc/                       # PySide6 上位机
│   ├── main.py                     # 入口
│   ├── main_window.py              # 主窗口：视频显示、控制面板、状态栏
│   ├── video_worker.py             # 工作线程：收流→检测→解算→下发
│   ├── angle_solver.py             # PD 角度解算器
│   ├── protocol.py                 # 通信协议定义
│   ├── config.py                   # 默认参数
│   ├── tcp_control_sender.py       # TCP 控制通道（服务器端）
│   ├── udp_video_receiver.py       # UDP 视频接收与分片重组
│   └── models/
│       └── blaze_face_short_range.tflite   # MediaPipe 人脸检测模型
└── 硬件/                            # 嘉立创 EDA（EasyEDA v6）设计文件
    ├── SCH_人脸跟踪云台_2026-10-23.json   # 原理图
    └── PCB_NEW_PCB_2026-10-23.json        # PCB
```

## Hardware

主要硬件：

- ESP32-S3-N16R8 主控（16MB Flash / 8MB OPI PSRAM / 2.4GHz Wi-Fi）
- OV2640 图像传感器，200 万像素，DVP 8-bit 并行接口，硬件 JPEG 压缩
- SG90 舵机 × 2（水平 Yaw / 垂直 Pitch 云台）
- 七向按键（五向导航键 + Set/Reset），低电平有效，内部上拉
- 有源蜂鸣器，高电平触发
- Windows 电脑，运行 PySide6 上位机，与 ESP32 同一局域网

## Wiring

以下接线来自 `ov2640_psram_test/ov2640_psram_test.ino` 中的实际引脚定义，与 `硬件/` 目录下原理图一致。

**OV2640（DVP 接口）**

| 信号  | ESP32 GPIO | 信号  | ESP32 GPIO |
| ----- | ---------- | ----- | ---------- |
| PWDN  | 39         | Y2    | 11         |
| RESET | 38         | Y3    | 9          |
| XCLK  | -1（内部） | Y4    | 8          |
| SIOD  | 4          | Y5    | 10         |
| SIOC  | 5          | Y6    | 12         |
| VSYNC | 6          | Y7    | 18         |
| HREF  | 7          | Y8    | 17         |
| PCLK  | 13         | Y9    | 16         |

闪光灯控制引脚为 GPIO40。

**舵机、按键、蜂鸣器**

| 模块     | 信号 / 按键 | ESP32 GPIO | 说明                                       |
| -------- | ----------- | ---------- | ------------------------------------------ |
| 水平舵机 | Signal      | 41         | 50Hz PWM，0.5–2.5ms 对应 0–180°，方向反转 |
| 垂直舵机 | Signal      | 42         | 50Hz PWM，行程中点 Pitch 45°               |
| 摇杆     | 上          | 1          | 输入，内部上拉，按下为低                   |
| 摇杆     | 下          | 2          | 同上                                       |
| 摇杆     | 左          | 19         | 同上                                       |
| 摇杆     | 右          | 20         | 同上                                       |
| 摇杆     | 中          | 21         | 校准模式下记录角点                         |
| 摇杆     | Set         | 15         | 进入 / 退出校准                            |
| 摇杆     | Reset       | 14         | 一键回中点                                 |
| 蜂鸣器   | I/O         | 47         | 有源，高电平触发                           |

Important hardware notes:

- 摇杆左右键占用 GPIO19/20，与 USB CDC 引脚冲突，Arduino IDE 必须将 USB CDC On Boot 设为 Disabled。
- SG90 建议独立 5V 供电并与 ESP32 共地，否则舵机动作可能导致复位或 Wi-Fi 掉线。
- OV2640 帧缓冲放在 PSRAM，须将 PSRAM 配置为 OPI PSRAM (8MB)。
- 分区方案须选择带 LittleFS 的项，否则校准 JSON 无法落盘。

## ESP32 Firmware

固件位于：

```text
ov2640_psram_test/ov2640_psram_test.ino
```

上传前需修改文件开头的连接配置：

```c
#define WIFI_SSID        "YOUR_WIFI_SSID"
#define WIFI_PASS        "YOUR_WIFI_PASSWORD"
#define PC_IP            "192.168.1.100"      // PC 局域网 IP，ipconfig 查看
#define TCP_PORT         8888                 // PC TCP 控制端口
#define UDP_VIDEO_PORT   8889                 // PC UDP 视频端口
#define STREAM_FPS       10                   // VGA 10fps
```

Arduino IDE 工具菜单关键设置：

| 选项              | 值                        | 原因                       |
| ----------------- | ------------------------- | -------------------------- |
| Board             | ESP32S3 Dev Module        | 目标板卡                   |
| Flash Size        | 16MB (128Mb)              | N16R8 规格                 |
| Partition Scheme  | 带 LittleFS 的方案        | 校准 JSON 需要文件系统     |
| PSRAM             | OPI PSRAM (8MB)           | 摄像头帧缓冲与发送缓冲     |
| CPU Freq          | 240MHz                    | Wi-Fi 正常工作             |
| USB CDC On Boot   | Disabled                  | 释放 GPIO19/20 给摇杆按键  |

依赖库：Arduino IDE 库管理器搜索并安装 `esp32-camera`（Espressif），LittleFS / WiFi / WiFiUdp / WiFiClient 为 arduino-esp32 自带。

主要默认参数（见 `.ino` 文件顶部宏定义）：

| 参数                | 默认值           | 说明                         |
| ------------------- | ---------------- | ---------------------------- |
| 帧尺寸              | VGA 640×480      | `CAM_FRAME_SIZE`             |
| JPEG 质量           | 12               | `CAM_JPEG_QUALITY`           |
| 帧缓冲数            | 3                | `CAM_FB_COUNT`               |
| 舵机更新频率        | 50Hz             | `SERVO_UPDATE_HZ`            |
| 舵机单步上限        | 8°               | `SERVO_MAX_STEP`             |
| Yaw 中点 / 行程     | 90° / 0–180°     | `YAW_CENTER` 等              |
| Pitch 中点 / 行程   | 45° / 0–90°      | `PITCH_CENTER` 等            |

校准流程：按 Set 进入校准（短鸣）→ 方向键移到最小角点（短按 1° 步进，长按连发）→ 按中间键记录 → 移到最大角点按中间键记录 → 长鸣完成并保存。期间按 Set 可放弃。非校准模式下方向键直接手动微调，Reset 一键回中点。

## Desktop Visualizer

上位机位于：

```text
upper_pc/
```

安装依赖：

```powershell
pip install PySide6 opencv-python mediapipe numpy
```

确保模型文件存在（缺失时可从 `config.py` 中的 `MODEL_URL` 下载）：

```text
upper_pc/models/blaze_face_short_range.tflite
```

运行：

```powershell
cd upper_pc
python main.py
```

启动后点击"启动"按钮，等待 ESP32 TCP 连接（状态栏显示"已连接"），随后出现视频画面并自动跟踪。右侧面板支持：自动下发开关、标注显示、检测置信度阈值；"设置"对话框调整 Kp/Kd、死区、平滑系数、行程范围；"舵机复位"与"获取舵机校准"一键操作。

上位机默认参数（见 `upper_pc/config.py`）：

| 参数                  | 默认值 | 说明                       |
| --------------------- | ------ | -------------------------- |
| TCP 控制端口          | 8888   | `TCP_PORT`                 |
| UDP 视频端口          | 8889   | `UDP_VIDEO_PORT`           |
| Kp Yaw / Pitch        | 3 / 2  | `KP_YAW` / `KP_PITCH`      |
| Kd Yaw / Pitch        | 0.6 / 0.4 | `KD_YAW` / `KD_PITCH`   |
| 归一化死区            | 0.05   | `DEADZONE`                 |
| 输出 EMA 平滑         | 0.45   | `OUTPUT_SMOOTHING`         |
| 单帧最大角度增量      | 12°    | `MAX_DELTA_PER_FRAME`      |
| 指令最小发送间隔      | 30ms   | `CMD_MIN_INTERVAL`         |

## Protocol

视频上行（ESP32 → PC，UDP，端口 8889），每包 ≤ 1400 字节：

```text
[frame_id uint16 小端][total uint8][idx uint8][JPEG 片段]
```

接收端按 `frame_id` 归集分片，收齐拼出完整 JPEG；同一帧超过 0.15s 未拼齐整帧丢弃，只保留最新帧。

控制通道（PC ↔ ESP32，TCP，端口 8888），ESP32 作客户端主动连接 PC，指令为以 `\n` 结尾的 ASCII 文本行：

| 指令     | 格式                              | 说明                                       |
| -------- | --------------------------------- | ------------------------------------------ |
| 设定角度 | `Y{yaw}P{pitch}\n`                | 例 `Y95.50P42.30\n`，受校准范围钳制        |
| 复位     | `R\n`                             | 舵机复位到行程中点                         |
| 查询校准 | `C\n`                             | ESP32 回送 `{"yaw_min":..,...}\n`          |
| 蜂鸣器   | `B0\n` / `B1\n` / `B2\n`          | 关 / 开 / 短鸣 120ms                       |

## Workflow

1. 按接线表连接 OV2640、双 SG90 舵机、七向按键和蜂鸣器，或直接使用 `硬件/` 目录下原理图与 PCB。
2. 修改 `ov2640_psram_test/ov2640_psram_test.ino` 中的 Wi-Fi 名称、密码和 PC 局域网 IP。
3. 使用 Arduino IDE 打开固件，按上表设置板卡选项，安装 `esp32-camera` 库，编译并烧录。
4. 在 PC 上安装依赖并运行 `upper_pc/main.py`。
5. 给 ESP32 上电，等待串口打印 Wi-Fi 连接成功与 TCP 连接成功，上位机状态栏显示"已连接"。
6. 点击上位机"启动"，观察视频画面与云台跟随效果。
7. 机械安装有偏差时按 Set 进入校准，记录最小/最大角点后自动保存。
8. 跟踪振荡时增大 Kd 或减小 Kp；响应迟缓时增大 Kp 或非线性增益系数。

## Troubleshooting

| 现象                       | 检查项                                                         |
| -------------------------- | -------------------------------------------------------------- |
| 串口一直显示 Wi-Fi 连接失败 | SSID/密码是否正确，路由器是否 2.4GHz                           |
| TCP 一直连不上             | PC IP 是否正确、防火墙是否放行 8888、上位机是否先启动          |
| 上位机无视频               | UDP 8889 是否被占用，ESP32 与 PC 是否同一网段，串口帧率统计    |
| 视频卡顿丢帧               | Wi-Fi 信号强度，`STREAM_FPS` 与 JPEG 质量，路由器 2.4G 拥挤    |
| 人脸检测不触发             | 光线是否充足，检测阈值是否过高，模型文件是否存在               |
| 舵机抖动或 ESP32 复位      | SG90 是否独立 5V 供电、是否与 ESP32 共地                       |
| 舵机顶机械限位             | 重新执行 Set/MID 校准，缩小行程范围                            |
| 检测到但不跟随             | 上位机"自动下发控制指令"是否勾选，TCP 是否已连接               |
| 左右方向键无效             | Arduino IDE USB CDC On Boot 是否已设为 Disabled                |
