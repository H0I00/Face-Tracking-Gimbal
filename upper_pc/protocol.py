"""上位机与 ESP32 之间的通信协议定义（UDP 视频 + TCP 控制版）。

==== 拓扑 ====
  视频上行：ESP32 -> PC，UDP 单播，端口 VIDEO_DEFAULT_PORT(8889)
  控制双向：ESP32 <-> PC，TCP，端口 TCP_PORT(8888)
  ESP32 作 TCP 客户端主动连接 PC，同时用 UDP 向 PC 发送视频分片。

==== 上行视频帧（ESP32 -> PC，UDP 分片）====
  每个 UDP 包 ≤ UDP_MAX_PAYLOAD 字节，包头 HEADER_SIZE 字节：
    [0-1] frame_id   uint16 小端，帧序号（循环 0..65535）
    [2]   total      uint8，本帧总分片数
    [3]   idx        uint8，当前分片序号（0-based）
  后接 JPEG 数据片段。
  接收方按 frame_id 归集分片，收齐则拼成完整 JPEG；
  超时或不齐则丢弃整帧（丢帧不卡顿）。

==== 下行控制指令（PC -> ESP32，TCP 流）====
  文本行，以 '\\n' 结尾，ASCII：
    "Y{yaw}P{pitch}\\n"  例: "Y123.45P45.67\\n"   设定绝对角度
    "R\\n"               舵机复位到行程中点
    "C\\n"               查询校准范围，ESP32 回送一行 JSON：
                         {"yaw_min":..,"yaw_max":..,"pitch_min":..,"pitch_max":..}\\n
    "B0\\n"              蜂鸣器关
    "B1\\n"              蜂鸣器开（持续）
    "B2\\n"              蜂鸣器短鸣 120ms
"""

import struct

# ---------- 网络 ----------
TCP_PORT = 8888                # TCP 控制连接端口
VIDEO_DEFAULT_PORT = 8889      # UDP 视频接收端口

# ---------- UDP 视频分片 ----------
UDP_MAX_PAYLOAD = 1400         # 单包最大载荷（避免 IP 分片）
HEADER_FMT = "<HBB"            # frame_id(u16) + total(u8) + idx(u8)，小端
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 4
MAX_FRAME_SIZE = 200_000       # 兜底：单帧 JPEG 超此值视为异常丢弃

# ---------- 控制指令（\n 分隔文本行）-------------
LINE_ENDING = b"\n"


def pack_chunk(frame_id: int, idx: int, total: int, payload: bytes) -> bytes:
    """打包一个 UDP 视频分片包。"""
    return struct.pack(HEADER_FMT, frame_id & 0xFFFF, total, idx) + payload


def unpack_header(data: bytes):
    """解析 UDP 视频包头部，返回 (raw, frame_id, idx, total) 或 None。"""
    if len(data) < HEADER_SIZE:
        return None
    frame_id, total, idx = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
    # struct unpack 顺序是 (frame_id, total, idx)，交换 idx/total 以匹配调用方
    return data, frame_id, idx, total


def encode_control(yaw: float, pitch: float) -> bytes:
    """编码下行角度指令（含\\n）。"""
    return f"Y{yaw:.2f}P{pitch:.2f}\n".encode("ascii")


RESET_CMD = "R"
CALIB_QUERY_CMD = "C"
BUZZER_OFF_CMD = "B0"
BUZZER_ON_CMD = "B1"
BUZZER_BEEP_CMD = "B2"


def encode_reset() -> bytes:
    return b"R\n"


def encode_calib_query() -> bytes:
    return b"C\n"


def encode_buzzer(cmd: str = BUZZER_BEEP_CMD) -> bytes:
    return (cmd + "\n").encode("ascii")


def parse_calib_json(data: bytes):
    """解析 ESP32 回送的校准 JSON 行，返回 dict 或 None。"""
    import json
    try:
        obj = json.loads(data.decode("ascii").strip())
    except (ValueError, UnicodeDecodeError):
        return None
    keys = ("yaw_min", "yaw_max", "pitch_min", "pitch_max")
    if not all(k in obj for k in keys):
        return None
    try:
        return {k: float(obj[k]) for k in keys}
    except (TypeError, ValueError):
        return None


def decode_control(text: str):
    """解析下行角度指令文本，返回 (yaw, pitch) 或 None。"""
    text = text.strip()
    try:
        y = text.index("Y")
        p = text.index("P")
        if p <= y:
            return None
        yaw = float(text[y + 1:p])
        pitch = float(text[p + 1:])
        return yaw, pitch
    except (ValueError, IndexError):
        return None
