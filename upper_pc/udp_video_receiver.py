"""UDP 上行视频流接收与 JPEG 分片重组。

[优化] 后台线程持续收包：人脸检测期间不丢帧。
用法：
    rx = UdpVideoReceiver(port=8889)
    rx.start()
    jpeg = rx.get_latest()   # 非阻塞取最新完整帧，无则 None
    rx.close()
"""

import socket
import threading
import time

import protocol


class UdpVideoReceiver:
    RECV_BUF = 2048                 # 单包缓冲（4 头 + 1400 载荷 + 余量）
    FRAME_TIMEOUT = 0.15            # 同一帧超 0.15s 未拼齐则丢弃（降低陈旧帧延迟）

    def __init__(self, port=protocol.VIDEO_DEFAULT_PORT):
        self.port = int(port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 加大内核接收缓冲，减少高帧率下分片丢包
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 512 * 1024)
        self._sock.bind(("0.0.0.0", self.port))
        self._sock.settimeout(0.0)  # 完全非阻塞

        # 重组状态（仅后台线程访问，无需加锁）
        self._cur_id = -1
        self._total = 0
        self._frags = {}
        self._frame_start = 0.0
        self._peer_addr = None      # 最近一个包的来源 (ip, port)

        # [优化] 最新完整帧：后台线程写入，主线程读取
        # 用 threading.Lock 保护，避免主线程读到半写的 bytes 引用
        self._lock = threading.Lock()
        self._latest_jpeg = None

        self._stop_event = threading.Event()
        self._thread = None

    @property
    def peer_addr(self):
        """返回最近收到包的来源地址 (ip, port)，无包则为 None。"""
        return self._peer_addr

    def start(self):
        """启动后台接收线程。"""
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def get_latest(self):
        """非阻塞返回最新完整 JPEG（bytes）或 None，取走后清空。"""
        with self._lock:
            jpeg = self._latest_jpeg
            self._latest_jpeg = None
            return jpeg

    def poll(self):
        """兼容旧接口：等价于 get_latest()。"""
        return self.get_latest()

    def _rx_loop(self):
        """后台线程：持续排空 socket，拼帧并更新 _latest_jpeg。"""
        while not self._stop_event.is_set():
            got_any = False
            while True:
                try:
                    data, addr = self._sock.recvfrom(self.RECV_BUF)
                except (BlockingIOError, OSError):
                    break
                got_any = True
                self._peer_addr = addr
                self._handle(data)
            # 清理超时未拼齐的帧
            if self._cur_id != -1 and (time.monotonic() - self._frame_start) > self.FRAME_TIMEOUT:
                self._reset_buffer()
            # 无包时短暂休眠，避免空转消耗 CPU
            if not got_any:
                time.sleep(0.001)

    def _handle(self, data):
        """处理一个包，拼齐后更新 _latest_jpeg。"""
        hdr = protocol.unpack_header(data)
        if hdr is None:
            return
        _, fid, idx, total = hdr
        payload = data[protocol.HEADER_SIZE:]
        if total == 0:
            return

        # 新帧到达：丢弃旧缓冲
        if fid != self._cur_id:
            self._cur_id = fid
            self._total = total
            self._frags = {}
            self._frame_start = time.monotonic()

        # 同帧内乱序/重复都按字典覆盖
        self._frags[idx] = payload
        if len(self._frags) >= self._total:
            jpeg = b"".join(self._frags[i] for i in range(self._total))
            with self._lock:
                self._latest_jpeg = jpeg
            self._reset_buffer()

    def _reset_buffer(self):
        self._cur_id = -1
        self._total = 0
        self._frags = {}
        self._frame_start = 0.0

    def close(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        try:
            self._sock.close()
        except OSError:
            pass
