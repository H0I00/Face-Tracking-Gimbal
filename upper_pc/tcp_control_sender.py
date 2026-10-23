"""TCP 下行控制指令发送：独立 TCP 服务器，与 UDP 视频分离。

ESP32 连接到此 TCP 端口仅收发控制指令。
所有指令以 '\\n' 结尾，与 ESP32 固件约定一致。
"""

import socket

import protocol


class TcpControlSender:
    def __init__(self, port=protocol.TCP_PORT):
        self.port = port
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("0.0.0.0", port))
        self._srv.listen(1)
        self._srv.setblocking(False)

        self._client = None
        self._client_addr = None

    @property
    def has_client(self):
        return self._client is not None

    @property
    def client_addr(self):
        return self._client_addr

    def _accept_if_pending(self):
        if self._client is not None:
            return
        try:
            self._client, self._client_addr = self._srv.accept()
            self._client.setblocking(False)
            # [优化] 禁用 Nagle 算法：控制指令是小包(~15字节)，
            # Nagle 会等待凑满 MSS 再发，造成 40ms+ 延迟
            try:
                self._client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except OSError:
                pass
        except (BlockingIOError, OSError):
            pass

    def _check_alive(self):
        if self._client is None:
            return
        try:
            data = self._client.recv(1, socket.MSG_PEEK)
            if not data:
                self._on_disconnect()
        except (BlockingIOError, OSError):
            pass

    def _on_disconnect(self):
        if self._client is not None:
            try:
                self._client.close()
            except OSError:
                pass
        self._client = None
        self._client_addr = None

    def poll(self):
        self._accept_if_pending()
        self._check_alive()
        return self._client is not None

    def _send(self, data: bytes):
        if self._client is None:
            return False
        try:
            self._client.sendall(data)
            return True
        except BlockingIOError:
            # [优化] 发送缓冲区暂时满：不断开连接，丢弃这条指令
            # 控制指令高频发送，丢一条不影响跟踪
            return False
        except OSError:
            self._on_disconnect()
            return False

    def send_angles(self, yaw, pitch):
        return self._send(protocol.encode_control(float(yaw), float(pitch)))

    def send_reset(self):
        return self._send(protocol.encode_reset())

    def send_buzzer(self, cmd: str = protocol.BUZZER_BEEP_CMD):
        return self._send(protocol.encode_buzzer(cmd))

    def query_calibration(self, timeout: float = 0.6):
        """发送 'C\\n' 查询校准范围，阻塞等待 ESP32 回送 JSON 行。

        返回 dict 或 None。临时将 socket 设为阻塞超时，
        调用后恢复非阻塞——仅供按钮点击使用，勿在视频循环中调用。
        """
        if self._client is None:
            return None
        buf = b""
        try:
            self._client.setblocking(True)
            self._client.settimeout(timeout)
            self._client.sendall(protocol.encode_calib_query())
            while b"\n" not in buf:
                chunk = self._client.recv(512)
                if not chunk:
                    return None
                buf += chunk
        except (socket.timeout, OSError):
            return None
        finally:
            try:
                self._client.setblocking(False)
            except OSError:
                pass
        return protocol.parse_calib_json(buf)

    def close(self):
        self._on_disconnect()
        try:
            self._srv.close()
        except OSError:
            pass
