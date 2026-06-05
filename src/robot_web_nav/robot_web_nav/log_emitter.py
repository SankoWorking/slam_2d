import os
import time
import threading
from collections import deque


class LogEmitter:
    """Wraps a ROS2 logger and maintains a ring buffer for WebSocket streaming."""

    BUFFER_SIZE = 200

    def __init__(self, node):
        self._node = node
        self._buffer = deque(maxlen=self.BUFFER_SIZE)
        self._lock = threading.Lock()
        self._subscribers = []
        self._console_info = os.environ.get('WEB_NAV_CONSOLE_INFO', '').lower() in (
            '1',
            'true',
            'yes',
            'on',
        )

    def _append(self, level, module, message):
        entry = {
            'timestamp': time.time(),
            'level': level,
            'module': module,
            'message': message,
        }
        with self._lock:
            self._buffer.append(entry)
        for cb in list(self._subscribers):
            try:
                cb(entry)
            except Exception:
                pass

    def info(self, module, message):
        if self._console_info:
            self._node.get_logger().info(f'[{module}] {message}')
        self._append('info', module, message)

    def warn(self, module, message):
        self._node.get_logger().warn(f'[{module}] {message}')
        self._append('warn', module, message)

    def error(self, module, message):
        self._node.get_logger().error(f'[{module}] {message}')
        self._append('error', module, message)

    def get_recent(self, n=50):
        with self._lock:
            return list(self._buffer)[-n:]

    def subscribe(self, cb):
        self._subscribers.append(cb)

    def unsubscribe(self, cb):
        try:
            self._subscribers.remove(cb)
        except ValueError:
            pass
