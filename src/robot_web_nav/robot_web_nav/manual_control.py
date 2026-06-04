import threading
import time

from rclpy.node import Node
from geometry_msgs.msg import Twist


class ManualControlManager:
    """Manages manual /cmd_vel publishing with single-controller claim.

    Only one WebSocket client (identified by client_id) can hold control at
    a time. The claimed velocity is republished at 10 Hz so the robot's
    safety timeout (typically 0.5s on the controller side) never fires.
    Control auto-releases if the claiming client stops sending set_velocity
    for more than 2 seconds, or if it disconnects.
    """

    PUBLISH_HZ = 10.0
    AUTO_RELEASE_S = 2.0

    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self._lock = threading.Lock()
        self._owner: str | None = None
        self._last_seen: float = 0.0
        self._linear_x = 0.0
        self._linear_y = 0.0
        self._angular_z = 0.0
        self._max_linear = 0.3
        self._max_angular = 1.0
        self._status_callback = None
        self._stop_sent = False
        self._timer = node.create_timer(1.0 / self.PUBLISH_HZ, self._tick)

    def set_status_callback(self, cb):
        self._status_callback = cb

    def set_limits(self, max_linear: float, max_angular: float):
        self._max_linear = max(0.05, max_linear)
        self._max_angular = max(0.05, max_angular)

    def claim(self, client_id: str) -> bool:
        with self._lock:
            if self._owner is not None and self._owner != client_id:
                return False
            self._owner = client_id
            self._last_seen = time.monotonic()
        self._emit_status()
        return True

    def release(self, client_id: str):
        with self._lock:
            if self._owner == client_id:
                self._owner = None
                self._linear_x = 0.0
                self._linear_y = 0.0
                self._angular_z = 0.0
                self._stop_sent = False
        self._emit_status()

    def force_release_for(self, client_id):
        """Release control regardless of who owns it (used on disconnect)."""
        with self._lock:
            if self._owner == client_id:
                self._owner = None
                self._linear_x = 0.0
                self._linear_y = 0.0
                self._angular_z = 0.0
                self._stop_sent = False
        self._emit_status()

    def set_velocity(self, client_id: str, linear_x: float, linear_y: float, angular_z: float) -> bool:
        """Returns True if accepted (caller owns control)."""
        with self._lock:
            if self._owner != client_id:
                return False
            self._linear_x = max(-self._max_linear, min(self._max_linear, linear_x))
            self._linear_y = max(-self._max_linear, min(self._max_linear, linear_y))
            self._angular_z = max(-self._max_angular, min(self._max_angular, angular_z))
            self._last_seen = time.monotonic()
            self._stop_sent = False
        return True

    def get_status(self) -> dict:
        with self._lock:
            return {
                'owner': self._owner,
                'linear_x': round(self._linear_x, 3),
                'linear_y': round(self._linear_y, 3),
                'angular_z': round(self._angular_z, 3),
                'max_linear': round(self._max_linear, 3),
                'max_angular': round(self._max_angular, 3),
            }

    def _tick(self):
        now = time.monotonic()
        with self._lock:
            # Auto-release if owner went silent
            if self._owner is not None and (now - self._last_seen) > self.AUTO_RELEASE_S:
                self._node.get_logger().info(
                    f'Manual control auto-released (owner {self._owner} silent)'
                )
                self._owner = None
                self._linear_x = 0.0
                self._linear_y = 0.0
                self._angular_z = 0.0
                self._stop_sent = False
                self._emit_status_locked()

            if (
                self._owner is None
                and self._linear_x == 0.0
                and self._linear_y == 0.0
                and self._angular_z == 0.0
            ):
                if self._stop_sent:
                    return
                self._stop_sent = True

            # Publish current velocity (or zero if no owner)
            msg = Twist()
            msg.linear.x = self._linear_x
            msg.linear.y = self._linear_y
            msg.angular.z = self._angular_z
            self._pub.publish(msg)

    def _emit_status_locked(self):
        # Caller already holds lock
        if not self._status_callback:
            return
        status = {
            'owner': self._owner,
            'linear_x': round(self._linear_x, 3),
            'linear_y': round(self._linear_y, 3),
            'angular_z': round(self._angular_z, 3),
            'max_linear': round(self._max_linear, 3),
            'max_angular': round(self._max_angular, 3),
        }
        try:
            self._status_callback(status)
        except Exception as e:
            self._node.get_logger().warn(f'status_callback error: {e}')

    def _emit_status(self):
        with self._lock:
            self._emit_status_locked()
