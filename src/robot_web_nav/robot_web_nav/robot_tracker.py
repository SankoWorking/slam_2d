import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class RobotTracker:
    """Subscribes to /amcl_pose to broadcast robot pose + localization quality.

    This is the single owner of the /amcl_pose subscription. Other modules
    (e.g. LocalizationManager) register as covariance listeners instead of
    creating their own duplicate subscriptions.
    """

    def __init__(self, node: Node):
        self._node = node
        self._pose_callback = None
        self._cov_listeners = []
        self._lock = threading.Lock()
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_yaw = 0.0
        self._last_cov_xy = None
        self._last_cov_yaw = None
        self._sub = node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._on_pose,
            10,
        )

    def set_pose_callback(self, cb):
        """cb(x, y, yaw, cov_xy, cov_yaw). cov values may be None."""
        self._pose_callback = cb

    def add_covariance_listener(self, cb):
        """cb(cov_xy, cov_yaw). Called from _on_pose with validated values."""
        self._cov_listeners.append(cb)

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        cov = msg.pose.covariance
        cov_xy = math.sqrt(max(0, cov[0] + cov[7]))
        cov_yaw = math.sqrt(max(0, cov[35]))

        with self._lock:
            self._last_x = p.x
            self._last_y = p.y
            self._last_yaw = yaw
            self._last_cov_xy = cov_xy
            self._last_cov_yaw = cov_yaw

        if self._pose_callback:
            try:
                self._pose_callback(p.x, p.y, yaw, cov_xy, cov_yaw)
            except Exception as e:
                self._node.get_logger().warn(f'pose_callback error: {e}')

        for listener in self._cov_listeners:
            try:
                listener(cov_xy, cov_yaw)
            except Exception as e:
                self._node.get_logger().warn(f'covariance_listener error: {e}')

    def get_last_pose(self) -> dict:
        with self._lock:
            return {
                'x': round(self._last_x, 3),
                'y': round(self._last_y, 3),
                'yaw': round(self._last_yaw, 3),
                'cov_xy': round(self._last_cov_xy, 4) if self._last_cov_xy is not None else None,
                'cov_yaw': round(self._last_cov_yaw, 4) if self._last_cov_yaw is not None else None,
            }
