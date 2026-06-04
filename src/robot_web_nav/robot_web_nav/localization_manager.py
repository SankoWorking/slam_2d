import threading

from rclpy.node import Node
from std_srvs.srv import Empty


class LocalizationManager:
    """Manages AMCL global localization and convergence monitoring.

    Receives covariance data from RobotTracker (via on_covariance_update)
    instead of maintaining its own /amcl_pose subscription.
    """

    CONVERGE_THRESHOLD = 0.05
    DIVERGE_THRESHOLD = 0.15

    def __init__(self, node: Node):
        self._node = node
        self._status_callback = None
        self._last_cov_xy = float('inf')
        self._last_cov_yaw = float('inf')
        self._localizing = False
        self._was_converged = False
        self._lock = threading.Lock()

        self._cli = node.create_client(Empty, '/reinitialize_global_localization')

    def set_status_callback(self, cb):
        """cb(status_dict) where status_dict has:
            cov_xy, cov_yaw, localizing, converged"""
        self._status_callback = cb

    def trigger_global(self) -> bool:
        """Call /reinitialize_global_localization. Returns True if request was sent."""
        if not self._cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn('reinitialize_global_localization service not available')
            return False
        with self._lock:
            self._localizing = True
            self._was_converged = False
            self._last_cov_xy = float('inf')
            self._last_cov_yaw = float('inf')
        self._node.get_logger().info('Triggering AMCL global localization')
        future = self._cli.call_async(Empty.Request())
        future.add_done_callback(self._on_service_done)
        self._emit_status()
        return True

    def on_covariance_update(self, cov_xy: float, cov_yaw: float):
        """Called by RobotTracker with validated covariance values."""
        changed = False
        with self._lock:
            if abs(cov_xy - self._last_cov_xy) > 1e-4 or abs(cov_yaw - self._last_cov_yaw) > 1e-4:
                changed = True
            self._last_cov_xy = cov_xy
            self._last_cov_yaw = cov_yaw

            # Auto-clear localizing flag when converged
            if self._localizing and cov_xy < self.CONVERGE_THRESHOLD and cov_yaw < self.CONVERGE_THRESHOLD:
                self._localizing = False
                self._was_converged = True

            # Detect re-divergence after convergence
            if self._was_converged and (cov_xy > self.DIVERGE_THRESHOLD or cov_yaw > self.DIVERGE_THRESHOLD):
                self._was_converged = False

        if changed:
            self._emit_status()

    def _on_service_done(self, future):
        try:
            future.result()
            self._node.get_logger().info('reinitialize_global_localization: call succeeded')
        except Exception as e:
            self._node.get_logger().error(f'reinitialize_global_localization call failed: {e}')
            with self._lock:
                self._localizing = False
            self._emit_status()

    def _emit_status(self):
        if not self._status_callback:
            return
        with self._lock:
            status = self._get_status_locked()
        self._status_callback(status)

    def _get_status_locked(self) -> dict:
        converged = (self._last_cov_xy < self.CONVERGE_THRESHOLD
                     and self._last_cov_yaw < self.CONVERGE_THRESHOLD)
        return {
            'cov_xy': round(self._last_cov_xy, 4)
                      if self._last_cov_xy != float('inf') else None,
            'cov_yaw': round(self._last_cov_yaw, 4)
                       if self._last_cov_yaw != float('inf') else None,
            'localizing': self._localizing,
            'converged': converged,
        }

    def get_status(self) -> dict:
        with self._lock:
            return self._get_status_locked()
