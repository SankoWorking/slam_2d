import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion


def yaw_to_quaternion(yaw: float) -> tuple:
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


class NavActionClient:
    def __init__(self, node: Node):
        self._node = node
        self._action_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self._goal_handle = None
        self._result_callback = None
        self._feedback_callback = None

    def set_callbacks(self, result_cb=None, feedback_cb=None):
        self._result_callback = result_cb
        self._feedback_callback = feedback_cb

    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self._node.get_logger().warn('NavigateToPose action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        q = yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        self._node.get_logger().info(f'Sending nav goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._internal_feedback,
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().warn('Goal rejected')
            if self._result_callback:
                self._result_callback('rejected')
            return
        self._goal_handle = goal_handle
        self._node.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_done_callback)

    def _result_done_callback(self, future):
        result = future.result()
        status = 'unknown'
        if result.status == 4:  # STATUS_SUCCEEDED
            status = 'succeeded'
        elif result.status == 5:  # STATUS_ABORTED
            status = 'aborted'
        elif result.status == 6:  # STATUS_CANCELED
            status = 'canceled'
        self._node.get_logger().info(f'Navigation result: {status}')
        self._goal_handle = None
        if self._result_callback:
            self._result_callback(status)

    def _internal_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if self._feedback_callback:
            self._feedback_callback(feedback)

    def cancel_goal(self):
        if self._goal_handle is not None:
            self._node.get_logger().info('Canceling navigation goal')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
