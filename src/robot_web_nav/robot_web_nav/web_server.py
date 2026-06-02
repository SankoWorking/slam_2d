import asyncio
import json
import math
import os
import threading
import uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from aiohttp import web, WSMsgType
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path

from .log_emitter import LogEmitter
from .map_service import MapService
from .waypoint_manager import WaypointManager
from .nav_action_client import NavActionClient
from .robot_tracker import RobotTracker
from .manual_control import ManualControlManager
from .localization_manager import LocalizationManager


class WebNavNode(Node):
    def __init__(self):
        super().__init__('web_nav_server')

        pkg_share = self._find_share_dir()
        maps_dir = self._resolve_maps_dir()

        self._web_dir = os.path.join(pkg_share, 'web')
        self._log = LogEmitter(self)
        self._map_service = MapService(maps_dir)
        self._waypoint_mgr = WaypointManager(
            os.path.join(maps_dir, 'waypoints.json')
        )
        self._nav_client = NavActionClient(self)
        self._robot_tracker = RobotTracker(self)
        self._control = ManualControlManager(self)
        self._localization = LocalizationManager(self)

        # Publisher for initial pose setting
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        # Subscription for planned path visualization
        self._plan_sub = self.create_subscription(
            Path, '/plan', self._on_plan, 10
        )
        self._last_plan = None

        # Reasonable defaults matching nav2_params.yaml velocity caps
        self._control.set_limits(max_linear=0.3, max_angular=1.0)

        self._current_map: str = 'map'
        self._active_nav_goal = None  # Track current navigation goal for reconnect recovery
        # Each WebSocket gets a client_id and is registered here
        self._ws_clients: set[web.WebSocketResponse] = set()
        self._ws_client_ids: dict[web.WebSocketResponse, str] = {}
        # Log subscribers (WebSocket clients who opted in)
        self._log_subscribers: set[web.WebSocketResponse] = set()

        self._log.info('init', f'Web dir: {self._web_dir}')
        self._log.info('init', f'Maps dir: {maps_dir}')
        self._log.info('init', f'Available maps: {self._map_service.list_maps()}')

    def _on_plan(self, msg: Path):
        """Receive planned path from Nav2 and broadcast to clients."""
        path_points = []
        for pose_stamped in msg.poses:
            p = pose_stamped.pose.position
            path_points.append([round(p.x, 3), round(p.y, 3)])
        self._last_plan = path_points
        if _main_loop is not None:
            try:
                asyncio.run_coroutine_threadsafe(
                    _broadcast(self, {'type': 'planned_path', 'path': path_points}),
                    _main_loop,
                )
            except Exception as e:
                self.get_logger().warn(f'_on_plan broadcast error: {e}')

    def _find_share_dir(self) -> str:
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('robot_web_nav')
            web_path = os.path.join(share_dir, 'web')
            if os.path.isdir(web_path):
                return share_dir
        except Exception:
            pass
        src_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
        if os.path.isdir(os.path.join(src_dir, 'web')):
            return os.path.abspath(src_dir)
        return os.path.abspath(src_dir)

    def _resolve_maps_dir(self) -> str:
        try:
            from ament_index_python.packages import get_package_share_directory
            share_dir = get_package_share_directory('robot_bringup')
            maps_dir = os.path.join(share_dir, 'maps')
            if os.path.isdir(maps_dir):
                return maps_dir
        except Exception:
            pass
        src_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')
        maps_dir = os.path.join(src_dir, 'robot_bringup', 'maps')
        if os.path.isdir(maps_dir):
            return os.path.abspath(maps_dir)
        raise RuntimeError(f'Cannot locate robot_bringup/maps; AMENT_PREFIX_PATH={os.environ.get("AMENT_PREFIX_PATH")}')


def _make_app(node: WebNavNode) -> web.Application:
    app = web.Application()
    app['node'] = node

    app.router.add_get('/ws', websocket_handler)
    app.router.add_get('/', lambda r: web.FileResponse(
        os.path.join(node._web_dir, 'index.html')
    ))
    app.router.add_static('/', node._web_dir, name='static', follow_symlinks=True)
    return app


async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    node: WebNavNode = request.app['node']

    client_id = str(uuid.uuid4())[:8]
    node._ws_clients.add(ws)
    node._ws_client_ids[ws] = client_id

    node._log.info('ws', f'Client connected (id={client_id}), total: {len(node._ws_clients)}')

    # Send current state on connect (includes reconnect recovery)
    try:
        await _send_map_data(ws, node)
        await _send_waypoints(ws, node)
        await _send_map_list(ws, node)
        await ws.send_json({
            'type': 'hello',
            'client_id': client_id,
        })
        await _send_control_status_to(ws, node)
        await _send_localization_status_to(ws, node)
        # Reconnect recovery: send active navigation state
        if node._active_nav_goal:
            await ws.send_json({
                'type': 'nav_status',
                'status': 'navigating',
                'goal': node._active_nav_goal,
            })
        # Send last planned path if available
        if node._last_plan:
            await ws.send_json({
                'type': 'planned_path',
                'path': node._last_plan,
            })
    except Exception as e:
        node._log.warn('ws', f'Error sending initial state to client {client_id}: {e}')

    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                await _handle_message(ws, node, msg.data)
            elif msg.type == WSMsgType.ERROR:
                node._log.error('ws', f'WS error: {ws.exception()}')
    finally:
        node._log_subscribers.discard(ws)
        node._control.force_release_for(client_id)
        node._ws_clients.discard(ws)
        node._ws_client_ids.pop(ws, None)
        node._log.info('ws', f'Client disconnected (id={client_id}), total: {len(node._ws_clients)}')
        await _broadcast_control_status(node)

    return ws


async def _handle_message(ws: web.WebSocketResponse, node: WebNavNode, raw: str):
    try:
        msg = json.loads(raw)
    except json.JSONDecodeError:
        return

    msg_type = msg.get('type', '')
    client_id = node._ws_client_ids.get(ws)

    # Ignore ping/pong (handled by transport)
    if msg_type in ('ping', 'pong'):
        return

    if msg_type == 'load_map':
        map_name = msg.get('map_name', 'map')
        node._current_map = map_name
        await _send_map_data(ws, node)
        await _send_waypoints(ws, node)

    elif msg_type == 'list_maps':
        await _send_map_list(ws, node)

    elif msg_type == 'add_waypoint':
        name = msg.get('name', '').strip()
        if not name:
            return
        px, py = msg.get('pixel_x', 0), msg.get('pixel_y', 0)
        wx, wy = node._map_service.pixel_to_world(node._current_map, px, py)
        wp = node._waypoint_mgr.add_waypoint(name, wx, wy)
        node._log.info('waypoint', f'Added: {name} at ({wx:.2f}, {wy:.2f})')
        await _broadcast(node, {'type': 'waypoint_added', 'waypoint': wp})

    elif msg_type == 'add_waypoint_xy':
        name = msg.get('name', '').strip()
        if not name:
            return
        x, y, yaw = msg.get('x', 0), msg.get('y', 0), msg.get('yaw', 0)
        wp = node._waypoint_mgr.add_waypoint(name, x, y, yaw)
        await _broadcast(node, {'type': 'waypoint_added', 'waypoint': wp})

    elif msg_type == 'delete_waypoint':
        name = msg.get('name', '')
        if node._waypoint_mgr.delete_waypoint(name):
            await _broadcast(node, {'type': 'waypoint_deleted', 'name': name})

    elif msg_type == 'list_waypoints':
        await _send_waypoints(ws, node)

    elif msg_type == 'navigate_to':
        name = msg.get('name', '')
        wp = node._waypoint_mgr.get_waypoint(name)
        if wp:
            node._active_nav_goal = wp
            node._nav_client.send_goal(wp['x'], wp['y'], wp.get('yaw', 0.0))
            await _broadcast(node, {
                'type': 'nav_status', 'status': 'navigating', 'goal': wp
            })
        else:
            await ws.send_json({'type': 'error', 'message': f'Waypoint not found: {name}'})

    elif msg_type == 'navigate_to_xy':
        x, y = msg.get('x', 0), msg.get('y', 0)
        yaw = msg.get('yaw', 0.0)
        goal = {'name': f'({x:.1f}, {y:.1f})', 'x': x, 'y': y, 'yaw': yaw}
        node._active_nav_goal = goal
        node._nav_client.send_goal(x, y, yaw)
        await _broadcast(node, {
            'type': 'nav_status', 'status': 'navigating', 'goal': goal
        })

    elif msg_type == 'cancel_nav':
        node._active_nav_goal = None
        node._nav_client.cancel_goal()
        await _broadcast(node, {'type': 'nav_status', 'status': 'canceled'})

    elif msg_type == 'claim_control':
        if not client_id:
            return
        ok = node._control.claim(client_id)
        await ws.send_json({'type': 'claim_control_result', 'ok': ok})
        await _broadcast_control_status(node)

    elif msg_type == 'release_control':
        if client_id:
            node._control.release(client_id)
        await _broadcast_control_status(node)

    elif msg_type == 'set_velocity':
        if not client_id:
            return
        lx = float(msg.get('linear_x', 0.0))
        ly = float(msg.get('linear_y', 0.0))
        az = float(msg.get('angular_z', 0.0))
        accepted = node._control.set_velocity(client_id, lx, ly, az)
        if not accepted:
            await ws.send_json({
                'type': 'error',
                'message': 'You do not own manual control. Send claim_control first.',
            })

    elif msg_type == 'start_localization':
        ok = node._localization.trigger_global()
        await ws.send_json({
            'type': 'localization_started',
            'ok': ok,
            'message': '' if ok else 'global_localization service not available',
        })

    elif msg_type == 'set_initial_pose':
        x = float(msg.get('x', 0.0))
        y = float(msg.get('y', 0.0))
        yaw = float(msg.get('yaw', 0.0))
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = node.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        # Set moderate initial covariance
        cov = [0.0] * 36
        cov[0] = 0.25   # x
        cov[7] = 0.25   # y
        cov[35] = 0.07  # yaw
        pose_msg.pose.covariance = cov
        node._initial_pose_pub.publish(pose_msg)
        node._log.info('pose', f'Initial pose set: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        await ws.send_json({'type': 'initial_pose_set', 'x': x, 'y': y, 'yaw': yaw})

    elif msg_type == 'set_map_display_name':
        map_id = msg.get('map_id', '')
        display = msg.get('display_name', '')
        try:
            node._map_service.set_display_name(map_id, display)
            await _send_map_list(ws, node)
        except Exception as e:
            await ws.send_json({'type': 'error', 'message': str(e)})

    elif msg_type == 'rename_map':
        old = msg.get('old', '')
        new = msg.get('new', '')
        try:
            new_id = node._map_service.rename_map(old, new)
            await _broadcast(node, {'type': 'map_renamed', 'old': old, 'new': new_id})
            await _send_map_list_all(node)
        except Exception as e:
            await ws.send_json({'type': 'error', 'message': str(e)})

    elif msg_type == 'duplicate_map':
        src = msg.get('src', '')
        new = msg.get('new', '')
        try:
            new_id = node._map_service.duplicate_map(src, new)
            await _broadcast(node, {'type': 'map_duplicated', 'src': src, 'new': new_id})
            await _send_map_list_all(node)
        except Exception as e:
            await ws.send_json({'type': 'error', 'message': str(e)})

    elif msg_type == 'get_robot_pose':
        await ws.send_json({
            'type': 'robot_pose_full',
            **node._robot_tracker.get_last_pose(),
        })

    elif msg_type == 'get_logs':
        entries = node._log.get_recent(msg.get('count', 50))
        await ws.send_json({'type': 'log_entries', 'entries': entries})

    elif msg_type == 'subscribe_logs':
        node._log_subscribers.add(ws)
        node._log.info('logs', f'Client {client_id} subscribed to logs')

    elif msg_type == 'unsubscribe_logs':
        node._log_subscribers.discard(ws)

    else:
        if msg_type:
            await ws.send_json({'type': 'error', 'message': f'Unknown type: {msg_type}'})


async def _send_map_data(ws: web.WebSocketResponse, node: WebNavNode):
    try:
        b64, info = node._map_service.get_map_png_base64(node._current_map)
        await ws.send_json({
            'type': 'map_data',
            'image': b64,
            'width': info.width,
            'height': info.height,
            'resolution': info.resolution,
            'origin': [info.origin_x, info.origin_y],
            'map_name': node._current_map,
            'display_name': node._map_service.get_display_name(node._current_map),
        })
    except Exception as e:
        node._log.error('map', f'Failed to load map: {e}')
        await ws.send_json({'type': 'error', 'message': f'Failed to load map: {str(e)}'})


async def _send_waypoints(ws: web.WebSocketResponse, node: WebNavNode):
    wps = node._waypoint_mgr.list_waypoints()
    await ws.send_json({'type': 'waypoints', 'waypoints': wps})


async def _send_map_list(ws: web.WebSocketResponse, node: WebNavNode):
    await ws.send_json({
        'type': 'map_list',
        'maps': node._map_service.list_maps_with_display(),
        'current': node._current_map,
    })


async def _send_map_list_all(node: WebNavNode):
    msg = json.dumps({
        'type': 'map_list',
        'maps': node._map_service.list_maps_with_display(),
        'current': node._current_map,
    })
    await _broadcast_raw(node, msg)


async def _send_control_status_to(ws: web.WebSocketResponse, node: WebNavNode):
    await ws.send_json({'type': 'control_status', **node._control.get_status()})


async def _send_localization_status_to(ws: web.WebSocketResponse, node: WebNavNode):
    await ws.send_json({'type': 'localization_status', **node._localization.get_status()})


async def _broadcast_control_status(node: WebNavNode):
    await _broadcast(node, {'type': 'control_status', **node._control.get_status()})


async def _broadcast_localization_status(node: WebNavNode, status: dict):
    await _broadcast(node, {'type': 'localization_status', **status})


async def _broadcast(node: WebNavNode, data: dict):
    await _broadcast_raw(node, json.dumps(data))


async def _broadcast_raw(node: WebNavNode, msg_str: str):
    dead = set()
    for ws in node._ws_clients:
        try:
            await ws.send_str(msg_str)
        except Exception:
            dead.add(ws)
    node._ws_clients -= dead


def _broadcast_log_entry(node: WebNavNode, entry: dict):
    """Called from LogEmitter subscriber to push logs to opted-in WebSocket clients."""
    if _main_loop is None:
        return
    if entry.get('level', 'info') == 'info':
        return  # Only stream warn and above
    try:
        asyncio.run_coroutine_threadsafe(
            _broadcast_raw_to(node, json.dumps({'type': 'log_entries', 'entries': [entry]}), node._log_subscribers),
            _main_loop,
        )
    except Exception:
        pass


async def _broadcast_raw_to(node: WebNavNode, msg_str: str, targets: set):
    dead = set()
    for ws in targets:
        try:
            await ws.send_str(msg_str)
        except Exception:
            dead.add(ws)
    node._log_subscribers -= dead


# Shared event loop reference for thread-safe callbacks
_main_loop: asyncio.AbstractEventLoop = None


def _on_nav_result(node: WebNavNode, status: str):
    if _main_loop is None:
        return
    node._active_nav_goal = None
    asyncio.run_coroutine_threadsafe(
        _broadcast(node, {'type': 'nav_result', 'status': status}),
        _main_loop,
    )


def _on_nav_feedback(node: WebNavNode, feedback):
    if _main_loop is None:
        return
    try:
        dist = feedback.distance_remaining
        asyncio.run_coroutine_threadsafe(
            _broadcast(node, {'type': 'nav_feedback', 'distance_remaining': dist}),
            _main_loop,
        )
    except Exception as e:
        node.get_logger().warn(f'_on_nav_feedback error: {e}')


def _on_robot_pose(node: WebNavNode, x: float, y: float, yaw: float, cov_xy, cov_yaw):
    if _main_loop is None:
        return
    try:
        asyncio.run_coroutine_threadsafe(
            _broadcast(node, {
                'type': 'robot_pose',
                'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3),
                'cov_xy': round(cov_xy, 4) if cov_xy is not None else None,
                'cov_yaw': round(cov_yaw, 4) if cov_yaw is not None else None,
            }),
            _main_loop,
        )
    except Exception as e:
        node.get_logger().warn(f'_on_robot_pose error: {e}')


def _on_control_status(node: WebNavNode, status: dict):
    if _main_loop is None:
        return
    try:
        asyncio.run_coroutine_threadsafe(
            _broadcast(node, {'type': 'control_status', **status}),
            _main_loop,
        )
    except Exception as e:
        node.get_logger().warn(f'_on_control_status error: {e}')


def _on_localization_status(node: WebNavNode, status: dict):
    if _main_loop is None:
        return
    try:
        asyncio.run_coroutine_threadsafe(
            _broadcast_localization_status(node, status),
            _main_loop,
        )
    except Exception as e:
        node.get_logger().warn(f'_on_localization_status error: {e}')


def main():
    global _main_loop

    rclpy.init()
    node = WebNavNode()

    executor = MultiThreadedExecutor()
    ros_thread = threading.Thread(
        target=lambda: (executor.add_node(node), executor.spin()),
        daemon=True,
    )
    ros_thread.start()

    app = _make_app(node)

    node._nav_client.set_callbacks(
        result_cb=lambda status: _on_nav_result(node, status),
        feedback_cb=lambda fb: _on_nav_feedback(node, fb),
    )
    node._robot_tracker.set_pose_callback(
        lambda x, y, yaw, cxy, cyaw: _on_robot_pose(node, x, y, yaw, cxy, cyaw)
    )
    node._control.set_status_callback(
        lambda status: _on_control_status(node, status)
    )
    node._localization.set_status_callback(
        lambda status: _on_localization_status(node, status)
    )
    node._robot_tracker.add_covariance_listener(node._localization.on_covariance_update)
    # Wire log emitter to broadcast to subscribed WebSocket clients
    node._log.subscribe(lambda entry: _broadcast_log_entry(node, entry))

    port = int(os.environ.get('WEB_NAV_PORT', '9090'))
    node._log.info('server', f'Starting web server on port {port}')

    async def run():
        global _main_loop
        _main_loop = asyncio.get_running_loop()
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, '0.0.0.0', port)
        await site.start()
        node._log.info('server', f'Running at http://0.0.0.0:{port}')

        try:
            while True:
                await asyncio.sleep(3600)
        except asyncio.CancelledError:
            pass
        finally:
            await runner.cleanup()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        pass
    finally:
        node._log.info('server', 'Shutting down')
        loop.close()
        executor.shutdown()
        rclpy.shutdown()
        ros_thread.join(timeout=3)


if __name__ == '__main__':
    main()
