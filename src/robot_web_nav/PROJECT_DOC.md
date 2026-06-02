# robot_web_nav — 项目技术文档

## 1. 背景与目的

`robot_web_nav` 是一个 ROS2 Python 包，为已建好 SLAM 地图的移动机器人提供 **Web 端 2D 地图可视化**和**点击导航**能力。它解决了以下问题：

- 原有系统只能通过 RViz 发送导航目标，无法远程操作
- 没有命名路径点管理机制，每次导航需要手动输入坐标
- 缺少移动端友好的操控界面

**目标硬件**：NVIDIA Jetson Orin Nano (ARM64, 6核, 7.4GB RAM)
**ROS2 发行版**：Humble
**运行端口**：9090（HTTP + WebSocket）

---

## 2. 包结构与文件索引

```
src/robot_web_nav/
├── package.xml                      # ament_python 包定义，依赖声明
├── setup.py                         # 构建配置，入口点 web_nav_server
├── setup.cfg                        # 安装路径映射
├── resource/robot_web_nav           # ament_python 资源标记（空文件）
├── launch/
│   └── web_nav.launch.py            # 启动文件
├── robot_web_nav/
│   ├── __init__.py
│   ├── web_server.py                # 【核心】aiohttp HTTP+WS 服务器 + rclpy 桥接
│   ├── map_service.py               # PGM/YAML 地图加载 + 坐标转换
│   ├── waypoint_manager.py          # 路径点 CRUD + JSON 持久化
│   ├── nav_action_client.py         # Nav2 NavigateToPose 动作客户端
│   └── robot_tracker.py             # tf2 机器人位姿追踪
└── web/
    ├── index.html                   # 单页应用入口
    ├── css/style.css                # 响应式样式（桌面/移动）
    └── js/
        ├── app.js                   # WebSocket 连接、消息路由、UI 逻辑
        ├── map.js                   # Canvas 地图渲染、平移缩放、坐标转换
        └── robot.js                 # 机器人显示（预留扩展）
```

---

## 3. 关键点说明

### 3.1 双线程架构 (web_server.py)

系统同时运行 rclpy 和 aiohttp 两个事件循环，通过线程桥接：

```
主线程                              ROS 线程 (daemon)
┌─────────────────────────┐        ┌──────────────────────────┐
│ asyncio event loop      │        │ rclpy MultiThreaded      │
│  ├─ aiohttp AppRunner   │        │  Executor.spin()         │
│  ├─ WebSocket handlers  │        │  ├─ NavActionClient      │
│  └─ _broadcast()        │◄───────┤  ├─ RobotTracker         │
│                         │  run_   │  └─ WebNavNode           │
│ _main_loop (全局引用)    │  coro_  │                          │
│                         │  thread │                          │
└─────────────────────────┘  safe()└──────────────────────────┘
```

**关键实现** (web_server.py:268-322)：
- `_main_loop` 全局变量保存主线程的 asyncio 事件循环引用
- ROS 回调（导航结果、机器人位姿）通过 `asyncio.run_coroutine_threadsafe()` 跨线程投递到主循环
- rclpy 使用 `MultiThreadedExecutor` 支持从任意线程调用回调

### 3.2 坐标转换系统 (map_service.py + map.js)

ROS 地图使用**世界坐标系**（x 向右，y 向上），PGM 图像使用**像素坐标系**（y 向下），浏览器 Canvas 使用**屏幕坐标系**（y 向下 + 缩放/平移偏移）。三级转换关系：

```
世界坐标 (meters)              像素坐标               屏幕坐标 (pixels)
wx = origin_x + px * res      px = (wx - ox) / res   sx = px * scale + offsetX
wy = origin_y + (H-py)*res    py = H - (wy - oy)/res  sy = py * scale + offsetY
```

**Python 端** (map_service.py:67-77) — 服务端像素→世界转换，用于路径点保存和导航：
```python
def pixel_to_world(self, map_name, px, py):
    wx = origin_x + px * resolution
    wy = origin_y + (height - py) * resolution  # Y 轴翻转
```

**JS 端** (map.js:153-178) — 前端世界→屏幕转换，用于地图渲染：
```javascript
function worldToScreen(wx, wy) {
    px = (wx - origin[0]) / resolution;
    py = height - (wy - origin[1]) / resolution;
    return [px * viewScale + viewOffset.x, py * viewScale + viewOffset.y];
}
```

**现有地图参数**：
| 地图 | 分辨率 | Origin | 尺寸 |
|------|--------|--------|------|
| `map` (办公室) | 0.05 m/px | [-5.62, -2.98] | 209×330 px |
| `gongxun.map` | 0.05 m/px | [-10.1, -32.0] | 1437×931 px |

### 3.3 导航动作客户端 (nav_action_client.py)

通过 `nav2_msgs/action/NavigateToPose` 与 Nav2 导航栈交互。

**调用链**：
```
浏览器 "navigate_to" → web_server._handle_message()
  → waypoint_manager.get_waypoint(name)    # 查找路径点坐标
  → nav_action_client.send_goal(x, y, yaw) # 发送 Nav2 动作目标
    → ActionClient.send_goal_async()
      → _goal_response_callback()           # 目标被接受/拒绝
      → _result_done_callback()             # 导航完成/失败/取消
        → _on_nav_result()                  # 跨线程回调
          → _broadcast() → WebSocket        # 通知前端
```

**四元数转换** (nav_action_client.py:10-12) — 从 yaw 角生成四元数，避免外部依赖：
```python
def yaw_to_quaternion(yaw):
    half = yaw / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))
```

**动作状态码映射** (nav_action_client.py:67-73)：
| Nav2 status code | 含义 | 文本 |
|-----------------|------|------|
| 4 | STATUS_SUCCEEDED | `succeeded` |
| 5 | STATUS_ABORTED | `aborted` |
| 6 | STATUS_CANCELED | `canceled` |

### 3.4 机器人位姿追踪 (robot_tracker.py)

通过 tf2 查询 `map → base_link` 变换获取 AMCL 校正后的位姿。

**TF 链**：
```
map ──(AMCL 发布)──→ odom ──(里程计发布)──→ base_link
                                              ├── imu_link (静态)
                                              └── laser_frame (静态)
```

**更新频率**：0.5 秒一次定时器 (robot_tracker.py:16)
**数据流**：`tf2 lookup` → callback → `run_coroutine_threadsafe` → `_broadcast` → WebSocket → 前端 Canvas 绘制绿色圆点+方向箭头

### 3.5 路径点持久化 (waypoint_manager.py)

- 存储路径：`robot_bringup/maps/waypoints.json`
- 格式：`{"waypoints": [{"name": "会议室", "x": 1.5, "y": 3.2, "yaw": 0.0}, ...]}`
- 线程安全：使用 `threading.Lock` 保护所有读写操作
- 同名覆盖：添加已存在名称的路径点会更新坐标而非重复创建

### 3.6 前端交互 (map.js + app.js)

**操作方式**：
| 操作 | 桌面 | 移动端 |
|------|------|--------|
| 平移 | 鼠标拖拽 | 单指拖拽 |
| 缩放 | 滚轮 | 双指捏合 |
| 添加路径点 | 双击地图 | 暂未实现（需添加长按） |
| 导航 | 侧边栏"导航"按钮 | 同左 |
| 取消导航 | 顶部状态栏按钮 | 同左 |

**Canvas 渲染层次** (map.js:63-119)：背景色 → PGM 地图图像 → 路径点标记（蓝色圆+白色文字）→ 机器人位姿（绿色圆+方向箭头）→ 待确认点击标记（橙色虚线圆）

---

## 4. 接口详情

### 4.1 WebSocket 协议

端点：`ws://<host>:9090/ws`

所有消息均为 JSON，必须包含 `type` 字段。

#### 客户端 → 服务器

| type | 参数 | 说明 |
|------|------|------|
| `load_map` | `map_name: string` | 切换地图，返回地图图片和元数据 |
| `list_maps` | 无 | 返回可用地图名称列表 |
| `add_waypoint` | `name, pixel_x, pixel_y` | 按像素坐标添加路径点（服务端转世界坐标） |
| `add_waypoint_xy` | `name, x, y, yaw?` | 按世界坐标添加路径点 |
| `delete_waypoint` | `name` | 删除路径点 |
| `list_waypoints` | 无 | 请求当前路径点列表 |
| `navigate_to` | `name` | 按路径点名称导航 |
| `navigate_to_xy` | `x, y, yaw?` | 按坐标直接导航 |
| `cancel_nav` | 无 | 取消正在执行的导航 |
| `get_robot_pose` | 无 | 预留（位姿通过 robot_pose 持续推送） |

#### 服务器 → 客户端

| type | 数据 | 说明 |
|------|------|------|
| `map_data` | `image(base64 PNG), width, height, resolution, origin[x,y], map_name` | 地图图片和元数据 |
| `map_list` | `maps: string[]` | 可用地图列表 |
| `waypoints` | `waypoints: [{name, x, y, yaw}]` | 完整路径点列表 |
| `waypoint_added` | `waypoint: {name, x, y, yaw}` | 新增/更新路径点 |
| `waypoint_deleted` | `name` | 已删除路径点 |
| `robot_pose` | `x, y, yaw` (2Hz 推送) | 机器人实时位姿 |
| `nav_status` | `status, goal` | 导航状态变更 |
| `nav_result` | `status: succeeded/aborted/canceled/rejected` | 导航最终结果 |
| `nav_feedback` | `distance_remaining: float` | 导航中间反馈 |
| `error` | `message` | 错误信息 |

#### 连接生命周期

1. 客户端连接 WebSocket → 服务器自动推送 `map_data` + `waypoints`
2. 客户端发送 `list_maps` → 服务器返回 `map_list`
3. 持续接收 `robot_pose`（2Hz）和 `nav_feedback`（导航中）
4. 断开后客户端每 3 秒自动重连

### 4.2 HTTP 路由

| 路由 | 方法 | 说明 |
|------|------|------|
| `/` | GET | 返回 index.html |
| `/ws` | GET (Upgrade) | WebSocket 端点 |
| `/js/*`, `/css/*` | GET | 静态资源文件 |

### 4.3 ROS2 接口依赖

| 类型 | 名称 | 方向 | 消息类型 |
|------|------|------|----------|
| Action Client | `/navigate_to_pose` | → Nav2 | `nav2_msgs/action/NavigateToPose` |
| TF Lookup | `map → base_link` | ← AMCL+odom | `geometry_msgs/TransformStamped` |

---

## 5. 调试指南

### 5.1 启动顺序与前置条件

```bash
# 1. 编译
cd ~/Desktop/agent-glm/robot_ws
colcon build --packages-select robot_web_nav
source install/setup.bash

# 2. 启动导航栈（必须先启动，提供 AMCL + Nav2 动作服务器）
ros2 launch robot_bringup navigation.launch.py

# 3. 启动 Web 服务
ros2 run robot_web_nav web_nav_server

# 4. 浏览器访问
# http://<jetson-ip>:9090
```

**端口可通过环境变量修改**：`WEB_NAV_PORT=8080 ros2 run robot_web_nav web_nav_server`

### 5.2 日志查看

```bash
# 实时查看 Web 服务器日志（已在终端输出）
# 关键日志行：
#   [web_nav_server] Web dir: /path/to/web          — 静态文件目录
#   [web_nav_server] Maps dir: /path/to/maps         — 地图目录
#   [web_nav_server] Available maps: ['map', ...]    — 发现的地图
#   [web_nav_server] Server running at http://0.0.0.0:9090
#   [web_nav_server] WebSocket client connected      — 客户端连接
#   [web_nav_server] Waypoint added: xxx at (x, y)   — 路径点操作
#   [web_nav_server] Sending nav goal: x=X, y=Y      — 发送导航目标
#   [web_nav_server] Navigation result: succeeded     — 导航结果

# ROS2 日志级别调整
ros2 run robot_web_nav web_nav_server --ros-args --log-level debug
```

### 5.3 常见问题排查

#### 问题：页面打开但地图不显示

**排查步骤**：
1. 检查终端是否有 `Failed to load map` 错误日志
2. 确认地图文件存在：`ls src/robot_bringup/maps/` 应有 `map.yaml` 和 `map.pgm`
3. 确认 `map.yaml` 中 `image` 字段引用的文件名正确（注意 `gongxun.map.yaml` 中有多余空格）
4. 浏览器 F12 → Network → 查看 WebSocket 消息中是否有 `map_data`

#### 问题：机器人位置不更新（地图上无绿点）

**排查步骤**：
1. 检查 TF 树完整性：
   ```bash
   ros2 run tf2_tools view_frames
   # 应生成 map → odom → base_link → {imu_link, laser_frame}
   ```
2. 检查 AMCL 是否运行：
   ```bash
   ros2 topic echo /amcl_pose --once
   ```
3. 检查 odom 是否发布：
   ```bash
   ros2 topic hz /odom
   ```
4. 若 TF 缺少 `map → odom`：AMCL 未启动或未收敛，检查导航栈是否运行
5. 若 TF 缺少 `odom → base_link`：`robot_odometry` 节点未运行

#### 问题：点击"导航"后机器人不动

**排查步骤**：
1. 检查终端日志：
   - `NavigateToPose action server not available` → Nav2 未启动
   - `Goal rejected` → 目标点在障碍物上或 costmap 中不可达
2. 手动验证导航栈可用：
   ```bash
   ros2 action list  # 应看到 /navigate_to_pose
   ros2 action info /navigate_to_pose
   ```
3. 检查路径点坐标是否合理（在地图范围内，不在障碍物上）

#### 问题：WebSocket 连接失败

**排查步骤**：
1. 确认服务器已启动：`curl http://<jetson-ip>:9090/` 应返回 HTML
2. 检查防火墙：`sudo ufw status`，如有启用则 `sudo ufw allow 9090`
3. 浏览器 F12 → Console 查看连接错误
4. 本机测试：`curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" -H "Sec-WebSocket-Version: 13" -H "Sec-WebSocket-Key: test http://localhost:9090/ws`

#### 问题：地图上路径点位置与实际不符

**排查步骤**：
1. 确认坐标转换参数：检查 `map.yaml` 中的 `resolution` 和 `origin` 是否与实际地图匹配
2. 验证像素→世界坐标转换：
   ```python
   from robot_web_nav.map_service import MapService
   ms = MapService('src/robot_bringup/maps/')
   print(ms.pixel_to_world('map', 100, 100))
   ```
3. 检查前端 Canvas 缩放和平移状态（`viewScale`, `viewOffset`）

### 5.4 断点设置建议

**Python 后端**（推荐使用 VS Code 远程调试）：
- `web_server.py:119` — `_handle_message()` 入口，所有 WebSocket 消息经过此处
- `web_server.py:141` — `pixel_to_world()` 调用点，验证坐标转换
- `nav_action_client.py:27` — `send_goal()` 入口，检查目标参数
- `robot_tracker.py:22` — `_tick()` 定时回调，检查 TF 查询结果
- `waypoint_manager.py:31` — `add_waypoint()` 入口，检查持久化逻辑

**JavaScript 前端**（浏览器 DevTools）：
- `app.js:46` — `handleMessage()` 入口，所有服务器消息路由
- `map.js:39` — `onMapData()` 地图加载回调
- `map.js:218` — `onMapDblClick()` 双击添加路径点处理
- `map.js:153` — `worldToScreen()` 坐标转换函数

---

## 6. 与其他系统集成要点

### 6.1 依赖关系图

```
robot_web_nav
  ├── rclpy                    (ROS2 Python 客户端库)
  ├── nav2_msgs                (NavigateToPose 动作定义)
  ├── geometry_msgs            (PoseStamped, Quaternion)
  ├── nav_msgs                 (Odometry — 间接通过 TF)
  ├── tf2_ros                  (TransformListener)
  ├── tf2_geometry_msgs        (TF2 几何工具)
  ├── aiohttp                  (HTTP + WebSocket 服务器) [系统 Python 包]
  ├── Pillow                   (PGM 图像读取) [系统 Python 包]
  ├── PyYAML                   (地图元数据解析) [系统 Python 包]
  │
  ├── robot_bringup (运行时)   ← 提供 maps/ 目录和 Nav2 launch
  ├── Nav2 导航栈 (运行时)     ← 提供 /navigate_to_pose 动作服务器
  ├── AMCL (运行时)            ← 提供 map→odom TF
  └── robot_odometry (运行时)  ← 提供 odom→base_link TF
```

### 6.2 与 Nav2 导航栈交互

| 阶段 | robot_web_nav | Nav2 |
|------|---------------|------|
| 目标发送 | `NavigateToPose.Goal(pose=PoseStamped)` | BT Navigator 接收 |
| 路径规划 | 等待 | SmacPlanner2D 生成路径 |
| 路径跟踪 | 接收 feedback | MPPI Omni Controller 执行 |
| 结果返回 | 收到 status code | BT Navigator 报告结果 |

**Nav2 配置约束** (来自 `nav2_params.yaml`)：
- 全局坐标系：`map`
- 机器人坐标系：`base_link`
- 目标容差：xy=0.15m, yaw=0.2rad
- 最大速度：vx=0.3m/s, vy=0.2m/s, wz=1.2rad/s
- 规划器：SmacPlanner2D (A*)，最大 2 秒
- 控制器：MPPI Omni（全向运动模型）

### 6.3 地图文件兼容性

**支持的地图格式**：标准 ROS 地图（SLAM Toolbox / Cartographer / Gmapping 生成）
- YAML 元数据 + PGM (P5 binary) 或 PNG 图像
- YAML 必须包含 `image`, `resolution`, `origin` 字段
- PGM 像素值约定：0=占据(黑), 205=未知(灰), 254=自由(白)

**已知问题**：`gongxun.map.yaml` 的 `image` 字段值 `"map.pgm "` 末尾有多余空格，代码中已用 `.strip()` 处理 (map_service.py:41)。

### 6.4 外部系统集成接口

其他系统可通过以下方式与 `robot_web_nav` 交互：

**方式一：直接 WebSocket 连接**
```python
import asyncio, websockets, json

async def navigate():
    async with websockets.connect('ws://jetson-ip:9090/ws') as ws:
        # 添加路径点
        await ws.send(json.dumps({
            'type': 'add_waypoint_xy',
            'name': '目标A', 'x': 1.0, 'y': 2.0, 'yaw': 0.0
        }))
        # 触发导航
        await ws.send(json.dumps({
            'type': 'navigate_to', 'name': '目标A'
        }))
        # 接收结果
        while True:
            msg = json.loads(await ws.recv())
            if msg['type'] == 'nav_result':
                print(f"导航结果: {msg['status']}")
                break
```

**方式二：直接调用 ROS2 动作**（绕过 Web 层）
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}}}}"
```

**方式三：读写 waypoints.json 文件**
- 路径：`src/robot_bringup/maps/waypoints.json`
- 格式：`{"waypoints": [{"name": "xxx", "x": 1.0, "y": 2.0, "yaw": 0.0}]}`
- 修改后需重启 Web 服务生效（或在下一个 WebSocket 消息时刷新列表）

### 6.5 网络与安全注意事项

- 服务监听 `0.0.0.0:9090`，同一局域网内所有设备可访问
- 无身份认证机制，建议仅在可信网络中使用
- WebSocket 无加密（ws://），如需安全传输需前置 Nginx 反向代理 + TLS
- 浏览器访问跨域：前端与 WebSocket 同源（同端口），无 CORS 问题

---

## 7. 测试验证

### 7.1 编译验证

```bash
cd ~/Desktop/agent-glm/robot_ws
colcon build --packages-select robot_web_nav
# 预期：编译成功，无错误
```

验证安装文件：
```bash
find install/robot_web_nav/share/robot_web_nav -type f | sort
# 预期输出应包含：
#   web/index.html
#   web/css/style.css
#   web/js/app.js
#   web/js/map.js
#   web/js/robot.js
#   launch/web_nav.launch.py
```

### 7.2 模块导入验证

```bash
source install/setup.bash
python3 -c "
from robot_web_nav.web_server import WebNavNode
from robot_web_nav.map_service import MapService
from robot_web_nav.waypoint_manager import WaypointManager
from robot_web_nav.nav_action_client import NavActionClient
from robot_web_nav.robot_tracker import RobotTracker
print('All imports OK')
"
# 预期：All imports OK
```

### 7.3 功能验证清单

以下测试需在启动导航栈后进行：

| # | 测试项 | 操作 | 预期结果 |
|---|--------|------|----------|
| 1 | 页面加载 | 浏览器打开 `http://<ip>:9090` | 显示地图，左侧边栏显示"点击地图添加路径点" |
| 2 | 地图切换 | 下拉框选择 `gongxun.map` | 地图切换为较大地图 |
| 3 | 添加路径点 | 双击地图，输入"test"，点确定 | 地图上出现蓝色圆点标记，侧边栏显示路径点 |
| 4 | 路径点持久化 | 刷新页面 | 路径点仍存在 |
| 5 | 删除路径点 | 点击"删除"按钮 | 路径点从地图和列表中移除 |
| 6 | 机器人显示 | （导航栈运行中） | 地图上显示绿色圆点+方向箭头，2Hz 更新 |
| 7 | 触发导航 | 点击路径点"导航"按钮 | 顶部显示"正在导航到..."，机器人开始移动 |
| 8 | 导航反馈 | 观察导航过程 | 顶部显示剩余距离 |
| 9 | 导航结果 | 等待导航完成 | 显示结果（succeeded/aborted） |
| 10 | 取消导航 | 导航中点击"取消导航" | 机器人停止，状态恢复 |
| 11 | 移动端 | 手机浏览器访问 | 界面自适应，地图上方、侧边栏下方 |
| 12 | 连接恢复 | 重启 Web 服务 | 浏览器 3 秒内自动重连 |

### 7.4 性能预估

| 指标 | 预估值 | 说明 |
|------|--------|------|
| 内存占用 | ~60 MB | aiohttp + rclpy + 地图缓存 |
| CPU 占用 | < 3% | 事件驱动，2Hz 位姿推送 |
| 地图传输延迟 | < 100ms | 小地图 ~90KB base64，大地图 ~1.8MB |
| 位姿推送频率 | 2 Hz | robot_tracker 定时器 0.5s |
| 并发客户端 | 5-10 | 受 Jetson 带宽限制，非服务器性能 |

---

## 8. 后续优化方向

### 8.1 语音交互（已规划，暂未实现）

- **方案**：浏览器 Web Speech API 进行中文语音识别
- **命令模式**：`"去{路径点名}"` → 匹配路径点 → 触发导航
- **扩展点**：voice_command.py 已预留解析逻辑，前端 voice.js 文件已创建
- **备选方案**：接入本地 Vosk 模型（`/home/sanko/Desktop/agent-glm/voicebot/models/` 已有中文模型）实现离线语音识别

### 8.2 地图交互增强

- **长按添加路径点**：移动端缺少双击操作，需添加长按 500ms 触发
- **拖拽路径点**：允许拖动已有路径点调整位置
- **设置初始位姿**：点击地图发送 `/initialpose` 帮助 AMCL 初始化
- **路径显示**：订阅 `/plan` 话题在地图上绘制规划路径

### 8.3 集成 voicebot

同项目下已有 `voicebot` 系统（aiohttp + Vosk ASR + DeepSeek LLM + TTS），未来可：
- 统一 Web 端口，合并界面
- 复用 Vosk 实现离线语音识别
- 通过 LLM 理解模糊导航指令（"去开会的地方" → "会议室"）

### 8.4 多地图路径点隔离

当前所有地图共享一个 `waypoints.json`，切换地图后路径点坐标不匹配。应按地图名称分文件存储：`waypoints_map.json`, `waypoints_gongxun.map.json`。

### 8.5 导航偏航角设置

当前所有路径点的 `yaw` 默认为 0。可扩展为：
- 添加路径点时记录从机器人到路径点的方向角
- 前端 UI 允许手动设置朝向
- 导航到达后自动转向目标朝向
