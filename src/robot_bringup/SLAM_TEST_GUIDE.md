# SLAM 导航测试运行指南(办公室小地图)

> 适用范围:在已建好的办公室小地图(`map.pgm`/`map.yaml`)基础上,使用 Nav2 + AMCL + `robot_web_nav` Web UI 进行**点击导航功能测试**。
> 测试平台:NVIDIA Jetson Orin Nano(ARM64,6 核,7.4 GB RAM),ROS2 Humble。
> 文档面向技术人员,可按章节顺序独立完成测试。

---

## 1. 环境准备

### 1.1 硬件清单

| 类别 | 设备 | 用途 | 备注 |
|------|------|------|------|
| 计算平台 | Jetson Orin Nano + 5V/4A 电源 | 跑全套 ROS2/Nav2/Web | 已部署 JetPack + ROS2 Humble |
| SLAM/导航传感器 | RPLidar 雷达 | 提供 `/scan` 给 AMCL/Nav2 | 通过 USB 接入,见 `robot_lidar` 包 |
| IMU | 板载/外接 IMU | 里程计姿态融合 | 通过 `robot_imu` 包,依赖 `imu_filter_madgwick` |
| 下位机 | MCU(STM32 等) | 编码器读取 + 电机控制 | **必须接在 `/dev/ttyACM0`,波特率 115200** |
| 测试终端 | 笔记本/手机(同局域网) | 浏览器访问 Web UI | 推荐 Chrome 最新版 |
| 网络 | 5GHz Wi-Fi 或有线 | 浏览器 ↔ Jetson 通信 | 同一子网,延迟 < 50 ms |

### 1.2 软件依赖与版本

| 组件 | 版本 | 验证命令 |
|------|------|----------|
| Ubuntu | 22.04 LTS(Jetson 版) | `lsb_release -a` |
| ROS2 | Humble | `ros2 --version` 或 `echo $ROS_DISTRO`(应输出 `humble`) |
| Nav2 | Humble 配套版本 | `ros2 pkg list \| grep nav2_bringup` |
| Python | 3.10+ | `python3 --version` |
| aiohttp | ≥ 3.8(系统 Python) | `python3 -c "import aiohttp; print(aiohttp.__version__)"` |
| Pillow | 任意 | `python3 -c "import PIL; print(PIL.__version__)"` |
| PyYAML | 任意 | `python3 -c "import yaml; print(yaml.__version__)"` |
| 浏览器 | Chrome/Edge/Firefox 最新版 | — |

### 1.3 系统与权限准备

```bash
# 1. 把当前用户加入 dialout,允许访问串口和雷达 USB 设备
sudo usermod -aG dialout $USER
# 注销重新登录后生效;不重启可临时:
sudo chmod 666 /dev/ttyACM0

# 2. 检查关键设备节点
ls -l /dev/ttyACM0          # 下位机串口(必须存在)
lsusb | grep -i cp210\|ftdi\|ch340   # 雷达 USB 适配器(型号视具体型号)

# 3. 确认 ROS2 环境
source /opt/ros/humble/setup.bash
source ~/Desktop/agent-glm/robot_ws/install/setup.bash
ros2 pkg list | grep -E "robot_|nav2_bringup|sllidar"

# 4. 工作空间编译(若尚未编译或代码有变更)
cd ~/Desktop/agent-glm/robot_ws
colcon build --symlink-install
source install/setup.bash
```

### 1.4 网络环境

- **Web 服务监听端口**:Jetson 上 `9090/tcp`(可由 `WEB_NAV_PORT` 环境变量覆盖)。
- **同网段要求**:测试终端与 Jetson 处于同一子网,如 `192.168.x.x/24`。
- **防火墙**:若启用 ufw,执行 `sudo ufw allow 9090/tcp`。
- **获取 Jetson IP**:`ip addr show` 或 `hostname -I`。
- **浏览器侧无 CORS 问题**:Web 与 WebSocket 同源同端口。

---

## 2. 地图加载

### 2.1 文件路径

办公室小地图位于 `robot_bringup` 包的 `maps/` 目录:

```
src/robot_bringup/maps/
├── map.pgm              # 占据栅格图(209 × 330 px)
├── map.yaml             # 元数据
├── gongxun.map.pgm      # 大地图(本测试不使用)
├── gongxun.map.yaml
└── waypoints.json       # 路径点(所有地图共享,注意 §2.4)
```

绝对路径:`/home/sanko/Desktop/agent-glm/robot_ws/src/robot_bringup/maps/map.yaml`

### 2.2 元数据格式验证

`map.yaml` 应包含以下字段,缺失或异常会导致 Nav2 map_server 启动失败:

```yaml
image: map.pgm            # 与 yaml 同目录的 PGM 文件名
mode: trinary             # 取值:trinary(默认)/scale/raw
resolution: 0.05          # 米/像素,> 0
origin: [-5.62, -2.98, 0] # [x, y, yaw],地图左下角像素在 world 下的位姿
negate: 0                 # 0 或 1
occupied_thresh: 0.65     # 概率阈值
free_thresh: 0.25
```

验证命令:

```bash
cat src/robot_bringup/maps/map.yaml
python3 -c "
import yaml
with open('src/robot_bringup/maps/map.yaml') as f:
    d = yaml.safe_load(f)
assert d['resolution'] > 0, 'resolution 必须为正'
assert len(d['origin']) == 3, 'origin 必须为三元组'
print('map.yaml OK:', d)
"
```

### 2.3 PGM 完整性检查

```bash
# 1. 文件存在与大小
ls -lh src/robot_bringup/maps/map.pgm
# 2. 校验 P5 binary 格式 + 尺寸与 yaml 中期望一致
python3 -c "
from PIL import Image
im = Image.open('src/robot_bringup/maps/map.pgm')
print('format=', im.format, 'mode=', im.mode, 'size=', im.size)
assert im.format == 'PGM' and im.mode in ('L', 'P')
print('PGM OK,期望尺寸约 209x330,实际:', im.size)
"
# 3. 目视检查(可选)
xdg-open src/robot_bringup/maps/map.pgm
```

**已知问题**:`gongxun.map.yaml` 的 `image` 字段历史上含多余空格,本测试不涉及,但若误改成 `gongxun.map` 测试,需要保留 `MapService` 中的 `.strip()` 处理。

### 2.4 路径点文件检查(可选,但建议)

```bash
cat src/robot_bringup/maps/waypoints.json
# 格式:
# {"waypoints": [{"name": "xxx", "x": 1.5, "y": 3.2, "yaw": 0.0}, ...]}
python3 -c "
import json
d = json.load(open('src/robot_bringup/maps/waypoints.json'))
for w in d['waypoints']:
    assert all(k in w for k in ('name','x','y','yaw')), w
print('waypoints OK, 共', len(d['waypoints']), '个')
"
```

> ⚠️ **重要提醒**:当前 `waypoints.json` 在所有地图间共享,切到非办公室地图时这些坐标会失效。本测试只使用办公室地图,无需处理此问题,但需知晓。

---

## 3. 浏览器选点导航功能

### 3.1 界面布局

打开浏览器访问 `http://<jetson-ip>:9090`,界面结构如下:

```
┌────────────────────────────────────────────────────────┐
│ 顶部状态栏  [当前地图] [导航状态] [取消导航]            │
├──────────────┬─────────────────────────────────────────┤
│              │                                         │
│ 左侧边栏     │           地图 Canvas                   │
│  - 地图切换  │   ┌──────────────────────────────┐       │
│  - 路径点    │   │  ▓▓▓▓  障碍物                │       │
│    列表      │   │       ●  路径点(蓝)          │       │
│    导航/删除 │   │       ▲  机器人(绿,2Hz)    │       │
│              │   │  ░░░░  未知区/未探索          │       │
│              │   └──────────────────────────────┘       │
└──────────────┴─────────────────────────────────────────┘
```

### 3.2 操作方式

| 操作 | 桌面 | 移动端 |
|------|------|--------|
| 平移地图 | 鼠标拖拽 | 单指拖拽 |
| 缩放 | 鼠标滚轮 | 双指捏合 |
| 添加路径点 | **双击地图空白处** | 暂不支持(需后续加长按) |
| 删除路径点 | 侧边栏点"删除" | 同左 |
| 触发导航 | 侧边栏点"导航" | 同左 |
| 取消导航 | 顶部"取消导航" | 同左 |

### 3.3 选点与导航指令流程

**双击添加路径点的完整数据流**:

```
用户双击 Canvas
  └─ map.js:onMapDblClick() → 弹出 prompt 输入名称
      └─ WebSocket 发送 {type:"add_waypoint", name, pixel_x, pixel_y}
          └─ web_server._handle_message()
              └─ WaypointManager.add_waypoint()
                  ├─ pixel_to_world() 转世界坐标(用 map.yaml 的 origin/resolution)
                  └─ 写入 waypoints.json
                      └─ 广播 {type:"waypoint_added"} → 所有客户端刷新
```

**点击"导航"按钮的数据流**:

```
侧边栏点"导航" → 发送 {type:"navigate_to", name:"<路径点名>"}
  └─ web_server → WaypointManager.get_waypoint(name)
      └─ NavActionClient.send_goal(x, y, yaw)
          └─ Nav2 /navigate_to_pose Action
              ├─ 反馈:nav_feedback (distance_remaining)
              └─ 结果:nav_result (succeeded/aborted/canceled/rejected)
                  └─ WebSocket 广播 → 浏览器顶部状态栏显示
```

**直接坐标导航**(不通过路径点):

```javascript
// 在浏览器 Console 执行(已连接 ws://... 后):
ws.send(JSON.stringify({type:"navigate_to_xy", x:1.0, y:2.0, yaw:0.0}));
```

### 3.4 坐标转换要点(便于理解测试结果)

| 坐标系 | 朝向 | 转换 |
|--------|------|------|
| World(ROS) | +x 右,+y 上,米 | `wx = origin_x + px * res`<br>`wy = origin_y + (H - py) * res` |
| PGM 像素 | +x 右,+y 下 | `px = (wx - ox) / res`<br>`py = H - (wy - oy) / res` |
| Canvas 屏幕 | +x 右,+y 下 + 缩放偏移 | 见 `web/js/map.js:worldToScreen` |

办公室地图参数:`resolution=0.05`,`origin=[-5.62, -2.98]`,`H=330 px`,因此实际覆盖的世界范围约为 `x ∈ [-5.62, 4.83]`、`y ∈ [-2.98, 13.52]`(米)。

---

## 4. 测试步骤

### 4.1 启动顺序(必须按此顺序)

```bash
# 终端 A —— 编译 + 环境源(每个新终端都要做)
cd ~/Desktop/agent-glm/robot_ws
source install/setup.bash

# 终端 A —— 硬件驱动(串口 + IMU + 雷达 + 里程计)
ros2 launch robot_bringup bringup.launch.py
# 预期日志:
#   [serial_reader_node] Serial Started:/dev/ttyACM0
#   各节点 "Started" 且无红字错误

# 终端 B —— Nav2 导航栈(加载办公室地图 + AMCL + 规划/控制)
ros2 launch robot_bringup navigation.launch.py
# 预期日志:
#   Loading map from: .../robot_bringup/maps/map.yaml
#   AMCL 用 nav2_params.yaml 中写死的 initial_pose: x=-2.64, y=4.39, yaw=-1.59
#   lifecycle_manager: Successfully configured/brought up [...]
```

> 注:`nav2_params.yaml` 中 `amcl.initial_pose` 已硬编码为办公室地图中的合法起点。**如果换其他地图,必须先改这个值,否则 AMCL 初始化会失败或定位错乱**。

```bash
# 终端 C —— Web 服务
ros2 run robot_web_nav web_nav_server
# 预期日志:
#   [web_nav_server] Web dir: ...
#   [web_nav_server] Maps dir: ...
#   [web_nav_server] Available maps: ['map', 'gongxun.map']
#   [web_nav_server] Server running at http://0.0.0.0:9090
```

### 4.2 初始化验证清单

每一步都必须通过再继续:

```bash
# 1. 节点全部就位
ros2 node list | grep -E "serial_reader|imu|laser|odom|amcl|bt_navigator|controller_server|planner_server"

# 2. 关键话题有数据
ros2 topic hz /scan         # 雷达,通常 8~10 Hz
ros2 topic hz /odom         # 里程计,30~50 Hz
ros2 topic hz /amcl_pose    # AMCL 输出,1~2 Hz
ros2 topic hz /tf           # 含 map→odom→base_link

# 3. TF 链完整
ros2 run tf2_tools view_frames
# 生成 frames.pdf,检查是否包含:
#   map → odom → base_link → {imu_link, laser_frame}
```

浏览器侧验证:

```bash
# 在测试终端(笔记本)打开
xdg-open http://<jetson-ip>:9090
```

- ✅ 页面加载后左侧"地图"自动选 `map`,Canvas 显示办公室地图
- ✅ 地图上有**绿色圆点 + 方向箭头**代表机器人,2 Hz 刷新
- ✅ 若已有路径点,会显示为蓝色圆 + 白色文字
- ✅ F12 → Network → WS → 消息中出现 `map_data`、`robot_pose`、`waypoints`

### 4.3 选点与路径规划验证(不动机器人)

| # | 操作 | 预期 |
|---|------|------|
| 1 | 双击地图空白处,输入名称 `test-A`,确认 | Canvas 出现蓝色路径点;侧边栏出现 `test-A` |
| 2 | 检查 `waypoints.json` | 多一条记录,坐标为世界单位(米) |
| 3 | 点击侧边栏 `test-A` 的"导航"按钮 | 顶部出现"正在导航到 test-A";机器人开始规划 |
| 4 | 观察终端 C 日志 | 出现 `Sending nav goal: x=..., y=...` |
| 5 | 观察终端 B(Nav2) | `[planner_server]` + `[controller_server]` 日志滚动,无错误 |
| 6 | 浏览器 Console 查 `nav_feedback` 消息 | `distance_remaining` 数值递减 |

### 4.4 实际导航执行

按以下场景依次测试,每个场景记录 §4.5 中的指标:

| 场景 | 描述 | 通过标准 |
|------|------|----------|
| S1 短直线 | 距离 < 1.5 m,无转向 | 到点成功,定位无漂移 |
| S2 转角 | 走廊拐角 | 转向平滑,无碰墙 |
| S3 长路径 | 距离 > 3 m | 全程成功,速度符合上限 |
| S4 障碍绕行 | 临时放置纸箱在路径上 | 局部路径重新规划,绕过 |
| S5 取消 | 导航中途点"取消导航" | 机器人立即停止,状态变 `canceled` |
| S6 重复目标 | 连续两次导航同一目标 | 第二次仍成功 |
| S7 不可达 | 双击点选在墙上或封闭区域 | 收到 `aborted` 或 `rejected`,机器人不动 |

### 4.5 结果记录

每次测试记录到表格(建议建 `test_results/<日期>.csv`):

| 字段 | 示例 |
|------|------|
| `test_id` | 20260602-S1-001 |
| `scenario` | S1 短直线 |
| `start_pose(x,y,yaw)` | -2.64, 4.39, -1.59 |
| `goal_pose(x,y,yaw)` | -1.20, 5.10, 0.0 |
| `euclidean_distance_m` | 1.51 |
| `planning_time_ms` | 浏览器 WS 收到 `nav_status:accepted` 距 `navigate_to` 发送的时间 |
| `execution_time_s` | `nav_result` 到达耗时 |
| `final_xy_error_m` | 终点实际位姿与目标距离 |
| `result` | succeeded / aborted / canceled / rejected |
| `notes` | 任何异常 |

`final_xy_error_m` 可在 Nav2 到点静止后从浏览器 Console 读取:

```javascript
// 假设 ws 是 WebSocket 实例
ws.addEventListener('message', e => {
  const m = JSON.parse(e.data);
  if (m.type === 'robot_pose') console.log('pose', m.x, m.y, m.yaw);
});
```

---

## 5. 故障排除

### 5.1 地图加载失败

| 症状 | 排查 | 解决 |
|------|------|------|
| 终端 C `Failed to load map` | `ls src/robot_bringup/maps/map.{pgm,yaml}` | 文件缺失时从 git 恢复 |
| Nav2 启动报 `Invalid map yaml` | 检查 yaml 字段类型(resolution 是 float 而非 string) | 修正 `map.yaml` |
| 浏览器地图灰屏 | F12 → WS Frames 看是否收到 `map_data` | 没有 → 重启 `web_nav_server`;有 → 检查 `image` 字段 base64 是否完整 |
| `gongxun.map` 误被加载 | 切换地图下拉框回到 `map` | — |
| 路径点位置错位 | `waypoints.json` 坐标与地图不匹配(可能跨地图污染) | 清空 waypoints 或仅保留本地图测试期间创建的点 |

### 5.2 导航指令无响应

| 症状 | 排查 | 解决 |
|------|------|------|
| `NavigateToPose action server not available` | `ros2 action list \| grep navigate_to_pose` | Nav2 未启动,重新跑 `navigation.launch.py` |
| `Goal rejected` | 目标点落在障碍物/costmap 未知区 | 选自由空间(灰色或白色像素)再试 |
| 点击"导航"后无任何日志 | 浏览器 Console 看 `navigate_to` 是否发出 | 检查 ws.readyState === 1 |
| 导航卡在 "正在导航到..." 不结束 | `ros2 topic echo /navigate_to_pose/_events/feedback` | 看 distance_remaining,若长时间不变 → 控制器卡死,重启 Nav2 |
| 机器人原地不动 | `ros2 topic echo /cmd_vel` | 无输出 → 控制器未发速度;有输出 → 串口/下位机问题 |

### 5.3 定位偏差/位姿跳变

| 症状 | 排查 | 解决 |
|------|------|------|
| 地图上绿点位置明显错 | `ros2 topic echo /amcl_pose --once` 与实际比对 | 推送 `/initialpose` 重置初始位姿 |
| AMCL 长时间不收敛 | `view_frames` 确认 TF 链;`/scan` 是否对齐 | 把机器人推到地图上的已知点,RPi 上 `ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped ...` |
| TF 报 `transform_tolerance` 超时 | Orin Nano 负载过高 | `nav2_params.yaml:39` 已设为 1.0,仍报错可调大到 1.5 |
| 绿点完全不动 | `robot_tracker.py` 的 0.5s 定时器未触发 | 检查 `ros2 node list \| grep web_nav` 是否在;`ros2 topic hz /tf` |
| 朝向箭头方向反了 | IMU 标定或安装朝向错误 | 检查 `robot_imu` launch 中 IMU 坐标系约定 |

### 5.4 通信问题

| 症状 | 排查 | 解决 |
|------|------|------|
| 浏览器打不开 9090 | Jetson 上 `curl http://localhost:9090/` | 不通 → web_nav_server 未起;通 → 防火墙/网段 |
| WebSocket 连上后立刻断开 | 终端 C 看是否有 `WebSocket client connected` 后立即 disconnect | 通常是 aiohttp 异常,看 stderr |
| 重启 Web 后浏览器不重连 | 等待 3 秒;F12 Console 看重连日志 | 客户端 3 秒自动重连,如未生效刷新页面 |

---

## 6. 测试指标

### 6.1 关键性能指标(KPI)

| 指标 | 单位 | 通过阈值 | 测量方法 |
|------|------|----------|----------|
| **定位精度** `pose_error_m` | 米 | ≤ 0.20 | 测试结束静态位姿与目标点欧氏距离 |
| **导航成功率** `success_rate` | % | ≥ 90% | `succeeded / (succeeded+aborted+canceled)` × 100,至少 10 次 |
| **响应时间** `cmd_response_ms` | ms | ≤ 500 | 浏览器发送 `navigate_to` 到收到 `nav_status:accepted` 的时间戳差 |
| **路径规划耗时** `planning_ms` | ms | ≤ 2000 | Nav2 `planner_server` 日志或 BT 反馈 |
| **执行时间** `exec_s` | 秒 | ≤ `distance / 0.15` | `nav_result` 减 `nav_status:accepted` |
| **距离剩余反馈频率** `feedback_hz` | Hz | ≥ 2 | 单位时间内 `nav_feedback` 消息计数 |
| **机器人位姿推送频率** `pose_hz` | Hz | ≥ 1.5(标称 2) | 单位时间内 `robot_pose` 消息计数 |
| **CPU 占用** `cpu_pct` | % | < 60(单核) | `top -p $(pgrep -f web_nav_server\|bt_navigator)` |
| **内存占用** `mem_mb` | MB | < 500 | 同上 |
| **断线重连时间** `reconnect_s` | 秒 | ≤ 5 | 拔网线 → 测算到 `robot_pose` 恢复的时间 |

### 6.2 测量脚本(辅助)

启动测试时在测试终端运行,自动收集 WS 端的指标:

```bash
# 保存为 scripts/measure.py,运行 python3 scripts/measure.py
import asyncio, websockets, json, time, statistics, sys

URL = f"ws://{sys.argv[1] if len(sys.argv)>1 else 'localhost'}:9090/ws"
POSE_TS, FB_TS = [], []
NAV_SEND_TS, NAV_ACCEPT_TS = None, None
RESULTS = []

async def main():
    global NAV_SEND_TS, NAV_ACCEPT_TS
    async with websockets.connect(URL) as ws:
        print("connected")
        start = time.time()
        async for raw in ws:
            m = json.loads(raw)
            t = time.time()
            if m["type"] == "robot_pose":
                POSE_TS.append(t)
            if m["type"] == "nav_feedback":
                FB_TS.append(t)
            if m["type"] == "nav_status" and m.get("status") == "accepted":
                NAV_ACCEPT_TS = t
            if m["type"] == "nav_result":
                RESULTS.append(m["status"])
                # 周期统计
                pose_hz = len(POSE_TS) / max(1, t - start)
                fb_hz   = len(FB_TS)   / max(1, t - start)
                resp_ms = ((NAV_ACCEPT_TS - NAV_SEND_TS) * 1000) if (NAV_ACCEPT_TS and NAV_SEND_TS) else None
                print(f"[result] {m['status']} | pose_hz={pose_hz:.2f} fb_hz={fb_hz:.2f} resp_ms={resp_ms}")
                NAV_SEND_TS = NAV_ACCEPT_TS = None
asyncio.run(main())
```

> 浏览器侧响应时间也可在 DevTools Console 用 `performance.now()` 包住 `ws.send` 到下次 `nav_status` 的时间差测量。

### 6.3 报告模板

测试结束生成 Markdown 报告,建议字段:

```
测试日期: 2026-MM-DD   测试人员: ___   地图: map (办公室)
硬件版本: Jetson Orin Nano + RPLidar ___ + MCU ___
软件版本: ROS2 Humble + Nav2 ___ + robot_ws commit ___

| 场景 | 次数 | 成功 | 成功率 | 平均响应ms | 平均执行s | 最大pose_err m |
|------|------|------|--------|-----------|----------|----------------|
| S1   |  5   |  5   | 100%   |   187     |  10.2    |   0.12         |
| ...  |      |      |        |           |          |                |

异常记录:
- 2026-MM-DD HH:MM S4 第一次因纸箱过近 aborted,移远后通过

结论: [通过 / 通过(条件性) / 不通过]
```

---

## 附录 A:常用命令速查

```bash
# 查看当前位姿
ros2 topic echo /amcl_pose --once

# 查看当前地图
ros2 service call /map_server/get_map nav_msgs/srv/GetMap

# 强制重置 AMCL 位姿(改 x,y,yaw 后回车)
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -2.64, y: 4.39, z: 0}, orientation: {z: -0.7157, w: 0.6984}}}}"

# 取消正在进行的导航(命令行)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{}" --feedback

# 录制测试用 bag(包含定位/雷达/控制)
ros2 bag record -o test_bag /scan /tf /tf_static /odom /amcl_pose /cmd_vel

# 实时观察 Nav2 行为树状态
ros2 topic echo /behavior_tree_log | head
```

## 附录 B:文档版本

| 版本 | 日期 | 作者 | 变更 |
|------|------|------|------|
| v1.0 | 2026-06-02 | — | 初版,基于办公室小地图与 robot_web_nav Web UI |
