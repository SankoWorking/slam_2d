# ROS2 差速驱动移动机器人导航系统

面向 NVIDIA Jetson Orin Nano（ARM64、6 核、7.4 GB RAM）的 ROS2 Humble 差速驱动移动机器人工作空间。集成硬件驱动、SLAM/AMCL 定位、Nav2 自主导航以及 Web 远程控制界面。

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    浏览器 Web UI (:9090)                   │
│            点击导航 / 手动遥控 / 定位 / 日志查看            │
└──────────────────────────┬──────────────────────────────┘
                           │ WebSocket
┌──────────────────────────▼──────────────────────────────┐
│              robot_web_nav (aiohttp + rclpy)             │
│  web_server / map_service / nav_action_client            │
│  manual_control / localization_manager / robot_tracker   │
└──────────────────────────┬──────────────────────────────┘
                           │ ROS2 Actions / Topics
┌──────────────────────────▼──────────────────────────────┐
│                    Nav2 导航栈                             │
│     AMCL 定位 · SmacPlanner2D(A*) · MPPI Omni 控制器      │
└──────────────────────────┬──────────────────────────────┘
                           │ TF / Topics
┌──────────────────────────▼──────────────────────────────┐
│                  硬件驱动层                                │
│   串口通信(MCU) · IMU 姿态融合 · RPLidar 雷达 · 轮式里程计  │
└─────────────────────────────────────────────────────────┘
```

TF 链: `map →(AMCL)→ odom →(robot_odometry)→ base_link →{imu_link, laser_frame}`

## 功能特性

- **Web 远程控制** — 浏览器端点击导航、D-pad 手动遥控、键盘方向键支持
- **AMCL 自动定位** — 全局定位粒子撒播 + 协方差收敛监控
- **初始位姿设置** — 地图上点击定位 + 拖拽设定朝向
- **路径可视化** — Nav2 规划路径实时显示在地图上
- **路径点管理** — 双击地图添加、命名、一键导航、删除
- **地图管理** — 多地图切换、重命名、复制、显示名称
- **日志系统** — 后端日志实时推送到前端，可折叠查看
- **断线重连** — WebSocket 自动重连（指数退避）+ 导航状态恢复
- **移动端适配** — 响应式布局，触屏 D-pad + 双指缩放

## 工作空间结构

```
robot_ws/
├── src/
│   ├── robot_bringup/        # 顶层 launch 组合、地图、Nav2/SLAM 配置
│   ├── robot_serial/         # 串口读取节点（C++，libserial）
│   ├── robot_imu/            # IMU launch（imu_filter_madgwick）
│   ├── robot_lidar/          # RPLidar 雷达封装（sllidar_ros2）
│   ├── robot_odometry/       # 编码器+IMU 里程计（C++）
│   ├── robot_web_nav/        # Web 导航服务器（Python，aiohttp）
│   └── third_party/sllidar_ros2/  # SLAMTEC 雷达驱动（只读）
├── CLAUDE.md                 # AI 辅助开发文档
└── README.md
```

## 环境要求

- **硬件**: NVIDIA Jetson Orin Nano（ARM64）
- **系统**: Ubuntu 22.04 + JetPack
- **ROS**: ROS2 Humble Hawksbill
- **依赖**: `libserial-dev`, `python3-aiohttp`, `python3-pil`, `python3-yaml`

### ROS2 依赖包

```bash
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-imu-filter-madgwick ros-humble-laser-filters \
  ros-humble-tf2-ros ros-humble-rviz2
```

## 构建与运行

### 构建

```bash
cd robot_ws
colcon build --symlink-install
source install/setup.bash

# 单包构建
colcon build --symlink-install --packages-select robot_web_nav
```

### 一键启动（推荐）

```bash
# 启动全部（硬件 + 导航 + Web UI）
ros2 launch robot_bringup full_system.launch.py

# 指定地图
ros2 launch robot_bringup full_system.launch.py map:=gongxun.map

# 同时打开 RViz
ros2 launch robot_bringup full_system.launch.py rviz:=true

# 修改 Web 端口
ros2 launch robot_bringup full_system.launch.py web_port:=8080

# 只启动部分组件
ros2 launch robot_bringup full_system.launch.py hardware:=false navigation:=true
```

可用参数:

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `hardware` | `true` | 启动硬件驱动（串口/IMU/雷达/里程计） |
| `navigation` | `true` | 启动 Nav2 导航栈 |
| `web_nav` | `true` | 启动 Web 界面 |
| `rviz` | `false` | 启动 RViz 可视化 |
| `map` | `map` | 地图名称（不含 .yaml 后缀） |
| `web_port` | `9090` | Web 服务器端口 |

### 分步启动（调试用）

```bash
# 终端 1: 硬件驱动
ros2 launch robot_bringup bringup.launch.py

# 终端 2: Nav2 导航栈
ros2 launch robot_bringup navigation.launch.py map:=map

# 终端 3: Web 界面
ros2 launch robot_web_nav web_nav.launch.py port:=9090
```

### SLAM 建图

```bash
# 方式一: SLAM Toolbox（推荐）
ros2 launch robot_bringup slam.launch.py

# 方式二: Google Cartographer
ros2 launch robot_bringup cartographer.launch.py
```

### USB 设备配置（首次使用）

```bash
# 安装 udev 规则，创建稳定的串口符号链接
sudo cp src/robot_bringup/config/99-robot-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# 重新插拔 USB 设备后检查
ls -la /dev/sllidar /dev/chassis
```

- `/dev/sllidar` → RPLidar 雷达
- `/dev/chassis` → 下位机 MCU

## Web 界面使用

启动后浏览器访问 `http://<Jetson-IP>:9090`。

### 操作说明

| 功能 | 操作 |
|------|------|
| 导航到路径点 | 双击地图 → 输入名称 → 导航按钮 |
| 导航到坐标 | 双击地图 → 确认后点击路径点的"导航" |
| 取消导航 | 导航状态栏的"取消导航"按钮 |
| 手动遥控 | "申请控制权" → D-pad 或键盘方向键 |
| 自动定位 | "自动定位" → 遥控机器人走 1~2m 并旋转 360° |
| 设置初始位姿 | "设置初始位姿" → 地图上点击定位 → 拖拽设定朝向 |
| 切换地图 | 左侧地图下拉选择 |
| 查看日志 | 底部"日志"面板展开 |

### 键盘快捷键（需先申请控制权）

- `↑↓←→` — 前进/后退/左转/右转
- `空格` — 紧急停止

## 地图

地图文件存放在 `src/robot_bringup/maps/`:

| 地图 | 文件 | 分辨率 | 原点 | 说明 |
|------|------|--------|------|------|
| 办公室 | `map.yaml` + `map.pgm` | 0.05 m/px | [-5.62, -2.98] | 209×330 |
| 公训楼 | `gongxun.map.yaml` + `gongxun.map.pgm` | 0.05 m/px | [-10.1, -32.0] | 1437×931 |

## 测试

```bash
colcon test --packages-select robot_web_nav
colcon test-result --verbose --all

# 单个 pytest
pytest src/robot_web_nav/test/<file>::<test>
```

## Nav2 配置要点

配置文件: `src/robot_bringup/config/nav2_params.yaml`

- 全局坐标系: `map`，机器人坐标系: `base_link`
- 规划器: SmacPlanner2D（A*），最大规划时间 2s
- 控制器: MPPI Omni，20Hz，batch=800
- 速度限制: vx=0.3 m/s, vy=0.2 m/s, wz=1.2 rad/s
- 到达容差: xy=0.15 m
- 参数已针对 Orin Nano 的 CPU/内存规格调低，勿使用 Nav2 默认值

## 技术栈

| 层 | 技术 |
|----|------|
| 导航 | ROS2 Humble + Nav2 |
| SLAM | SLAM Toolbox / Google Cartographer |
| 定位 | AMCL（Omni 运动模型） |
| Web 后端 | Python 3 + aiohttp + rclpy |
| Web 前端 | Vanilla JS + Canvas + CSS |
| 硬件通信 | libserial (C++) + USB |
| 雷达 | SLAMTEC RPLidar C1 |
| IMU | imu_filter_madgwick |
| 目标平台 | NVIDIA Jetson Orin Nano (ARM64) |

## License

MIT
