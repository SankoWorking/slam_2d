# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## 仓库概览

这是一个面向 NVIDIA Jetson Orin Nano（ARM64、6 核、7.4 GB RAM）的差速驱动移动机器人 ROS2 Humble 工作空间。集合了硬件驱动、SLAM/AMCL+Nav2 导航，以及一个点击导航 + 手动遥控的 Web 界面。

`src/` 下的包：

- `robot_bringup`（ament_cmake）—— 顶层 launch 组合、地图、Nav2/SLAM 配置、RViz。通过 `bringup.launch.py` 拉起下面四个驱动包 + 激光遮罩。
- `robot_serial`(ament_cmake, C++) —— 串口读取节点。使用 `libserial` 与下位机通信,发布被 `robot_odometry` 消费的编码器/IMU 数据帧。
- `robot_imu`（ament_cmake）—— IMU launch;依赖 `imu_filter_madgwick` 做姿态融合。
- `robot_lidar`（ament_cmake）—— 封装 `third_party/sllidar_ros2` 驱动 RPLidar 雷达,发布 `laser_frame`。
- `robot_odometry`（ament_cmake, C++, `src/odom_node.cpp`) —— 融合串口编码器与 IMU,输出 `/odom` 和 `odom → base_link` TF。
- `robot_web_nav`（ament_python）—— aiohttp HTTP+WebSocket 服务器,在浏览器与 Nav2/tf2 之间做桥接。包含导航、手动遥控、AMCL 重定位、地图管理等功能。仓库中唯一的 Python 包,其余均为 C++。
- `third_party/sllidar_ros2` —— 引入的 SLTeC/Slamtec 雷达驱动,部分检出版本中 `src/third_party/` 被 gitignore;按只读对待。

运行时预期的完整 TF 链:`map →(AMCL)→ odom →(robot_odometry)→ base_link →{imu_link, laser_frame(静态)}`。

## 构建与运行

```bash
# 在工作空间根目录
colcon build --symlink-install
source install/setup.bash

# 单包构建(典型迭代循环)
colcon build --symlink-install --packages-select robot_web_nav

# 整机硬件拉起(串口 + IMU + 雷达 + 里程计 + 激光遮罩)
ros2 launch robot_bringup bringup.launch.py

# Nav2 导航栈(需已有建好的地图;加载 maps/map.yaml + config/nav2_params.yaml)
ros2 launch robot_bringup navigation.launch.py

# 建图(SLAM)—— 用 cartographer.launch.py 或 slam.launch.py
ros2 launch robot_bringup slam.launch.py        # slam_toolbox (使用 config/slam_params.yaml)
ros2 launch robot_bringup cartographer.launch.py # 使用 config/slam.lua

# RViz 可视化(SLAM 用)
ros2 launch robot_bringup rviz.launch.py

# Web 界面(依赖 navigation.launch.py 已运行)
ros2 run robot_web_nav web_nav_server           # http://<jetson-ip>:9090
ros2 launch robot_web_nav web_nav.launch.py     # 同上,通过 launch 启动
WEB_NAV_PORT=8080 ros2 run robot_web_nav web_nav_server   # 端口覆盖
```

`build/`、`install/`、`log/`、`src/third_party/` 均被 gitignore —— 雷达驱动是单独获取/构建的,不在仓库内。

## 测试

`robot_web_nav` 是唯一带 Python 测试脚手架的包(ament_copyright/flake8/pep257 + pytest):

```bash
colcon test --packages-select robot_web_nav
colcon test-result --verbose --all
# 跑单个 pytest
pytest src/robot_web_nav/test/<file>::<test>
```

C++ 包没有接入单元测试。

## 编辑前值得了解的架构要点

### robot_web_nav 的模块结构

`web_server.py` 在**主线程跑 aiohttp**、在**守护线程跑 rclpy**(`MultiThreadedExecutor`)。跨线程投递通过模块级全局变量 `_main_loop`(主线程的 asyncio 事件循环)和 `asyncio.run_coroutine_threadsafe(...)` 完成 —— 任何需要把数据推给 WebSocket 客户端的 ROS 回调,都必须走这条桥。新增 ROS 驱动的广播或 async/await 行为时,请遵循 `web_server.py` 的现有模式,不要引入新的线程原语。

后端模块:
- `web_server.py` —— HTTP/WebSocket 主入口,路由分发
- `map_service.py` —— 地图加载、pixel↔world 转换、地图重命名/复制、显示名称管理
- `nav_action_client.py` —— Nav2 `/navigate_to_pose` 动作客户端
- `waypoint_manager.py` —— 路径点 CRUD,持久化到 `waypoints.json`(首次写入时创建)
- `robot_tracker.py` —— 订阅 `/amcl_pose`(`PoseWithCovarianceStamped`),提取位姿和协方差
- `localization_manager.py` —— AMCL 全局重定位(` /reinitialize_global_localization`),协方差收敛监控(cov_xy < 0.05, cov_yaw < 0.05 时自动清除 localizing 状态)
- `manual_control.py` —— 单客户端独占手动遥控,通过 `/cmd_vel` 以 10 Hz 发布,速度钳位(max vx=0.3, wz=1.0),2 秒心跳超时自动释放

前端模块(`web/`):
- `js/app.js` —— WebSocket 连接管理、消息路由
- `js/map.js` —— 地图渲染、点击导航、world↔screen 坐标转换
- `js/robot.js` —— 机器人位姿可视化
- `js/control.js` —— D-pad 手动遥控 UI,200ms 速度发送,键盘方向键支持,释放控制
- `js/localize.js` —— AMCL 全局重定位按钮,协方差显示,收敛状态徽章

WebSocket 消息类型包括:导航(`navigate_to_pose`/`navigate_to_xy`/`cancel_nav`)、地图(`load_map`/`rename_map`/`duplicate_map`/`set_map_display_name`)、路径点(`add_waypoint`/`delete_waypoint`/`navigate_waypoint`)、手动控制(`claim_control`/`release_control`/`set_velocity`)、定位(`start_localization`/`localization_status`)、状态(`get_robot_pose`/`robot_pose_full`/`nav_status`/`control_status`)。

### 坐标系约定
本系统中存在三套坐标系,转换代码分散在两门语言中:
- ROS 世界坐标系:米,+x 向右,+y 向上
- PGM 像素:y 向下,原点在左上
- 浏览器 Canvas:y 向下 + 平移/缩放偏移

服务端 `pixel ↔ world` 在 `robot_web_nav/map_service.py`(Y 轴翻转用 `height - py`)。浏览器端 `world ↔ screen` 在 `web/js/map.js`。改动地图加载或渲染时,**两侧都要同步更新**,否则路径点和点击位置会漂移。现有地图:`map`(209×330,origin [-5.62, -2.98])和 `gongxun.map`(1437×931,origin [-10.1, -32.0]),均为 0.05 m/px。

### 路径点存储
所有地图当前共享同一份文件 `src/robot_bringup/maps/waypoints.json`(格式 `{"waypoints": [{name, x, y, yaw}]}`)。该文件在首次写入时自动创建,不存在时以空列表初始化。切换地图**不会**换路径点集合 —— 坐标是全局的,在另一张地图上会错位。`WaypointManager` 使用 `threading.Lock` 保护;同名添加会覆盖,不会重复。

### Nav2 集成约束
`nav_action_client.py` 调用 `/navigate_to_pose`,并把状态码 4/5/6 映射为 succeeded/aborted/canceled。Yaw 到四元数的转换在文件内直接实现(不依赖 `tf_transformations`)。Nav2 配置在 `src/robot_bringup/config/nav2_params.yaml`:全局坐标系 `map`、机器人坐标系 `base_link`、SmacPlanner2D(A*)、MPPI Omni 控制器,最大 vx=0.3 m/s、vy=0.2 m/s、wz=1.2 rad/s,xy 容差 0.15 m。把这些当作动作客户端默认假设的契约。

### 地图目录布局
`src/robot_bringup/maps/` 存放 `map.{pgm,yaml}`(办公室)、`gongxun.map.{pgm,yaml}` 以及共享的 `waypoints.json`。`MapService` 还维护 `maps_meta.json` 存储地图显示名称。注意:`gongxun.map.yaml` 的 `image:` 字段历史上末尾有多余空格,`MapService` 已用 `.strip()` 处理 —— 不要在不去除 strip 的情况下"修复"这个 YAML。

### USB 设备与 udev 规则
`src/robot_bringup/config/99-robot-usb.rules` 为 USB 串口设备创建稳定符号链接:
- `/dev/sllidar` → RPLidar (Silicon Labs CP210x)
- `/dev/chassis` → 下位机 MCU (QinHeng CH9102)

安装: `sudo cp 99-robot-usb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`。

### 激光遮罩
`bringup.launch.py` 会自动拉起 `laserfilter.launch.py`(使用 `config/laser_filter_params.yaml`),对雷达遮挡区域做遮罩处理。`robot_lidar` 的 launch 文件中串口路径可能指向 `/dev/ttyUSB1`(硬编码),udev 规则安装后应改用 `/dev/sllidar`。

### SLAM 配置
`slam.launch.py` 使用 `config/slam_params.yaml`(slam_toolbox 参数,针对 RPLidar C1 + Orin Nano 调优)。`cartographer.launch.py` 使用 `config/slam.lua`。

### 目标硬件的 quirks
运行环境为 Jetson Orin Nano + JetPack。Nav2 参数已为这套 CPU/内存规格调低(见提交 `db159e9` —— "调整后的参数文件适配 orin nano");使用更激进的 Nav2 默认值会导致资源超载。`laser_filter_params.yaml` 的存在是因为雷达加装后有遮挡,需要遮罩处理(见提交 `73c8ed0`)。

## 延伸阅读

`src/robot_web_nav/PROJECT_DOC.md` 是 web/nav 包的权威深度文档 —— 包含 WebSocket 协议表、消息类型、断点建议和性能估算。**注意**:该文档部分内容滞后于代码(缺少 `localization_manager.py`/`manual_control.py`/`control.js`/`localize.js` 的描述,`robot_tracker.py` 的实现描述已过时)。对 `robot_web_nav` 做非平凡修改前先读它,但以实际代码为准。
