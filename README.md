# ROS2 差速驱动移动机器人导航与建图系统

这是一个面向 NVIDIA Jetson Orin Nano 的 ROS2 Humble 移动机器人工作空间，适用于差速驱动底盘。项目集成了串口下位机通信、IMU、RPLidar、轮式里程计、SLAM 建图、AMCL + Nav2 导航，以及一个可在浏览器和手机端使用的 Web 控制台。

Web 控制台现在拆分为三个功能页：导航、控制、建图。建图页支持从网页启动 SLAM Toolbox 或 Cartographer，实时查看 `/map` 中已经探明的区域，并保存地图文件。

## 功能特性

- Web 三页式界面：导航、控制、建图，适配手机端触屏操作
- 点击导航：在地图上选择目标点或路径点并发送 Nav2 导航任务
- 路径点管理：新增、删除、命名、导航到路径点
- 手动遥控：独占控制权、D-pad、键盘方向键、急停、心跳超时释放
- AMCL 定位：全局重定位、初始位姿设置、协方差收敛状态显示
- Web 建图：启动/停止 SLAM Toolbox 或 Cartographer，保存当前 `/map`
- 实时建图预览：建图运行时把 `/map` OccupancyGrid 转为图片，通过 WebSocket 约 1 Hz 推送到前端
- 地图管理：地图切换、复制、重命名、显示名称管理
- 雷达遮罩：硬件 bringup 默认包含 `laserfilter.launch.py`，原始 `/scan_raw` 过滤后输出 `/scan`
- 断线重连：WebSocket 自动重连，并恢复地图、机器人、导航、控制和建图状态

## 系统架构

```text
Browser Web UI (:9090)
  - 导航页: 地图、路径点、点击导航、定位
  - 控制页: 手动遥控、控制权、速度状态
  - 建图页: SLAM 启停、实时地图预览、地图保存
        |
        | HTTP + WebSocket
        v
robot_web_nav (aiohttp + rclpy)
  - web_server.py
  - map_service.py
  - nav_action_client.py
  - mapping_manager.py
  - manual_control.py
  - localization_manager.py
  - robot_tracker.py
        |
        | ROS2 actions / topics / services
        v
Nav2 / AMCL / SLAM Toolbox / Cartographer
        |
        | TF / /scan / /odom / /cmd_vel / /map
        v
硬件驱动层: MCU 串口、IMU、RPLidar、轮式里程计
```

预期 TF 链：

```text
map -> odom -> base_link -> {imu_link, laser_frame}
```

导航模式下 `map -> odom` 由 AMCL 发布。建图模式下 `map -> odom` 由 SLAM 发布。因此不要同时运行 Nav2/AMCL 和 SLAM 建图，避免 TF 冲突。

## 工作空间结构

```text
src/
  robot_bringup/       顶层 launch、地图、Nav2/SLAM/雷达遮罩配置
  robot_serial/        串口读取节点，读取下位机编码器和 IMU 数据帧
  robot_imu/           IMU launch，使用 imu_filter_madgwick
  robot_lidar/         RPLidar 驱动封装
  robot_odometry/      编码器 + IMU 里程计，发布 /odom 和 odom -> base_link
  robot_web_nav/       Web 导航、遥控、建图服务
  third_party/         第三方雷达驱动，通常按只读处理
```

## 环境要求

- 硬件：NVIDIA Jetson Orin Nano
- 系统：Ubuntu 22.04 + JetPack
- ROS：ROS2 Humble
- Python：Python 3

常用依赖：

```bash
sudo apt install \
  libserial-dev \
  python3-aiohttp python3-pil python3-yaml \
  ros-humble-nav2-bringup ros-humble-nav2-map-server \
  ros-humble-slam-toolbox ros-humble-cartographer-ros \
  ros-humble-imu-filter-madgwick ros-humble-laser-filters \
  ros-humble-tf2-ros ros-humble-rviz2
```

## 构建

在工作空间根目录执行：

```bash
colcon build --symlink-install
source install/setup.bash
```

只构建 Web 包：

```bash
colcon build --symlink-install --packages-select robot_web_nav
source install/setup.bash
```

## 运行方式

### 导航模式

一键启动硬件、Nav2 和 Web：

```bash
ros2 launch robot_bringup full_system.launch.py
```

指定地图：

```bash
ros2 launch robot_bringup full_system.launch.py map:=gongxun.map
```

打开 RViz：

```bash
ros2 launch robot_bringup full_system.launch.py rviz:=true
```

修改 Web 端口：

```bash
ros2 launch robot_bringup full_system.launch.py web_port:=8080
```

分步启动：

```bash
# 终端 1: 硬件驱动，包含串口、IMU、雷达、里程计、雷达遮罩
ros2 launch robot_bringup bringup.launch.py

# 终端 2: Nav2 导航栈，加载已有地图
ros2 launch robot_bringup navigation.launch.py map:=map

# 终端 3: Web 控制台
ros2 launch robot_web_nav web_nav.launch.py port:=9090
```

### Web 建图模式

建图时推荐只启动硬件和 Web，不启动 Nav2/AMCL：

```bash
# 终端 1: 硬件驱动，包含雷达遮罩
ros2 launch robot_bringup bringup.launch.py

# 终端 2: Web 控制台
ros2 launch robot_web_nav web_nav.launch.py port:=9090
```

然后访问：

```text
http://<Jetson-IP>:9090
```

进入 Web 的“建图”页，选择 `SLAM Toolbox` 或 `Cartographer`，点击开始建图。建图运行时前端会实时显示 `/map` 中已探明区域，并显示已知区域比例和障碍物栅格数量。

也可以使用完整 launch，但关闭导航：

```bash
ros2 launch robot_bringup full_system.launch.py navigation:=false
```

注意：`file://.../web/index.html` 只能看到静态页面，不能连接 ROS 和 WebSocket。实际使用必须通过 `http://<Jetson-IP>:9090` 打开。

### 命令行建图

Web 建图之外，也可以直接使用命令行：

```bash
# SLAM Toolbox，推荐
ros2 launch robot_bringup slam.launch.py

# Cartographer
ros2 launch robot_bringup cartographer.launch.py
```

保存地图可以使用：

```bash
ros2 run nav2_map_server map_saver_cli -f src/robot_bringup/maps/<map_name>
```

## Web 控制台

| 页面 | 功能 |
| --- | --- |
| 导航 | 地图查看、机器人位姿、点击导航、路径点、地图切换、AMCL 定位 |
| 控制 | 申请/释放控制权、D-pad 遥控、速度状态、急停 |
| 建图 | 启动/停止 SLAM、实时地图预览、保存地图、填写显示名称 |

常用操作：

| 功能 | 操作 |
| --- | --- |
| 导航到目标点 | 在导航页点击地图目标点，确认后发送导航 |
| 添加路径点 | 在导航页双击地图，输入路径点名称 |
| 导航到路径点 | 在路径点列表中点击导航 |
| 取消导航 | 点击导航状态栏中的取消导航 |
| 手动遥控 | 在控制页申请控制权后使用 D-pad 或方向键 |
| 急停 | 控制页点击停止，或键盘空格键 |
| 全局重定位 | 导航页点击自动定位，然后遥控机器人移动和旋转 |
| 开始建图 | 建图页选择后端，点击开始建图 |
| 保存地图 | 建图页输入英文文件名和可选中文显示名，点击保存 |

## Web 建图细节

后端新增 `mapping_manager.py`，负责启动和停止 SLAM 进程，并调用 `nav2_map_server/map_saver_cli` 保存地图。

建图相关 WebSocket 消息：

| 方向 | 消息类型 | 说明 |
| --- | --- | --- |
| 前端到后端 | `start_mapping` | 启动 `slam_toolbox` 或 `cartographer` |
| 前端到后端 | `stop_mapping` | 停止当前建图进程 |
| 前端到后端 | `save_mapping` | 保存当前 `/map` 到地图目录 |
| 前端到后端 | `get_mapping_status` | 查询建图状态 |
| 后端到前端 | `mapping_status` | 当前建图状态 |
| 后端到前端 | `live_map_data` | 实时地图预览图片和统计信息 |
| 后端到前端 | `live_map_cleared` | 清空实时预览 |
| 后端到前端 | `map_saved` | 地图保存完成 |

实时预览说明：

- 数据来源是 `/map` 的 `nav_msgs/OccupancyGrid`
- 后端将占据栅格转为 PNG 图片，再用 base64 通过 WebSocket 推送
- 推送频率约 1 Hz，避免 Web 页面和 Jetson 负载过高
- 只在 Web 启动的建图任务运行时推送
- 建图停止后会清空实时预览，并恢复普通地图显示

保存规则：

- 地图文件保存在 `src/robot_bringup/maps/`
- 保存后生成 `<map_name>.yaml` 和 `<map_name>.pgm`
- 文件名只允许英文字母、数字、点、短横线和下划线，例如 `office_20260604`
- 显示名称可以使用中文，会写入 `maps_meta.json`
- 如果同名地图已存在，必须勾选覆盖才会保存

## 地图与路径点

地图目录：

```text
src/robot_bringup/maps/
```

仓库内已有地图：

| 地图 | 文件 | 分辨率 | 说明 |
| --- | --- | --- | --- |
| 办公室 | `map.yaml` + `map.pgm` | 0.05 m/px | 默认地图 |
| 公训楼 | `gongxun.map.yaml` + `gongxun.map.pgm` | 0.05 m/px | 大地图 |

路径点文件：

```text
src/robot_bringup/maps/waypoints.json
```

当前所有地图共享同一份路径点文件。切换地图不会自动切换路径点集合，如果不同地图坐标系不一致，路径点位置会错位。

## USB 与雷达遮罩

安装 udev 规则后会创建稳定设备名：

```bash
sudo cp src/robot_bringup/config/99-robot-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔设备后检查：

```bash
ls -la /dev/sllidar /dev/chassis
```

- `/dev/sllidar`：RPLidar
- `/dev/chassis`：下位机 MCU

`robot_bringup/launch/bringup.launch.py` 会包含 `laserfilter.launch.py`，雷达过滤链使用 `config/laser_filter_params.yaml`。预期话题流为：

```text
RPLidar -> /scan_raw -> laser_filters -> /scan
```

如果现场发现遮罩没有生效，优先检查：

- `ros2 topic echo /scan_raw` 和 `ros2 topic echo /scan` 是否都有数据
- `laser_filter_params.yaml` 中的遮罩盒参数是否覆盖实际遮挡区域
- laser filter 节点名是否与参数文件顶层键一致

## 测试与静态检查

`robot_web_nav` 带有 Python 测试脚手架：

```bash
colcon test --packages-select robot_web_nav
colcon test-result --verbose --all
```

只运行单个 pytest：

```bash
pytest src/robot_web_nav/test/<file>::<test>
```

前端静态检查：

```bash
node --check src/robot_web_nav/web/js/app.js
node --check src/robot_web_nav/web/js/map.js
node --check src/robot_web_nav/web/js/mapping.js
node --check src/robot_web_nav/web/js/control.js
node --check src/robot_web_nav/web/js/localize.js
```

后端 Python 语法检查：

```bash
python -m py_compile \
  src/robot_web_nav/robot_web_nav/web_server.py \
  src/robot_web_nav/robot_web_nav/mapping_manager.py \
  src/robot_web_nav/robot_web_nav/map_service.py
```

## Nav2 配置要点

Nav2 参数文件：

```text
src/robot_bringup/config/nav2_params.yaml
```

关键约定：

- 全局坐标系：`map`
- 机器人坐标系：`base_link`
- 规划器：SmacPlanner2D
- 控制器：MPPI
- 最大线速度：`vx=0.3 m/s`
- 最大角速度：`wz=1.2 rad/s`
- 到达容差：`xy=0.15 m`

参数已经按 Jetson Orin Nano 的 CPU 和内存规格做过收敛，不建议直接换成 Nav2 默认重载参数。

## 技术栈

| 层级 | 技术 |
| --- | --- |
| 导航 | ROS2 Humble + Nav2 |
| 建图 | SLAM Toolbox / Cartographer |
| 定位 | AMCL |
| 后端 | Python 3 + aiohttp + rclpy |
| 前端 | Vanilla JavaScript + Canvas + CSS |
| 地图保存 | nav2_map_server map_saver_cli |
| 硬件通信 | C++ + libserial |
| 雷达 | SLAMTEC RPLidar |
| IMU | imu_filter_madgwick |
| 目标平台 | NVIDIA Jetson Orin Nano |

## License

MIT
