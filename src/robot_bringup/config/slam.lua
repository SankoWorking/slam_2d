include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",

  odom_frame = "odom",
  -- 是否由cartographer发布map->odom的变换。true为开启。
  provide_odom_frame = true,
  -- false->true 启用里程计
  use_odometry = true,

  -- 剔除Roll、Pitch和z轴高度，认为机器人是在2D平面上运动。ture为开启投影。
  publish_frame_projected_to_2d = true,
  -- 是否启用位姿外推器。因为雷达的频率较低，在两次雷达扫描间，机器人仍然在移动，使用位姿外推器会利用IMU和里程计推算机器人打精确位置。
  use_pose_extrapolator = true,
  -- 是否启用GPS信号。开启后会订阅sensor_msgs/NavSatFix，会利用GPS信号修正机器人的位置。
  use_nav_sat = false,
  -- 是否启用地标。开启后会订阅cartographer_ros_msgs/LandmarkList话题，当摄像头看到（二维码、反光柱）等地标时，算法会瞬间把机器人拉回对应精确坐标。
  use_landmarks = false,

  -- 接收2D激光雷达话题的数量
  num_laser_scans = 1,
  -- 接收多回波激光雷达的数量
  num_multi_echo_laser_scans = 0,
  -- 10->1将完整的一帧激光扫描拆分成多少子块处理。当雷达扫描频率很慢而机器人移动很快时，一帧扫描开始和结束时机器人的位置已经变了，所以需要对雷达扫描进行切分
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
-- 10->1 原来是攒够10帧雷达才进行一次匹配计算，现在改为1,使得每一帧都立即参与匹配。
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

return options
