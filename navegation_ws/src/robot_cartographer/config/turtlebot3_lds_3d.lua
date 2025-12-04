-- /* map → odom */
include "map_builder.lua"
include "trajectory_builder.lua"

MAP_BUILDER.use_trajectory_builder_3d = true

-- Ajustes 3D
TRAJECTORY_BUILDER_3D.min_range = 0.30
TRAJECTORY_BUILDER_3D.max_range = 20.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 2
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.07
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false


options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "imu_link",      
  published_frame = "base_footprint",   
  odom_frame = "odom",

  provide_odom_frame = false,       
  publish_frame_projected_to_2d = false,  -- << en 3D 

  use_odometry = true,            
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,             -- lidar  3D en /points2

  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

return options
