/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/far_planner.h"
#include <octomap_msgs/Octomap.h>
#include <ros/package.h>
#include <XmlRpcValue.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

namespace {

bool ParseSemanticGroups(const ros::NodeHandle& private_nh,
                         const std::string& parameter,
                         std::vector<SemanticClassGroup>& groups) {
  XmlRpc::XmlRpcValue value;
  if (!private_nh.getParam(parameter, value)) {
    ROS_ERROR("FARMaster: required semantic class parameter ~%s is missing.",
              parameter.c_str());
    return false;
  }
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() == 0) {
    ROS_ERROR("FARMaster: ~%s must be a nonempty YAML list.", parameter.c_str());
    return false;
  }

  std::vector<SemanticClassGroup> parsed;
  parsed.reserve(value.size());
  for (int i = 0; i < value.size(); ++i) {
    XmlRpc::XmlRpcValue& entry = value[i];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !entry.hasMember("name") || !entry.hasMember("rgb") ||
        entry["name"].getType() != XmlRpc::XmlRpcValue::TypeString ||
        entry["rgb"].getType() != XmlRpc::XmlRpcValue::TypeArray ||
        entry["rgb"].size() != 3) {
      ROS_ERROR("FARMaster: invalid entry %d in ~%s; expected {name, rgb: [r,g,b]}.",
                i, parameter.c_str());
      return false;
    }
    int rgb[3];
    for (int channel = 0; channel < 3; ++channel) {
      if (entry["rgb"][channel].getType() != XmlRpc::XmlRpcValue::TypeInt) {
        ROS_ERROR("FARMaster: RGB channels in ~%s must be integers.", parameter.c_str());
        return false;
      }
      rgb[channel] = static_cast<int>(entry["rgb"][channel]);
      if (rgb[channel] < 0 || rgb[channel] > 255) {
        ROS_ERROR("FARMaster: RGB channels in ~%s must be in [0,255].", parameter.c_str());
        return false;
      }
    }
    const uint32_t key = (static_cast<uint32_t>(rgb[0]) << 16) |
                         (static_cast<uint32_t>(rgb[1]) << 8) |
                         static_cast<uint32_t>(rgb[2]);
    parsed.emplace_back(static_cast<std::string>(entry["name"]), key);
  }
  groups.swap(parsed);
  return true;
}

bool ParseLocalVoxelLabels(const ros::NodeHandle& private_nh,
                           const std::string& parameter,
                           std::vector<uint32_t>& labels) {
  XmlRpc::XmlRpcValue value;
  if (!private_nh.getParam(parameter, value)) return true;
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("FARMaster: ~%s must be a YAML list of nonnegative integers.",
              parameter.c_str());
    return false;
  }
  std::vector<uint32_t> parsed;
  parsed.reserve(value.size());
  for (int i = 0; i < value.size(); ++i) {
    if (value[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
      ROS_ERROR("FARMaster: entry %d in ~%s is not an integer.",
                i, parameter.c_str());
      return false;
    }
    const int label = static_cast<int>(value[i]);
    if (label < 0) {
      ROS_ERROR("FARMaster: entry %d in ~%s is negative.",
                i, parameter.c_str());
      return false;
    }
    parsed.push_back(static_cast<uint32_t>(label));
  }
  std::sort(parsed.begin(), parsed.end());
  parsed.erase(std::unique(parsed.begin(), parsed.end()), parsed.end());
  labels.swap(parsed);
  return true;
}

struct PointCloudFieldView {
  bool valid = false;
  uint32_t offset = 0;
  uint8_t datatype = 0;
  uint32_t byte_width = 0;
};

uint32_t PointCloudDatatypeWidth(const uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8:
      return 1;
    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16:
      return 2;
    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32:
      return 4;
    case sensor_msgs::PointField::FLOAT64:
      return 8;
    default:
      return 0;
  }
}

PointCloudFieldView FindPointCloudField(
    const sensor_msgs::PointCloud2& cloud, const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) {
      const uint32_t byte_width = PointCloudDatatypeWidth(field.datatype);
      const bool fits_point = field.count > 0 && byte_width > 0 &&
          field.offset <= cloud.point_step &&
          byte_width <= cloud.point_step - field.offset;
      return PointCloudFieldView{
          fits_point, field.offset, field.datatype, byte_width};
    }
  }
  return PointCloudFieldView();
}

template <typename T>
T ReadPointCloudRaw(const uint8_t* data) {
  T output;
  std::memcpy(&output, data, sizeof(T));
  return output;
}

bool ReadPointCloudNumber(const uint8_t* point,
                          const PointCloudFieldView& field,
                          double& output) {
  if (!field.valid) return false;
  const uint8_t* data = point + field.offset;
  switch (field.datatype) {
    case sensor_msgs::PointField::INT8:
      output = ReadPointCloudRaw<int8_t>(data); return true;
    case sensor_msgs::PointField::UINT8:
      output = ReadPointCloudRaw<uint8_t>(data); return true;
    case sensor_msgs::PointField::INT16:
      output = ReadPointCloudRaw<int16_t>(data); return true;
    case sensor_msgs::PointField::UINT16:
      output = ReadPointCloudRaw<uint16_t>(data); return true;
    case sensor_msgs::PointField::INT32:
      output = ReadPointCloudRaw<int32_t>(data); return true;
    case sensor_msgs::PointField::UINT32:
      output = ReadPointCloudRaw<uint32_t>(data); return true;
    case sensor_msgs::PointField::FLOAT32:
      output = ReadPointCloudRaw<float>(data); return true;
    case sensor_msgs::PointField::FLOAT64:
      output = ReadPointCloudRaw<double>(data); return true;
    default:
      return false;
  }
}

void FinalizePointCloud(const PointCloudPtr& cloud) {
  if (!cloud) return;
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
}

void PublishSeconds(const ros::Publisher& publisher, const ros::WallDuration& duration) {
  std_msgs::Float32 message;
  message.data = static_cast<float>(duration.toSec());
  publisher.publish(message);
}

std::string LocalTimestampNow() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time;
  localtime_r(&time, &local_time);
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y-%m-%dT%H:%M:%S") << '.'
         << std::setw(3) << std::setfill('0') << milliseconds.count();
  return stream.str();
}

const char* GoalRecordLegacyHeader() {
  return "sequence,system_time,ros_time,message_stamp,source_frame,world_frame,"
         "odom_valid,graph_initialized,start_x,start_y,start_z,"
         "source_goal_x,source_goal_y,source_goal_z,goal_x,goal_y,goal_z";
}

const char* GoalRecordHeader() {
  return "sequence,system_time,ros_time,message_stamp,source_frame,world_frame,"
         "odom_valid,graph_initialized,start_x,start_y,start_z,"
         "source_goal_x,source_goal_y,source_goal_z,goal_x,goal_y,goal_z,"
         "session_id,selection_in_session,odom_stamp,semantic_map_stamp,"
         "odom_age_wall_s,semantic_map_age_wall_s,start_heading_x,"
         "start_heading_y,start_heading_z,nav_graph_nodes,static_global_nodes,"
         "static_candidate_nodes,dynamic_local_nodes,current_static_points,"
         "current_dynamic_points,semantic_graph_dirty,planner_running,"
         "previous_waypoint_valid,previous_waypoint_x,previous_waypoint_y,"
         "previous_waypoint_z";
}

constexpr std::size_t kGoalRecordAddedColumns = 21;

bool ParseGoalRecordSequence(const std::string& line, std::uint64_t* sequence) {
  if (!sequence) return false;
  const std::size_t comma = line.find(',');
  const std::string token = line.substr(0, comma);
  if (token.empty()) return false;
  std::istringstream stream(token);
  std::uint64_t value = 0;
  stream >> value;
  if (!stream || !stream.eof()) return false;
  *sequence = value;
  return true;
}

}  // namespace

/***************************************************************************************/

void FARMaster::Init() {
  /* initialize subscriber and publisher */
  reset_graph_sub_    = nh.subscribe("reset_visibility_graph", 5, &FARMaster::ResetGraphCallBack, this);
  odom_sub_           = nh.subscribe("odom_world", 5, &FARMaster::OdomCallBack, this);
  // registered_scan
  // scan_sub_           = nh.subscribe("/scan_cloud", 5, &FARMaster::ScanCallBack, this);
  // 这个是什么级别的终点。用户全局指定的终点，不是规划器自己的目标点。
  waypoint_sub_       = nh.subscribe("goal_point", 1, &FARMaster::WaypointCallBack, this);
  // joy_command_sub_    = nh.subscribe("/joy", 5, &FARMaster::JoyCommandCallBack, this);
  update_command_sub_ = nh.subscribe("update_visibility_graph", 5, &FARMaster::UpdateCommandCallBack, this);
  // 发给下游规划器的目标点，也就是局部终点
  goal_pub_           = nh.advertise<geometry_msgs::PointStamped>("way_point",5);
  boundary_pub_       = nh.advertise<geometry_msgs::PolygonStamped>("navigation_boundary",5);
  // Timers
  runtime_pub_        = nh.advertise<std_msgs::Float32>("runtime",1);
  planning_time_pub_  = nh.advertise<std_msgs::Float32>("planning_time",1);
  traverse_time_pub_  = nh.advertise<std_msgs::Float32>("far_traverse_time", 5);
  semantic_snapshot_time_pub_ = nh.advertise<std_msgs::Float32>("semantic_snapshot_time", 5);
  semantic_update_time_pub_ = nh.advertise<std_msgs::Float32>("semantic_planner_update_time", 5);
  semantic_callback_time_pub_ = nh.advertise<std_msgs::Float32>("semantic_callback_time", 5);
  main_loop_time_pub_ = nh.advertise<std_msgs::Float32>("far_main_loop_time", 5);
  // planning status publisher
  reach_goal_pub_     = nh.advertise<std_msgs::Bool>("far_reach_goal_status", 5);
  // Terminal formatting subscriber
  read_command_sub_   = nh.subscribe("read_file_dir", 1, &FARMaster::ReadFileCommand, this);
  save_command_sub_   = nh.subscribe("save_file_dir", 1, &FARMaster::SaveFileCommand, this);
  // DEBUG Publisher
  dynamic_obs_pub_     = nh.advertise<sensor_msgs::PointCloud2>("FAR_dynamic_obs_debug",1);
  surround_obs_debug_  = nh.advertise<sensor_msgs::PointCloud2>("FAR_obs_debug",1);
  // Expose the exact persistent-global plus current-local static cloud used
  // by Graph edge validation. The local-planner cloud below is deliberately
  // denser and therefore is not equivalent for reproducing Graph clearance.
  graph_static_obs_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("semantic_graph_static_obstacles", 1);
  global_confirmed_static_obs_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
      "semantic_global_confirmed_static_obstacles", 1);
  local_planner_static_obs_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("semantic_local_static_obstacles", 1);
  local_planner_dynamic_obs_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("semantic_local_dynamic_obstacles", 1);
  surround_obs_before_dyremove_debug_ = nh.advertise<sensor_msgs::PointCloud2>("FAR_obs_before_dyremove_debug",1);
  surround_obs_after_dyremove_debug_  = nh.advertise<sensor_msgs::PointCloud2>("FAR_obs_after_dyremove_debug",1);
  scan_grid_debug_     = nh.advertise<sensor_msgs::PointCloud2>("FAR_scanGrid_debug",1);
  new_PCL_pub_         = nh.advertise<sensor_msgs::PointCloud2>("FAR_new_debug",1);
  terrain_height_pub_  = nh.advertise<sensor_msgs::PointCloud2>("FAR_terrain_height_debug",1);

  this->LoadROSParams();
  this->InitializeGoalRecorder();

  semantic_map_sub_   = nh.subscribe(master_params_.semantic_map_topic, 1, &FARMaster::SemanticMapCallBack, this);
  if (master_params_.use_local_voxel_map) {
    local_voxel_sub_ = nh.subscribe(
        master_params_.local_voxel_topic, 1,
        &FARMaster::LocalVoxelMapCallBack, this);
    ROS_INFO("FARMaster: current contours use local voxel snapshots from %s; "
             "SemanticOcTree is global static evidence only.",
             master_params_.local_voxel_topic.c_str());
  }

  /*init path generation thred callback*/
  // 在这设置了全局地图规划器的循环频率，默认是5hz，但是调用默认参数后是2.5hz。
  const float duration_time = 0.99f / master_params_.main_run_freq;
  planning_event_ = nh.createTimer(ros::Duration(duration_time), &FARMaster::PlanningCallBack, this);

  /* init Dynamic Planner Processing Objects */
  contour_detector_.Init(cdetect_params_);
  graph_manager_.Init(nh, graph_params_);
  graph_planner_.Init(nh, gp_params_);
  contour_graph_.Init(cg_params_);
  planner_viz_.Init(nh);
  map_handler_.Init(map_params_);
  scan_handler_.Init(scan_params_);
  graph_msger_.Init(nh, msger_parmas_);

  /* init internal params */
  odom_node_ptr_      = NULL;
  is_cloud_init_      = false;
  is_odom_init_       = false;
  is_scan_init_       = false;
  is_planner_running_ = false;
  is_graph_init_      = false;
  has_pending_route_goal_ = false;
  has_commanded_goal_ = false;
  is_reset_env_       = false;
  is_stop_update_     = false;
  semantic_graph_dirty_ = false;
  planning_requested_ = false;
  odom_connections_dirty_ = true;
  has_odom_connection_position_ = false;
  timeout_stop_active_ = false;
  last_odom_receipt_ = ros::WallTime();
  last_semantic_map_receipt_ = ros::WallTime();
  last_local_voxel_receipt_ = ros::WallTime();
  last_odom_stamp_ = ros::Time();
  last_semantic_map_stamp_ = ros::Time();
  last_local_voxel_stamp_ = ros::Time();
  has_local_voxel_snapshot_ = false;

  // allocate memory to pointers
  new_vertices_ptr_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_obs_ptr_         = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_free_ptr_        = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_cloud_ptr_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  scan_grid_ptr_        = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  terrain_height_ptr_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  dyremove_before_obs_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  collision_obs_ptr_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  current_static_obs_ptr_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  persistent_static_obs_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  graph_static_collision_obs_ptr_ =
      PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  effective_dynamic_obs_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  dynamic_added_ptr_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  dynamic_removed_ptr_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  local_planner_static_obs_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  local_planner_dynamic_obs_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  viewpoint_around_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  kdtree_viewpoint_obs_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());

  // set kdtree sorted value
  FARUtil::kdtree_new_cloud_->setSortedResults(false);
  FARUtil::kdtree_filter_cloud_->setSortedResults(false);
  kdtree_viewpoint_obs_cloud_->setSortedResults(false);

  // init global utility cloud
  FARUtil::stack_new_cloud_->clear();
  FARUtil::stack_dyobs_cloud_->clear();

  // init TF listener
  tf_listener_ = new tf::TransformListener();

  // clear temp vectors and memory
  this->ClearTempMemory();
  FARUtil::robot_pos = Point3D(0,0,0);
  FARUtil::free_odom_p = Point3D(0,0,0);

  robot_pos_   = Point3D(0,0,0);
  nav_heading_ = Point3D(0,0,0);
  goal_waypoint_stamped_.header.frame_id = master_params_.world_frame;
  printf("\033[2J"), printf("\033[0;0H"); // cleanup screen
  std::cout<<std::endl;
  if (master_params_.is_static_env) {
    std::cout<<"\033[1;33m **************** STATIC ENV PLANNING **************** \033[0m\n"<<std::endl;
  } else {
    std::cout<< "\033[1;33m **************** DYNAMIC ENV PLANNING **************** \033[0m\n" << std::endl;
  }
  std::cout<<"\n"<<std::endl;
}

void FARMaster::ResetEnvironmentAndGraph() {
  this->ResetInternalValues();
  has_commanded_goal_ = false;
  if (!FARUtil::IsDebug) { // Terminal Output
    printf("\033[A"), printf("\033[A"), printf("\033[2K");
    std::cout<< "\033[1;31m V-Graph Resetting...\033[0m\n" << std::endl;
  }
  graph_manager_.ResetCurrentGraph();
  map_handler_.ResetGripMapCloud();
  graph_planner_.ResetPlannerInternalValues();
  contour_graph_.ResetCurrentContour();
  /* Reset clouds */
  FARUtil::surround_obs_cloud_->clear();
  FARUtil::stack_new_cloud_->clear();
  FARUtil::stack_dyobs_cloud_->clear();
  FARUtil::cur_new_cloud_->clear();
  FARUtil::cur_dyobs_cloud_->clear();
  if (collision_obs_ptr_) collision_obs_ptr_->clear();
  if (current_static_obs_ptr_) current_static_obs_ptr_->clear();
  if (persistent_static_obs_ptr_) persistent_static_obs_ptr_->clear();
  if (graph_static_collision_obs_ptr_) {
    graph_static_collision_obs_ptr_->clear();
  }
  if (effective_dynamic_obs_ptr_) effective_dynamic_obs_ptr_->clear();
  if (dynamic_added_ptr_) dynamic_added_ptr_->clear();
  if (dynamic_removed_ptr_) dynamic_removed_ptr_->clear();
  if (local_planner_static_obs_ptr_) local_planner_static_obs_ptr_->clear();
  if (local_planner_dynamic_obs_ptr_) local_planner_dynamic_obs_ptr_->clear();
  is_cloud_init_ = false;
  last_semantic_map_receipt_ = ros::WallTime();
  last_local_voxel_receipt_ = ros::WallTime();
  last_semantic_map_stamp_ = ros::Time();
  last_local_voxel_stamp_ = ros::Time();
  has_local_voxel_snapshot_ = false;
  /* Stop the robot if it is moving */
  goal_waypoint_stamped_.header.stamp = ros::Time::now();
  goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
  goal_pub_.publish(goal_waypoint_stamped_);
  NodePtrStack empty_path;
  planner_viz_.VizPath(empty_path);
}

bool FARMaster::DataIsFresh(std::string* reason) const {
  if (!is_cloud_init_ || !is_odom_init_) return false;
  const ros::WallTime now = ros::WallTime::now();
  if (master_params_.odom_timeout > 0.0f &&
      !last_odom_receipt_.isZero() &&
      (now - last_odom_receipt_).toSec() > master_params_.odom_timeout) {
    if (reason) *reason = "odometry input timed out";
    return false;
  }
  if (master_params_.use_local_voxel_map) {
    if (!has_local_voxel_snapshot_) {
      if (reason) *reason = "waiting for the first local voxel snapshot";
      return false;
    }
    if (master_params_.local_voxel_timeout > 0.0f &&
        (!last_local_voxel_receipt_.isZero() &&
         (now - last_local_voxel_receipt_).toSec() >
             master_params_.local_voxel_timeout)) {
      if (reason) *reason = "local voxel input timed out in wall time";
      return false;
    }
  } else if (master_params_.semantic_map_timeout > 0.0f &&
             !last_semantic_map_receipt_.isZero() &&
             (now - last_semantic_map_receipt_).toSec() >
                 master_params_.semantic_map_timeout) {
      if (reason) *reason = "semantic octomap input timed out";
      return false;
  }
  return true;
}

bool FARMaster::GlobalStaticEvidenceIsFresh() const {
  if (!map_handler_.HasSemanticMap() ||
      last_semantic_map_receipt_.isZero()) {
    return false;
  }
  if (master_params_.semantic_map_timeout <= 0.0f) return true;
  if ((ros::WallTime::now() - last_semantic_map_receipt_).toSec() >
      master_params_.semantic_map_timeout) {
    return false;
  }
  return true;
}

void FARMaster::PublishStopCommand(const std::string& reason) {
  if (timeout_stop_active_) return;
  timeout_stop_active_ = true;
  is_planner_running_ = false;
  nav_heading_ = Point3D(0, 0, 0);
  goal_waypoint_stamped_.header.stamp = ros::Time::now();
  goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
  goal_pub_.publish(goal_waypoint_stamped_);
  planner_viz_.VizPath(NodePtrStack());
  ROS_ERROR("FARMaster: %s; publishing a stop waypoint and empty path.",
            reason.c_str());
}

bool FARMaster::PreconditionCheck() {
  if (!is_cloud_init_ || !is_odom_init_) return false;
  std::string reason;
  if (!this->DataIsFresh(&reason)) {
    this->PublishStopCommand(reason);
    return false;
  }
  if (timeout_stop_active_) {
    timeout_stop_active_ = false;
    ROS_INFO("FARMaster: required odometry and local obstacle inputs are fresh "
             "again; planning may resume.");
  }
  return true;
}

void FARMaster::Loop() {
  ros::Rate loop_rate(master_params_.main_run_freq);
  while (ros::ok()) {
    const ros::WallTime loop_start = ros::WallTime::now();
    if (is_reset_env_) {
      this->ResetEnvironmentAndGraph(); 
      is_reset_env_ = false;
      if (FARUtil::IsDebug) ROS_WARN("****************** Graph and Env Reset ******************");
      loop_rate.sleep(); // skip this iteration
      continue;
    }
    /* Process callback functions */
    ros::spinOnce(); 
    if (!this->PreconditionCheck()) {
      loop_rate.sleep();
      continue;
    }
    /* add main process after this line */
    graph_manager_.UpdateRobotPosition(robot_pos_);
    odom_node_ptr_ = graph_manager_.GetOdomNode();
    if (odom_node_ptr_ == NULL) {
      ROS_WARN("FAR: Waiting for Odometry...");
      loop_rate.sleep();
      continue;
    }

    if (!has_odom_connection_position_ ||
        (robot_pos_ - last_odom_connection_position_).norm_flat() >=
            master_params_.odom_connection_update_distance) {
      odom_connections_dirty_ = true;
    }

    // A new semantic snapshot owns the expensive contour/topology rebuild.
    // Between snapshots, robot motion only refreshes transient start edges.
    if (!semantic_graph_dirty_) {
      bool publish_graph_snapshot = false;
      if (is_graph_init_ && odom_connections_dirty_) {
        graph_manager_.UpdateOdomConnections();
        nav_graph_ = graph_manager_.GetNavGraph();
        graph_planner_.UpdaetVGraph(nav_graph_);
        graph_msger_.UpdateGlobalGraph(nav_graph_);
        last_odom_connection_position_ = robot_pos_;
        has_odom_connection_position_ = true;
        odom_connections_dirty_ = false;
        planning_requested_ = true;
        publish_graph_snapshot = true;
      }
      if (planning_requested_ && is_graph_init_) {
        planning_requested_ = false;
        this->ExecutePlanningCycle();
        publish_graph_snapshot = true;
      }
      if (publish_graph_snapshot) {
        // Goal and odom edges are a transient query layer. Publish only after
        // ExecutePlanningCycle has rebuilt them against this obstacle cloud,
        // otherwise RViz/monitors observe a stale edge with a new map.
        planner_viz_.VizPointCloud(graph_static_obs_pub_,
                                   graph_static_collision_obs_ptr_);
        planner_viz_.VizPointCloud(global_confirmed_static_obs_pub_,
                                   persistent_static_obs_ptr_);
        planner_viz_.VizGraph(nav_graph_);
        planner_viz_.VizSemanticGraphLayers(
            graph_manager_.GetStaticGraphNodes(),
            graph_manager_.GetStaticMainGraphNodes(),
            graph_manager_.GetDynamicLocalNodes(),
            graph_manager_.GetEligibleSearchGraph(), nav_graph_);
      }
      PublishSeconds(main_loop_time_pub_, ros::WallTime::now() - loop_start);
      loop_rate.sleep();
      continue;
    }

    graph_manager_.BeginSemanticGraphUpdate();
    /* Extract Vertices and new nodes */
    FARUtil::Timer.start_time("Total V-Graph Update");
    // 在做“从周围障碍点云里提取轮廓”的核心步骤，作用是把稠密点云转换成后续可建图的几何边界。
    // 不是“检测整个地图的轮廓”，而是检测机器人当前周围局部障碍云的轮廓。
    // 以当前 odom 节点为中心，把机器人周围的局部障碍点云投影成图像，然后提取这片局部环境的障碍轮廓。
    static_contours_.clear();
    dynamic_contours_.clear();
    contour_detector_.BuildTerrainImgAndExtractContour(
        odom_node_ptr_, current_static_obs_ptr_, static_contours_,
        true);  // semantic octomap points are verified occupied voxels
    if (effective_dynamic_obs_ptr_ && !effective_dynamic_obs_ptr_->empty()) {
      contour_detector_.BuildTerrainImgAndExtractContour(
          odom_node_ptr_, effective_dynamic_obs_ptr_, dynamic_contours_, true,
          cdetect_params_.dynamic_simplify_ratio);
    }
    // 把刚提取出的“真实世界轮廓”写进轮廓图模块，完成轮廓图的本帧更新。
    contour_graph_.UpdateContourGraph(odom_node_ptr_, static_contours_,
                                      dynamic_contours_);
    if (is_graph_init_) {
      if (!FARUtil::IsDebug) printf("\033[2K");
      std::cout<<"    "<<"Local V-Graph Updated. Number of local vertices: "<<ContourGraph::contour_graph_.size()<<std::endl;
    }
    /* Adjust heights with terrain */
    // 你看到的这两行就是在把“平面上的图结构”贴到“可通行地形高程”上，说明这个规划不是纯 2D，而是带高度约束的 2.5D/3D 处理。
    // 为什么这里会有高程：
    // 上游 semantic octomap 回调会更新局部 terrain-support/free 语义点缓存。
    // TerrainCallBack 中调用 UpdateTerrainHeightGrid(...)，内部基于语义点更新 terrain KDTree。
    // 高度查询由 KDTree 驱动，不再依赖本地 terrain 栅格推断。
    // AdjustCTNodeHeight(...) 和 AdjustNodesHeight(...)
    // 分别给轮廓图节点、导航图节点校正 z 值，让节点高度与可通行地面一致，避免路径“悬空”或“钻地”。
    // 所以你问“当前可通行地形是高程图吗”：
    // 是，但实现形态是语义点云派生的局部地形高度查询（KDTree），不是旧的本地高程栅格推断。

    // 修正contour_graph_和nav_graph_的高度，使它们与地形高度一致，避免出现悬空或钻地的情况。
    // CTNode：轮廓几何节点, 本质上是“障碍轮廓上的一个角点/轮廓点”，服务于轮廓图构建和多边形几何关系。
    // nav_graph_：导航图节点,真正参与图搜索和路径规划的节点
    map_handler_.AdjustCTNodeHeight(ContourGraph::contour_graph_);
    NodePtrStack matching_graph = graph_manager_.GetMatchingGraph();
    map_handler_.AdjustNodesHeight(matching_graph);
    // Truncate for local range nodes
    // “从全局图里筛出机器人附近节点集合”，给后续局部匹配和增量更新用。
    // 根据当前机器人位置，更新 DynamicGraph 内部的“近邻节点缓存”（哪些全局导航节点算近、算扩展近邻）。
    // 先筛出机器人当前局部真正相关的图节点，再把这个局部子集交给后面的轮廓匹配和增量更新流程使用。换句话说，这一步是在做“全局图的局部裁剪与近邻维护”，让后续的 contour matching 和 graph update 只处理机器人周围有效节点。
    graph_manager_.UpdateGlobalNearNodes();
    // 把刚更新好的“扩展局部近邻节点集合”取出来，赋给 near_nav_graph_。
    // 只是把前面在 DynamicGraph::UpdateGlobalNearNodes 里筛出来的“扩展局部近邻节点”拿出来给外部用。
    near_nav_graph_ = graph_manager_.GetExtendLocalNode();
    // Match near nav nodes with contour
    // 先缩小到局部相关节点，再做轮廓与导航图匹配，减少无关计算、提升稳定性和速度。
    // MatchContourWithNavGraph() 是把当前帧提取出来的轮廓点，和全局导航图里的近邻节点做匹配的函数。它的输入是两组节点：global_nodes 是全局导航图，near_nodes 是前面筛出来的局部近邻节点；输出是 new_convex_vertices，也就是这帧里“没被现有导航节点匹配上、需要当成新顶点处理”的轮廓点。
    // 它的作用就是“先拿局部轮廓去对齐已有图节点，剩下的才当作新顶点候选”。这一步是增量建图的关键：不是直接把所有轮廓都变成新节点，而是先复用已有导航节点，减少重复顶点和错误建图。
    contour_graph_.MatchContourWithNavGraph(
        matching_graph, near_nav_graph_, new_ctnodes_,
        graph_params_.static_duplicate_radius);
    graph_manager_.FinalizeDynamicGraphUpdate();
    if (master_params_.is_visual_opencv) {
      FARUtil::ConvertCTNodeStackToPCL(new_ctnodes_, new_vertices_ptr_);
      cv::Mat cloud_img = contour_detector_.GetCloudImgMat();
      contour_detector_.ShowCornerImage(cloud_img, new_vertices_ptr_);
    }
    /* update planner graph */
    new_nodes_.clear();
    // 把本帧“候选轮廓顶点”转成“可加入全局图的新导航节点”。
    // new_ctnodes_ 就是上一阶段输出的“新增候选轮廓点”，来源是 contour_graph.cpp:65 的第三个输出参数。这个函数会把未匹配且满足几何条件的 CTNode 放进 new_convex_vertices，实际在主流程里就是 new_ctnodes_。
    if (!is_stop_update_ && graph_manager_.ExtractGraphNodes(new_ctnodes_)) {
      new_nodes_ = graph_manager_.GetNewNodes();
    }
    if (is_graph_init_) {
      if (!FARUtil::IsDebug) printf("\033[2K");
      std::cout<<"    "<< "Number of new vertices adding to global V-Graph: "<< new_nodes_.size()<<std::endl;
    }
    /* Graph Updating */
    // 真正把“本帧增量信息”合并进全局可视图的核心更新器
    graph_manager_.UpdateNavGraph(new_nodes_, is_stop_update_, clear_nodes_);
    graph_manager_.CommitSemanticGraphUpdate(
        [this](const Point3D& point) {
          if (graph_params_.static_promotion_requires_global_evidence &&
              !this->GlobalStaticEvidenceIsFresh()) {
            return StaticNodeEvidence::UNKNOWN;
          }
          return map_handler_.QueryStaticNodeEvidence(point);
        });
    runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", is_graph_init_) / 1000.f; // Unit: second
    // runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", is_graph_init_); // Unit: ms
    runtime_pub_.publish(runtimer_);
    /* Update v-graph in other modules */
    // 在调用 UpdateNavGraph 之后，立刻拿当前最新图，后面会连续喂给规划器、消息发布和可视化（同一帧一致性）。
    nav_graph_ = graph_manager_.GetNavGraph();
    if (is_graph_init_) {
      if (!FARUtil::IsDebug) printf("\033[2K");
      std::cout<<"    "<<"Global V-Graph Updated. Number of global vertices: "<<nav_graph_.size()<<std::endl;
    }
    // 从当前全局图连接关系里重新提取轮廓集合，更新全局/局部边界相关缓存
    contour_graph_.ExtractGlobalContours();      // Global Polygon Update
    // 把最新 nav_graph_ 交给路径规划器，让后续 PathToGoal、可达性判断都基于刚更新后的图。
    graph_planner_.UpdaetVGraph(nav_graph_);     // Graph Planner Update
    // 把同一份最新全局图同步到消息发布模块，用于对外发送图结构（多机通信/可视化消费者）。
    graph_msger_.UpdateGlobalGraph(nav_graph_);  // Graph Messager Update
    // Planning callbacks are allowed again only after contours, nodes, edges,
    // and the GraphPlanner snapshot all correspond to the newest semantic map.
    semantic_graph_dirty_ = false;
    odom_connections_dirty_ = false;
    last_odom_connection_position_ = robot_pos_;
    has_odom_connection_position_ = true;
    planning_requested_ = true;

    /* Publish local boundary to lower level local planner */
    // 局部地图中和当前运动相关的一些边界点对，供下游局部规划器使用。比如在局部规划器里做“局部可通行性检查”时，可以用这些边界点对来判断是否碰到障碍。
    this->LocalBoundaryHandler(ContourGraph::local_boundary_);

    /* Viz Navigation Graph */
    const NavNodePtr last_internav_ptr = graph_manager_.GetLastInterNavNode();
    if (last_internav_ptr != NULL) {
      planner_viz_.VizPoint3D(last_internav_ptr->position, "last_nav_node", VizColor::MAGNA, 1.0);
    }
    planner_viz_.VizNodes(clear_nodes_, "clear_nodes", VizColor::ORANGE);
    planner_viz_.VizNodes(graph_manager_.GetOutContourNodes(), "out_contour", VizColor::YELLOW);
    planner_viz_.VizPoint3D(FARUtil::free_odom_p, "free_odom_position", VizColor::ORANGE, 1.0);
    planner_viz_.VizContourGraph(ContourGraph::contour_graph_);
    planner_viz_.VizGlobalPolygons(ContourGraph::global_contour_, ContourGraph::unmatched_contour_);

    if (is_graph_init_) { 
      if (FARUtil::IsDebug) {
        std::cout<<" ========================================================== "<<std::endl;
      } else { // cleanup outputs in terminal
        for (int i = 0; i < 6; i++) {
          printf("\033[A");
        }
      }
    }

    if (!is_graph_init_ && !nav_graph_.empty()) {
      is_graph_init_ = true;
      printf("\033[A"), printf("\033[A"), printf("\033[2K");
      std::cout<< "\033[1;32m V-Graph Initialized \033[0m\n" << std::endl;
      if (has_pending_route_goal_) {
        const geometry_msgs::PointStamped pending_goal = pending_route_goal_;
        has_pending_route_goal_ = false;
        ROS_INFO("FARMaster: applying the goal retained during V-Graph initialization.");
        this->ApplyWorldGoal(Point3D(pending_goal.point.x,
                                    pending_goal.point.y,
                                    pending_goal.point.z));
      }
    }
    if (planning_requested_ && is_graph_init_) {
      planning_requested_ = false;
      this->ExecutePlanningCycle();
    }
    // Keep the collision cloud and graph geometry in the same post-planning
    // snapshot. In particular, no GOAL_CONNECT edge from the preceding map
    // may be visualized as if it belonged to the newly rebuilt map.
    planner_viz_.VizPointCloud(graph_static_obs_pub_,
                               graph_static_collision_obs_ptr_);
    planner_viz_.VizPointCloud(global_confirmed_static_obs_pub_,
                               persistent_static_obs_ptr_);
    planner_viz_.VizGraph(nav_graph_);
    planner_viz_.VizSemanticGraphLayers(
        graph_manager_.GetStaticGraphNodes(),
        graph_manager_.GetStaticMainGraphNodes(),
        graph_manager_.GetDynamicLocalNodes(),
        graph_manager_.GetEligibleSearchGraph(), nav_graph_);
    PublishSeconds(main_loop_time_pub_, ros::WallTime::now() - loop_start);
    loop_rate.sleep();
  }
}

// The timer never reads input freshness and never publishes a stop.  It only
// requests work; Loop() executes that request after spinOnce() has processed
// the current subscription callbacks and after the graph snapshot is ready.
void FARMaster::PlanningCallBack(const ros::TimerEvent& event) {
  (void)event;
  planning_requested_ = true;
}

void FARMaster::ExecutePlanningCycle() {
  const NavNodePtr goal_ptr = graph_planner_.GetGoalNodePtr();
  if (goal_ptr == NULL) {
    /* Graph Traversablity Update */
    if (!FARUtil::IsDebug) printf("\033[2K");
    std::cout<<"    "<<"Adding Goal to V-Graph "<<"Time: "<<0.f<<"ms"<<std::endl;
    // 图可达性更新
    graph_planner_.UpdateGraphTraverability(odom_node_ptr_, NULL);
    if (!FARUtil::IsDebug) printf("\033[2K");
    std::cout<<"    "<<"Path Search "<<"Time: "<<0.f<<"ms"<<std::endl;
  } else { 
    // 会先在目标附近拿障碍/自由点云，更新目标附近可通行信息，再重新评估 goal 位置是否合理。
    // Update goal postion with nearby terrain cloud
    const Point3D ori_p = graph_planner_.GetOriginNodePos(true);
    PointCloudPtr goal_obs(new pcl::PointCloud<PCLPoint>());
    PointCloudPtr goal_free(new pcl::PointCloud<PCLPoint>());
    map_handler_.GetCloudOfPoint(ori_p, goal_obs, CloudType::OBS_CLOUD, true);
    map_handler_.GetCloudOfPoint(ori_p, goal_free, CloudType::FREE_CLOUD, true);
    graph_planner_.UpdateFreeTerrainGrid(ori_p, goal_obs, goal_free);
    graph_planner_.ReEvaluateGoalPosition(goal_ptr, !master_params_.is_multi_layer);

    // Refresh reachability on the current obstacle Graph before deciding
    // which contour vertices may connect to the goal.  Passing no goal makes
    // the traversal ignore every old GOAL_CONNECT edge, so a stale goal edge
    // cannot make an otherwise disconnected component appear reachable.
    graph_planner_.UpdateGraphTraverability(odom_node_ptr_, NULL);

    // Adding goal into v-graph
    FARUtil::Timer.start_time("Adding Goal to V-Graph");
    graph_planner_.UpdateGoalNavNodeConnects(goal_ptr);
    // The goal pointer is connected directly to every eligible node in the
    // current snapshot above. UpdaetVGraph() remains owned by the ordinary
    // semantic Graph update so this planning callback cannot mix snapshots.
    if (!FARUtil::IsDebug) printf("\033[2K");
    FARUtil::Timer.end_time("Adding Goal to V-Graph");

    // Update v-graph traversibility 
    FARUtil::Timer.start_time("Path Search");
    graph_planner_.UpdateGraphTraverability(odom_node_ptr_, goal_ptr);

    // Construct path to gaol and publish waypoint
    NodePtrStack global_path;
    Point3D current_free_goal;
    NavNodePtr last_nav_ptr = nav_node_ptr_;
    bool is_planning_fails = false;
    goal_waypoint_stamped_.header.stamp = ros::Time::now();
    bool is_current_free_nav = false;
    bool is_reach_goal = false;
    bool is_retry_wait = false;
    const bool has_effective_dynamic_obstacles =
        effective_dynamic_obs_ptr_ && !effective_dynamic_obs_ptr_->empty();
    // 根据当前可见图和目标节点搜索出来的一条“图路径”（节点序列）global_path
    if (graph_planner_.PathToGoal(goal_ptr, global_path, nav_node_ptr_,
                                  current_free_goal, is_planning_fails,
                                  has_effective_dynamic_obstacles,
                                  is_retry_wait, is_reach_goal,
                                  is_current_free_nav) &&
        nav_node_ptr_ != NULL) {
      Point3D waypoint = nav_node_ptr_->position;
      const bool has_contour_route = graph_planner_.NextContourRouteWaypoint(
          global_path, robot_pos_, waypoint);
      if (has_contour_route) {
        waypoint = this->ProjectContourWaypointProgressively(
            nav_node_ptr_, waypoint);
        const Point3D route_heading = waypoint - robot_pos_;
        if (route_heading.norm() > FARUtil::kEpsilon) {
          nav_heading_ = route_heading.normalize();
        }
      } else if (nav_node_ptr_ != goal_ptr) {
        waypoint = this->ProjectNavWaypoint(nav_node_ptr_, last_nav_ptr);
      } else if (master_params_.is_viewpoint_extend) {
        planner_viz_.VizViewpointExtend(goal_ptr, goal_ptr->position);
      }
      goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(waypoint);
      // 发布waypoint
      goal_pub_.publish(goal_waypoint_stamped_);
      is_planner_running_ = true;
      planner_viz_.VizPoint3D(waypoint, "waypoint", VizColor::MAGNA, 1.5);
      planner_viz_.VizPoint3D(current_free_goal, "free_goal", VizColor::GREEN, 1.5);
      planner_viz_.VizPath(global_path, is_current_free_nav,
                           has_commanded_goal_ ? &commanded_goal_ : nullptr);
    } else if (is_retry_wait) {
      // The current static+dynamic Graph has no route. Stop safely without
      // deleting the commanded destination; a later freshly rebuilt Graph
      // may supply a direct edge, a door corner, or a restored stitch edge.
      global_path.clear();
      planner_viz_.VizPath(global_path);
      goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
      goal_pub_.publish(goal_waypoint_stamped_);
      is_planner_running_ = false;
      nav_heading_ = Point3D(0,0,0);
    } else if (is_planner_running_) {
      // stop robot
      global_path.clear();
      planner_viz_.VizPath(global_path);
      is_planner_running_ = false;
      nav_heading_ = Point3D(0,0,0);
      if (is_planning_fails) { // stops the robot
        goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
        goal_pub_.publish(goal_waypoint_stamped_);
      }
    }
    if (!FARUtil::IsDebug) printf("\033[2K");

    // publish planner status and timers
    std_msgs::Bool reach_goal_msg;
    reach_goal_msg.data = is_reach_goal;
    reach_goal_pub_.publish(reach_goal_msg);
    std_msgs::Float32 traverse_timer;
    traverse_timer.data = FARUtil::Timer.record_time("Overall_executing");
    traverse_time_pub_.publish(traverse_timer);
    if (is_reach_goal) {
      FARUtil::Timer.end_time("Overall_executing", false);
    }
    plan_timer_.data = FARUtil::Timer.end_time("Path Search");
    planning_time_pub_.publish(plan_timer_);
  }
}

// 把已经算好的局部边界线段，筛一遍并打包成 ROS 消息，发布给下游局部规划器/可视化使用。
void FARMaster::LocalBoundaryHandler(const std::vector<PointPair>& local_boundary) {
  if (!master_params_.is_pub_boundary || local_boundary.empty()) return;
  geometry_msgs::PolygonStamped boundary_poly;
  boundary_poly.header.frame_id = master_params_.world_frame;
  boundary_poly.header.stamp = ros::Time::now();
  float index_z = robot_pos_.z;
  std::vector<PointPair> sorted_boundary;
  // 按距离筛选边界
  // 遍历每条边界线段 edge，计算机器人到线段的 2D 距离；
  // 超过 local_planner_range 的边界段丢弃，只保留“当前局部相关”的。
  for (const auto& edge : local_boundary) {
    if (FARUtil::DistanceToLineSeg2D(robot_pos_, edge) > master_params_.local_planner_range) continue;
    sorted_boundary.push_back(edge);
  }
  // 把保留的边界按机器人中心做顺时针排序（主要是为了 RViz 显示更整齐）。
  FARUtil::SortEdgesClockWise(robot_pos_, sorted_boundary); /* For better rviz visualization 
  // 把每条线段的两个端点写入 polygon.points，然后发布到 /navigation_boundary。purpose only! */ 
  for (const auto& edge : sorted_boundary) {
    geometry_msgs::Point32 geo_p1, geo_p2;
    geo_p1.x = edge.first.x,  geo_p1.y = edge.first.y,  geo_p1.z = index_z;
    geo_p2.x = edge.second.x, geo_p2.y = edge.second.y, geo_p2.z = index_z;
    boundary_poly.polygon.points.push_back(geo_p1), boundary_poly.polygon.points.push_back(geo_p2);
    index_z += 0.001f; // seperate polygon lines
  }
  boundary_pub_.publish(boundary_poly);
}

// 把图上的导航节点位置，投影/修正成一个更适合下游机器人去跟踪的 waypoint。点是xyz坐标，方向是 heading 向量。这个函数的作用是“把图节点位置投影成一个更平滑、更前瞻的 waypoint”，避免机器人在跟踪时出现“急转弯”或“方向突变”。

// 局部跟踪点生成器
// 输入当前选中的导航节点 nav_node_ptr（来自图搜索结果）
// 可选做前瞻延伸（ExtendViewpointOnObsCloud）
// 融合历史 heading 做动量平滑，减少急转
// 避免方向反跳（接近反向时重投影）
// 保证最小前视距离，输出最终 waypoint 给底层控制跟踪
Point3D FARMaster::ProjectNavWaypoint(const NavNodePtr& nav_node_ptr, const NavNodePtr& last_point_ptr) {
  bool is_momentum = false;
  // 判断是否要保留运动惯性
  // 如果上一次跟踪的点和这次节点很接近，或者就是同一个点，就认为可以保留一部分上一时刻的 heading，避免 waypoint 方向突变。
  if (last_point_ptr == nav_node_ptr || (last_point_ptr != NULL && (last_point_ptr->position - nav_node_ptr_->position).norm() < FARUtil::kNearDist)) {
    is_momentum = true;
  }
  // 计算候选 waypoint
  // 初始先取 nav_node_ptr->position。
  // 然后调用 ExtendViewpointOnObsCloud(...)，尝试把点沿着可视方向再往前“推一点”，让 waypoint 更像一个前瞻点，而不是死贴在图节点上。
  Point3D waypoint = nav_node_ptr->position;
  // 默认参数输入值5m
  float free_dist = master_params_.local_planner_range;
  const Point3D extend_p = this->ExtendViewpointOnObsCloud(nav_node_ptr_, FARUtil::surround_obs_cloud_, free_dist);
  free_dist = std::max(free_dist, master_params_.robot_dim * 2.5f);
  if (master_params_.is_viewpoint_extend) {
    waypoint = extend_p;
    planner_viz_.VizViewpointExtend(nav_node_ptr_, waypoint);
  }
  // 融合 heading，避免急转
  // 如果启用了 momentum，它会把“当前应去方向”和“上一时刻 heading”做加权融合，让新方向更平滑。
  // 这个的目的是当终点就在前一个附近时，不要反复的调换方向
  // nav_heading_ 当前的方向
  const Point3D diff_p = waypoint - robot_pos_;
  Point3D new_heading;
  if (is_momentum && nav_heading_.norm() > FARUtil::kEpsilon) {
    const float hdist = free_dist / 2.0f;
    const float ratio = std::min(hdist, diff_p.norm()) / hdist;
    new_heading = diff_p.normalize() * ratio + nav_heading_ * (1.0f - ratio);
  } else {
    new_heading = diff_p.normalize();
  }
  // 防止方向反跳
  // 如果新 heading 和旧 heading 近乎反向，它会做一次侧向重投影，避免 waypoint 一下子跳到相反方向，导致控制不稳定。
  if (nav_heading_.norm() > FARUtil::kEpsilon && new_heading.norm_dot(nav_heading_) < 0.0f) { // negative direction reproject
    Point3D temp_heading(nav_heading_.y, -nav_heading_.x, nav_heading_.z);
    if (temp_heading.norm_dot(new_heading) < 0.0f) {
      temp_heading.x = -temp_heading.x, temp_heading.y = -temp_heading.y;
    }
    new_heading = temp_heading;
  }
  // 保证 waypoint 有前瞻距离
  // 如果 waypoint 离机器人太近，它会沿 heading 再往前推一点，至少保持 free_dist 的跟踪前视距离。
  nav_heading_ = new_heading.normalize();
  if (diff_p.norm() < free_dist) {
    waypoint = waypoint + nav_heading_ * (free_dist - diff_p.norm());
  }
  return waypoint;
}

EdgeRejectReason FARMaster::ValidateProjectedWaypoint(
    const Point3D& candidate) const {
  // The waypoint is an execution-time shortcut rather than persistent Graph
  // topology.  Check the exact robot-centre segment against only the latest
  // already-cropped semantic layers; this never queries the complete OctoMap.
  if (!ContourGraph::IsRouteConnectFreeStaticLayer(robot_pos_, candidate)) {
    return EdgeRejectReason::STATIC_CLOUD_BLOCKED;
  }
  if (!ContourGraph::IsRouteConnectFreeDynamicLayer(robot_pos_, candidate)) {
    return EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED;
  }
  if (!DynamicGraph::IsOnTerrainRoute(robot_pos_, candidate)) {
    return EdgeRejectReason::TERRAIN_BLOCKED;
  }
  return EdgeRejectReason::NONE;
}

Point3D FARMaster::ProjectContourWaypointProgressively(
    const NavNodePtr& nav_node_ptr, const Point3D& safe_fallback) {
  if (!master_params_.is_viewpoint_extend || !nav_node_ptr ||
      nav_node_ptr->free_direct != NodeFreeDirect::CONVEX) {
    return safe_fallback;
  }

  bool is_wall = false;
  // Preserve the original FAR convention: surf_dirs point into the obstacle,
  // so their negative topological sum points toward free space.
  const Point3D free_direction =
      -FARUtil::SurfTopoDirect(nav_node_ptr->surf_dirs, is_wall);
  if (is_wall || free_direction.norm_flat() <= FARUtil::kEpsilon) {
    return safe_fallback;
  }

  const float first_distance = std::max(
      FARUtil::kNearDist,
      FARUtil::kNavClearDist + FARUtil::kLeafSize * 0.5f);
  const float maximum_distance = master_params_.local_planner_range;
  if (maximum_distance < first_distance) return safe_fallback;
  const float distance_step = std::max(FARUtil::kLeafSize, 0.05f);
  EdgeRejectReason first_rejection = EdgeRejectReason::NONE;
  const ProgressiveProjectionResult projection =
      FindFurthestConsecutiveSafeProjection(
          first_distance, maximum_distance, distance_step,
          [this, &nav_node_ptr, &free_direction, &first_rejection](
              const float distance) {
            Point3D candidate =
                nav_node_ptr->position + free_direction * distance;
            candidate.z = nav_node_ptr->position.z;
            const EdgeRejectReason reason =
                this->ValidateProjectedWaypoint(candidate);
            if (reason != EdgeRejectReason::NONE) {
              first_rejection = reason;
              return false;
            }
            return true;
          });

  if (!projection.has_safe_projection) {
    ROS_DEBUG_THROTTLE(
        1.0,
        "FAR waypoint projection: node=%zu no free-side candidate; "
        "fallback to checked edge geometry (reason=%d).",
        nav_node_ptr->id, static_cast<int>(first_rejection));
    return safe_fallback;
  }

  Point3D waypoint = nav_node_ptr->position +
      free_direction * projection.distance;
  waypoint.z = nav_node_ptr->position.z;
  planner_viz_.VizViewpointExtend(nav_node_ptr, waypoint);
  ROS_DEBUG_THROTTLE(
      1.0,
      "FAR waypoint projection: node=%zu accepted %.2f m after %zu "
      "near-to-far checks%s.",
      nav_node_ptr->id, projection.distance,
      projection.evaluated_candidates,
      first_rejection == EdgeRejectReason::NONE
          ? " (maximum reached)" : " (next candidate blocked)");
  return waypoint;
}

// 它试图把当前导航节点沿着一个“更合理的可视/拓扑方向”往前延伸成一个更好的 waypoint，但如果前面有障碍，就提前停下甚至往回收一点。

// 这个地方存在一个代码瑕疵,但是还不会影响整体规划器的功能.就是nav_node_ptr_和nav_node_ptr的使用不一致,导致在某些情况下会出现错误的结果.但是目前还没有发现明显的bug,所以暂时不修改.
Point3D FARMaster::ExtendViewpointOnObsCloud(const NavNodePtr& nav_node_ptr, const PointCloudPtr& obsCloudIn, float& free_dist) {
  // 如果这个导航节点不是凸自由方向，或者周围没有障碍点云，就直接返回原节点位置。
  // 也就是说，不是所有节点都会被延伸。
  if (nav_node_ptr_->free_direct != NodeFreeDirect::CONVEX || obsCloudIn->empty())        
    return nav_node_ptr_->position;
  // 从障碍云里裁一小块“节点周围的局部障碍”
  // 它会把 nav node 周围一定范围内的障碍点取出来，后面只在这小片局部障碍里判断。
  // 所以这是一个局部几何修正，不是全局搜索。
  FARUtil::CropPCLCloud(obsCloudIn, viewpoint_around_ptr_, nav_node_ptr_->position, free_dist + FARUtil::kNearDist);
  float maxR = std::min((nav_node_ptr_->position - robot_pos_).norm(), free_dist) - FARUtil::kNearDist;
  maxR = std::max(maxR, 0.0f);
  bool is_wall = false;
  // 确定要往哪个方向推
  // 从节点附近障碍/表面拓扑里估一个方向
  // 再取反，得到“朝开放空间/前方”去的方向
  const Point3D direct = -FARUtil::SurfTopoDirect(nav_node_ptr_->surf_dirs, is_wall);
  if (!is_wall) {
    Point3D waypoint = nav_node_ptr_->position;
    // 如果周围根本没有障碍点
    // 那就简单很多：直接沿 direct 推到 maxR 那么远
    if (viewpoint_around_ptr_->empty()) {
      waypoint = waypoint + direct * maxR;
    } else {
      kdtree_viewpoint_obs_cloud_->setInputCloud(viewpoint_around_ptr_);
      const int N_Thred = (int)std::floor(FARUtil::kNearDist / FARUtil::kLeafSize);
      const float R = FARUtil::kNearDist / 2.0f + FARUtil::kLeafSize;
      // ray tracing
      Point3D start_p = waypoint + direct * FARUtil::kNearDist;
      float ray_dist  = FARUtil::kNearDist; 
      bool is_occupied = FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_) > N_Thred;
      waypoint = start_p;
      // 如果周围有障碍点，就做一段“射线试探”
      // 这部分最像你看到的“反弹感”来源。
      // 它会从节点前方一点点开始，沿 direct 一步一步往前探：
      // 每走一步，就检查当前位置附近是否有足够多障碍点
      // 如果还没碰到障碍，继续往前
      // 一旦碰到障碍，停止前推
      while (!is_occupied && ray_dist < free_dist) {
        start_p = start_p + direct * FARUtil::kNearDist;
        ray_dist += FARUtil::kNearDist;
        is_occupied = FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_) > N_Thred;
        if (ray_dist < maxR) {
          waypoint = start_p;
        }
      }
      // 碰到障碍后，不是停在碰撞点，而是回收一点
      if (is_occupied) {
        waypoint = (nav_node_ptr_->position + waypoint - direct * FARUtil::kNearDist) / 2.0f;
        waypoint.z = nav_node_ptr_->position.z;
        free_dist = ray_dist - FARUtil::kNearDist;
      }
      return waypoint;
    }
  }
  return nav_node_ptr_->position;
}


void FARMaster::LoadROSParams() {
  // All parameters are private to the node. This keeps the planner reusable
  // under a namespace while preserving the existing YAML key layout.
  const std::string master_prefix   = "";
  const std::string map_prefix      = master_prefix + "MapHandler/";
  const std::string scan_prefix     = master_prefix + "ScanHandler/";
  const std::string cdetect_prefix  = master_prefix + "CDetector/";
  const std::string graph_prefix    = master_prefix + "Graph/";
  const std::string viz_prefix      = master_prefix + "Viz/";
  const std::string utility_prefix  = master_prefix + "Util/";
  const std::string planner_prefix  = master_prefix + "GPlanner/";
  const std::string contour_prefix  = master_prefix + "ContourGraph/";
  const std::string msger_prefix    = master_prefix + "GraphMsger/";

  // master params
  private_nh_.param<float>(master_prefix + "main_run_freq",         master_params_.main_run_freq, 5.0);
  private_nh_.param<float>(master_prefix + "contour_grid_resolution",
                           master_params_.contour_grid_resolution, 0.2f);
  private_nh_.param<float>(master_prefix + "robot_dim",             master_params_.robot_dim, 0.8);
  private_nh_.param<float>(master_prefix + "vehicle_height",        master_params_.vehicle_height, 0.75);
  private_nh_.param<float>(master_prefix + "sensor_range",          master_params_.sensor_range, 20.0);
  private_nh_.param<float>(master_prefix + "terrain_range",         master_params_.terrain_range, 15.0);
  private_nh_.param<float>(master_prefix + "local_planner_range",   master_params_.local_planner_range, 5.0);
  private_nh_.param<float>(master_prefix + "visualize_ratio",       master_params_.viz_ratio, 1.0);
  private_nh_.param<bool>(master_prefix  + "is_viewpoint_extend",   master_params_.is_viewpoint_extend, true);
  private_nh_.param<bool>(master_prefix  + "is_multi_layer",        master_params_.is_multi_layer, false);
  private_nh_.param<bool>(master_prefix  + "is_opencv_visual",      master_params_.is_visual_opencv, true);
  private_nh_.param<bool>(master_prefix  + "is_static_env",         master_params_.is_static_env, true);
  private_nh_.param<bool>(master_prefix  + "is_pub_boundary",       master_params_.is_pub_boundary, true);
  private_nh_.param<bool>(master_prefix  + "is_debug_output",       master_params_.is_debug_output, false);
  private_nh_.param<bool>(master_prefix  + "is_attempt_autoswitch", master_params_.is_attempt_autoswitch, true);
  private_nh_.param<float>(master_prefix + "odom_timeout",          master_params_.odom_timeout, 3.0f);
  private_nh_.param<float>(master_prefix + "semantic_map_timeout",  master_params_.semantic_map_timeout, 3.0f);
  private_nh_.param<bool>(master_prefix + "use_local_voxel_map",
                          master_params_.use_local_voxel_map, false);
  private_nh_.param<float>(master_prefix + "local_voxel_timeout",
                           master_params_.local_voxel_timeout, 0.5f);
  private_nh_.param<float>(master_prefix + "odom_connection_update_distance",
                           master_params_.odom_connection_update_distance,
                           0.2f);
  if (master_params_.odom_connection_update_distance <= 0.0f) {
    ROS_WARN("FARMaster: odom_connection_update_distance must be positive; using 0.2 m.");
    master_params_.odom_connection_update_distance = 0.2f;
  }
  private_nh_.param<bool>(master_prefix + "enable_goal_recording",
                          master_params_.enable_goal_recording, true);
  private_nh_.param<std::string>(master_prefix + "goal_record_file",
                                 master_params_.goal_record_file,
                                 std::string());
  private_nh_.param<std::string>(master_prefix + "world_frame",     master_params_.world_frame, "map");
  private_nh_.param<std::string>(master_prefix + "semantic_map_topic", master_params_.semantic_map_topic, "octomap_full");
  private_nh_.param<std::string>(master_prefix + "local_voxel_topic",
                                 master_params_.local_voxel_topic,
                                 "/local_3d_semantic_voxel_map/voxel_cloud");
  master_params_.terrain_range = std::min(master_params_.terrain_range, master_params_.sensor_range);

  // map handler params
  private_nh_.param<float>(map_prefix + "floor_height",        map_params_.floor_height, 2.0);
  private_nh_.param<float>(map_prefix + "semantic_local_window_radius",
                  map_params_.semantic_params.local_window_radius,
                  master_params_.sensor_range);
  private_nh_.param<float>(map_prefix + "terrain_search_radius",
                  map_params_.semantic_params.terrain_search_radius,
                  master_params_.robot_dim);
  private_nh_.param<float>(map_prefix + "terrain_neighbor_radius",
                  map_params_.semantic_params.terrain_neighbor_radius,
                  1.0f);
  private_nh_.param<float>(map_prefix + "local_planner_radius",
                  map_params_.semantic_params.local_planner_radius,
                  master_params_.local_planner_range);
  private_nh_.param<float>(map_prefix + "local_planner_resolution",
                  map_params_.semantic_params.local_planner_resolution,
                  0.2f);
  private_nh_.param<float>(map_prefix + "local_planner_obstacle_intensity",
                  map_params_.semantic_params.local_planner_obstacle_intensity,
                  200.0f);
  map_params_.semantic_params.use_local_voxel_map =
      master_params_.use_local_voxel_map;
  private_nh_.param<float>(map_prefix + "local_voxel_resolution",
                  map_params_.semantic_params.local_voxel_resolution,
                  0.10f);
  private_nh_.param<bool>(map_prefix + "semantic_top1_only",
                          map_params_.semantic_params.use_top1_only, true);
  private_nh_.param<float>(map_prefix + "semantic_min_probability",
                           map_params_.semantic_params.min_semantic_prob, 0.55f);
  const bool obstacle_groups_ok = ParseSemanticGroups(
      private_nh_, map_prefix + "obstacle_groups", map_params_.obstacle_groups);
  const bool terrain_groups_ok = ParseSemanticGroups(
      private_nh_, map_prefix + "terrain_support_groups", map_params_.terrain_support_groups);
  const bool dynamic_groups_ok = ParseSemanticGroups(
      private_nh_, map_prefix + "dynamic_obstacle_groups", map_params_.dynamic_obstacle_groups);
  if (!obstacle_groups_ok || !terrain_groups_ok || !dynamic_groups_ok) {
    ROS_FATAL("FARMaster: semantic class parameters are invalid; refusing to continue.");
    ros::shutdown();
  }
  private_nh_.param<float>(map_prefix + "local_voxel_min_semantic_confidence",
      local_voxel_policy_params_.minimum_semantic_confidence, 0.55f);
  private_nh_.param<float>(map_prefix + "local_voxel_obstacle_cost_threshold",
      local_voxel_policy_params_.obstacle_cost_threshold, 0.60f);
  const bool local_static_labels_ok = ParseLocalVoxelLabels(
      private_nh_, map_prefix + "local_voxel_static_labels",
      local_voxel_policy_params_.static_labels);
  const bool local_terrain_labels_ok = ParseLocalVoxelLabels(
      private_nh_, map_prefix + "local_voxel_terrain_labels",
      local_voxel_policy_params_.terrain_labels);
  const bool local_dynamic_labels_ok = ParseLocalVoxelLabels(
      private_nh_, map_prefix + "local_voxel_dynamic_labels",
      local_voxel_policy_params_.dynamic_labels);
  if (!local_static_labels_ok || !local_terrain_labels_ok ||
      !local_dynamic_labels_ok) {
    ROS_FATAL("FARMaster: local voxel semantic label parameters are invalid.");
    ros::shutdown();
  }
  local_voxel_policy_params_.minimum_semantic_confidence = std::max(
      0.0f, std::min(1.0f,
                     local_voxel_policy_params_.minimum_semantic_confidence));
  local_voxel_policy_params_.obstacle_cost_threshold = std::max(
      0.0f, std::min(1.0f,
                     local_voxel_policy_params_.obstacle_cost_threshold));
  map_params_.semantic_params.local_voxel_resolution = std::max(
      1e-3f, map_params_.semantic_params.local_voxel_resolution);
  map_params_.sensor_range     = master_params_.sensor_range;

  // utility params
  private_nh_.param<float>(utility_prefix + "angle_noise",            FARUtil::kAngleNoise, 15.0);
  private_nh_.param<float>(utility_prefix + "accept_max_align_angle", FARUtil::kAcceptAlign, 15.0);
  private_nh_.param<float>(utility_prefix + "new_intensity_thred",    FARUtil::kNewPIThred, 2.0);
  private_nh_.param<float>(utility_prefix + "robot_collision_clearance",
                           FARUtil::kNavClearDist, 0.45f);
  private_nh_.param<float>(utility_prefix + "visibility_edge_projection",
                           FARUtil::kProjectDist, 0.15f);
  private_nh_.param<float>(utility_prefix + "terrain_free_Z",         FARUtil::kFreeZ, 0.1);
  private_nh_.param<int>(utility_prefix   + "dyosb_update_thred",     FARUtil::kDyObsThred, 4);
  private_nh_.param<int>(utility_prefix   + "new_point_counter",      FARUtil::KNewPointC, 10);
  private_nh_.param<float>(utility_prefix + "dynamic_obs_dacay_time", FARUtil::kObsDecayTime, 10.0);
  private_nh_.param<float>(utility_prefix + "new_points_decay_time",  FARUtil::kNewDecayTime, 2.0);
  private_nh_.param<int>(utility_prefix   + "obs_inflate_size",       FARUtil::kObsInflate, 2);
  FARUtil::kLeafSize       = master_params_.contour_grid_resolution;
  FARUtil::kNearDist       = master_params_.robot_dim;
  FARUtil::kHeightVoxel    = master_params_.contour_grid_resolution * 2.0f;
  FARUtil::kMatchDist      = master_params_.robot_dim * 2.0f + FARUtil::kLeafSize;
  FARUtil::worldFrameId    = master_params_.world_frame;
  FARUtil::kVizRatio       = master_params_.viz_ratio;
  FARUtil::kTolerZ         = map_params_.floor_height - FARUtil::kHeightVoxel;
  FARUtil::kCellLength     = master_params_.contour_grid_resolution;
  FARUtil::kCellHeight     = map_params_.floor_height / 2.5f;
  FARUtil::kAcceptAlign    = FARUtil::kAcceptAlign / 180.0f * M_PI;
  FARUtil::kAngleNoise     = FARUtil::kAngleNoise  / 180.0f * M_PI; 
  FARUtil::robot_dim       = master_params_.robot_dim;
  FARUtil::IsStaticEnv     = master_params_.is_static_env;
  FARUtil::IsDebug         = master_params_.is_debug_output;
  FARUtil::IsMultiLayer    = master_params_.is_multi_layer;
  FARUtil::vehicle_height  = master_params_.vehicle_height;
  FARUtil::kSensorRange    = master_params_.sensor_range;
  FARUtil::kMarginDist     = master_params_.sensor_range - FARUtil::kMatchDist;
  FARUtil::kMarginHeight   = FARUtil::kTolerZ - FARUtil::kCellHeight / 2.0f;
  FARUtil::kTerrainRange   = master_params_.terrain_range;
  FARUtil::kLocalPlanRange = master_params_.local_planner_range;

  // graph planner params
  private_nh_.param<float>(planner_prefix + "converge_distance",    gp_params_.converge_dist, 1.0);
  private_nh_.param<float>(planner_prefix + "goal_adjust_radius",   gp_params_.adjust_radius, 10.0);
  private_nh_.param<int>(planner_prefix   + "free_counter_thred",   gp_params_.free_thred, 5);
  private_nh_.param<int>(planner_prefix   + "reach_goal_vote_size", gp_params_.votes_size, 5);
  private_nh_.param<int>(planner_prefix   + "path_momentum_thred",  gp_params_.momentum_thred, 5);
  gp_params_.momentum_dist = master_params_.robot_dim / 2.0f;
  gp_params_.is_autoswitch = master_params_.is_attempt_autoswitch;

  // contour graph params
  cg_params_.kPillarPerimeter = master_params_.robot_dim * 4.0f;
  private_nh_.param<float>(contour_prefix + "projection_min",
                           cg_params_.contour_projection_min, 0.15f);
  private_nh_.param<float>(contour_prefix + "projection_step",
                           cg_params_.contour_projection_step, 0.075f);
  private_nh_.param<float>(contour_prefix + "projection_max",
                           cg_params_.contour_projection_max, 0.60f);
  private_nh_.param<float>(contour_prefix + "boundary_guard",
                           cg_params_.contour_boundary_guard,
                           master_params_.contour_grid_resolution * 2.0f);

  // dynamic graph params
  private_nh_.param<int>(graph_prefix    + "connect_votes_size",        graph_params_.votes_size, 10);
  private_nh_.param<int>(graph_prefix    + "clear_dumper_thred",        graph_params_.dumper_thred, 3);
  private_nh_.param<int>(graph_prefix    + "node_finalize_thred",       graph_params_.finalize_thred, 3);
  private_nh_.param<int>(graph_prefix    + "filter_pool_size",          graph_params_.pool_size, 12);
  private_nh_.param<float>(graph_prefix  + "static_update_radius",
                           graph_params_.static_update_radius, 28.5f);
  private_nh_.param<float>(graph_prefix  + "static_stitch_radius",
                           graph_params_.static_stitch_radius, 28.5f);
  private_nh_.param<float>(graph_prefix  + "start_connection_max_distance",
                           graph_params_.start_connection_max_distance, -1.0f);
  private_nh_.param<float>(graph_prefix  + "dynamic_position_alpha",
                           graph_params_.dynamic_position_alpha, 0.65f);
  private_nh_.param<float>(graph_prefix  + "diagnostic_near_pair_radius",
                           graph_params_.diagnostic_near_pair_radius, 0.0f);
  private_nh_.param<int>(graph_prefix    + "static_confirm_frames",
                         graph_params_.static_confirm_frames, 3);
  private_nh_.param<int>(graph_prefix    + "static_remove_frames",
                         graph_params_.static_remove_frames, 3);
  private_nh_.param<int>(graph_prefix    + "static_topology_remove_frames",
                         graph_params_.static_topology_remove_frames, 5);
  private_nh_.param<int>(graph_prefix    + "static_visibility_remove_frames",
                         graph_params_.static_visibility_remove_frames, 3);
  private_nh_.param<float>(graph_prefix  + "static_duplicate_radius",
                           graph_params_.static_duplicate_radius, 0.5f);
  private_nh_.param<bool>(graph_prefix   + "static_promotion_requires_finalized",
                          graph_params_.static_promotion_requires_finalized, true);
  private_nh_.param<bool>(graph_prefix   + "static_promotion_requires_active_edge",
                          graph_params_.static_promotion_requires_active_edge, true);
  private_nh_.param<bool>(graph_prefix   + "static_promotion_requires_main_component",
                          graph_params_.static_promotion_requires_main_component,
                          true);
  private_nh_.param<bool>(graph_prefix   + "static_promotion_requires_global_evidence",
                          graph_params_.static_promotion_requires_global_evidence,
                          master_params_.use_local_voxel_map);
  private_nh_.param<float>(graph_prefix  + "connect_angle_thred",       graph_params_.kConnectAngleThred, 10.0);
  private_nh_.param<float>(graph_prefix  + "dirs_filter_margin",        graph_params_.filter_dirs_margin, 10.0);
  graph_params_.filter_pos_margin        = FARUtil::kNavClearDist;
  graph_params_.filter_dirs_margin       = FARUtil::kAngleNoise;
  graph_params_.kConnectAngleThred       = FARUtil::kAcceptAlign;
  graph_params_.frontier_perimeter_thred = FARUtil::kMatchDist * 4.0f;
  graph_params_.static_update_radius = std::max(0.1f, graph_params_.static_update_radius);
  graph_params_.static_stitch_radius = std::max(
      graph_params_.static_update_radius, graph_params_.static_stitch_radius);
  graph_params_.dynamic_position_alpha = std::max(
      0.0f, std::min(1.0f, graph_params_.dynamic_position_alpha));
  graph_params_.diagnostic_near_pair_radius = std::max(
      0.0f, graph_params_.diagnostic_near_pair_radius);
  graph_params_.static_confirm_frames = std::max(1, graph_params_.static_confirm_frames);
  graph_params_.static_remove_frames = std::max(1, graph_params_.static_remove_frames);
  graph_params_.static_topology_remove_frames = std::max(
      1, graph_params_.static_topology_remove_frames);
  graph_params_.static_visibility_remove_frames = std::max(
      1, graph_params_.static_visibility_remove_frames);
  graph_params_.static_duplicate_radius = std::max(
      0.0f, graph_params_.static_duplicate_radius);
  // The semantic query window and contour raster are axis-aligned squares.
  // Static Graph maintenance therefore needs the square's diagonal radius,
  // plus a small contour/voxel projection allowance at its boundary.  Using
  // sensor_range directly here would turn the requested square back into an
  // inscribed circle and discard its visible corner vertices.
  const float semantic_window_usable_radius =
      map_params_.semantic_params.local_window_radius * std::sqrt(2.0f) +
      FARUtil::kNavClearDist;
  if (semantic_window_usable_radius < graph_params_.static_stitch_radius) {
    ROS_WARN("FARMaster: semantic square-window usable radial coverage %.2f m is smaller than the %.2f m stitch radius; clamping stitch radius.",
             semantic_window_usable_radius, graph_params_.static_stitch_radius);
    graph_params_.static_stitch_radius =
        semantic_window_usable_radius;
    graph_params_.static_update_radius = std::min(
        graph_params_.static_update_radius, graph_params_.static_stitch_radius);
  }

  // graph messager params
  private_nh_.param<int>(msger_prefix + "robot_id", msger_parmas_.robot_id, 0);
  msger_parmas_.frame_id    = master_params_.world_frame;
  msger_parmas_.votes_size  = graph_params_.votes_size;
  msger_parmas_.pool_size   = graph_params_.pool_size;
  msger_parmas_.dist_margin = graph_params_.filter_pos_margin;

  // scan handler params
  scan_params_.terrain_range = master_params_.terrain_range;
  scan_params_.voxel_size    = master_params_.contour_grid_resolution;
  scan_params_.ceil_height   = map_params_.floor_height;

  // contour detector params
  private_nh_.param<float>(cdetect_prefix       + "resize_ratio",       cdetect_params_.kRatio, 5.0);
  private_nh_.param<int>(cdetect_prefix         + "filter_count_value", cdetect_params_.kThredValue, 5);
  private_nh_.param<float>(cdetect_prefix       + "dynamic_simplify_ratio", cdetect_params_.dynamic_simplify_ratio, 2.0f);
  private_nh_.param<float>(cdetect_prefix       + "collinear_tolerance", cdetect_params_.collinear_tolerance, 0.20f);
  private_nh_.param<float>(cdetect_prefix       + "collinear_angle_deg", cdetect_params_.collinear_angle_deg, 8.0f);
  private_nh_.param<bool>(cdetect_prefix        + "is_save_img",        cdetect_params_.is_save_img, false);
  private_nh_.param<std::string>(cdetect_prefix + "img_folder_path",    cdetect_params_.img_path, "");
  cdetect_params_.kBlurSize    = (int)std::round(
      FARUtil::kNavClearDist / master_params_.contour_grid_resolution);
  cdetect_params_.dynamic_simplify_ratio =
      std::max(1.0f, cdetect_params_.dynamic_simplify_ratio);
  cdetect_params_.collinear_tolerance =
      std::max(0.0f, cdetect_params_.collinear_tolerance);
  cdetect_params_.collinear_angle_deg = std::max(
      0.0f, std::min(30.0f, cdetect_params_.collinear_angle_deg));
  cdetect_params_.sensor_range = master_params_.sensor_range;
  cdetect_params_.contour_grid_resolution =
      master_params_.contour_grid_resolution;
}

// 初始时刻，如果odom frame和world frame不一致，则需要进行坐标变换，将odom frame下的位姿转换到world frame下，并将此位置作为初始位置
void FARMaster::OdomCallBack(const nav_msgs::OdometryConstPtr& msg) {
  // transform from odom frame to mapping frame
  std::string odom_frame = msg->header.frame_id;
  tf::Pose tf_odom_pose;
  tf::poseMsgToTF(msg->pose.pose, tf_odom_pose);
  if (!FARUtil::IsSameFrameID(odom_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) ROS_WARN_ONCE("FARMaster: odom frame does NOT match with world frame!");
    tf::StampedTransform odom_to_world_tf_stamp;
    try
    {
      tf_listener_->waitForTransform(master_params_.world_frame, odom_frame, ros::Time(0), ros::Duration(1.0));
      tf_listener_->lookupTransform(master_params_.world_frame, odom_frame, ros::Time(0), odom_to_world_tf_stamp);
      tf_odom_pose = odom_to_world_tf_stamp * tf_odom_pose;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Tracking odom TF lookup: %s",ex.what());
      return;
    }
  }
  robot_pos_.x = tf_odom_pose.getOrigin().getX(); 
  robot_pos_.y = tf_odom_pose.getOrigin().getY();
  robot_pos_.z = tf_odom_pose.getOrigin().getZ();
  // extract robot heading
  FARUtil::robot_pos = robot_pos_;
  double roll, pitch, yaw;
  tf_odom_pose.getBasis().getRPY(roll, pitch, yaw);
  robot_heading_ = Point3D(cos(yaw), sin(yaw), 0);
  last_odom_receipt_ = ros::WallTime::now();
  last_odom_stamp_ = msg->header.stamp;
  map_handler_.SetMapOrigin(robot_pos_);

  if (!is_odom_init_) {
    // system start time
    FARUtil::systemStartTime = ros::Time::now().toSec();
    FARUtil::map_origin = robot_pos_;
    if (map_handler_.HasSemanticMap()) {
      map_handler_.UpdateRobotPosition(robot_pos_);
    }
    if (master_params_.use_local_voxel_map) {
      if (has_local_voxel_snapshot_) {
        this->UpdatePlannerCloudsFromLocalVoxelMap();
      }
    } else if (map_handler_.HasSemanticMap()) {
      this->UpdatePlannerCloudsFromSemanticMap();
    }
  }

  is_odom_init_ = true;
}

void FARMaster::SemanticMapCallBack(const octomap_msgs::OctomapConstPtr& msg) {
  if (!msg) return;
  if (master_params_.use_local_voxel_map) {
    if (msg->header.stamp.isZero()) {
      ROS_ERROR_THROTTLE(
          2.0, "FARMaster: confirmed-global octomap has a zero stamp.");
      return;
    }
    if (!last_semantic_map_stamp_.isZero() &&
        msg->header.stamp <= last_semantic_map_stamp_) {
      if (msg->header.stamp < last_semantic_map_stamp_) {
        ROS_WARN_THROTTLE(
            2.0, "FARMaster: dropping out-of-order confirmed-global octomap "
                 "(%.6f < %.6f).",
            msg->header.stamp.toSec(), last_semantic_map_stamp_.toSec());
      }
      // Repetition of a latched global tree is not a new confirmation event
      // and must not keep the promotion-evidence watchdog alive.
      return;
    }
  }

  const ros::WallTime callback_start = ros::WallTime::now();
  const ros::WallTime snapshot_start = ros::WallTime::now();
  if (!map_handler_.SetSemanticOctomap(msg)) {
    PublishSeconds(semantic_callback_time_pub_,
                   ros::WallTime::now() - callback_start);
    return;
  }
  PublishSeconds(semantic_snapshot_time_pub_,
                 ros::WallTime::now() - snapshot_start);
  last_semantic_map_receipt_ = ros::WallTime::now();
  last_semantic_map_stamp_ = msg->header.stamp;
  if (!is_odom_init_) {
    PublishSeconds(semantic_callback_time_pub_,
                   ros::WallTime::now() - callback_start);
    return;
  }

  if (master_params_.use_local_voxel_map) {
    // The global tree may refresh static evidence and persistent collision
    // memory, but it never owns current contours. The next fresh local voxel
    // snapshot atomically rebuilds the graph against both layers.
    planning_requested_ = true;
    PublishSeconds(semantic_callback_time_pub_,
                   ros::WallTime::now() - callback_start);
    return;
  }

  const ros::WallTime planner_update_start = ros::WallTime::now();
  this->UpdatePlannerCloudsFromSemanticMap();
  PublishSeconds(semantic_update_time_pub_,
                 ros::WallTime::now() - planner_update_start);
  PublishSeconds(semantic_callback_time_pub_,
                 ros::WallTime::now() - callback_start);
}

void FARMaster::LocalVoxelMapCallBack(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (!master_params_.use_local_voxel_map || !msg) return;
  if (msg->is_bigendian) {
    ROS_ERROR_THROTTLE(
        2.0, "FARMaster: big-endian local voxel PointCloud2 is unsupported.");
    return;
  }
  if (msg->header.stamp.isZero()) {
    ROS_ERROR_THROTTLE(
        2.0, "FARMaster: local voxel snapshot has a zero acquisition stamp.");
    return;
  }
  if (!last_local_voxel_stamp_.isZero() &&
      msg->header.stamp <= last_local_voxel_stamp_) {
    if (msg->header.stamp < last_local_voxel_stamp_) {
      ROS_WARN_THROTTLE(
          2.0, "FARMaster: dropping out-of-order local voxel snapshot "
               "(%.6f < %.6f).",
          msg->header.stamp.toSec(), last_local_voxel_stamp_.toSec());
    }
    // A latched/timer publication of the same sensor snapshot must not
    // refresh wall-time freshness or increment Graph observation counters.
    return;
  }

  const PointCloudFieldView x_field = FindPointCloudField(*msg, "x");
  const PointCloudFieldView y_field = FindPointCloudField(*msg, "y");
  const PointCloudFieldView z_field = FindPointCloudField(*msg, "z");
  const PointCloudFieldView label_field = FindPointCloudField(*msg, "label");
  const PointCloudFieldView confidence_field =
      FindPointCloudField(*msg, "semantic_confidence");
  PointCloudFieldView cost_field =
      FindPointCloudField(*msg, "traversability");
  if (!cost_field.valid) cost_field = FindPointCloudField(*msg, "intensity");
  if (!x_field.valid || !y_field.valid || !z_field.valid) {
    ROS_ERROR_THROTTLE(
        2.0, "FARMaster: local voxel snapshot requires x/y/z fields.");
    return;
  }

  tf::StampedTransform cloud_to_world;
  const bool transform_required = msg->header.frame_id.empty() ||
      !FARUtil::IsSameFrameID(
          msg->header.frame_id, master_params_.world_frame);
  if (msg->header.frame_id.empty()) {
    ROS_ERROR_THROTTLE(
        2.0, "FARMaster: local voxel snapshot has an empty frame_id.");
    return;
  }
  if (transform_required) {
    try {
      tf_listener_->waitForTransform(
          master_params_.world_frame, msg->header.frame_id,
          msg->header.stamp, ros::Duration(0.2));
      tf_listener_->lookupTransform(
          master_params_.world_frame, msg->header.frame_id,
          msg->header.stamp, cloud_to_world);
    } catch (const tf::TransformException& ex) {
      ROS_WARN_THROTTLE(
          2.0, "FARMaster: local voxel TF lookup failed: %s", ex.what());
      return;
    }
  }

  PointCloudPtr static_obstacles(new PointCloud());
  PointCloudPtr transient_obstacles(new PointCloud());
  PointCloudPtr terrain_support(new PointCloud());
  const size_t point_count =
      static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  static_obstacles->reserve(point_count / 3 + 1);
  transient_obstacles->reserve(point_count / 6 + 1);
  terrain_support->reserve(point_count / 2 + 1);
  size_t invalid_points = 0;

  for (uint32_t row = 0; row < msg->height; ++row) {
    for (uint32_t column = 0; column < msg->width; ++column) {
      const size_t offset = static_cast<size_t>(row) * msg->row_step +
                            static_cast<size_t>(column) * msg->point_step;
      if (offset + msg->point_step > msg->data.size()) {
        ROS_ERROR_THROTTLE(
            2.0, "FARMaster: malformed local voxel PointCloud2 buffer.");
        return;
      }
      const uint8_t* raw = msg->data.data() + offset;
      double x = 0.0, y = 0.0, z = 0.0;
      if (!ReadPointCloudNumber(raw, x_field, x) ||
          !ReadPointCloudNumber(raw, y_field, y) ||
          !ReadPointCloudNumber(raw, z_field, z) ||
          !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++invalid_points;
        continue;
      }

      double label_value = 0.0;
      const bool has_label = ReadPointCloudNumber(
          raw, label_field, label_value) && std::isfinite(label_value) &&
          label_value >= 0.0 &&
          label_value <= std::numeric_limits<uint32_t>::max();
      const uint32_t label = has_label
          ? static_cast<uint32_t>(label_value) : 0u;
      double confidence_value = 1.0;
      if (confidence_field.valid &&
          (!ReadPointCloudNumber(raw, confidence_field, confidence_value) ||
           !std::isfinite(confidence_value))) {
        confidence_value = 0.0;
      }
      double cost_value = 0.0;
      const bool has_cost = ReadPointCloudNumber(
          raw, cost_field, cost_value) && std::isfinite(cost_value);
      const LocalVoxelLayer layer = ClassifyLocalVoxel(
          label, has_label, static_cast<float>(confidence_value), has_cost,
          static_cast<float>(cost_value), local_voxel_policy_params_);
      if (layer == LocalVoxelLayer::IGNORE) continue;

      tf::Vector3 position(x, y, z);
      if (transform_required) position = cloud_to_world * position;
      PCLPoint point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = has_cost ? static_cast<float>(cost_value) : 0.0f;
      if (layer == LocalVoxelLayer::STATIC_OBSTACLE) {
        static_obstacles->push_back(point);
      } else if (layer == LocalVoxelLayer::TRANSIENT_OBSTACLE) {
        transient_obstacles->push_back(point);
      } else if (layer == LocalVoxelLayer::TERRAIN_SUPPORT) {
        terrain_support->push_back(point);
      }
    }
  }
  FinalizePointCloud(static_obstacles);
  FinalizePointCloud(transient_obstacles);
  FinalizePointCloud(terrain_support);

  map_handler_.SetLocalVoxelSnapshot(
      static_obstacles, transient_obstacles, terrain_support);
  last_local_voxel_stamp_ = msg->header.stamp;
  last_local_voxel_receipt_ = ros::WallTime::now();
  has_local_voxel_snapshot_ = true;
  if (is_odom_init_) this->UpdatePlannerCloudsFromLocalVoxelMap();
  ROS_INFO_THROTTLE(
      2.0, "FAR local voxel snapshot: static=%zu transient=%zu terrain=%zu "
           "invalid=%zu stamp=%.6f",
      static_obstacles->size(), transient_obstacles->size(),
      terrain_support->size(), invalid_points, msg->header.stamp.toSec());
}

void FARMaster::UpdatePlannerCloudsFromSemanticMap() {
  // The original FAR incremental pipeline now sees the effective static plus
  // dynamic collision view. Dynamic add/remove events enter the changed-point
  // KD-tree, so their contour vertices and affected connections are rebuilt
  // locally instead of becoming permanent global-map structure.
  this->UpdatePlannerCloudsFromCurrentLayers();
}

void FARMaster::UpdatePlannerCloudsFromLocalVoxelMap() {
  this->UpdatePlannerCloudsFromCurrentLayers();
}

void FARMaster::UpdatePlannerCloudsFromCurrentLayers() {
  map_handler_.GetSurroundObsCloud(FARUtil::surround_obs_cloud_);
  map_handler_.GetCurrentStaticObsCloud(current_static_obs_ptr_);
  map_handler_.GetPersistentStaticObsCloud(persistent_static_obs_ptr_);
  map_handler_.GetCollisionObsCloud(collision_obs_ptr_);
  map_handler_.GetEffectiveDynamicObsCloud(effective_dynamic_obs_ptr_);
  *graph_static_collision_obs_ptr_ = *persistent_static_obs_ptr_;
  if (master_params_.use_local_voxel_map && current_static_obs_ptr_) {
    *graph_static_collision_obs_ptr_ += *current_static_obs_ptr_;
  }
  FinalizePointCloud(graph_static_collision_obs_ptr_);
  ContourGraph::SetLocalCollisionCloud(graph_static_collision_obs_ptr_,
                                       effective_dynamic_obs_ptr_);
  map_handler_.GetDynamicAddedCloud(dynamic_added_ptr_);
  map_handler_.GetDynamicRemovedCloud(dynamic_removed_ptr_);
  map_handler_.GetLocalPlannerStaticObsCloud(local_planner_static_obs_ptr_);
  map_handler_.GetLocalPlannerDynamicObsCloud(local_planner_dynamic_obs_ptr_);
  semantic_graph_dirty_ = true;
  odom_connections_dirty_ = true;
  planning_requested_ = true;
  if (is_stop_update_) {
    FARUtil::cur_new_cloud_->clear();
  } else {
    map_handler_.GetChangedObsCloud(FARUtil::cur_new_cloud_);
  }
  FARUtil::StackCloudByTime(FARUtil::cur_new_cloud_,
                            FARUtil::stack_new_cloud_,
                            FARUtil::kNewDecayTime);
  FARUtil::UpdateKdTrees(FARUtil::stack_new_cloud_);

  // A valid current-obstacle snapshot is sufficient to start FAR, including
  // a valid empty-obstacle scene.
  is_cloud_init_ = true;

  planner_viz_.VizPointCloud(new_PCL_pub_, FARUtil::stack_new_cloud_);
  planner_viz_.VizPointCloud(dynamic_obs_pub_, effective_dynamic_obs_ptr_);
  planner_viz_.VizPointCloud(surround_obs_debug_, FARUtil::surround_obs_cloud_);
  // Keep static semantics and the latest-snapshot dynamic layer on separate
  // topics. The trajectory planner clears the latter immediately when an
  // empty dynamic cloud arrives without erasing static geometry.
  planner_viz_.VizPointCloud(local_planner_static_obs_pub_,
                             local_planner_static_obs_ptr_);
  planner_viz_.VizPointCloud(local_planner_dynamic_obs_pub_,
                             local_planner_dynamic_obs_ptr_);
}

// 得到动态点云
void FARMaster::ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                          const PointCloudPtr& obsCloudIn,
                                          const PointCloudPtr& freeCloudIn,
                                          const PointCloudPtr& dyObsCloudOut)
{
  scan_handler_.ReInitGrids();
  scan_handler_.SetCurrentScanCloud(scanCloudIn, freeCloudIn);
  scan_handler_.ExtractDyObsCloud(obsCloudIn, dyObsCloudOut);
}

// 终点回调函数，接收来自上层规划器的目标点，更新图规划器的目标点，并可视化原始目标点
// 接收外部发来的目标点话题 /goal_point
// 把目标点从目标坐标系转换到 world_frame（如果 frame 不一致）
// 调用 graph_planner_.UpdateGoal(goal_p) 更新全局目标
// 可视化原始目标点并启动整体计时
void FARMaster::WaypointCallBack(const geometry_msgs::PointStamped& route_goal) {
  Point3D goal_p(route_goal.point.x, route_goal.point.y, route_goal.point.z);
  const std::string goal_frame = route_goal.header.frame_id;
  if (!goal_frame.empty() &&
      !FARUtil::IsSameFrameID(goal_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) ROS_WARN_THROTTLE(1.0, "FARMaster: waypoint published is not on world frame!");
    FARUtil::TransformPoint3DFrame(goal_frame, master_params_.world_frame,
                                   tf_listener_, goal_p);
  }
  this->RecordGoalSelection(route_goal, goal_p);
  if (!is_graph_init_) {
    pending_route_goal_ = route_goal;
    pending_route_goal_.header.frame_id = master_params_.world_frame;
    pending_route_goal_.point = FARUtil::Point3DToGeoMsgPoint(goal_p);
    has_pending_route_goal_ = true;
    planning_requested_ = true;
    odom_connections_dirty_ = true;
    ROS_WARN_THROTTLE(1.0,
        "FARMaster: V-Graph is not initialized; retaining the latest goal for automatic application.");
    return;
  }
  this->ApplyWorldGoal(goal_p);
}

void FARMaster::ApplyWorldGoal(const Point3D& goal_p) {
  commanded_goal_ = goal_p;
  has_commanded_goal_ = true;
  graph_planner_.UpdateGoal(goal_p);
  // A new destination starts a new search snapshot. Revalidate start edges
  // first, then plan; no previous path or waypoint is reused.
  odom_connections_dirty_ = true;
  planning_requested_ = true;
  ROS_INFO("FARMaster: accepted goal in %s at (%.2f, %.2f, %.2f).",
           master_params_.world_frame.c_str(), goal_p.x, goal_p.y, goal_p.z);
  FARUtil::Timer.start_time("Overall_executing", true);
  // visualize original goal
  planner_viz_.VizPoint3D(goal_p, "original_goal", VizColor::RED, 1.5);
}

void FARMaster::InitializeGoalRecorder() {
  if (!master_params_.enable_goal_recording) return;
  if (master_params_.goal_record_file.empty()) {
    const std::string package_path = ros::package::getPath("far_planner");
    master_params_.goal_record_file =
        package_path.empty()
            ? std::string("goal_selections.csv")
            : package_path + "/logs/goal_selections.csv";
  }
  goal_record_session_id_ = LocalTimestampNow();
  goal_record_session_sequence_ = 0;
  goal_record_sequence_ = 0;

  std::vector<std::string> existing_lines;
  std::ifstream existing(master_params_.goal_record_file.c_str());
  std::string line;
  while (std::getline(existing, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    existing_lines.push_back(line);
  }
  existing.close();

  bool write_header = existing_lines.empty();
  if (!existing_lines.empty()) {
    const bool current_schema = existing_lines.front() == GoalRecordHeader();
    const bool legacy_schema =
        existing_lines.front() == GoalRecordLegacyHeader();
    if (!current_schema && !legacy_schema) {
      ROS_ERROR("FARMaster: goal record file has an unknown CSV schema; "
                "refusing to append: %s",
                master_params_.goal_record_file.c_str());
      return;
    }

    for (std::size_t i = 1; i < existing_lines.size(); ++i) {
      std::uint64_t sequence = 0;
      if (ParseGoalRecordSequence(existing_lines[i], &sequence)) {
        goal_record_sequence_ = std::max(goal_record_sequence_, sequence);
      }
    }

    if (legacy_schema) {
      const std::string temporary_path =
          master_params_.goal_record_file + ".schema.tmp";
      std::ofstream migrated(temporary_path.c_str(),
                             std::ios::out | std::ios::trunc);
      if (!migrated.is_open()) {
        ROS_ERROR("FARMaster: cannot create migrated goal record: %s",
                  temporary_path.c_str());
        return;
      }
      migrated << GoalRecordHeader() << '\n';
      for (std::size_t i = 1; i < existing_lines.size(); ++i) {
        if (existing_lines[i].empty()) continue;
        migrated << existing_lines[i];
        for (std::size_t column = 0; column < kGoalRecordAddedColumns;
             ++column) {
          migrated << ',';
        }
        migrated << '\n';
      }
      migrated.close();
      if (!migrated.good() ||
          std::rename(temporary_path.c_str(),
                      master_params_.goal_record_file.c_str()) != 0) {
        ROS_ERROR("FARMaster: failed to migrate goal record schema: %s",
                  master_params_.goal_record_file.c_str());
        return;
      }
      ROS_INFO("FARMaster: migrated the goal record to the extended schema.");
    }
  }

  goal_record_stream_.open(master_params_.goal_record_file.c_str(),
                           std::ios::out | std::ios::app);
  if (!goal_record_stream_.is_open()) {
    ROS_ERROR("FARMaster: cannot open goal record file: %s",
              master_params_.goal_record_file.c_str());
    return;
  }
  if (write_header) {
    goal_record_stream_ << GoalRecordHeader() << '\n';
    goal_record_stream_.flush();
  }
  ROS_INFO("FARMaster: goal selections will be recorded in %s "
           "(session=%s, next sequence=%llu).",
           master_params_.goal_record_file.c_str(),
           goal_record_session_id_.c_str(),
           static_cast<unsigned long long>(goal_record_sequence_ + 1));
}

void FARMaster::RecordGoalSelection(
    const geometry_msgs::PointStamped& route_goal,
    const Point3D& world_goal) {
  if (!master_params_.enable_goal_recording ||
      !goal_record_stream_.is_open()) {
    return;
  }
  ++goal_record_sequence_;
  ++goal_record_session_sequence_;
  const ros::WallTime wall_now = ros::WallTime::now();
  const double odom_age = last_odom_receipt_.isZero()
      ? -1.0 : (wall_now - last_odom_receipt_).toSec();
  const double semantic_map_age = last_semantic_map_receipt_.isZero()
      ? -1.0 : (wall_now - last_semantic_map_receipt_).toSec();
  const std::size_t nav_graph_nodes = nav_graph_.size();
  const std::size_t static_global_nodes =
      graph_manager_.GetStaticGraphNodes().size();
  const std::size_t static_candidate_nodes =
      graph_manager_.GetStaticCandidateNodes().size();
  const std::size_t dynamic_local_nodes =
      graph_manager_.GetDynamicLocalNodes().size();
  const std::size_t current_static_points = current_static_obs_ptr_
      ? current_static_obs_ptr_->size() : 0;
  const std::size_t current_dynamic_points = effective_dynamic_obs_ptr_
      ? effective_dynamic_obs_ptr_->size() : 0;
  const bool previous_waypoint_valid = is_planner_running_;
  const geometry_msgs::Point& previous_waypoint = goal_waypoint_stamped_.point;
  goal_record_stream_ << goal_record_sequence_ << ','
      << LocalTimestampNow() << ',' << std::fixed << std::setprecision(9)
      << ros::Time::now().toSec() << ',' << route_goal.header.stamp.toSec()
      << ',' << route_goal.header.frame_id << ','
      << master_params_.world_frame << ',' << (is_odom_init_ ? 1 : 0)
      << ',' << (is_graph_init_ ? 1 : 0) << ',' << std::setprecision(4)
      << robot_pos_.x << ',' << robot_pos_.y << ',' << robot_pos_.z << ','
      << route_goal.point.x << ',' << route_goal.point.y << ','
      << route_goal.point.z << ',' << world_goal.x << ',' << world_goal.y
      << ',' << world_goal.z << ',' << goal_record_session_id_ << ','
      << goal_record_session_sequence_ << ',' << std::setprecision(9)
      << last_odom_stamp_.toSec() << ',' << last_semantic_map_stamp_.toSec()
      << ',' << odom_age << ',' << semantic_map_age << ','
      << std::setprecision(4) << robot_heading_.x << ',' << robot_heading_.y
      << ',' << robot_heading_.z << ',' << nav_graph_nodes << ','
      << static_global_nodes << ',' << static_candidate_nodes << ','
      << dynamic_local_nodes << ',' << current_static_points << ','
      << current_dynamic_points << ',' << (semantic_graph_dirty_ ? 1 : 0)
      << ',' << (is_planner_running_ ? 1 : 0) << ','
      << (previous_waypoint_valid ? 1 : 0) << ',' << previous_waypoint.x
      << ',' << previous_waypoint.y << ',' << previous_waypoint.z << '\n';
  goal_record_stream_.flush();
}

/* allocate static utility PointCloud pointer memory */

// FARUtil 里都是静态变量，机器人通过这些静态变量来共享点云数据和参数设置
// 但是还需要查看这些变量相互之间有没有竞争关系

PointCloudPtr  FARUtil::surround_obs_cloud_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_new_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_new_cloud_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_dyobs_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_dyobs_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_scan_cloud_      = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_new_cloud_    = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_filter_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
/* init static utility values */
const float FARUtil::kEpsilon = 1e-7;
const float FARUtil::kINF     = std::numeric_limits<float>::max();
std::string FARUtil::worldFrameId;
float   FARUtil::kAngleNoise; 
Point3D FARUtil::robot_pos;
Point3D FARUtil::odom_pos;
Point3D FARUtil::map_origin;
Point3D FARUtil::free_odom_p;
float   FARUtil::robot_dim;
float   FARUtil::vehicle_height;
float   FARUtil::kLeafSize;
float   FARUtil::kHeightVoxel;
float   FARUtil::kNavClearDist;
float   FARUtil::kCellLength;
float   FARUtil::kCellHeight;
float   FARUtil::kNewPIThred;
float   FARUtil::kSensorRange;
float   FARUtil::kMarginDist;
float   FARUtil::kMarginHeight;
float   FARUtil::kTerrainRange;
float   FARUtil::kLocalPlanRange;
float   FARUtil::kFreeZ;
float   FARUtil::kVizRatio;
double  FARUtil::systemStartTime;
float   FARUtil::kObsDecayTime;
float   FARUtil::kNewDecayTime;
float   FARUtil::kNearDist;
float   FARUtil::kMatchDist;
float   FARUtil::kProjectDist;
int     FARUtil::kDyObsThred;
int     FARUtil::KNewPointC;
int     FARUtil::kObsInflate;
float   FARUtil::kTolerZ;
float   FARUtil::kAcceptAlign;
bool    FARUtil::IsStaticEnv;
bool    FARUtil::IsDebug;
bool    FARUtil::IsMultiLayer;
TimeMeasure FARUtil::Timer;

/* Global Graph */
DynamicGraphParams DynamicGraph::dg_params_;
NodePtrStack DynamicGraph::globalGraphNodes_;
NodePtrStack DynamicGraph::staticCandidateGraphNodes_;
NodePtrStack DynamicGraph::dynamicLocalGraphNodes_;
std::unordered_set<std::size_t> DynamicGraph::staticMainNodeIds_;
std::size_t  DynamicGraph::id_tracker_;
std::unordered_map<std::size_t, NavNodePtr> DynamicGraph::idx_node_map_;
std::unordered_map<NavNodePtr, std::pair<int, std::unordered_set<NavNodePtr>>> DynamicGraph::out_contour_nodes_map_;
std::vector<EdgeDiagnostic> DynamicGraph::contour_edge_diagnostics_;

/* init static contour graph values */
CTNodeStack ContourGraph::polys_ctnodes_;
CTNodeStack ContourGraph::contour_graph_;
PolygonStack ContourGraph::contour_polygons_;
std::vector<PointPair> ContourGraph::global_contour_;
std::vector<PointPair> ContourGraph::unmatched_contour_;
std::vector<PointPair> ContourGraph::inactive_contour_;
std::vector<PointPair> ContourGraph::boundary_contour_;
std::vector<PointPair> ContourGraph::local_boundary_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::global_contour_set_;
std::unordered_set<NavEdge, navedge_hash> ContourGraph::boundary_contour_set_;

/* init terrain map values */
PointKdTreePtr MapHandler::kdtree_terrain_clould_;


int main(int argc, char** argv){
  ros::init(argc, argv, "far_planner_node");
  FARMaster dp_node;
  dp_node.Init();
  dp_node.Loop();
}
