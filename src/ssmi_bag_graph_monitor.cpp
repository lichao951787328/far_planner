#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap_msgs/Octomap.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

using Point = geometry_msgs::Point;
using MarkerArray = visualization_msgs::MarkerArray;

double Distance3D(const Point& first, const Point& second) {
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double PointSegmentDistance(const Point& point, const Point& start,
                            const Point& end) {
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double dz = end.z - start.z;
  const double length_squared = dx * dx + dy * dy + dz * dz;
  if (length_squared <= 1e-12) return Distance3D(point, start);
  double ratio = ((point.x - start.x) * dx +
                  (point.y - start.y) * dy +
                  (point.z - start.z) * dz) / length_squared;
  ratio = std::max(0.0, std::min(1.0, ratio));
  Point projection;
  projection.x = start.x + ratio * dx;
  projection.y = start.y + ratio * dy;
  projection.z = start.z + ratio * dz;
  return Distance3D(point, projection);
}

double NormalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::string NormalizeFrame(const std::string& frame) {
  if (!frame.empty() && frame.front() == '/') return frame.substr(1);
  return frame;
}

bool SameFrame(const std::string& first, const std::string& second) {
  return NormalizeFrame(first) == NormalizeFrame(second);
}

bool HasPointCloudField(const sensor_msgs::PointCloud2& cloud,
                        const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name && field.count > 0) return true;
  }
  return false;
}

struct QuantizedPoint {
  long long x = 0;
  long long y = 0;
  long long z = 0;

  bool operator<(const QuantizedPoint& other) const {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
  }
};

QuantizedPoint Quantize(const Point& point, double resolution = 1e-3) {
  QuantizedPoint key;
  key.x = std::llround(point.x / resolution);
  key.y = std::llround(point.y / resolution);
  key.z = std::llround(point.z / resolution);
  return key;
}

uint64_t VoxelKey(float x, float y, float z, float resolution) {
  const int64_t ix = static_cast<int64_t>(std::floor(x / resolution));
  const int64_t iy = static_cast<int64_t>(std::floor(y / resolution));
  const int64_t iz = static_cast<int64_t>(std::floor(z / resolution));
  constexpr uint64_t mask = (1ULL << 21) - 1ULL;
  return ((static_cast<uint64_t>(ix) & mask) << 42) |
         ((static_cast<uint64_t>(iy) & mask) << 21) |
         (static_cast<uint64_t>(iz) & mask);
}

}  // namespace

class SsmiBagGraphMonitor {
 public:
  SsmiBagGraphMonitor()
      : private_nh_("~"), static_cloud_(new pcl::PointCloud<pcl::PointXYZ>()),
        static_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    private_nh_.param<std::string>("expected_frame", expected_frame_,
                                   "map_start");
    private_nh_.param<std::string>("source_odom_topic", source_odom_topic_,
                                   "/fusion_localization");
    private_nh_.param<std::string>(
        "local_voxel_topic", local_voxel_topic_,
        "/local_3d_semantic_voxel_map/voxel_cloud");
    private_nh_.param<std::string>("output_csv", output_csv_, "");
    private_nh_.param("auto_goal", auto_goal_, false);
    private_nh_.param("goal_x", goal_.x, 7.98);
    private_nh_.param("goal_y", goal_.y, 0.24);
    private_nh_.param("goal_z", goal_.z, 0.0);
    private_nh_.param("goal_publish_delay", goal_publish_delay_, 5.0);
    private_nh_.param("finish_on_bag_idle", finish_on_bag_idle_, true);
    private_nh_.param("bag_idle_timeout", bag_idle_timeout_, 5.0);
    private_nh_.param("position_tolerance", position_tolerance_, 0.02);
    private_nh_.param("yaw_tolerance_deg", yaw_tolerance_deg_, 1.0);
    private_nh_.param("collision_clearance", collision_clearance_, 0.45);
    private_nh_.param("collision_sample_step", collision_sample_step_, 0.10);
    private_nh_.param("collision_check_radius", collision_check_radius_, 10.0);
    private_nh_.param("visibility_endpoint_exclusion",
                      visibility_endpoint_exclusion_, 0.875);
    private_nh_.param("static_accumulation_resolution",
                      static_accumulation_resolution_, 0.20);
    private_nh_.param("static_component_failure_frames",
                      static_component_failure_frames_, 5);
    static_component_failure_frames_ =
        std::max(1, static_component_failure_frames_);

    if (output_csv_.empty()) {
      const std::string package = ros::package::getPath("far_planner");
      output_csv_ = package.empty()
                        ? "ssmi_bag_graph_test.csv"
                        : package + "/logs/ssmi_bag_graph_test.csv";
    }
    csv_.open(output_csv_.c_str(), std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      ROS_ERROR("SSMI monitor cannot open CSV: %s", output_csv_.c_str());
    } else {
      csv_ << "wall_elapsed_s,ros_time,map_stamp,odom_stamp,"
              "local_voxel_stamp,map_frame,source_odom_frame,"
              "local_voxel_frame,local_voxel_points,"
              "local_voxel_acquisitions,local_voxel_schema_valid,"
              "graph_frame,goal_frame,path_frame,"
              "initial_position_error_m,initial_yaw_error_deg,"
              "static_points,dynamic_points,graph_static_points,"
              "accumulated_static_points,"
              "graph_nodes,graph_edges,graph_components,largest_component,"
              "static_graph_nodes,static_graph_edges,static_graph_components,"
              "static_graph_largest,static_graph_invalid_frames,"
              "robot_component,robot_degree,blocked_edges,path_poses,"
              "path_length_m,path_goal_error_m,path_min_clearance_m,"
              "graph_edges_checked,graph_min_clearance_m,"
              "graph_clearance_violations,robot_static_clearance_m,"
              "waypoint_path_distance_m,reached,"
              "semantic_snapshot_s,semantic_update_s,semantic_callback_s,"
              "main_loop_s\n";
    }

    odom_sub_ = nh_.subscribe(source_odom_topic_, 20,
                              &SsmiBagGraphMonitor::OdomCallback, this);
    map_sub_ = nh_.subscribe("/octomap_full", 1,
                             &SsmiBagGraphMonitor::MapCallback, this);
    local_voxel_sub_ = nh_.subscribe(
        local_voxel_topic_, 2,
        &SsmiBagGraphMonitor::LocalVoxelCallback, this);
    static_cloud_sub_ = nh_.subscribe(
        "/semantic_local_static_obstacles", 2,
        &SsmiBagGraphMonitor::StaticCloudCallback, this);
    graph_static_cloud_sub_ = nh_.subscribe(
        "/semantic_graph_static_obstacles", 2,
        &SsmiBagGraphMonitor::GraphStaticCloudCallback, this);
    dynamic_cloud_sub_ = nh_.subscribe(
        "/semantic_local_dynamic_obstacles", 2,
        &SsmiBagGraphMonitor::DynamicCloudCallback, this);
    search_graph_sub_ = nh_.subscribe(
        "/viz_current_search_graph", 2,
        &SsmiBagGraphMonitor::SearchGraphCallback, this);
    static_graph_sub_ = nh_.subscribe(
        "/viz_static_main_graph", 2,
        &SsmiBagGraphMonitor::StaticGraphCallback, this);
    full_graph_sub_ = nh_.subscribe(
        "/viz_graph_topic", 2,
        &SsmiBagGraphMonitor::FullGraphCallback, this);
    blocked_graph_sub_ = nh_.subscribe(
        "/viz_dynamic_blocked_edges", 2,
        &SsmiBagGraphMonitor::BlockedGraphCallback, this);
    path_sub_ = nh_.subscribe("/far_global_path", 2,
                              &SsmiBagGraphMonitor::PathCallback, this);
    waypoint_sub_ = nh_.subscribe("/way_point", 5,
                                  &SsmiBagGraphMonitor::WaypointCallback, this);
    reach_sub_ = nh_.subscribe("/far_reach_goal_status", 5,
                               &SsmiBagGraphMonitor::ReachCallback, this);
    goal_sub_ = nh_.subscribe("/goal_point", 5,
                              &SsmiBagGraphMonitor::GoalCallback, this);
    clock_sub_ = nh_.subscribe("/clock", 20,
                               &SsmiBagGraphMonitor::ClockCallback, this);
    snapshot_time_sub_ = nh_.subscribe(
        "/semantic_snapshot_time", 5,
        &SsmiBagGraphMonitor::SnapshotTimeCallback, this);
    update_time_sub_ = nh_.subscribe(
        "/semantic_planner_update_time", 5,
        &SsmiBagGraphMonitor::UpdateTimeCallback, this);
    callback_time_sub_ = nh_.subscribe(
        "/semantic_callback_time", 5,
        &SsmiBagGraphMonitor::CallbackTimeCallback, this);
    main_loop_time_sub_ = nh_.subscribe(
        "/far_main_loop_time", 5,
        &SsmiBagGraphMonitor::MainLoopTimeCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/goal_point", 1,
                                                            true);
    wall_timer_ = nh_.createWallTimer(ros::WallDuration(0.25),
                                      &SsmiBagGraphMonitor::WallTimerCallback,
                                      this);
    start_wall_ = ros::WallTime::now();

    ROS_INFO("SSMI graph monitor: expected frame=%s, local voxel=%s, CSV=%s, auto_goal=%s",
             expected_frame_.c_str(), local_voxel_topic_.c_str(),
             output_csv_.c_str(), auto_goal_ ? "true" : "false");
  }

  ~SsmiBagGraphMonitor() {
    if (!finalized_) Finalize("shutdown before bag-idle completion");
  }

  bool passed() const { return final_pass_; }

 private:
  void OdomCallback(const nav_msgs::OdometryConstPtr& message) {
    if (!message) return;
    source_odom_frame_ = message->header.frame_id;
    odom_stamp_ = message->header.stamp;

    tf::Pose source_pose;
    tf::poseMsgToTF(message->pose.pose, source_pose);
    tf::Pose aligned_pose = source_pose;
    if (!SameFrame(message->header.frame_id, expected_frame_)) {
      try {
        tf::StampedTransform transform;
        // map -> map_start is constant for one bag session.  Time(0) avoids
        // comparing the intentionally old acquisition stamps with /clock.
        tf_listener_.lookupTransform(expected_frame_, message->header.frame_id,
                                     ros::Time(0), transform);
        aligned_pose = transform * source_pose;
      } catch (const tf::TransformException& exception) {
        ROS_WARN_THROTTLE(2.0, "SSMI monitor waiting for initial reference TF: %s",
                          exception.what());
        return;
      }
    }

    current_robot_.x = aligned_pose.getOrigin().x();
    current_robot_.y = aligned_pose.getOrigin().y();
    current_robot_.z = aligned_pose.getOrigin().z();
    have_aligned_odom_ = true;
    if (!have_initial_alignment_) {
      initial_position_error_ = std::sqrt(
          current_robot_.x * current_robot_.x +
          current_robot_.y * current_robot_.y +
          current_robot_.z * current_robot_.z);
      initial_yaw_error_deg_ = std::fabs(NormalizeAngle(
          tf::getYaw(aligned_pose.getRotation()))) * 180.0 / M_PI;
      have_initial_alignment_ = true;
      ROS_INFO("SSMI initial alignment: position error=%.6f m, yaw error=%.4f deg",
               initial_position_error_, initial_yaw_error_deg_);
    }
  }

  void MapCallback(const octomap_msgs::OctomapConstPtr& message) {
    if (!message) return;
    have_map_ = true;
    map_frame_ = message->header.frame_id;
    map_stamp_ = message->header.stamp;
    map_id_ = message->id;
  }

  void LocalVoxelCallback(
      const sensor_msgs::PointCloud2ConstPtr& message) {
    if (!message) return;
    local_voxel_frame_ = message->header.frame_id;
    local_voxel_points_ =
        static_cast<size_t>(message->width) * message->height;
    local_voxel_schema_valid_ =
        HasPointCloudField(*message, "x") &&
        HasPointCloudField(*message, "y") &&
        HasPointCloudField(*message, "z") &&
        HasPointCloudField(*message, "label") &&
        HasPointCloudField(*message, "semantic_confidence") &&
        (HasPointCloudField(*message, "traversability") ||
         HasPointCloudField(*message, "intensity"));
    if (message->header.stamp.isZero()) {
      local_voxel_zero_stamp_seen_ = true;
      return;
    }
    have_local_voxel_ = true;
    if (local_voxel_stamp_.isZero() ||
        message->header.stamp > local_voxel_stamp_) {
      local_voxel_stamp_ = message->header.stamp;
      ++local_voxel_acquisitions_;
    } else if (message->header.stamp < local_voxel_stamp_) {
      local_voxel_out_of_order_seen_ = true;
    }
  }

  void StaticCloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    if (!message) return;
    static_frame_ = message->header.frame_id;
    static_points_ = static_cast<size_t>(message->width) * message->height;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        pcl::PointXYZ point(*x, *y, *z);
        accumulated_static_[VoxelKey(*x, *y, *z,
                                     static_accumulation_resolution_)] = point;
      }
    } catch (const std::runtime_error& exception) {
      ROS_ERROR_THROTTLE(2.0, "SSMI static cloud has invalid XYZ fields: %s",
                         exception.what());
    }
  }

  void GraphStaticCloudCallback(
      const sensor_msgs::PointCloud2ConstPtr& message) {
    if (!message) return;
    graph_static_frame_ = message->header.frame_id;
    graph_static_points_ =
        static_cast<size_t>(message->width) * message->height;
    static_cloud_->clear();
    static_cloud_->reserve(graph_static_points_);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) ||
            !std::isfinite(*z)) {
          continue;
        }
        static_cloud_->push_back(pcl::PointXYZ(*x, *y, *z));
      }
      static_cloud_->width = static_cast<uint32_t>(static_cloud_->size());
      static_cloud_->height = 1;
      static_kdtree_->setInputCloud(static_cloud_);
      robot_static_clearance_ = std::numeric_limits<double>::infinity();
      if (have_aligned_odom_ && !static_cloud_->empty()) {
        pcl::PointXYZ query(current_robot_.x, current_robot_.y,
                            current_robot_.z);
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        if (static_kdtree_->nearestKSearch(
                query, 1, indices, squared_distances) > 0) {
          robot_static_clearance_ = std::sqrt(squared_distances.front());
        }
      }
      graph_static_stamp_ = message->header.stamp;
      static_tree_dirty_ = false;
      if (have_full_graph_message_) EvaluateFullGraph(latest_full_graph_);
    } catch (const std::runtime_error& exception) {
      ROS_ERROR_THROTTLE(
          2.0, "SSMI graph static cloud has invalid XYZ fields: %s",
          exception.what());
    }
  }

  void DynamicCloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    if (!message) return;
    dynamic_frame_ = message->header.frame_id;
    dynamic_points_ = static_cast<size_t>(message->width) * message->height;
  }

  void SearchGraphCallback(const MarkerArray::ConstPtr& message) {
    if (!message) return;
    const visualization_msgs::Marker* node_marker = nullptr;
    const visualization_msgs::Marker* edge_marker = nullptr;
    for (const auto& marker : message->markers) {
      if (!marker.header.frame_id.empty()) graph_frame_ = marker.header.frame_id;
      if (marker.ns == "search_graph_nodes") node_marker = &marker;
      if (marker.ns == "search_graph_edges") edge_marker = &marker;
    }
    if (!node_marker || !edge_marker) return;

    graph_nodes_ = node_marker->points.size();
    graph_edges_ = edge_marker->points.size() / 2;
    std::map<QuantizedPoint, size_t> point_index;
    for (size_t index = 0; index < node_marker->points.size(); ++index) {
      point_index.emplace(Quantize(node_marker->points[index]), index);
    }
    std::vector<std::set<size_t>> adjacency(graph_nodes_);
    for (size_t index = 0; index + 1 < edge_marker->points.size(); index += 2) {
      const auto first = point_index.find(Quantize(edge_marker->points[index]));
      const auto second = point_index.find(Quantize(edge_marker->points[index + 1]));
      if (first == point_index.end() || second == point_index.end()) continue;
      adjacency[first->second].insert(second->second);
      adjacency[second->second].insert(first->second);
    }

    std::vector<size_t> component_of(graph_nodes_, graph_nodes_);
    std::vector<size_t> component_sizes;
    for (size_t root = 0; root < graph_nodes_; ++root) {
      if (component_of[root] != graph_nodes_) continue;
      const size_t component = component_sizes.size();
      size_t count = 0;
      std::queue<size_t> pending;
      pending.push(root);
      component_of[root] = component;
      while (!pending.empty()) {
        const size_t current = pending.front();
        pending.pop();
        ++count;
        for (const size_t neighbor : adjacency[current]) {
          if (component_of[neighbor] == graph_nodes_) {
            component_of[neighbor] = component;
            pending.push(neighbor);
          }
        }
      }
      component_sizes.push_back(count);
    }
    graph_components_ = component_sizes.size();
    largest_component_ = component_sizes.empty()
                             ? 0
                             : *std::max_element(component_sizes.begin(),
                                                 component_sizes.end());

    robot_component_ = 0;
    robot_degree_ = 0;
    if (have_aligned_odom_ && !node_marker->points.empty()) {
      size_t nearest = 0;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (size_t index = 0; index < node_marker->points.size(); ++index) {
        const double distance = Distance3D(
            current_robot_, node_marker->points[index]);
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest = index;
        }
      }
      robot_degree_ = adjacency[nearest].size();
      if (nearest < component_of.size() &&
          component_of[nearest] < component_sizes.size()) {
        robot_component_ = component_sizes[component_of[nearest]];
      }
    }

    have_graph_ = graph_nodes_ > 0;
    if (have_graph_ && have_aligned_odom_ &&
        (robot_component_ == 0 || robot_component_ != largest_component_ ||
         robot_degree_ == 0)) {
      graph_connectivity_ever_failed_ = true;
    }
    if (have_graph_ && first_graph_wall_.isZero()) {
      first_graph_wall_ = ros::WallTime::now();
    }
    WriteCsvRow();
  }

  void StaticGraphCallback(const MarkerArray::ConstPtr& message) {
    if (!message) return;
    const visualization_msgs::Marker* node_marker = nullptr;
    const visualization_msgs::Marker* edge_marker = nullptr;
    for (const auto& marker : message->markers) {
      if (!marker.header.frame_id.empty()) {
        static_graph_frame_ = marker.header.frame_id;
      }
      if (marker.ns == "static_main_nodes") node_marker = &marker;
      if (marker.ns == "static_main_edges") edge_marker = &marker;
    }
    if (!node_marker || !edge_marker) return;

    static_graph_nodes_ = node_marker->points.size();
    static_graph_edges_ = edge_marker->points.size() / 2;
    std::map<QuantizedPoint, size_t> point_index;
    for (size_t index = 0; index < node_marker->points.size(); ++index) {
      point_index.emplace(Quantize(node_marker->points[index]), index);
    }
    std::vector<std::set<size_t>> adjacency(static_graph_nodes_);
    for (size_t index = 0; index + 1 < edge_marker->points.size();
         index += 2) {
      const auto first = point_index.find(
          Quantize(edge_marker->points[index]));
      const auto second = point_index.find(
          Quantize(edge_marker->points[index + 1]));
      if (first == point_index.end() || second == point_index.end()) continue;
      adjacency[first->second].insert(second->second);
      adjacency[second->second].insert(first->second);
    }

    std::vector<bool> visited(static_graph_nodes_, false);
    std::vector<size_t> component_sizes;
    for (size_t root = 0; root < static_graph_nodes_; ++root) {
      if (visited[root]) continue;
      size_t count = 0;
      std::queue<size_t> pending;
      pending.push(root);
      visited[root] = true;
      while (!pending.empty()) {
        const size_t current = pending.front();
        pending.pop();
        ++count;
        for (const size_t neighbor : adjacency[current]) {
          if (!visited[neighbor]) {
            visited[neighbor] = true;
            pending.push(neighbor);
          }
        }
      }
      component_sizes.push_back(count);
    }
    static_graph_components_ = component_sizes.size();
    static_graph_largest_ = component_sizes.empty()
        ? 0
        : *std::max_element(component_sizes.begin(), component_sizes.end());
    if (static_graph_nodes_ > 0) have_static_graph_ = true;

    const bool split = static_graph_nodes_ > 0 &&
        (static_graph_components_ != 1 ||
         static_graph_largest_ != static_graph_nodes_);
    if (split) {
      ++static_graph_invalid_frames_;
      if (static_graph_invalid_frames_ >=
          static_cast<size_t>(static_component_failure_frames_)) {
        static_graph_connectivity_ever_failed_ = true;
      }
    } else {
      static_graph_invalid_frames_ = 0;
    }
    maximum_static_graph_components_ = std::max(
        maximum_static_graph_components_, static_graph_components_);
  }

  void FullGraphCallback(const MarkerArray::ConstPtr& message) {
    if (!message) return;
    latest_full_graph_ = *message;
    have_full_graph_message_ = true;
    EvaluateFullGraph(latest_full_graph_);
  }

  void EvaluateFullGraph(const MarkerArray& message) {
    static const std::set<std::string> traversal_namespaces = {
        "validated_route_edge",
        "validated_route_edge_endpoint_excluded"};
    ros::Time graph_stamp;
    for (const auto& marker : message.markers) {
      if (traversal_namespaces.count(marker.ns) != 0) {
        graph_stamp = marker.header.stamp;
        break;
      }
    }
    // Graph geometry and collision cloud are published as one snapshot by
    // FAR.  Never compare an old graph against a newer moving-window cloud.
    if (graph_stamp.isZero() || graph_static_stamp_.isZero() ||
        graph_stamp != graph_static_stamp_) {
      return;
    }
    graph_edges_checked_ = 0;
    graph_clearance_violations_ = 0;
    graph_violation_details_.clear();
    graph_min_clearance_ = std::numeric_limits<double>::infinity();
    RebuildStaticTree();
    for (const auto& marker : message.markers) {
      if (traversal_namespaces.count(marker.ns) == 0) continue;
      for (size_t index = 0; index + 1 < marker.points.size(); index += 2) {
        const Point& start = marker.points[index];
        const Point& end = marker.points[index + 1];
        if (have_aligned_odom_ &&
            PointSegmentDistance(current_robot_, start, end) >
                collision_check_radius_) {
          continue;
        }
        // Obstacle-to-obstacle visibility routes intentionally exclude their
        // contour endpoints in FAR's collision test.  Odom, goal and
        // contour-follow routes are checked over their complete geometry.
        const double endpoint_exclusion =
            marker.ns == "validated_route_edge_endpoint_excluded"
                ? visibility_endpoint_exclusion_
                : 0.0;
        pcl::PointXYZ nearest_obstacle;
        Point nearest_route;
        const double clearance = SegmentClearance(
            start, end, endpoint_exclusion, &nearest_obstacle,
            &nearest_route);
        if (!std::isfinite(clearance)) continue;
        ++graph_edges_checked_;
        graph_min_clearance_ = std::min(graph_min_clearance_, clearance);
        if (clearance + 1e-6 < collision_clearance_) {
          ++graph_clearance_violations_;
          std::ostringstream detail;
          detail << std::fixed << std::setprecision(3) << marker.ns
                 << " edge[(" << start.x << ',' << start.y << ',' << start.z
                 << ")->(" << end.x << ',' << end.y << ',' << end.z
                 << ")] route_sample=(" << nearest_route.x << ','
                 << nearest_route.y << ',' << nearest_route.z
                 << ") obstacle=(" << nearest_obstacle.x << ','
                 << nearest_obstacle.y << ',' << nearest_obstacle.z
                 << ") clearance=" << clearance;
          graph_violation_details_.push_back(detail.str());
        }
      }
    }
    maximum_graph_edges_checked_ =
        std::max(maximum_graph_edges_checked_, graph_edges_checked_);
    maximum_graph_clearance_violations_ = std::max(
        maximum_graph_clearance_violations_, graph_clearance_violations_);
    if (graph_edges_checked_ > 0) {
      minimum_graph_clearance_over_run_ = std::min(
          minimum_graph_clearance_over_run_, graph_min_clearance_);
    }
    // Keep the worst offending snapshot for the final report.  The current
    // details are rebuilt on every graph/cloud pair, so without this copy a
    // transient unsafe edge is usually gone by the time the bag finishes.
    if (!graph_violation_details_.empty() &&
        graph_min_clearance_ < worst_graph_violation_clearance_) {
      worst_graph_violation_clearance_ = graph_min_clearance_;
      worst_graph_violation_details_ = graph_violation_details_;
    }
  }

  void BlockedGraphCallback(const MarkerArray::ConstPtr& message) {
    blocked_edges_ = 0;
    if (!message) return;
    for (const auto& marker : message->markers) {
      blocked_edges_ += marker.points.size() / 2;
    }
  }

  void PathCallback(const nav_msgs::PathConstPtr& message) {
    if (!message) return;
    path_frame_ = message->header.frame_id;
    path_poses_ = message->poses.size();
    latest_path_points_.clear();
    latest_path_points_.reserve(message->poses.size());
    for (const auto& pose : message->poses) {
      latest_path_points_.push_back(pose.pose.position);
    }
    path_length_ = 0.0;
    path_min_clearance_ = std::numeric_limits<double>::infinity();
    RebuildStaticTree();
    for (size_t index = 1; index < message->poses.size(); ++index) {
      const Point& previous = message->poses[index - 1].pose.position;
      const Point& current = message->poses[index].pose.position;
      path_length_ += Distance3D(previous, current);
      path_min_clearance_ = std::min(path_min_clearance_,
                                     SegmentClearance(previous, current,
                                                      collision_clearance_));
    }
    if (!message->poses.empty() && have_goal_) {
      path_goal_error_ = Distance3D(message->poses.back().pose.position, goal_);
    } else {
      path_goal_error_ = std::numeric_limits<double>::infinity();
    }
    if (!message->poses.empty()) {
      ever_nonempty_path_ = true;
      maximum_path_poses_ = std::max(maximum_path_poses_, path_poses_);
      minimum_path_goal_error_ = std::min(minimum_path_goal_error_,
                                          path_goal_error_);
      minimum_path_clearance_ = std::min(minimum_path_clearance_,
                                         path_min_clearance_);
    }
  }

  void WaypointCallback(const geometry_msgs::PointStampedConstPtr& message) {
    if (!message) return;
    waypoint_ = message->point;
    waypoint_frame_ = message->header.frame_id;
    have_waypoint_ = true;
    waypoint_path_distance_ = std::numeric_limits<double>::infinity();
    if (latest_path_points_.size() == 1) {
      waypoint_path_distance_ = Distance3D(waypoint_, latest_path_points_[0]);
    }
    for (size_t index = 1; index < latest_path_points_.size(); ++index) {
      waypoint_path_distance_ = std::min(
          waypoint_path_distance_, PointSegmentDistance(
              waypoint_, latest_path_points_[index - 1],
              latest_path_points_[index]));
    }
  }

  void ReachCallback(const std_msgs::BoolConstPtr& message) {
    if (!message) return;
    reached_ = message->data;
    ever_reached_ = ever_reached_ || reached_;
  }

  void GoalCallback(const geometry_msgs::PointStampedConstPtr& message) {
    if (!message) return;
    goal_ = message->point;
    goal_frame_ = message->header.frame_id;
    have_goal_ = true;
  }

  void ClockCallback(const rosgraph_msgs::ClockConstPtr& message) {
    if (!message) return;
    last_clock_ = message->clock;
    last_clock_wall_ = ros::WallTime::now();
    have_clock_ = true;
  }

  void SnapshotTimeCallback(const std_msgs::Float32ConstPtr& message) {
    if (message) semantic_snapshot_time_ = message->data;
  }
  void UpdateTimeCallback(const std_msgs::Float32ConstPtr& message) {
    if (message) semantic_update_time_ = message->data;
  }
  void CallbackTimeCallback(const std_msgs::Float32ConstPtr& message) {
    if (message) semantic_callback_time_ = message->data;
  }
  void MainLoopTimeCallback(const std_msgs::Float32ConstPtr& message) {
    if (message) main_loop_time_ = message->data;
  }

  void WallTimerCallback(const ros::WallTimerEvent&) {
    if (auto_goal_ && !goal_sent_ && have_graph_ && have_map_ &&
        have_local_voxel_ && local_voxel_schema_valid_ &&
        have_initial_alignment_ && !first_graph_wall_.isZero() &&
        (ros::WallTime::now() - first_graph_wall_).toSec() >=
            goal_publish_delay_) {
      geometry_msgs::PointStamped command;
      command.header.stamp = ros::Time::now();
      command.header.frame_id = expected_frame_;
      command.point = goal_;
      goal_pub_.publish(command);
      goal_sent_ = true;
      have_goal_ = true;
      goal_frame_ = expected_frame_;
      ROS_INFO("SSMI monitor published deterministic goal (%.3f, %.3f, %.3f) in %s",
               goal_.x, goal_.y, goal_.z, expected_frame_.c_str());
    }

    if (finish_on_bag_idle_ && have_clock_ && !last_clock_wall_.isZero() &&
        (ros::WallTime::now() - last_clock_wall_).toSec() >=
            bag_idle_timeout_) {
      Finalize("bag clock became idle");
      ros::shutdown();
    }
  }

  void RebuildStaticTree() {
    if (!static_tree_dirty_) return;
    static_kdtree_->setInputCloud(static_cloud_);
    static_tree_dirty_ = false;
  }

  double SegmentClearance(const Point& start, const Point& end,
                          const double endpoint_exclusion = 0.0,
                          pcl::PointXYZ* nearest_obstacle = nullptr,
                          Point* nearest_route = nullptr) const {
    if (!static_cloud_ || static_cloud_->empty()) {
      return std::numeric_limits<double>::infinity();
    }
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    // Match ContourGraph::IsEdgeCollisionFreeInCloud(): its sampling distance
    // and endpoint exclusion are defined by planar route length.
    const double length = std::hypot(dx, dy);
    if (length <= 1e-9) return std::numeric_limits<double>::infinity();
    const double margin = std::min(
        std::max(0.0, endpoint_exclusion), length * 0.45);
    const double checked_length = std::max(0.0, length - 2.0 * margin);
    // FAR performs the swept-body query in XY at the edge's mid-height.  Use
    // the same vertical slice so this independent monitor reproduces the
    // planner's 0.45 m clearance test instead of measuring a different,
    // interpolated-height trajectory.
    const double mid_z = (start.z + end.z) * 0.5;
    const size_t samples = std::max<size_t>(
        1, static_cast<size_t>(std::ceil(checked_length /
                                        std::max(0.02, collision_sample_step_))));
    double minimum = std::numeric_limits<double>::infinity();
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    for (size_t sample = 0; sample <= samples; ++sample) {
      const double along = margin + checked_length *
          static_cast<double>(sample) / samples;
      const double ratio = along / length;
      pcl::PointXYZ query;
      query.x = start.x + ratio * dx;
      query.y = start.y + ratio * dy;
      query.z = mid_z;
      if (static_kdtree_->nearestKSearch(query, 1, indices, squared_distances) > 0) {
        const double distance =
            std::sqrt(static_cast<double>(squared_distances[0]));
        if (distance < minimum) {
          minimum = distance;
          if (nearest_obstacle) {
            *nearest_obstacle = static_cloud_->points[indices[0]];
          }
          if (nearest_route) {
            nearest_route->x = query.x;
            nearest_route->y = query.y;
            nearest_route->z = query.z;
          }
        }
      }
    }
    return minimum;
  }

  bool FramesValid() const {
    if (!have_map_ || !SameFrame(map_frame_, expected_frame_)) return false;
    if (!have_local_voxel_ ||
        !SameFrame(local_voxel_frame_, expected_frame_)) {
      return false;
    }
    if (!graph_frame_.empty() && !SameFrame(graph_frame_, expected_frame_)) {
      return false;
    }
    if (!static_graph_frame_.empty() &&
        !SameFrame(static_graph_frame_, expected_frame_)) {
      return false;
    }
    if (!static_frame_.empty() && !SameFrame(static_frame_, expected_frame_)) {
      return false;
    }
    if (!dynamic_frame_.empty() && !SameFrame(dynamic_frame_, expected_frame_)) {
      return false;
    }
    if (!graph_static_frame_.empty() &&
        !SameFrame(graph_static_frame_, expected_frame_)) {
      return false;
    }
    if (have_goal_ && !SameFrame(goal_frame_, expected_frame_)) return false;
    if (ever_nonempty_path_ && !SameFrame(path_frame_, expected_frame_)) {
      return false;
    }
    if (have_waypoint_ && !SameFrame(waypoint_frame_, expected_frame_)) {
      return false;
    }
    return true;
  }

  void WriteCsvRow() {
    if (!csv_.is_open()) return;
    csv_ << std::fixed << std::setprecision(6)
         << (ros::WallTime::now() - start_wall_).toSec() << ','
         << ros::Time::now().toSec() << ',' << map_stamp_.toSec() << ','
         << odom_stamp_.toSec() << ',' << local_voxel_stamp_.toSec() << ','
         << map_frame_ << ',' << source_odom_frame_ << ','
         << local_voxel_frame_ << ',' << local_voxel_points_ << ','
         << local_voxel_acquisitions_ << ','
         << (local_voxel_schema_valid_ ? 1 : 0) << ','
         << graph_frame_ << ',' << goal_frame_
         << ',' << path_frame_ << ',' << initial_position_error_ << ','
         << initial_yaw_error_deg_ << ',' << static_points_ << ','
         << dynamic_points_ << ',' << graph_static_points_ << ','
         << accumulated_static_.size() << ','
         << graph_nodes_ << ',' << graph_edges_ << ',' << graph_components_
         << ',' << largest_component_ << ',' << static_graph_nodes_ << ','
         << static_graph_edges_ << ',' << static_graph_components_ << ','
         << static_graph_largest_ << ',' << static_graph_invalid_frames_ << ','
         << robot_component_ << ','
         << robot_degree_ << ',' << blocked_edges_ << ',' << path_poses_ << ','
         << path_length_ << ',' << path_goal_error_ << ','
         << path_min_clearance_ << ',' << graph_edges_checked_ << ','
         << graph_min_clearance_ << ',' << graph_clearance_violations_ << ','
         << robot_static_clearance_ << ','
         << waypoint_path_distance_ << ',' << (reached_ ? 1 : 0) << ','
         << semantic_snapshot_time_ << ',' << semantic_update_time_ << ','
         << semantic_callback_time_ << ',' << main_loop_time_ << '\n';
    csv_.flush();
  }

  void Finalize(const std::string& reason) {
    if (finalized_) return;
    finalized_ = true;
    const bool alignment_ok = have_initial_alignment_ &&
                              initial_position_error_ <= position_tolerance_ &&
                              initial_yaw_error_deg_ <= yaw_tolerance_deg_;
    const bool semantic_ok = have_map_ && map_id_ == "SemanticOcTree" &&
                             static_points_ > 0;
    const bool local_voxel_ok = have_local_voxel_ &&
                                local_voxel_acquisitions_ > 0 &&
                                local_voxel_schema_valid_ &&
                                !local_voxel_zero_stamp_seen_ &&
                                !local_voxel_out_of_order_seen_;
    const bool graph_ok = have_graph_ && !graph_connectivity_ever_failed_ &&
                          robot_component_ > 0 &&
                          robot_component_ == largest_component_ &&
                          robot_degree_ > 0 && have_static_graph_ &&
                          !static_graph_connectivity_ever_failed_ &&
                          static_graph_components_ == 1 &&
                          static_graph_largest_ == static_graph_nodes_;
    const bool goal_ok = !auto_goal_ ||
                         (goal_sent_ && ever_nonempty_path_ &&
                          maximum_path_poses_ > 1 &&
                          minimum_path_goal_error_ <= position_tolerance_);
    // Clearance is reported as a first-class acceptance metric.  A failure is
    // intentionally non-destructive: the CSV and final summary remain usable
    // for tuning robot geometry and contour projections.
    const bool clearance_ok = maximum_graph_edges_checked_ > 0 &&
                              maximum_graph_clearance_violations_ == 0;
    final_pass_ = alignment_ok && FramesValid() && semantic_ok &&
                  local_voxel_ok && graph_ok && goal_ok && clearance_ok;

    ROS_INFO("SSMI monitor finalizing: %s", reason.c_str());
    ROS_INFO("SSMI_MONITOR_RESULT %s alignment=%s frames=%s semantic=%s local_voxel=%s graph=%s goal=%s clearance=%s",
             final_pass_ ? "PASS" : "FAIL", alignment_ok ? "PASS" : "FAIL",
             FramesValid() ? "PASS" : "FAIL",
             semantic_ok ? "PASS" : "FAIL",
             local_voxel_ok ? "PASS" : "FAIL",
             graph_ok ? "PASS" : "FAIL",
             goal_ok ? "PASS" : "FAIL", clearance_ok ? "PASS" : "FAIL");
    ROS_INFO("SSMI metrics: initial_error=%.4fm yaw_error=%.3fdeg local_voxel=%zu acquisitions=%zu schema=%s static=%zu accumulated=%zu graph=%zu/%zu components=%zu robot_component=%zu degree=%zu connectivity_ever_failed=%s static_graph=%zu/%zu components=%zu largest=%zu max_components=%zu connectivity_ever_failed=%s max_path=%zu min_goal_error=%.4fm min_graph_clearance=%.4fm max_violations=%zu reached=%s",
             initial_position_error_, initial_yaw_error_deg_,
             local_voxel_points_, local_voxel_acquisitions_,
             local_voxel_schema_valid_ ? "true" : "false",
             static_points_, accumulated_static_.size(),
             graph_nodes_, graph_edges_,
             graph_components_, robot_component_, robot_degree_,
             graph_connectivity_ever_failed_ ? "true" : "false",
             static_graph_nodes_, static_graph_edges_,
             static_graph_components_, static_graph_largest_,
             maximum_static_graph_components_,
             static_graph_connectivity_ever_failed_ ? "true" : "false",
             maximum_path_poses_, minimum_path_goal_error_,
             minimum_graph_clearance_over_run_,
             maximum_graph_clearance_violations_,
             ever_reached_ ? "true" : "false");
    for (const auto& detail : worst_graph_violation_details_) {
      ROS_ERROR("SSMI worst graph clearance violation: %s", detail.c_str());
    }
    if (csv_.is_open()) csv_.close();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf::TransformListener tf_listener_;
  ros::Subscriber odom_sub_, map_sub_, local_voxel_sub_, static_cloud_sub_;
  ros::Subscriber dynamic_cloud_sub_;
  ros::Subscriber graph_static_cloud_sub_;
  ros::Subscriber search_graph_sub_, static_graph_sub_, full_graph_sub_;
  ros::Subscriber blocked_graph_sub_;
  ros::Subscriber path_sub_, waypoint_sub_, reach_sub_, goal_sub_, clock_sub_;
  ros::Subscriber snapshot_time_sub_, update_time_sub_, callback_time_sub_;
  ros::Subscriber main_loop_time_sub_;
  ros::Publisher goal_pub_;
  ros::WallTimer wall_timer_;

  std::string expected_frame_, source_odom_topic_, local_voxel_topic_;
  std::string output_csv_;
  std::string source_odom_frame_, map_frame_, graph_frame_, goal_frame_;
  std::string path_frame_, waypoint_frame_, static_frame_, dynamic_frame_;
  std::string graph_static_frame_, static_graph_frame_;
  std::string local_voxel_frame_;
  std::string map_id_;
  std::ofstream csv_;
  ros::WallTime start_wall_, first_graph_wall_, last_clock_wall_;
  ros::Time last_clock_, map_stamp_, odom_stamp_, local_voxel_stamp_;
  ros::Time graph_static_stamp_;

  bool auto_goal_ = false;
  bool finish_on_bag_idle_ = true;
  bool have_clock_ = false;
  bool have_map_ = false;
  bool have_local_voxel_ = false;
  bool local_voxel_schema_valid_ = false;
  bool local_voxel_zero_stamp_seen_ = false;
  bool local_voxel_out_of_order_seen_ = false;
  bool have_aligned_odom_ = false;
  bool have_initial_alignment_ = false;
  bool have_graph_ = false;
  bool have_static_graph_ = false;
  bool have_goal_ = false;
  bool have_waypoint_ = false;
  bool goal_sent_ = false;
  bool reached_ = false;
  bool ever_reached_ = false;
  bool ever_nonempty_path_ = false;
  bool static_tree_dirty_ = false;
  bool have_full_graph_message_ = false;
  bool graph_connectivity_ever_failed_ = false;
  bool static_graph_connectivity_ever_failed_ = false;
  bool finalized_ = false;
  bool final_pass_ = false;

  double goal_publish_delay_ = 5.0;
  double bag_idle_timeout_ = 5.0;
  double position_tolerance_ = 0.02;
  double yaw_tolerance_deg_ = 1.0;
  double collision_clearance_ = 0.45;
  double collision_sample_step_ = 0.10;
  double collision_check_radius_ = 10.0;
  double visibility_endpoint_exclusion_ = 0.875;
  double static_accumulation_resolution_ = 0.20;
  int static_component_failure_frames_ = 5;
  double initial_position_error_ = std::numeric_limits<double>::infinity();
  double initial_yaw_error_deg_ = std::numeric_limits<double>::infinity();
  double path_length_ = 0.0;
  double path_goal_error_ = std::numeric_limits<double>::infinity();
  double minimum_path_goal_error_ = std::numeric_limits<double>::infinity();
  double path_min_clearance_ = std::numeric_limits<double>::infinity();
  double minimum_path_clearance_ = std::numeric_limits<double>::infinity();
  double graph_min_clearance_ = std::numeric_limits<double>::infinity();
  double minimum_graph_clearance_over_run_ =
      std::numeric_limits<double>::infinity();
  double worst_graph_violation_clearance_ =
      std::numeric_limits<double>::infinity();
  double robot_static_clearance_ =
      std::numeric_limits<double>::infinity();
  double waypoint_path_distance_ = std::numeric_limits<double>::infinity();
  float semantic_snapshot_time_ = 0.0f;
  float semantic_update_time_ = 0.0f;
  float semantic_callback_time_ = 0.0f;
  float main_loop_time_ = 0.0f;

  Point current_robot_, goal_, waypoint_;
  std::vector<Point> latest_path_points_;
  MarkerArray latest_full_graph_;
  std::vector<std::string> graph_violation_details_;
  std::vector<std::string> worst_graph_violation_details_;
  size_t static_points_ = 0;
  size_t local_voxel_points_ = 0;
  size_t local_voxel_acquisitions_ = 0;
  size_t dynamic_points_ = 0;
  size_t graph_static_points_ = 0;
  size_t graph_nodes_ = 0;
  size_t graph_edges_ = 0;
  size_t graph_components_ = 0;
  size_t largest_component_ = 0;
  size_t static_graph_nodes_ = 0;
  size_t static_graph_edges_ = 0;
  size_t static_graph_components_ = 0;
  size_t static_graph_largest_ = 0;
  size_t static_graph_invalid_frames_ = 0;
  size_t maximum_static_graph_components_ = 0;
  size_t robot_component_ = 0;
  size_t robot_degree_ = 0;
  size_t blocked_edges_ = 0;
  size_t path_poses_ = 0;
  size_t maximum_path_poses_ = 0;
  size_t graph_edges_checked_ = 0;
  size_t graph_clearance_violations_ = 0;
  size_t maximum_graph_edges_checked_ = 0;
  size_t maximum_graph_clearance_violations_ = 0;

  std::unordered_map<uint64_t, pcl::PointXYZ> accumulated_static_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr static_kdtree_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ssmi_bag_graph_monitor");
  SsmiBagGraphMonitor monitor;
  ros::spin();
  return monitor.passed() ? 0 : 2;
}
