#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <semantic_octree/SemanticOcTree.h>
#include <semantic_octree/Semantics.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

namespace {

using SemanticOctree = octomap::SemanticOcTree<octomap::SemanticsLogOdds>;

double Distance2D(double ax, double ay, double bx, double by) {
  return std::hypot(ax - bx, ay - by);
}

double NormalizeAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double DistanceToSegment2D(double px, double py,
                           double ax, double ay,
                           double bx, double by) {
  const double dx = bx - ax;
  const double dy = by - ay;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared < 1e-12) return Distance2D(px, py, ax, ay);
  const double projection = std::max(
      0.0, std::min(1.0, ((px - ax) * dx + (py - ay) * dy) /
                             length_squared));
  return Distance2D(px, py, ax + projection * dx, ay + projection * dy);
}

std::string MakeTimestamp() {
  const std::time_t now = std::time(nullptr);
  std::tm local_time;
  localtime_r(&now, &local_time);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local_time);
  return buffer;
}

std::string SummaryPathForCsv(const std::string& csv_path) {
  const std::string extension = ".csv";
  if (csv_path.size() >= extension.size() &&
      csv_path.compare(csv_path.size() - extension.size(), extension.size(),
                       extension) == 0) {
    return csv_path.substr(0, csv_path.size() - extension.size()) +
           "_summary.txt";
  }
  return csv_path + "_summary.txt";
}

enum class GoalMapState {
  kUnavailable,
  kUnknown,
  kKnownColumn,
  kFree,
  kOccupied,
};

const char* GoalMapStateName(GoalMapState state) {
  switch (state) {
    case GoalMapState::kUnknown: return "unknown";
    case GoalMapState::kKnownColumn: return "known_column";
    case GoalMapState::kFree: return "free";
    case GoalMapState::kOccupied: return "occupied";
    default: return "unavailable";
  }
}

template <typename TreeType>
GoalMapState QueryGoalState(const TreeType& tree,
                            const geometry_msgs::Point& goal,
                            double vertical_probe) {
  const auto* exact = tree.search(goal.x, goal.y, goal.z);
  if (exact != nullptr) {
    return tree.isNodeOccupied(*exact)
        ? GoalMapState::kOccupied : GoalMapState::kFree;
  }

  const double step = std::max(tree.getResolution(), 0.05);
  for (double offset = -vertical_probe;
       offset <= vertical_probe + 1e-9; offset += step) {
    const auto* node = tree.search(goal.x, goal.y, goal.z + offset);
    if (node != nullptr) return GoalMapState::kKnownColumn;
  }
  return GoalMapState::kUnknown;
}

const char* PassFail(bool pass) {
  return pass ? "PASS" : "FAIL";
}

}  // namespace

class NavigationDemoMonitor {
 public:
  NavigationDemoMonitor()
      : private_nh_("~"),
        start_wall_time_(ros::WallTime::now()),
        last_map_parse_wall_time_(ros::WallTime(0)),
        minimum_dynamic_robot_distance_(
            std::numeric_limits<double>::infinity()),
        minimum_dynamic_corridor_distance_(
            std::numeric_limits<double>::infinity()) {
    // Ensure the custom semantic tree registers with OctoMap's factory before
    // octomap_msgs::msgToMap() is called.
    static SemanticOctree semantic_registration_probe(0.1);
    (void)semantic_registration_probe;

    LoadParameters();
    OpenOutputs();
    Subscribe();

    sample_timer_ = private_nh_.createWallTimer(
        ros::WallDuration(1.0 / sample_rate_),
        &NavigationDemoMonitor::SampleTimerCallback, this);
    status_timer_ = private_nh_.createWallTimer(
        ros::WallDuration(status_period_),
        &NavigationDemoMonitor::StatusTimerCallback, this);

    ROS_INFO_STREAM("Navigation demo monitor started. CSV: " << output_csv_);
    ROS_INFO_STREAM("Waiting for one goal on " << goal_topic_
                    << "; start this monitor before publishing the demo goal.");
  }

  ~NavigationDemoMonitor() { Finalize(); }

 private:
  void LoadParameters() {
    private_nh_.param<std::string>("goal_topic", goal_topic_, "/goal_point");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/odom_world");
    private_nh_.param<std::string>("octomap_topic", octomap_topic_,
                                   "/octomap_full");
    private_nh_.param<std::string>("dynamic_cloud_topic", dynamic_cloud_topic_,
                                   "/FAR_dynamic_obs_debug");
    private_nh_.param<std::string>("waypoint_topic", waypoint_topic_,
                                   "/way_point");
    private_nh_.param<std::string>("path_topic", path_topic_, "/path");
    private_nh_.param<std::string>("reached_topic", reached_topic_,
                                   "/far_reach_goal_status");
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_,
                                   "/far_cmd_vel");

    private_nh_.param<double>("sample_rate", sample_rate_, 5.0);
    private_nh_.param<double>("status_period", status_period_, 2.0);
    private_nh_.param<double>("map_check_period", map_check_period_, 1.0);
    private_nh_.param<double>("goal_vertical_probe", goal_vertical_probe_, 0.8);
    private_nh_.param<double>("goal_tolerance", goal_tolerance_, 0.5);
    private_nh_.param<double>("dynamic_corridor_distance",
                              dynamic_corridor_distance_, 0.6);
    private_nh_.param<double>("minimum_safe_distance",
                              minimum_safe_distance_, 0.6);
    private_nh_.param<double>("waypoint_change_distance",
                              waypoint_change_distance_, 0.2);
    private_nh_.param<double>("minimum_path_heading_baseline",
                              minimum_path_heading_baseline_, 0.2);
    private_nh_.param<double>("maximum_roll_pitch_deg",
                              maximum_roll_pitch_deg_, 25.0);
    private_nh_.param<double>("minimum_robot_height",
                              minimum_robot_height_, 0.2);
    private_nh_.param<double>("maximum_duration", maximum_duration_, 0.0);
    private_nh_.param<double>("post_reach_duration", post_reach_duration_, 2.0);
    private_nh_.param<bool>("auto_finish_on_reach",
                            auto_finish_on_reach_, true);
    private_nh_.param<bool>("require_initial_unknown",
                            require_initial_unknown_, true);
    private_nh_.param<bool>("require_dynamic_interaction",
                            require_dynamic_interaction_, true);

    sample_rate_ = std::max(sample_rate_, 0.2);
    status_period_ = std::max(status_period_, 0.5);
    map_check_period_ = std::max(map_check_period_, 0.2);
    goal_vertical_probe_ = std::max(goal_vertical_probe_, 0.0);

    private_nh_.param<std::string>("output_csv", output_csv_, "");
    if (output_csv_.empty()) {
      output_csv_ = "/tmp/far_navigation_demo_" + MakeTimestamp() + ".csv";
    }
    private_nh_.param<std::string>("output_summary", output_summary_, "");
    if (output_summary_.empty()) {
      output_summary_ = SummaryPathForCsv(output_csv_);
    }
  }

  void OpenOutputs() {
    csv_.open(output_csv_);
    if (!csv_.is_open()) {
      ROS_FATAL_STREAM("Cannot open demo CSV output: " << output_csv_);
      throw std::runtime_error("cannot open navigation demo CSV");
    }
    csv_ << "wall_elapsed_s,ros_time_s,goal_map_state,robot_x,robot_y,robot_z,"
            "roll_deg,pitch_deg,goal_x,goal_y,goal_z,goal_distance_m,"
            "waypoint_x,waypoint_y,waypoint_z,waypoint_changes,"
            "dynamic_points,dynamic_encounter,minimum_dynamic_robot_m,"
            "minimum_dynamic_corridor_m,path_points,path_heading_rad,"
            "cmd_linear_x,cmd_angular_z,reached\n";
    csv_ << std::fixed << std::setprecision(6);
  }

  void Subscribe() {
    goal_sub_ = nh_.subscribe(goal_topic_, 5,
                              &NavigationDemoMonitor::GoalCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 20,
                              &NavigationDemoMonitor::OdomCallback, this);
    map_sub_ = nh_.subscribe(octomap_topic_, 1,
                             &NavigationDemoMonitor::MapCallback, this);
    dynamic_sub_ = nh_.subscribe(dynamic_cloud_topic_, 5,
                                 &NavigationDemoMonitor::DynamicCallback, this);
    waypoint_sub_ = nh_.subscribe(waypoint_topic_, 10,
                                  &NavigationDemoMonitor::WaypointCallback,
                                  this);
    path_sub_ = nh_.subscribe(path_topic_, 10,
                              &NavigationDemoMonitor::PathCallback, this);
    reached_sub_ = nh_.subscribe(reached_topic_, 10,
                                 &NavigationDemoMonitor::ReachedCallback,
                                 this);
    cmd_sub_ = nh_.subscribe(cmd_vel_topic_, 20,
                             &NavigationDemoMonitor::CmdCallback, this);
  }

  void GoalCallback(const geometry_msgs::PointStampedConstPtr& message) {
    ++goal_message_count_;
    if (!have_goal_) {
      goal_ = *message;
      have_goal_ = true;
      goal_receipt_wall_time_ = ros::WallTime::now();
      // The report window begins with the first accepted input goal. Ignore
      // default local-planner paths, simulator initialization poses, and
      // dynamic-cloud samples seen while the operator is still preparing.
      have_waypoint_ = false;
      have_path_ = false;
      have_path_heading_ = false;
      reached_ = false;
      current_dynamic_robot_distance_ =
          std::numeric_limits<double>::infinity();
      current_dynamic_corridor_distance_ =
          std::numeric_limits<double>::infinity();
      minimum_dynamic_robot_distance_ =
          std::numeric_limits<double>::infinity();
      minimum_dynamic_corridor_distance_ =
          std::numeric_limits<double>::infinity();
      minimum_robot_z_ = std::numeric_limits<double>::infinity();
      maximum_abs_roll_deg_ = 0.0;
      maximum_abs_pitch_deg_ = 0.0;
      maximum_abs_linear_speed_ = 0.0;
      maximum_abs_angular_speed_ = 0.0;
      if (have_odom_) {
        initial_goal_distance_ = Distance2D(
            odom_.pose.pose.position.x, odom_.pose.pose.position.y,
            goal_.point.x, goal_.point.y);
      }
      ROS_INFO_STREAM("Demo goal captured: (" << goal_.point.x << ", "
                      << goal_.point.y << ", " << goal_.point.z << ")");
      if (latest_map_) EvaluateMap(latest_map_, true);
      return;
    }

    const double change = Distance2D(goal_.point.x, goal_.point.y,
                                     message->point.x, message->point.y);
    if (change > waypoint_change_distance_ ||
        std::fabs(goal_.point.z - message->point.z) >
            waypoint_change_distance_) {
      ++goal_change_count_;
      ROS_ERROR_STREAM("Input goal changed during demo by " << change
                       << " m; the original captured goal remains the report "
                          "reference.");
    }
  }

  void OdomCallback(const nav_msgs::OdometryConstPtr& message) {
    odom_ = *message;
    have_odom_ = true;
    if (have_goal_ && !std::isfinite(initial_goal_distance_)) {
      initial_goal_distance_ = Distance2D(
          odom_.pose.pose.position.x, odom_.pose.pose.position.y,
          goal_.point.x, goal_.point.y);
    }

    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(message->pose.pose.orientation, quaternion);
    double yaw = 0.0;
    tf::Matrix3x3(quaternion).getRPY(current_roll_, current_pitch_, yaw);
    constexpr double kRadiansToDegrees = 57.2957795130823208768;
    if (have_goal_) {
      maximum_abs_roll_deg_ = std::max(
          maximum_abs_roll_deg_, std::fabs(current_roll_) * kRadiansToDegrees);
      maximum_abs_pitch_deg_ = std::max(
          maximum_abs_pitch_deg_, std::fabs(current_pitch_) * kRadiansToDegrees);
      minimum_robot_z_ = std::min(minimum_robot_z_, message->pose.pose.position.z);
      if (std::fabs(current_roll_) * kRadiansToDegrees > maximum_roll_pitch_deg_ ||
          std::fabs(current_pitch_) * kRadiansToDegrees > maximum_roll_pitch_deg_ ||
          message->pose.pose.position.z < minimum_robot_height_) {
        posture_violation_ = true;
      }
    }
  }

  void MapCallback(const octomap_msgs::OctomapConstPtr& message) {
    latest_map_ = message;
    have_map_ = true;
    if (!have_goal_) return;
    const ros::WallTime now = ros::WallTime::now();
    if (!have_initial_goal_map_state_ || last_map_parse_wall_time_.isZero() ||
        (now - last_map_parse_wall_time_).toSec() >= map_check_period_) {
      EvaluateMap(message, !have_initial_goal_map_state_);
    }
  }

  void EvaluateMap(const octomap_msgs::OctomapConstPtr& message,
                   bool capture_initial) {
    last_map_parse_wall_time_ = ros::WallTime::now();
    std::unique_ptr<octomap::AbstractOcTree> tree(
        octomap_msgs::msgToMap(*message));
    if (!tree) {
      ROS_WARN_THROTTLE(2.0, "Demo monitor could not deserialize octomap.");
      return;
    }

    GoalMapState state = GoalMapState::kUnavailable;
    if (const auto* semantic_tree =
            dynamic_cast<const SemanticOctree*>(tree.get())) {
      state = QueryGoalState(*semantic_tree, goal_.point, goal_vertical_probe_);
    } else if (const auto* color_tree =
                   dynamic_cast<const octomap::ColorOcTree*>(tree.get())) {
      state = QueryGoalState(*color_tree, goal_.point, goal_vertical_probe_);
    } else if (const auto* occupancy_tree =
                   dynamic_cast<const octomap::OcTree*>(tree.get())) {
      state = QueryGoalState(*occupancy_tree, goal_.point, goal_vertical_probe_);
    } else {
      ROS_WARN_THROTTLE(2.0,
          "Demo monitor received an unsupported octomap tree type.");
      return;
    }

    current_goal_map_state_ = state;
    if (capture_initial && !have_initial_goal_map_state_) {
      initial_goal_map_state_ = state;
      have_initial_goal_map_state_ = true;
      ROS_INFO_STREAM("Initial goal map state: " << GoalMapStateName(state));
    }
    if (state != GoalMapState::kUnknown &&
        state != GoalMapState::kUnavailable && !goal_became_known_) {
      goal_became_known_ = true;
      goal_known_wall_elapsed_ =
          (ros::WallTime::now() - goal_receipt_wall_time_).toSec();
      ROS_INFO_STREAM("Goal area became known at wall elapsed "
                      << goal_known_wall_elapsed_ << " s ("
                      << GoalMapStateName(state) << ").");
    }
  }

  void DynamicCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    if (!have_goal_) return;
    dynamic_point_count_ = static_cast<std::size_t>(message->width) *
                           static_cast<std::size_t>(message->height);
    if (dynamic_point_count_ == 0 || !have_odom_) {
      SetDynamicEncounter(false);
      return;
    }

    double frame_min_robot = std::numeric_limits<double>::infinity();
    double frame_min_corridor = std::numeric_limits<double>::infinity();
    const double robot_x = odom_.pose.pose.position.x;
    const double robot_y = odom_.pose.pose.position.y;
    const bool valid_segment = have_waypoint_ &&
        Distance2D(robot_x, robot_y, waypoint_.point.x, waypoint_.point.y) >
            waypoint_change_distance_;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      for (; x != x.end(); ++x, ++y) {
        if (!std::isfinite(*x) || !std::isfinite(*y)) continue;
        frame_min_robot = std::min(
            frame_min_robot, Distance2D(*x, *y, robot_x, robot_y));
        if (valid_segment) {
          frame_min_corridor = std::min(
              frame_min_corridor,
              DistanceToSegment2D(*x, *y, robot_x, robot_y,
                                  waypoint_.point.x, waypoint_.point.y));
        }
      }
    } catch (const std::runtime_error& error) {
      ROS_WARN_THROTTLE(2.0, "Dynamic cloud has no float x/y fields: %s",
                        error.what());
      SetDynamicEncounter(false);
      return;
    }

    current_dynamic_robot_distance_ = frame_min_robot;
    current_dynamic_corridor_distance_ = frame_min_corridor;
    minimum_dynamic_robot_distance_ = std::min(
        minimum_dynamic_robot_distance_, frame_min_robot);
    minimum_dynamic_corridor_distance_ = std::min(
        minimum_dynamic_corridor_distance_, frame_min_corridor);
    if (std::isfinite(frame_min_robot)) ++dynamic_cloud_nonempty_samples_;

    const bool encounter = valid_segment &&
        frame_min_corridor <= dynamic_corridor_distance_;
    SetDynamicEncounter(encounter);
  }

  void SetDynamicEncounter(bool encounter) {
    if (encounter) {
      ++dynamic_encounter_samples_;
      if (!dynamic_encounter_) ++dynamic_encounter_events_;
    }
    dynamic_encounter_ = encounter;
  }

  void WaypointCallback(const geometry_msgs::PointStampedConstPtr& message) {
    if (!have_goal_) return;
    if (have_waypoint_) {
      const double jump = Distance2D(waypoint_.point.x, waypoint_.point.y,
                                     message->point.x, message->point.y);
      maximum_waypoint_jump_ = std::max(maximum_waypoint_jump_, jump);
      if (jump > waypoint_change_distance_) ++waypoint_change_count_;
    }
    waypoint_ = *message;
    have_waypoint_ = true;
  }

  void PathCallback(const nav_msgs::PathConstPtr& message) {
    if (!have_goal_) return;
    path_point_count_ = message->poses.size();
    have_path_ = true;
    if (message->poses.size() < 2) return;

    const auto& origin = message->poses.front().pose.position;
    bool found_heading = false;
    double heading = 0.0;
    for (std::size_t i = 1; i < message->poses.size(); ++i) {
      const auto& point = message->poses[i].pose.position;
      if (Distance2D(origin.x, origin.y, point.x, point.y) >=
          minimum_path_heading_baseline_) {
        heading = std::atan2(point.y - origin.y, point.x - origin.x);
        found_heading = true;
        break;
      }
    }
    if (!found_heading) return;

    if (have_path_heading_) {
      maximum_path_heading_jump_ = std::max(
          maximum_path_heading_jump_,
          std::fabs(NormalizeAngle(heading - path_heading_)));
    }
    path_heading_ = heading;
    have_path_heading_ = true;
  }

  void ReachedCallback(const std_msgs::BoolConstPtr& message) {
    if (!have_goal_) return;
    reached_ = message->data;
    if (!message->data || reached_ever_) return;
    reached_ever_ = true;
    reach_wall_time_ = ros::WallTime::now();
    if (have_goal_ && have_odom_) {
      reached_goal_error_ = Distance2D(
          odom_.pose.pose.position.x, odom_.pose.pose.position.y,
          goal_.point.x, goal_.point.y);
      false_reach_ = reached_goal_error_ > goal_tolerance_;
      ROS_INFO_STREAM("Reached=true at goal error " << reached_goal_error_
                      << " m (" << PassFail(!false_reach_) << ").");
    } else {
      false_reach_ = true;
      ROS_ERROR("Reached=true arrived before goal/odometry was available.");
    }
  }

  void CmdCallback(const geometry_msgs::TwistConstPtr& message) {
    current_cmd_linear_ = message->linear.x;
    current_cmd_angular_ = message->angular.z;
    if (!have_goal_) return;
    maximum_abs_linear_speed_ = std::max(
        maximum_abs_linear_speed_, std::fabs(current_cmd_linear_));
    maximum_abs_angular_speed_ = std::max(
        maximum_abs_angular_speed_, std::fabs(current_cmd_angular_));
  }

  void SampleTimerCallback(const ros::WallTimerEvent&) {
    if (finalized_) return;
    WriteCsvRow();

    const double elapsed = have_goal_
        ? (ros::WallTime::now() - goal_receipt_wall_time_).toSec() : 0.0;
    if (have_goal_ && maximum_duration_ > 0.0 &&
        elapsed >= maximum_duration_) {
      ROS_WARN("Navigation demo monitor reached maximum_duration.");
      ros::shutdown();
      return;
    }
    if (auto_finish_on_reach_ && reached_ever_ && !reach_wall_time_.isZero() &&
        (ros::WallTime::now() - reach_wall_time_).toSec() >=
            post_reach_duration_) {
      ROS_INFO("Navigation demo monitor captured post-reach samples; stopping.");
      ros::shutdown();
    }
  }

  void StatusTimerCallback(const ros::WallTimerEvent&) {
    if (!have_goal_) {
      ROS_INFO_THROTTLE(5.0, "Demo monitor is waiting for /goal_point.");
      return;
    }
    const double goal_distance = CurrentGoalDistance();
    ROS_INFO_STREAM("DEMO status: goal_state="
                    << GoalMapStateName(current_goal_map_state_)
                    << " goal_distance=" << goal_distance
                    << "m dynamic_points=" << dynamic_point_count_
                    << " corridor_distance="
                    << FiniteOrMinusOne(current_dynamic_corridor_distance_)
                    << "m waypoint_changes=" << waypoint_change_count_
                    << " path_points=" << path_point_count_
                    << " reached=" << (reached_ ? "true" : "false"));
  }

  double CurrentGoalDistance() const {
    if (!have_goal_ || !have_odom_) return -1.0;
    return Distance2D(odom_.pose.pose.position.x, odom_.pose.pose.position.y,
                      goal_.point.x, goal_.point.y);
  }

  static double FiniteOrMinusOne(double value) {
    return std::isfinite(value) ? value : -1.0;
  }

  void WriteCsvRow() {
    if (!csv_.is_open()) return;
    constexpr double kRadiansToDegrees = 57.2957795130823208768;
    const double elapsed = have_goal_
        ? (ros::WallTime::now() - goal_receipt_wall_time_).toSec() : -1.0;
    csv_ << elapsed << ',' << ros::Time::now().toSec() << ','
         << GoalMapStateName(current_goal_map_state_) << ','
         << (have_odom_ ? odom_.pose.pose.position.x : 0.0) << ','
         << (have_odom_ ? odom_.pose.pose.position.y : 0.0) << ','
         << (have_odom_ ? odom_.pose.pose.position.z : 0.0) << ','
         << current_roll_ * kRadiansToDegrees << ','
         << current_pitch_ * kRadiansToDegrees << ','
         << (have_goal_ ? goal_.point.x : 0.0) << ','
         << (have_goal_ ? goal_.point.y : 0.0) << ','
         << (have_goal_ ? goal_.point.z : 0.0) << ','
         << CurrentGoalDistance() << ','
         << (have_waypoint_ ? waypoint_.point.x : 0.0) << ','
         << (have_waypoint_ ? waypoint_.point.y : 0.0) << ','
         << (have_waypoint_ ? waypoint_.point.z : 0.0) << ','
         << waypoint_change_count_ << ',' << dynamic_point_count_ << ','
         << (dynamic_encounter_ ? 1 : 0) << ','
         << FiniteOrMinusOne(current_dynamic_robot_distance_) << ','
         << FiniteOrMinusOne(current_dynamic_corridor_distance_) << ','
         << path_point_count_ << ','
         << (have_path_heading_ ? path_heading_ : 0.0) << ','
         << current_cmd_linear_ << ',' << current_cmd_angular_ << ','
         << (reached_ ? 1 : 0) << '\n';
    csv_.flush();
  }

  std::string Evaluation(bool applicable, bool pass) const {
    if (!applicable) return "NOT_EVALUATED";
    return PassFail(pass);
  }

  void Finalize() {
    if (finalized_) return;
    finalized_ = true;
    if (csv_.is_open()) {
      WriteCsvRow();
      csv_.close();
    }

    const bool initial_unknown_pass = have_initial_goal_map_state_ &&
        initial_goal_map_state_ == GoalMapState::kUnknown;
    const bool interaction_pass = dynamic_encounter_events_ > 0;
    const bool clearance_evaluated =
        std::isfinite(minimum_dynamic_robot_distance_);
    const bool clearance_pass = clearance_evaluated &&
        minimum_dynamic_robot_distance_ >= minimum_safe_distance_;
    const bool goal_pass = have_goal_ && goal_change_count_ == 0 && !false_reach_;
    const bool reach_pass = reached_ever_ && !false_reach_ &&
        std::isfinite(reached_goal_error_) &&
        reached_goal_error_ <= goal_tolerance_;
    const bool posture_pass = have_odom_ && !posture_violation_;

    bool overall_pass = have_goal_ && have_odom_ && have_map_ && goal_pass &&
                        posture_pass && reach_pass;
    if (require_initial_unknown_) overall_pass &= initial_unknown_pass;
    if (require_dynamic_interaction_) overall_pass &= interaction_pass;
    if (clearance_evaluated) overall_pass &= clearance_pass;

    std::ostringstream summary;
    summary << std::fixed << std::setprecision(3);
    summary << "FAR navigation demo summary\n"
            << "overall: " << PassFail(overall_pass) << "\n"
            << "csv: " << output_csv_ << "\n"
            << "wall_duration_s: "
            << (have_goal_
                    ? (ros::WallTime::now() - goal_receipt_wall_time_).toSec()
                    : 0.0)
            << "\n"
            << "goal_received: " << (have_goal_ ? "yes" : "no") << "\n"
            << "goal_xyz: " << goal_.point.x << ", " << goal_.point.y
            << ", " << goal_.point.z << "\n"
            << "initial_goal_distance_m: "
            << FiniteOrMinusOne(initial_goal_distance_) << "\n"
            << "initial_goal_map_state: "
            << GoalMapStateName(initial_goal_map_state_) << "\n"
            << "initial_unknown_check: "
            << Evaluation(have_initial_goal_map_state_, initial_unknown_pass)
            << "\n"
            << "goal_became_known: "
            << (goal_became_known_ ? "yes" : "no") << "\n"
            << "goal_known_wall_elapsed_s: "
            << FiniteOrMinusOne(goal_known_wall_elapsed_) << "\n"
            << "goal_message_count: " << goal_message_count_ << "\n"
            << "goal_change_count: " << goal_change_count_ << "\n"
            << "waypoint_change_count: " << waypoint_change_count_ << "\n"
            << "maximum_waypoint_jump_m: " << maximum_waypoint_jump_ << "\n"
            << "maximum_path_heading_jump_rad: "
            << maximum_path_heading_jump_ << "\n"
            << "dynamic_nonempty_samples: "
            << dynamic_cloud_nonempty_samples_ << "\n"
            << "dynamic_encounter_events: " << dynamic_encounter_events_
            << "\n"
            << "dynamic_encounter_samples: " << dynamic_encounter_samples_
            << "\n"
            << "dynamic_interaction_check: "
            << Evaluation(true, interaction_pass) << "\n"
            << "minimum_dynamic_robot_distance_m: "
            << FiniteOrMinusOne(minimum_dynamic_robot_distance_) << "\n"
            << "minimum_dynamic_corridor_distance_m: "
            << FiniteOrMinusOne(minimum_dynamic_corridor_distance_) << "\n"
            << "clearance_check: "
            << Evaluation(clearance_evaluated, clearance_pass) << "\n"
            << "maximum_abs_linear_speed_mps: "
            << maximum_abs_linear_speed_ << "\n"
            << "maximum_abs_angular_speed_radps: "
            << maximum_abs_angular_speed_ << "\n"
            << "minimum_robot_z_m: " << FiniteOrMinusOne(minimum_robot_z_)
            << "\n"
            << "maximum_abs_roll_deg: " << maximum_abs_roll_deg_ << "\n"
            << "maximum_abs_pitch_deg: " << maximum_abs_pitch_deg_ << "\n"
            << "posture_check: " << Evaluation(have_odom_, posture_pass)
            << "\n"
            << "reached_received: " << (reached_ever_ ? "yes" : "no")
            << "\n"
            << "reached_goal_error_m: "
            << FiniteOrMinusOne(reached_goal_error_) << "\n"
            << "false_reach: " << (false_reach_ ? "yes" : "no") << "\n"
            << "goal_integrity_check: " << Evaluation(have_goal_, goal_pass)
            << "\n"
            << "reach_check: " << Evaluation(true, reach_pass) << "\n";

    std::ofstream summary_file(output_summary_);
    if (summary_file.is_open()) summary_file << summary.str();
    ROS_INFO_STREAM("\n" << summary.str());
    ROS_INFO_STREAM("Demo summary written to: " << output_summary_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber goal_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber dynamic_sub_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber reached_sub_;
  ros::Subscriber cmd_sub_;
  ros::WallTimer sample_timer_;
  ros::WallTimer status_timer_;

  std::string goal_topic_;
  std::string odom_topic_;
  std::string octomap_topic_;
  std::string dynamic_cloud_topic_;
  std::string waypoint_topic_;
  std::string path_topic_;
  std::string reached_topic_;
  std::string cmd_vel_topic_;
  std::string output_csv_;
  std::string output_summary_;

  double sample_rate_ = 5.0;
  double status_period_ = 2.0;
  double map_check_period_ = 1.0;
  double goal_vertical_probe_ = 0.8;
  double goal_tolerance_ = 0.5;
  double dynamic_corridor_distance_ = 0.6;
  double minimum_safe_distance_ = 0.6;
  double waypoint_change_distance_ = 0.2;
  double minimum_path_heading_baseline_ = 0.2;
  double maximum_roll_pitch_deg_ = 25.0;
  double minimum_robot_height_ = 0.2;
  double maximum_duration_ = 0.0;
  double post_reach_duration_ = 2.0;
  bool auto_finish_on_reach_ = true;
  bool require_initial_unknown_ = true;
  bool require_dynamic_interaction_ = true;

  ros::WallTime start_wall_time_;
  ros::WallTime goal_receipt_wall_time_;
  ros::WallTime reach_wall_time_;
  ros::WallTime last_map_parse_wall_time_;
  octomap_msgs::OctomapConstPtr latest_map_;
  std::ofstream csv_;

  geometry_msgs::PointStamped goal_;
  geometry_msgs::PointStamped waypoint_;
  nav_msgs::Odometry odom_;
  bool have_goal_ = false;
  bool have_odom_ = false;
  bool have_map_ = false;
  bool have_waypoint_ = false;
  bool have_path_ = false;
  bool have_path_heading_ = false;
  bool reached_ = false;
  bool reached_ever_ = false;
  bool false_reach_ = false;
  bool posture_violation_ = false;
  bool dynamic_encounter_ = false;
  bool finalized_ = false;

  GoalMapState initial_goal_map_state_ = GoalMapState::kUnavailable;
  GoalMapState current_goal_map_state_ = GoalMapState::kUnavailable;
  bool have_initial_goal_map_state_ = false;
  bool goal_became_known_ = false;
  double goal_known_wall_elapsed_ =
      std::numeric_limits<double>::quiet_NaN();
  double initial_goal_distance_ =
      std::numeric_limits<double>::quiet_NaN();
  double reached_goal_error_ =
      std::numeric_limits<double>::quiet_NaN();

  std::size_t goal_message_count_ = 0;
  std::size_t goal_change_count_ = 0;
  std::size_t waypoint_change_count_ = 0;
  std::size_t path_point_count_ = 0;
  std::size_t dynamic_point_count_ = 0;
  std::size_t dynamic_cloud_nonempty_samples_ = 0;
  std::size_t dynamic_encounter_events_ = 0;
  std::size_t dynamic_encounter_samples_ = 0;

  double current_roll_ = 0.0;
  double current_pitch_ = 0.0;
  double path_heading_ = 0.0;
  double maximum_path_heading_jump_ = 0.0;
  double maximum_waypoint_jump_ = 0.0;
  double current_dynamic_robot_distance_ =
      std::numeric_limits<double>::infinity();
  double current_dynamic_corridor_distance_ =
      std::numeric_limits<double>::infinity();
  double minimum_dynamic_robot_distance_;
  double minimum_dynamic_corridor_distance_;
  double current_cmd_linear_ = 0.0;
  double current_cmd_angular_ = 0.0;
  double maximum_abs_linear_speed_ = 0.0;
  double maximum_abs_angular_speed_ = 0.0;
  double minimum_robot_z_ = std::numeric_limits<double>::infinity();
  double maximum_abs_roll_deg_ = 0.0;
  double maximum_abs_pitch_deg_ = 0.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation_demo_monitor");
  try {
    NavigationDemoMonitor monitor;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Navigation demo monitor failed: %s", error.what());
    return 1;
  }
  return 0;
}
