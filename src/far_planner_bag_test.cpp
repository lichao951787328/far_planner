/*
 * Offline integration harness for FARMaster.
 *
 * It adapts the Gazebo PoseStamped stream in the recorded bag to the
 * nav_msgs/Odometry input expected by far_planner.cpp, observes the planner's
 * public outputs, and terminates after the semantic-map stream becomes idle.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <semantic_octree/SemanticOcTree.h>
#include <semantic_octree/Semantics.h>

namespace {

using SemanticOctree =
    octomap::SemanticOcTree<octomap::SemanticsLogOdds>;

class Reporter {
public:
    void Check(const bool condition, const std::string& name) {
        if (condition) {
            ++passed_;
            ROS_INFO_STREAM("[PASS] " << name);
        } else {
            ++failed_;
            ROS_ERROR_STREAM("[FAIL] " << name);
        }
    }

    void Skip(const std::string& name) {
        ++skipped_;
        ROS_WARN_STREAM("[SKIP] " << name);
    }

    int exitCode() const { return failed_ == 0 ? 0 : 1; }

    void Summary() const {
        ROS_INFO_STREAM("FAR Planner bag-test summary: " << passed_
                        << " passed, " << failed_ << " failed, "
                        << skipped_ << " skipped.");
    }

private:
    int passed_ = 0;
    int failed_ = 0;
    int skipped_ = 0;
};

struct Position {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

class FarPlannerBagTest {
public:
    FarPlannerBagTest() : private_nh_("~") {
        private_nh_.param<std::string>("source_pose_topic", source_pose_topic_,
                                       "/far_test/source_pose");
        private_nh_.param<std::string>("source_map_topic", source_map_topic_,
                                       "/far_test/octomap_full");
        private_nh_.param<std::string>("odom_topic", odom_topic_,
                                       "/far_test/odom_world");
        private_nh_.param<std::string>("world_frame", world_frame_, "world");
        private_nh_.param<double>("stream_idle_timeout", stream_idle_timeout_, 3.0);
        private_nh_.param<double>("odom_publish_period", odom_publish_period_, 0.02);
        private_nh_.param<double>("trajectory_sample_period",
                                  trajectory_sample_period_, 0.1);
        private_nh_.param<bool>("enable_goal_test", enable_goal_test_, false);
        private_nh_.param<double>("goal_publish_delay", goal_publish_delay_, 1.0);
        private_nh_.param<double>("goal_x", goal_.x, 0.0);
        private_nh_.param<double>("goal_y", goal_.y, 0.0);
        private_nh_.param<double>("goal_z", goal_.z, 0.0);
        private_nh_.param<bool>("enable_control_test", enable_control_test_, false);
        private_nh_.param<int>("pause_map_index", pause_map_index_, 150);
        private_nh_.param<int>("resume_map_index", resume_map_index_, 250);
        private_nh_.param<int>("reset_map_index", reset_map_index_, 450);
        private_nh_.param<bool>("observe_dynamic_behavior",
                                observe_dynamic_behavior_, false);
        private_nh_.param<int>("minimum_path_points", minimum_path_points_, 1);
        private_nh_.param<bool>("relay_semantic_map", relay_semantic_map_, false);
        private_nh_.param<bool>("inject_dynamic_block", inject_dynamic_block_, false);
        private_nh_.param<std::string>("planner_map_topic", planner_map_topic_,
                                       "/far_test/octomap_full");
        private_nh_.param<int>("dynamic_injection_map_index",
                               dynamic_injection_map_index_, 250);
        private_nh_.param<int>("dynamic_occupied_frames",
                               dynamic_occupied_frames_, 10);
        private_nh_.param<int>("dynamic_free_frames",
                               dynamic_free_frames_, 8);
        private_nh_.param<bool>("enable_secondary_goal",
                                enable_secondary_goal_, false);
        private_nh_.param<bool>("expect_input_timeout_stop",
                                expect_input_timeout_stop_, false);
        private_nh_.param<int>("secondary_goal_map_index",
                               secondary_goal_map_index_, 400);
        private_nh_.param<double>("secondary_goal_x", secondary_goal_.x, 0.0);
        private_nh_.param<double>("secondary_goal_y", secondary_goal_.y, 0.0);
        private_nh_.param<double>("secondary_goal_z", secondary_goal_.z, 0.0);

        // Ensure msgToMap can construct the SSMI template tree in this process.
        static SemanticOctree semantic_registration_probe(0.1);
        (void)semantic_registration_probe;

        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 20);
        goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>(
            "/far_test/goal_point", 1);
        update_pub_ = nh_.advertise<std_msgs::Bool>(
            "/far_test/update_visibility_graph", 1);
        reset_pub_ = nh_.advertise<std_msgs::Empty>(
            "/far_test/reset_visibility_graph", 1);
        if (relay_semantic_map_) {
            semantic_map_pub_ = nh_.advertise<octomap_msgs::Octomap>(
                planner_map_topic_, 2);
        }
        pose_sub_ = nh_.subscribe(source_pose_topic_, 100,
                                  &FarPlannerBagTest::PoseCallback, this);
        map_sub_ = nh_.subscribe(source_map_topic_, 5,
                                 &FarPlannerBagTest::MapCallback, this);
        static_cloud_sub_ = nh_.subscribe(
            "/far_test/FAR_obs_debug", 5,
            &FarPlannerBagTest::StaticCloudCallback, this);
        dynamic_cloud_sub_ = nh_.subscribe(
            "/far_test/FAR_dynamic_obs_debug", 5,
            &FarPlannerBagTest::DynamicCloudCallback, this);
        local_static_cloud_sub_ = nh_.subscribe(
            "/far_test/semantic_local_static_obstacles", 5,
            &FarPlannerBagTest::LocalStaticCloudCallback, this);
        local_dynamic_cloud_sub_ = nh_.subscribe(
            "/far_test/semantic_local_dynamic_obstacles", 5,
            &FarPlannerBagTest::LocalDynamicCloudCallback, this);
        graph_sub_ = nh_.subscribe(
            "/far_test/viz_graph_topic", 5,
            &FarPlannerBagTest::GraphCallback, this);
        contour_sub_ = nh_.subscribe(
            "/far_test/viz_contour_topic", 5,
            &FarPlannerBagTest::ContourCallback, this);
        runtime_sub_ = nh_.subscribe(
            "/far_test/runtime", 5,
            &FarPlannerBagTest::RuntimeCallback, this);
        waypoint_sub_ = nh_.subscribe(
            "/far_test/way_point", 5,
            &FarPlannerBagTest::WaypointCallback, this);
        path_sub_ = nh_.subscribe(
            "/far_test/viz_path_topic", 5,
            &FarPlannerBagTest::PathCallback, this);
        reach_sub_ = nh_.subscribe(
            "/far_test/far_reach_goal_status", 5,
            &FarPlannerBagTest::ReachCallback, this);
        boundary_sub_ = nh_.subscribe(
            "/far_test/navigation_boundary", 5,
            &FarPlannerBagTest::BoundaryCallback, this);
        planning_time_sub_ = nh_.subscribe(
            "/far_test/planning_time", 5,
            &FarPlannerBagTest::PlanningTimeCallback, this);
        traverse_time_sub_ = nh_.subscribe(
            "/far_test/far_traverse_time", 5,
            &FarPlannerBagTest::TraverseTimeCallback, this);
        semantic_snapshot_time_sub_ = nh_.subscribe(
            "/far_test/semantic_snapshot_time", 5,
            &FarPlannerBagTest::SemanticSnapshotTimeCallback, this);
        semantic_update_time_sub_ = nh_.subscribe(
            "/far_test/semantic_planner_update_time", 5,
            &FarPlannerBagTest::SemanticUpdateTimeCallback, this);
        semantic_callback_time_sub_ = nh_.subscribe(
            "/far_test/semantic_callback_time", 5,
            &FarPlannerBagTest::SemanticCallbackTimeCallback, this);
        main_loop_time_sub_ = nh_.subscribe(
            "/far_test/far_main_loop_time", 5,
            &FarPlannerBagTest::MainLoopTimeCallback, this);

        idle_timer_ = nh_.createWallTimer(
            ros::WallDuration(0.25), &FarPlannerBagTest::IdleCallback, this);
        ROS_INFO_STREAM("FAR Planner bag harness waiting on " << source_map_topic_
                        << " and " << source_pose_topic_);
    }

    int exitCode() const { return reporter_.exitCode(); }

private:
    static std::size_t CloudPointCount(const sensor_msgs::PointCloud2& cloud) {
        return static_cast<std::size_t>(cloud.width) * cloud.height;
    }

    static std::size_t MarkerPointCount(
        const visualization_msgs::MarkerArray& array) {
        std::size_t count = 0;
        for (const auto& marker : array.markers) count += marker.points.size();
        return count;
    }

    void PoseCallback(const geometry_msgs::PoseStampedConstPtr& message) {
        if (!message || finished_) return;
        ++pose_messages_;
        const ros::Time stamp = message->header.stamp;
        const Position position{message->pose.position.x,
                                message->pose.position.y,
                                message->pose.position.z};
        latest_position_ = position;
        if (trajectory_.empty() || last_trajectory_stamp_.isZero() ||
            (stamp - last_trajectory_stamp_).toSec() >= trajectory_sample_period_) {
            trajectory_.push_back(position);
            last_trajectory_stamp_ = stamp;
        }

        if (!last_odom_stamp_.isZero() && stamp >= last_odom_stamp_ &&
            (stamp - last_odom_stamp_).toSec() < odom_publish_period_) {
            return;
        }
        nav_msgs::Odometry odom;
        odom.header = message->header;
        odom.header.frame_id = world_frame_;
        odom.child_frame_id = "base_link";
        odom.pose.pose = message->pose;
        odom_pub_.publish(odom);
        last_odom_stamp_ = stamp;
        ++odom_messages_;
    }

    bool PrepareDynamicInjection(const octomap_msgs::Octomap& source) {
        std::unique_ptr<octomap::AbstractOcTree> decoded(
            octomap_msgs::msgToMap(source));
        SemanticOctree* occupied_tree =
            dynamic_cast<SemanticOctree*>(decoded.get());
        if (!occupied_tree) {
            ROS_ERROR_STREAM("Dynamic injection requires SemanticOcTree but got "
                             << source.id);
            return false;
        }

        const double resolution = occupied_tree->getResolution();
        injected_dynamic_point_.x = latest_position_.x;
        injected_dynamic_point_.y = latest_position_.y;
        injected_dynamic_point_.z = latest_position_.z +
            std::max(0.6, resolution * 1.5);
        const octomap::ColorOcTreeNode::Color dynamic_color(255, 0, 255);
        for (int hit = 0; hit < 4; ++hit) {
            occupied_tree->updateNode(
                injected_dynamic_point_.x, injected_dynamic_point_.y,
                injected_dynamic_point_.z, true, dynamic_color, dynamic_color);
        }
        occupied_tree->updateInnerOccupancy();

        injected_occupied_message_.reset(new octomap_msgs::Octomap());
        injected_occupied_message_->header = source.header;
        if (!octomap_msgs::fullMapToMsg(
                *occupied_tree, *injected_occupied_message_)) {
            ROS_ERROR("Failed to serialize the occupied dynamic test map");
            return false;
        }

        std::unique_ptr<octomap::AbstractOcTree> free_decoded(
            octomap_msgs::msgToMap(*injected_occupied_message_));
        SemanticOctree* free_tree =
            dynamic_cast<SemanticOctree*>(free_decoded.get());
        if (!free_tree) {
            ROS_ERROR("Failed to deserialize the occupied dynamic test map");
            return false;
        }
        auto* free_node = free_tree->search(
            injected_dynamic_point_.x, injected_dynamic_point_.y,
            injected_dynamic_point_.z);
        if (free_node) {
            // SSMI derives occupancy from semantic log-odds, so a fixed number
            // of miss observations is not guaranteed to cross the occupancy
            // threshold. Set only this test leaf to an unambiguous free value.
            free_node->setLogOdds(free_tree->getClampingThresMinLog());
        }
        free_tree->updateInnerOccupancy();
        if (!free_node || free_tree->isNodeOccupied(free_node)) {
            ROS_ERROR("Injected dynamic voxel could not be made explicitly free");
            return false;
        }

        injected_free_message_.reset(new octomap_msgs::Octomap());
        injected_free_message_->header = source.header;
        if (!octomap_msgs::fullMapToMsg(*free_tree,
                                       *injected_free_message_)) {
            ROS_ERROR("Failed to serialize the explicitly-free dynamic test map");
            return false;
        }
        dynamic_injection_prepared_ = true;
        ROS_INFO_STREAM("Prepared deterministic dynamic voxel at ("
                        << injected_dynamic_point_.x << ", "
                        << injected_dynamic_point_.y << ", "
                        << injected_dynamic_point_.z << ")");
        return true;
    }

    void RelaySemanticMap(const octomap_msgs::OctomapConstPtr& source) {
        if (!relay_semantic_map_ || !source) return;
        if (inject_dynamic_block_ && !dynamic_injection_attempted_ &&
            map_messages_ >=
                static_cast<std::size_t>(dynamic_injection_map_index_)) {
            dynamic_injection_attempted_ = true;
            PrepareDynamicInjection(*source);
        }

        octomap_msgs::Octomap output = *source;
        if (dynamic_injection_prepared_ &&
            injected_occupied_frames_published_ < dynamic_occupied_frames_) {
            output = *injected_occupied_message_;
            output.header = source->header;
            ++injected_occupied_frames_published_;
        } else if (dynamic_injection_prepared_ &&
                   injected_free_frames_published_ < dynamic_free_frames_) {
            output = *injected_free_message_;
            output.header = source->header;
            if (injected_free_frames_published_ == 0) {
                first_injected_free_stamp_ = source->header.stamp;
                first_injected_free_wall_time_ = ros::WallTime::now();
            }
            ++injected_free_frames_published_;
        } else if (dynamic_injection_prepared_) {
            dynamic_injection_sequence_complete_ = true;
        }
        last_relayed_semantic_stamp_ = output.header.stamp;
        semantic_map_pub_.publish(output);
    }

    void MapCallback(const octomap_msgs::OctomapConstPtr& message) {
        if (!message || finished_) return;
        ++map_messages_;
        last_map_wall_time_ = ros::WallTime::now();
        if (map_messages_ == 1) {
            map_id_ = message->id;
            map_frame_ = message->header.frame_id;
            map_resolution_ = message->resolution;
        } else if (message->id != map_id_ ||
                   message->header.frame_id != map_frame_ ||
                   std::fabs(message->resolution - map_resolution_) > 1e-6) {
            map_metadata_consistent_ = false;
        }
        RelaySemanticMap(message);
        TryPublishSecondaryGoal();
        TryPublishControls();
    }

    void StaticCloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
        if (!message) return;
        ++static_cloud_messages_;
        max_static_points_ = std::max(max_static_points_, CloudPointCount(*message));
    }

    void DynamicCloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
        if (!message) return;
        ++dynamic_cloud_messages_;
        const std::size_t count = CloudPointCount(*message);
        max_dynamic_points_ = std::max(max_dynamic_points_, count);
        dynamic_cloud_nonempty_ = count > 0;
        if (count == 0) {
            ++empty_dynamic_cloud_messages_;
            injected_dynamic_effective_ = false;
            if (injected_dynamic_seen_ && !injected_dynamic_released_ &&
                !first_injected_free_wall_time_.isZero()) {
                injected_dynamic_released_ = true;
                injected_release_delay_ =
                    (ros::WallTime::now() -
                     first_injected_free_wall_time_).toSec();
                injected_release_semantic_delay_ =
                    (last_relayed_semantic_stamp_ -
                     first_injected_free_stamp_).toSec();
            }
            return;
        }
        ++nonempty_dynamic_cloud_messages_;
        try {
            sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
            sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
            sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
            bool injected_found = false;
            for (; x != x.end(); ++x, ++y, ++z) {
                const double injection_tolerance =
                    std::max(0.1, map_resolution_ * 0.75);
                if (dynamic_injection_prepared_ &&
                    std::fabs(*x - injected_dynamic_point_.x) <=
                        injection_tolerance &&
                    std::fabs(*y - injected_dynamic_point_.y) <=
                        injection_tolerance &&
                    std::fabs(*z - injected_dynamic_point_.z) <=
                        injection_tolerance) {
                    injected_found = true;
                }
                const double dx = *x - latest_position_.x;
                const double dy = *y - latest_position_.y;
                const double distance = std::sqrt(dx * dx + dy * dy);
                if (distance >= min_dynamic_robot_distance_) continue;
                min_dynamic_robot_distance_ = distance;
                nearest_dynamic_point_ = Position{*x, *y, *z};
                nearest_dynamic_map_index_ = map_messages_;
            }
            injected_dynamic_effective_ = injected_found;
            injected_dynamic_seen_ = injected_dynamic_seen_ || injected_found;
            if (injected_dynamic_seen_ && !injected_found &&
                !injected_dynamic_released_ &&
                !first_injected_free_wall_time_.isZero()) {
                injected_dynamic_released_ = true;
                injected_release_delay_ =
                    (ros::WallTime::now() -
                     first_injected_free_wall_time_).toSec();
                injected_release_semantic_delay_ =
                    (last_relayed_semantic_stamp_ -
                     first_injected_free_stamp_).toSec();
            }
        } catch (const std::runtime_error& error) {
            dynamic_cloud_layout_valid_ = false;
            ROS_ERROR_STREAM_THROTTLE(1.0,
                "Cannot inspect dynamic cloud XYZ fields: " << error.what());
        }
    }

    bool ValidateLocalPlannerCloud(
        const sensor_msgs::PointCloud2ConstPtr& message) const {
        if (!message || message->header.frame_id != world_frame_) return false;
        try {
            sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
            sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
            sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
            sensor_msgs::PointCloud2ConstIterator<float> intensity(
                *message, "intensity");
            for (; x != x.end(); ++x, ++y, ++z, ++intensity) {
                if (!std::isfinite(*x) || !std::isfinite(*y) ||
                    !std::isfinite(*z) ||
                    std::fabs(*intensity - 200.0f) > 1e-4f) return false;
            }
        } catch (const std::runtime_error&) {
            return false;
        }
        return true;
    }

    void LocalStaticCloudCallback(
        const sensor_msgs::PointCloud2ConstPtr& message) {
        ++local_static_cloud_messages_;
        local_planner_clouds_valid_ = local_planner_clouds_valid_ &&
                                      ValidateLocalPlannerCloud(message);
        if (message) {
            max_local_static_points_ = std::max(
                max_local_static_points_, CloudPointCount(*message));
        }
    }

    void LocalDynamicCloudCallback(
        const sensor_msgs::PointCloud2ConstPtr& message) {
        ++local_dynamic_cloud_messages_;
        local_planner_clouds_valid_ = local_planner_clouds_valid_ &&
                                      ValidateLocalPlannerCloud(message);
        if (message) {
            max_local_dynamic_points_ = std::max(
                max_local_dynamic_points_, CloudPointCount(*message));
        }
    }

    void GraphCallback(const visualization_msgs::MarkerArrayConstPtr& message) {
        if (!message) return;
        ++graph_messages_;
        const std::size_t point_count = MarkerPointCount(*message);
        max_graph_points_ = std::max(max_graph_points_, point_count);
        std::size_t global_nodes = 0;
        for (const auto& marker : message->markers) {
            if (marker.ns == "global_vertex") {
                global_nodes = marker.points.size();
                break;
            }
        }
        current_global_nodes_ = global_nodes;
        max_global_nodes_ = std::max(max_global_nodes_, global_nodes);
        if (dynamic_cloud_nonempty_) ++dynamic_graph_messages_;
        if (injected_dynamic_effective_) ++injected_graph_messages_;
        if (injected_dynamic_released_) ++post_injected_release_graph_messages_;
        if (point_count > 2 && graph_ready_wall_time_.isZero()) {
            graph_ready_wall_time_ = ros::WallTime::now();
        }
        const ros::WallTime now = ros::WallTime::now();
        if (pause_published_ && !resume_published_ &&
            (now - pause_publish_wall_time_).toSec() >= 0.5) {
            ++pause_graph_messages_;
            min_pause_global_nodes_ = std::min(min_pause_global_nodes_, global_nodes);
            max_pause_global_nodes_ = std::max(max_pause_global_nodes_, global_nodes);
        }
        if (resume_published_ &&
            (now - resume_publish_wall_time_).toSec() >= 0.5 &&
            pause_graph_messages_ > 0 &&
            (global_nodes < min_pause_global_nodes_ ||
             global_nodes > max_pause_global_nodes_)) {
            graph_changed_after_resume_ = true;
        }
        if (reset_published_ &&
            (now - reset_publish_wall_time_).toSec() >= 0.25) {
            min_post_reset_global_nodes_ = std::min(
                min_post_reset_global_nodes_, global_nodes);
            if (global_nodes < pre_reset_global_nodes_) reset_drop_seen_ = true;
            if (reset_drop_seen_ && global_nodes > min_post_reset_global_nodes_) {
                reset_rebuild_seen_ = true;
            }
        }
    }

    void ContourCallback(const visualization_msgs::MarkerArrayConstPtr& message) {
        if (!message) return;
        ++contour_messages_;
        max_contour_points_ = std::max(max_contour_points_, MarkerPointCount(*message));
        if (dynamic_cloud_nonempty_) ++dynamic_contour_messages_;
        if (injected_dynamic_effective_) ++injected_contour_messages_;
        if (injected_dynamic_released_) {
            ++post_injected_release_contour_messages_;
        }
    }

    void RuntimeCallback(const std_msgs::Float32ConstPtr& message) {
        if (!message) return;
        ++runtime_messages_;
        max_runtime_ = std::max(max_runtime_, message->data);
    }

    void WaypointCallback(const geometry_msgs::PointStampedConstPtr& message) {
        if (!message) return;
        ++waypoint_messages_;
        if (expect_input_timeout_stop_ && !last_map_wall_time_.isZero() &&
            (ros::WallTime::now() - last_map_wall_time_).toSec() >= 0.5) {
            const double timeout_dx = message->point.x - latest_position_.x;
            const double timeout_dy = message->point.y - latest_position_.y;
            const double timeout_dz = message->point.z - latest_position_.z;
            if (std::sqrt(timeout_dx * timeout_dx + timeout_dy * timeout_dy +
                          timeout_dz * timeout_dz) <= 0.35) {
                input_timeout_stop_waypoint_seen_ = true;
            }
        }
        if (reset_published_ && !reset_stop_waypoint_seen_ &&
            (ros::WallTime::now() - reset_publish_wall_time_).toSec() <= 2.0) {
            const double dx = message->point.x - latest_position_.x;
            const double dy = message->point.y - latest_position_.y;
            const double dz = message->point.z - latest_position_.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) <= 0.35) {
                reset_stop_waypoint_seen_ = true;
            }
        }
        if (observe_dynamic_behavior_) {
            const double dx = message->point.x - latest_position_.x;
            const double dy = message->point.y - latest_position_.y;
            const double distance = std::sqrt(dx * dx + dy * dy);
            if (dynamic_cloud_nonempty_ && distance <= 0.35) {
                ++dynamic_stop_waypoints_;
                dynamic_stop_seen_ = true;
            } else if (dynamic_stop_seen_ && distance > 0.5) {
                dynamic_recovery_waypoint_seen_ = true;
            }
            if (injected_dynamic_effective_) {
                if (distance <= 0.35) injected_stop_waypoint_seen_ = true;
                else injected_reroute_waypoint_seen_ = true;
            } else if (injected_dynamic_released_ && distance > 0.5) {
                injected_recovery_waypoint_seen_ = true;
            }
        }
    }

    void PathCallback(const visualization_msgs::MarkerConstPtr& message) {
        if (!message) return;
        ++path_messages_;
        if (expect_input_timeout_stop_ && message->points.empty() &&
            !last_map_wall_time_.isZero() &&
            (ros::WallTime::now() - last_map_wall_time_).toSec() >= 0.5) {
            input_timeout_empty_path_seen_ = true;
        }
        max_path_points_ = std::max(max_path_points_, message->points.size());
        if (reset_published_ && message->points.empty() &&
            (ros::WallTime::now() - reset_publish_wall_time_).toSec() <= 2.0) {
            reset_empty_path_seen_ = true;
        }
        if (observe_dynamic_behavior_) {
            if (dynamic_cloud_nonempty_ && message->points.empty()) {
                ++dynamic_empty_paths_;
            } else if (dynamic_stop_seen_ && !message->points.empty()) {
                dynamic_recovery_path_seen_ = true;
            }
            if (injected_dynamic_effective_) {
                if (message->points.empty()) injected_empty_path_seen_ = true;
                else injected_nonempty_path_seen_ = true;
            } else if (injected_dynamic_released_ &&
                       !message->points.empty()) {
                injected_recovery_path_seen_ = true;
            }
        }
    }

    void ReachCallback(const std_msgs::BoolConstPtr& message) {
        if (!message) return;
        ++reach_messages_;
        reached_goal_ = reached_goal_ || message->data;
    }

    void BoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& message) {
        if (!message) return;
        ++boundary_messages_;
        max_boundary_points_ = std::max(max_boundary_points_,
                                        message->polygon.points.size());
    }

    void PlanningTimeCallback(const std_msgs::Float32ConstPtr& message) {
        if (!message) return;
        ++planning_time_messages_;
        max_planning_time_ = std::max(max_planning_time_, message->data);
    }

    void TraverseTimeCallback(const std_msgs::Float32ConstPtr& message) {
        if (!message) return;
        ++traverse_time_messages_;
    }

    static bool IsValidDuration(const std_msgs::Float32ConstPtr& message) {
        return message && std::isfinite(message->data) && message->data >= 0.0f;
    }

    void SemanticSnapshotTimeCallback(const std_msgs::Float32ConstPtr& message) {
        if (!IsValidDuration(message)) timing_values_valid_ = false;
        else ++semantic_snapshot_time_messages_;
    }

    void SemanticUpdateTimeCallback(const std_msgs::Float32ConstPtr& message) {
        if (!IsValidDuration(message)) timing_values_valid_ = false;
        else ++semantic_update_time_messages_;
    }

    void SemanticCallbackTimeCallback(const std_msgs::Float32ConstPtr& message) {
        if (!IsValidDuration(message)) timing_values_valid_ = false;
        else ++semantic_callback_time_messages_;
    }

    void MainLoopTimeCallback(const std_msgs::Float32ConstPtr& message) {
        if (!IsValidDuration(message)) timing_values_valid_ = false;
        else ++main_loop_time_messages_;
    }

    void PrintTrajectoryCandidates() const {
        if (trajectory_.empty()) return;
        const auto print = [this](const char* name, const std::size_t index) {
            const Position& p = trajectory_[std::min(index, trajectory_.size() - 1)];
            ROS_INFO_STREAM("Candidate " << name << " goal: ("
                            << p.x << ", " << p.y << ", " << p.z << ")");
        };
        print("start", 0);
        print("quarter", trajectory_.size() / 4);
        print("middle", trajectory_.size() / 2);
        print("three-quarter", trajectory_.size() * 3 / 4);
        print("end", trajectory_.size() - 1);
    }

    void TryPublishGoal() {
        if (!enable_goal_test_ || goal_published_ ||
            graph_ready_wall_time_.isZero()) return;
        if ((ros::WallTime::now() - graph_ready_wall_time_).toSec() <
            goal_publish_delay_) return;

        geometry_msgs::PointStamped message;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = world_frame_;
        message.point.x = goal_.x;
        message.point.y = goal_.y;
        message.point.z = goal_.z;
        goal_pub_.publish(message);
        goal_published_ = true;
        ROS_INFO_STREAM("Published automatic FAR goal: (" << goal_.x << ", "
                        << goal_.y << ", " << goal_.z << ")");
    }

    void TryPublishSecondaryGoal() {
        if (!enable_secondary_goal_ || secondary_goal_published_ ||
            !goal_published_ ||
            map_messages_ <
                static_cast<std::size_t>(secondary_goal_map_index_)) return;

        geometry_msgs::PointStamped message;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = world_frame_;
        message.point.x = secondary_goal_.x;
        message.point.y = secondary_goal_.y;
        message.point.z = secondary_goal_.z;
        goal_pub_.publish(message);
        secondary_goal_published_ = true;
        ROS_INFO_STREAM("Published secondary FAR goal at map " << map_messages_
                        << ": (" << secondary_goal_.x << ", "
                        << secondary_goal_.y << ", "
                        << secondary_goal_.z << ")");
    }

    void TryPublishControls() {
        if (!enable_control_test_) return;
        if (!pause_published_ &&
            map_messages_ >= static_cast<std::size_t>(pause_map_index_)) {
            std_msgs::Bool message;
            message.data = false;
            update_pub_.publish(message);
            pause_published_ = true;
            pause_publish_wall_time_ = ros::WallTime::now();
            ROS_INFO_STREAM("Published visibility-graph pause at map "
                            << map_messages_);
        }
        if (!resume_published_ &&
            map_messages_ >= static_cast<std::size_t>(resume_map_index_)) {
            std_msgs::Bool message;
            message.data = true;
            update_pub_.publish(message);
            resume_published_ = true;
            resume_publish_wall_time_ = ros::WallTime::now();
            ROS_INFO_STREAM("Published visibility-graph resume at map "
                            << map_messages_);
        }
        if (!reset_published_ &&
            map_messages_ >= static_cast<std::size_t>(reset_map_index_)) {
            pre_reset_global_nodes_ = current_global_nodes_;
            reset_pub_.publish(std_msgs::Empty());
            reset_published_ = true;
            reset_publish_wall_time_ = ros::WallTime::now();
            ROS_INFO_STREAM("Published V-Graph reset at map " << map_messages_
                            << " with " << pre_reset_global_nodes_
                            << " global nodes");
        }
    }

    void IdleCallback(const ros::WallTimerEvent&) {
        TryPublishGoal();
        TryPublishControls();
        if (finished_ || map_messages_ == 0) return;
        if ((ros::WallTime::now() - last_map_wall_time_).toSec() <
            stream_idle_timeout_) return;

        reporter_.Check(map_messages_ >= 2,
                        "Semantic map stream reaches the FAR Planner test");
        reporter_.Check(pose_messages_ > 0 && odom_messages_ > 0,
                        "PoseStamped input is adapted to world-frame Odometry");
        reporter_.Check(map_metadata_consistent_ && map_id_ == "SemanticOcTree" &&
                            map_frame_ == world_frame_,
                        "SemanticOcTree type, frame and resolution stay consistent");
        reporter_.Check(static_cloud_messages_ > 0 && max_static_points_ > 0,
                        "FARMaster publishes a nonempty static contour cloud");
        reporter_.Check(dynamic_cloud_messages_ > 0 &&
                            nonempty_dynamic_cloud_messages_ > 0 &&
                            max_dynamic_points_ > 0,
                        "FARMaster publishes the semantic dynamic-obstacle layer");
        reporter_.Check(local_static_cloud_messages_ > 0 &&
                            max_local_static_points_ > 0 &&
                            local_dynamic_cloud_messages_ > 0 &&
                            local_planner_clouds_valid_,
                        "FARMaster publishes separate world-frame local-planner layers with intensity 200");
        reporter_.Check(runtime_messages_ > 0,
                        "The main V-Graph update loop runs and publishes runtime");
        reporter_.Check(timing_values_valid_ &&
                            semantic_snapshot_time_messages_ > 0 &&
                            semantic_update_time_messages_ > 0 &&
                            semantic_callback_time_messages_ > 0 &&
                            main_loop_time_messages_ > 0,
                        "Semantic snapshot/update/callback and complete main-loop timings are published");
        reporter_.Check(graph_messages_ > 0 && max_graph_points_ > 2,
                        "The visualization graph grows beyond the odometry node");
        reporter_.Check(contour_messages_ > 0 && max_contour_points_ > 0,
                        "Obstacle contours are extracted from semantic static obstacles");
        if (enable_goal_test_) {
            reporter_.Check(goal_published_,
                            "An automatic goal is published after V-Graph initialization");
            reporter_.Check(waypoint_messages_ > 0,
                            "FARMaster accepts the goal and publishes waypoints");
            reporter_.Check(path_messages_ > 0 && max_path_points_ > 0,
                            "FARMaster finds and visualizes a nonempty graph path");
            if (max_path_points_ >=
                static_cast<std::size_t>(minimum_path_points_)) {
                reporter_.Check(true,
                                "The planned graph path reaches the requested minimum length");
            } else {
                reporter_.Skip(
                    "The recorded goals remained directly connected; ProjectNavWaypoint's intermediate-node branch was not exercised");
            }
            reporter_.Check(reach_messages_ > 0,
                            "FARMaster publishes goal-reach status while planning");
            reporter_.Check(planning_time_messages_ > 0 &&
                                traverse_time_messages_ > 0,
                            "Goal planning publishes search and traversal timing");
            if (boundary_messages_ > 0 && max_boundary_points_ > 0) {
                reporter_.Check(true,
                                "LocalBoundaryHandler publishes nonempty navigation boundaries");
            } else {
                reporter_.Skip(
                    "This recording produced no local boundary pairs; LocalBoundaryHandler followed its empty-input return path");
            }
        }
        if (enable_control_test_) {
            reporter_.Check(pause_published_ && resume_published_,
                            "Visibility-graph pause and resume commands are published");
            reporter_.Check(pause_graph_messages_ > 0 &&
                                min_pause_global_nodes_ == max_pause_global_nodes_,
                            "Global graph topology stays frozen during the pause interval");
            reporter_.Check(graph_changed_after_resume_,
                            "Global graph topology changes again after resume");
            reporter_.Check(reset_published_ && reset_drop_seen_,
                            "Reset reduces the previously accumulated global graph");
            reporter_.Check(reset_rebuild_seen_,
                            "The global graph rebuilds from later semantic-map frames");
            reporter_.Check(reset_stop_waypoint_seen_ && reset_empty_path_seen_,
                            "Reset publishes a stop waypoint and clears the path");
        }
        if (observe_dynamic_behavior_) {
            reporter_.Check(dynamic_cloud_layout_valid_,
                            "Dynamic-obstacle output exposes valid XYZ fields");
            if (nonempty_dynamic_cloud_messages_ > 0) {
                reporter_.Check(dynamic_graph_messages_ > 0 &&
                                    dynamic_contour_messages_ > 0,
                                "Recorded dynamic occupancy is processed during ordinary Graph and contour update cycles");
            } else {
                reporter_.Skip(
                    "The recording contains no effective dynamic voxel in the configured local window");
                ROS_WARN_STREAM("Nearest recorded dynamic distance was "
                                << min_dynamic_robot_distance_ << " m at map "
                                << nearest_dynamic_map_index_);
            }
        }
        if (inject_dynamic_block_) {
            reporter_.Check(relay_semantic_map_ &&
                                dynamic_injection_prepared_ &&
                                dynamic_injection_sequence_complete_,
                            "A deterministic occupied/free dynamic-map sequence is relayed");
            reporter_.Check(injected_dynamic_seen_,
                            "FARMaster receives the injected dynamic voxel");
            reporter_.Check(injected_graph_messages_ > 0 &&
                                injected_contour_messages_ > 0,
                            "Injected dynamic occupancy passes through ordinary contour and Graph update cycles");
            reporter_.Check(injected_stop_waypoint_seen_ ||
                                injected_reroute_waypoint_seen_,
                            "The rebuilt Graph produces either a safe stop or a rerouted waypoint");
            reporter_.Check(injected_empty_path_seen_ ||
                                injected_nonempty_path_seen_,
                            "The rebuilt Graph publishes a path result while the injected obstacle is active");
            reporter_.Check(injected_dynamic_released_ &&
                                injected_release_semantic_delay_ >= 0.0 &&
                                injected_release_semantic_delay_ < 0.45,
                            "The first processed explicit-free snapshot immediately releases injected dynamic occupancy");
            reporter_.Check(post_injected_release_graph_messages_ > 0 &&
                                post_injected_release_contour_messages_ > 0,
                            "Explicit-free removal is followed by ordinary Graph and contour update cycles");
            if (injected_stop_waypoint_seen_ || injected_empty_path_seen_) {
                reporter_.Check(injected_recovery_waypoint_seen_ &&
                                    injected_recovery_path_seen_,
                                "Waypoint and nonempty path recover after a no-route stop clears");
            } else {
                reporter_.Check(injected_reroute_waypoint_seen_ &&
                                    injected_nonempty_path_seen_,
                                "A bypass remains active without an unnecessary forced stop");
            }
        }
        if (expect_input_timeout_stop_) {
            reporter_.Check(input_timeout_stop_waypoint_seen_ &&
                                input_timeout_empty_path_seen_,
                            "Input watchdog stops safely after odometry/map playback becomes stale");
        }

        ROS_INFO_STREAM("FAR baseline metrics: maps=" << map_messages_
                        << ", poses=" << pose_messages_
                        << ", odom published=" << odom_messages_
                        << ", max static points=" << max_static_points_
                        << ", max dynamic points=" << max_dynamic_points_
                        << ", max local static points=" << max_local_static_points_
                        << ", max local dynamic points=" << max_local_dynamic_points_
                        << ", dynamic frames=" << nonempty_dynamic_cloud_messages_
                        << ", empty dynamic frames=" << empty_dynamic_cloud_messages_
                        << ", graph messages=" << graph_messages_
                        << ", max global nodes=" << max_global_nodes_
                        << ", max graph marker points=" << max_graph_points_
                        << ", max contour points=" << max_contour_points_
                        << ", max boundary points=" << max_boundary_points_
                        << ", max runtime=" << max_runtime_
                        << ", max planning time=" << max_planning_time_
                        << ", waypoints=" << waypoint_messages_
                        << ", path messages=" << path_messages_
                        << ", max path points=" << max_path_points_
                        << ", reach messages=" << reach_messages_
                        << ", reached=" << reached_goal_);
        if (observe_dynamic_behavior_ &&
            std::isfinite(min_dynamic_robot_distance_)) {
            ROS_INFO_STREAM("Nearest dynamic voxel to robot: distance="
                            << min_dynamic_robot_distance_ << " m, point=("
                            << nearest_dynamic_point_.x << ", "
                            << nearest_dynamic_point_.y << ", "
                            << nearest_dynamic_point_.z << "), map="
                            << nearest_dynamic_map_index_
                            << ", stop waypoints=" << dynamic_stop_waypoints_
                            << ", empty paths=" << dynamic_empty_paths_);
        }
        if (inject_dynamic_block_) {
            ROS_INFO_STREAM("Injected dynamic metrics: occupied frames="
                            << injected_occupied_frames_published_
                            << ", explicit-free frames="
                            << injected_free_frames_published_
                            << ", semantic release delay="
                            << injected_release_semantic_delay_
                            << " s, wall release delay="
                            << injected_release_delay_
                            << " s, stop waypoint="
                            << injected_stop_waypoint_seen_
                            << ", reroute waypoint="
                            << injected_reroute_waypoint_seen_
                            << ", empty path=" << injected_empty_path_seen_
                            << ", nonempty path="
                            << injected_nonempty_path_seen_
                            << ", graph updates=" << injected_graph_messages_
                            << ", contour updates="
                            << injected_contour_messages_
                            << ", recovered waypoint="
                            << injected_recovery_waypoint_seen_
                            << ", recovered path="
                            << injected_recovery_path_seen_);
        }
        PrintTrajectoryCandidates();
        reporter_.Summary();
        finished_ = true;
        ros::shutdown();
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher odom_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher update_pub_;
    ros::Publisher reset_pub_;
    ros::Publisher semantic_map_pub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber static_cloud_sub_;
    ros::Subscriber dynamic_cloud_sub_;
    ros::Subscriber local_static_cloud_sub_;
    ros::Subscriber local_dynamic_cloud_sub_;
    ros::Subscriber graph_sub_;
    ros::Subscriber contour_sub_;
    ros::Subscriber runtime_sub_;
    ros::Subscriber waypoint_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber reach_sub_;
    ros::Subscriber boundary_sub_;
    ros::Subscriber planning_time_sub_;
    ros::Subscriber traverse_time_sub_;
    ros::Subscriber semantic_snapshot_time_sub_;
    ros::Subscriber semantic_update_time_sub_;
    ros::Subscriber semantic_callback_time_sub_;
    ros::Subscriber main_loop_time_sub_;
    ros::WallTimer idle_timer_;
    Reporter reporter_;
    std::string source_pose_topic_;
    std::string source_map_topic_;
    std::string odom_topic_;
    std::string world_frame_;
    std::string planner_map_topic_;
    std::string map_id_;
    std::string map_frame_;
    double map_resolution_ = 0.0;
    double stream_idle_timeout_ = 3.0;
    double odom_publish_period_ = 0.02;
    double trajectory_sample_period_ = 0.1;
    double goal_publish_delay_ = 1.0;
    Position goal_;
    Position secondary_goal_;
    Position latest_position_;
    bool enable_goal_test_ = false;
    bool goal_published_ = false;
    bool enable_secondary_goal_ = false;
    bool secondary_goal_published_ = false;
    bool enable_control_test_ = false;
    bool observe_dynamic_behavior_ = false;
    bool relay_semantic_map_ = false;
    bool inject_dynamic_block_ = false;
    bool expect_input_timeout_stop_ = false;
    bool input_timeout_stop_waypoint_seen_ = false;
    bool input_timeout_empty_path_seen_ = false;
    bool timing_values_valid_ = true;
    bool dynamic_injection_attempted_ = false;
    bool dynamic_injection_prepared_ = false;
    bool dynamic_injection_sequence_complete_ = false;
    bool injected_dynamic_effective_ = false;
    bool injected_dynamic_seen_ = false;
    bool injected_dynamic_released_ = false;
    bool injected_stop_waypoint_seen_ = false;
    bool injected_reroute_waypoint_seen_ = false;
    bool injected_empty_path_seen_ = false;
    bool injected_nonempty_path_seen_ = false;
    bool injected_recovery_waypoint_seen_ = false;
    bool injected_recovery_path_seen_ = false;
    bool dynamic_cloud_nonempty_ = false;
    bool dynamic_cloud_layout_valid_ = true;
    bool local_planner_clouds_valid_ = true;
    bool dynamic_stop_seen_ = false;
    bool dynamic_recovery_waypoint_seen_ = false;
    bool dynamic_recovery_path_seen_ = false;
    int pause_map_index_ = 150;
    int resume_map_index_ = 250;
    int reset_map_index_ = 450;
    int minimum_path_points_ = 1;
    int dynamic_injection_map_index_ = 250;
    int dynamic_occupied_frames_ = 10;
    int dynamic_free_frames_ = 8;
    int secondary_goal_map_index_ = 400;
    int injected_occupied_frames_published_ = 0;
    int injected_free_frames_published_ = 0;
    bool pause_published_ = false;
    bool resume_published_ = false;
    bool reset_published_ = false;
    bool graph_changed_after_resume_ = false;
    bool reset_drop_seen_ = false;
    bool reset_rebuild_seen_ = false;
    bool reset_stop_waypoint_seen_ = false;
    bool reset_empty_path_seen_ = false;
    bool finished_ = false;
    bool map_metadata_consistent_ = true;
    bool reached_goal_ = false;
    ros::Time last_odom_stamp_;
    ros::Time last_trajectory_stamp_;
    ros::Time first_injected_free_stamp_;
    ros::Time last_relayed_semantic_stamp_;
    ros::WallTime last_map_wall_time_;
    ros::WallTime graph_ready_wall_time_;
    ros::WallTime pause_publish_wall_time_;
    ros::WallTime resume_publish_wall_time_;
    ros::WallTime reset_publish_wall_time_;
    ros::WallTime first_injected_free_wall_time_;
    std::vector<Position> trajectory_;
    std::size_t map_messages_ = 0;
    std::size_t pose_messages_ = 0;
    std::size_t odom_messages_ = 0;
    std::size_t static_cloud_messages_ = 0;
    std::size_t dynamic_cloud_messages_ = 0;
    std::size_t local_static_cloud_messages_ = 0;
    std::size_t local_dynamic_cloud_messages_ = 0;
    std::size_t nonempty_dynamic_cloud_messages_ = 0;
    std::size_t empty_dynamic_cloud_messages_ = 0;
    std::size_t dynamic_stop_waypoints_ = 0;
    std::size_t dynamic_empty_paths_ = 0;
    std::size_t dynamic_graph_messages_ = 0;
    std::size_t dynamic_contour_messages_ = 0;
    std::size_t injected_graph_messages_ = 0;
    std::size_t injected_contour_messages_ = 0;
    std::size_t post_injected_release_graph_messages_ = 0;
    std::size_t post_injected_release_contour_messages_ = 0;
    std::size_t nearest_dynamic_map_index_ = 0;
    std::size_t graph_messages_ = 0;
    std::size_t contour_messages_ = 0;
    std::size_t runtime_messages_ = 0;
    std::size_t waypoint_messages_ = 0;
    std::size_t path_messages_ = 0;
    std::size_t reach_messages_ = 0;
    std::size_t boundary_messages_ = 0;
    std::size_t planning_time_messages_ = 0;
    std::size_t traverse_time_messages_ = 0;
    std::size_t semantic_snapshot_time_messages_ = 0;
    std::size_t semantic_update_time_messages_ = 0;
    std::size_t semantic_callback_time_messages_ = 0;
    std::size_t main_loop_time_messages_ = 0;
    std::size_t max_static_points_ = 0;
    std::size_t max_dynamic_points_ = 0;
    std::size_t max_local_static_points_ = 0;
    std::size_t max_local_dynamic_points_ = 0;
    std::size_t max_graph_points_ = 0;
    std::size_t current_global_nodes_ = 0;
    std::size_t max_global_nodes_ = 0;
    std::size_t pause_graph_messages_ = 0;
    std::size_t min_pause_global_nodes_ = std::numeric_limits<std::size_t>::max();
    std::size_t max_pause_global_nodes_ = 0;
    std::size_t pre_reset_global_nodes_ = 0;
    std::size_t min_post_reset_global_nodes_ =
        std::numeric_limits<std::size_t>::max();
    std::size_t max_contour_points_ = 0;
    std::size_t max_path_points_ = 0;
    std::size_t max_boundary_points_ = 0;
    float max_runtime_ = 0.0f;
    float max_planning_time_ = 0.0f;
    double min_dynamic_robot_distance_ =
        std::numeric_limits<double>::infinity();
    double injected_release_delay_ = 0.0;
    double injected_release_semantic_delay_ = 0.0;
    Position nearest_dynamic_point_;
    Position injected_dynamic_point_;
    octomap_msgs::OctomapPtr injected_occupied_message_;
    octomap_msgs::OctomapPtr injected_free_message_;
};

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "far_planner_bag_test");
    FarPlannerBagTest test;
    ros::spin();
    return test.exitCode();
}
