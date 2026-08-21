/*
 * Interactive integration tests for MapHandler.
 *
 * Run this node, then publish a semantic octomap on /octomap_full.  Tests that
 * do not require a map run immediately; lifecycle tests finish on the first
 * valid map message.  Setting ~continuous_mode:=true keeps consuming snapshots
 * and separates overlap-region map changes from local-window boundary churn.
 * The node also prints the top-1 semantic RGB histogram so
 * the obstacle and terrain-support class lists can be configured from the
 * actual segmentation palette.
 */

#include "far_planner/map_handler.h"
#include "far_planner/contour_detector.h"
#include "far_planner/contour_graph.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <semantic_octree/SemanticOcTree.h>
#include <semantic_octree/Semantics.h>

namespace {

using SemanticOctree = octomap::SemanticOcTree<octomap::SemanticsLogOdds>;
using SemanticOcTreeNode = octomap::SemanticOcTreeNode<octomap::SemanticsLogOdds>;

uint32_t MakeRgbKey(const uint8_t r, const uint8_t g, const uint8_t b) {
    return (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) |
           static_cast<uint32_t>(b);
}
std::string RgbString(const uint32_t key) {
    std::ostringstream out;
    out << "(" << ((key >> 16) & 0xffu)
        << "," << ((key >> 8) & 0xffu)
        << "," << (key & 0xffu) << ")"
        << " [0x" << std::hex << std::setw(6) << std::setfill('0') << key << "]";
    return out.str();
}

bool SameFloat(const float lhs, const float rhs) {
    return std::fabs(lhs - rhs) <= 1e-6f;
}

bool SameCloud(const PointCloud& lhs, const PointCloud& rhs) {
    if (lhs.size() != rhs.size() || lhs.width != rhs.width ||
        lhs.height != rhs.height || lhs.is_dense != rhs.is_dense) {
        return false;
    }
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        const PCLPoint& a = lhs.points[i];
        const PCLPoint& b = rhs.points[i];
        if (!SameFloat(a.x, b.x) || !SameFloat(a.y, b.y) ||
            !SameFloat(a.z, b.z) || !SameFloat(a.intensity, b.intensity)) {
            return false;
        }
    }
    return true;
}

bool SamePoints(const PointStack& lhs, const PointStack& rhs) {
    if (lhs.size() != rhs.size()) return false;
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        if (!SameFloat(lhs[i].x, rhs[i].x) || !SameFloat(lhs[i].y, rhs[i].y) ||
            !SameFloat(lhs[i].z, rhs[i].z) ||
            !SameFloat(lhs[i].intensity, rhs[i].intensity)) {
            return false;
        }
    }
    return true;
}

bool CloudContainsNear(const PointCloud& cloud,
                       const Point3D& point,
                       const float tolerance) {
    for (const auto& candidate : cloud.points) {
        if ((Point3D(candidate) - point).norm() <= tolerance) return true;
    }
    return false;
}

PointCloudPtr MakeSentinelCloud() {
    PointCloudPtr cloud(new PointCloud());
    PCLPoint first;
    first.x = 1.25f;
    first.y = -2.5f;
    first.z = 3.75f;
    first.intensity = 4.5f;
    PCLPoint second;
    second.x = -5.5f;
    second.y = 6.25f;
    second.z = -7.0f;
    second.intensity = 8.75f;
    cloud->points.push_back(first);
    cloud->points.push_back(second);
    cloud->width = 2;
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

class TestReporter {
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

    void Skip(const std::string& name, const std::string& reason) {
        ++skipped_;
        ROS_WARN_STREAM("[SKIP] " << name << ": " << reason);
    }

    int failed() const { return failed_; }

    void PrintSummary() const {
        ROS_INFO_STREAM("MapHandler test summary: " << passed_ << " passed, "
                        << failed_ << " failed, " << skipped_ << " skipped.");
    }

private:
    int passed_ = 0;
    int failed_ = 0;
    int skipped_ = 0;
};

class MapHandlerIntegrationTest {
public:
    MapHandlerIntegrationTest()
        : private_nh_("~") {
        private_nh_.param<std::string>("semantic_map_topic", topic_, "/octomap_full");
        private_nh_.param<std::string>("robot_pose_topic", robot_pose_topic_,
                                       "/model/go2_semantic/pose");
        private_nh_.param<std::string>("world_frame", world_frame_, "world");
        private_nh_.param<bool>("continuous_mode", continuous_mode_, false);
        private_nh_.param<double>("stream_idle_timeout", stream_idle_timeout_, 3.0);
        private_nh_.param<double>("overlap_margin", overlap_margin_, 0.5);
        private_nh_.param<int>("minimum_pose_samples", minimum_pose_samples_, 20);
        minimum_pose_samples_ = std::max(1, minimum_pose_samples_);
        private_nh_.param<float>("sensor_range", params_.sensor_range, 4.0f);
        private_nh_.param<float>("floor_height", params_.floor_height, 2.0f);
        private_nh_.param<float>("robot_x", robot_position_.x, 0.0f);
        private_nh_.param<float>("robot_y", robot_position_.y, 0.0f);
        private_nh_.param<float>("robot_z", robot_position_.z, 0.0f);
        bool use_synthetic_map = false;
        private_nh_.param<bool>("use_synthetic_map", use_synthetic_map, false);
        if (use_synthetic_map) {
            // Keep the deterministic fixture independent of parameters left by
            // an earlier real-bag run under the same ROS node name.
            continuous_mode_ = false;
            params_.sensor_range = 4.0f;
            params_.floor_height = 2.0f;
            params_.semantic_params.local_window_radius = 4.0f;
            robot_position_ = Point3D(0.0f, 0.0f, 0.0f);
        }

        ConfigureUtilityDefaults();
        RunMapIndependentTests();

        pose_subscriber_ = nh_.subscribe(
            robot_pose_topic_, 50,
            &MapHandlerIntegrationTest::RobotPoseCallback, this);
        subscriber_ = nh_.subscribe(topic_, 1,
                                    &MapHandlerIntegrationTest::SemanticMapCallback,
                                    this);
        if (continuous_mode_) {
            idle_timer_ = nh_.createWallTimer(
                ros::WallDuration(0.25),
                &MapHandlerIntegrationTest::ContinuousIdleCallback, this);
        }
        if (use_synthetic_map) {
            RunSyntheticMapTests();
            return;
        }
        ROS_INFO_STREAM("Waiting for a semantic octomap on " << topic_
                        << (continuous_mode_
                                ? ". Continuous mode will run until the map stream is idle."
                                : ". Publish one message to run map-dependent tests."));
    }

    int exitCode() const { return reporter_.failed() == 0 ? 0 : 1; }

private:
    void ConfigureUtilityDefaults() {
        // The production class list is YAML-owned. Keep the deterministic
        // fixture self-contained by declaring its eight test classes here.
        params_.obstacle_groups = {
            SemanticClassGroup("chair", 0x0021C1u),
            SemanticClassGroup("television", 0x004382u),
            SemanticClassGroup("table", 0x006544u),
            SemanticClassGroup("boundary_wall", 0x008605u),
            SemanticClassGroup("maze_wall", 0x00A8C7u)};
        params_.terrain_support_groups = {
            SemanticClassGroup("background_floor", 0x000000u),
            SemanticClassGroup("staircase", 0xFFA500u)};
        params_.dynamic_obstacle_groups = {
            SemanticClassGroup("dynamic_obstacle", 0xFF00FFu)};
        params_.semantic_params.local_planner_radius = 5.0f;
        params_.semantic_params.local_planner_resolution = 0.2f;
        params_.semantic_params.local_planner_obstacle_intensity = 200.0f;
        FARUtil::kLeafSize = 0.2f;
        FARUtil::vehicle_height = 0.5f;
        FARUtil::kMatchDist = 1.0f;
        FARUtil::kTolerZ = params_.floor_height;
        FARUtil::kSensorRange = params_.sensor_range;
        FARUtil::kLocalPlanRange = params_.sensor_range;
        FARUtil::worldFrameId = world_frame_;
        FARUtil::robot_pos = robot_position_;
        FARUtil::odom_pos = robot_position_;
    }

    void RunMapIndependentTests() {
        handler_.Init(params_);
        handler_.UpdateRobotPosition(robot_position_);

        reporter_.Check(!handler_.HasSemanticMap(),
                        "Init starts without a semantic map");

        reporter_.Check(
            !handler_.SetSemanticOctomap(octomap_msgs::OctomapConstPtr()) &&
                !handler_.HasSemanticMap(),
            "SetSemanticOctomap(nullptr) does not create a map");

        PointCloudPtr no_map_output = MakeSentinelCloud();
        handler_.GetCloudOfPoint(robot_position_, no_map_output, OBS_CLOUD, false);
        reporter_.Check(no_map_output->empty(),
                        "GetCloudOfPoint clears output when no map is available");
        handler_.GetCloudOfPoint(robot_position_, PointCloudPtr(), OBS_CLOUD, false);
        reporter_.Check(true, "GetCloudOfPoint accepts a null output pointer");

        octomap_msgs::OctomapPtr invalid(new octomap_msgs::Octomap());
        invalid->id = "MapHandlerTestInvalidTreeType";
        invalid->binary = false;
        invalid->resolution = FARUtil::kLeafSize;
        reporter_.Check(!handler_.SetSemanticOctomap(invalid) &&
                            !handler_.HasSemanticMap(),
                        "An unserializable message is rejected");

        RunCompatibilityNoOpTests();

        bool matched = true;
        const Point3D query(2.0f, -1.0f, 3.0f);
        const float height = MapHandler::TerrainHeightOfPoint(query, matched, false);
        reporter_.Check(!matched && SameFloat(height, query.z),
                        "TerrainHeightOfPoint falls back to query z without a map");

        handler_.ResetGripMapCloud();
        reporter_.Check(!handler_.HasSemanticMap(),
                        "ResetGripMapCloud is safe before the first map");

        RunPillarConnectivityTests();
        RunVerifiedSemanticContourTests();
        RunContourFollowEdgeTests();
    }

    void RunContourFollowEdgeTests() {
        const float original_leaf = FARUtil::kLeafSize;
        const float original_clearance = FARUtil::kNavClearDist;
        const float original_sensor_range = FARUtil::kSensorRange;
        const Point3D original_odom = FARUtil::odom_pos;
        const Point3D original_free_odom = FARUtil::free_odom_p;
        FARUtil::kLeafSize = 0.2f;
        FARUtil::kNavClearDist = 0.45f;
        FARUtil::kSensorRange = 20.0f;
        FARUtil::odom_pos = Point3D(1.0f, 2.0f, 0.5f);
        FARUtil::free_odom_p = FARUtil::odom_pos;

        ContourGraph graph;
        ContourGraphParams params;
        params.kPillarPerimeter = 0.4f;
        params.contour_projection_min = 0.15f;
        params.contour_projection_step = 0.075f;
        params.contour_projection_max = 0.60f;
        graph.Init(params);

        // Regression for goal selection 16 captured at 20:10:26 on
        // 2026-08-11. The 1.432 m odom-to-goal edge used to inherit the
        // obstacle-corner endpoint exclusion, leaving only one sampled centre
        // and allowing a wall close to the goal to be missed.
        NavNodePtr captured_start(new NavNode());
        captured_start->id = 9001;
        captured_start->is_odom = true;
        captured_start->position = Point3D(
            36.5611f, -45.4854f, 0.2710f);
        NavNodePtr captured_goal(new NavNode());
        captured_goal->id = 9002;
        captured_goal->is_goal = true;
        captured_goal->position = Point3D(
            37.7938f, -46.2144f, 0.0f);
        FARUtil::odom_pos = captured_start->position;
        FARUtil::free_odom_p = captured_start->position;

        const Point3D captured_delta =
            captured_goal->position - captured_start->position;
        const float captured_length = captured_delta.norm_flat();
        const Point3D captured_direction = captured_delta / captured_length;
        const Point3D captured_normal(
            -captured_direction.y, captured_direction.x, 0.0f);
        PointCloudPtr captured_endpoint_wall(new PointCloud());
        for (int index = -3; index <= 3; ++index) {
            const Point3D wall_point =
                captured_start->position + captured_direction * 1.30f +
                captured_normal * (0.10f * static_cast<float>(index));
            PCLPoint point;
            point.x = wall_point.x;
            point.y = wall_point.y;
            point.z = (captured_start->position.z +
                       captured_goal->position.z) * 0.5f;
            point.intensity = 1.0f;
            captured_endpoint_wall->push_back(point);
        }
        captured_endpoint_wall->width = captured_endpoint_wall->size();
        captured_endpoint_wall->height = 1;
        captured_endpoint_wall->is_dense = true;
        PointCloudPtr empty_terminal_cloud(new PointCloud());
        ContourGraph::SetLocalCollisionCloud(
            captured_endpoint_wall, empty_terminal_cloud);
        const EdgeValidationResult captured_static_block =
            ContourGraph::ValidateDirectOdomGoalEdgeWithRoute(
                captured_start, captured_goal, true);
        reporter_.Check(
            !captured_static_block.valid &&
                captured_static_block.reason ==
                    EdgeRejectReason::STATIC_CLOUD_BLOCKED,
            "Strict odom-goal validation detects a wall in the old short-edge endpoint blind zone");

        ContourGraph::SetLocalCollisionCloud(
            empty_terminal_cloud, captured_endpoint_wall);
        const EdgeValidationResult captured_dynamic_block =
            ContourGraph::ValidateDirectOdomGoalEdgeWithRoute(
                captured_start, captured_goal, true);
        reporter_.Check(
            !captured_dynamic_block.valid &&
                captured_dynamic_block.dynamic_blocked &&
                captured_dynamic_block.reason ==
                    EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED,
            "Strict odom-goal validation immediately rejects a dynamic endpoint blocker");

        ContourGraph::SetLocalCollisionCloud(
            empty_terminal_cloud, empty_terminal_cloud);
        const EdgeValidationResult captured_clear =
            ContourGraph::ValidateDirectOdomGoalEdgeWithRoute(
                captured_start, captured_goal, true);
        reporter_.Check(
            captured_clear.valid &&
                captured_clear.projection_distance == 0.0f &&
                (captured_clear.route_start - captured_start->position)
                        .norm_flat() < 1e-5f &&
                (captured_clear.route_end - captured_goal->position)
                        .norm_flat() < 1e-5f,
            "A clear odom-goal edge keeps its exact robot-centre geometry without corner projection");

        FARUtil::odom_pos = Point3D(1.0f, 2.0f, 0.5f);
        FARUtil::free_odom_p = FARUtil::odom_pos;

        PolygonPtr wall(new Polygon());
        wall->N = 4;
        wall->vertices = {
            Point3D(0.0f, -0.20f, 0.5f),
            Point3D(2.0f, -0.20f, 0.5f),
            Point3D(2.0f,  0.25f, 0.5f),
            Point3D(0.0f,  0.25f, 0.5f)};
        wall->is_robot_inside = false;
        wall->is_pillar = false;
        wall->source = GraphNodeSource::STATIC_CANDIDATE;

        const auto make_ct = [&wall](const Point3D& position) {
            CTNodePtr ct(new CTNode());
            ct->position = position;
            ct->poly_ptr = wall;
            ct->source = GraphNodeSource::STATIC_CANDIDATE;
            ct->free_direct = NodeFreeDirect::CONVEX;
            // Sum points into the obstacle (-Y); a CONVEX node therefore
            // projects toward the robot/free side (+Y).
            ct->surf_dirs = PointPair(Point3D(1.0f, -1.0f, 0.0f),
                                      Point3D(-1.0f, -1.0f, 0.0f));
            return ct;
        };
        CTNodePtr first_ct = make_ct(Point3D(0.0f, 0.25f, 0.5f));
        CTNodePtr second_ct = make_ct(Point3D(2.0f, 0.25f, 0.5f));
        first_ct->front = second_ct;
        first_ct->back = second_ct;
        second_ct->front = first_ct;
        second_ct->back = first_ct;

        const auto make_nav = [](const std::size_t id,
                                 const CTNodePtr& ct) {
            NavNodePtr node(new NavNode());
            node->id = id;
            node->position = ct->position;
            node->surf_dirs = ct->surf_dirs;
            node->free_direct = ct->free_direct;
            node->source = GraphNodeSource::STATIC_GLOBAL;
            node->is_odom = false;
            node->is_goal = false;
            node->is_boundary = false;
            node->is_navpoint = false;
            node->is_merged = false;
            node->is_active = true;
            node->ctnode = ct;
            node->is_contour_match = true;
            ct->is_global_match = true;
            ct->nav_node_id = id;
            return node;
        };
        NavNodePtr first = make_nav(1001, first_ct);
        NavNodePtr second = make_nav(1002, second_ct);

        const auto make_row = [](const float y) {
            PointCloudPtr cloud(new PointCloud());
            for (int index = 0; index <= 20; ++index) {
                PCLPoint point;
                point.x = index * 0.1f;
                point.y = y;
                point.z = 0.5f;
                point.intensity = 1.0f;
                cloud->push_back(point);
            }
            cloud->width = cloud->size();
            cloud->height = 1;
            cloud->is_dense = true;
            return cloud;
        };

        PointCloudPtr static_wall = make_row(0.0f);
        PointCloudPtr empty_dynamic(new PointCloud());
        ContourGraph::SetLocalCollisionCloud(static_wall, empty_dynamic);
        const EdgeValidationResult clear =
            ContourGraph::ValidateContourFollowEdge(first, second);
        reporter_.Check(
            clear.valid &&
                (clear.route_end - clear.route_start).norm_flat() > 1.9f,
            "Adaptive contour validation accepts a free-side wall route");
        reporter_.Check(
            clear.valid && clear.projection_distance > 0.15f &&
                clear.projection_distance <= 0.60f + 1e-4f &&
                clear.route_start.y > first->position.y,
            "A fixed 0.15 m offset grows until the 0.45 m corridor is clear");
        reporter_.Check(
            !ContourGraph::IsPoint3DConnectFreePolygon(
                Point3D(1.0f, 1.0f, 0.5f),
                Point3D(1.0f, -1.0f, 0.5f)),
            "A strict ordinary visibility segment through the wall is rejected");

        PointCloudPtr dynamic_block(new PointCloud());
        PCLPoint blocker;
        blocker.x = (clear.route_start.x + clear.route_end.x) * 0.5f;
        blocker.y = (clear.route_start.y + clear.route_end.y) * 0.5f;
        blocker.z = 0.5f;
        blocker.intensity = 1.0f;
        dynamic_block->push_back(blocker);
        dynamic_block->width = 1;
        dynamic_block->height = 1;
        dynamic_block->is_dense = true;
        ContourGraph::SetLocalCollisionCloud(static_wall, dynamic_block);
        const EdgeValidationResult blocked =
            ContourGraph::ValidateContourFollowEdge(first, second);
        reporter_.Check(
            blocked.valid && blocked.dynamic_blocked &&
                blocked.reason == EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED,
            "A current dynamic point immediately masks but does not erase the static contour route");

        ContourGraph::SetLocalCollisionCloud(static_wall, empty_dynamic);
        const EdgeValidationResult restored =
            ContourGraph::ValidateContourFollowEdge(first, second);
        reporter_.Check(
            restored.valid && !restored.dynamic_blocked &&
                SameFloat(restored.projection_distance,
                          clear.projection_distance),
            "Removing the dynamic point restores the same static route geometry");

        PointCloudPtr extended_static_wall(new PointCloud(*static_wall));
        PCLPoint newly_observed_wall = blocker;
        extended_static_wall->push_back(newly_observed_wall);
        extended_static_wall->width = extended_static_wall->size();
        ContourGraph::SetLocalCollisionCloud(extended_static_wall,
                                             empty_dynamic);
        reporter_.Check(
            !ContourGraph::IsRouteConnectFreeStaticLayer(
                clear.route_start, clear.route_end),
            "A newly observed static wall immediately blocks stored contour-route geometry");

        first_ct->is_boundary_clipped = true;
        wall->is_boundary_clipped = true;
        ContourGraph::SetLocalCollisionCloud(static_wall, empty_dynamic);
        const EdgeValidationResult clipped =
            ContourGraph::ValidateContourFollowEdge(first, second);
        reporter_.Check(
            clipped.valid && !clipped.dynamic_blocked,
            "An actually clipped polygon may provide a temporary FAR-style wall-end route when the persistent static corridor is free");
        first_ct->is_boundary_clipped = false;
        wall->is_boundary_clipped = false;

        reporter_.Check(
            ContourGraph::IsPointInsideReliableContourWindow(
                Point3D(20.5f, 2.0f, 0.5f)) &&
                !ContourGraph::IsPointInsideReliableContourWindow(
                    Point3D(20.7f, 2.0f, 0.5f)) &&
                ContourGraph::DoesSegmentIntersectReliableContourWindow(
                    Point3D(-30.0f, 2.0f, 0.5f),
                    Point3D(30.0f, 2.0f, 0.5f)),
            "The square contour guard rejects cropped endpoints while detecting crossing stored routes");

        PointCloudPtr narrow_channel = make_row(0.0f);
        *narrow_channel += *make_row(0.85f);
        narrow_channel->width = narrow_channel->size();
        ContourGraph::SetLocalCollisionCloud(narrow_channel, empty_dynamic);
        const EdgeValidationResult narrow =
            ContourGraph::ValidateContourFollowEdge(first, second);
        reporter_.Check(
            !narrow.valid &&
                narrow.reason == EdgeRejectReason::STATIC_CLOUD_BLOCKED,
            "A narrow channel with no 0.45 m-clear offset is rejected");

        PolygonPtr other_wall(new Polygon(*wall));
        second_ct->poly_ptr = other_wall;
        const EdgeValidationResult accidental_neighbor =
            ContourGraph::ValidateContourFollowEdge(first, second);
        reporter_.Check(
            !accidental_neighbor.valid &&
                accidental_neighbor.reason ==
                    EdgeRejectReason::NOT_CURRENT_ADJACENT,
            "Nearby vertices from different obstacles cannot become a contour-follow edge");
        second_ct->poly_ptr = wall;

        ContourGraph::SetLocalCollisionCloud(PointCloudPtr(new PointCloud()),
                                             PointCloudPtr(new PointCloud()));
        FARUtil::kLeafSize = original_leaf;
        FARUtil::kNavClearDist = original_clearance;
        FARUtil::kSensorRange = original_sensor_range;
        FARUtil::odom_pos = original_odom;
        FARUtil::free_odom_p = original_free_odom;
    }

    void RunVerifiedSemanticContourTests() {
        const bool original_static_env = FARUtil::IsStaticEnv;
        const Point3D original_free_odom = FARUtil::free_odom_p;
        FARUtil::IsStaticEnv = false;
        FARUtil::free_odom_p = Point3D(0.0f, 0.0f, 0.5f);

        ContourDetectParams params;
        params.sensor_range = 5.0f;
        params.contour_grid_resolution = 0.2f;
        params.kRatio = 3.0f;
        params.kThredValue = 3;
        params.kBlurSize = 3;
        params.is_save_img = false;
        params.img_path.clear();
        ContourDetector detector;
        detector.Init(params);

        NavNodePtr odom_node(new NavNode());
        odom_node->position = FARUtil::free_odom_p;
        odom_node->is_odom = true;
        PointCloudPtr sparse_occupied(new PointCloud());
        PCLPoint occupied_point;
        occupied_point.x = 1.0f;
        occupied_point.y = 0.0f;
        occupied_point.z = 0.5f;
        occupied_point.intensity = 0.0f;
        sparse_occupied->push_back(occupied_point);

        std::vector<PointStack> raw_scan_contours;
        detector.BuildTerrainImgAndExtractContour(
            odom_node, sparse_occupied, raw_scan_contours, false);
        reporter_.Check(
            raw_scan_contours.empty(),
            "Raw-scan filtering still rejects a single unconfirmed hit");

        std::vector<PointStack> semantic_contours;
        detector.BuildTerrainImgAndExtractContour(
            odom_node, sparse_occupied, semantic_contours, true);
        reporter_.Check(
            !semantic_contours.empty(),
            "A sparse verified semantic voxel survives contour rasterization");

        FARUtil::IsStaticEnv = original_static_env;
        FARUtil::free_odom_p = original_free_odom;
    }

    void RunPillarConnectivityTests() {
        const float original_clearance = FARUtil::kNavClearDist;
        const float original_tolerance = FARUtil::kTolerZ;
        const float original_sensor_range = FARUtil::kSensorRange;
        const Point3D original_odom = FARUtil::odom_pos;
        const Point3D original_free_odom = FARUtil::free_odom_p;
        FARUtil::kNavClearDist = 0.45f;
        FARUtil::kTolerZ = 0.2f;
        FARUtil::kSensorRange = 15.0f;
        FARUtil::odom_pos = Point3D(0.0f, 0.0f, 0.5f);
        FARUtil::free_odom_p = FARUtil::odom_pos;

        ContourGraph graph;
        ContourGraphParams graph_params;
        graph_params.kPillarPerimeter = 2.4f;
        graph.Init(graph_params);

        NavNodePtr odom_node(new NavNode());
        odom_node->position = FARUtil::odom_pos;
        odom_node->is_odom = true;
        const std::vector<std::vector<Point3D>> pillar_contour = {{
            Point3D(1.8f, -0.2f, 0.5f),
            Point3D(2.2f, -0.2f, 0.5f),
            Point3D(2.2f,  0.2f, 0.5f),
            Point3D(1.8f,  0.2f, 0.5f)}};
        graph.UpdateContourGraph(odom_node, pillar_contour);

        reporter_.Check(
            !ContourGraph::IsPoint3DConnectFreePolygon(
                Point3D(0.0f, 0.0f, 0.5f),
                Point3D(4.0f, 0.0f, 0.5f)),
            "A pillar on a local candidate edge blocks the connection");
        reporter_.Check(
            ContourGraph::IsPoint3DConnectFreePolygon(
                Point3D(0.0f, 0.7f, 0.5f),
                Point3D(4.0f, 0.7f, 0.5f)),
            "A candidate edge outside pillar clearance remains free");
        reporter_.Check(
            !ContourGraph::IsPoint3DConnectFreePolygon(
                Point3D(0.0f, 0.0f, 0.5f),
                Point3D(20.0f, 0.0f, 0.5f)),
            "A pillar also blocks the global-range connection branch");
        reporter_.Check(
            ContourGraph::IsPoint3DConnectFreePolygon(
                Point3D(0.0f, 0.0f, 2.0f),
                Point3D(4.0f, 0.0f, 2.0f)),
            "A pillar on another height layer does not block the edge");

        // A visibility node represents the free-space route around its owning
        // obstacle.  The owning pillar must not reject its own incident edge,
        // while the same pillar must still reject an edge passing through it.
        ContourGraph::SetLocalCollisionCloud(PointCloudPtr(new PointCloud()));
        const bool has_pillar_vertex = !ContourGraph::contour_graph_.empty();
        reporter_.Check(has_pillar_vertex,
                        "A pillar contour creates a routing vertex");
        if (has_pillar_vertex) {
            const CTNodePtr pillar_vertex = ContourGraph::contour_graph_.front();
            NavNodePtr pillar_node(new NavNode());
            pillar_node->id = 1000;
            pillar_node->position = pillar_vertex->position;
            pillar_node->source = GraphNodeSource::STATIC_CANDIDATE;
            pillar_node->free_direct = NodeFreeDirect::PILLAR;
            pillar_node->is_odom = false;
            pillar_node->is_goal = false;
            pillar_node->is_boundary = false;
            pillar_node->is_contour_match = false;
            ContourGraph::MatchCTNodeWithNavNode(pillar_vertex, pillar_node);
            odom_node->source = GraphNodeSource::ODOM;
            odom_node->free_direct = NodeFreeDirect::PILLAR;
            odom_node->is_goal = false;
            odom_node->is_boundary = false;
            odom_node->is_contour_match = false;
            reporter_.Check(
                ContourGraph::IsNavNodesConnectFreePolygon(
                    odom_node, pillar_node),
                "An obstacle endpoint does not collide with its own incident edge");

            PointCloudPtr endpoint_occupancy(new PointCloud());
            PCLPoint endpoint_point;
            endpoint_point.x = pillar_node->position.x;
            endpoint_point.y = pillar_node->position.y;
            endpoint_point.z = pillar_node->position.z;
            endpoint_point.intensity = 0.0f;
            endpoint_occupancy->push_back(endpoint_point);
            ContourGraph::SetLocalCollisionCloud(endpoint_occupancy);
            reporter_.Check(
                ContourGraph::IsNavNodesConnectFreePolygon(
                    odom_node, pillar_node),
                "Raw occupancy at a graph endpoint is excluded from the edge interior");
        }

        graph.UpdateContourGraph(
            odom_node, std::vector<std::vector<Point3D>>());
        ContourGraph::SetLocalCollisionCloud(PointCloudPtr(new PointCloud()));
        reporter_.Check(
            ContourGraph::IsPoint3DConnectFreePolygon(
                Point3D(0.0f, 0.0f, 0.5f),
                Point3D(4.0f, 0.0f, 0.5f)),
            "Removing a pillar restores the ordinary connection check");

        PointCloudPtr raw_collision(new PointCloud());
        PCLPoint raw_obstacle;
        raw_obstacle.x = 2.0f;
        raw_obstacle.y = 0.0f;
        raw_obstacle.z = 0.5f;
        raw_obstacle.intensity = 0.0f;
        raw_collision->push_back(raw_obstacle);
        ContourGraph::SetLocalCollisionCloud(raw_collision);
        reporter_.Check(
            !ContourGraph::IsPoint3DConnectFreePolygon(
                Point3D(0.0f, 0.0f, 0.5f),
                Point3D(4.0f, 0.0f, 0.5f)),
            "Raw local semantic occupancy blocks an edge even without a contour segment");
        reporter_.Check(
            ContourGraph::IsPoint3DConnectFreePolygon(
                Point3D(0.0f, 1.0f, 0.5f),
                Point3D(4.0f, 1.0f, 0.5f)),
            "Raw local collision validation keeps a geometrically clear edge");

        NavNodePtr edge_start(new NavNode());
        NavNodePtr edge_end(new NavNode());
        edge_start->id = 1001;
        edge_end->id = 1002;
        edge_start->position = Point3D(0.0f, 0.0f, 0.5f);
        edge_end->position = Point3D(4.0f, 0.0f, 0.5f);
        edge_start->source = GraphNodeSource::STATIC_GLOBAL;
        edge_end->source = GraphNodeSource::STATIC_GLOBAL;
        edge_start->free_direct = NodeFreeDirect::PILLAR;
        edge_end->free_direct = NodeFreeDirect::PILLAR;
        edge_start->is_contour_match = false;
        edge_end->is_contour_match = false;
        edge_start->is_boundary = false;
        edge_end->is_boundary = false;

        ContourGraph::SetLocalCollisionCloud(
            PointCloudPtr(new PointCloud()), raw_collision);
        reporter_.Check(
            ContourGraph::IsNavNodesConnectFreeStaticPolygon(
                edge_start, edge_end),
            "A dynamic point does not invalidate persistent static geometry");
        reporter_.Check(
            !ContourGraph::IsNavNodesConnectFreeDynamicLayer(
                edge_start, edge_end),
            "The same dynamic point temporarily blocks the static edge");

        ContourGraph::SetLocalCollisionCloud(
            raw_collision, PointCloudPtr(new PointCloud()));
        reporter_.Check(
            !ContourGraph::IsNavNodesConnectFreeStaticPolygon(
                edge_start, edge_end),
            "A static point invalidates the static edge itself");
        reporter_.Check(
            ContourGraph::IsNavNodesConnectFreeDynamicLayer(
                edge_start, edge_end),
            "An empty dynamic layer does not block a valid search edge");
        ContourGraph::SetLocalCollisionCloud(PointCloudPtr(new PointCloud()));
        graph.ResetCurrentContour();

        FARUtil::kNavClearDist = original_clearance;
        FARUtil::kTolerZ = original_tolerance;
        FARUtil::kSensorRange = original_sensor_range;
        FARUtil::odom_pos = original_odom;
        FARUtil::free_odom_p = original_free_odom;
    }

    void RunCompatibilityNoOpTests() {
        const PointCloudPtr input = MakeSentinelCloud();
        const PointCloud input_before = *input;
        const PointCloudPtr output = MakeSentinelCloud();
        const PointCloud output_before = *output;

        handler_.UpdateObsCloudGrid(input);
        reporter_.Check(SameCloud(*input, input_before),
                        "UpdateObsCloudGrid is a non-mutating compatibility no-op");

        handler_.UpdateFreeCloudGrid(input);
        reporter_.Check(SameCloud(*input, input_before),
                        "UpdateFreeCloudGrid is a non-mutating compatibility no-op");

        handler_.UpdateTerrainHeightGrid(input, output);
        reporter_.Check(SameCloud(*input, input_before) &&
                            SameCloud(*output, output_before),
                        "UpdateTerrainHeightGrid leaves input and output unchanged");

        handler_.GetSurroundObsCloud(output);
        reporter_.Check(output->empty(),
                        "GetSurroundObsCloud returns an empty derived view without a map");

        PointCloudPtr changed = MakeSentinelCloud();
        handler_.GetChangedObsCloud(changed);
        reporter_.Check(changed->empty(),
                        "GetChangedObsCloud is empty before the first semantic map");

        PointCloudPtr dynamic_view = MakeSentinelCloud();
        handler_.GetCurrentDynamicObsCloud(dynamic_view);
        reporter_.Check(dynamic_view->empty(),
                        "Dynamic obstacle views are empty before the first semantic map");
        handler_.GetEffectiveDynamicObsCloud(dynamic_view);
        reporter_.Check(dynamic_view->empty(),
                        "Effective dynamic view is empty before the first semantic map");
        handler_.GetCollisionObsCloud(dynamic_view);
        reporter_.Check(dynamic_view->empty(),
                        "Collision obstacle view is empty before the first semantic map");

        PointStack neighbor_centers;
        neighbor_centers.emplace_back(1.0f, 2.0f, 3.0f, 4.0f);
        const PointStack neighbor_before = neighbor_centers;
        handler_.GetNeighborCeilsCenters(neighbor_centers);
        reporter_.Check(SamePoints(neighbor_centers, neighbor_before),
                        "GetNeighborCeilsCenters is a non-mutating compatibility no-op");

        PointStack occupancy_centers;
        occupancy_centers.emplace_back(-1.0f, -2.0f, -3.0f, -4.0f);
        const PointStack occupancy_before = occupancy_centers;
        handler_.GetOccupancyCeilsCenters(occupancy_centers);
        reporter_.Check(SamePoints(occupancy_centers, occupancy_before),
                        "GetOccupancyCeilsCenters is a non-mutating compatibility no-op");

        handler_.RemoveObsCloudFromGrid(input);
        reporter_.Check(SameCloud(*input, input_before),
                        "RemoveObsCloudFromGrid is a non-mutating compatibility no-op");

        handler_.ClearObsCellThroughPosition(Point3D(1.0f, 2.0f, 3.0f));
        reporter_.Check(true, "ClearObsCellThroughPosition returns safely");

        handler_.UpdateObsCloudGrid(PointCloudPtr());
        handler_.UpdateFreeCloudGrid(PointCloudPtr());
        handler_.UpdateTerrainHeightGrid(PointCloudPtr(), PointCloudPtr());
        handler_.GetSurroundObsCloud(PointCloudPtr());
        handler_.RemoveObsCloudFromGrid(PointCloudPtr());
        reporter_.Check(true, "Compatibility no-ops accept null cloud pointers");
    }

    bool MatchesGroup(const uint32_t key,
                      const std::vector<SemanticClassGroup>& groups) const {
        for (const auto& group : groups) {
            if (group.rgb_key == key) return true;
        }
        return false;
    }

    bool GetOccupiedRgbAt(const octomap::AbstractOcTree& tree,
                          const Point3D& point,
                          uint32_t& rgb_key) const {
        const octomap::point3d query(point.x, point.y, point.z);
        const SemanticOctree* semantic_tree =
            dynamic_cast<const SemanticOctree*>(&tree);
        if (semantic_tree) {
            const SemanticOcTreeNode* node = semantic_tree->search(query);
            if (!node || !semantic_tree->isNodeOccupied(node)) return false;
            const auto color = node->isSemanticsSet()
                ? node->getSemantics().getSemanticColor()
                : node->getColor();
            rgb_key = MakeRgbKey(color.r, color.g, color.b);
            return true;
        }

        const octomap::ColorOcTree* color_tree =
            dynamic_cast<const octomap::ColorOcTree*>(&tree);
        if (!color_tree) return false;
        const octomap::ColorOcTreeNode* node = color_tree->search(query);
        if (!node || !color_tree->isNodeOccupied(node)) return false;
        const auto color = node->getColor();
        rgb_key = MakeRgbKey(color.r, color.g, color.b);
        return true;
    }

    std::map<uint32_t, Point3D> FindClassRepresentatives(
        const octomap::AbstractOcTree& tree) const {
        std::map<uint32_t, Point3D> representatives;
        const auto keep_nearest_to_robot =
            [this, &representatives](const uint32_t key,
                                     const Point3D& candidate) {
                const auto current = representatives.find(key);
                const float candidate_distance =
                    std::hypot(candidate.x - robot_position_.x,
                               candidate.y - robot_position_.y);
                const float current_distance = current == representatives.end()
                    ? std::numeric_limits<float>::max()
                    : std::hypot(current->second.x - robot_position_.x,
                                 current->second.y - robot_position_.y);
                if (candidate_distance < current_distance) {
                    representatives[key] = candidate;
                }
            };
        const SemanticOctree* semantic_tree =
            dynamic_cast<const SemanticOctree*>(&tree);
        if (semantic_tree) {
            for (auto it = semantic_tree->begin_leafs(), end = semantic_tree->end_leafs();
                 it != end; ++it) {
                if (!semantic_tree->isNodeOccupied(*it)) continue;
                const SemanticOcTreeNode* node = it.operator->();
                if (!node) continue;
                const auto color = node->isSemanticsSet()
                    ? node->getSemantics().getSemanticColor()
                    : node->getColor();
                const uint32_t key = MakeRgbKey(color.r, color.g, color.b);
                const auto coordinate = it.getCoordinate();
                keep_nearest_to_robot(
                    key, Point3D(coordinate.x(), coordinate.y(), coordinate.z()));
            }
            return representatives;
        }

        const octomap::ColorOcTree* color_tree =
            dynamic_cast<const octomap::ColorOcTree*>(&tree);
        if (!color_tree) return representatives;
        for (auto it = color_tree->begin_leafs(), end = color_tree->end_leafs();
             it != end; ++it) {
            if (!color_tree->isNodeOccupied(*it)) continue;
            const auto color = it->getColor();
            const uint32_t key = MakeRgbKey(color.r, color.g, color.b);
            const auto coordinate = it.getCoordinate();
            keep_nearest_to_robot(
                key, Point3D(coordinate.x(), coordinate.y(), coordinate.z()));
        }
        return representatives;
    }

    bool ValidateExtractedCloud(const PointCloud& cloud,
                                const Point3D& center,
                                const float radius,
                                const CloudType type,
                                const octomap::AbstractOcTree& tree) const {
        const float voxel_half_extent =
            static_cast<float>(tree.getResolution() * 0.5);
        const float max_horizontal_offset = radius + voxel_half_extent + 1e-4f;
        const float vertical_half_extent = std::max(
            std::max(FARUtil::kTolerZ, FARUtil::kCellHeight * 2.0f),
            FARUtil::vehicle_height + static_cast<float>(tree.getResolution()));
        const float max_vertical_offset =
            vertical_half_extent + voxel_half_extent + 1e-4f;
        std::set<std::tuple<int64_t, int64_t, int64_t>> unique_points;

        for (std::size_t i = 0; i < cloud.size(); ++i) {
            const Point3D point(cloud.points[i]);
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z)) {
                return false;
            }
            if (std::fabs(point.x - center.x) > max_horizontal_offset ||
                std::fabs(point.y - center.y) > max_horizontal_offset ||
                std::fabs(point.z - center.z) > max_vertical_offset) return false;

            uint32_t key = 0;
            if (!GetOccupiedRgbAt(tree, point, key)) return false;
            const bool class_matches = type == OBS_CLOUD
                ? MatchesGroup(key, params_.obstacle_groups)
                : MatchesGroup(key, params_.terrain_support_groups);
            if (!class_matches) return false;
            const auto quantized = std::make_tuple(
                static_cast<int64_t>(std::llround(point.x * 100000.0)),
                static_cast<int64_t>(std::llround(point.y * 100000.0)),
                static_cast<int64_t>(std::llround(point.z * 100000.0)));
            if (!unique_points.insert(quantized).second) return false;
        }
        return true;
    }

    bool CloudContainsClass(const PointCloud& cloud,
                            const uint32_t expected_key,
                            const octomap::AbstractOcTree& tree) const {
        for (const auto& pcl_point : cloud.points) {
            uint32_t actual_key = 0;
            if (GetOccupiedRgbAt(tree, Point3D(pcl_point), actual_key) &&
                actual_key == expected_key) {
                return true;
            }
        }
        return false;
    }

    bool IsApproximateSubset(const PointCloud& subset,
                             const PointCloud& superset,
                             const float tolerance) const {
        if (subset.empty()) return true;
        if (superset.empty()) return false;
        PointCloudPtr superset_ptr(new PointCloud(superset));
        pcl::KdTreeFLANN<PCLPoint> tree;
        tree.setInputCloud(superset_ptr);
        for (const auto& subset_point : subset.points) {
            std::vector<int> indices(1);
            std::vector<float> squared_distances(1);
            if (tree.nearestKSearch(subset_point, 1, indices, squared_distances) != 1 ||
                squared_distances[0] > tolerance * tolerance) return false;
        }
        return true;
    }

    void RunSemanticCloudTests(const octomap::AbstractOcTree& tree) {
        const float large_radius = params_.semantic_params.local_window_radius > 0.0f
            ? params_.semantic_params.local_window_radius
            : params_.sensor_range;
        const float small_radius = large_radius * 0.5f;

        PointCloudPtr small_obstacles(new PointCloud());
        PointCloudPtr large_obstacles(new PointCloud());
        PointCloudPtr small_terrain(new PointCloud());
        PointCloudPtr large_terrain(new PointCloud());
        handler_.GetCloudOfPoint(robot_position_, small_obstacles, OBS_CLOUD, false);
        handler_.GetCloudOfPoint(robot_position_, large_obstacles, OBS_CLOUD, true);
        handler_.GetCloudOfPoint(robot_position_, small_terrain, FREE_CLOUD, false);
        handler_.GetCloudOfPoint(robot_position_, large_terrain, FREE_CLOUD, true);
        PointCloudPtr current_static_obstacles(new PointCloud());
        handler_.GetCurrentStaticObsCloud(current_static_obstacles);
        if (!current_static_obstacles->empty()) {
            const auto nearest_static = std::min_element(
                current_static_obstacles->begin(),
                current_static_obstacles->end(),
                [this](const PCLPoint& first, const PCLPoint& second) {
                    const float first_distance = std::hypot(
                        first.x - robot_position_.x,
                        first.y - robot_position_.y);
                    const float second_distance = std::hypot(
                        second.x - robot_position_.x,
                        second.y - robot_position_.y);
                    return first_distance < second_distance;
                });
            const float nearest_distance = std::hypot(
                nearest_static->x - robot_position_.x,
                nearest_static->y - robot_position_.y);
            if (nearest_distance <= large_radius) {
                reporter_.Check(
                    handler_.QueryStaticNodeEvidence(
                        Point3D(*nearest_static)) ==
                        StaticNodeEvidence::STATIC_OCCUPIED,
                    "A current static semantic voxel is positive lifecycle evidence");
            } else {
                reporter_.Skip(
                    "Static-node occupied evidence",
                    "the local box contains no static voxel inside the radial lifecycle window");
            }
        } else {
            reporter_.Skip("Static-node occupied evidence",
                           "the received map has no configured static obstacle class");
        }
        const Point3D outside_window(
            robot_position_.x + large_radius + 2.0f,
            robot_position_.y, robot_position_.z);
        reporter_.Check(
            handler_.QueryStaticNodeEvidence(outside_window) ==
                StaticNodeEvidence::UNKNOWN,
            "Space outside R_update/window is never static deletion evidence");

        reporter_.Check(ValidateExtractedCloud(*small_obstacles, robot_position_,
                                               small_radius, OBS_CLOUD, tree),
                        "Small obstacle cloud contains only occupied obstacle classes in range");
        reporter_.Check(ValidateExtractedCloud(*large_obstacles, robot_position_,
                                               large_radius, OBS_CLOUD, tree),
                        "Large obstacle cloud contains only occupied obstacle classes in range");
        reporter_.Check(ValidateExtractedCloud(*small_terrain, robot_position_,
                                               small_radius, FREE_CLOUD, tree),
                        "Small terrain cloud contains only occupied floor voxels in range");
        reporter_.Check(ValidateExtractedCloud(*large_terrain, robot_position_,
                                               large_radius, FREE_CLOUD, tree),
                        "Large terrain cloud contains only occupied floor voxels in range");

        const float subset_tolerance =
            std::max(1e-4f, static_cast<float>(tree.getResolution()) * 0.25f);
        reporter_.Check(IsApproximateSubset(*small_obstacles, *large_obstacles,
                                            subset_tolerance),
                        "Small obstacle window is a subset of the large window");
        reporter_.Check(IsApproximateSubset(*small_terrain, *large_terrain,
                                            subset_tolerance),
                        "Small terrain window is a subset of the large window");

        PointCloudPtr local_static(new PointCloud());
        handler_.GetLocalPlannerStaticObsCloud(local_static);
        bool local_static_valid = true;
        for (const auto& point : local_static->points) {
            const float dx = point.x - robot_position_.x;
            const float dy = point.y - robot_position_.y;
            if (dx * dx + dy * dy >
                    params_.semantic_params.local_planner_radius *
                    params_.semantic_params.local_planner_radius + 1e-4f ||
                std::fabs(point.intensity - 200.0f) > 1e-4f) {
                local_static_valid = false;
                break;
            }
        }
        reporter_.Check(local_static_valid,
                        "Trajectory-planner static layer is radius-cropped and uses hard-obstacle intensity 200");

        PointCloudPtr invalid_type_output = MakeSentinelCloud();
        handler_.GetCloudOfPoint(robot_position_, invalid_type_output,
                                 static_cast<CloudType>(99), false);
        reporter_.Check(invalid_type_output->empty(),
                        "GetCloudOfPoint rejects an invalid CloudType and clears output");

        const auto representatives = FindClassRepresentatives(tree);
        for (const auto& group : params_.obstacle_groups) {
            const auto representative = representatives.find(group.rgb_key);
            const std::string test_name =
                "Obstacle extraction includes class " + group.name + " " +
                RgbString(group.rgb_key);
            if (representative == representatives.end()) {
                reporter_.Skip(test_name, "class does not occur in this map");
                continue;
            }
            PointCloudPtr class_cloud(new PointCloud());
            handler_.GetCloudOfPoint(representative->second, class_cloud,
                                     OBS_CLOUD, false);
            reporter_.Check(!class_cloud->empty() &&
                                ValidateExtractedCloud(*class_cloud,
                                                       representative->second,
                                                       small_radius,
                                                       OBS_CLOUD, tree) &&
                                CloudContainsClass(*class_cloud, group.rgb_key, tree),
                            test_name);
        }

        for (const auto& group : params_.terrain_support_groups) {
            const auto representative = representatives.find(group.rgb_key);
            const std::string test_name =
                "Terrain extraction includes class " + group.name + " " +
                RgbString(group.rgb_key);
            if (representative == representatives.end()) {
                reporter_.Skip(test_name, "class does not occur in this map");
                continue;
            }
            PointCloudPtr class_cloud(new PointCloud());
            handler_.GetCloudOfPoint(representative->second, class_cloud,
                                     FREE_CLOUD, false);
            reporter_.Check(!class_cloud->empty() &&
                                ValidateExtractedCloud(*class_cloud,
                                                       representative->second,
                                                       small_radius,
                                                       FREE_CLOUD, tree) &&
                                CloudContainsClass(*class_cloud, group.rgb_key, tree),
                            test_name);
        }
    }

    void RunTerrainHeightTests(
        const octomap::AbstractOcTree& tree,
        const octomap_msgs::OctomapConstPtr& message) {
        const auto representatives = FindClassRepresentatives(tree);
        const uint32_t floor_key = params_.terrain_support_groups.front().rgb_key;
        const auto floor = representatives.find(floor_key);
        if (floor == representatives.end()) {
            reporter_.Skip("Terrain height query on a real floor voxel",
                           "the map contains no configured floor class");
            reporter_.Skip("NearestHeightOfRadius associates with terrain",
                           "the map contains no configured floor class");
            reporter_.Skip("NearestTerrainHeightofNavPoint associates with terrain",
                           "the map contains no configured floor class");
            return;
        }

        const float local_margin = params_.sensor_range +
            static_cast<float>(tree.getResolution()) * 0.5f;
        if (std::fabs(floor->second.x - robot_position_.x) > local_margin ||
            std::fabs(floor->second.y - robot_position_.y) > local_margin) {
            reporter_.Skip("Terrain height query on a real floor voxel",
                           "first floor sample lies outside the local terrain window");
            reporter_.Skip("NearestHeightOfRadius associates with terrain",
                           "first floor sample lies outside the local terrain window");
            reporter_.Skip("NearestTerrainHeightofNavPoint associates with terrain",
                           "first floor sample lies outside the local terrain window");
            return;
        }

        const Point3D nav_point(floor->second.x, floor->second.y,
                                floor->second.z + FARUtil::vehicle_height);
        bool ray_matched = false;
        const float ray_height =
            MapHandler::TerrainHeightOfPoint(nav_point, ray_matched, false);
        reporter_.Check(ray_matched && SameFloat(ray_height, floor->second.z),
                        "TerrainHeightOfPoint returns the occupied floor voxel center");

        float min_height = 0.0f;
        float max_height = 0.0f;
        bool radius_matched = false;
        const float average_height = MapHandler::NearestHeightOfRadius(
            nav_point, FARUtil::kMatchDist, min_height, max_height, radius_matched);
        if (!radius_matched || !std::isfinite(min_height) ||
            !std::isfinite(average_height) || !std::isfinite(max_height) ||
            min_height > average_height || average_height > max_height) {
            ROS_WARN_STREAM("Terrain radius diagnostic: matched=" << radius_matched
                            << ", min=" << min_height
                            << ", average=" << average_height
                            << ", max=" << max_height
                            << ", query=(" << nav_point.x << ","
                            << nav_point.y << "," << nav_point.z << ")");
        }
        reporter_.Check(radius_matched && min_height <= average_height &&
                            average_height <= max_height,
                        "NearestHeightOfRadius reports ordered min/average/max terrain heights");

        bool nav_associated = false;
        const float nav_height = MapHandler::NearestTerrainHeightofNavPoint(
            nav_point, nav_associated);
        reporter_.Check(nav_associated && nav_height >= min_height - 1e-5f &&
                            nav_height <= max_height + 1e-5f,
                        "NearestTerrainHeightofNavPoint returns an associated terrain height");

        if (using_synthetic_map_) {
            RunSyntheticTerrainHeightTests(message);
        }
    }

    void RunSyntheticTerrainHeightTests(
        const octomap_msgs::OctomapConstPtr& message) {
        const Point3D first_floor(0.25f, 0.25f, 0.25f);
        const Point3D middle_step(0.25f, 1.75f, 0.75f);

        bool matched = false;
        const float direct_height = MapHandler::TerrainHeightOfPoint(
            Point3D(first_floor.x, first_floor.y, 2.0f), matched, false);
        reporter_.Check(matched && SameFloat(direct_height, 0.25f),
                        "Vertical ray returns the synthetic floor center z=0.25");

        matched = false;
        const float searched_height = MapHandler::TerrainHeightOfPoint(
            Point3D(0.60f, first_floor.y, 2.0f), matched, true);
        reporter_.Check(matched && SameFloat(searched_height, 0.25f),
                        "KD-tree search recovers nearby terrain when the vertical ray misses");

        float min_height = 0.0f;
        float max_height = 0.0f;
        bool radius_matched = false;
        const float average_height = MapHandler::NearestHeightOfRadius(
            Point3D(first_floor.x, first_floor.y, 3.0f), 2.0f,
            min_height, max_height, radius_matched);
        reporter_.Check(radius_matched && SameFloat(min_height, 0.25f) &&
                            SameFloat(max_height, 1.25f) &&
                            SameFloat(average_height, 0.75f),
                        "Radius query computes stair heights min=0.25 avg=0.75 max=1.25");

        bool associated = false;
        const float nearest_nav_height = MapHandler::NearestTerrainHeightofNavPoint(
            Point3D(first_floor.x, first_floor.y,
                    first_floor.z + FARUtil::vehicle_height), associated);
        reporter_.Check(associated && SameFloat(nearest_nav_height, 0.25f),
                        "NearestTerrainHeightofNavPoint returns the closest floor center");

        associated = true;
        const Point3D far_nav_point(100.0f, 100.0f, 3.0f);
        const float fallback_height = MapHandler::NearestTerrainHeightofNavPoint(
            far_nav_point, associated);
        reporter_.Check(!associated &&
                            SameFloat(fallback_height,
                                      far_nav_point.z - FARUtil::vehicle_height),
                        "Unassociated nav point falls back to point.z - vehicle_height");

        MapHandlerParams crop_params = params_;
        crop_params.sensor_range = 0.6f;
        crop_params.semantic_params.local_window_radius = 0.6f;
        MapHandler crop_handler;
        crop_handler.Init(crop_params);
        crop_handler.SetMapOrigin(first_floor);
        crop_handler.SetSemanticOctomap(message);

        bool first_visible = false;
        bool step_visible = false;
        MapHandler::TerrainHeightOfPoint(first_floor, first_visible, false);
        MapHandler::TerrainHeightOfPoint(middle_step, step_visible, false);
        reporter_.Check(first_visible && !step_visible,
                        "SetMapOrigin builds terrain only inside the initial local window");

        crop_handler.UpdateRobotPosition(middle_step);
        first_visible = false;
        step_visible = false;
        MapHandler::TerrainHeightOfPoint(first_floor, first_visible, false);
        const float moved_height = MapHandler::TerrainHeightOfPoint(
            middle_step, step_visible, false);
        reporter_.Check(!first_visible && step_visible &&
                            SameFloat(moved_height, middle_step.z),
                        "UpdateRobotPosition moves the local terrain window onto a stair");

        // MapHandler's terrain query cache is static; restore the primary fixture.
        handler_.UpdateRobotPosition(robot_position_);
    }

    NavNodePtr MakeAdjustableNavNode(const Point3D& position) const {
        NavNodePtr node(new NavNode());
        node->position = position;
        node->is_active = true;
        node->is_boundary = false;
        node->is_odom = false;
        node->is_navpoint = false;
        node->is_goal = false;
        return node;
    }

    CTNodePtr MakeCTNode(const Point3D& position) const {
        CTNodePtr node(new CTNode());
        node->position = position;
        node->is_ground_associate = false;
        return node;
    }

    void RunNodeAdjustmentTests() {
        const float expected_floor_nav_z = 0.25f + FARUtil::vehicle_height;

        NavNodePtr direct = MakeAdjustableNavNode(
            Point3D(0.25f, 0.25f, 0.0f));
        NavNodePtr filtered = MakeAdjustableNavNode(
            Point3D(0.25f, 0.25f, 0.0f));
        filtered->pos_filter_vec.push_back(Point3D(0.25f, 0.25f, 0.2f));
        filtered->pos_filter_vec.push_back(Point3D(0.25f, 0.25f, 0.4f));

        NavNodePtr inactive = MakeAdjustableNavNode(
            Point3D(0.25f, 0.25f, -1.0f));
        inactive->is_active = false;
        NavNodePtr boundary = MakeAdjustableNavNode(
            Point3D(0.25f, 0.25f, -2.0f));
        boundary->is_boundary = true;
        NavNodePtr odom = MakeAdjustableNavNode(
            Point3D(0.25f, 0.25f, -3.0f));
        odom->is_odom = true;
        NavNodePtr navpoint = MakeAdjustableNavNode(
            Point3D(0.25f, 0.25f, -4.0f));
        navpoint->is_navpoint = true;
        NavNodePtr outside_goal = MakeAdjustableNavNode(
            Point3D(0.25f, 0.25f, -5.0f));
        outside_goal->is_goal = true;
        NavNodePtr outside_local_range = MakeAdjustableNavNode(
            Point3D(100.0f, 100.0f, -6.0f));
        NavNodePtr no_terrain = MakeAdjustableNavNode(
            Point3D(3.0f, 0.0f, 0.3f));

        const NodePtrStack nodes = {
            direct, filtered, inactive, boundary, odom, navpoint,
            outside_goal, outside_local_range, no_terrain, NavNodePtr()};
        handler_.AdjustNodesHeight(nodes);

        reporter_.Check(SameFloat(direct->position.z, expected_floor_nav_z),
                        "AdjustNodesHeight sets node z=floor center+vehicle_height");
        reporter_.Check(SameFloat(filtered->pos_filter_vec.back().z,
                                  expected_floor_nav_z) &&
                            SameFloat(filtered->position.z,
                                      (0.2f + expected_floor_nav_z) * 0.5f),
                        "AdjustNodesHeight updates and averages position-filter history");
        reporter_.Check(SameFloat(inactive->position.z, -1.0f) &&
                            SameFloat(boundary->position.z, -2.0f) &&
                            SameFloat(odom->position.z, -3.0f) &&
                            SameFloat(navpoint->position.z, -4.0f) &&
                            SameFloat(outside_goal->position.z, -5.0f) &&
                            SameFloat(outside_local_range->position.z, -6.0f),
                        "AdjustNodesHeight preserves every documented skip category");
        reporter_.Check(SameFloat(no_terrain->position.z, 0.3f),
                        "AdjustNodesHeight preserves a local node with no terrain match");

        CTNodePtr ray_match = MakeCTNode(Point3D(0.25f, 0.25f, 2.0f));
        CTNodePtr nearest_fallback = MakeCTNode(Point3D(0.60f, 0.25f, 0.4f));
        CTNodePtr stair_edge = MakeCTNode(Point3D(0.25f, 1.10f, 0.4f));
        CTNodePtr no_match = MakeCTNode(Point3D(3.5f, 3.5f, 0.4f));
        const CTNodeStack ct_nodes = {
            ray_match, nearest_fallback, stair_edge, no_match, CTNodePtr()};
        handler_.AdjustCTNodeHeight(ct_nodes);

        reporter_.Check(ray_match->is_ground_associate &&
                            SameFloat(ray_match->position.z, expected_floor_nav_z),
                        "AdjustCTNodeHeight prefers the exact vertical-ray floor");
        reporter_.Check(nearest_fallback->is_ground_associate &&
                            SameFloat(nearest_fallback->position.z,
                                      expected_floor_nav_z),
                        "AdjustCTNodeHeight falls back to nearest XY terrain");
        reporter_.Check(stair_edge->is_ground_associate &&
                            SameFloat(stair_edge->position.z,
                                      0.75f + FARUtil::vehicle_height),
                        "AdjustCTNodeHeight selects the nearer stair, not minimum/average height");
        reporter_.Check(!no_match->is_ground_associate &&
                            SameFloat(no_match->position.z, 0.4f),
                        "AdjustCTNodeHeight preserves height when terrain is unavailable");

        const float original_tolerance = FARUtil::kTolerZ;
        const Point3D original_robot_position = FARUtil::robot_pos;
        const float clamp_tolerance = std::max(
            0.1f, expected_floor_nav_z - 0.1f);
        FARUtil::kTolerZ = clamp_tolerance;

        FARUtil::robot_pos.z = 0.0f;
        CTNodePtr upper_clamped = MakeCTNode(Point3D(0.25f, 0.25f, 0.0f));
        handler_.AdjustCTNodeHeight(CTNodeStack{upper_clamped});
        reporter_.Check(
            SameFloat(upper_clamped->position.z, clamp_tolerance),
                        "AdjustCTNodeHeight applies the upper robot-height clamp");

        FARUtil::robot_pos.z = expected_floor_nav_z + 1.0f;
        CTNodePtr lower_clamped = MakeCTNode(
            Point3D(0.25f, 0.25f, FARUtil::robot_pos.z));
        handler_.AdjustCTNodeHeight(CTNodeStack{lower_clamped});
        reporter_.Check(
            SameFloat(lower_clamped->position.z,
                      FARUtil::robot_pos.z - clamp_tolerance),
                        "AdjustCTNodeHeight applies the lower robot-height clamp");

        FARUtil::kTolerZ = original_tolerance;
        FARUtil::robot_pos = original_robot_position;

        handler_.AdjustNodesHeight(NodePtrStack());
        handler_.AdjustCTNodeHeight(CTNodeStack());
        reporter_.Check(true, "Node height adjustment accepts empty and null entries");
    }

    void RunTerrainNeighborTests(const octomap::AbstractOcTree& tree) {
        const auto representatives = FindClassRepresentatives(tree);
        const uint32_t floor_key = params_.terrain_support_groups.front().rgb_key;
        const auto floor = representatives.find(floor_key);
        if (floor == representatives.end()) {
            reporter_.Skip("IsNavPointOnTerrainNeighbor matches a floor-aligned nav point",
                           "the map contains no configured floor class");
            return;
        }

        const float local_margin = params_.sensor_range +
            static_cast<float>(tree.getResolution()) * 0.5f;
        if (std::fabs(floor->second.x - robot_position_.x) > local_margin ||
            std::fabs(floor->second.y - robot_position_.y) > local_margin) {
            reporter_.Skip("IsNavPointOnTerrainNeighbor matches a floor-aligned nav point",
                           "the map has no configured floor inside the local terrain window");
            return;
        }

        const Point3D floor_nav(floor->second.x, floor->second.y,
                                floor->second.z + FARUtil::vehicle_height);
        reporter_.Check(
            MapHandler::IsNavPointOnTerrainNeighbor(floor_nav, false),
            "IsNavPointOnTerrainNeighbor accepts a floor-aligned nav point");
        reporter_.Check(
            MapHandler::IsNavPointOnTerrainNeighbor(floor_nav, true),
            "Extended terrain-neighbor band includes the unextended band");

        if (!using_synthetic_map_) return;

        const float cell_height = FARUtil::kCellHeight;
        const float floor_height = 0.25f;
        const float extended_ground_height =
            floor_height - cell_height - cell_height * 0.5f;
        const Point3D downward_extended_nav(
            0.25f, 0.25f,
            extended_ground_height + FARUtil::vehicle_height);
        reporter_.Check(
            !MapHandler::IsNavPointOnTerrainNeighbor(
                downward_extended_nav, false) &&
            MapHandler::IsNavPointOnTerrainNeighbor(
                downward_extended_nav, true),
            "is_extend adds exactly the legacy downward terrain band");

        const float upper_bound = floor_height + FARUtil::kTolerZ + cell_height;
        const Point3D above_band_nav(
            0.25f, 0.25f,
            upper_bound + 0.1f + FARUtil::vehicle_height);
        reporter_.Check(
            !MapHandler::IsNavPointOnTerrainNeighbor(above_band_nav, false) &&
            !MapHandler::IsNavPointOnTerrainNeighbor(above_band_nav, true),
            "Terrain-neighbor query rejects a ground reference above its height band");

        const Point3D obstacle_without_floor_nav(
            2.75f, 0.25f, 0.75f + FARUtil::vehicle_height);
        reporter_.Check(
            !MapHandler::IsNavPointOnTerrainNeighbor(
                obstacle_without_floor_nav, false) &&
            !MapHandler::IsNavPointOnTerrainNeighbor(
                obstacle_without_floor_nav, true),
            "Semantic obstacle occupancy alone is not a terrain-neighbor match");
    }

    void RunCoarseLeafExpansionTest(const octomap::AbstractOcTree& tree) {
        if (!using_synthetic_map_) return;
        const auto* color_tree = dynamic_cast<const octomap::ColorOcTree*>(&tree);
        if (!color_tree) {
            reporter_.Check(false, "Synthetic coarse obstacle leaf is available");
            return;
        }

        Point3D coarse_center;
        float coarse_size = 0.0f;
        for (auto it = color_tree->begin_leafs(), end = color_tree->end_leafs();
             it != end; ++it) {
            if (!color_tree->isNodeOccupied(*it) ||
                it.getSize() <= tree.getResolution() + 1e-6) continue;
            const auto color = it->getColor();
            if (MakeRgbKey(color.r, color.g, color.b) != 0x0021C1u) continue;
            const auto coordinate = it.getCoordinate();
            coarse_center = Point3D(coordinate.x(), coordinate.y(), coordinate.z());
            coarse_size = static_cast<float>(it.getSize());
            break;
        }
        reporter_.Check(coarse_size > static_cast<float>(tree.getResolution()),
                        "Synthetic map contains a pruned coarse obstacle leaf");
        if (coarse_size <= static_cast<float>(tree.getResolution())) return;

        PointCloudPtr expanded(new PointCloud());
        handler_.GetCloudOfPoint(coarse_center, expanded, OBS_CLOUD, false);
        const float half = coarse_size * 0.5f + 1e-5f;
        std::size_t inside_count = 0;
        for (const auto& point : expanded->points) {
            if (std::fabs(point.x - coarse_center.x) > half ||
                std::fabs(point.y - coarse_center.y) > half ||
                std::fabs(point.z - coarse_center.z) > half) continue;
            uint32_t key = 0;
            if (GetOccupiedRgbAt(tree, Point3D(point), key) && key == 0x0021C1u) {
                ++inside_count;
            }
        }
        const std::size_t edge_voxels = static_cast<std::size_t>(
            std::llround(coarse_size / tree.getResolution()));
        const std::size_t expected_count =
            edge_voxels * edge_voxels * edge_voxels;
        reporter_.Check(inside_count == expected_count,
                        "GetCloudOfPoint expands a coarse leaf into aligned map-resolution centers");
    }

    void RunSyntheticChangeDetectionTests(
        const octomap_msgs::OctomapConstPtr& original_message) {
        if (!using_synthetic_map_) return;

        handler_.SetSemanticOctomap(original_message);
        PointCloudPtr unchanged(new PointCloud());
        handler_.GetChangedObsCloud(unchanged);
        reporter_.Check(unchanged->empty(),
                        "An identical semantic snapshot produces no obstacle-change event");

        octomap_msgs::OctomapPtr wrong_frame_message(
            new octomap_msgs::Octomap(*original_message));
        wrong_frame_message->header.frame_id = "wrong_semantic_map_frame";
        reporter_.Check(!handler_.SetSemanticOctomap(wrong_frame_message),
                        "A semantic map in a different world frame is rejected");
        handler_.GetChangedObsCloud(unchanged);
        reporter_.Check(unchanged->empty(),
                        "A rejected map does not replay the previous obstacle delta");

        std::unique_ptr<octomap::AbstractOcTree> decoded(
            octomap_msgs::msgToMap(*original_message));
        auto* modified_tree = dynamic_cast<octomap::ColorOcTree*>(decoded.get());
        const Point3D removed_obstacle(2.75f, 0.25f, 0.75f);
        if (!modified_tree) {
            reporter_.Check(false, "Synthetic obstacle map supports occupancy updates");
            return;
        }

        // Model the upstream octree's dynamic-obstacle clearing path: repeated
        // free-space observations lower the voxel's occupancy probability.
        const octomap::point3d removed_coordinate(
            removed_obstacle.x, removed_obstacle.y, removed_obstacle.z);
        for (int miss = 0; miss < 8; ++miss) {
            modified_tree->updateNode(removed_coordinate, false);
        }
        modified_tree->updateInnerOccupancy();
        const octomap::ColorOcTreeNode* cleared_node =
            modified_tree->search(removed_coordinate);
        reporter_.Check(cleared_node && !modified_tree->isNodeOccupied(cleared_node),
                        "Repeated octree misses clear a synthetic dynamic obstacle");
        if (!cleared_node || modified_tree->isNodeOccupied(cleared_node)) return;

        octomap_msgs::OctomapPtr modified_message(new octomap_msgs::Octomap());
        modified_message->header = original_message->header;
        modified_message->header.stamp = ros::Time::now();
        if (!octomap_msgs::fullMapToMsg(*modified_tree, *modified_message)) {
            reporter_.Check(false, "Modified semantic map can be serialized");
            return;
        }

        // The physical static layer has a different lifetime from contour
        // vertices: it survives outside the current square and is removed
        // only after explicit-free evidence is observed at the old cell.
        MapHandler persistent_handler;
        MapHandlerParams persistent_params = params_;
        persistent_params.semantic_params.local_window_radius = 1.0f;
        persistent_handler.Init(persistent_params);
        persistent_handler.UpdateRobotPosition(removed_obstacle);
        reporter_.Check(
            persistent_handler.SetSemanticOctomap(original_message),
            "Persistent-static fixture accepts the occupied snapshot");
        PointCloudPtr persistent_static(new PointCloud());
        persistent_handler.GetPersistentStaticObsCloud(persistent_static);
        reporter_.Check(
            CloudContainsNear(*persistent_static, removed_obstacle, 0.2f),
            "A current static obstacle enters the persistent collision layer");

        persistent_handler.UpdateRobotPosition(
            Point3D(-3.0f, -3.0f, removed_obstacle.z));
        persistent_handler.SetSemanticOctomap(modified_message);
        persistent_handler.GetPersistentStaticObsCloud(persistent_static);
        reporter_.Check(
            CloudContainsNear(*persistent_static, removed_obstacle, 0.2f),
            "A static obstacle outside the current square is retained");

        persistent_handler.UpdateRobotPosition(removed_obstacle);
        persistent_handler.SetSemanticOctomap(modified_message);
        persistent_handler.GetPersistentStaticObsCloud(persistent_static);
        reporter_.Check(
            !CloudContainsNear(*persistent_static, removed_obstacle, 0.2f),
            "Explicit-free evidence removes an old persistent static cell");

        // local_grid represents traversable terrain as an occupied semantic
        // endpoint, not as an OctoMap miss.  Reclassifying an old obstacle
        // voxel to a configured terrain class must therefore be explicit-free
        // evidence for the obstacle lifecycle and persistent collision layer.
        std::unique_ptr<octomap::AbstractOcTree> terrain_decoded(
            octomap_msgs::msgToMap(*original_message));
        auto* terrain_tree =
            dynamic_cast<octomap::ColorOcTree*>(terrain_decoded.get());
        if (!terrain_tree) {
            reporter_.Check(false,
                            "Synthetic terrain reclassification supports color updates");
        } else {
            octomap::ColorOcTreeNode* terrain_node =
                terrain_tree->search(removed_coordinate);
            if (!terrain_node) {
                reporter_.Check(false,
                                "Synthetic terrain reclassification finds the old obstacle");
            } else {
                terrain_node->setColor(0, 0, 0);
                terrain_tree->updateInnerOccupancy();
                octomap_msgs::OctomapPtr terrain_message(
                    new octomap_msgs::Octomap());
                terrain_message->header = original_message->header;
                terrain_message->header.stamp = ros::Time::now();
                if (!octomap_msgs::fullMapToMsg(
                        *terrain_tree, *terrain_message)) {
                    reporter_.Check(false,
                                    "Terrain-reclassified semantic map can be serialized");
                } else {
                    MapHandler terrain_release_handler;
                    terrain_release_handler.Init(params_);
                    terrain_release_handler.UpdateRobotPosition(
                        removed_obstacle);
                    terrain_release_handler.SetSemanticOctomap(
                        original_message);
                    terrain_release_handler.SetSemanticOctomap(
                        terrain_message);
                    terrain_release_handler.GetPersistentStaticObsCloud(
                        persistent_static);
                    reporter_.Check(
                        !CloudContainsNear(
                            *persistent_static, removed_obstacle, 0.2f),
                        "Occupied terrain semantics clear old persistent static collision cells");
                }
            }
        }

        handler_.SetSemanticOctomap(modified_message);
        PointCloudPtr removed_changes(new PointCloud());
        PointCloudPtr local_obstacles(new PointCloud());
        handler_.GetChangedObsCloud(removed_changes);
        handler_.GetSurroundObsCloud(local_obstacles);
        const auto contains_near = [](const PointCloud& cloud,
                                      const Point3D& point,
                                      const float tolerance) {
            for (const auto& candidate : cloud.points) {
                if ((Point3D(candidate) - point).norm() <= tolerance) return true;
            }
            return false;
        };
        const float tolerance = static_cast<float>(modified_tree->getResolution()) * 0.25f;
        reporter_.Check(contains_near(*removed_changes, removed_obstacle, tolerance) &&
                            !contains_near(*local_obstacles, removed_obstacle, tolerance),
                        "A disappeared octree obstacle becomes a change event and leaves the local cloud");

        FARUtil::stack_new_cloud_->clear();
        FARUtil::StackCloudByTime(removed_changes, FARUtil::stack_new_cloud_,
                                  FARUtil::kNewDecayTime);
        FARUtil::UpdateKdTrees(FARUtil::stack_new_cloud_);
        reporter_.Check(FARUtil::PointInNewCounter(
                            removed_obstacle, FARUtil::kMatchDist) > 0,
                        "Octree obstacle changes feed the DynamicGraph new-point KDTree");

        handler_.SetSemanticOctomap(original_message);
        PointCloudPtr restored_changes(new PointCloud());
        handler_.GetChangedObsCloud(restored_changes);
        reporter_.Check(contains_near(*restored_changes, removed_obstacle, tolerance),
                        "A reappearing octree obstacle also becomes a change event");
    }

    void RunSyntheticDynamicObstacleTests(
        const octomap_msgs::OctomapConstPtr& original_message) {
        if (!using_synthetic_map_) return;

        const Point3D dynamic_point(3.25f, 0.25f, 0.75f);
        const float tolerance = 0.1f;
        MapHandler dynamic_handler;
        MapHandlerParams dynamic_params = params_;
        dynamic_handler.Init(dynamic_params);
        dynamic_handler.SetMapOrigin(robot_position_);

        octomap_msgs::OctomapPtr occupied_message(
            new octomap_msgs::Octomap(*original_message));
        occupied_message->header.stamp = ros::Time(10.0);
        reporter_.Check(dynamic_handler.SetSemanticOctomap(occupied_message),
                        "Dynamic fixture accepts an occupied semantic snapshot");

        PointCloudPtr contour_obstacles(new PointCloud());
        PointCloudPtr current_dynamic(new PointCloud());
        PointCloudPtr effective_dynamic(new PointCloud());
        PointCloudPtr collision_obstacles(new PointCloud());
        PointCloudPtr dynamic_added(new PointCloud());
        PointCloudPtr dynamic_removed(new PointCloud());
        PointCloudPtr changed_obstacles(new PointCloud());
        PointCloudPtr local_dynamic_obstacles(new PointCloud());
        dynamic_handler.GetSurroundObsCloud(contour_obstacles);
        dynamic_handler.GetCurrentDynamicObsCloud(current_dynamic);
        dynamic_handler.GetEffectiveDynamicObsCloud(effective_dynamic);
        dynamic_handler.GetCollisionObsCloud(collision_obstacles);
        dynamic_handler.GetDynamicAddedCloud(dynamic_added);
        dynamic_handler.GetLocalPlannerDynamicObsCloud(local_dynamic_obstacles);
        reporter_.Check(CloudContainsNear(*contour_obstacles, dynamic_point,
                                          tolerance) &&
                            CloudContainsNear(*current_dynamic, dynamic_point,
                                              tolerance),
                        "Effective dynamic occupancy participates in FAR contour extraction");
        reporter_.Check(CloudContainsNear(*effective_dynamic, dynamic_point, tolerance) &&
                            CloudContainsNear(*collision_obstacles, dynamic_point, tolerance),
                        "Current dynamic occupancy remains in the collision layer");
        reporter_.Check(CloudContainsNear(*dynamic_added, dynamic_point, tolerance),
                        "A newly occupied dynamic voxel is reported as an addition");
        bool local_dynamic_intensity_valid = !local_dynamic_obstacles->empty();
        for (const auto& point : local_dynamic_obstacles->points) {
            if (std::fabs(point.intensity - 200.0f) > 1e-4f) {
                local_dynamic_intensity_valid = false;
                break;
            }
        }
        reporter_.Check(local_dynamic_intensity_valid,
                        "Trajectory-planner dynamic layer is separate and uses hard-obstacle intensity 200");
        const auto contains_dynamic_xy = [&dynamic_point](const PointCloud& cloud) {
            for (const auto& point : cloud.points) {
                if (std::hypot(point.x - dynamic_point.x,
                               point.y - dynamic_point.y) <= 0.3f) return true;
            }
            return false;
        };
        reporter_.Check(contains_dynamic_xy(*local_dynamic_obstacles),
                        "Trajectory-planner dynamic layer covers the occupied dynamic voxel in XY");

        dynamic_handler.UpdateRobotPosition(Point3D(20.0f, 0.0f, 0.0f));
        dynamic_handler.GetEffectiveDynamicObsCloud(effective_dynamic);
        dynamic_handler.GetDynamicRemovedCloud(dynamic_removed);
        dynamic_handler.GetChangedObsCloud(changed_obstacles);
        reporter_.Check(effective_dynamic->empty() &&
                            CloudContainsNear(*dynamic_removed, dynamic_point,
                                              tolerance) &&
                            CloudContainsNear(*changed_obstacles, dynamic_point,
                                              tolerance),
                        "Leaving the moving local window immediately emits a dynamic removal for Graph updates");
        dynamic_handler.UpdateRobotPosition(robot_position_);
        dynamic_handler.GetEffectiveDynamicObsCloud(effective_dynamic);
        dynamic_handler.GetDynamicAddedCloud(dynamic_added);
        reporter_.Check(CloudContainsNear(*effective_dynamic, dynamic_point,
                                          tolerance) &&
                            CloudContainsNear(*dynamic_added, dynamic_point,
                                              tolerance),
                        "Returning the local window re-observes and re-adds occupancy from the latest map snapshot");

        std::unique_ptr<octomap::AbstractOcTree> unknown_decoded(
            octomap_msgs::msgToMap(*occupied_message));
        auto* unknown_tree =
            dynamic_cast<octomap::ColorOcTree*>(unknown_decoded.get());
        const octomap::point3d coordinate(
            dynamic_point.x, dynamic_point.y, dynamic_point.z);
        if (!unknown_tree) {
            reporter_.Check(false,
                            "Dynamic fixture can create an unknown octree snapshot");
            handler_.UpdateRobotPosition(robot_position_);
            return;
        }
        // An empty but valid octree models a sensor frame with no knowledge in
        // this local region. This is distinct from an explicitly free node.
        unknown_tree->clear();
        reporter_.Check(unknown_tree->search(coordinate) == nullptr,
                        "Deleted dynamic voxel is unknown rather than explicit free");
        octomap_msgs::OctomapPtr unknown_message(new octomap_msgs::Octomap());
        unknown_message->header = occupied_message->header;
        if (!octomap_msgs::fullMapToMsg(*unknown_tree, *unknown_message)) {
            reporter_.Check(false, "Unknown dynamic fixture can be serialized");
            handler_.UpdateRobotPosition(robot_position_);
            return;
        }

        unknown_message->header.stamp = ros::Time(10.1);
        dynamic_handler.SetSemanticOctomap(unknown_message);
        dynamic_handler.GetCurrentDynamicObsCloud(current_dynamic);
        dynamic_handler.GetEffectiveDynamicObsCloud(effective_dynamic);
        dynamic_handler.GetDynamicRemovedCloud(dynamic_removed);
        dynamic_handler.GetSurroundObsCloud(contour_obstacles);
        dynamic_handler.GetCollisionObsCloud(collision_obstacles);
        reporter_.Check(current_dynamic->empty() &&
                            !CloudContainsNear(*effective_dynamic, dynamic_point,
                                               tolerance) &&
                            !CloudContainsNear(*contour_obstacles, dynamic_point,
                                               tolerance) &&
                            !CloudContainsNear(*collision_obstacles, dynamic_point,
                                               tolerance) &&
                            CloudContainsNear(*dynamic_removed, dynamic_point,
                                              tolerance),
                        "A dynamic voxel missing from the latest local snapshot is removed immediately");

        unknown_message->header.stamp = ros::Time(10.2);
        dynamic_handler.SetSemanticOctomap(unknown_message);
        dynamic_handler.GetDynamicRemovedCloud(dynamic_removed);
        reporter_.Check(dynamic_removed->empty(),
                        "An already-removed dynamic voxel does not emit repeated removal events");

        occupied_message->header.stamp = ros::Time(20.0);
        dynamic_handler.SetSemanticOctomap(occupied_message);

        std::unique_ptr<octomap::AbstractOcTree> decoded(
            octomap_msgs::msgToMap(*occupied_message));
        auto* free_tree = dynamic_cast<octomap::ColorOcTree*>(decoded.get());
        if (!free_tree) {
            reporter_.Check(false,
                            "Dynamic fixture supports explicit-free occupancy updates");
            handler_.UpdateRobotPosition(robot_position_);
            return;
        }
        for (int miss = 0; miss < 8; ++miss) {
            free_tree->updateNode(coordinate, false);
        }
        free_tree->updateInnerOccupancy();

        octomap_msgs::OctomapPtr free_message(new octomap_msgs::Octomap());
        free_message->header = occupied_message->header;
        if (!octomap_msgs::fullMapToMsg(*free_tree, *free_message)) {
            reporter_.Check(false,
                            "Explicit-free dynamic fixture can be serialized");
            handler_.UpdateRobotPosition(robot_position_);
            return;
        }

        free_message->header.stamp = ros::Time(20.1);
        dynamic_handler.SetSemanticOctomap(free_message);
        dynamic_handler.GetCurrentDynamicObsCloud(current_dynamic);
        dynamic_handler.GetEffectiveDynamicObsCloud(effective_dynamic);
        dynamic_handler.GetDynamicRemovedCloud(dynamic_removed);
        dynamic_handler.GetSurroundObsCloud(contour_obstacles);
        dynamic_handler.GetCollisionObsCloud(collision_obstacles);
        dynamic_handler.GetChangedObsCloud(changed_obstacles);
        reporter_.Check(current_dynamic->empty() &&
                            !CloudContainsNear(*effective_dynamic, dynamic_point,
                                               tolerance) &&
                            !CloudContainsNear(*contour_obstacles, dynamic_point,
                                               tolerance) &&
                            !CloudContainsNear(*collision_obstacles, dynamic_point,
                                               tolerance) &&
                            CloudContainsNear(*dynamic_removed, dynamic_point,
                                              tolerance) &&
                            CloudContainsNear(*changed_obstacles, dynamic_point,
                                              tolerance),
                        "Explicit-free occupancy immediately removes the dynamic contour, collision, and Graph-change state");

        // MapHandler's terrain cache is static; restore the primary fixture.
        handler_.UpdateRobotPosition(robot_position_);
    }

    void RunSyntheticMapTests() {
        octomap::ColorOcTree tree(0.5);
        struct ColoredVoxel {
            octomap::point3d coordinate;
            uint8_t r;
            uint8_t g;
            uint8_t b;
        };
        const std::vector<ColoredVoxel> voxels = {
            {octomap::point3d(0.25f, 0.25f, 0.25f), 0, 0, 0},
            {octomap::point3d(0.25f, 1.75f, 0.75f), 0, 0, 0},
            {octomap::point3d(0.25f, -1.25f, 1.25f), 0, 0, 0},
            {octomap::point3d(0.75f, 0.25f, 0.75f), 0, 33, 193},
            {octomap::point3d(1.25f, 0.25f, 0.75f), 0, 67, 130},
            {octomap::point3d(1.75f, 0.25f, 0.75f), 0, 101, 68},
            {octomap::point3d(2.25f, 0.25f, 0.75f), 0, 134, 5},
            {octomap::point3d(2.75f, 0.25f, 0.75f), 0, 168, 199},
            {octomap::point3d(3.25f, 0.25f, 0.75f), 255, 0, 255},
            {octomap::point3d(3.25f, -3.25f, 0.25f), 255, 165, 0},
            {octomap::point3d(-1.75f, -1.75f, 0.25f), 0, 33, 193},
            {octomap::point3d(-1.25f, -1.75f, 0.25f), 0, 33, 193},
            {octomap::point3d(-1.75f, -1.25f, 0.25f), 0, 33, 193},
            {octomap::point3d(-1.25f, -1.25f, 0.25f), 0, 33, 193},
            {octomap::point3d(-1.75f, -1.75f, 0.75f), 0, 33, 193},
            {octomap::point3d(-1.25f, -1.75f, 0.75f), 0, 33, 193},
            {octomap::point3d(-1.75f, -1.25f, 0.75f), 0, 33, 193},
            {octomap::point3d(-1.25f, -1.25f, 0.75f), 0, 33, 193},
        };
        for (const auto& voxel : voxels) {
            tree.updateNode(voxel.coordinate, true);
            tree.setNodeColor(voxel.coordinate.x(), voxel.coordinate.y(),
                              voxel.coordinate.z(), voxel.r, voxel.g, voxel.b);
        }
        tree.updateInnerOccupancy();
        tree.prune();

        octomap_msgs::OctomapPtr message(new octomap_msgs::Octomap());
        message->header.frame_id = FARUtil::worldFrameId;
        message->header.stamp = ros::Time::now();
        if (!octomap_msgs::fullMapToMsg(tree, *message)) {
            reporter_.Check(false, "Synthetic ColorOcTree can be serialized");
            reporter_.PrintSummary();
            ros::shutdown();
            return;
        }
        ROS_INFO("Running deterministic semantic-cloud tests with an eight-color synthetic map.");
        using_synthetic_map_ = true;
        SemanticMapCallback(message);
    }

    void RobotPoseCallback(const geometry_msgs::PoseStampedConstPtr& message) {
        if (!message || finished_) return;
        robot_position_ = Point3D(message->pose.position.x,
                                  message->pose.position.y,
                                  message->pose.position.z);
        ++pose_sample_count_;
        FARUtil::robot_pos = robot_position_;
        FARUtil::odom_pos = robot_position_;
        // Mirror FARMaster: pose callbacks only move the cached query center;
        // the next semantic snapshot rebuilds the derived local caches once.
        handler_.SetMapOrigin(robot_position_);
        pose_received_ = true;

        if (!pose_frame_warning_printed_ &&
            !message->header.frame_id.empty() &&
            !FARUtil::IsSameFrameID(message->header.frame_id, world_frame_)) {
            ROS_WARN_STREAM("Pose frame is " << message->header.frame_id
                            << " while the test world frame is " << world_frame_
                            << "; using the numeric pose directly as configured by the Gazebo dynamic_tf bridge.");
            pose_frame_warning_printed_ = true;
        }
    }

    void SemanticMapCallback(const octomap_msgs::OctomapConstPtr& message) {
        if (finished_) return;
        received_map_message_ = true;
        last_map_wall_time_ = ros::WallTime::now();
        if (!using_synthetic_map_ &&
            (!pose_received_ || pose_sample_count_ < minimum_pose_samples_)) {
            ROS_WARN_THROTTLE(1.0,
                "MapHandler test is waiting for stable robot pose samples before processing a real semantic map (%d/%d).",
                pose_sample_count_, minimum_pose_samples_);
            return;
        }

        std::unique_ptr<octomap::AbstractOcTree> decoded(
            octomap_msgs::msgToMap(*message));
        if (!decoded) {
            ROS_ERROR("The received message is not testable; waiting for another message.");
            return;
        }

        const std::size_t occupied_count = CountOccupiedLeaves(*decoded);
        if (occupied_count == 0) {
            ROS_WARN_THROTTLE(1.0,
                "The semantic octomap is still empty; waiting for a populated snapshot.");
            return;
        }

        const auto representatives = FindClassRepresentatives(*decoded);
        bool has_obstacle = false;
        bool has_terrain_support = false;
        for (const auto& representative : representatives) {
            has_obstacle = has_obstacle ||
                MatchesGroup(representative.first, params_.obstacle_groups);
            has_terrain_support = has_terrain_support ||
                MatchesGroup(representative.first, params_.terrain_support_groups);
        }
        if ((!continuous_mode_ || continuous_frames_ == 0) &&
            (!has_obstacle || !has_terrain_support)) {
            ROS_WARN_THROTTLE(1.0,
                "Semantic octomap has %zu occupied leaves but does not yet contain both terrain support and obstacles; waiting.",
                occupied_count);
            return;
        }

        if (continuous_mode_) {
            ProcessContinuousMap(message, *decoded);
            return;
        }

        reporter_.Check(true, "Received semantic octomap message can be deserialized");
        ROS_INFO_STREAM("Map metadata: id=" << message->id
                        << ", frame=" << message->header.frame_id
                        << ", resolution=" << decoded->getResolution()
                        << ", nodes=" << decoded->size());
        PrintSemanticColorHistogram(*decoded);

        reporter_.Check(handler_.SetSemanticOctomap(message) &&
                            handler_.HasSemanticMap(),
                        "A valid semantic octomap sets HasSemanticMap=true");

        bool robot_terrain_matched = false;
        const float robot_terrain_height = MapHandler::TerrainHeightOfPoint(
            robot_position_, robot_terrain_matched, true);
        ROS_INFO_STREAM("Robot/terrain height diagnostic: robot_z="
                        << robot_position_.z
                        << ", terrain_voxel_center_z="
                        << robot_terrain_height
                        << ", center_offset="
                        << (robot_position_.z - robot_terrain_height)
                        << ", matched=" << std::boolalpha
                        << robot_terrain_matched);

        RunSemanticCloudTests(*decoded);
        RunCoarseLeafExpansionTest(*decoded);
        RunTerrainHeightTests(*decoded, message);
        if (using_synthetic_map_) RunNodeAdjustmentTests();
        RunTerrainNeighborTests(*decoded);
        RunSyntheticChangeDetectionTests(message);
        RunSyntheticDynamicObstacleTests(message);

        octomap_msgs::OctomapPtr invalid(new octomap_msgs::Octomap());
        invalid->id = "MapHandlerTestInvalidTreeType";
        invalid->binary = false;
        invalid->resolution = FARUtil::kLeafSize;
        reporter_.Check(!handler_.SetSemanticOctomap(invalid) &&
                            handler_.HasSemanticMap(),
                        "An invalid update preserves the last valid map");

        handler_.ResetGripMapCloud();
        reporter_.Check(!handler_.HasSemanticMap(),
                        "ResetGripMapCloud clears a loaded map");

        bool matched = true;
        const float height = MapHandler::TerrainHeightOfPoint(
            robot_position_, matched, true);
        reporter_.Check(!matched && SameFloat(height, robot_position_.z),
                        "Reset clears ray and KD-tree terrain data used by height queries");
        reporter_.Check(
            !MapHandler::IsNavPointOnTerrainNeighbor(robot_position_, false) &&
            !MapHandler::IsNavPointOnTerrainNeighbor(robot_position_, true),
            "Reset clears both terrain-neighbor height bands");

        handler_.SetSemanticOctomap(message);
        reporter_.Check(handler_.HasSemanticMap(),
                        "MapHandler can load a map again after reset");

        finished_ = true;
        reporter_.PrintSummary();
        ros::shutdown();
    }

    using QuantizedPoint = std::tuple<int64_t, int64_t, int64_t>;

    QuantizedPoint QuantizePoint(const PCLPoint& point) const {
        return std::make_tuple(
            static_cast<int64_t>(std::llround(point.x * 10000.0)),
            static_cast<int64_t>(std::llround(point.y * 10000.0)),
            static_cast<int64_t>(std::llround(point.z * 10000.0)));
    }

    bool IsInsideBothLocalWindows(const PCLPoint& point,
                                  const Point3D& previous_center,
                                  const Point3D& current_center,
                                  const float radius) const {
        const float usable_radius = std::max(
            0.0f, radius - static_cast<float>(overlap_margin_));
        const auto inside = [usable_radius, &point](const Point3D& center) {
            return std::fabs(point.x - center.x) <= usable_radius &&
                   std::fabs(point.y - center.y) <= usable_radius;
        };
        return inside(previous_center) && inside(current_center);
    }

    void ProcessContinuousMap(const octomap_msgs::OctomapConstPtr& message,
                              const octomap::AbstractOcTree& decoded) {
        if (!pose_received_) {
            ++continuous_frames_without_pose_;
        }

        if (!continuous_tree_metadata_set_) {
            continuous_tree_metadata_set_ = true;
            continuous_tree_id_ = message->id;
            continuous_tree_frame_ = message->header.frame_id;
            continuous_tree_resolution_ = decoded.getResolution();
            ROS_INFO_STREAM("Continuous map metadata: id=" << message->id
                            << ", frame=" << message->header.frame_id
                            << ", resolution=" << decoded.getResolution()
                            << ", nodes=" << decoded.size());
            PrintSemanticColorHistogram(decoded);
        } else if (message->id != continuous_tree_id_ ||
                   message->header.frame_id != continuous_tree_frame_ ||
                   std::fabs(decoded.getResolution() -
                             continuous_tree_resolution_) > 1e-6) {
            continuous_metadata_consistent_ = false;
        }

        if (!handler_.SetSemanticOctomap(message)) {
            ++continuous_rejected_frames_;
            return;
        }

        PointCloudPtr current_obstacles(new PointCloud());
        PointCloudPtr current_terrain(new PointCloud());
        PointCloudPtr changed(new PointCloud());
        PointCloudPtr current_dynamic(new PointCloud());
        PointCloudPtr effective_dynamic(new PointCloud());
        PointCloudPtr collision_obstacles(new PointCloud());
        PointCloudPtr dynamic_added(new PointCloud());
        PointCloudPtr dynamic_removed(new PointCloud());
        handler_.GetSurroundObsCloud(current_obstacles);
        handler_.GetCloudOfPoint(robot_position_, current_terrain,
                                 FREE_CLOUD, true);
        handler_.GetChangedObsCloud(changed);
        handler_.GetCurrentDynamicObsCloud(current_dynamic);
        handler_.GetEffectiveDynamicObsCloud(effective_dynamic);
        handler_.GetCollisionObsCloud(collision_obstacles);
        handler_.GetDynamicAddedCloud(dynamic_added);
        handler_.GetDynamicRemovedCloud(dynamic_removed);

        const auto cloud_keys = [this](const PointCloud& cloud) {
            std::set<QuantizedPoint> keys;
            for (const auto& point : cloud.points) keys.insert(QuantizePoint(point));
            return keys;
        };
        const std::set<QuantizedPoint> contour_keys = cloud_keys(*current_obstacles);
        const std::set<QuantizedPoint> current_dynamic_keys =
            cloud_keys(*current_dynamic);
        const std::set<QuantizedPoint> effective_dynamic_keys =
            cloud_keys(*effective_dynamic);
        const std::set<QuantizedPoint> collision_keys =
            cloud_keys(*collision_obstacles);
        const std::set<QuantizedPoint> dynamic_added_keys =
            cloud_keys(*dynamic_added);
        const std::set<QuantizedPoint> dynamic_removed_keys =
            cloud_keys(*dynamic_removed);
        const auto is_subset = [](const std::set<QuantizedPoint>& subset,
                                  const std::set<QuantizedPoint>& superset) {
            return std::includes(superset.begin(), superset.end(),
                                 subset.begin(), subset.end());
        };
        if (current_dynamic_keys != effective_dynamic_keys ||
            !is_subset(effective_dynamic_keys, contour_keys) ||
            !is_subset(effective_dynamic_keys, collision_keys) ||
            !is_subset(dynamic_added_keys, current_dynamic_keys)) {
            continuous_dynamic_layers_valid_ = false;
        }
        for (const auto& key : dynamic_removed_keys) {
            if (effective_dynamic_keys.count(key) != 0) {
                continuous_dynamic_layers_valid_ = false;
            }
        }

        if (!current_dynamic_keys.empty()) ++continuous_frames_with_dynamic_;
        continuous_max_dynamic_voxels_ = std::max(
            continuous_max_dynamic_voxels_, current_dynamic_keys.size());
        continuous_dynamic_additions_ += dynamic_added_keys.size();
        continuous_dynamic_removals_ += dynamic_removed_keys.size();

        const float local_radius = params_.semantic_params.local_window_radius > 0.0f
            ? params_.semantic_params.local_window_radius
            : params_.sensor_range;
        if (!ValidateExtractedCloud(*current_obstacles, robot_position_,
                                    local_radius, OBS_CLOUD, decoded) ||
            !ValidateExtractedCloud(*current_terrain, robot_position_,
                                    local_radius, FREE_CLOUD, decoded)) {
            continuous_local_clouds_valid_ = false;
        }
        if (!current_terrain->empty()) ++continuous_frames_with_terrain_;

        if (continuous_frames_ == 0) {
            reporter_.Check(true,
                            "First continuous semantic octomap can be deserialized");
            reporter_.Check(handler_.HasSemanticMap(),
                            "First continuous semantic octomap initializes MapHandler");
            RunSemanticCloudTests(decoded);
            RunTerrainHeightTests(decoded, message);
            RunTerrainNeighborTests(decoded);
            first_continuous_robot_position_ = robot_position_;
        } else {
            std::set<QuantizedPoint> current_keys;
            for (const auto& point : current_obstacles->points) {
                current_keys.insert(QuantizePoint(point));
            }

            std::size_t frame_added = 0;
            std::size_t frame_removed = 0;
            std::size_t frame_boundary = 0;
            for (const auto& point : changed->points) {
                if (!IsInsideBothLocalWindows(point, previous_map_robot_position_,
                                              robot_position_, local_radius)) {
                    ++frame_boundary;
                    continue;
                }
                if (current_keys.count(QuantizePoint(point)) != 0) {
                    ++frame_added;
                } else {
                    ++frame_removed;
                }
            }
            continuous_added_voxels_ += frame_added;
            continuous_removed_voxels_ += frame_removed;
            continuous_boundary_voxels_ += frame_boundary;
            if (frame_added > 0) ++continuous_frames_with_additions_;
            if (frame_removed > 0) ++continuous_frames_with_removals_;
            if (frame_added > continuous_max_frame_additions_) {
                continuous_max_frame_additions_ = frame_added;
                continuous_max_addition_stamp_ = message->header.stamp;
            }
            if (frame_removed > continuous_max_frame_removals_) {
                continuous_max_frame_removals_ = frame_removed;
                continuous_max_removal_stamp_ = message->header.stamp;
            }

            const float robot_step = std::hypot(
                robot_position_.x - previous_map_robot_position_.x,
                robot_position_.y - previous_map_robot_position_.y);
            if (robot_step < 0.005f) {
                continuous_stationary_added_voxels_ += frame_added;
                continuous_stationary_removed_voxels_ += frame_removed;
                ++continuous_stationary_frame_pairs_;
            }

            if (!changed->empty()) {
                PointCloudPtr kd_input(new PointCloud(*changed));
                FARUtil::stack_new_cloud_->clear();
                FARUtil::StackCloudByTime(kd_input, FARUtil::stack_new_cloud_,
                                          FARUtil::kNewDecayTime);
                FARUtil::UpdateKdTrees(FARUtil::stack_new_cloud_);
                if (FARUtil::PointInNewCounter(
                        Point3D(changed->points.front()), 0.01f) > 0) {
                    continuous_kdtree_verified_ = true;
                }
            }
        }

        previous_map_robot_position_ = robot_position_;
        ++continuous_frames_;
        const float travel = std::hypot(
            robot_position_.x - first_continuous_robot_position_.x,
            robot_position_.y - first_continuous_robot_position_.y);
        continuous_max_robot_displacement_ = std::max(
            continuous_max_robot_displacement_, travel);

        if (continuous_frames_ % 100 == 0) {
            ROS_INFO_STREAM("Continuous progress: frames=" << continuous_frames_
                            << ", overlap added=" << continuous_added_voxels_
                            << ", overlap removed=" << continuous_removed_voxels_
                            << ", boundary=" << continuous_boundary_voxels_
                            << ", robot displacement="
                            << continuous_max_robot_displacement_ << " m");
        }
    }

    void ContinuousIdleCallback(const ros::WallTimerEvent&) {
        if (finished_ || !continuous_mode_ || !received_map_message_) return;
        if ((ros::WallTime::now() - last_map_wall_time_).toSec() <
            stream_idle_timeout_) return;

        reporter_.Check(continuous_frames_ >= 2,
                        "Continuous stream supplies at least two usable semantic snapshots");
        reporter_.Check(continuous_rejected_frames_ == 0,
                        "Every usable continuous snapshot is accepted by MapHandler");
        reporter_.Check(continuous_metadata_consistent_,
                        "Tree type, frame and resolution remain consistent across snapshots");
        reporter_.Check(continuous_frames_without_pose_ == 0,
                        "A robot pose is available before every processed map snapshot");
        reporter_.Check(continuous_local_clouds_valid_,
                        "Moving local obstacle and terrain windows remain semantically valid");
        reporter_.Check(continuous_frames_with_terrain_ > 0,
                        "Continuous local windows retain queryable terrain support");
        reporter_.Check(continuous_max_robot_displacement_ >= 1.0f,
                        "Robot movement exercises local-window rebuilding");
        reporter_.Check(continuous_added_voxels_ > 0,
                        "Overlap-region obstacle appearances are detected");
        reporter_.Check(continuous_removed_voxels_ > 0,
                        "Overlap-region obstacle disappearances are detected");
        reporter_.Check(continuous_kdtree_verified_,
                        "Real map changes can populate the DynamicGraph new-point KD-tree");
        reporter_.Check(continuous_frames_with_dynamic_ > 0,
                        "Real map stream contains local dynamic_obstacle occupancy");
        reporter_.Check(continuous_dynamic_additions_ > 0,
                        "Dynamic obstacle appearances are reported immediately");
        reporter_.Check(continuous_dynamic_removals_ > 0,
                        "Dynamic voxels missing from the next local snapshot emit removal events");
        reporter_.Check(continuous_dynamic_layers_valid_,
                        "Effective dynamic occupancy exactly matches the latest local map and enters contour/collision layers");

        ROS_INFO_STREAM("Continuous map summary: frames=" << continuous_frames_
                        << ", rejected=" << continuous_rejected_frames_
                        << ", overlap additions=" << continuous_added_voxels_
                        << " in " << continuous_frames_with_additions_ << " frames"
                        << ", overlap removals=" << continuous_removed_voxels_
                        << " in " << continuous_frames_with_removals_ << " frames"
                        << ", window-boundary changes=" << continuous_boundary_voxels_
                        << ", stationary pairs=" << continuous_stationary_frame_pairs_
                        << ", stationary additions="
                        << continuous_stationary_added_voxels_
                        << ", stationary removals="
                        << continuous_stationary_removed_voxels_
                        << ", largest addition event="
                        << continuous_max_frame_additions_ << " at "
                        << continuous_max_addition_stamp_.toSec()
                        << ", largest removal event="
                        << continuous_max_frame_removals_ << " at "
                        << continuous_max_removal_stamp_.toSec()
                        << ", dynamic frames=" << continuous_frames_with_dynamic_
                        << ", max dynamic voxels=" << continuous_max_dynamic_voxels_
                        << ", dynamic additions=" << continuous_dynamic_additions_
                        << ", dynamic removals=" << continuous_dynamic_removals_);

        finished_ = true;
        reporter_.PrintSummary();
        ros::shutdown();
    }

    void PrintSemanticColorHistogram(const octomap::AbstractOcTree& tree) const {
        std::map<uint32_t, std::size_t> counts;
        std::size_t occupied_count = 0;

        const SemanticOctree* semantic_tree =
            dynamic_cast<const SemanticOctree*>(&tree);
        if (semantic_tree) {
            for (auto it = semantic_tree->begin_leafs(), end = semantic_tree->end_leafs();
                 it != end; ++it) {
                if (!semantic_tree->isNodeOccupied(*it)) continue;
                ++occupied_count;
                const SemanticOcTreeNode* node = it.operator->();
                if (!node) continue;
                const octomap::ColorOcTreeNode::Color color =
                    node->isSemanticsSet()
                        ? node->getSemantics().getSemanticColor()
                        : node->getColor();
                ++counts[MakeRgbKey(color.r, color.g, color.b)];
            }
        } else {
            const octomap::ColorOcTree* color_tree =
                dynamic_cast<const octomap::ColorOcTree*>(&tree);
            if (!color_tree) {
                ROS_WARN_STREAM("Tree type " << tree.getTreeType()
                                << " has no semantic/color histogram support.");
                return;
            }
            for (auto it = color_tree->begin_leafs(), end = color_tree->end_leafs();
                 it != end; ++it) {
                if (!color_tree->isNodeOccupied(*it)) continue;
                ++occupied_count;
                const auto color = it->getColor();
                ++counts[MakeRgbKey(color.r, color.g, color.b)];
            }
        }

        std::vector<std::pair<uint32_t, std::size_t>> sorted(counts.begin(),
                                                              counts.end());
        std::sort(sorted.begin(), sorted.end(),
                  [](const std::pair<uint32_t, std::size_t>& lhs,
                     const std::pair<uint32_t, std::size_t>& rhs) {
                      return lhs.second > rhs.second;
                  });

        ROS_INFO_STREAM("Occupied leaves=" << occupied_count
                        << ", distinct top-1 semantic colors=" << sorted.size());
        const std::size_t print_count = std::min<std::size_t>(sorted.size(), 20);
        for (std::size_t i = 0; i < print_count; ++i) {
            const double ratio = occupied_count == 0
                ? 0.0
                : 100.0 * static_cast<double>(sorted[i].second) /
                      static_cast<double>(occupied_count);
            ROS_INFO_STREAM("  semantic RGB " << RgbString(sorted[i].first)
                            << ": " << sorted[i].second << " leaves ("
                            << std::fixed << std::setprecision(2) << ratio << "%)");
        }
    }

    std::size_t CountOccupiedLeaves(const octomap::AbstractOcTree& tree) const {
        std::size_t occupied_count = 0;
        const SemanticOctree* semantic_tree =
            dynamic_cast<const SemanticOctree*>(&tree);
        if (semantic_tree) {
            for (auto it = semantic_tree->begin_leafs(), end = semantic_tree->end_leafs();
                 it != end; ++it) {
                if (semantic_tree->isNodeOccupied(*it)) ++occupied_count;
            }
            return occupied_count;
        }

        const octomap::ColorOcTree* color_tree =
            dynamic_cast<const octomap::ColorOcTree*>(&tree);
        if (!color_tree) return 0;
        for (auto it = color_tree->begin_leafs(), end = color_tree->end_leafs();
             it != end; ++it) {
            if (color_tree->isNodeOccupied(*it)) ++occupied_count;
        }
        return occupied_count;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber subscriber_;
    ros::Subscriber pose_subscriber_;
    ros::WallTimer idle_timer_;
    std::string topic_;
    std::string robot_pose_topic_;
    std::string world_frame_;
    MapHandlerParams params_;
    Point3D robot_position_ = Point3D(0.0f, 0.0f, 0.0f);
    int minimum_pose_samples_ = 20;
    int pose_sample_count_ = 0;
    MapHandler handler_;
    TestReporter reporter_;
    bool using_synthetic_map_ = false;
    bool continuous_mode_ = false;
    bool finished_ = false;
    bool pose_frame_warning_printed_ = false;
    bool pose_received_ = false;
    bool received_map_message_ = false;
    bool continuous_tree_metadata_set_ = false;
    bool continuous_metadata_consistent_ = true;
    bool continuous_local_clouds_valid_ = true;
    bool continuous_kdtree_verified_ = false;
    bool continuous_dynamic_layers_valid_ = true;
    double stream_idle_timeout_ = 3.0;
    double overlap_margin_ = 0.5;
    ros::WallTime last_map_wall_time_;
    std::string continuous_tree_id_;
    std::string continuous_tree_frame_;
    double continuous_tree_resolution_ = 0.0;
    std::size_t continuous_frames_ = 0;
    std::size_t continuous_rejected_frames_ = 0;
    std::size_t continuous_frames_without_pose_ = 0;
    std::size_t continuous_frames_with_terrain_ = 0;
    std::size_t continuous_frames_with_additions_ = 0;
    std::size_t continuous_frames_with_removals_ = 0;
    std::size_t continuous_added_voxels_ = 0;
    std::size_t continuous_removed_voxels_ = 0;
    std::size_t continuous_boundary_voxels_ = 0;
    std::size_t continuous_stationary_frame_pairs_ = 0;
    std::size_t continuous_stationary_added_voxels_ = 0;
    std::size_t continuous_stationary_removed_voxels_ = 0;
    std::size_t continuous_max_frame_additions_ = 0;
    std::size_t continuous_max_frame_removals_ = 0;
    std::size_t continuous_frames_with_dynamic_ = 0;
    std::size_t continuous_max_dynamic_voxels_ = 0;
    std::size_t continuous_dynamic_additions_ = 0;
    std::size_t continuous_dynamic_removals_ = 0;
    ros::Time continuous_max_addition_stamp_;
    ros::Time continuous_max_removal_stamp_;
    float continuous_max_robot_displacement_ = 0.0f;
    Point3D first_continuous_robot_position_ = Point3D(0.0f, 0.0f, 0.0f);
    Point3D previous_map_robot_position_ = Point3D(0.0f, 0.0f, 0.0f);
};

}  // namespace

// The production executable defines these globals in far_planner.cpp.  This
// standalone test target links MapHandler and FARUtil without FARMaster/main.
PointCloudPtr FARUtil::surround_obs_cloud_(new PointCloud());
PointCloudPtr FARUtil::stack_new_cloud_(new PointCloud());
PointCloudPtr FARUtil::stack_dyobs_cloud_(new PointCloud());
PointCloudPtr FARUtil::cur_new_cloud_(new PointCloud());
PointCloudPtr FARUtil::cur_dyobs_cloud_(new PointCloud());
PointCloudPtr FARUtil::cur_scan_cloud_(new PointCloud());
PointKdTreePtr FARUtil::kdtree_new_cloud_(new pcl::KdTreeFLANN<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_filter_cloud_(new pcl::KdTreeFLANN<PCLPoint>());
const float FARUtil::kEpsilon = 1e-7f;
const float FARUtil::kINF = std::numeric_limits<float>::max();
bool FARUtil::IsStaticEnv = true;
bool FARUtil::IsDebug = true;
bool FARUtil::IsMultiLayer = false;
Point3D FARUtil::robot_pos;
Point3D FARUtil::odom_pos;
Point3D FARUtil::map_origin;
Point3D FARUtil::free_odom_p;
float FARUtil::robot_dim = 0.8f;
float FARUtil::kAngleNoise = 0.0f;
float FARUtil::kCellLength = 0.5f;
float FARUtil::kCellHeight = 0.5f;
float FARUtil::vehicle_height = 0.5f;
float FARUtil::kLeafSize = 0.5f;
float FARUtil::kHeightVoxel = 1.0f;
float FARUtil::kNavClearDist = 0.9f;
float FARUtil::kNearDist = 0.8f;
float FARUtil::kMatchDist = 1.0f;
float FARUtil::kProjectDist = 0.5f;
float FARUtil::kNewPIThred = 2.0f;
float FARUtil::kSensorRange = 15.0f;
float FARUtil::kMarginDist = 14.0f;
float FARUtil::kLocalPlanRange = 15.0f;
float FARUtil::kMarginHeight = 1.0f;
float FARUtil::kTerrainRange = 15.0f;
float FARUtil::kFreeZ = 0.1f;
float FARUtil::kObsDecayTime = 10.0f;
float FARUtil::kNewDecayTime = 2.0f;
int FARUtil::kDyObsThred = 4;
int FARUtil::KNewPointC = 10;
int FARUtil::kObsInflate = 2;
float FARUtil::kTolerZ = 2.0f;
float FARUtil::kAcceptAlign = 0.0f;
float FARUtil::kVizRatio = 1.0f;
double FARUtil::systemStartTime = 0.0;
TimeMeasure FARUtil::Timer;
std::string FARUtil::worldFrameId = "map";
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
PointKdTreePtr MapHandler::kdtree_terrain_clould_;

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_handler_test");
    MapHandlerIntegrationTest test;
    ros::spin();
    return test.exitCode();
}
