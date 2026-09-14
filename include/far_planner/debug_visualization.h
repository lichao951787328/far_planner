#ifndef FAR_PLANNER_DEBUG_VISUALIZATION_H
#define FAR_PLANNER_DEBUG_VISUALIZATION_H

#include "far_planner/utility.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

enum class DebugVoxelClass : std::uint8_t {
    STATIC_OBSTACLE = 0,
    TRANSIENT_OBSTACLE = 1,
    TERRAIN_SUPPORT = 2,
    IGNORE = 3,
    // A valid classified point whose source XY pixel was removed by the
    // configured erosion+dilation opening. It is visualization-only and is
    // not passed to any FAR geometry layer.
    MORPHOLOGY_REMOVED = 4
};

struct DebugVoxelPoint {
    Point3D position;
    DebugVoxelClass classification = DebugVoxelClass::IGNORE;
    std::uint32_t label = 0;
    float semantic_confidence = 0.0f;
    float cost = 0.0f;
};

enum class ContourMatchDebugOutcome : std::uint8_t {
    CANDIDATE = 0,
    ACCEPTED,
    ONE_TO_ONE_LOST,
    DISTANCE_REJECTED,
    DIRECTION_REJECTED,
    LINE_BLOCKED
};

struct ContourMatchDebugRecord {
    Point3D contour_position;
    Point3D graph_position;
    std::size_t graph_node_id = 0;
    float distance = 0.0f;
    float direction_angle_deg = 0.0f;
    float match_radius = 0.0f;
    float score = 0.0f;
    bool free_direction_reliable = false;
    ContourMatchDebugOutcome outcome =
        ContourMatchDebugOutcome::CANDIDATE;
};

struct ContourDuplicateDebugRecord {
    Point3D suppressed_position;
    Point3D keeper_position;
    std::size_t suppressed_node_id = 0;
    std::size_t keeper_node_id = 0;
    float distance = 0.0f;
    float duplicate_radius = 0.0f;
    bool keeper_is_historical = false;
    // True when two already-created static graph identities were
    // consolidated.  The normal false case is a current CT candidate that
    // never entered the graph.
    bool is_history_consolidation = false;
};

struct DebugVisualizationParams {
    bool enabled = false;
    bool deterministic_step_mode = false;
    bool show_text = true;
    bool publish_images = true;
    bool show_opencv_window = false;
    bool save_frames = false;
    std::string save_directory;
    int opencv_window_width = 1200;
    int opencv_window_height = 1000;
    int max_match_candidates_per_corner = 3;
    float event_marker_lifetime = 2.0f;
    float sensor_range = 10.0f;
};

struct DebugNodeSnapshot {
    std::size_t id = 0;
    Point3D position;
    GraphNodeSource source = GraphNodeSource::UNKNOWN;
    NodeFreeDirect free_direct = NodeFreeDirect::UNKNOW;
    bool is_merged = false;
    bool is_finalized = false;
    bool observed = false;
    bool boundary_clipped = false;
    bool contour_necessary = false;
    bool topology_blocked = false;
    int static_seen_count = 0;
    int static_missed_count = 0;
    int topology_missed_count = 0;
};

struct DebugEdgeSnapshot {
    std::size_t first_id = 0;
    std::size_t second_id = 0;
    Point3D first_position;
    Point3D second_position;
    GraphEdgeState state;
};

struct DebugGraphSnapshot {
    std::unordered_map<std::size_t, DebugNodeSnapshot> nodes;
    std::unordered_map<std::string, DebugEdgeSnapshot> edges;
};

/** Read-only frame transaction used exclusively by optional debug output. */
class GraphDebugVisualizer {
public:
    void Init(const ros::NodeHandle& nh,
              const DebugVisualizationParams& params,
              const std::string& world_frame);

    bool Enabled() const { return params_.enabled; }

    /** Pump HighGUI while bag replay is paused so the window stays usable. */
    void ProcessGuiEvents();

    static DebugGraphSnapshot CaptureGraph(const NodePtrStack& graph);

    void PublishClassifiedCloud(const std::vector<DebugVoxelPoint>& points,
                                const std_msgs::Header& header);

    void PublishFrame(
        std::uint64_t frame_sequence, const std_msgs::Header& source_header,
        const Point3D& robot_position,
        const std::vector<DebugVoxelPoint>& classified_points,
        const std::vector<PointStack>& static_contours,
        const std::vector<PointStack>& dynamic_contours,
        const CTNodeStack& contour_nodes,
        const std::vector<ContourMatchDebugRecord>& matches,
        const std::vector<ContourDuplicateDebugRecord>& duplicates,
        const NodePtrStack& new_nodes,
        const NodePtrStack& cleared_nodes,
        const DebugGraphSnapshot& before,
        const DebugGraphSnapshot& after,
        const std::vector<EdgeDiagnostic>& edge_diagnostics,
        const NodePtrStack& eligible_graph,
        const NodePtrStack& search_graph,
        const cv::Mat& static_base,
        const cv::Mat& static_processed,
        const cv::Mat& dynamic_base,
        const cv::Mat& dynamic_processed);

private:
    ros::NodeHandle nh_;
    DebugVisualizationParams params_;
    std::string world_frame_;
    ros::Publisher classified_cloud_pub_;
    ros::Publisher contours_pub_;
    ros::Publisher ctnodes_pub_;
    ros::Publisher graph_before_pub_;
    ros::Publisher matches_pub_;
    ros::Publisher duplicates_pub_;
    ros::Publisher node_events_pub_;
    ros::Publisher edge_events_pub_;
    ros::Publisher graph_after_pub_;
    ros::Publisher graph_delta_pub_;
    ros::Publisher edge_diagnostics_pub_;
    ros::Publisher color_legend_pub_;
    ros::Publisher static_base_image_pub_;
    ros::Publisher static_processed_image_pub_;
    ros::Publisher dynamic_base_image_pub_;
    ros::Publisher dynamic_processed_image_pub_;
    ros::Publisher summary_image_pub_;
    bool opencv_window_active_ = false;
    const std::string opencv_window_name_ = "FAR frame summary";

    void PublishContours(const std_msgs::Header& header,
                         const std::vector<PointStack>& static_contours,
                         const std::vector<PointStack>& dynamic_contours);
    void PublishContourNodes(const std_msgs::Header& header,
                             const CTNodeStack& contour_nodes);
    void PublishMatches(
        const std_msgs::Header& header,
        const std::vector<ContourMatchDebugRecord>& matches,
        const std::vector<ContourDuplicateDebugRecord>& duplicates);
    void PublishGraph(const std_msgs::Header& header,
                      const DebugGraphSnapshot& graph,
                      const ros::Publisher& publisher,
                      const std::string& ns,
                      float z_offset);
    void PublishGraphDelta(const std_msgs::Header& header,
                           const DebugGraphSnapshot& before,
                           const DebugGraphSnapshot& after,
                           const NodePtrStack& new_nodes,
                           const NodePtrStack& cleared_nodes);
    void PublishEdgeDiagnostics(
        const std_msgs::Header& header,
        const std::vector<EdgeDiagnostic>& edge_diagnostics);
    void PublishColorLegend(const std_msgs::Header& header,
                            const Point3D& robot_position);
    void PublishImages(
        std::uint64_t frame_sequence, const std_msgs::Header& header,
        const Point3D& robot_position,
        const std::vector<DebugVoxelPoint>& classified_points,
        const std::vector<PointStack>& static_contours,
        const std::vector<PointStack>& dynamic_contours,
        const CTNodeStack& contour_nodes,
        const std::vector<ContourMatchDebugRecord>& matches,
        const std::vector<ContourDuplicateDebugRecord>& duplicates,
        const DebugGraphSnapshot& before,
        const DebugGraphSnapshot& after,
        const NodePtrStack& eligible_graph,
        const NodePtrStack& search_graph,
        const cv::Mat& static_base,
        const cv::Mat& static_processed,
        const cv::Mat& dynamic_base,
        const cv::Mat& dynamic_processed);
};

#endif
