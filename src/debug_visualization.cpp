#include "far_planner/debug_visualization.h"

#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <unordered_set>

namespace {

std_msgs::ColorRGBA Color(float r, float g, float b, float a = 1.0f) {
    std_msgs::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

geometry_msgs::Point Geo(const Point3D& p, float dz = 0.0f) {
    geometry_msgs::Point q;
    q.x = p.x; q.y = p.y; q.z = p.z + dz;
    return q;
}

void InitMarker(visualization_msgs::Marker& marker,
                const std_msgs::Header& header, const std::string& ns,
                int id, int type, float scale,
                const std_msgs::ColorRGBA& color, float lifetime = 0.0f) {
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = scale;
    marker.color = color;
    if (lifetime > 0.0f) marker.lifetime = ros::Duration(lifetime);
}

visualization_msgs::Marker DeleteAll(const std_msgs::Header& header) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.action = visualization_msgs::Marker::DELETEALL;
    return marker;
}

std::string EdgeKey(std::size_t first, std::size_t second) {
    if (second < first) std::swap(first, second);
    return std::to_string(first) + ":" + std::to_string(second);
}

const char* SourceName(GraphNodeSource source) {
    switch (source) {
        case GraphNodeSource::ODOM: return "ODOM";
        case GraphNodeSource::GOAL: return "GOAL";
        case GraphNodeSource::STATIC_CANDIDATE: return "STATIC_CAND";
        case GraphNodeSource::STATIC_GLOBAL: return "STATIC_GLOBAL";
        case GraphNodeSource::DYNAMIC_LOCAL: return "DYNAMIC";
        case GraphNodeSource::PATH_HISTORY: return "PATH_HISTORY";
        default: return "UNKNOWN";
    }
}

const char* DirectionName(NodeFreeDirect direction) {
    switch (direction) {
        case NodeFreeDirect::CONVEX: return "CONVEX";
        case NodeFreeDirect::CONCAVE: return "CONCAVE";
        case NodeFreeDirect::PILLAR: return "PILLAR";
        default: return "UNKNOWN";
    }
}

const char* EdgeModeName(EdgeValidationMode mode) {
    switch (mode) {
        case EdgeValidationMode::CONTOUR_FOLLOW: return "CONTOUR";
        case EdgeValidationMode::CLIP_ATTEMPT: return "CLIP_ATTEMPT";
        default: return "VIS";
    }
}

const char* EdgeReasonName(EdgeRejectReason reason) {
    switch (reason) {
        case EdgeRejectReason::NONE: return "ACCEPTED";
        case EdgeRejectReason::NOT_CURRENT_ADJACENT: return "NOT_ADJACENT";
        case EdgeRejectReason::UNREACHABLE: return "UNREACHABLE";
        case EdgeRejectReason::DIRECTION_REJECTED: return "DIRECTION";
        case EdgeRejectReason::DIRECTION_SPARSIFIED: return "SPARSIFIED";
        case EdgeRejectReason::TRIANGLE_SPARSIFIED: return "TRIANGLE_SPARSE";
        case EdgeRejectReason::STATIC_CLOUD_BLOCKED: return "STATIC_CLOUD";
        case EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED: return "DYNAMIC_CLOUD";
        case EdgeRejectReason::POLYGON_BLOCKED: return "POLYGON";
        case EdgeRejectReason::TERRAIN_BLOCKED: return "TERRAIN";
        case EdgeRejectReason::OFFSET_FAILED: return "OFFSET";
        case EdgeRejectReason::SELF_POLYGON_BLOCKED: return "SELF_POLYGON";
        case EdgeRejectReason::OTHER_STATIC_BLOCKED: return "OTHER_STATIC";
        case EdgeRejectReason::CLIPPED_CONTOUR: return "OUTSIDE_WINDOW";
        case EdgeRejectReason::VOTE_PENDING: return "VOTE_PENDING";
    }
    return "UNKNOWN";
}

std_msgs::ColorRGBA EdgeReasonColor(EdgeRejectReason reason) {
    switch (reason) {
        case EdgeRejectReason::STATIC_CLOUD_BLOCKED:
            return Color(1.0f, 0.38f, 0.0f);
        case EdgeRejectReason::DYNAMIC_CLOUD_BLOCKED:
            return Color(1.0f, 0.0f, 0.2f);
        case EdgeRejectReason::TERRAIN_BLOCKED:
            return Color(1.0f, 1.0f, 0.0f);
        case EdgeRejectReason::NOT_CURRENT_ADJACENT:
            return Color(0.65f, 0.0f, 0.9f);
        case EdgeRejectReason::CLIPPED_CONTOUR:
            return Color(0.1f, 0.45f, 1.0f);
        case EdgeRejectReason::VOTE_PENDING:
            return Color(1.0f, 1.0f, 1.0f);
        case EdgeRejectReason::DIRECTION_REJECTED:
        case EdgeRejectReason::DIRECTION_SPARSIFIED:
            return Color(1.0f, 0.8f, 0.0f);
        case EdgeRejectReason::TRIANGLE_SPARSIFIED:
            return Color(0.1f, 0.8f, 1.0f);
        default:
            return Color(1.0f, 0.55f, 0.0f);
    }
}

std_msgs::ColorRGBA VoxelColor(DebugVoxelClass type) {
    switch (type) {
        case DebugVoxelClass::STATIC_OBSTACLE: return Color(1.0f, 0.05f, 0.03f);
        case DebugVoxelClass::TRANSIENT_OBSTACLE: return Color(0.9f, 0.0f, 0.9f);
        case DebugVoxelClass::TERRAIN_SUPPORT: return Color(0.0f, 0.9f, 0.1f);
        case DebugVoxelClass::MORPHOLOGY_REMOVED: return Color(0.1f, 0.75f, 1.0f);
        default: return Color(0.45f, 0.45f, 0.45f);
    }
}

cv::Scalar VoxelCvColor(DebugVoxelClass type) {
    switch (type) {
        case DebugVoxelClass::STATIC_OBSTACLE: return cv::Scalar(15, 15, 245);
        case DebugVoxelClass::TRANSIENT_OBSTACLE: return cv::Scalar(220, 15, 220);
        case DebugVoxelClass::TERRAIN_SUPPORT: return cv::Scalar(15, 220, 15);
        case DebugVoxelClass::MORPHOLOGY_REMOVED: return cv::Scalar(245, 190, 25);
        default: return cv::Scalar(100, 100, 100);
    }
}

std::uint32_t PackedRgb(const std_msgs::ColorRGBA& color) {
    return (static_cast<std::uint32_t>(color.r * 255.0f) << 16) |
           (static_cast<std::uint32_t>(color.g * 255.0f) << 8) |
           static_cast<std::uint32_t>(color.b * 255.0f);
}

sensor_msgs::Image ImageMessage(const cv::Mat& input,
                                const std_msgs::Header& header) {
    cv::Mat bgr;
    if (input.empty()) {
        bgr = cv::Mat::zeros(1, 1, CV_8UC3);
    } else if (input.type() == CV_8UC3) {
        bgr = input;
    } else {
        cv::Mat mono;
        if (input.type() == CV_8UC1) {
            mono = input;
        } else if (input.type() == CV_32FC1) {
            input.convertTo(mono, CV_8UC1, 255.0);
        } else {
            input.convertTo(mono, CV_8UC1);
        }
        cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
    }
    if (!bgr.isContinuous()) bgr = bgr.clone();
    sensor_msgs::Image output;
    output.header = header;
    output.height = bgr.rows;
    output.width = bgr.cols;
    output.encoding = "bgr8";
    output.is_bigendian = false;
    output.step = bgr.cols * 3;
    output.data.assign(bgr.datastart, bgr.dataend);
    return output;
}

cv::Mat DisplayImage(const cv::Mat& input, const cv::Size& size) {
    cv::Mat bgr;
    if (input.empty()) return cv::Mat::zeros(size, CV_8UC3);
    if (input.type() == CV_8UC3) {
        bgr = input;
    } else {
        cv::Mat mono;
        if (input.type() == CV_8UC1) mono = input;
        else if (input.type() == CV_32FC1) input.convertTo(mono, CV_8UC1, 255.0);
        else input.convertTo(mono, CV_8UC1);
        cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
    }
    cv::resize(bgr, bgr, size, 0.0, 0.0, cv::INTER_NEAREST);
    return bgr;
}

cv::Point WorldPixel(const Point3D& point, const Point3D& robot,
                     float range, const cv::Rect& panel) {
    range = std::max(0.1f, range);
    return cv::Point(
        panel.x + static_cast<int>(((point.x - robot.x) / (2.0f * range) + 0.5f) * panel.width),
        panel.y + static_cast<int>((0.5f - (point.y - robot.y) / (2.0f * range)) * panel.height));
}

bool InPanel(const cv::Point& point, const cv::Rect& panel) {
    return point.x >= panel.x && point.x < panel.x + panel.width &&
           point.y >= panel.y && point.y < panel.y + panel.height;
}

std::vector<const ContourMatchDebugRecord*> SelectMatchRecords(
    const std::vector<ContourMatchDebugRecord>& matches,
    const int max_per_corner) {
    std::vector<const ContourMatchDebugRecord*> selected;
    std::unordered_map<std::string, int> counts;
    for (const auto& match : matches) {
        std::ostringstream key;
        key << std::fixed << std::setprecision(3)
            << match.contour_position.x << ':' << match.contour_position.y
            << ':' << match.contour_position.z;
        int& count = counts[key.str()];
        const bool accepted =
            match.outcome == ContourMatchDebugOutcome::ACCEPTED;
        if (max_per_corner > 0 && count >= max_per_corner && !accepted) {
            continue;
        }
        selected.push_back(&match);
        ++count;
    }
    return selected;
}

}  // namespace

void GraphDebugVisualizer::Init(const ros::NodeHandle& nh,
                                const DebugVisualizationParams& params,
                                const std::string& world_frame) {
    nh_ = nh;
    params_ = params;
    world_frame_ = world_frame;
    if (!params_.enabled) return;
    classified_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/far_debug/classified_cloud", 1, true);
    contours_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/far_debug/current_contours", 1, true);
    ctnodes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/far_debug/current_ctnodes", 1, true);
    graph_before_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/far_debug/graph_before_update", 1, true);
    matches_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/far_debug/match_results", 1, true);
    duplicates_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/far_debug/duplicate_suppressed", 1, true);
    node_events_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/far_debug/node_events", 1, true);
    edge_events_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/far_debug/edge_events", 1, true);
    graph_after_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/far_debug/graph_after_update", 1, true);
    graph_delta_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/far_debug/graph_delta", 1, true);
    edge_diagnostics_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/far_debug/edge_diagnostics", 1, true);
    color_legend_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/far_debug/color_legend", 1, true);
    if (params_.publish_images) {
        static_base_image_pub_ = nh_.advertise<sensor_msgs::Image>("/far_debug/raster/static_base", 1, true);
        static_processed_image_pub_ = nh_.advertise<sensor_msgs::Image>("/far_debug/raster/static_processed", 1, true);
        dynamic_base_image_pub_ = nh_.advertise<sensor_msgs::Image>("/far_debug/raster/dynamic_base", 1, true);
        dynamic_processed_image_pub_ = nh_.advertise<sensor_msgs::Image>("/far_debug/raster/dynamic_processed", 1, true);
        summary_image_pub_ = nh_.advertise<sensor_msgs::Image>("/far_debug/frame_summary_image", 1, true);
    }
    if (params_.show_opencv_window) {
        const char* display = std::getenv("DISPLAY");
        if (display == nullptr || display[0] == '\0') {
            params_.show_opencv_window = false;
            ROS_WARN("FAR OpenCV debug window requested, but DISPLAY is not set; "
                     "continuing with the ROS debug image only.");
        } else {
            try {
                cv::namedWindow(opencv_window_name_,
                                cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
                cv::resizeWindow(opencv_window_name_,
                                 params_.opencv_window_width,
                                 params_.opencv_window_height);
                opencv_window_active_ = true;
                cv::waitKey(1);
                ROS_INFO("FAR OpenCV frame-summary window enabled (%dx%d initial size).",
                         params_.opencv_window_width,
                         params_.opencv_window_height);
            } catch (const cv::Exception& exception) {
                params_.show_opencv_window = false;
                ROS_ERROR("Unable to create FAR OpenCV debug window: %s. "
                          "Continuing without the window.", exception.what());
            }
        }
    }
    ROS_INFO("FAR frame debug visualization enabled.");
}

void GraphDebugVisualizer::ProcessGuiEvents() {
    if (!opencv_window_active_) return;
    try {
        cv::waitKey(1);
    } catch (const cv::Exception& exception) {
        opencv_window_active_ = false;
        ROS_ERROR("FAR OpenCV debug window event processing failed: %s; "
                  "disabling the window.", exception.what());
    }
}

void GraphDebugVisualizer::PublishColorLegend(
    const std_msgs::Header& header, const Point3D& robot_position) {
    visualization_msgs::MarkerArray output;
    output.markers.push_back(DeleteAll(header));

    struct LegendLine {
        const char* text;
        std_msgs::ColorRGBA color;
    };
    struct LegendColumn {
        const char* title;
        std::vector<LegendLine> lines;
    };
    const std_msgs::ColorRGBA white = Color(1.0f, 1.0f, 1.0f);
    const std::vector<LegendColumn> columns = {
        {"CLASSIFIED / CONTOUR / CT", {
            {"RED = static obstacle", Color(1.0f, 0.05f, 0.03f)},
            {"MAGENTA = explicit dynamic", Color(0.9f, 0.0f, 0.9f)},
            {"GREEN = terrain support", Color(0.0f, 0.9f, 0.1f)},
            {"CYAN = morphology removed", Color(0.1f, 0.75f, 1.0f)},
            {"GRAY = ignored point", Color(0.55f, 0.55f, 0.55f)},
            {"BLUE = static contour", Color(0.0f, 0.45f, 1.0f)},
            {"MAGENTA = dynamic contour", Color(0.95f, 0.0f, 0.95f)},
            {"GREEN = convex CT", Color(0.0f, 1.0f, 0.1f)},
            {"ORANGE = concave CT", Color(1.0f, 0.45f, 0.0f)},
            {"MAGENTA CUBE = pillar CT", Color(0.75f, 0.0f, 0.9f)},
            {"GRAY = unknown CT", Color(0.55f, 0.55f, 0.55f)},
            {"RED CUBE = CLIP overlay", Color(1.0f, 0.0f, 0.0f)},
            {"CYAN = CT surface dirs", Color(0.1f, 0.8f, 1.0f)},
            {"YELLOW = CT topology dir", Color(1.0f, 1.0f, 0.0f)},
            {"GREEN LINE = verified free dir", Color(0.0f, 1.0f, 0.2f)},
            {"GRAY LINE = unverified free dir", Color(0.5f, 0.5f, 0.5f)}}},
        {"MATCH / DUPLICATE / NODE EVENT", {
            {"GREEN = match accepted", Color(0.0f, 1.0f, 0.1f)},
            {"YELLOW = one-to-one lost", Color(1.0f, 0.85f, 0.0f)},
            {"RED = match rejected/blocked", Color(1.0f, 0.1f, 0.1f)},
            {"ORANGE = duplicate suppressed", Color(1.0f, 0.4f, 0.0f)},
            {"GREEN = node created", Color(0.0f, 1.0f, 0.2f)},
            {"BLUE = node promoted", Color(0.0f, 0.35f, 1.0f)},
            {"CYAN = node position updated", Color(0.0f, 0.9f, 0.9f)},
            {"RED CUBE = node removed", Color(1.0f, 0.0f, 0.0f)},
            {"MAGENTA CUBE = node cleared", Color(1.0f, 0.0f, 0.8f)}}},
        {"GRAPH BEFORE / AFTER", {
            {"ORANGE = static candidate", Color(1.0f, 0.65f, 0.0f)},
            {"BLUE = static global", Color(0.1f, 0.45f, 1.0f)},
            {"MAGENTA CUBE = dynamic local", Color(0.95f, 0.0f, 0.9f)},
            {"GREEN = odom/start", Color(0.1f, 1.0f, 0.1f)},
            {"RED = goal", Color(1.0f, 0.1f, 0.1f)},
            {"GRAY = other node", Color(0.7f, 0.7f, 0.7f)},
            {"GRAY LINE = active edge", Color(0.55f, 0.55f, 0.55f)},
            {"PINK LINE = CLIP attempt", Color(1.0f, 0.2f, 0.75f)},
            {"ORANGE LINE = static blocked, kept", Color(1.0f, 0.4f, 0.0f)},
            {"RED LINE = dynamic blocked, kept", Color(1.0f, 0.0f, 0.1f)},
            {"PURPLE LINE = topology blocked", Color(0.7f, 0.0f, 0.85f)}}},
        {"EDGE EVENT / RESULT", {
            {"CYAN = edge created", Color(0.0f, 1.0f, 1.0f)},
            {"PINK = CLIP attempt created", Color(1.0f, 0.2f, 0.75f)},
            {"RED = old edge deleted this frame", Color(1.0f, 0.0f, 0.0f)},
            {"ORANGE = static blocked, identity kept", Color(1.0f, 0.4f, 0.0f)},
            {"PINK = dynamic blocked, identity kept", Color(1.0f, 0.0f, 0.15f)},
            {"BLUE = edge restored", Color(0.0f, 0.3f, 1.0f)},
            {"PURPLE = edge topology blocked", Color(0.6f, 0.0f, 0.8f)},
            {"CYAN = flat-triangle long edge pruned", Color(0.1f, 0.8f, 1.0f)},
            {"ORANGE = eligible graph", Color(1.0f, 0.45f, 0.1f)},
            {"YELLOW = odom-connected search", Color(0.9f, 0.9f, 0.1f)}}}
    };

    const float range = std::max(4.0f, params_.sensor_range);
    const float left = robot_position.x - range * 0.78f;
    const float top = robot_position.y + range * 0.90f;
    const float column_spacing = range * 0.57f;
    const float row_spacing = 0.52f;
    int marker_id = 5000;
    for (std::size_t column = 0; column < columns.size(); ++column) {
        const float x = left + static_cast<float>(column) * column_spacing;
        const auto append_text = [&](const std::string& text_value,
                                     const std_msgs::ColorRGBA& color,
                                     const float y, const float scale) {
            visualization_msgs::Marker text_marker;
            InitMarker(text_marker, header,
                       "debug_color_legend_" + std::to_string(column),
                       marker_id++,
                       visualization_msgs::Marker::TEXT_VIEW_FACING, scale,
                       color);
            text_marker.pose.position.x = x;
            text_marker.pose.position.y = y;
            text_marker.pose.position.z = robot_position.z + 1.8f;
            text_marker.text = text_value;
            output.markers.push_back(text_marker);
        };
        append_text(columns[column].title, white, top, 0.42f);
        for (std::size_t row = 0; row < columns[column].lines.size(); ++row) {
            append_text(columns[column].lines[row].text,
                        columns[column].lines[row].color,
                        top - (static_cast<float>(row) + 1.0f) * row_spacing,
                        0.32f);
        }
    }
    color_legend_pub_.publish(output);
}

DebugGraphSnapshot GraphDebugVisualizer::CaptureGraph(const NodePtrStack& graph) {
    DebugGraphSnapshot snapshot;
    for (const auto& node : graph) {
        if (!node) continue;
        DebugNodeSnapshot item;
        item.id = node->id;
        item.position = node->position;
        item.source = node->source;
        item.free_direct = node->free_direct;
        item.is_merged = node->is_merged;
        item.is_finalized = node->is_finalized;
        item.observed = node->observed_in_semantic_snapshot;
        item.boundary_clipped = node->is_transient_contour_endpoint;
        item.contour_necessary =
            node->ctnode && node->ctnode->is_contour_necessary;
        item.topology_blocked = node->topology_blocked;
        item.static_seen_count = node->static_seen_count;
        item.static_missed_count = node->static_missed_count;
        item.topology_missed_count = node->topology_missed_count;
        snapshot.nodes[node->id] = item;
    }
    for (const auto& node : graph) {
        if (!node) continue;
        for (const auto& neighbor : node->connect_nodes) {
            if (!neighbor || node->id >= neighbor->id ||
                !snapshot.nodes.count(neighbor->id)) continue;
            DebugEdgeSnapshot edge;
            edge.first_id = node->id;
            edge.second_id = neighbor->id;
            edge.first_position = node->position;
            edge.second_position = neighbor->position;
            const auto state = node->edge_states.find(neighbor->id);
            if (state != node->edge_states.end()) edge.state = state->second;
            snapshot.edges[EdgeKey(edge.first_id, edge.second_id)] = edge;
        }
    }
    return snapshot;
}

void GraphDebugVisualizer::PublishClassifiedCloud(
    const std::vector<DebugVoxelPoint>& points,
    const std_msgs::Header& header) {
    if (!params_.enabled) return;
    sensor_msgs::PointCloud2 cloud;
    cloud.header = header;
    cloud.header.frame_id = world_frame_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        8,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::PointField::FLOAT32,
        "debug_class", 1, sensor_msgs::PointField::UINT8,
        "label", 1, sensor_msgs::PointField::UINT32,
        "semantic_confidence", 1, sensor_msgs::PointField::FLOAT32,
        "cost", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"), rgb(cloud, "rgb");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> category(cloud, "debug_class");
    sensor_msgs::PointCloud2Iterator<std::uint32_t> label(cloud, "label");
    sensor_msgs::PointCloud2Iterator<float> confidence(cloud, "semantic_confidence"), cost(cloud, "cost");
    for (const auto& point : points) {
        *x = point.position.x; *y = point.position.y; *z = point.position.z;
        const std::uint32_t packed = PackedRgb(VoxelColor(point.classification));
        float packed_float = 0.0f;
        std::memcpy(&packed_float, &packed, sizeof(float));
        *rgb = packed_float;
        *category = static_cast<std::uint8_t>(point.classification);
        *label = point.label;
        *confidence = point.semantic_confidence;
        *cost = point.cost;
        ++x; ++y; ++z; ++rgb; ++category; ++label; ++confidence; ++cost;
    }
    cloud.is_dense = false;
    classified_cloud_pub_.publish(cloud);
}

void GraphDebugVisualizer::PublishContours(
    const std_msgs::Header& header,
    const std::vector<PointStack>& static_contours,
    const std::vector<PointStack>& dynamic_contours) {
    visualization_msgs::MarkerArray output;
    output.markers.push_back(DeleteAll(header));
    visualization_msgs::Marker statics, dynamics;
    InitMarker(statics, header, "debug_static_contours", 0,
               visualization_msgs::Marker::LINE_LIST, 0.07f,
               Color(0.0f, 0.45f, 1.0f, 0.95f));
    InitMarker(dynamics, header, "debug_dynamic_contours", 0,
               visualization_msgs::Marker::LINE_LIST, 0.09f,
               Color(0.95f, 0.0f, 0.95f, 0.95f));
    const auto append = [](const std::vector<PointStack>& contours,
                           visualization_msgs::Marker& marker) {
        for (const auto& contour : contours) {
            if (contour.size() < 2) continue;
            for (std::size_t i = 0; i < contour.size(); ++i) {
                marker.points.push_back(Geo(contour[i], 0.05f));
                marker.points.push_back(Geo(contour[(i + 1) % contour.size()], 0.05f));
            }
        }
    };
    append(static_contours, statics);
    append(dynamic_contours, dynamics);
    output.markers.push_back(statics);
    output.markers.push_back(dynamics);
    contours_pub_.publish(output);
}

void GraphDebugVisualizer::PublishContourNodes(
    const std_msgs::Header& header, const CTNodeStack& contour_nodes) {
    visualization_msgs::MarkerArray output;
    output.markers.push_back(DeleteAll(header));
    visualization_msgs::Marker convex, concave, pillar, unknown, clipped,
        surface, topology, free_reliable, free_unreliable;
    InitMarker(convex, header, "debug_ct_convex", 0, visualization_msgs::Marker::SPHERE_LIST, 0.22f, Color(0,1,0.1f));
    InitMarker(concave, header, "debug_ct_concave", 0, visualization_msgs::Marker::SPHERE_LIST, 0.22f, Color(1,0.45f,0));
    InitMarker(pillar, header, "debug_ct_pillar", 0, visualization_msgs::Marker::CUBE_LIST, 0.25f, Color(0.75f,0,0.9f));
    InitMarker(unknown, header, "debug_ct_unknown", 0, visualization_msgs::Marker::SPHERE_LIST, 0.18f, Color(0.45f,0.45f,0.45f));
    InitMarker(clipped, header, "debug_ct_clipped", 0, visualization_msgs::Marker::CUBE_LIST, 0.31f, Color(1,0,0,0.75f));
    InitMarker(surface, header, "debug_ct_surface_dirs", 0, visualization_msgs::Marker::LINE_LIST, 0.035f, Color(0.1f,0.8f,1));
    InitMarker(topology, header, "debug_ct_topology_dir", 0, visualization_msgs::Marker::LINE_LIST, 0.055f, Color(1,1,0));
    InitMarker(free_reliable, header, "debug_ct_free_dir_reliable", 0,
               visualization_msgs::Marker::LINE_LIST, 0.075f,
               Color(0,1,0.2f));
    InitMarker(free_unreliable, header, "debug_ct_free_dir_unreliable", 0,
               visualization_msgs::Marker::LINE_LIST, 0.045f,
               Color(0.5f,0.5f,0.5f,0.7f));
    int text_id = 1000;
    for (std::size_t i = 0; i < contour_nodes.size(); ++i) {
        const auto& node = contour_nodes[i];
        if (!node) continue;
        const geometry_msgs::Point p = Geo(node->position, 0.10f);
        if (node->free_direct == NodeFreeDirect::CONVEX) convex.points.push_back(p);
        else if (node->free_direct == NodeFreeDirect::CONCAVE) concave.points.push_back(p);
        else if (node->free_direct == NodeFreeDirect::PILLAR) pillar.points.push_back(p);
        else unknown.points.push_back(p);
        if (node->is_boundary_clipped) clipped.points.push_back(p);
        if (node->free_direct != NodeFreeDirect::PILLAR) {
            surface.points.push_back(p);
            surface.points.push_back(Geo(node->position + node->surf_dirs.first * 0.55f, 0.10f));
            surface.points.push_back(p);
            surface.points.push_back(Geo(node->position + node->surf_dirs.second * 0.55f, 0.10f));
            Point3D direction = node->surf_dirs.first + node->surf_dirs.second;
            if (direction.norm_flat() > 1e-5f) {
                direction = direction.normalize_flat();
                topology.points.push_back(p);
                topology.points.push_back(Geo(node->position + direction * 0.70f, 0.12f));
            }
            if (node->free_space_dir.norm_flat() > 1e-5f) {
                visualization_msgs::Marker& free_marker =
                    node->is_free_space_dir_reliable
                        ? free_reliable : free_unreliable;
                free_marker.points.push_back(p);
                free_marker.points.push_back(Geo(
                    node->position + node->free_space_dir * 0.85f, 0.14f));
            }
        }
        if (params_.show_text) {
            visualization_msgs::Marker text;
            InitMarker(text, header, "debug_ct_labels", text_id++,
                       visualization_msgs::Marker::TEXT_VIEW_FACING, 0.22f, Color(1,1,1));
            text.pose.position = Geo(node->position, 0.38f);
            std::ostringstream stream;
            stream << "CT" << i << " " << DirectionName(node->free_direct)
                   << (node->source == GraphNodeSource::DYNAMIC_LOCAL ? " D" : " S");
            if (node->is_boundary_clipped) stream << " CLIP";
            if (node->is_contour_necessary) stream << " NEC";
            if (node->free_direct != NodeFreeDirect::PILLAR &&
                node->free_direct != NodeFreeDirect::UNKNOW) {
                stream << (node->is_free_space_dir_reliable ? " F+" : " F?");
            }
            text.text = stream.str();
            output.markers.push_back(text);
        }
    }
    output.markers.push_back(convex); output.markers.push_back(concave);
    output.markers.push_back(pillar); output.markers.push_back(unknown);
    output.markers.push_back(clipped); output.markers.push_back(surface);
    output.markers.push_back(topology);
    output.markers.push_back(free_reliable);
    output.markers.push_back(free_unreliable);
    ctnodes_pub_.publish(output);
}

void GraphDebugVisualizer::PublishMatches(
    const std_msgs::Header& header,
    const std::vector<ContourMatchDebugRecord>& matches,
    const std::vector<ContourDuplicateDebugRecord>& duplicates) {
    visualization_msgs::MarkerArray match_output;
    match_output.markers.push_back(DeleteAll(header));
    visualization_msgs::Marker accepted, lost, rejected;
    InitMarker(accepted, header, "debug_match_accepted", 0, visualization_msgs::Marker::LINE_LIST, 0.07f, Color(0,1,0.1f));
    InitMarker(lost, header, "debug_match_one_to_one_lost", 0, visualization_msgs::Marker::LINE_LIST, 0.035f, Color(1,0.85f,0,0.8f));
    InitMarker(rejected, header, "debug_match_rejected", 0, visualization_msgs::Marker::LINE_LIST, 0.025f, Color(1,0.1f,0.1f,0.55f));
    int text_id = 2000;
    for (const auto* match_ptr : SelectMatchRecords(
             matches, params_.max_match_candidates_per_corner)) {
        const auto& match = *match_ptr;
        visualization_msgs::Marker* marker = &rejected;
        if (match.outcome == ContourMatchDebugOutcome::ACCEPTED) marker = &accepted;
        else if (match.outcome == ContourMatchDebugOutcome::ONE_TO_ONE_LOST) marker = &lost;
        marker->points.push_back(Geo(match.contour_position, 0.22f));
        marker->points.push_back(Geo(match.graph_position, 0.22f));
        if (params_.show_text &&
            (match.outcome == ContourMatchDebugOutcome::ACCEPTED ||
             match.outcome == ContourMatchDebugOutcome::LINE_BLOCKED)) {
            visualization_msgs::Marker text;
            InitMarker(text, header, "debug_match_labels", text_id++,
                       visualization_msgs::Marker::TEXT_VIEW_FACING, 0.20f, Color(1,1,1));
            text.pose.position = Geo((match.contour_position + match.graph_position) * 0.5f, 0.45f);
            std::ostringstream stream;
            stream << "N" << match.graph_node_id << " d=" << std::fixed
                   << std::setprecision(2) << match.distance << " a="
                   << std::setprecision(0) << match.direction_angle_deg
                   << " R=" << std::setprecision(2) << match.match_radius;
            text.text = stream.str();
            match_output.markers.push_back(text);
        }
    }
    match_output.markers.push_back(accepted);
    match_output.markers.push_back(lost);
    match_output.markers.push_back(rejected);
    matches_pub_.publish(match_output);

    visualization_msgs::MarkerArray duplicate_output;
    duplicate_output.markers.push_back(DeleteAll(header));
    visualization_msgs::Marker links, points;
    InitMarker(links, header, "debug_duplicate_links", 0, visualization_msgs::Marker::LINE_LIST, 0.06f, Color(1,0.4f,0));
    InitMarker(points, header, "debug_duplicate_suppressed", 0, visualization_msgs::Marker::CUBE_LIST, 0.28f, Color(1,0.25f,0));
    text_id = 3000;
    for (const auto& duplicate : duplicates) {
        points.points.push_back(Geo(duplicate.suppressed_position, 0.28f));
        links.points.push_back(Geo(duplicate.suppressed_position, 0.28f));
        links.points.push_back(Geo(duplicate.keeper_position, 0.28f));
        if (params_.show_text) {
            visualization_msgs::Marker text;
            InitMarker(text, header, "debug_duplicate_labels", text_id++,
                       visualization_msgs::Marker::TEXT_VIEW_FACING, 0.20f, Color(1,0.65f,0.1f));
            text.pose.position = Geo(duplicate.suppressed_position, 0.55f);
            std::ostringstream stream;
            if (duplicate.is_history_consolidation) {
                stream << "MERGE N" << duplicate.suppressed_node_id
                       << " ->N" << duplicate.keeper_node_id << " d="
                       << std::fixed << std::setprecision(2)
                       << duplicate.distance << " R="
                       << duplicate.duplicate_radius;
            } else {
                stream << "SUPPRESS d=" << std::fixed
                       << std::setprecision(2) << duplicate.distance
                       << " R=" << duplicate.duplicate_radius;
                if (duplicate.keeper_is_historical) {
                    stream << " ->N" << duplicate.keeper_node_id;
                } else {
                    stream << " ->current CT";
                }
            }
            text.text = stream.str();
            duplicate_output.markers.push_back(text);
        }
    }
    duplicate_output.markers.push_back(links);
    duplicate_output.markers.push_back(points);
    duplicates_pub_.publish(duplicate_output);
}

void GraphDebugVisualizer::PublishGraph(
    const std_msgs::Header& header, const DebugGraphSnapshot& graph,
    const ros::Publisher& publisher, const std::string& ns, float z_offset) {
    visualization_msgs::MarkerArray output;
    output.markers.push_back(DeleteAll(header));
    visualization_msgs::Marker candidates, globals, dynamics, odom, goals,
        others, active_edges, clip_attempt_edges, static_blocked_edges,
        dynamic_blocked_edges, topology_blocked_edges;
    InitMarker(candidates, header, ns + "_static_candidate", 0,
               visualization_msgs::Marker::SPHERE_LIST, 0.20f,
               Color(1.0f, 0.65f, 0.0f));
    InitMarker(globals, header, ns + "_static_global", 0,
               visualization_msgs::Marker::SPHERE_LIST, 0.22f,
               Color(0.1f, 0.45f, 1.0f));
    InitMarker(dynamics, header, ns + "_dynamic_local", 0,
               visualization_msgs::Marker::CUBE_LIST, 0.23f,
               Color(0.95f, 0.0f, 0.9f));
    InitMarker(odom, header, ns + "_odom", 0,
               visualization_msgs::Marker::SPHERE_LIST, 0.28f,
               Color(0.1f, 1.0f, 0.1f));
    InitMarker(goals, header, ns + "_goal", 0,
               visualization_msgs::Marker::SPHERE_LIST, 0.28f,
               Color(1.0f, 0.1f, 0.1f));
    InitMarker(others, header, ns + "_other", 0,
               visualization_msgs::Marker::SPHERE_LIST, 0.18f,
               Color(0.7f, 0.7f, 0.7f));
    InitMarker(active_edges, header, ns + "_edge_active", 0,
               visualization_msgs::Marker::LINE_LIST, 0.035f,
               Color(0.55f, 0.55f, 0.55f, 0.75f));
    InitMarker(clip_attempt_edges, header, ns + "_edge_clip_attempt", 0,
               visualization_msgs::Marker::LINE_LIST, 0.10f,
               Color(1.0f, 0.2f, 0.75f, 0.95f));
    InitMarker(static_blocked_edges, header, ns + "_edge_static_blocked", 0,
               visualization_msgs::Marker::LINE_LIST, 0.08f,
               Color(1.0f, 0.4f, 0.0f));
    InitMarker(dynamic_blocked_edges, header, ns + "_edge_dynamic_blocked", 0,
               visualization_msgs::Marker::LINE_LIST, 0.09f,
               Color(1.0f, 0.0f, 0.1f));
    InitMarker(topology_blocked_edges, header,
               ns + "_edge_topology_blocked", 0,
               visualization_msgs::Marker::LINE_LIST, 0.08f,
               Color(0.7f, 0.0f, 0.85f));
    for (const auto& item : graph.nodes) {
        const DebugNodeSnapshot& node = item.second;
        visualization_msgs::Marker* marker = &others;
        if (node.source == GraphNodeSource::STATIC_CANDIDATE) marker = &candidates;
        else if (node.source == GraphNodeSource::STATIC_GLOBAL) marker = &globals;
        else if (node.source == GraphNodeSource::DYNAMIC_LOCAL) marker = &dynamics;
        else if (node.source == GraphNodeSource::ODOM) marker = &odom;
        else if (node.source == GraphNodeSource::GOAL) marker = &goals;
        marker->points.push_back(Geo(node.position, z_offset));
        if (params_.show_text) {
            visualization_msgs::Marker label;
            InitMarker(label, header, ns + "_labels",
                       static_cast<int>(node.id & 0x7fffffffU),
                       visualization_msgs::Marker::TEXT_VIEW_FACING, 0.18f,
                       Color(1, 1, 1, 0.9f));
            label.pose.position = Geo(node.position, z_offset + 0.30f);
            std::ostringstream stream;
            stream << "N" << node.id << " " << SourceName(node.source);
            if (node.contour_necessary) stream << " NEC";
            if (node.source == GraphNodeSource::STATIC_CANDIDATE ||
                node.source == GraphNodeSource::STATIC_GLOBAL) {
                stream << " seen=" << node.static_seen_count
                       << " miss=" << node.static_missed_count;
                if (node.topology_missed_count > 0) {
                    stream << " topo=" << node.topology_missed_count;
                }
            }
            label.text = stream.str();
            output.markers.push_back(label);
        }
    }
    for (const auto& item : graph.edges) {
        const DebugEdgeSnapshot& edge = item.second;
        visualization_msgs::Marker* marker = &active_edges;
        if (edge.state.dynamic_blocked) marker = &dynamic_blocked_edges;
        else if (!edge.state.static_valid) marker = &static_blocked_edges;
        else if (edge.state.topology_blocked) marker = &topology_blocked_edges;
        else if (edge.state.validation_mode ==
                 EdgeValidationMode::CLIP_ATTEMPT) {
            marker = &clip_attempt_edges;
        }
        if (edge.state.has_clearance_geometry &&
            edge.state.route_points.size() >= 2) {
            // The stored route is projected away from the obstacle surface.
            // Include the two anchor stubs so the debug drawing remains a
            // continuous N-first -> clearance route -> N-second graph edge.
            marker->points.push_back(Geo(edge.first_position, z_offset));
            marker->points.push_back(Geo(edge.state.route_points.front(),
                                         z_offset));
            for (std::size_t index = 1;
                 index < edge.state.route_points.size(); ++index) {
                marker->points.push_back(Geo(
                    edge.state.route_points[index - 1], z_offset));
                marker->points.push_back(Geo(
                    edge.state.route_points[index], z_offset));
            }
            marker->points.push_back(Geo(edge.state.route_points.back(),
                                         z_offset));
            marker->points.push_back(Geo(edge.second_position, z_offset));
        } else {
            marker->points.push_back(Geo(edge.first_position, z_offset));
            marker->points.push_back(Geo(edge.second_position, z_offset));
        }
    }
    output.markers.push_back(candidates); output.markers.push_back(globals);
    output.markers.push_back(dynamics); output.markers.push_back(odom);
    output.markers.push_back(goals); output.markers.push_back(others);
    output.markers.push_back(active_edges);
    output.markers.push_back(clip_attempt_edges);
    output.markers.push_back(static_blocked_edges);
    output.markers.push_back(dynamic_blocked_edges);
    output.markers.push_back(topology_blocked_edges);
    publisher.publish(output);
}

void GraphDebugVisualizer::PublishGraphDelta(
    const std_msgs::Header& header, const DebugGraphSnapshot& before,
    const DebugGraphSnapshot& after, const NodePtrStack&,
    const NodePtrStack& cleared_nodes) {
    const float lifetime = params_.event_marker_lifetime;
    visualization_msgs::MarkerArray node_output;
    node_output.markers.push_back(DeleteAll(header));
    visualization_msgs::Marker created, promoted, updated, removed, cleared;
    InitMarker(created, header, "debug_node_created", 0, visualization_msgs::Marker::SPHERE_LIST, 0.30f, Color(0,1,0.2f), lifetime);
    InitMarker(promoted, header, "debug_node_promoted", 0, visualization_msgs::Marker::SPHERE_LIST, 0.34f, Color(0,0.35f,1), lifetime);
    InitMarker(updated, header, "debug_node_updated", 0, visualization_msgs::Marker::SPHERE_LIST, 0.24f, Color(0,0.9f,0.9f), lifetime);
    InitMarker(removed, header, "debug_node_removed", 0, visualization_msgs::Marker::CUBE_LIST, 0.36f, Color(1,0,0), lifetime);
    InitMarker(cleared, header, "debug_node_cleared", 0, visualization_msgs::Marker::CUBE_LIST, 0.42f, Color(1,0,0.8f), lifetime);
    int text_id = 4000;
    for (const auto& item : after.nodes) {
        const auto previous = before.nodes.find(item.first);
        if (previous == before.nodes.end()) {
            created.points.push_back(Geo(item.second.position, 0.35f));
        } else if (previous->second.source == GraphNodeSource::STATIC_CANDIDATE &&
                   item.second.source == GraphNodeSource::STATIC_GLOBAL) {
            promoted.points.push_back(Geo(item.second.position, 0.38f));
        } else if ((previous->second.position - item.second.position).norm_flat() > 1e-4f) {
            updated.points.push_back(Geo(item.second.position, 0.32f));
        }
    }
    std::unordered_set<std::size_t> explicitly_cleared;
    for (const auto& node : cleared_nodes) {
        if (node) explicitly_cleared.insert(node->id);
    }
    for (const auto& item : before.nodes) {
        if (after.nodes.count(item.first)) continue;
        const bool was_cleared = explicitly_cleared.count(item.first) > 0;
        (was_cleared ? cleared : removed).points.push_back(Geo(item.second.position, 0.42f));
        if (params_.show_text) {
            visualization_msgs::Marker text;
            InitMarker(text, header, "debug_removed_labels", text_id++,
                       visualization_msgs::Marker::TEXT_VIEW_FACING, 0.22f, Color(1,0.2f,0.2f), lifetime);
            text.pose.position = Geo(item.second.position, 0.70f);
            std::ostringstream stream;
            stream << (was_cleared ? "CLEARED " : "REMOVED ") << "N"
                   << item.first << " " << SourceName(item.second.source);
            if (item.second.source == GraphNodeSource::DYNAMIC_LOCAL) stream << " DYNAMIC_EXPIRED";
            else if (item.second.topology_missed_count > 0) stream << " TOPOLOGY miss=" << item.second.topology_missed_count;
            else if (item.second.static_missed_count > 0) stream << " FREE miss=" << item.second.static_missed_count;
            text.text = stream.str();
            node_output.markers.push_back(text);
        }
    }
    node_output.markers.push_back(created); node_output.markers.push_back(promoted);
    node_output.markers.push_back(updated); node_output.markers.push_back(removed);
    node_output.markers.push_back(cleared);
    node_events_pub_.publish(node_output);

    visualization_msgs::MarkerArray edge_output;
    edge_output.markers.push_back(DeleteAll(header));
    visualization_msgs::Marker created_edge, created_clip_attempt,
        deleted_edge, static_blocked, dynamic_blocked, restored,
        topology_blocked;
    InitMarker(created_edge, header, "debug_edge_created", 0, visualization_msgs::Marker::LINE_LIST, 0.09f, Color(0,1,1), lifetime);
    InitMarker(created_clip_attempt, header, "debug_edge_clip_attempt_created",
               0, visualization_msgs::Marker::LINE_LIST, 0.13f,
               Color(1.0f, 0.2f, 0.75f), lifetime);
    InitMarker(deleted_edge, header, "debug_edge_deleted", 0, visualization_msgs::Marker::LINE_LIST, 0.12f, Color(1,0,0), lifetime);
    InitMarker(static_blocked, header, "debug_edge_static_blocked", 0, visualization_msgs::Marker::LINE_LIST, 0.11f, Color(1,0.4f,0), lifetime);
    InitMarker(dynamic_blocked, header, "debug_edge_dynamic_blocked", 0, visualization_msgs::Marker::LINE_LIST, 0.13f, Color(1,0,0.15f), lifetime);
    InitMarker(restored, header, "debug_edge_restored", 0, visualization_msgs::Marker::LINE_LIST, 0.11f, Color(0,0.3f,1), lifetime);
    InitMarker(topology_blocked, header, "debug_edge_topology_blocked", 0, visualization_msgs::Marker::LINE_LIST, 0.11f, Color(0.6f,0,0.8f), lifetime);
    const auto append = [](const DebugEdgeSnapshot& edge,
                           visualization_msgs::Marker& marker,
                           const float z_offset) {
        marker.points.push_back(Geo(edge.first_position, z_offset));
        marker.points.push_back(Geo(edge.second_position, z_offset));
    };
    for (const auto& item : after.edges) {
        const auto previous = before.edges.find(item.first);
        if (previous == before.edges.end()) {
            append(item.second,
                   item.second.state.validation_mode ==
                           EdgeValidationMode::CLIP_ATTEMPT
                       ? created_clip_attempt : created_edge,
                   0.58f);
        } else if (item.second.state.IsActive() && !previous->second.state.IsActive()) {
            append(item.second, restored, 0.54f);
        } else if (!item.second.state.static_valid && previous->second.state.static_valid) {
            append(item.second, static_blocked, 0.52f);
        } else if (item.second.state.dynamic_blocked && !previous->second.state.dynamic_blocked) {
            append(item.second, dynamic_blocked, 0.52f);
        } else if (item.second.state.topology_blocked && !previous->second.state.topology_blocked) {
            append(item.second, topology_blocked, 0.52f);
        }
    }
    for (const auto& item : before.edges) {
        if (!after.edges.count(item.first)) {
            append(item.second, deleted_edge, 0.46f);
        }
    }
    // Deleted history is drawn lower; a replacement created at the same XY
    // (common for per-frame CLIP identities) remains visibly cyan on top.
    edge_output.markers.push_back(deleted_edge);
    edge_output.markers.push_back(static_blocked); edge_output.markers.push_back(dynamic_blocked);
    edge_output.markers.push_back(restored); edge_output.markers.push_back(topology_blocked);
    edge_output.markers.push_back(created_edge);
    edge_output.markers.push_back(created_clip_attempt);
    edge_events_pub_.publish(edge_output);
    graph_delta_pub_.publish(edge_output);
}

void GraphDebugVisualizer::PublishEdgeDiagnostics(
    const std_msgs::Header& header,
    const std::vector<EdgeDiagnostic>& edge_diagnostics) {
    visualization_msgs::MarkerArray output;
    output.markers.push_back(DeleteAll(header));
    int marker_id = 0;
    for (const auto& diagnostic : edge_diagnostics) {
        // /viz_graph_topic already contains all raw visibility rejections.
        // Keep this focused, labelled topic for the much smaller set of
        // contour-follow candidates, otherwise labels become unreadable.
        if (!IsContourTopologyMode(diagnostic.mode)) continue;
        const std_msgs::ColorRGBA color = EdgeReasonColor(diagnostic.reason);
        visualization_msgs::Marker line;
        std::string reason_ns = EdgeReasonName(diagnostic.reason);
        std::transform(reason_ns.begin(), reason_ns.end(), reason_ns.begin(),
                       [](unsigned char value) {
                           return static_cast<char>(std::tolower(value));
                       });
        InitMarker(line, header, "debug_edge_reject_" + reason_ns,
                   marker_id++, visualization_msgs::Marker::LINE_LIST,
                   IsContourTopologyMode(diagnostic.mode)
                       ? 0.16f : 0.09f,
                   color);
        line.points.push_back(Geo(diagnostic.start, 0.60f));
        line.points.push_back(Geo(diagnostic.end, 0.60f));
        output.markers.push_back(line);

        visualization_msgs::Marker label;
        InitMarker(label, header, "debug_edge_reject_labels", marker_id++,
                   visualization_msgs::Marker::TEXT_VIEW_FACING, 0.24f,
                   color);
        label.pose.position = Geo(
            (diagnostic.start + diagnostic.end) / 2.0f, 0.78f);
        std::ostringstream text;
        text << 'N' << diagnostic.first_id << "-N" << diagnostic.second_id
             << ' ' << EdgeModeName(diagnostic.mode) << ':'
             << EdgeReasonName(diagnostic.reason);
        label.text = text.str();
        output.markers.push_back(label);
    }
    edge_diagnostics_pub_.publish(output);
}

void GraphDebugVisualizer::PublishImages(
    std::uint64_t frame_sequence, const std_msgs::Header& header,
    const Point3D& robot_position,
    const std::vector<DebugVoxelPoint>& classified_points,
    const std::vector<PointStack>& static_contours,
    const std::vector<PointStack>& dynamic_contours,
    const CTNodeStack& contour_nodes,
    const std::vector<ContourMatchDebugRecord>& matches,
    const std::vector<ContourDuplicateDebugRecord>& duplicates,
    const DebugGraphSnapshot& before, const DebugGraphSnapshot& after,
    const NodePtrStack& eligible_graph, const NodePtrStack& search_graph,
    const cv::Mat& static_base, const cv::Mat& static_processed,
    const cv::Mat& dynamic_base, const cv::Mat& dynamic_processed) {
    if (!params_.publish_images && !params_.show_opencv_window &&
        !params_.save_frames) return;
    if (params_.publish_images) {
        static_base_image_pub_.publish(ImageMessage(static_base, header));
        static_processed_image_pub_.publish(ImageMessage(static_processed, header));
        dynamic_base_image_pub_.publish(ImageMessage(dynamic_base, header));
        dynamic_processed_image_pub_.publish(ImageMessage(dynamic_processed, header));
    }

    constexpr int kPanel = 480;
    constexpr int kHeader = 90;
    cv::Mat summary = cv::Mat::zeros(kPanel * 2 + kHeader, kPanel * 2, CV_8UC3);
    const cv::Rect input_panel(0, kHeader, kPanel, kPanel);
    const cv::Rect raster_panel(kPanel, kHeader, kPanel, kPanel);
    const cv::Rect match_panel(0, kHeader + kPanel, kPanel, kPanel);
    const cv::Rect graph_panel(kPanel, kHeader + kPanel, kPanel, kPanel);
    for (const auto& point : classified_points) {
        const cv::Point pixel = WorldPixel(point.position, robot_position, params_.sensor_range, input_panel);
        if (InPanel(pixel, input_panel)) cv::circle(summary, pixel, 1, VoxelCvColor(point.classification), -1);
    }
    cv::circle(summary, WorldPixel(robot_position, robot_position, params_.sensor_range, input_panel), 5, cv::Scalar(255,255,255), -1);

    cv::Mat static_display = DisplayImage(static_processed, raster_panel.size());
    cv::Mat dynamic_display = DisplayImage(dynamic_processed, raster_panel.size());
    cv::addWeighted(static_display, 0.75, dynamic_display, 0.45, 0.0, summary(raster_panel));

    for (const auto& item : before.nodes) {
        const cv::Point pixel = WorldPixel(item.second.position, robot_position, params_.sensor_range, match_panel);
        if (InPanel(pixel, match_panel)) cv::circle(summary, pixel, 2, cv::Scalar(150,150,150), -1);
    }
    for (const auto* match_ptr : SelectMatchRecords(
             matches, params_.max_match_candidates_per_corner)) {
        const auto& match = *match_ptr;
        const cv::Point a = WorldPixel(match.contour_position, robot_position, params_.sensor_range, match_panel);
        const cv::Point b = WorldPixel(match.graph_position, robot_position, params_.sensor_range, match_panel);
        cv::Scalar color = match.outcome == ContourMatchDebugOutcome::ACCEPTED
            ? cv::Scalar(30,240,30)
            : (match.outcome == ContourMatchDebugOutcome::ONE_TO_ONE_LOST
               ? cv::Scalar(20,220,240) : cv::Scalar(20,20,240));
        cv::line(summary, a, b, color, 1);
    }
    for (const auto& duplicate : duplicates) {
        cv::drawMarker(summary, WorldPixel(duplicate.suppressed_position, robot_position, params_.sensor_range, match_panel),
                       cv::Scalar(0,100,255), cv::MARKER_TILTED_CROSS, 8, 2);
    }

    std::unordered_set<std::size_t> search_ids;
    for (const auto& node : search_graph) if (node) search_ids.insert(node->id);
    for (const auto& item : after.edges) {
        cv::Scalar color(130,130,130);
        if (!item.second.state.static_valid) color = cv::Scalar(0,120,255);
        if (item.second.state.dynamic_blocked) color = cv::Scalar(0,0,255);
        if (search_ids.count(item.second.first_id) && search_ids.count(item.second.second_id) && item.second.state.IsActive())
            color = cv::Scalar(0,240,240);
        const std::vector<Point3D>& route_points =
            item.second.state.route_points;
        if (item.second.state.has_clearance_geometry &&
            route_points.size() >= 2) {
            cv::line(summary,
                     WorldPixel(item.second.first_position, robot_position,
                                params_.sensor_range, graph_panel),
                     WorldPixel(route_points.front(), robot_position,
                                params_.sensor_range, graph_panel),
                     color, 1);
            for (std::size_t index = 1; index < route_points.size(); ++index) {
                cv::line(summary,
                         WorldPixel(route_points[index - 1], robot_position,
                                    params_.sensor_range, graph_panel),
                         WorldPixel(route_points[index], robot_position,
                                    params_.sensor_range, graph_panel),
                         color, 1);
            }
            cv::line(summary,
                     WorldPixel(route_points.back(), robot_position,
                                params_.sensor_range, graph_panel),
                     WorldPixel(item.second.second_position, robot_position,
                                params_.sensor_range, graph_panel),
                     color, 1);
        } else {
            cv::line(summary,
                     WorldPixel(item.second.first_position, robot_position,
                                params_.sensor_range, graph_panel),
                     WorldPixel(item.second.second_position, robot_position,
                                params_.sensor_range, graph_panel),
                     color, 1);
        }
    }
    for (const auto& item : after.nodes) {
        const cv::Point p = WorldPixel(item.second.position, robot_position, params_.sensor_range, graph_panel);
        if (InPanel(p, graph_panel)) cv::circle(summary, p, 3, cv::Scalar(255,255,255), -1);
    }

    cv::putText(summary, "1 classified cloud", cv::Point(8,kHeader+22), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 1);
    cv::putText(summary, "2 raster / contours", cv::Point(kPanel+8,kHeader+22), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 1);
    cv::putText(summary, "3 current-to-history matching", cv::Point(8,kHeader+kPanel+22), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 1);
    cv::putText(summary, "4 final graph / search graph", cv::Point(kPanel+8,kHeader+kPanel+22), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 1);

    std::size_t s=0,d=0,t=0,i=0,r=0;
    for (const auto& point : classified_points) {
        if (point.classification == DebugVoxelClass::STATIC_OBSTACLE) ++s;
        else if (point.classification == DebugVoxelClass::TRANSIENT_OBSTACLE) ++d;
        else if (point.classification == DebugVoxelClass::TERRAIN_SUPPORT) ++t;
        else if (point.classification == DebugVoxelClass::MORPHOLOGY_REMOVED) ++r;
        else ++i;
    }
    std::ostringstream title;
    title << "frame=" << frame_sequence << " stamp=" << std::fixed << std::setprecision(3)
          << header.stamp.toSec() << " points S/D/T/I/R=" << s << "/" << d << "/" << t << "/" << i << "/" << r;
    cv::putText(summary, title.str(), cv::Point(10,25), cv::FONT_HERSHEY_SIMPLEX, 0.58, cv::Scalar(255,255,255), 1);
    std::ostringstream counts;
    counts << "contours S/D=" << static_contours.size() << "/" << dynamic_contours.size()
           << " CT=" << contour_nodes.size() << " matches=" << matches.size()
           << " suppressed=" << duplicates.size() << " graph " << before.nodes.size()
           << "->" << after.nodes.size() << " eligible=" << eligible_graph.size()
           << " search=" << search_graph.size();
    cv::putText(summary, counts.str(), cv::Point(10,52), cv::FONT_HERSHEY_SIMPLEX, 0.52, cv::Scalar(210,210,210), 1);
    cv::putText(
        summary,
        "colors: cloud R=static M=dynamic G=terrain C=morph-removed | match G=accept Y=lost R=blocked | graph O=eligible Y=search",
        cv::Point(10,76), cv::FONT_HERSHEY_SIMPLEX, 0.42,
        cv::Scalar(220,220,220), 1);

    if (params_.publish_images) {
        summary_image_pub_.publish(ImageMessage(summary, header));
    }
    if (opencv_window_active_) {
        try {
            cv::imshow(opencv_window_name_, summary);
            cv::waitKey(1);
        } catch (const cv::Exception& exception) {
            opencv_window_active_ = false;
            ROS_ERROR("Unable to update FAR OpenCV debug window: %s; "
                      "disabling the window.", exception.what());
        }
    }
    if (params_.save_frames && !params_.save_directory.empty()) {
        std::ostringstream path;
        path << params_.save_directory << "/frame_" << std::setw(8)
             << std::setfill('0') << frame_sequence << ".png";
        if (!cv::imwrite(path.str(), summary)) {
            ROS_WARN_THROTTLE(2.0, "Failed to save FAR debug frame to %s", path.str().c_str());
        }
    }
}

void GraphDebugVisualizer::PublishFrame(
    std::uint64_t frame_sequence, const std_msgs::Header& source_header,
    const Point3D& robot_position,
    const std::vector<DebugVoxelPoint>& classified_points,
    const std::vector<PointStack>& static_contours,
    const std::vector<PointStack>& dynamic_contours,
    const CTNodeStack& contour_nodes,
    const std::vector<ContourMatchDebugRecord>& matches,
    const std::vector<ContourDuplicateDebugRecord>& duplicates,
    const NodePtrStack& new_nodes, const NodePtrStack& cleared_nodes,
    const DebugGraphSnapshot& before,
    const DebugGraphSnapshot& after,
    const std::vector<EdgeDiagnostic>& edge_diagnostics,
    const NodePtrStack& eligible_graph,
    const NodePtrStack& search_graph, const cv::Mat& static_base,
    const cv::Mat& static_processed, const cv::Mat& dynamic_base,
    const cv::Mat& dynamic_processed) {
    if (!params_.enabled) return;
    std_msgs::Header header = source_header;
    header.seq = static_cast<std::uint32_t>(frame_sequence);
    header.frame_id = world_frame_;
    PublishClassifiedCloud(classified_points, header);
    PublishContours(header, static_contours, dynamic_contours);
    PublishContourNodes(header, contour_nodes);
    PublishGraph(header, before, graph_before_pub_, "debug_graph_before", 0.04f);
    PublishMatches(header, matches, duplicates);
    PublishGraphDelta(header, before, after, new_nodes, cleared_nodes);
    PublishGraph(header, after, graph_after_pub_, "debug_graph_after", 0.08f);
    PublishEdgeDiagnostics(header, edge_diagnostics);
    PublishColorLegend(header, robot_position);
    PublishImages(frame_sequence, header, robot_position, classified_points,
                  static_contours, dynamic_contours, contour_nodes, matches,
                  duplicates, before, after, eligible_graph, search_graph,
                  static_base, static_processed, dynamic_base, dynamic_processed);
}
