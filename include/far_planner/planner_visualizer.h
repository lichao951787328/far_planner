#ifndef PLANNER_VISUALIZER_H
#define PLANNER_VISUALIZER_H

#include "utility.h"
#include "contour_graph.h"
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef visualization_msgs::Marker Marker;
typedef visualization_msgs::MarkerArray MarkerArray;


enum VizColor {
    RED     = 0,
    ORANGE  = 1,
    BLACK   = 2,
    YELLOW  = 3,
    BLUE    = 4,
    GREEN   = 5,
    EMERALD = 6,
    WHITE   = 7,
    MAGNA   = 8,
    PURPLE  = 9
};

class DPVisualizer {
private:
    ros::NodeHandle nh_;
    // Utility Cloud 
    PointCloudPtr point_cloud_ptr_;
    // rviz publisher 
    ros::Publisher viz_node_pub_, viz_path_pub_, nav_path_pub_;
    ros::Publisher viz_poly_pub_, viz_graph_pub_;
    ros::Publisher viz_contour_pub_, viz_map_pub_, viz_view_extend;
    ros::Publisher viz_static_global_pub_, viz_static_main_pub_;
    ros::Publisher viz_dynamic_local_pub_;
    ros::Publisher viz_eligible_graph_pub_, viz_search_graph_pub_;
    ros::Publisher viz_dynamic_blocked_pub_;

public:
    DPVisualizer() = default;
    ~DPVisualizer() = default;

    void Init(const ros::NodeHandle& nh);

    void VizNodes(const NodePtrStack& node_stack, 
                  const std::string& ns,
                  const VizColor& color,
                  const float scale=0.75f,
                  const float alpha=0.75f);

    void VizGlobalPolygons(const std::vector<PointPair>& contour_pairs, 
                           const std::vector<PointPair>& unmatched_pairs);

    void VizViewpointExtend(const NavNodePtr& ori_nav_ptr, const Point3D& extend_point);

    // True for non-attempts path
    void VizPath(const NodePtrStack& global_path,
                 const bool& is_free_nav=false,
                 const Point3D* commanded_goal=nullptr);

    void VizMapGrids(const PointStack& neighbor_centers, 
                     const PointStack& occupancy_centers,
                     const float& ceil_length,
                     const float& ceil_height);

    void VizContourGraph(const CTNodeStack& contour_graph);

    void VizPoint3D(const Point3D& point, 
                    const std::string& ns,
                    const VizColor& color,
                    const float scale=1.0f,
                    const float alpha=0.9f);

    void VizGraph(const NodePtrStack& graph);

    /** Publish source-separated semantic graph layers on independent topics. */
    void VizSemanticGraphLayers(const NodePtrStack& static_global,
                                const NodePtrStack& static_main,
                                const NodePtrStack& dynamic_local,
                                const NodePtrStack& eligible_graph,
                                const NodePtrStack& search_graph);
    void VizPointCloud(const ros::Publisher& viz_pub, 
                       const PointCloudPtr& pc);

    static void SetMarker(const VizColor& color, 
                   const std::string& ns,
                   const float& scale, 
                   const float& alpha, 
                   Marker& scan_marker,
                   const float& scale_ratio=FARUtil::kVizRatio);

    static void SetColor(const VizColor& color, const float& alpha, Marker& scan_marker);

};

#endif
