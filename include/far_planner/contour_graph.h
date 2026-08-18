#ifndef CONTOUR_GRAPH_H
#define CONTOUR_GRAPH_H

#include "utility.h"

struct ConnectPair
{
    cv::Point2f start_p;
    cv::Point2f end_p;

    ConnectPair() = default;
    ConnectPair(const cv::Point2f& p1, const cv::Point2f& p2):start_p(p1), end_p(p2) {}
    ConnectPair(const Point3D& p1, const Point3D& p2) {
        this->start_p.x = p1.x;
        this->start_p.y = p1.y;
        this->end_p.x = p2.x;
        this->end_p.y = p2.y;
    }
    
    bool operator ==(const ConnectPair& pt) const 
    {
        return (this->start_p == pt.start_p && this->end_p == pt.end_p) || (this->start_p == pt.end_p && this->end_p == pt.start_p);
    }
};

struct HeightPair
{
    float minH;
    float maxH;
    HeightPair() = default;
    HeightPair(const float& minV, const float& maxV):minH(minV), maxH(maxV) {}
    HeightPair(const Point3D& p1, const Point3D p2) {
        this->minH = std::min(p1.z, p2.z);
        this->maxH = std::max(p1.z, p2.z);
    }
};

struct ContourGraphParams {
    ContourGraphParams() = default;
    float kPillarPerimeter;
    float contour_projection_min = 0.15f;
    float contour_projection_step = 0.075f;
    float contour_projection_max = 0.60f;
    // Vertices this close to the square contour raster boundary are treated
    // as cropped observations, not as physical obstacle endpoints.
    float contour_boundary_guard = 0.40f;
};

class ContourGraph {
public:
    ContourGraph() = default;
    ~ContourGraph() = default;

    static CTNodeStack  contour_graph_;
    static std::vector<PointPair> global_contour_;
    static std::vector<PointPair> inactive_contour_;
    static std::vector<PointPair> unmatched_contour_;
    static std::vector<PointPair> boundary_contour_;
    static std::vector<PointPair> local_boundary_;

    void Init(const ContourGraphParams& params);
    
    // static functions
    void UpdateContourGraph(const NavNodePtr& odom_node_ptr,
                            const std::vector<std::vector<Point3D>>& filtered_contours);

    /** Build one local collision/contour view while preserving semantic source. */
    void UpdateContourGraph(
        const NavNodePtr& odom_node_ptr,
        const std::vector<std::vector<Point3D>>& static_contours,
        const std::vector<std::vector<Point3D>>& dynamic_contours);

    /** Legacy single-layer setter; treats the supplied cloud as static. */
    static void SetLocalCollisionCloud(const PointCloudPtr& collision_cloud);

    /** Latest cropped static/dynamic layers used for local edge checks. */
    static void SetLocalCollisionCloud(const PointCloudPtr& static_cloud,
                                       const PointCloudPtr& dynamic_cloud);

    /* Match current contour with global navigation nodes */
    void MatchContourWithNavGraph(const NodePtrStack& global_nodes,
                                  const NodePtrStack& near_nodes,
                                  CTNodeStack& new_convex_vertices,
                                  float static_duplicate_radius = 0.5f);

    void ExtractGlobalContours();

    static NavNodePtr MatchOutrangeNodeWithCTNode(const NavNodePtr& out_node_ptr, const NodePtrStack& near_nodes);

    static bool IsContourLineMatch(const NavNodePtr& inNode_ptr, const NavNodePtr& outNode_ptr, CTNodePtr& matched_ctnode);
    
    static bool IsNavNodesConnectFromContour(const NavNodePtr& node_ptr1, 
                                             const NavNodePtr& node_ptr2);

    static bool IsCTNodesConnectFromContour(const CTNodePtr& ctnode1, 
                                            const CTNodePtr& ctnode2);

    static bool IsNavNodesConnectFreePolygon(const NavNodePtr& node_ptr1,
                                             const NavNodePtr& node_ptr2);

    static bool IsNavNodesConnectFreeStaticPolygon(
        const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    static bool IsNavNodesConnectFreeDynamicLayer(
        const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    /** Pure geometry classification for an ordinary visibility edge. */
    static EdgeRejectReason ValidateVisibilityEdgeGeometry(
        const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2,
        bool include_dynamic = true);

    /** Validate and return the exact projected robot-centre geometry used by
     * a transient start/query visibility edge. */
    static EdgeValidationResult ValidateVisibilityEdgeWithRoute(
        const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2,
        bool include_dynamic = true);

    /** Pure geometry classification for a node-to-goal visibility edge. */
    static EdgeRejectReason ValidateGoalEdgeGeometry(
        const NavNodePtr& node_ptr, const NavNodePtr& goal_ptr);

    /** Goal counterpart of ValidateVisibilityEdgeWithRoute(). */
    static EdgeValidationResult ValidateGoalEdgeWithRoute(
        const NavNodePtr& node_ptr, const NavNodePtr& goal_ptr);

    /** Strict robot-centre line-of-sight check for odom directly to goal.
     * Unlike a contour-corner terminal edge, this route has no obstacle
     * endpoint and therefore receives neither corner projection nor endpoint
     * collision exclusion. */
    static EdgeValidationResult ValidateDirectOdomGoalEdgeWithRoute(
        const NavNodePtr& odom_ptr, const NavNodePtr& goal_ptr,
        bool include_dynamic = true);

    /** Check the current dynamic layer against the exact robot-centre route
     * geometry stored for a contour-follow edge. Unlike obstacle-anchor
     * visibility checks, no endpoint exclusion is applied because both route
     * endpoints have already been projected into free space. */
    static bool IsRouteConnectFreeDynamicLayer(const Point3D& route_start,
                                               const Point3D& route_end);

    /** Recheck stored free-side geometry against the latest local static
     * cloud and polygons.  This never queries the complete OctoMap. */
    static bool IsRouteConnectFreeStaticLayer(const Point3D& route_start,
                                              const Point3D& route_end);

    /** Whether a historical point is covered by a currently extracted
     * static contour.  Used to distinguish a real local contradiction from a
     * frame in which the area was not observed. */
    static bool IsPointObservedOnCurrentStaticContour(
        const Point3D& point, float tolerance);

    /** Strong evidence that an old corner now lies in the interior of a
     * reliable current static contour segment.  Merely being close to any
     * contour is intentionally insufficient: intersections, current corners,
     * short segments and cropped raster boundaries are excluded. */
    static bool IsPointConfirmedOnCurrentStaticSegmentInterior(
        const Point3D& point, float tolerance, float endpoint_guard,
        PolygonPtr* matched_polygon = nullptr);

    static bool IsPointInsideReliableContourWindow(const Point3D& point);
    static bool DoesSegmentIntersectReliableContourWindow(
        const Point3D& start, const Point3D& end);

    /** Validate a current same-polygon contour edge using an adaptive,
     * robot-clear free-side segment. Static contour geometry is retained when
     * a current dynamic obstacle blocks the validated segment. */
    static EdgeValidationResult ValidateContourFollowEdge(
        const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    static bool IsNavToGoalConnectFreePolygon(const NavNodePtr& node_ptr,
                                              const NavNodePtr& goal_ptr);

    static bool IsPoint3DConnectFreePolygon(const Point3D& p1, const Point3D& p2);

    static bool IsEdgeCollideBoundary(const Point3D& p1, const Point3D& p2);

    static bool IsPointsConnectFreePolygon(const ConnectPair& cedge,
                                           const ConnectPair& bd_cedge,
                                           const HeightPair h_pair,
                                           const bool& is_global_check);

    static bool IsEdgeCollisionFreeInLocalCloud(const ConnectPair& edge,
                                                const HeightPair& edge_height);
    
    static inline void MatchCTNodeWithNavNode(const CTNodePtr& ctnode_ptr, const NavNodePtr& node_ptr) {
        if (ctnode_ptr == NULL || node_ptr == NULL) return;
        ctnode_ptr->is_global_match = true;
        ctnode_ptr->nav_node_id = node_ptr->id;
        node_ptr->ctnode = ctnode_ptr;
        node_ptr->is_contour_match = true;
        // A transient cropped endpoint can become an ordinary static
        // candidate if a later, larger observation reveals a real corner at
        // the same place. Never demote an already confirmed/ordinary node
        // merely because it lies close to this frame's raster boundary.
        if (node_ptr->source == GraphNodeSource::STATIC_CANDIDATE &&
            node_ptr->is_transient_contour_endpoint &&
            !ctnode_ptr->is_boundary_clipped) {
            node_ptr->is_transient_contour_endpoint = false;
            node_ptr->static_seen_count = 0;
        }
        if (ctnode_ptr->source == GraphNodeSource::STATIC_CANDIDATE ||
            ctnode_ptr->source == GraphNodeSource::STATIC_GLOBAL ||
            ctnode_ptr->source == GraphNodeSource::DYNAMIC_LOCAL) {
            node_ptr->observed_in_semantic_snapshot = true;
        }
    }

    static bool ReprojectPointOutsidePolygons(Point3D& point, const float& free_radius);

    static void AddContourToSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    static void DeleteContourFromSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    bool IsPointInVetexAngleRestriction(const CTNodePtr& ctnode, const Point3D end_p);

    void ResetCurrentContour();

private:

    static CTNodeStack polys_ctnodes_;
    static PolygonStack contour_polygons_;
    static PointCloudPtr local_collision_cloud_;
    static PointKdTreePtr local_collision_kdtree_;
    static PointCloudPtr local_static_collision_cloud_;
    static PointKdTreePtr local_static_collision_kdtree_;
    static PointCloudPtr local_dynamic_collision_cloud_;
    static PointKdTreePtr local_dynamic_collision_kdtree_;
    static float contour_projection_min_;
    static float contour_projection_step_;
    static float contour_projection_max_;
    static float contour_boundary_guard_;
    ContourGraphParams ctgraph_params_;
    float ALIGN_ANGLE_COS;
    NavNodePtr odom_node_ptr_ = NULL;
    bool is_robot_inside_poly_ = false;

    // global contour set
    static std::unordered_set<NavEdge, navedge_hash> global_contour_set_;
    static std::unordered_set<NavEdge, navedge_hash> boundary_contour_set_;

    enum class CollisionLayer { COMBINED, STATIC_ONLY, DYNAMIC_ONLY };

    static bool IsPointsConnectFreePolygonForLayer(
        const ConnectPair& cedge, const ConnectPair& bd_cedge,
        const HeightPair h_pair, const bool& is_global_check,
        CollisionLayer layer,
        const PolygonPtr& endpoint_poly1 = PolygonPtr(),
        const PolygonPtr& endpoint_poly2 = PolygonPtr(),
        bool check_raw_cloud = true);

    static bool IsEdgeCollisionFreeInCloud(
        const ConnectPair& edge, const HeightPair& edge_height,
        const PointCloudPtr& cloud, const PointKdTreePtr& kdtree,
        float endpoint_exclusion = -1.0f);

    /** Validate a query edge with exactly one obstacle-corner endpoint.  The
     * corner's direction proposes progressively farther projections, while
     * static/dynamic geometry remains the sole acceptance authority. */
    static EdgeValidationResult ValidateTerminalVisibilityEdgeWithRoute(
        const NavNodePtr& obstacle_node, const NavNodePtr& terminal_node,
        bool obstacle_is_start, bool include_dynamic);

    /** Robot-centre clearance against the persistent static contour layer.
     * This is a segment-distance test, not only a line intersection test. */
    static bool IsRouteClearOfGlobalContours(
        const ConnectPair& route, const HeightPair& height,
        float clearance);

    
    /* static private functions */
    inline void AddCTNodeToGraph(const CTNodePtr& ctnode_ptr) {
        if (ctnode_ptr == NULL && ctnode_ptr->free_direct == NodeFreeDirect::UNKNOW) {
            if (FARUtil::IsDebug) ROS_ERROR_THROTTLE(1.0, "CG: Add ctnode to contour graph fails, ctnode is invaild.");
            return;
        }
        ContourGraph::contour_graph_.push_back(ctnode_ptr);
    }

    inline void AddPolyToContourPolygon(const PolygonPtr& poly_ptr) {
        if (poly_ptr == NULL || poly_ptr->vertices.empty()) {
            if (FARUtil::IsDebug) ROS_ERROR_THROTTLE(1.0, "CG: Add polygon fails, polygon is invaild.");
            return;
        }
        ContourGraph::contour_polygons_.push_back(poly_ptr);
    } 

    inline bool IsActiveEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1->is_active && node_ptr2->is_active) {
            return true;
        }
        return false;
    }
    
    inline void AddConnect(const CTNodePtr& ctnode_ptr1, const CTNodePtr& ctnode_ptr2) {
        if (ctnode_ptr1 != ctnode_ptr2 &&
            !FARUtil::IsTypeInStack(ctnode_ptr2, ctnode_ptr1->connect_nodes) &&
            !FARUtil::IsTypeInStack(ctnode_ptr1, ctnode_ptr2->connect_nodes))
        {
            ctnode_ptr1->connect_nodes.push_back(ctnode_ptr2);
            ctnode_ptr2->connect_nodes.push_back(ctnode_ptr1);
        }
    }

    template <typename NodeType1, typename NodeType2>
    static inline bool IsInMatchHeight(const NodeType1& node_ptr1, const NodeType2& node_ptr2) {
        if (abs(node_ptr1->position.z - node_ptr2->position.z) < FARUtil::kTolerZ) {
            return true;
        }
        return false;
    }

    static inline bool IsEdgeOverlapInHeight(const HeightPair& cur_hpair, HeightPair ref_hpair, const bool is_extend=true) {
        if (is_extend) {
            ref_hpair.minH -= FARUtil::kTolerZ, ref_hpair.maxH += FARUtil::kTolerZ;
        }
        if (cur_hpair.maxH < ref_hpair.minH || cur_hpair.minH > ref_hpair.maxH) {
            return false;
        }
        return true;
    }

    static inline bool IsEdgeInLocalRange(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
        if (node_ptr1->is_near_nodes || node_ptr2->is_near_nodes || FARUtil::IsNodeInLocalRange(node_ptr1) || FARUtil::IsNodeInLocalRange(node_ptr2)) {
            return true;
        }
        return false;
    }

    template <typename NodeType>
    static inline cv::Point2f NodeProjectDir(const NodeType& node) {
        cv::Point2f project_dir(0,0);
        if (node->free_direct != NodeFreeDirect::PILLAR && node->free_direct != NodeFreeDirect::UNKNOW) {
            const Point3D topo_dir = FARUtil::SurfTopoDirect(node->surf_dirs);
            if (node->free_direct == NodeFreeDirect::CONCAVE) {
                project_dir = cv::Point2f(topo_dir.x, topo_dir.y);
            } else {
                project_dir = cv::Point2f(-topo_dir.x, -topo_dir.y);
            }
        }
        return project_dir;
    }

    template <typename NodeType>
    static inline cv::Point2f ProjectNode(const NodeType& node, const float& dist) {
        const cv::Point2f node_cv = cv::Point2f(node->position.x, node->position.y);
        const cv::Point2f dir = NodeProjectDir(node);
        return node_cv + dist * dir;
    }

    static inline void RemoveMatchWithNavNode(const NavNodePtr& node_ptr) {
        if (!node_ptr->is_contour_match) return;
        node_ptr->ctnode->is_global_match = false;
        node_ptr->ctnode->nav_node_id = 0;
        node_ptr->ctnode = NULL;
        node_ptr->is_contour_match = false;
    }

    /**
     * @brief extract necessary ctnodes that are essential for contour construction
     */
    void EnclosePolygonsCheck();

    CTNodePtr FirstMatchedCTNode(const CTNodePtr& ctnode_ptr);

    void UpdateOdomFreePosition(const NavNodePtr& odom_ptr, Point3D& global_free_p);

    static bool IsCTNodesConnectWithinOrder(const CTNodePtr& ctnode1, const CTNodePtr& ctnode2,
                                            CTNodePtr& block_vertex);

    static ConnectPair ReprojectEdge(const NavNodePtr& node1, const NavNodePtr& node2, const float& dist, const bool& is_global_check);

    static ConnectPair ReprojectEdge(const CTNodePtr& node1, const NavNodePtr& node2, const float& dist);

    static bool IsEdgeCollidePoly(const PointStack& poly, const ConnectPair& edge);

    static bool IsEdgeCollideSegment(const PointPair& line, const ConnectPair& edge);

    /** Inflated 2.5D collision test for a small polygon represented as one pillar node. */
    static bool IsPillarConnectBlocked(const PolygonPtr& poly_ptr,
                                       const ConnectPair& edge,
                                       const HeightPair& edge_height);

    static bool IsCTMatchLineFreePolygon(const CTNodePtr& matched_ctnode, const NavNodePtr& matched_navnode, const bool& is_global_check);

    static bool IsValidBoundary(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, bool& is_new);

    inline static bool IsNeedGlobalCheck(const Point3D& p1, const Point3D& p2) {
        if (!FARUtil::IsPointInLocalRange(p1) || !FARUtil::IsPointInLocalRange(p2)) {
            return true;
        }
        return false;
    }

    static inline bool IsOverlapRange(const HeightPair& hpair, const HeightPair& hpairRef) {
        if (hpair.maxH > hpairRef.minH - FARUtil::kTolerZ || hpair.minH < hpairRef.maxH + FARUtil::kTolerZ) {
            return true;
        }
        return false;
    }

    inline void ClearContourGraph() {
        ContourGraph::polys_ctnodes_.clear();
        ContourGraph::contour_graph_.clear();
        ContourGraph::contour_polygons_.clear(); 
    }

    bool IsAPillarPolygon(const PointStack& vertex_points, float& perimeter);

    void CreateCTNode(const Point3D& pos, CTNodePtr& ctnode_ptr, const PolygonPtr& poly_ptr, const bool& is_pillar);

    void CreatePolygon(const PointStack& poly_points, PolygonPtr& poly_ptr,
                       const GraphNodeSource source = GraphNodeSource::UNKNOWN);

    NavNodePtr NearestNavNodeForCTNode(const CTNodePtr& ctnode_ptr, const NodePtrStack& near_nodes);

    void AnalysisConvexityOfCTNode(const CTNodePtr& ctnode_ptr);
    
    /* Analysis CTNode surface angle */
    void AnalysisSurfAngleAndConvexity(const CTNodeStack& contour_graph);


};



#endif
