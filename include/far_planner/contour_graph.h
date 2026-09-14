#ifndef CONTOUR_GRAPH_H
#define CONTOUR_GRAPH_H

#include "utility.h"
#include "far_planner/debug_visualization.h"
#include "far_planner/local_observation_window.h"

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
    // Cross-frame static-corner identity tolerance.  These are deliberately
    // independent of robot_dim: robot footprint controls collision clearance,
    // while identity uncertainty is set by contour-grid/simplification error.
    float static_match_tight_radius = 0.40f;
    float static_match_max_radius = 0.60f;
    // Dynamic identities have a separate motion allowance and therefore do
    // not inherit the legacy robot-dimension-derived kMatchDist either.
    float dynamic_match_max_radius = 1.40f;
    // Exact robot-relative footprint used by the upstream local voxel map.
    // It may be asymmetric and rotates with the robot.
    bool use_local_observation_window = false;
    float local_window_min_x = 0.0f;
    float local_window_max_x = 0.0f;
    float local_window_min_y = 0.0f;
    float local_window_max_y = 0.0f;
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

    void UpdateContourGraph(
        const NavNodePtr& odom_node_ptr,
        const std::vector<std::vector<Point3D>>& static_contours,
        const std::vector<std::vector<Point3D>>& dynamic_contours,
        const std::vector<std::vector<Point3D>>& static_dense_contours,
        const std::vector<std::vector<Point3D>>& dynamic_dense_contours,
        const std::vector<std::vector<std::size_t>>&
            static_simplified_dense_indices,
        const std::vector<std::vector<std::size_t>>&
            dynamic_simplified_dense_indices);

    /** Legacy single-layer setter; treats the supplied cloud as static. */
    static void SetLocalCollisionCloud(const PointCloudPtr& collision_cloud);

    /** Latest cropped static/dynamic layers used for local edge checks. */
    static void SetLocalCollisionCloud(const PointCloudPtr& static_cloud,
                                       const PointCloudPtr& dynamic_cloud);

    /** Current binary robot-centre configuration-space grids.  Occupied
     * pixels are non-zero; both grids share a world-aligned centre and metric
     * resolution.  Once supplied, these grids are the local collision
     * authority and the raw cloud KD trees become compatibility fallbacks. */
    static void SetLocalCollisionGrids(const cv::Mat& static_occupied,
                                       const cv::Mat& dynamic_occupied,
                                       const Point3D& raster_center,
                                       float resolution);

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

    /** Current-frame artificial closing-cap relation between two CLIP
     * vertices.  This is deliberately separate from observed contour
     * adjacency and is never reusable static topology. */
    static bool IsNavNodesConnectFromClipAttempt(
        const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    static bool IsCTNodesConnectFromContour(const CTNodePtr& ctnode1, 
                                            const CTNodePtr& ctnode2);

    static bool IsCTNodesConnectFromClipAttempt(
        const CTNodePtr& ctnode1, const CTNodePtr& ctnode2);

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
    static bool IsRouteConnectFreeDynamicLayer(
        const std::vector<Point3D>& route_points);

    /** Recheck stored free-side geometry against the latest local static
     * cloud and polygons.  This never queries the complete OctoMap. */
    static bool IsRouteConnectFreeStaticLayer(const Point3D& route_start,
                                              const Point3D& route_end);
    static bool IsRouteConnectFreeStaticLayer(
        const std::vector<Point3D>& route_points);

    /** Check robot-centre clearance at one query point against the exact
     * current static/dynamic collision clouds used by visibility edges.  This
     * is intentionally separate from a zero-length edge: zero-length edges
     * are accepted by the segment checker before consulting its KD-tree. */
    static bool IsPointCollisionFreeStaticLayer(const Point3D& point);
    static bool IsPointCollisionFreeDynamicLayer(const Point3D& point);

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
    static bool IsSegmentFullyInsideReliableContourWindow(
        const Point3D& start, const Point3D& end);
    /** Complete local voxel footprint plus the known contour-processing halo
     * and one raster-cell pose/quantization tolerance. This is valid only for
     * current transient CLIP routing, never for persistence/deletion evidence. */
    static bool IsPointInsideCurrentObservationWindow(const Point3D& point);
    static bool IsSegmentFullyInsideCurrentObservationWindow(
        const Point3D& start, const Point3D& end);
    /** Update the robot pose associated with the latest local snapshot. */
    static void SetLocalObservationPose(const Point3D& origin,
                                        const Point3D& forward);
    static bool UsesLocalObservationWindow() {
        return local_observation_window_.enabled;
    }

    static void SetDebugVisualizationEnabled(bool enabled) {
        debug_visualization_enabled_ = enabled;
    }
    static const std::vector<ContourMatchDebugRecord>&
    GetMatchDebugRecords() { return match_debug_records_; }
    static const std::vector<ContourDuplicateDebugRecord>&
    GetDuplicateDebugRecords() { return duplicate_debug_records_; }
    static void RecordHistoricalDuplicate(
        const NavNodePtr& obsolete, const NavNodePtr& keeper,
        float duplicate_radius);

    /** Classify a current same-polygon relation as observed FAR contour
     * topology or a snapshot-local CLIP attempt. Neither is a robot-centre
     * execution segment; start/goal query geometry supplies clearance. */
    static EdgeValidationResult ValidateContourFollowEdge(
        const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2);

    /** Reduced-contour adjacency with unmatched intermediate CT vertices in
     * traversal order (including concave support vertices). */
    static bool GetContourChain(const CTNodePtr& ctnode1,
                                const CTNodePtr& ctnode2,
                                CTNodeStack& chain,
                                bool allow_artificial_cap = false,
                                bool* crossed_artificial_cap = nullptr);

    static bool GetDenseContourChain(const CTNodePtr& ctnode1,
                                     const CTNodePtr& ctnode2,
                                     const CTNodeStack& sparse_chain,
                                     PointStack& dense_chain);

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
        node_ptr->free_space_dir = ctnode_ptr->free_space_dir;
        node_ptr->is_free_space_dir_reliable =
            ctnode_ptr->is_free_space_dir_reliable;
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
    struct LocalCollisionGrid2D {
        cv::Mat occupied;
        Point3D center;
        float resolution = 0.0f;
        float center_row = 0.0f;
        float center_col = 0.0f;

        bool IsValid() const {
            return !occupied.empty() && occupied.type() == CV_8UC1 &&
                   resolution > FARUtil::kEpsilon;
        }
        void Clear() {
            occupied.release();
            resolution = 0.0f;
            center_row = center_col = 0.0f;
        }
    };


    static CTNodeStack polys_ctnodes_;
    static PolygonStack contour_polygons_;
    static PointCloudPtr local_collision_cloud_;
    static PointKdTreePtr local_collision_kdtree_;
    static PointCloudPtr local_static_collision_cloud_;
    static PointKdTreePtr local_static_collision_kdtree_;
    static PointCloudPtr local_dynamic_collision_cloud_;
    static PointKdTreePtr local_dynamic_collision_kdtree_;
    static LocalCollisionGrid2D local_static_collision_grid_;
    static LocalCollisionGrid2D local_dynamic_collision_grid_;
    static LocalCollisionGrid2D local_collision_grid_;
    static float contour_projection_min_;
    static float contour_projection_step_;
    static float contour_projection_max_;
    static float contour_boundary_guard_;
    static LocalObservationWindow2D local_observation_window_;
    inline static bool debug_visualization_enabled_ = false;
    inline static std::vector<ContourMatchDebugRecord> match_debug_records_;
    inline static std::vector<ContourDuplicateDebugRecord>
        duplicate_debug_records_;
    ContourGraphParams ctgraph_params_;
    float ALIGN_ANGLE_COS;
    NavNodePtr odom_node_ptr_ = NULL;
    bool is_robot_inside_poly_ = false;

    // global contour set
    static std::unordered_set<NavEdge, navedge_hash> global_contour_set_;
    static std::unordered_set<NavEdge, navedge_hash> boundary_contour_set_;

    enum class CollisionLayer { COMBINED, STATIC_ONLY, DYNAMIC_ONLY };

    enum class GridCellState { OUTSIDE, FREE, OCCUPIED };

    static GridCellState GridStateAtPoint(
        const Point3D& point, const LocalCollisionGrid2D& grid);

    /** Build a signed direction into observed free robot-centre space.  The
     * CONVEX/CONCAVE convention supplies the initial sign; symmetric samples
     * in the current configuration grid verify it or flip it. */
    static void UpdateFreeSpaceDirection(const CTNodePtr& node_ptr);

    static bool IsPointsConnectFreePolygonForLayer(
        const ConnectPair& cedge, const ConnectPair& bd_cedge,
        const HeightPair h_pair, const bool& is_global_check,
        CollisionLayer layer,
        const PolygonPtr& endpoint_poly1 = PolygonPtr(),
        const PolygonPtr& endpoint_poly2 = PolygonPtr(),
        bool check_raw_cloud = true);

    /** A cropped contour is open only along the local-window clipping cap.
     * Keep every observed contour segment collision-active instead of
     * discarding the complete polygon merely because one vertex is CLIP. */
    static bool IsBoundaryClippedPolygonBlocked(
        const PolygonPtr& polygon, const ConnectPair& edge);

    /** True only for the synthetic cap that closes a contour along one side
     * of the local observation window (including the contour-processing
     * halo). */
    static bool IsArtificialBoundaryClosingSegment(
        const Point3D& first, const Point3D& second);

    static bool IsEdgeCollisionFreeInCloud(
        const ConnectPair& edge, const HeightPair& edge_height,
        const PointCloudPtr& cloud, const PointKdTreePtr& kdtree,
        float endpoint_exclusion = -1.0f);

    static bool IsEdgeCollisionFreeInGrid(
        const ConnectPair& edge, const LocalCollisionGrid2D& grid,
        float endpoint_exclusion);

    static const LocalCollisionGrid2D* CollisionGridForCloud(
        const PointCloudPtr& cloud);

    static bool IsPointCollisionFreeInCloud(
        const Point3D& point, const PointCloudPtr& cloud,
        const PointKdTreePtr& kdtree);

    /** Validate a query edge with exactly one obstacle-corner endpoint.  The
     * corner's direction proposes progressively farther projections, while
     * static/dynamic geometry remains the sole acceptance authority. */
    static EdgeValidationResult ValidateTerminalVisibilityEdgeWithRoute(
        const NavNodePtr& obstacle_node, const NavNodePtr& terminal_node,
        bool obstacle_is_start, bool include_dynamic);

    /** Legacy helper retained for executable projected routes. Same-contour
     * FAR topology edges do not call this validator. */
    static bool ValidateProjectedContourRoute(
        const std::vector<Point3D>& route_points,
        const PolygonPtr& endpoint_polygon, bool static_structure,
        EdgeRejectReason& reason);

    /** Legacy projected-route helper. Same-contour FAR topology is reduced by
     * GetContourChain and does not synthesize a convex-hull execution path. */
    static bool BuildConvexHullContourRoute(
        const CTNodePtr& first, const CTNodePtr& second,
        const PointStack& dense_chain, const PolygonPtr& polygon,
        std::vector<Point3D>& route_points, float& projection_distance,
        EdgeRejectReason& reason);

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
        if (local_observation_window_.enabled) {
            return DoesSegmentIntersectReliableContourWindow(
                node_ptr1->position, node_ptr2->position);
        }
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
        if (local_observation_window_.enabled) {
            return !IsSegmentFullyInsideReliableContourWindow(p1, p2);
        }
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

    void CreatePolygon(
        const PointStack& poly_points, PolygonPtr& poly_ptr,
        const GraphNodeSource source = GraphNodeSource::UNKNOWN,
        const PointStack& dense_points = PointStack(),
        const std::vector<std::size_t>& simplified_dense_indices =
            std::vector<std::size_t>());

    void AnalysisConvexityOfCTNode(const CTNodePtr& ctnode_ptr);
    
    /* Analysis CTNode surface angle */
    void AnalysisSurfAngleAndConvexity(const CTNodeStack& contour_graph);


};



#endif
