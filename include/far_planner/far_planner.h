#ifndef FAR_PLANNER_H
#define FAR_PLANNER_H

#include "utility.h"
#include "dynamic_graph.h"
#include "contour_detector.h"
#include "contour_graph.h"
#include "graph_planner.h"
#include "map_handler.h"
#include "local_voxel_policy.h"
#include "planner_visualizer.h"
#include "scan_handler.h"
#include "graph_msger.h"
#include "waypoint_projection_policy.h"

#include <cstdint>
#include <fstream>

struct FARMasterParams {
    FARMasterParams() = default;
    float robot_dim; 
    float vehicle_height;
    float contour_grid_resolution;
    float sensor_range;
    float terrain_range;
    float local_planner_range;
    float main_run_freq;
    float viz_ratio;
    bool  is_multi_layer;
    bool  is_viewpoint_extend;
    bool  is_visual_opencv;
    bool  is_static_env;
    bool  is_pub_boundary;
    bool  is_debug_output;
    bool  is_attempt_autoswitch;
    float odom_timeout;
    float semantic_map_timeout;
    bool  use_local_voxel_map;
    float local_voxel_timeout;
    float odom_connection_update_distance;
    bool enable_goal_recording;
    std::string goal_record_file;
    std::string world_frame;
    std::string semantic_map_topic;
    std::string local_voxel_topic;
};

class FARMaster {
public:
    FARMaster() : private_nh_("~") {}
    ~FARMaster() = default;

    void Init(); // ROS initialization
    void Loop(); // Main Loop Function

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh_;
    ros::Subscriber reset_graph_sub_, update_command_sub_;
    ros::Subscriber odom_sub_, waypoint_sub_;
    ros::Subscriber semantic_map_sub_, local_voxel_sub_;
    ros::Subscriber read_command_sub_, save_command_sub_; // only use for terminal formatting
    ros::Publisher  goal_pub_, boundary_pub_;
    ros::Publisher  dynamic_obs_pub_, surround_obs_debug_;
    ros::Publisher  graph_static_obs_pub_;
    ros::Publisher  global_confirmed_static_obs_pub_;
    ros::Publisher  local_planner_static_obs_pub_;
    ros::Publisher  local_planner_dynamic_obs_pub_;
    ros::Publisher  surround_obs_before_dyremove_debug_, surround_obs_after_dyremove_debug_;
    ros::Publisher  scan_grid_debug_, new_PCL_pub_, terrain_height_pub_;
    ros::Publisher  runtime_pub_, planning_time_pub_, traverse_time_pub_, reach_goal_pub_;
    ros::Publisher  semantic_snapshot_time_pub_, semantic_update_time_pub_;
    ros::Publisher  semantic_callback_time_pub_, main_loop_time_pub_;

    ros::Timer planning_event_;
    std_msgs::Float32 runtimer_, plan_timer_;

    Point3D robot_pos_, robot_heading_, nav_heading_;

    bool is_reset_env_, is_stop_update_;

    geometry_msgs::PointStamped goal_waypoint_stamped_;

    bool is_cloud_init_, is_scan_init_, is_odom_init_, is_planner_running_;
    bool is_graph_init_;
    // Keep the latest RViz/user goal received during graph startup.  The
    // original message is applied once the first usable graph exists instead
    // of being silently discarded.
    bool has_pending_route_goal_ = false;
    geometry_msgs::PointStamped pending_route_goal_;
    // Keep the exact command independently from the terrain-adjusted Graph
    // goal so /far_global_path can end at the point the operator requested.
    bool has_commanded_goal_ = false;
    Point3D commanded_goal_;
    // A current obstacle snapshot has changed the local contour/terrain
    // inputs but its visibility-graph update has not completed yet.
    bool semantic_graph_dirty_ = false;
    // The ROS timer only records demand.  Planning, freshness checks and stop
    // publication are serialized by Loop() after spinOnce() has drained the
    // subscription callbacks for that iteration.
    bool planning_requested_ = false;
    // Odom/start visibility edges are a query layer, not obstacle topology.
    // They can therefore be refreshed without rebuilding contours and all
    // obstacle-obstacle edges.
    bool odom_connections_dirty_ = true;
    bool has_odom_connection_position_ = false;
    Point3D last_odom_connection_position_;
    bool timeout_stop_active_ = false;
    ros::WallTime last_odom_receipt_;
    ros::WallTime last_semantic_map_receipt_;
    ros::WallTime last_local_voxel_receipt_;
    ros::Time last_odom_stamp_;
    ros::Time last_semantic_map_stamp_;
    ros::Time last_local_voxel_stamp_;
    bool has_local_voxel_snapshot_ = false;
    std::ofstream goal_record_stream_;
    std::uint64_t goal_record_sequence_ = 0;
    std::uint64_t goal_record_session_sequence_ = 0;
    std::string goal_record_session_id_;

    PointCloudPtr new_vertices_ptr_;
    PointCloudPtr temp_obs_ptr_;
    PointCloudPtr temp_free_ptr_;
    PointCloudPtr temp_cloud_ptr_;
    PointCloudPtr scan_grid_ptr_;
    PointCloudPtr terrain_height_ptr_;
    PointCloudPtr dyremove_before_obs_ptr_;
    PointCloudPtr collision_obs_ptr_;
    PointCloudPtr current_static_obs_ptr_;
    PointCloudPtr persistent_static_obs_ptr_;
    PointCloudPtr graph_static_collision_obs_ptr_;
    PointCloudPtr effective_dynamic_obs_ptr_;
    PointCloudPtr dynamic_added_ptr_;
    PointCloudPtr dynamic_removed_ptr_;
    PointCloudPtr local_planner_static_obs_ptr_;
    PointCloudPtr local_planner_dynamic_obs_ptr_;

    /* veiwpoint extension clouds */
    PointCloudPtr  viewpoint_around_ptr_;
    PointKdTreePtr kdtree_viewpoint_obs_cloud_;

    NavNodePtr odom_node_ptr_ = NULL;
    NavNodePtr nav_node_ptr_  = NULL;
    NodePtrStack new_nodes_;
    NodePtrStack nav_graph_;
    NodePtrStack near_nav_graph_;
    NodePtrStack clear_nodes_;

    CTNodeStack new_ctnodes_;
    std::vector<PointStack> static_contours_;
    std::vector<PointStack> dynamic_contours_;

    tf::TransformListener* tf_listener_;

    /* module objects */
    ContourDetector contour_detector_;
    DynamicGraph graph_manager_;
    DPVisualizer planner_viz_;
    GraphPlanner graph_planner_;
    ContourGraph contour_graph_;
    MapHandler map_handler_;
    ScanHandler scan_handler_;
    GraphMsger graph_msger_;

    /* ROS Params */
    FARMasterParams     master_params_;
    ContourDetectParams cdetect_params_;
    DynamicGraphParams  graph_params_;
    GraphPlannerParams  gp_params_;
    ContourGraphParams  cg_params_;
    MapHandlerParams    map_params_;
    LocalVoxelPolicyParams local_voxel_policy_params_;
    ScanHandlerParams   scan_params_;
    GraphMsgerParams    msger_parmas_;
    
    void LoadROSParams();

    void ResetEnvironmentAndGraph();

    void LocalBoundaryHandler(const std::vector<PointPair>& local_boundary);

    void PlanningCallBack(const ros::TimerEvent& event);
    void ExecutePlanningCycle();
    void InitializeGoalRecorder();
    void RecordGoalSelection(const geometry_msgs::PointStamped& route_goal,
                             const Point3D& world_goal);
    void ApplyWorldGoal(const Point3D& goal);
    
    void PrcocessCloud(const sensor_msgs::PointCloud2ConstPtr& pc,
                       const PointCloudPtr& cloudOut);

    Point3D ProjectNavWaypoint(const NavNodePtr& nav_node_ptr, const NavNodePtr& last_point_ptr);

    // Restore FAR's free-side contour waypoint extension while validating
    // every candidate from near to far against the current semantic layers.
    Point3D ProjectContourWaypointProgressively(
        const NavNodePtr& nav_node_ptr, const Point3D& safe_fallback);

    EdgeRejectReason ValidateProjectedWaypoint(
        const Point3D& candidate) const;

    /* Callback Functions */
    void OdomCallBack(const nav_msgs::OdometryConstPtr& msg);
    void SemanticMapCallBack(const octomap_msgs::OctomapConstPtr& msg);
    void LocalVoxelMapCallBack(const sensor_msgs::PointCloud2ConstPtr& msg);
    void UpdatePlannerCloudsFromSemanticMap();
    void UpdatePlannerCloudsFromLocalVoxelMap();
    void UpdatePlannerCloudsFromCurrentLayers();
    bool GlobalStaticEvidenceIsFresh() const;

    bool PreconditionCheck();
    bool DataIsFresh(std::string* reason = nullptr) const;
    void PublishStopCommand(const std::string& reason);

    Point3D ExtendViewpointOnObsCloud(const NavNodePtr& nav_node_ptr, const PointCloudPtr& obsCloudIn, float& free_dist);

    inline void ResetGraphCallBack(const std_msgs::EmptyConstPtr& msg) {
        is_reset_env_ = true;
    }

    // inline void JoyCommandCallBack(const sensor_msgs::JoyConstPtr& msg) {
    //     if (msg->buttons[4] > 0.5) {
    //         is_reset_env_ = true;
    //     }
    // } 

    inline void UpdateCommandCallBack(const std_msgs::Bool& msg) {
        if (is_stop_update_ && msg.data) {
            if (FARUtil::IsDebug) ROS_WARN("FARMaster: Resume visibility graph update.");
            is_stop_update_ = !msg.data;
        }
        if (!is_stop_update_ && !msg.data) {
            if (FARUtil::IsDebug) ROS_WARN("FARMaster: Stop visibility graph update.");
            is_stop_update_ = !msg.data;
        }   
    }

    inline void FakeTerminalInit() {
        std::cout<<std::endl;
        if (master_params_.is_static_env) {
            std::cout<<"\033[1;33m **************** STATIC ENV PLANNING **************** \033[0m\n"<<std::endl;
        } else {
            std::cout<< "\033[1;33m **************** DYNAMIC ENV PLANNING **************** \033[0m\n" << std::endl;
        }
        std::cout<<"\n"<<std::endl;
        // This callback is display-only. Freshness and stop ownership remain
        // exclusively in Loop() after spinOnce().
        if (!is_cloud_init_ || !is_odom_init_) return;
        printf("\033[A"), printf("\033[A"), printf("\033[2K");
        if (is_graph_init_) {
            std::cout<< "\033[1;32m V-Graph Initialized \033[0m\n" << std::endl;
            std::cout<<std::endl<<std::endl;
        } else {
            std::cout<< "\033[1;31m V-Graph Resetting...\033[0m\n" << std::endl;
        }
    }

    inline void ReadFileCommand(const std_msgs::StringConstPtr& msg) {
        if (!FARUtil::IsDebug) { // Terminal Output
            printf("\033[2J"), printf("\033[0;0H"); // cleanup screen
            FakeTerminalInit();
        }
    }

    inline void SaveFileCommand(const std_msgs::StringConstPtr& msg) {
        if (!FARUtil::IsDebug) { // Terminal Output
            printf("\033[2J"), printf("\033[0;0H"); // cleanup screen
            FakeTerminalInit();
        }
    }

    // void ScanCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);
    void WaypointCallBack(const geometry_msgs::PointStamped& route_goal);

    void ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                   const PointCloudPtr& obsCloudIn,
                                   const PointCloudPtr& freeCloudIn, 
                                   const PointCloudPtr& dyObsCloudOut);

    /* define inline functions */
    inline void ClearTempMemory() {
        new_vertices_ptr_->clear();
        new_nodes_.clear();
        nav_graph_.clear();
        clear_nodes_.clear();
        new_ctnodes_.clear();
        near_nav_graph_.clear();
        static_contours_.clear();
        dynamic_contours_.clear();
    }

    inline void ResetInternalValues() {
        odom_node_ptr_ = NULL;
        is_planner_running_ = false;  
        is_graph_init_      = false; 
        semantic_graph_dirty_ = false;
        planning_requested_ = false;
        odom_connections_dirty_ = true;
        has_odom_connection_position_ = false;
        ClearTempMemory();
    }
};

#endif
