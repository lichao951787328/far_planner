#ifndef FAR_PLANNER_H
#define FAR_PLANNER_H

#include <deque>

#include "utility.h"
#include "dynamic_graph.h"
#include "contour_detector.h"
#include "contour_graph.h"
#include "graph_planner.h"
#include "map_handler.h"
#include "planner_visualizer.h"
#include "scan_handler.h"
#include "graph_msger.h"


struct FARMasterParams {
    FARMasterParams() = default;
    float robot_dim; 
    float vehicle_height;
    float voxel_dim;
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
    bool  require_scan_origin;
    bool  protect_static_obstacles;
    std::string world_frame;
};

class FARMaster {
public:
    FARMaster() = default;
    ~FARMaster() = default;

    void Init(); // ROS initialization
    void Loop(); // Main Loop Function

private:
    ros::NodeHandle nh;
    ros::Subscriber reset_graph_sub_, joy_command_sub_, update_command_sub_;
    ros::Subscriber odom_sub_, terrain_sub_, terrain_local_sub_, scan_sub_;
    ros::Subscriber scan_origin_sub_, waypoint_sub_;
    ros::Subscriber read_command_sub_, save_command_sub_; // only use for terminal formatting
    ros::Publisher  goal_pub_, boundary_pub_;
    ros::Publisher  dynamic_obs_pub_, surround_free_debug_, surround_obs_debug_;
    ros::Publisher  scan_grid_debug_, new_PCL_pub_, terrain_height_pub_;
    ros::Publisher  protected_static_debug_pub_;
    ros::Publisher  runtime_pub_, planning_time_pub_, traverse_time_pub_, reach_goal_pub_;

    ros::Timer planning_event_;
    std_msgs::Float32 runtimer_, plan_timer_;

    Point3D robot_pos_, robot_heading_, nav_heading_;

    bool is_reset_env_, is_stop_update_;

    geometry_msgs::PointStamped goal_waypoint_stamped_;

    bool is_cloud_init_, is_scan_init_, is_odom_init_, is_planner_running_;
    bool is_graph_init_;

    PointCloudPtr new_vertices_ptr_;
    PointCloudPtr temp_obs_ptr_;
    PointCloudPtr temp_free_ptr_;
    PointCloudPtr temp_protected_static_ptr_;
    PointCloudPtr scan_grid_ptr_;
    PointCloudPtr terrain_height_ptr_;

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
    std::vector<PointStack> realworld_contour_;

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
    ScanHandlerParams   scan_params_;
    GraphMsgerParams    msger_parmas_;
    
    void LoadROSParams();

    void ResetEnvironmentAndGraph();

    void LocalBoundaryHandler(const std::vector<PointPair>& local_boundary);

    void PlanningCallBack(const ros::TimerEvent& event);
    
    bool PrcocessCloud(const sensor_msgs::PointCloud2ConstPtr& pc,
                       const PointCloudPtr& cloudOut);

    bool ProcessTerrainCloud(const sensor_msgs::PointCloud2ConstPtr& pc,
                             const PointCloudPtr& freeCloudOut,
                             const PointCloudPtr& obsCloudOut,
                             const PointCloudPtr& protectedStaticCloudOut =
                                 PointCloudPtr());

    bool IsStrictlyNewStamp(const ros::Time& stamp,
                            const ros::Time& last_stamp,
                            const char* input_name) const;

    void ResetInputStamps();

    Point3D ProjectNavWaypoint(const NavNodePtr& nav_node_ptr, const NavNodePtr& last_point_ptr);

    /* Callback Functions */
    void OdomCallBack(const nav_msgs::OdometryConstPtr& msg);
    void TerrainCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);
    void TerrainLocalCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);

    ros::Time last_terrain_stamp_;
    ros::Time last_terrain_local_stamp_;
    ros::Time last_scan_stamp_;
    std::deque<geometry_msgs::PointStamped> scan_origin_cache_;
    std::deque<sensor_msgs::PointCloud2ConstPtr> pending_scan_clouds_;

    Point3D ExtendViewpointOnObsCloud(const NavNodePtr& nav_node_ptr, const PointCloudPtr& obsCloudIn, float& free_dist);

    inline void ResetGraphCallBack(const std_msgs::EmptyConstPtr& msg) {
        is_reset_env_ = true;
    }

    inline void JoyCommandCallBack(const sensor_msgs::JoyConstPtr& msg) {
        if (msg->buttons[4] > 0.5) {
            is_reset_env_ = true;
        }
    } 

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
        if (!PreconditionCheck()) return;
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

    void ScanCallBack(const sensor_msgs::PointCloud2ConstPtr& pc);
    void ScanOriginCallBack(const geometry_msgs::PointStampedConstPtr& origin);
    void ProcessScanWithOrigin(const sensor_msgs::PointCloud2ConstPtr& pc,
                               const Point3D& scan_origin);
    bool FindScanOrigin(const ros::Time& stamp, Point3D* scan_origin) const;
    void ProcessPendingScans();
    void WaypointCallBack(const geometry_msgs::PointStamped& route_goal);

    void ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                   const PointCloudPtr& obsCloudIn,
                                   const PointCloudPtr& freeCloudIn, 
                                   const PointCloudPtr& dyObsCloudOut);

    /* define inline functions */
    inline bool PreconditionCheck() {
        if (is_cloud_init_ && is_odom_init_) {
            return true;
        }
        return false;
    }
    inline void ClearTempMemory() {
        new_vertices_ptr_->clear();
        new_nodes_.clear();
        nav_graph_.clear();
        clear_nodes_.clear();
        new_ctnodes_.clear();
        near_nav_graph_.clear();
        realworld_contour_.clear();
    }

    inline void ResetInternalValues() {
        odom_node_ptr_ = NULL;
        is_planner_running_ = false;  
        is_graph_init_      = false; 
        ClearTempMemory();
    }
};

#endif
