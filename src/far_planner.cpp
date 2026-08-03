/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



#include "far_planner/far_planner.h"

/***************************************************************************************/

void FARMaster::Init() {
  /* initialize subscriber and publisher */
  reset_graph_sub_    = nh.subscribe("/reset_visibility_graph", 5, &FARMaster::ResetGraphCallBack, this);
  odom_sub_           = nh.subscribe("/odom_world", 5, &FARMaster::OdomCallBack, this);
  // terrain_map_ext 订阅
  terrain_sub_        = nh.subscribe("/terrain_cloud", 1, &FARMaster::TerrainCallBack, this);
  // registered_scan
  scan_sub_           = nh.subscribe("/scan_cloud", 5, &FARMaster::ScanCallBack, this);
  // 这个是什么级别的终点。用户全局指定的终点，不是规划器自己的目标点。
  waypoint_sub_       = nh.subscribe("/goal_point", 1, &FARMaster::WaypointCallBack, this);
  // terrain_map 订阅
  terrain_local_sub_  = nh.subscribe("/terrain_local_cloud", 1, &FARMaster::TerrainLocalCallBack, this);
  joy_command_sub_    = nh.subscribe("/joy", 5, &FARMaster::JoyCommandCallBack, this);
  update_command_sub_ = nh.subscribe("/update_visibility_graph", 5, &FARMaster::UpdateCommandCallBack, this);
  // 发给下游规划器的目标点，也就是局部终点
  goal_pub_           = nh.advertise<geometry_msgs::PointStamped>("/way_point",5);
  boundary_pub_       = nh.advertise<geometry_msgs::PolygonStamped>("/navigation_boundary",5);
  // Timers
  runtime_pub_        = nh.advertise<std_msgs::Float32>("/runtime",1);
  planning_time_pub_  = nh.advertise<std_msgs::Float32>("/planning_time",1);
  traverse_time_pub_  = nh.advertise<std_msgs::Float32>("/far_traverse_time", 5);
  // planning status publisher
  reach_goal_pub_     = nh.advertise<std_msgs::Bool>("/far_reach_goal_status", 5);
  // Terminal formatting subscriber
  read_command_sub_   = nh.subscribe("/read_file_dir", 1, &FARMaster::ReadFileCommand, this);
  save_command_sub_   = nh.subscribe("/save_file_dir", 1, &FARMaster::SaveFileCommand, this);
  // DEBUG Publisher
  dynamic_obs_pub_     = nh.advertise<sensor_msgs::PointCloud2>("/FAR_dynamic_obs_debug",1);
  surround_free_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_free_debug",1);
  surround_obs_debug_  = nh.advertise<sensor_msgs::PointCloud2>("/FAR_obs_debug",1);
  surround_obs_before_dyremove_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_obs_before_dyremove_debug",1);
  surround_obs_after_dyremove_debug_  = nh.advertise<sensor_msgs::PointCloud2>("/FAR_obs_after_dyremove_debug",1);
  scan_grid_debug_     = nh.advertise<sensor_msgs::PointCloud2>("/FAR_scanGrid_debug",1);
  new_PCL_pub_         = nh.advertise<sensor_msgs::PointCloud2>("/FAR_new_debug",1);
  terrain_height_pub_  = nh.advertise<sensor_msgs::PointCloud2>("/FAR_terrain_height_debug",1);

  this->LoadROSParams();

  /*init path generation thred callback*/
  // 在这设置了全局地图规划器的循环频率，默认是5hz，但是调用默认参数后是2.5hz。
  const float duration_time = 0.99f / master_params_.main_run_freq;
  planning_event_ = nh.createTimer(ros::Duration(duration_time), &FARMaster::PlanningCallBack, this);

  /* init Dynamic Planner Processing Objects */
  contour_detector_.Init(cdetect_params_);
  graph_manager_.Init(nh, graph_params_);
  graph_planner_.Init(nh, gp_params_);
  contour_graph_.Init(cg_params_);
  planner_viz_.Init(nh);
  map_handler_.Init(map_params_);
  scan_handler_.Init(scan_params_);
  graph_msger_.Init(nh, msger_parmas_);

  /* init internal params */
  odom_node_ptr_      = NULL;
  is_cloud_init_      = false;
  is_odom_init_       = false;
  is_scan_init_       = false;
  is_planner_running_ = false;
  is_graph_init_      = false;
  is_reset_env_       = false;
  is_stop_update_     = false;

  // allocate memory to pointers
  new_vertices_ptr_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_obs_ptr_         = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_free_ptr_        = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  temp_cloud_ptr_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  scan_grid_ptr_        = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  local_terrain_ptr_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  terrain_height_ptr_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  dyremove_before_obs_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  viewpoint_around_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
  kdtree_viewpoint_obs_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());

  // set kdtree sorted value
  FARUtil::kdtree_new_cloud_->setSortedResults(false);
  FARUtil::kdtree_filter_cloud_->setSortedResults(false);
  kdtree_viewpoint_obs_cloud_->setSortedResults(false);

  // init global utility cloud
  FARUtil::stack_new_cloud_->clear();
  FARUtil::stack_dyobs_cloud_->clear();

  // init TF listener
  tf_listener_ = new tf::TransformListener();

  // clear temp vectors and memory
  this->ClearTempMemory();
  FARUtil::robot_pos = Point3D(0,0,0);
  FARUtil::free_odom_p = Point3D(0,0,0);

  robot_pos_   = Point3D(0,0,0);
  nav_heading_ = Point3D(0,0,0);
  goal_waypoint_stamped_.header.frame_id = master_params_.world_frame;
  printf("\033[2J"), printf("\033[0;0H"); // cleanup screen
  std::cout<<std::endl;
  if (master_params_.is_static_env) {
    std::cout<<"\033[1;33m **************** STATIC ENV PLANNING **************** \033[0m\n"<<std::endl;
  } else {
    std::cout<< "\033[1;33m **************** DYNAMIC ENV PLANNING **************** \033[0m\n" << std::endl;
  }
  std::cout<<"\n"<<std::endl;
}

void FARMaster::ResetEnvironmentAndGraph() {
  this->ResetInternalValues();
  if (!FARUtil::IsDebug) { // Terminal Output
    printf("\033[A"), printf("\033[A"), printf("\033[2K");
    std::cout<< "\033[1;31m V-Graph Resetting...\033[0m\n" << std::endl;
  }
  graph_manager_.ResetCurrentGraph();
  map_handler_.ResetGripMapCloud();
  graph_planner_.ResetPlannerInternalValues();
  contour_graph_.ResetCurrentContour();
  /* Reset clouds */
  FARUtil::surround_obs_cloud_->clear();
  FARUtil::surround_free_cloud_->clear();
  FARUtil::stack_new_cloud_->clear();
  FARUtil::stack_dyobs_cloud_->clear();
  FARUtil::cur_new_cloud_->clear();
  FARUtil::cur_dyobs_cloud_->clear();
  /* Stop the robot if it is moving */
  goal_waypoint_stamped_.header.stamp = ros::Time::now();
  goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
  goal_pub_.publish(goal_waypoint_stamped_);
  NodePtrStack empty_path;
  planner_viz_.VizPath(empty_path);
}

void FARMaster::Loop() {
  ros::Rate loop_rate(master_params_.main_run_freq);
  while (ros::ok()) {
    if (is_reset_env_) {
      this->ResetEnvironmentAndGraph(); 
      is_reset_env_ = false;
      if (FARUtil::IsDebug) ROS_WARN("****************** Graph and Env Reset ******************");
      loop_rate.sleep(); // skip this iteration
      continue;
    }
    /* Process callback functions */
    ros::spinOnce(); 
    if (!this->PreconditionCheck()) {
      loop_rate.sleep();
      continue;
    }
    /* add main process after this line */
    graph_manager_.UpdateRobotPosition(robot_pos_);
    odom_node_ptr_ = graph_manager_.GetOdomNode();
    if (odom_node_ptr_ == NULL) {
      ROS_WARN("FAR: Waiting for Odometry...");
      loop_rate.sleep();
      continue;
    }
    /* Extract Vertices and new nodes */
    FARUtil::Timer.start_time("Total V-Graph Update");
    // 在做“从周围障碍点云里提取轮廓”的核心步骤，作用是把稠密点云转换成后续可建图的几何边界。
    // 不是“检测整个地图的轮廓”，而是检测机器人当前周围局部障碍云的轮廓。
    // 以当前 odom 节点为中心，把机器人周围的局部障碍点云投影成图像，然后提取这片局部环境的障碍轮廓。
    contour_detector_.BuildTerrainImgAndExtractContour(odom_node_ptr_, FARUtil::surround_obs_cloud_, realworld_contour_);
    // 把刚提取出的“真实世界轮廓”写进轮廓图模块，完成轮廓图的本帧更新。
    contour_graph_.UpdateContourGraph(odom_node_ptr_, realworld_contour_);
    if (is_graph_init_) {
      if (!FARUtil::IsDebug) printf("\033[2K");
      std::cout<<"    "<<"Local V-Graph Updated. Number of local vertices: "<<ContourGraph::contour_graph_.size()<<std::endl;
    }
    /* Adjust heights with terrain */
    // 你看到的这两行就是在把“平面上的图结构”贴到“可通行地形高程”上，说明这个规划不是纯 2D，而是带高度约束的 2.5D/3D 处理。
    // 为什么这里会有高程：
    // 上游会把 free 点云写入地图，并更新地形高度网格
    // 在 TerrainCallBack 里先更新 free/obs，再调用 UpdateTerrainHeightGrid(...)。这一步会从自由空间点估计各网格高度。
    // MapHandler 内部维护了 terrain_height_grid_
    // 这就是局部高程图容器（按网格存高度统计），后续可查询某点附近地面高度。
    // AdjustCTNodeHeight(...) 和 AdjustNodesHeight(...)
    // 分别给轮廓图节点、导航图节点校正 z 值，让节点高度与可通行地面一致，避免路径“悬空”或“钻地”。
    // 所以你问“当前可通行地形是高程图吗”：
    // 是，内部确实有一个可通行性相关的高程网格（terrain height grid）。
    // 但它不是那种全局离线 DEM，而是由当前 free 点云在线更新的局部/滚动高程图。
    // 同时系统还保留 obs/free 点云栅格，因此它是“点云 + 高程网格”联合，而不只是单一高程图。

    // 修正contour_graph_和nav_graph_的高度，使它们与地形高度一致，避免出现悬空或钻地的情况。
    // CTNode：轮廓几何节点, 本质上是“障碍轮廓上的一个角点/轮廓点”，服务于轮廓图构建和多边形几何关系。
    // nav_graph_：导航图节点,真正参与图搜索和路径规划的节点
    map_handler_.AdjustCTNodeHeight(ContourGraph::contour_graph_);
    map_handler_.AdjustNodesHeight(nav_graph_);
    // Truncate for local range nodes
    // “从全局图里筛出机器人附近节点集合”，给后续局部匹配和增量更新用。
    // 根据当前机器人位置，更新 DynamicGraph 内部的“近邻节点缓存”（哪些全局导航节点算近、算扩展近邻）。
    // 先筛出机器人当前局部真正相关的图节点，再把这个局部子集交给后面的轮廓匹配和增量更新流程使用。换句话说，这一步是在做“全局图的局部裁剪与近邻维护”，让后续的 contour matching 和 graph update 只处理机器人周围有效节点。
    graph_manager_.UpdateGlobalNearNodes();
    // 把刚更新好的“扩展局部近邻节点集合”取出来，赋给 near_nav_graph_。
    // 只是把前面在 DynamicGraph::UpdateGlobalNearNodes 里筛出来的“扩展局部近邻节点”拿出来给外部用。
    near_nav_graph_ = graph_manager_.GetExtendLocalNode();
    // Match near nav nodes with contour
    // 先缩小到局部相关节点，再做轮廓与导航图匹配，减少无关计算、提升稳定性和速度。
    // MatchContourWithNavGraph() 是把当前帧提取出来的轮廓点，和全局导航图里的近邻节点做匹配的函数。它的输入是两组节点：global_nodes 是全局导航图，near_nodes 是前面筛出来的局部近邻节点；输出是 new_convex_vertices，也就是这帧里“没被现有导航节点匹配上、需要当成新顶点处理”的轮廓点。
    // 它的作用就是“先拿局部轮廓去对齐已有图节点，剩下的才当作新顶点候选”。这一步是增量建图的关键：不是直接把所有轮廓都变成新节点，而是先复用已有导航节点，减少重复顶点和错误建图。
    contour_graph_.MatchContourWithNavGraph(nav_graph_, near_nav_graph_, new_ctnodes_);
    if (master_params_.is_visual_opencv) {
      FARUtil::ConvertCTNodeStackToPCL(new_ctnodes_, new_vertices_ptr_);
      cv::Mat cloud_img = contour_detector_.GetCloudImgMat();
      contour_detector_.ShowCornerImage(cloud_img, new_vertices_ptr_);
    }
    /* update planner graph */
    new_nodes_.clear();
    // 把本帧“候选轮廓顶点”转成“可加入全局图的新导航节点”。
    // new_ctnodes_ 就是上一阶段输出的“新增候选轮廓点”，来源是 contour_graph.cpp:65 的第三个输出参数。这个函数会把未匹配且满足几何条件的 CTNode 放进 new_convex_vertices，实际在主流程里就是 new_ctnodes_。
    if (!is_stop_update_ && graph_manager_.ExtractGraphNodes(new_ctnodes_)) {
      new_nodes_ = graph_manager_.GetNewNodes();
    }
    if (is_graph_init_) {
      if (!FARUtil::IsDebug) printf("\033[2K");
      std::cout<<"    "<< "Number of new vertices adding to global V-Graph: "<< new_nodes_.size()<<std::endl;
    }
    /* Graph Updating */
    // 真正把“本帧增量信息”合并进全局可视图的核心更新器
    graph_manager_.UpdateNavGraph(new_nodes_, is_stop_update_, clear_nodes_);
    runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", is_graph_init_) / 1000.f; // Unit: second
    // runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", is_graph_init_); // Unit: ms
    runtime_pub_.publish(runtimer_);
    /* Update v-graph in other modules */
    // 在调用 UpdateNavGraph 之后，立刻拿当前最新图，后面会连续喂给规划器、消息发布和可视化（同一帧一致性）。
    nav_graph_ = graph_manager_.GetNavGraph();
    if (is_graph_init_) {
      if (!FARUtil::IsDebug) printf("\033[2K");
      std::cout<<"    "<<"Global V-Graph Updated. Number of global vertices: "<<nav_graph_.size()<<std::endl;
    }
    // 从当前全局图连接关系里重新提取轮廓集合，更新全局/局部边界相关缓存
    contour_graph_.ExtractGlobalContours();      // Global Polygon Update
    // 把最新 nav_graph_ 交给路径规划器，让后续 PathToGoal、可达性判断都基于刚更新后的图。
    graph_planner_.UpdaetVGraph(nav_graph_);     // Graph Planner Update
    // 把同一份最新全局图同步到消息发布模块，用于对外发送图结构（多机通信/可视化消费者）。
    graph_msger_.UpdateGlobalGraph(nav_graph_);  // Graph Messager Update

    /* Publish local boundary to lower level local planner */
    // 局部地图中和当前运动相关的一些边界点对，供下游局部规划器使用。比如在局部规划器里做“局部可通行性检查”时，可以用这些边界点对来判断是否碰到障碍。
    this->LocalBoundaryHandler(ContourGraph::local_boundary_);

    /* Viz Navigation Graph */
    const NavNodePtr last_internav_ptr = graph_manager_.GetLastInterNavNode();
    if (last_internav_ptr != NULL) {
      planner_viz_.VizPoint3D(last_internav_ptr->position, "last_nav_node", VizColor::MAGNA, 1.0);
    }
    planner_viz_.VizNodes(clear_nodes_, "clear_nodes", VizColor::ORANGE);
    planner_viz_.VizNodes(graph_manager_.GetOutContourNodes(), "out_contour", VizColor::YELLOW);
    planner_viz_.VizPoint3D(FARUtil::free_odom_p, "free_odom_position", VizColor::ORANGE, 1.0);
    planner_viz_.VizGraph(nav_graph_);
    planner_viz_.VizContourGraph(ContourGraph::contour_graph_);
    planner_viz_.VizGlobalPolygons(ContourGraph::global_contour_, ContourGraph::unmatched_contour_);

    if (is_graph_init_) { 
      if (FARUtil::IsDebug) {
        std::cout<<" ========================================================== "<<std::endl;
      } else { // cleanup outputs in terminal
        for (int i = 0; i < 6; i++) {
          printf("\033[A");
        }
      }
    }

    if (!is_graph_init_ && !nav_graph_.empty()) {
      is_graph_init_ = true;
      printf("\033[A"), printf("\033[A"), printf("\033[2K");
      std::cout<< "\033[1;32m V-Graph Initialized \033[0m\n" << std::endl;
    }
    loop_rate.sleep();
  }
}

// 规划器的核心循环函数，定时器触发，频率由 main_run_freq 决定。
void FARMaster::PlanningCallBack(const ros::TimerEvent& event) {
  if (!is_graph_init_) return;
  const NavNodePtr goal_ptr = graph_planner_.GetGoalNodePtr();
  if (goal_ptr == NULL) {
    /* Graph Traversablity Update */
    if (!FARUtil::IsDebug) printf("\033[2K");
    std::cout<<"    "<<"Adding Goal to V-Graph "<<"Time: "<<0.f<<"ms"<<std::endl;
    // 图可达性更新
    graph_planner_.UpdateGraphTraverability(odom_node_ptr_, NULL);
    if (!FARUtil::IsDebug) printf("\033[2K");
    std::cout<<"    "<<"Path Search "<<"Time: "<<0.f<<"ms"<<std::endl;
  } else { 
    // 会先在目标附近拿障碍/自由点云，更新目标附近可通行信息，再重新评估 goal 位置是否合理。
    // Update goal postion with nearby terrain cloud
    const Point3D ori_p = graph_planner_.GetOriginNodePos(true);
    PointCloudPtr goal_obs(new pcl::PointCloud<PCLPoint>());
    PointCloudPtr goal_free(new pcl::PointCloud<PCLPoint>());
    map_handler_.GetCloudOfPoint(ori_p, goal_obs, CloudType::OBS_CLOUD, true);
    map_handler_.GetCloudOfPoint(ori_p, goal_free, CloudType::FREE_CLOUD, true);
    graph_planner_.UpdateFreeTerrainGrid(ori_p, goal_obs, goal_free);
    graph_planner_.ReEvaluateGoalPosition(goal_ptr, !master_params_.is_multi_layer);

    // Adding goal into v-graph
    FARUtil::Timer.start_time("Adding Goal to V-Graph");
    graph_planner_.UpdateGoalNavNodeConnects(goal_ptr); 
    graph_planner_.UpdaetVGraph(graph_manager_.GetNavGraph());
    if (!FARUtil::IsDebug) printf("\033[2K");
    FARUtil::Timer.end_time("Adding Goal to V-Graph");

    // Update v-graph traversibility 
    FARUtil::Timer.start_time("Path Search");
    graph_planner_.UpdateGraphTraverability(odom_node_ptr_, goal_ptr);

    // Construct path to gaol and publish waypoint
    NodePtrStack global_path;
    Point3D current_free_goal;
    NavNodePtr last_nav_ptr = nav_node_ptr_;
    bool is_planning_fails = false;
    goal_waypoint_stamped_.header.stamp = ros::Time::now();
    bool is_current_free_nav = false;
    bool is_reach_goal = false;
    // 根据当前可见图和目标节点搜索出来的一条“图路径”（节点序列）global_path
    if (graph_planner_.PathToGoal(goal_ptr, global_path, nav_node_ptr_, current_free_goal, is_planning_fails, is_reach_goal, is_current_free_nav) && nav_node_ptr_ != NULL) {
      Point3D waypoint = nav_node_ptr_->position;
      if (nav_node_ptr_ != goal_ptr) {
        waypoint = this->ProjectNavWaypoint(nav_node_ptr_, last_nav_ptr);
      } else if (master_params_.is_viewpoint_extend) {
        planner_viz_.VizViewpointExtend(goal_ptr, goal_ptr->position);
      }
      goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(waypoint);
      // 发布waypoint
      goal_pub_.publish(goal_waypoint_stamped_);
      is_planner_running_ = true;
      planner_viz_.VizPoint3D(waypoint, "waypoint", VizColor::MAGNA, 1.5);
      planner_viz_.VizPoint3D(current_free_goal, "free_goal", VizColor::GREEN, 1.5);
      planner_viz_.VizPath(global_path, is_current_free_nav);
    } else if (is_planner_running_) {
      // stop robot
      global_path.clear();
      planner_viz_.VizPath(global_path);
      is_planner_running_ = false;
      nav_heading_ = Point3D(0,0,0);
      if (is_planning_fails) { // stops the robot
        goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
        goal_pub_.publish(goal_waypoint_stamped_);
      }
    }
    if (!FARUtil::IsDebug) printf("\033[2K");

    // publish planner status and timers
    std_msgs::Bool reach_goal_msg;
    reach_goal_msg.data = is_reach_goal;
    reach_goal_pub_.publish(reach_goal_msg);
    std_msgs::Float32 traverse_timer;
    traverse_timer.data = FARUtil::Timer.record_time("Overall_executing");
    traverse_time_pub_.publish(traverse_timer);
    if (is_reach_goal) {
      FARUtil::Timer.end_time("Overall_executing", false);
    }
    plan_timer_.data = FARUtil::Timer.end_time("Path Search");
    planning_time_pub_.publish(plan_timer_);
  }
}

// 把已经算好的局部边界线段，筛一遍并打包成 ROS 消息，发布给下游局部规划器/可视化使用。
void FARMaster::LocalBoundaryHandler(const std::vector<PointPair>& local_boundary) {
  if (!master_params_.is_pub_boundary || local_boundary.empty()) return;
  geometry_msgs::PolygonStamped boundary_poly;
  boundary_poly.header.frame_id = master_params_.world_frame;
  boundary_poly.header.stamp = ros::Time::now();
  float index_z = robot_pos_.z;
  std::vector<PointPair> sorted_boundary;
  // 按距离筛选边界
  // 遍历每条边界线段 edge，计算机器人到线段的 2D 距离；
  // 超过 local_planner_range 的边界段丢弃，只保留“当前局部相关”的。
  for (const auto& edge : local_boundary) {
    if (FARUtil::DistanceToLineSeg2D(robot_pos_, edge) > master_params_.local_planner_range) continue;
    sorted_boundary.push_back(edge);
  }
  // 把保留的边界按机器人中心做顺时针排序（主要是为了 RViz 显示更整齐）。
  FARUtil::SortEdgesClockWise(robot_pos_, sorted_boundary); /* For better rviz visualization 
  // 把每条线段的两个端点写入 polygon.points，然后发布到 /navigation_boundary。purpose only! */ 
  for (const auto& edge : sorted_boundary) {
    geometry_msgs::Point32 geo_p1, geo_p2;
    geo_p1.x = edge.first.x,  geo_p1.y = edge.first.y,  geo_p1.z = index_z;
    geo_p2.x = edge.second.x, geo_p2.y = edge.second.y, geo_p2.z = index_z;
    boundary_poly.polygon.points.push_back(geo_p1), boundary_poly.polygon.points.push_back(geo_p2);
    index_z += 0.001f; // seperate polygon lines
  }
  boundary_pub_.publish(boundary_poly);
}

// 把图上的导航节点位置，投影/修正成一个更适合下游机器人去跟踪的 waypoint。点是xyz坐标，方向是 heading 向量。这个函数的作用是“把图节点位置投影成一个更平滑、更前瞻的 waypoint”，避免机器人在跟踪时出现“急转弯”或“方向突变”。

// 局部跟踪点生成器
// 输入当前选中的导航节点 nav_node_ptr（来自图搜索结果）
// 可选做前瞻延伸（ExtendViewpointOnObsCloud）
// 融合历史 heading 做动量平滑，减少急转
// 避免方向反跳（接近反向时重投影）
// 保证最小前视距离，输出最终 waypoint 给底层控制跟踪
Point3D FARMaster::ProjectNavWaypoint(const NavNodePtr& nav_node_ptr, const NavNodePtr& last_point_ptr) {
  bool is_momentum = false;
  // 判断是否要保留运动惯性
  // 如果上一次跟踪的点和这次节点很接近，或者就是同一个点，就认为可以保留一部分上一时刻的 heading，避免 waypoint 方向突变。
  if (last_point_ptr == nav_node_ptr || (last_point_ptr != NULL && (last_point_ptr->position - nav_node_ptr_->position).norm() < FARUtil::kNearDist)) {
    is_momentum = true;
  }
  // 计算候选 waypoint
  // 初始先取 nav_node_ptr->position。
  // 然后调用 ExtendViewpointOnObsCloud(...)，尝试把点沿着可视方向再往前“推一点”，让 waypoint 更像一个前瞻点，而不是死贴在图节点上。
  Point3D waypoint = nav_node_ptr->position;
  // 默认参数输入值5m
  float free_dist = master_params_.local_planner_range;
  const Point3D extend_p = this->ExtendViewpointOnObsCloud(nav_node_ptr_, FARUtil::surround_obs_cloud_, free_dist);
  free_dist = std::max(free_dist, master_params_.robot_dim * 2.5f);
  if (master_params_.is_viewpoint_extend) {
    waypoint = extend_p;
    planner_viz_.VizViewpointExtend(nav_node_ptr_, waypoint);
  }
  // 融合 heading，避免急转
  // 如果启用了 momentum，它会把“当前应去方向”和“上一时刻 heading”做加权融合，让新方向更平滑。
  // 这个的目的是当终点就在前一个附近时，不要反复的调换方向
  // nav_heading_ 当前的方向
  const Point3D diff_p = waypoint - robot_pos_;
  Point3D new_heading;
  if (is_momentum && nav_heading_.norm() > FARUtil::kEpsilon) {
    const float hdist = free_dist / 2.0f;
    const float ratio = std::min(hdist, diff_p.norm()) / hdist;
    new_heading = diff_p.normalize() * ratio + nav_heading_ * (1.0f - ratio);
  } else {
    new_heading = diff_p.normalize();
  }
  // 防止方向反跳
  // 如果新 heading 和旧 heading 近乎反向，它会做一次侧向重投影，避免 waypoint 一下子跳到相反方向，导致控制不稳定。
  if (nav_heading_.norm() > FARUtil::kEpsilon && new_heading.norm_dot(nav_heading_) < 0.0f) { // negative direction reproject
    Point3D temp_heading(nav_heading_.y, -nav_heading_.x, nav_heading_.z);
    if (temp_heading.norm_dot(new_heading) < 0.0f) {
      temp_heading.x = -temp_heading.x, temp_heading.y = -temp_heading.y;
    }
    new_heading = temp_heading;
  }
  // 保证 waypoint 有前瞻距离
  // 如果 waypoint 离机器人太近，它会沿 heading 再往前推一点，至少保持 free_dist 的跟踪前视距离。
  nav_heading_ = new_heading.normalize();
  if (diff_p.norm() < free_dist) {
    waypoint = waypoint + nav_heading_ * (free_dist - diff_p.norm());
  }
  return waypoint;
}

// 它试图把当前导航节点沿着一个“更合理的可视/拓扑方向”往前延伸成一个更好的 waypoint，但如果前面有障碍，就提前停下甚至往回收一点。

// 这个地方存在一个代码瑕疵,但是还不会影响整体规划器的功能.就是nav_node_ptr_和nav_node_ptr的使用不一致,导致在某些情况下会出现错误的结果.但是目前还没有发现明显的bug,所以暂时不修改.
Point3D FARMaster::ExtendViewpointOnObsCloud(const NavNodePtr& nav_node_ptr, const PointCloudPtr& obsCloudIn, float& free_dist) {
  // 如果这个导航节点不是凸自由方向，或者周围没有障碍点云，就直接返回原节点位置。
  // 也就是说，不是所有节点都会被延伸。
  if (nav_node_ptr_->free_direct != NodeFreeDirect::CONVEX || obsCloudIn->empty())        
    return nav_node_ptr_->position;
  // 从障碍云里裁一小块“节点周围的局部障碍”
  // 它会把 nav node 周围一定范围内的障碍点取出来，后面只在这小片局部障碍里判断。
  // 所以这是一个局部几何修正，不是全局搜索。
  FARUtil::CropPCLCloud(obsCloudIn, viewpoint_around_ptr_, nav_node_ptr_->position, free_dist + FARUtil::kNearDist);
  float maxR = std::min((nav_node_ptr_->position - robot_pos_).norm(), free_dist) - FARUtil::kNearDist;
  maxR = std::max(maxR, 0.0f);
  bool is_wall = false;
  // 确定要往哪个方向推
  // 从节点附近障碍/表面拓扑里估一个方向
  // 再取反，得到“朝开放空间/前方”去的方向
  const Point3D direct = -FARUtil::SurfTopoDirect(nav_node_ptr_->surf_dirs, is_wall);
  if (!is_wall) {
    Point3D waypoint = nav_node_ptr_->position;
    // 如果周围根本没有障碍点
    // 那就简单很多：直接沿 direct 推到 maxR 那么远
    if (viewpoint_around_ptr_->empty()) {
      waypoint = waypoint + direct * maxR;
    } else {
      kdtree_viewpoint_obs_cloud_->setInputCloud(viewpoint_around_ptr_);
      const int N_Thred = (int)std::floor(FARUtil::kNearDist / FARUtil::kLeafSize);
      const float R = FARUtil::kNearDist / 2.0f + FARUtil::kLeafSize;
      // ray tracing
      Point3D start_p = waypoint + direct * FARUtil::kNearDist;
      float ray_dist  = FARUtil::kNearDist; 
      bool is_occupied = FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_) > N_Thred;
      waypoint = start_p;
      // 如果周围有障碍点，就做一段“射线试探”
      // 这部分最像你看到的“反弹感”来源。
      // 它会从节点前方一点点开始，沿 direct 一步一步往前探：
      // 每走一步，就检查当前位置附近是否有足够多障碍点
      // 如果还没碰到障碍，继续往前
      // 一旦碰到障碍，停止前推
      while (!is_occupied && ray_dist < free_dist) {
        start_p = start_p + direct * FARUtil::kNearDist;
        ray_dist += FARUtil::kNearDist;
        is_occupied = FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_) > N_Thred;
        if (ray_dist < maxR) {
          waypoint = start_p;
        }
      }
      // 碰到障碍后，不是停在碰撞点，而是回收一点
      if (is_occupied) {
        waypoint = (nav_node_ptr_->position + waypoint - direct * FARUtil::kNearDist) / 2.0f;
        waypoint.z = nav_node_ptr_->position.z;
        free_dist = ray_dist - FARUtil::kNearDist;
      }
      return waypoint;
    }
  }
  return nav_node_ptr_->position;
}


void FARMaster::LoadROSParams() {
  const std::string master_prefix   = "/far_planner/";
  const std::string map_prefix      = master_prefix + "MapHandler/";
  const std::string scan_prefix     = master_prefix + "ScanHandler/";
  const std::string cdetect_prefix  = master_prefix + "CDetector/";
  const std::string graph_prefix    = master_prefix + "Graph/";
  const std::string viz_prefix      = master_prefix + "Viz/";
  const std::string utility_prefix  = master_prefix + "Util/";
  const std::string planner_prefix  = master_prefix + "GPlanner/";
  const std::string contour_prefix  = master_prefix + "ContourGraph/";
  const std::string msger_prefix    = master_prefix + "GraphMsger/";

  // master params
  nh.param<float>(master_prefix + "main_run_freq",         master_params_.main_run_freq, 5.0);
  nh.param<float>(master_prefix + "voxel_dim",             master_params_.voxel_dim, 0.2);
  nh.param<float>(master_prefix + "robot_dim",             master_params_.robot_dim, 0.8);
  nh.param<float>(master_prefix + "vehicle_height",        master_params_.vehicle_height, 0.75);
  nh.param<float>(master_prefix + "sensor_range",          master_params_.sensor_range, 50.0);
  nh.param<float>(master_prefix + "terrain_range",         master_params_.terrain_range, 15.0);
  nh.param<float>(master_prefix + "local_planner_range",   master_params_.local_planner_range, 5.0);
  nh.param<float>(master_prefix + "visualize_ratio",       master_params_.viz_ratio, 1.0);
  nh.param<bool>(master_prefix  + "is_viewpoint_extend",   master_params_.is_viewpoint_extend, true);
  nh.param<bool>(master_prefix  + "is_multi_layer",        master_params_.is_multi_layer, false);
  nh.param<bool>(master_prefix  + "is_opencv_visual",      master_params_.is_visual_opencv, true);
  nh.param<bool>(master_prefix  + "is_static_env",         master_params_.is_static_env, true);
  nh.param<bool>(master_prefix  + "is_pub_boundary",       master_params_.is_pub_boundary, true);
  nh.param<bool>(master_prefix  + "is_debug_output",       master_params_.is_debug_output, false);
  nh.param<bool>(master_prefix  + "is_attempt_autoswitch", master_params_.is_attempt_autoswitch, true);
  nh.param<std::string>(master_prefix + "world_frame",     master_params_.world_frame, "map");
  master_params_.terrain_range = std::min(master_params_.terrain_range, master_params_.sensor_range);

  // map handler params
  nh.param<float>(map_prefix + "floor_height",        map_params_.floor_height, 2.0);
  nh.param<float>(map_prefix + "cell_length",         map_params_.cell_length, 5.0);
  nh.param<float>(map_prefix + "map_grid_max_length", map_params_.grid_max_length, 5000.0);
  nh.param<float>(map_prefix + "map_grad_max_height", map_params_.grid_max_height, 100.0);
  map_params_.height_voxel_dim = master_params_.voxel_dim * 2.0f;
  map_params_.cell_height      = map_params_.floor_height / 2.5f;
  map_params_.sensor_range     = master_params_.sensor_range;

  // utility params
  nh.param<float>(utility_prefix + "angle_noise",            FARUtil::kAngleNoise, 15.0);
  nh.param<float>(utility_prefix + "accept_max_align_angle", FARUtil::kAcceptAlign, 15.0);
  nh.param<float>(utility_prefix + "new_intensity_thred",    FARUtil::kNewPIThred, 2.0);
  nh.param<float>(utility_prefix + "nav_clear_dist",         FARUtil::kNavClearDist, 0.5);
  nh.param<float>(utility_prefix + "terrain_free_Z",         FARUtil::kFreeZ, 0.1);
  nh.param<int>(utility_prefix   + "dyosb_update_thred",     FARUtil::kDyObsThred, 4);
  nh.param<int>(utility_prefix   + "new_point_counter",      FARUtil::KNewPointC, 10);
  nh.param<float>(utility_prefix + "dynamic_obs_dacay_time", FARUtil::kObsDecayTime, 10.0);
  nh.param<float>(utility_prefix + "new_points_decay_time",  FARUtil::kNewDecayTime, 2.0);
  nh.param<int>(utility_prefix   + "obs_inflate_size",       FARUtil::kObsInflate, 2);
  FARUtil::kLeafSize       = master_params_.voxel_dim;
  FARUtil::kNearDist       = master_params_.robot_dim;
  FARUtil::kHeightVoxel    = map_params_.height_voxel_dim;
  FARUtil::kMatchDist      = master_params_.robot_dim * 2.0f + FARUtil::kLeafSize;
  FARUtil::kNavClearDist   = master_params_.robot_dim / 2.0f + FARUtil::kLeafSize;
  FARUtil::kProjectDist    = master_params_.voxel_dim;
  FARUtil::worldFrameId    = master_params_.world_frame;
  FARUtil::kVizRatio       = master_params_.viz_ratio;
  FARUtil::kTolerZ         = map_params_.floor_height - FARUtil::kHeightVoxel;
  FARUtil::kCellLength     = map_params_.cell_length;
  FARUtil::kCellHeight     = map_params_.cell_height;
  FARUtil::kAcceptAlign    = FARUtil::kAcceptAlign / 180.0f * M_PI;
  FARUtil::kAngleNoise     = FARUtil::kAngleNoise  / 180.0f * M_PI; 
  FARUtil::robot_dim       = master_params_.robot_dim;
  FARUtil::IsStaticEnv     = master_params_.is_static_env;
  FARUtil::IsDebug         = master_params_.is_debug_output;
  FARUtil::IsMultiLayer    = master_params_.is_multi_layer;
  FARUtil::vehicle_height  = master_params_.vehicle_height;
  FARUtil::kSensorRange    = master_params_.sensor_range;
  FARUtil::kMarginDist     = master_params_.sensor_range - FARUtil::kMatchDist;
  FARUtil::kMarginHeight   = FARUtil::kTolerZ - FARUtil::kCellHeight / 2.0f;
  FARUtil::kTerrainRange   = master_params_.terrain_range;
  FARUtil::kLocalPlanRange = master_params_.local_planner_range;

  // graph planner params
  nh.param<float>(planner_prefix + "converge_distance",    gp_params_.converge_dist, 1.0);
  nh.param<float>(planner_prefix + "goal_adjust_radius",   gp_params_.adjust_radius, 10.0);
  nh.param<int>(planner_prefix   + "free_counter_thred",   gp_params_.free_thred, 5);
  nh.param<int>(planner_prefix   + "reach_goal_vote_size", gp_params_.votes_size, 5);
  nh.param<int>(planner_prefix   + "path_momentum_thred",  gp_params_.momentum_thred, 5);
  gp_params_.momentum_dist = master_params_.robot_dim / 2.0f;
  gp_params_.is_autoswitch = master_params_.is_attempt_autoswitch;

  // contour graph params
  cg_params_.kPillarPerimeter = master_params_.robot_dim * 4.0f;

  // dynamic graph params
  nh.param<int>(graph_prefix    + "connect_votes_size",        graph_params_.votes_size, 10);
  nh.param<int>(graph_prefix    + "clear_dumper_thred",        graph_params_.dumper_thred, 3);
  nh.param<int>(graph_prefix    + "node_finalize_thred",       graph_params_.finalize_thred, 3);
  nh.param<int>(graph_prefix    + "filter_pool_size",          graph_params_.pool_size, 12);
  nh.param<float>(graph_prefix  + "connect_angle_thred",       graph_params_.kConnectAngleThred, 10.0);
  nh.param<float>(graph_prefix  + "dirs_filter_margin",        graph_params_.filter_dirs_margin, 10.0);
  graph_params_.filter_pos_margin        = FARUtil::kNavClearDist;
  graph_params_.filter_dirs_margin       = FARUtil::kAngleNoise;
  graph_params_.kConnectAngleThred       = FARUtil::kAcceptAlign;
  graph_params_.frontier_perimeter_thred = FARUtil::kMatchDist * 4.0f;

  // graph messager params
  nh.param<int>(msger_prefix + "robot_id", msger_parmas_.robot_id, 0);
  msger_parmas_.frame_id    = master_params_.world_frame;
  msger_parmas_.votes_size  = graph_params_.votes_size;
  msger_parmas_.pool_size   = graph_params_.pool_size;
  msger_parmas_.dist_margin = graph_params_.filter_pos_margin;

  // scan handler params
  scan_params_.terrain_range = master_params_.terrain_range;
  scan_params_.voxel_size    = master_params_.voxel_dim;
  scan_params_.ceil_height   = map_params_.floor_height;

  // contour detector params
  nh.param<float>(cdetect_prefix       + "resize_ratio",       cdetect_params_.kRatio, 5.0);
  nh.param<int>(cdetect_prefix         + "filter_count_value", cdetect_params_.kThredValue, 5);
  nh.param<bool>(cdetect_prefix        + "is_save_img",        cdetect_params_.is_save_img, false);
  nh.param<std::string>(cdetect_prefix + "img_folder_path",    cdetect_params_.img_path, "");
  cdetect_params_.kBlurSize    = (int)std::round(FARUtil::kNavClearDist / master_params_.voxel_dim);
  cdetect_params_.sensor_range = master_params_.sensor_range;
  cdetect_params_.voxel_dim    = master_params_.voxel_dim;
}

// 初始时刻，如果odom frame和world frame不一致，则需要进行坐标变换，将odom frame下的位姿转换到world frame下，并将此位置作为初始位置
void FARMaster::OdomCallBack(const nav_msgs::OdometryConstPtr& msg) {
  // transform from odom frame to mapping frame
  std::string odom_frame = msg->header.frame_id;
  tf::Pose tf_odom_pose;
  tf::poseMsgToTF(msg->pose.pose, tf_odom_pose);
  if (!FARUtil::IsSameFrameID(odom_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) ROS_WARN_ONCE("FARMaster: odom frame does NOT match with world frame!");
    tf::StampedTransform odom_to_world_tf_stamp;
    try
    {
      tf_listener_->waitForTransform(master_params_.world_frame, odom_frame, ros::Time(0), ros::Duration(1.0));
      tf_listener_->lookupTransform(master_params_.world_frame, odom_frame, ros::Time(0), odom_to_world_tf_stamp);
      tf_odom_pose = odom_to_world_tf_stamp * tf_odom_pose;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Tracking odom TF lookup: %s",ex.what());
      return;
    }
  }
  robot_pos_.x = tf_odom_pose.getOrigin().getX(); 
  robot_pos_.y = tf_odom_pose.getOrigin().getY();
  robot_pos_.z = tf_odom_pose.getOrigin().getZ();
  // extract robot heading
  FARUtil::robot_pos = robot_pos_;
  double roll, pitch, yaw;
  tf_odom_pose.getBasis().getRPY(roll, pitch, yaw);
  robot_heading_ = Point3D(cos(yaw), sin(yaw), 0);

  if (!is_odom_init_) {
    // system start time
    FARUtil::systemStartTime = ros::Time::now().toSec();
    FARUtil::map_origin = robot_pos_;
    map_handler_.UpdateRobotPosition(robot_pos_);
  }

  is_odom_init_ = true;
}

// 体素滤波、nan滤波，如果odom frame和world frame不一致，则进行坐标变换
void FARMaster::PrcocessCloud(const sensor_msgs::PointCloud2ConstPtr& pc,
                             const PointCloudPtr& cloudOut) 
{

  pcl::PointCloud<PCLPoint> temp_cloud;
  pcl::fromROSMsg(*pc, temp_cloud);
  cloudOut->clear(), *cloudOut = temp_cloud;
  if (cloudOut->empty()) return;
  FARUtil::FilterCloud(cloudOut, master_params_.voxel_dim);
  // transform cloud frame
  std::string cloud_frame = pc->header.frame_id;
  FARUtil::RemoveNanInfPoints(cloudOut);
  if (!FARUtil::IsSameFrameID(cloud_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) ROS_WARN_ONCE("FARMaster: cloud frame does NOT match with world frame!");
    try
    {
      FARUtil::TransformPCLFrame(cloud_frame, 
                                master_params_.world_frame, 
                                tf_listener_,
                                cloudOut);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("Tracking cloud TF lookup: %s",ex.what());
      return;
    }
  }
}

// ScanCallBack 的输出 cur_scan_cloud_ 会被 TerrainCallBack 在动态障碍提取中消费，
void FARMaster::ScanCallBack(const sensor_msgs::PointCloud2ConstPtr& scan_pc) {
  if (master_params_.is_static_env || !is_odom_init_) return;
  this->PrcocessCloud(scan_pc, FARUtil::cur_scan_cloud_);
  scan_handler_.UpdateRobotPosition(robot_pos_);
}

// 上游给的点云无论是TerrainLocal还是TerrainExt都是100HZ
void FARMaster::TerrainLocalCallBack(const sensor_msgs::PointCloud2ConstPtr& pc) {
  if (master_params_.is_static_env) return;
  this->PrcocessCloud(pc, local_terrain_ptr_);
  FARUtil::ExtractFreeAndObsCloud(local_terrain_ptr_, FARUtil::local_terrain_free_, FARUtil::local_terrain_obs_);
}

void FARMaster::TerrainCallBack(const sensor_msgs::PointCloud2ConstPtr& pc) {
  if (!is_odom_init_) return;
  // update map grid robot center
  map_handler_.UpdateRobotPosition(FARUtil::robot_pos);
  if (!is_stop_update_) {
    this->PrcocessCloud(pc, temp_cloud_ptr_);
    // 仅保留FARUtil::kTolerZ方格内的点
    FARUtil::CropBoxCloud(temp_cloud_ptr_, robot_pos_, Point3D(master_params_.terrain_range, master_params_.terrain_range, FARUtil::kTolerZ));
    // 根据kFreeZ来划分自由点云和障碍点云，intensity>kFreeZ的点是障碍点云，intensity<=kFreeZ的点是自由点云，intensity表示点与地面的高度差
    FARUtil::ExtractFreeAndObsCloud(temp_cloud_ptr_, temp_free_ptr_, temp_obs_ptr_);
    // 这个的意思是一旦假设环境是动态的，就会把当前帧的障碍点云和历史动态障碍点云做一个“去重”，避免重复计算。
    if (!master_params_.is_static_env) {
      // 在temp_obs_ptr_删除FARUtil::stack_dyobs_cloud_重复的点
      // 只是做了一个“空间上重叠”的去除：如果当前帧的障碍点和历史动态点云落在同一个体素/近邻区域，就会直接把当前点删掉。它没有考虑“这个历史点到底是刚刚出现的动态物体，还是已经停下来变成静态环境的一部分”。
      FARUtil::RemoveOverlapCloud(temp_obs_ptr_, FARUtil::stack_dyobs_cloud_, true);
    }
    map_handler_.UpdateObsCloudGrid(temp_obs_ptr_);
    map_handler_.UpdateFreeCloudGrid(temp_free_ptr_);
    // extract new points
    // 从“当前帧看到的障碍点云”里，提取出“相对当前周围已知障碍地图来说新出现的那些点”，然后把结果写进 cur_new_cloud_
    FARUtil::ExtractNewObsPointCloud(temp_obs_ptr_, FARUtil::surround_obs_cloud_, FARUtil::cur_new_cloud_);
  } else { // stop env update
    temp_cloud_ptr_->clear();
    FARUtil::cur_new_cloud_->clear();
  }
  // extract surround free cloud & update terrain height
  map_handler_.GetSurroundFreeCloud(FARUtil::surround_free_cloud_);

  map_handler_.UpdateTerrainHeightGrid(FARUtil::surround_free_cloud_, terrain_height_ptr_);
  // update surround obs cloud
  map_handler_.GetSurroundObsCloud(FARUtil::surround_obs_cloud_);
  // extract dynamic obstacles
  FARUtil::cur_dyobs_cloud_->clear();
  dyremove_before_obs_ptr_->clear();
  if (!master_params_.is_static_env && !is_stop_update_) {
    // 把当前激光扫描、当前环境障碍、当前自由空间这三份信息送进提取器，输出本帧动态障碍点云。
    this->ExtractDynamicObsFromScan(FARUtil::cur_scan_cloud_, 
                                    FARUtil::surround_obs_cloud_, 
                                    FARUtil::surround_free_cloud_, 
                                    FARUtil::cur_dyobs_cloud_);
    // 把这些动态点从地图里删除
    // 当 cur_dyobs_cloud_ 数量超过阈值后，会执行：
    if (FARUtil::cur_dyobs_cloud_->size() > FARUtil::kDyObsThred) {
      if (FARUtil::IsDebug) ROS_WARN("FARMaster: dynamic obstacle detected, size: %ld", FARUtil::cur_dyobs_cloud_->size());

      pcl::copyPointCloud(*FARUtil::surround_obs_cloud_, *dyremove_before_obs_ptr_);

      FARUtil::InflateCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim, 1, true);
      // 从地图中删除这些动态障碍点云，避免它们影响后续的可视图更新和路径规划。
      map_handler_.RemoveObsCloudFromGrid(FARUtil::cur_dyobs_cloud_);
      // 在该格子的障碍点云里，删除与 obsCloud 重叠的点。
      FARUtil::RemoveOverlapCloud(FARUtil::surround_obs_cloud_, FARUtil::cur_dyobs_cloud_);
      map_handler_.GetSurroundObsCloud(FARUtil::surround_obs_cloud_);
      FARUtil::FilterCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim);
      // update new cloud
      // 把“本帧检测到的动态障碍点”追加到“本帧新增障碍点缓冲”里。
      *FARUtil::cur_new_cloud_ += *FARUtil::cur_dyobs_cloud_;
      FARUtil::FilterCloud(FARUtil::cur_new_cloud_, master_params_.voxel_dim);
    }
    // update world dynamic obstacles
     // 动态障碍做时间栈衰减，动态障碍会保留一段时间再衰减，避免一帧抖动导致反复“加了又删”。
    FARUtil::StackCloudByTime(FARUtil::cur_dyobs_cloud_, FARUtil::stack_dyobs_cloud_, FARUtil::kObsDecayTime);
  }
  
  // create and update kdtrees
  // 把本帧新点 cur_new_cloud_ 加到历史缓冲 stack_new_cloud_，并按时间淘汰超过 kNewDecayTime 的旧点。也就是一个“带过期时间的滑动窗口点云”
 
  FARUtil::StackCloudByTime(FARUtil::cur_new_cloud_, FARUtil::stack_new_cloud_, FARUtil::kNewDecayTime);
  FARUtil::UpdateKdTrees(FARUtil::stack_new_cloud_);
  // 主要是为了保证后续可见图更新有障碍边界可提取，避免在全空数据时误进入主流程
  if (!FARUtil::surround_obs_cloud_->empty()) is_cloud_init_ = true;

  /* visualize clouds */
  planner_viz_.VizPointCloud(new_PCL_pub_, FARUtil::stack_new_cloud_);
  planner_viz_.VizPointCloud(dynamic_obs_pub_, FARUtil::cur_dyobs_cloud_);
  planner_viz_.VizPointCloud(surround_free_debug_, FARUtil::surround_free_cloud_);
  planner_viz_.VizPointCloud(surround_obs_debug_,  FARUtil::surround_obs_cloud_);
  planner_viz_.VizPointCloud(surround_obs_before_dyremove_debug_, dyremove_before_obs_ptr_);
  planner_viz_.VizPointCloud(surround_obs_after_dyremove_debug_, FARUtil::surround_obs_cloud_);
  planner_viz_.VizPointCloud(terrain_height_pub_, terrain_height_ptr_);
  // visualize map grid
  PointStack neighbor_centers, occupancy_centers;
  map_handler_.GetNeighborCeilsCenters(neighbor_centers);
  map_handler_.GetOccupancyCeilsCenters(occupancy_centers);
  planner_viz_.VizMapGrids(neighbor_centers, occupancy_centers, map_params_.cell_length, map_params_.cell_height);
  // DBBUG visual raycast grids
  if (!master_params_.is_static_env) {
    scan_handler_.GridVisualCloud(scan_grid_ptr_, GridStatus::RAY);
    planner_viz_.VizPointCloud(scan_grid_debug_, scan_grid_ptr_);
  }
}

// 得到动态点云
void FARMaster::ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                          const PointCloudPtr& obsCloudIn,
                                          const PointCloudPtr& freeCloudIn,
                                          const PointCloudPtr& dyObsCloudOut)
{
  scan_handler_.ReInitGrids();
  scan_handler_.SetCurrentScanCloud(scanCloudIn, freeCloudIn);
  scan_handler_.ExtractDyObsCloud(obsCloudIn, dyObsCloudOut);
}

// 终点回调函数，接收来自上层规划器的目标点，更新图规划器的目标点，并可视化原始目标点
// 接收外部发来的目标点话题 /goal_point
// 把目标点从目标坐标系转换到 world_frame（如果 frame 不一致）
// 调用 graph_planner_.UpdateGoal(goal_p) 更新全局目标
// 可视化原始目标点并启动整体计时
void FARMaster::WaypointCallBack(const geometry_msgs::PointStamped& route_goal) {
  if (!is_graph_init_) {
    if (FARUtil::IsDebug) ROS_WARN("FARMaster: wait for v-graph to init before sending any goals");
    return;
  }
  Point3D goal_p(route_goal.point.x, route_goal.point.y, route_goal.point.z);
  const std::string goal_frame = route_goal.header.frame_id;
  if (!FARUtil::IsSameFrameID(goal_frame, master_params_.world_frame)) {
    if (FARUtil::IsDebug) ROS_WARN_THROTTLE(1.0, "FARMaster: waypoint published is not on world frame!");
    FARUtil::TransformPoint3DFrame(goal_frame, master_params_.world_frame, tf_listener_, goal_p); 
  }
  graph_planner_.UpdateGoal(goal_p);
  FARUtil::Timer.start_time("Overall_executing", true);
  // visualize original goal
  planner_viz_.VizPoint3D(goal_p, "original_goal", VizColor::RED, 1.5);
}

/* allocate static utility PointCloud pointer memory */

// FARUtil 里都是静态变量，机器人通过这些静态变量来共享点云数据和参数设置
// 但是还需要查看这些变量相互之间有没有竞争关系

PointCloudPtr  FARUtil::surround_obs_cloud_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::surround_free_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_new_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_new_cloud_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_dyobs_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::stack_dyobs_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::cur_scan_cloud_      = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::local_terrain_obs_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointCloudPtr  FARUtil::local_terrain_free_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_new_cloud_    = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
PointKdTreePtr FARUtil::kdtree_filter_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
/* init static utility values */
const float FARUtil::kEpsilon = 1e-7;
const float FARUtil::kINF     = std::numeric_limits<float>::max();
std::string FARUtil::worldFrameId;
float   FARUtil::kAngleNoise; 
Point3D FARUtil::robot_pos;
Point3D FARUtil::odom_pos;
Point3D FARUtil::map_origin;
Point3D FARUtil::free_odom_p;
float   FARUtil::robot_dim;
float   FARUtil::vehicle_height;
float   FARUtil::kLeafSize;
float   FARUtil::kHeightVoxel;
float   FARUtil::kNavClearDist;
float   FARUtil::kCellLength;
float   FARUtil::kCellHeight;
float   FARUtil::kNewPIThred;
float   FARUtil::kSensorRange;
float   FARUtil::kMarginDist;
float   FARUtil::kMarginHeight;
float   FARUtil::kTerrainRange;
float   FARUtil::kLocalPlanRange;
float   FARUtil::kFreeZ;
float   FARUtil::kVizRatio;
double  FARUtil::systemStartTime;
float   FARUtil::kObsDecayTime;
float   FARUtil::kNewDecayTime;
float   FARUtil::kNearDist;
float   FARUtil::kMatchDist;
float   FARUtil::kProjectDist;
int     FARUtil::kDyObsThred;
int     FARUtil::KNewPointC;
int     FARUtil::kObsInflate;
float   FARUtil::kTolerZ;
float   FARUtil::kAcceptAlign;
bool    FARUtil::IsStaticEnv;
bool    FARUtil::IsDebug;
bool    FARUtil::IsMultiLayer;
TimeMeasure FARUtil::Timer;

/* Global Graph */
DynamicGraphParams DynamicGraph::dg_params_;
NodePtrStack DynamicGraph::globalGraphNodes_;
std::size_t  DynamicGraph::id_tracker_;
std::unordered_map<std::size_t, NavNodePtr> DynamicGraph::idx_node_map_;
std::unordered_map<NavNodePtr, std::pair<int, std::unordered_set<NavNodePtr>>> DynamicGraph::out_contour_nodes_map_;

/* init static contour graph values */
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

/* init terrain map values */
PointKdTreePtr MapHandler::kdtree_terrain_clould_;
std::vector<int> MapHandler::terrain_grid_occupy_list_;
std::vector<int> MapHandler::terrain_grid_traverse_list_;
std::unordered_set<int> MapHandler::neighbor_obs_indices_;
std::unordered_set<int> MapHandler::extend_obs_indices_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_free_cloud_grid_;
std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_obs_cloud_grid_;
std::unique_ptr<grid_ns::Grid<std::vector<float>>> MapHandler::terrain_height_grid_;


int main(int argc, char** argv){
  ros::init(argc, argv, "far_planner_node");
  FARMaster dp_node;
  dp_node.Init();
  dp_node.Loop();
}