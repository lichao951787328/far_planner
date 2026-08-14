# SSMI bag Graph 测试

该测试用一个 launch 完成 SSMI 点云适配、Semantic OctoMap、FAR Graph、目标适配、RViz、监控和 bag 播放。它不会启动 `local_planner`、`pathFollower`、`pathfollowing` 或 `graph_decoder`。

## 环境与默认数据

```bash
source /home/yanaibo/mapless_navigation/far_planner/devel/setup.bash
source /home/yanaibo/mapless_navigation/far_planner_semantic_ws/devel/setup.bash --extend
source /home/yanaibo/mapless_navigation/SSMI-example/ssmi_ws/devel/setup.bash --extend
```

默认 bag：

```text
/home/yanaibo/mapless_navigation/local_3d_semantic_voxel/local3DSemanticVoxelMap/dataset/2026-08-14-09-19-37.bag
```

统一启动：

```bash
roslaunch far_planner ssmi_bag_graph_test.launch
```

默认使用 `--clock --delay=3 --rate=0.5`，RViz Fixed Frame、OctoMap、Graph、Goal 和标准路径均为 `map_start`。RViz 的 `2D Nav Goal` 会转换成 `/goal_point`。

## 分阶段命令

35 秒接入烟测，不发目标：

```bash
roslaunch far_planner ssmi_bag_graph_test.launch \
  launch_rviz:=false \
  bag_play_args:='--clock --delay=3 --rate=0.5 --duration=35'
```

35 秒确定性目标测试：

```bash
roslaunch far_planner ssmi_bag_graph_test.launch \
  launch_rviz:=false auto_goal:=true \
  bag_play_args:='--clock --delay=3 --rate=0.5 --duration=35'
```

完整 bag 的纯 Graph 测试：

```bash
roslaunch far_planner ssmi_bag_graph_test.launch \
  launch_rviz:=false auto_goal:=false
```

完整 bag 的在线规划测试：

```bash
roslaunch far_planner ssmi_bag_graph_test.launch \
  launch_rviz:=false auto_goal:=true
```

全速压力测试：

```bash
roslaunch far_planner ssmi_bag_graph_test.launch \
  launch_rviz:=false auto_goal:=true \
  bag_play_args:='--clock --delay=3 --rate=1.0'
```

也可以在 Graph 初始化后手工发布：

```bash
rostopic pub -1 /goal_point geometry_msgs/PointStamped "
header:
  frame_id: 'map_start'
point:
  x: 7.98
  y: 0.24
  z: 0.0
"
```

## 输出与判定

- `/far_global_path`：`nav_msgs/Path`，包含搜索节点、轮廓跟随 route 几何和原始指令终点。
- `/viz_path_topic`：兼容原有 RViz Marker。
- `/way_point`：下一航点；无路或超时时回到机器人当前位置。
- `/far_reach_goal_status`：到达状态。
- `/semantic_graph_static_obstacles`：FAR Graph 碰撞校验实际使用的持久静态点云。
- `logs/ssmi_bag_graph_test.csv`：每次启动覆盖写入的独立监控结果。

监控进程在 bag 时钟停止后输出 `SSMI_MONITOR_RESULT PASS/FAIL` 并退出。自动目标模式要求：对齐和 frame 正确、SemanticOcTree 与静态点存在、机器人节点位于最大连通分量、路径包含至少两个点、末点到原始目标不超过 2 cm、所有当前 route 到静态障碍的距离不小于 0.45 m。

到达状态只说明 bag 中记录的轨迹进入目标范围；本测试没有让机器人按照 `/far_global_path` 执行。

不要使用 `rosbag play --loop`。重新播放前应重新启动整个 launch；若必须在同一进程中重播，应同时调用 Semantic OctoMap 和 FAR Graph 的重置接口，避免上一轮图和新一轮地图混合。

当前 bag 的 `map -> map_start` 仍由 Semantic OctoMap 在第一帧建立。统一 launch 会先启动所有 TF listener 并延迟播放，因此本测试不会丢失该变换；若以后允许 RViz 或监控节点中途启动，应在上游把该恒定变换改为 `StaticTransformBroadcaster`。
