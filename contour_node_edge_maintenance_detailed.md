# FAR Planner 当前轮廓、节点、连边与图维护详细说明

> 本文描述的是本仓库 **`far-planner-semantic-simple` 当前源码的实际行为**，不是原始 FAR Planner 论文的泛化介绍，也不是未来设计草案。
>
> 核对日期：2026-08-15。主要依据：`src/far_planner.cpp`、`src/contour_detector.cpp`、`src/contour_graph.cpp`、`src/dynamic_graph.cpp`、`src/map_handler.cpp`、`src/graph_planner.cpp` 及其对应头文件和 `config/default.yaml`。

## 1. 一句话总览

这套图不是把每个轮廓像素直接变成导航节点，而是按下面的层次工作：

```text
语义 OctoMap
  ├─ 当前静态障碍层 ────────> 静态轮廓
  ├─ 当前动态障碍层 ────────> 动态轮廓
  ├─ 持久静态碰撞记忆 ──────> 所有边的静态安全裁决
  └─ terrain-support 层 ─────> 节点贴地、边的高度/坡度检查
                                  │
                                  ▼
局部栅格投影 -> OpenCV 闭合轮廓 -> RDP/共线简化 -> CTNode 几何分类
                                  │
                                  ▼
与已有 NavNode 做确定性一对一匹配
  ├─ 匹配成功：复用节点身份、积累稳定性、更新当前轮廓关系
  └─ 未匹配：只保留有效角点/柱体，生成新的 NavNode 候选
                                  │
                                  ▼
构造并验证两类障碍图边
  ├─ contour-follow：沿同一当前轮廓的自由侧绕行
  └─ visibility：自由空间中的可视直连
                                  │
                                  ▼
静态候选确认、动态节点当帧维护、旧角点/旧轮廓边延迟替换
                                  │
                                  ▼
最后重建 odom/start 边；规划时再重建 goal 边；仅搜索 active 边
```

核心原则有五条：

1. **轮廓几何、导航节点身份、物理占用和图拓扑是四种不同生命周期。**
2. **静态结构可以持久，动态结构只属于最新语义快照。**
3. **动态障碍只临时屏蔽静态边，不删除静态拓扑历史。**
4. **动态 mask、查询边和 contour route 的物理碰撞会立即反映；普通 visibility 边仍保留 FAR 的投票阻尼。拓扑身份删除还要积累证据并保护连通性。**
5. **odom 和 goal 是查询层端点，不是持久障碍图结构。**

推荐按下面的方式阅读：先读第 2 章认识对象，再顺序读第 3 章掌握一次快照真实执行了什么；第 4～10 章分别展开轮廓、节点和各种边的算法细节；第 11 章再把分散在多个函数里的全局静态边规则收束成完整生命周期；最后查参数、状态机和调试信息。

---

## 2. 关键对象与“谁负责什么”

### 2.1 `Polygon`：当前快照中的障碍轮廓

`Polygon` 由本帧 OpenCV 轮廓创建，主要字段如下：

| 字段 | 含义 |
|---|---|
| `vertices` / `N` | 简化后的世界坐标轮廓顶点 |
| `is_robot_inside` | `free_odom_p` 是否在该多边形内部，用于区分边是否跑到了障碍错误的一侧 |
| `is_pillar` | 小轮廓是否整体压缩成一个柱体节点 |
| `is_boundary_clipped` | 多边形是否碰到局部轮廓画布的可靠边界带 |
| `perimeter` | 用于柱体和 frontier 判断的轮廓周长 |
| `source` | 静态 `STATIC_CANDIDATE` 或动态 `DYNAMIC_LOCAL` |

`Polygon` 每个语义快照都会重建，不承担跨帧身份。

### 2.2 `CTNode`：当前轮廓上的几何节点

`CTNode` 是“本帧轮廓顶点”，不是最终搜索图节点。

| 字段 | 含义 |
|---|---|
| `position` | 轮廓顶点坐标，稍后会按 terrain-support 调整 z |
| `front` / `back` | 同一闭合轮廓的前后邻接顶点 |
| `surf_dirs` | 从角点沿轮廓两侧得到的两个表面方向 |
| `free_direct` | `UNKNOW`、`CONVEX`、`CONCAVE` 或 `PILLAR` |
| `poly_ptr` | 所属当前 `Polygon` |
| `is_global_match` | 本帧是否已经匹配某个历史 `NavNode` |
| `nav_node_id` | 匹配到的 `NavNode::id` |
| `is_contour_necessary` | 为保持轮廓拓扑而强制保留的中间点 |
| `is_boundary_clipped` | 位于画布可靠范围之外的裁剪端点 |
| `source` | 静态或动态语义来源 |

### 2.3 `NavNode`：真正参与维护和搜索的导航图节点

最重要的三组信息：

1. 几何身份：`position`、`surf_dirs`、`free_direct`、`ctnode`。
2. 图关系：`connect_nodes`、`poly_connects`、`contour_connects`、投票表和 `edge_states`。
3. 生命周期：`source`、`observed_in_semantic_snapshot`、`static_seen_count`、`static_missed_count`、`topology_missed_count` 等。

`connect_nodes` 是搜索邻接总表；另外两张表表达边的“身份”：

| 容器 | 含义 |
|---|---|
| `poly_connects` | visibility 边身份 |
| `contour_connects` | contour-follow 边身份 |
| `connect_nodes` | 两类边及查询边汇总后的实际邻接 |
| `potential_edges` + `edge_votes` | visibility 候选和历史投票 |
| `potential_contours` + `contour_votes` | contour 候选和历史投票 |
| `edge_states[neighbor_id]` | 该有向邻接当前是否可被搜索、实际安全路线和边来源 |

同一对节点可能同时具有 visibility 和 contour 身份。删除一种身份时，只有在没有另一种身份的情况下才真正从 `connect_nodes` 删除。

### 2.4 节点来源 `GraphNodeSource`

| 枚举 | 当前实现中的生命周期 |
|---|---|
| `ODOM` | 唯一机器人当前位姿节点；持续移动，查询边反复重建 |
| `GOAL` | 当前命令目标；规划查询层 |
| `STATIC_CANDIDATE` | 当前可搜索但尚未持久确认的静态角点 |
| `STATIC_GLOBAL` | 已确认的持久静态角点 |
| `DYNAMIC_LOCAL` | 只属于最新语义快照的动态障碍角点 |
| `PATH_HISTORY` | 当前语义图明确排除；历史轨迹点不会进入搜索图 |
| `UNKNOWN` | 不应作为正常语义图搜索节点 |

### 2.5 边状态 `GraphEdgeState`

一条边是否可搜索由下面的合取决定：

```cpp
static_valid && !dynamic_blocked && !topology_blocked && active
```

字段含义：

| 字段 | 含义 |
|---|---|
| `source` | `STATIC_VISIBILITY`、`STATIC_CONTOUR`、`DYNAMIC_LOCAL`、`STITCH`、`ODOM_CONNECT`、`GOAL_CONNECT` 等 |
| `validation_mode` | 普通 `VISIBILITY` 或 `CONTOUR_FOLLOW` |
| `static_valid` | 最新静态几何检查是否通过；失败会立即停止搜索使用 |
| `dynamic_blocked` | 最新动态层是否挡住；只做临时 mask |
| `topology_blocked` | 已提交的拓扑阻塞；当前实现不会因一次局部 miss 就设置它 |
| `active` | 通用启用位 |
| `has_clearance_geometry` | 是否保存了真正经过碰撞检查的机器人中心路线 |
| `route_start/end` | 投影到自由侧后的实际路线端点 |
| `route_cost` | 原节点到投影点、投影线段、投影点回节点的总长度 |
| `current_contour_misses` | 旧 contour 关系连续被当前可靠轮廓否定的次数 |

搜索要求两个方向的 `edge_states` 都存在且都 active，避免单向状态不一致。

---

## 3. 一次语义快照的完整时序

这一章只沿着真实调用顺序向前讲。先区分两个时间点：

1. 语义地图回调先解析 OctoMap，生成当前静态、当前动态、terrain-support 和持久静态碰撞层，并把 `semantic_graph_dirty_` 置为需要更新；
2. 主循环发现有新语义快照后，才打开一次 `BeginSemanticGraphUpdate()` 到 `CommitSemanticGraphUpdate()` 的图更新事务。

完整调用链如下：

```text
语义地图回调：解析并生成四个语义/碰撞数据层
    ↓
主循环检测到 semantic_graph_dirty_
    ↓
BeginSemanticGraphUpdate()                    ┐
    ↓                                         │
分别提取静态/动态轮廓                          │
    ↓                                         │
UpdateContourGraph() 建立本帧 Polygon/CTNode   │
    ↓                                         │
高度校正 + UpdateGlobalNearNodes()             │ 一次语义图更新事务
    ↓                                         │
MatchContourWithNavGraph()                    │
    ↓                                         │
FinalizeDynamicGraphUpdate()                  │
    ↓                                         │
ExtractGraphNodes()                           │
    ↓                                         │
UpdateNavGraph()                              │
    ↓                                         │
CommitSemanticGraphUpdate()                   ┘
    ↓
GetNavGraph() + ExtractGlobalContours()
    ↓
UpdaetVGraph() + 消息/RViz 发布
```

因此，`BeginSemanticGraphUpdate()` 不是与轮廓提取并列的一整阶段；它是一次事务的**开场动作**。轮廓提取、匹配、连边和生命周期处理都发生在该事务尚未提交的期间，直到 `CommitSemanticGraphUpdate()` 才结束。

### 3.1 背景准备：语义地图回调派生四个关键层

`MapHandler::SetSemanticOctomap()` 先验证：

- 消息能否反序列化；
- 类型是否为支持语义颜色的 `SemanticOcTree` 或 `ColorOcTree`；
- frame 是否和 `world_frame` 一致。

随后以机器人为中心，从轴对齐方形 BBX 提取：

- `semantic_obs_cloud_`：当前静态障碍语义；
- `current_dynamic_obs_cloud_`：当前动态障碍语义；
- `semantic_terrain_support_cloud_`：地面/楼梯等 terrain-support 语义；
- `persistent_static_obs_cloud_`：跨快照积累的静态碰撞权威层。

> **作用注释——为什么需要 `persistent_static_obs_cloud_`：**它不是“当前这一帧看见的静态点”的另一个副本，而是给 Graph 做碰撞判定用的静态障碍记忆。机器人移动后，旧墙体可能离开当前局部语义方窗，或者因遮挡暂时没有出现在新快照中；如果直接拿 `semantic_obs_cloud_` 判碰撞，这些墙会在一帧内从碰撞世界消失，已有 visibility、contour、odom 或 goal 边就可能被错误地判为可穿越。该层因此把每帧确认的静态障碍按栅格键并入 `persistent_static_obs_voxels_`，保留旧体素；只有旧体素位于当前可观测方窗内、当前帧不再占用，并且 OctoMap 明确证明该位置为 `EXPLICIT_FREE` 时才删除。生成的点云会交给 `ContourGraph::SetLocalCollisionCloud()`，作为普通可见边、轮廓跟随边以及 odom/goal 查询边的静态碰撞依据，并通过 `/semantic_graph_static_obstacles` 发布用于核对。当前轮廓提取仍使用 `semantic_obs_cloud_`，所以这个持久层负责“不能穿过哪些静态实体”，不负责凭旧点重复生成当前轮廓节点，也不等同于持久 `NavNode`。

当前默认类颜色来自 `config/default.yaml`：

- 静态障碍：chair、television、table、boundary_wall、maze_wall；
- terrain support：background_floor、staircase；
- 动态障碍：dynamic_obstacle。

注意：`SemanticOcTree` 使用 top-1 语义，并把 top-3 类别及聚合 `others` 的 log-weight 归一化为 top-1 概率；低于 `semantic_min_probability` 的占用体素按 `UNKNOWN` 处理，既不进入静态/动态/terrain-support 分类云，也不能作为清除持久静态障碍的 `EXPLICIT_FREE` 证据。没有类别概率的兼容 `ColorOcTree` 仍按 RGB 精确匹配。

#### 3.1.1 静态碰撞记忆与静态节点不是同一个东西

当前静态障碍体素会以 `contour_grid_resolution` 量化后写入 `persistent_static_obs_voxels_`。旧体素只有在：

1. 位于当前语义方窗内；
2. 当前快照没有同一静态占用；
3. OctoMap 查询明确给出 `EXPLICIT_FREE`；

时才从持久碰撞层删除。

这意味着：

- 轮廓简化或角点删除不会顺带删除墙体碰撞信息；
- 未观测、遮挡、动态占用都不是静态清除证据；
- 静态碰撞体素可在一帧明确 free 后删除，但相应持久 `NavNode` 仍需自己的连续三帧删除门限。

### 3.2 Begin 到 Commit：一次完整的语义图更新事务

本节从 `BeginSemanticGraphUpdate()` 开始，严格顺序讲到 `CommitSemanticGraphUpdate()`。下面的每个四级标题都是事务内部的一个环节，而不是与事务并列的独立流程。

#### 3.2.1 `BeginSemanticGraphUpdate()`：打开事务并清理本帧标记

每次真正接受的新语义快照只调用一次：

- 设置 `semantic_update_in_progress_ = true`；
- 所有 `DYNAMIC_LOCAL` 节点清除“本帧已观测/已匹配”标记；
- 所有静态候选和静态全局节点清除 `observed_in_semantic_snapshot`；
- 静态边的 `dynamic_blocked` 先清零，等待新动态层重算；
- 清除旧历史 path 节点；
- 清理遗留 internav 临时集合。

因此各种 miss 计数是按“接受的语义快照”推进，不按 2.5 Hz 主循环空转次数推进。

这一步只负责把旧状态复位成“等待当前快照重新证明”，并没有完成轮廓提取、节点删除或连边。后面的轮廓提取、匹配和图更新都在 `semantic_update_in_progress_ == true` 的事务窗口中执行。

#### 3.2.2 静态、动态轮廓分别提取

```text
current_static_obs_ptr_    -> static_contours_
effective_dynamic_obs_ptr_ -> dynamic_contours_，使用更强简化倍率
```

二者最后进入同一个 `ContourGraph`，但 `Polygon/CTNode::source` 保持不同，匹配和生命周期绝不混用。

#### 3.2.3 当前轮廓图重建

`UpdateContourGraph()` 会：

1. 清空上帧 `CTNode` 和 `Polygon`；
2. 为静态/动态轮廓创建不同 source 的 `Polygon`；
3. 修正 `free_odom_p`；
4. 小轮廓压成一个 `PILLAR` 节点；
5. 其他轮廓的每个简化顶点建 `CTNode`，形成 `front/back` 闭环；
6. 计算表面方向、直墙/凸角/凹角/柱体分类。

#### 3.2.4 高度校正和局部历史节点筛选

- `AdjustCTNodeHeight()`：用 terrain-support 查询把 CTNode 放到 `terrain_height + vehicle_height`；找不到地面时保持原高度，并标为未关联。
- `AdjustNodesHeight()`：在当前语义方窗中的已有 NavNode 同样贴地；boundary、odom、goal 等自由节点不在这里改。
- `UpdateGlobalNearNodes()`：从全局正式节点、静态候选和当前动态节点中，按 source、距离、高度、地形和激活状态重建本次快照使用的局部工作集。

静态节点使用 `static_stitch_radius`；动态节点使用 `sensor_range`。默认静态半径 28.5 m 是为了覆盖 20 m 半边长方窗的对角区域。

##### 3.2.4.1 这一步是局部索引重建，不是删除全局节点

`UpdateGlobalNearNodes()` 的目的不是选出固定数量的“最近 N 个节点”，也不是把范围外的节点从全局图中删除。它以当前机器人位置为中心，重新标记哪些已有节点需要参加本次语义快照的：

- 当前 CTNode 与历史 NavNode 匹配；
- 历史节点有效性复检；
- 局部 visibility/contour 边重连；
- `covered`、`frontier` 等局部状态更新。

函数开始时清空上一轮工作集，并重置节点的局部标志：

```cpp
near_nav_nodes_.clear();
wide_near_nodes_.clear();
extend_match_nodes_.clear();
margin_near_nodes_.clear();

node_ptr->is_near_nodes = false;
node_ptr->is_wide_near  = false;
```

范围外的 `STATIC_GLOBAL` 仍保存在持久全局图中，只是不参加这一轮局部匹配和局部重连。因此，“不在 near 集合”不等于“节点已删除”，也不等于全局路径搜索不能再使用该节点。

##### 3.2.4.2 四个集合不是四个互斥距离环

| 集合 | 准入含义 | 当前主要用途 |
|---|---|---|
| `extend_match_nodes_` | 仍值得用当前轮廓尝试匹配或复检的扩展集合 | 交给 `MatchContourWithNavGraph()`；`UpdateNavGraph()` 也遍历它复检已有节点 |
| `wide_near_nodes_` | 严格局部高度和地形合理，但不强制要求 active 的宽松局部集合 | 保存较完整的局部/相邻拓扑上下文；也用于兼容现有 wide-near 标记流程 |
| `near_nav_nodes_` | 严格局部范围内且 active 或 boundary 的核心工作集合 | 节点两两连边、旧边复检、contour 投票、动态 mask、`covered/frontier` 更新 |
| `margin_near_nodes_` | 在扩展范围内，但严格高度或地形检查暂未通过的 active/boundary 节点 | 等当前轮廓匹配结果；匹配成功后再提升到 near/wide |

正常空间分类完成后，关系大致是：

```text
near_nav_nodes_
  ├─ 同时属于 wide_near_nodes_
  └─ 同时属于 extend_match_nodes_

margin_near_nodes_
  └─ 一般也属于 extend_match_nodes_，但暂时不属于 near/wide
```

这个关系不是永久严格的集合包含关系：后面还会把 odom 的一跳、二跳拓扑邻居补入 `wide_near_nodes_`；匹配成功的 margin 节点也会被补入 near/wide。

特别要注意，`margin_near_nodes_` 不是一个单纯的 XY 半径“外环”。扩展范围和严格范围使用相同的 source 半径，二者主要区别是高度容差和地形条件。因此 margin 更多表示“高度/地形边界上的待确认节点”。

##### 3.2.4.3 先处理机器人上一轮的旧连接

分类已有节点前，函数先检查 `odom_node_ptr_` 的旧邻接。距离使用 XY 平面距离：

```text
d_xy = sqrt((node.x - odom.x)^2 + (node.y - odom.y)^2)
```

`ShouldPruneStartConnectionForRange()` 的规则如下：

| 旧 odom 边的另一端 | 仅因距离超限而裁掉旧边的门限 |
|---|---:|
| `DYNAMIC_LOCAL` | `sensor_range` |
| `STATIC_CANDIDATE` | `static_stitch_radius` |
| `STATIC_GLOBAL` | 不仅因离开局部窗口而裁掉 |
| 其他局部来源 | `static_stitch_radius` |

`STATIC_GLOBAL` 是已经确认的全局静态知识，所以不会仅因机器人移动、节点离开当前语义方窗就立刻失去旧的 start 连接资格。不过这不是无条件永久保留旧边：本轮 `UpdateNavGraph()` 结束时，`UpdateOdomConnections()` 会清空 odom 的全部邻接，并根据最新的静态碰撞、动态碰撞、全局轮廓和地形重新验证。

##### 3.2.4.4 待筛选节点来自三个生命周期容器

函数构造临时 `local_graph`：

```text
local_graph
  = globalGraphNodes_
  + staticCandidateGraphNodes_
  + dynamicLocalGraphNodes_
```

三部分分别表示：

- `globalGraphNodes_`：已进入正式全局图的节点，包含持久 `STATIC_GLOBAL` 等；
- `staticCandidateGraphNodes_`：已观测到、但尚未达到静态持久确认条件的候选角点；
- `dynamicLocalGraphNodes_`：只属于最新语义快照的动态障碍图节点。

随后跳过：

```cpp
if (node_ptr->source == GraphNodeSource::PATH_HISTORY ||
    node_ptr->is_navpoint) continue;
```

因此历史路径点和 internav 不参加当前语义轮廓的这套局部分类。当前函数后半段虽然仍保留了向 `internav_near_nodes_` 添加 `is_navpoint` 的分支，但由于前面已经 `continue`，该分支在当前代码路径中不可达，属于遗留兼容逻辑，不能把它理解成当前实际执行的 internav 筛选。

##### 3.2.4.5 source 决定 XY/三维空间门限

每个节点先得到自己的 `graph_range`：

```cpp
graph_range =
    node.source == DYNAMIC_LOCAL
        ? sensor_range
        : static_stitch_radius;
```

即：

```text
R(node) = sensor_range          ，DYNAMIC_LOCAL
R(node) = static_stitch_radius ，其他参与此流程的节点
```

当前默认参数为：

```yaml
sensor_range: 20.0
Graph/static_stitch_radius: 28.5
```

所以动态节点只在机器人 20 m 范围内参加局部维护，静态候选和静态全局节点可在 28.5 m 范围内参加拼接。语义查询窗口是 XY 各 `[-20 m, +20 m]` 的正方形，角点的径向距离为：

```text
sqrt(20^2 + 20^2) = 28.284 m
```

如果静态节点也使用 20 m 圆形门限，已经位于方窗四角内的可见静态墙角会被错误裁掉。28.5 m 用于覆盖该对角区域。初始化时还会用：

```text
semantic_local_window_radius * sqrt(2)
+ robot_collision_clearance
```

计算方窗实际可用径向范围；如果配置的 `static_stitch_radius` 更大，会被钳制到这个可用范围。

动态节点不扩展到方窗对角，是因为它只表示最新局部覆盖层，离开当前有效观测圆后不应继续作为稳定拓扑拼接依据。

函数先用 `norm_flat()` 做一次便宜的 XY 快速排除，再用三维距离做正式范围判断：

```text
d_xy  = sqrt(dx^2 + dy^2)
d_xyz = sqrt(dx^2 + dy^2 + dz^2)
```

因此，一个 XY 距离刚好接近门限、同时具有明显高度差的节点，可能通过第一层 XY 检查，但因 `d_xyz >= graph_range` 在正式判断中失败。

##### 3.2.4.6 扩展高度范围与严格高度范围

函数为同一节点计算两种空间资格：

```cpp
in_extend_match_range =
    abs(node.z - robot.z) < 1.5 * kTolerZ &&
    d_xyz < graph_range;

in_local_graph_range =
    abs(node.z - robot.z) < kTolerZ &&
    d_xyz < graph_range;
```

`in_extend_match_range` 较宽松，目的是让高度调整误差、地图边缘抖动或临时错层的旧节点仍有机会和当前轮廓重新匹配。`in_local_graph_range` 较严格，决定节点是否可以直接进入 near/wide 核心局部集合。

相关常量不是独立 YAML 参数，而是这样计算：

```text
kHeightVoxel = 2 * contour_grid_resolution
kTolerZ      = floor_height - kHeightVoxel
```

当前默认值：

```text
contour_grid_resolution = 0.4 m
floor_height            = 3.0 m
kHeightVoxel            = 0.8 m
kTolerZ                 = 2.2 m
1.5 * kTolerZ           = 3.3 m
```

因此按当前配置可近似理解为：

```text
高度差 < 2.2 m
    可进入严格局部范围

2.2 m <= 高度差 < 3.3 m
    只保留扩展匹配机会，通常进入 margin

高度差 >= 3.3 m
    本轮完全不参加局部匹配和维护
```

代码中 XY 快速判断相对 `odom_node_ptr_->position`，三维距离相对 `FARUtil::odom_pos`，高度判断相对 `FARUtil::robot_pos.z`；正常主循环中三者同步表示当前机器人位姿。

##### 3.2.4.7 地形门控采用“未知乐观、已知矛盾拒绝”

函数首先调用 `TerrainHeightOfPoint(node.position, terrain_matched, true)`。随后：

```cpp
terrain_neighbor_valid =
    !terrain_matched ||
    IsNavPointOnTerrainNeighbor(node.position, true);
```

含义是：

- 查不到 terrain-support：当前只知道地形未知，不因此删除或裁掉节点；
- 查到 terrain-support：节点脚底必须落在附近地形高度带内，否则视为已知高度矛盾。

外层门控为：

```cpp
in_extend_match_range &&
(!node_ptr->is_active || terrain_neighbor_valid)
```

因此 active 旧节点遇到明确地形矛盾时会被挡在本轮局部集合外；inactive 节点则先保留一次根据本帧新观测重新激活的机会。

进入严格 near/wide 前还会调用 `IsPointOnTerrain()`。其策略同样是：

```text
没有地形样本 -> 返回 true，按未知可行处理

存在地形样本 -> 要求
abs(node.z - terrain_height - vehicle_height) < kTolerZ
```

未知地形只是允许节点继续成为候选，不代表边已经安全；实际建边仍必须通过持久静态层、当前动态层、轮廓几何和地形路线检查。

##### 3.2.4.8 节点激活及 `extend_match_nodes_`

通过扩展范围和外层地形门控后，函数排除 `IsOutsideGoal()` 节点，再调用：

```cpp
if (IsActivateNavNode(node_ptr) || node_ptr->is_boundary)
    extend_match_nodes_.push_back(node_ptr);
```

`IsActivateNavNode()` 依次检查：

1. 节点已经 `is_active`；
2. 节点是否靠近本帧新点，若是则重新激活；
3. 对 odom/navpoint 一类自由节点，再检查是否贴近机器人、是否已连接 odom、是否所有已有邻居都 active。

由于当前遍历前已跳过 `is_navpoint`，普通轮廓节点最常见的重新激活方式是“靠近本帧新观测点”。boundary 不依赖普通激活结果，直接保留扩展匹配资格。

`GetExtendLocalNode()` 返回的正是 `extend_match_nodes_`，外部却把它赋给名为 `near_nav_graph_` 的变量。因此后续 `MatchContourWithNavGraph()` 接收的是扩展匹配集合，不是严格的 `near_nav_nodes_`。匹配完成后，`UpdateNavGraph()` 也会遍历这个集合复检已有节点。

##### 3.2.4.9 `wide_near_nodes_` 与 `near_nav_nodes_`

直接进入 wide 的条件是：

```cpp
in_local_graph_range && IsPointOnTerrain(node.position)
```

`wide_near_nodes_` 不要求节点已经 active，所以可以保留位置、高度、地形合理但当前未激活的局部拓扑上下文。

在此基础上，只有下面两类节点进入核心 `near_nav_nodes_`：

```cpp
node_ptr->is_active || node_ptr->is_boundary
```

所以 near 的完整语义是：

```text
位于 source 对应半径内
+ 位于严格高度范围内
+ 地形未知或与已知地形相符
+ active 或 boundary
```

`near_nav_nodes_` 是随后大规模局部重连的主要集合，包括：

- 与局部范围外既有连接重新验证；
- near 节点无序 pair 的完整连接验证；
- contour 关系记录、删除和稳定投票；
- 静态边的当前动态 mask；
- `is_covered`、`is_frontier` 重新计算。

当前版本的 `UpdateOdomConnections()` 已不只遍历 `wide_near_nodes_`，而是直接扫描全局正式节点、静态候选和当前动态节点，再对每个候选执行完整验证。因此不要把 wide 理解成“odom 唯一允许连接的集合”；它当前主要是宽松局部分类和拓扑上下文缓存。

##### 3.2.4.10 `margin_near_nodes_` 的真实含义与提升

节点已经通过扩展范围，但没有同时通过：

```text
in_local_graph_range && IsPointOnTerrain(...)
```

并且节点是 active 或 boundary 时，进入 `margin_near_nodes_`。

因此 margin 常见原因是：

- 高度差位于严格容差与 1.5 倍扩展容差之间；
- XY 尚在门限内，但高度使三维距离跨过门限；
- 当前查到的地形高度与节点高度不符。

它的作用是避免节点因一次高度调整、局部方窗裁剪或 terrain-support 抖动就立刻退出所有匹配流程。当前轮廓匹配结束后，`UpdateNearNodesWithMatchedMarginNodes()` 检查：

```cpp
if (node_ptr->is_contour_match) {
    wide_near_nodes_.push_back(node_ptr);
    near_nav_nodes_.push_back(node_ptr);
}
```

也就是当前轮廓重新确认了该节点身份时，用更强的当帧几何证据把它提升进 near/wide，继续参加本轮连边维护。

##### 3.2.4.11 用 odom 的一跳、二跳邻居补全 wide 拓扑

完成按空间分类后，函数还会遍历 odom 当前连接：

```text
odom
  └─ cnode：一跳邻居
       └─ c2node：二跳邻居
```

规则是：

1. 一跳邻居不是 `OutsideGoal`；
2. 一跳邻居到 odom 的 XY 距离不超过 `static_stitch_radius`；
3. 将一跳邻居加入 `wide_near_nodes_`；
4. 再把它所有非 `OutsideGoal` 的二跳邻居加入 wide。

这一步按拓扑关系补偿空间裁剪，避免正在使用的局部拓扑链在边界处被截断。二跳节点在当前代码中没有再次执行距离和地形检查，所以极端情况下它可以位于 28.5 m 之外；它只是被补入 wide 上下文，不会因此绕过后续边碰撞验证。

##### 3.2.4.12 默认参数下的分类示例

假设机器人在 `(0, 0, 0)`，当前默认参数为：

```text
DYNAMIC_LOCAL 半径 = 20.0 m
静态拼接半径       = 28.5 m
严格高度容差       = 2.2 m
扩展高度容差       = 3.3 m
```

| 节点示例 | 判断结果 |
|---|---|
| active 静态墙角 `(19, 19, 0.5)` | XY 距离 26.87 m，小于 28.5 m；高度/地形正常时进入 extend、wide、near |
| active 动态角点 `(19, 19, 0.5)` | XY 距离 26.87 m，大于动态 20 m 门限，本轮跳过 |
| active 静态节点 `(10, 0, 2.7)` | 高度差位于 2.2～3.3 m；可进入 extend 和 margin，不直接进入 near/wide；当前轮廓匹配成功后可提升 |
| 持久静态节点 `(35, 0, 0.5)` | 不进入本轮四个普通空间工作集，但节点仍留在全局图中，不会仅因超距删除 |

整个分类可以压缩成下面的伪代码：

```text
清空本轮四个局部工作集
按 source 范围预处理旧 odom 局部边

for node in 正式全局图 + 静态候选 + 当前动态图:
    跳过 PATH_HISTORY 和 navpoint

    range = dynamic ? sensor_range : static_stitch_radius
    若 XY 距离超限：跳过

    extended = 扩展高度通过 && 三维距离小于 range
    strict   = 严格高度通过 && 三维距离小于 range

    若不在 extended：跳过
    若 active 节点存在已知地形矛盾：跳过
    若位于 goal 外侧：跳过

    尝试激活节点
    若 active 或 boundary：加入 extend

    若 strict 且地形有效/未知:
        加入 wide
        若 active 或 boundary：加入 near
    否则若 active 或 boundary:
        加入 margin

把 odom 一跳和二跳拓扑邻居补入 wide
匹配结束后，把匹配成功的 margin 节点提升进 near/wide
```

一句话记忆：`extend` 负责“还能不能匹配”，`wide` 负责“保留局部拓扑上下文”，`near` 负责“本轮真正连边和维护”，`margin` 负责“严格高度/地形暂时不满足，但先等当前轮廓再确认”。

#### 3.2.5 当前 CTNode 与历史 NavNode 匹配

详见第 6 节。匹配成功复用身份；未匹配且属于有效角点/柱体的 CTNode 输出到 `new_ctnodes_`。

#### 3.2.6 `FinalizeDynamicGraphUpdate()`：结算旧动态身份

`FinalizeDynamicGraphUpdate()` 在匹配之后立即执行：

- 本帧未匹配的动态节点立即彻底删除；
- 匹配成功的动态节点用 EMA 更新位置；
- 强制保持非 finalized，清除过去快照的 RANSAC 样本，再用当前更新结果各放入一个新的位置/方向种子样本。

##### 3.2.6.1 它在当前快照中的准确位置

完整顺序是：

```text
BeginSemanticGraphUpdate()
    旧动态节点全部重置为“本帧尚未观测”
                    ↓
UpdateGlobalNearNodes()
    选出允许参加当前轮廓匹配的旧节点
                    ↓
MatchContourWithNavGraph()
    当前动态 CTNode 与旧 DYNAMIC_LOCAL NavNode 一对一匹配
                    ↓
FinalizeDynamicGraphUpdate()
    ├─ 旧节点匹配失败：立即彻底删除
    └─ 旧节点匹配成功：沿用 id，更新位置和轮廓方向
                    ↓
ExtractGraphNodes(new_ctnodes_)
    本帧未匹配到旧身份的动态 CTNode 创建为新 DYNAMIC_LOCAL NavNode
                    ↓
UpdateNavGraph()
    加入新节点并重建当前局部图边
```

因此 `FinalizeDynamicGraphUpdate()` 只结算已经存在于 `dynamicLocalGraphNodes_` 中的旧动态身份。当前帧全新的动态轮廓点此时仍是 CTNode，要到后面的 `ExtractGraphNodes()` 才创建 NavNode。该函数也不会结束整个语义事务，`semantic_update_in_progress_` 要到 `CommitSemanticGraphUpdate()` 完成后才清零。

##### 3.2.6.2 Begin、匹配和 Finalize 组成一次动态身份确认

新快照开始时，`BeginSemanticGraphUpdate()` 先对每个旧动态节点执行：

```cpp
node_ptr->observed_in_semantic_snapshot = false;
node_ptr->is_contour_match = false;
node_ptr->ctnode = NULL;
```

这不是立即删除节点，而是把它置成：

```text
身份暂时保留，等待当前快照重新证明它仍然存在。
```

`MatchContourWithNavGraph()` 只允许动态 CTNode 与 `DYNAMIC_LOCAL` NavNode 匹配，静态和动态身份不能互相复用。匹配成功时 `MatchCTNodeWithNavNode()` 设置：

```cpp
node_ptr->ctnode = ctnode_ptr;
node_ptr->is_contour_match = true;
node_ptr->observed_in_semantic_snapshot = true;
```

所以 Finalize 判断一个旧动态节点本帧是否继续存活，需要三项同时成立：

```text
observed_in_semantic_snapshot == true
is_contour_match              == true
ctnode                        != nullptr
```

三项分别表达“生命周期上已观测”“匹配算法已确认身份”“存在当前几何可用于更新”。当前正常匹配会同时设置它们；Finalize 仍逐项检查，是为了避免部分状态不一致时保留一个没有当前几何依据的动态节点。

##### 3.2.6.3 未匹配动态节点为什么不累计 miss，而是立即删除

Finalize 先复制容器：

```cpp
const NodePtrStack dynamic_copy = dynamicLocalGraphNodes_;
```

因为 `RemoveNodeFromGraph()` 会修改原 `dynamicLocalGraphNodes_`，遍历副本可以避免边遍历边删除造成迭代失效。

随后执行：

```cpp
if (!node_ptr->observed_in_semantic_snapshot ||
    !node_ptr->is_contour_match ||
    !node_ptr->ctnode) {
    RemoveNodeFromGraph(node_ptr);
    continue;
}
```

动态节点表达的是“最新接受语义快照中的动态轮廓路由顶点”，不是跨时间保存的地图结构。若某个行人或车辆角点在下一张已接受快照中没有重新匹配，就不能把它继续当成当前障碍图节点，否则会在物体走后留下幽灵角点和幽灵边。因此它没有静态节点的连续多帧删除门限，而是在一次有效快照中未匹配就立即删除。

`RemoveNodeFromGraph()` 是完整删除，会清理：

- `connect_nodes` 和 poly/visibility 边；
- contour-follow 连接；
- trajectory 连接；
- 节点 ID 索引；
- near/wide/margin 等内部工作集；
- out-contour 记录；
- `dynamicLocalGraphNodes_` 中的节点本体。

##### 3.2.6.4 匹配成功后为什么使用 EMA 更新位置

旧动态身份匹配到当前 CTNode 后，位置按下面的指数移动平均更新：

```cpp
node.position =
    node.position * (1.0f - alpha) +
    node.ctnode.position * alpha;
```

默认配置：

```yaml
Graph/dynamic_position_alpha: 0.65
```

即：

```text
P_new = 0.35 * P_old + 0.65 * P_current_ctnode
```

例如旧位置为 `x=10.0 m`，本帧轮廓位置为 `x=11.0 m`：

```text
x_new = 10.0 * 0.35 + 11.0 * 0.65 = 10.65 m
```

这样做是为了抑制 Semantic OctoMap 体素量化、轮廓栅格、`findContours()` 和 `approxPolyDP()` 造成的角点跳格，同时仍让当前帧占主要权重以跟随运动物体。参数加载后会钳制到 `[0, 1]`：

| `dynamic_position_alpha` | 效果 |
|---:|---|
| `1.0` | 完全使用当前 CTNode，响应最快但最容易抖动 |
| `0.65` | 当前帧主导，同时保留少量历史平滑 |
| 接近 `0.0` | 跟随明显滞后 |
| `0.0` | 位置保持旧值，通常不适合运动物体 |

##### 3.2.6.5 EMA 只平滑路由顶点，不平滑碰撞事实

这里更新的是 `NavNode::position`，即用于构造局部图的路由顶点。动态边的碰撞判定继续使用最新语义快照产生的精确 `current_dynamic_obs_cloud_`。

因此，即使 EMA 后的图节点相对真实动态物体稍有滞后：

- 最新动态占用仍能立即设置静态边的 `dynamic_blocked`；
- 新出现的动态体素不会被 EMA 隐藏；
- 位置平滑只改善图几何稳定性，不替代动态碰撞安全检查。

匹配成功后，节点类型和表面方向不做跨帧 EMA，而是直接采用当前轮廓：

```cpp
node_ptr->free_direct = node_ptr->ctnode->free_direct;
node_ptr->surf_dirs   = node_ptr->ctnode->surf_dirs;
```

动态物体可能转向或改变当前可绕行侧，继续保留旧方向可能把边投影到错误的一侧。因此当前策略是：位置用 EMA 抑制像素抖动，角点类型和表面方向立即跟随当前轮廓，碰撞则服从最新动态点云。

##### 3.2.6.6 RANSAC 历史和 finalized 原本用于静态角点稳定化

普通未定型节点可以跨快照积累：

| 容器 | 保存内容 |
|---|---|
| `pos_filter_vec` | 多次匹配得到的历史位置 |
| `surf_dirs_vec` | 多次匹配得到的两侧轮廓表面方向 |

`Graph/filter_pool_size` 默认是 12，表示每个滑动窗口最多保存 12 组样本。

位置 RANSAC 会对每个历史位置统计距离小于 `filter_pos_margin` 的同类样本，选内点数量最多的一簇并取平均。当前 `filter_pos_margin` 实际赋值为 `robot_collision_clearance`，默认约 0.45 m。方向 RANSAC 以类似方式寻找相互一致的表面方向簇，并在柱体样本占多数时把节点分类为 `PILLAR`。

只有位置和方向两边的内点数都满足：

```cpp
inlier_size > node_finalize_thred
```

节点才会设置 `is_finalized=true`。当前 `Graph/node_finalize_thred=6`，代码使用严格大于，所以实际上至少需要 7 个一致内点。

对静态节点，finalized 表示这个固定物理角点的位置和表面方向已经得到多帧一致支持。此后 `UpdateNodePosition()` 和 `UpdateNodeSurfDirs()` 看到 finalized 会直接返回，不再继续改变已经稳定的几何身份。

##### 3.2.6.7 为什么动态节点必须丢弃旧 RANSAC 样本

RANSAC 的假设是多帧采样来自同一个不动的真实角点，离群样本只是检测噪声。但动态物体的位置变化不是噪声。

例如同一动态身份连续出现在：

```text
x = 2 m -> 3 m -> 4 m -> 5 m
```

如果把这些值持续压入静态 RANSAC 窗口，旧位置簇会拖住当前路由点，可能在物体后方留下滞后的幽灵角点。若物体短暂停留并积累出足够多内点，还可能错误进入 `is_finalized=true`；此后通用位置/方向更新函数会提前返回，动态节点就会冻结在旧位置。

所以每次动态节点匹配成功，Finalize 都执行：

```cpp
node_ptr->is_finalized = false;

node_ptr->pos_filter_vec.clear();
node_ptr->pos_filter_vec.push_back(node_ptr->position);

node_ptr->surf_dirs_vec.clear();
node_ptr->surf_dirs_vec.push_back(node_ptr->surf_dirs);
```

准确含义不是“把过滤器清空后不再使用”，而是：

1. 取消任何可能遗留的 finalized 状态；
2. 丢弃过去快照的位置 RANSAC 样本；
3. 以当前 EMA 后位置作为新的唯一位置种子；
4. 丢弃过去快照的方向 RANSAC 样本；
5. 以当前轮廓方向作为新的唯一方向种子。

保留一个种子样本可以让共用的节点过滤数据结构始终处于有效状态，避免后续通用函数遇到空输入，同时又不会继承运动轨迹历史。因为每个匹配快照都会再次重置，动态节点无法积累到静态定型所需的 7 个一致样本。

##### 3.2.6.8 非 finalized 不等于不能规划

`is_finalized` 表示的是“位置和表面方向是否已经通过多帧静态稳定性确认”，不等同于 `is_active`，也不直接等同于节点或边能否被搜索。

动态节点即使始终 `is_finalized=false`，只要它：

- 属于当前 `DYNAMIC_LOCAL`；
- 当前快照已观测并匹配；
- 具有 search-eligible 的活动边；
- 相关边没有被当前碰撞状态阻塞；

仍然可以进入当前搜索图。强制非 finalized 是为了禁止它被当作已经定型的持久静态角点，而不是为了停用它。

静态和动态节点的处理差异可以归纳为：

| 行为 | 静态节点 | 动态节点 |
|---|---|---|
| 一次未匹配 | 结合观测范围、明确 free 和连续 miss 证据处理 | 当前接受快照内立即删除 |
| 位置更新 | 跨帧 RANSAC | 当前帧主导的 EMA |
| 表面方向 | 跨帧 RANSAC | 直接采用当前轮廓 |
| `is_finalized` | 可以变为 true | 每帧强制 false |
| 历史过滤样本 | 保留到滑动窗口上限 | 每次只留下当前种子 |
| 生命周期 | 候选可晋升为 `STATIC_GLOBAL` | 永远停留在 `DYNAMIC_LOCAL` |
| 碰撞依据 | `persistent_static_obs_cloud_` | 最新动态障碍点云 |

一句话概括：静态节点通过多帧历史证明“它没有动”，动态节点则必须在每个新快照里重新证明“它现在还在这里”，所以旧 RANSAC 历史不能跨动态快照继承。

#### 3.2.7 新节点生成、全图连边、生命周期提交

```text
ExtractGraphNodes(new_ctnodes_)
    -> UpdateNavGraph(new_nodes_)
    -> CommitSemanticGraphUpdate(evidence_query)
```

这三个函数不是三个含义相近的“更新步骤”，而是三个职责明确的阶段：

| 阶段 | 输入 | 核心职责 | 主要输出/副作用 |
|---|---|---|---|
| `ExtractGraphNodes()` | 当前轮廓中未匹配旧身份的 `new_ctnodes_` | 判断哪些 CTNode 有资格成为新 NavNode，并初始化节点身份 | 内部 `new_nodes_`；尚未加入三个图容器 |
| `UpdateNavGraph()` | `new_nodes_`、freeze 标志、`clear_node` 输出引用 | 把本帧新节点与已有局部图合并，验证 contour/visibility 边，更新动态 mask，最后重建 odom 边 | 当前快照完整但尚未做最终静态生命周期提交的图 |
| `CommitSemanticGraphUpdate()` | 节点位置到当前静态证据的查询函数 | 晋升/删除静态节点，原子替换旧角点与旧 contour 边，结束事务 | 可发布给规划器的最终一致图 |

##### 3.2.7.1 `ExtractGraphNodes()`：把未匹配 CTNode 变成新 NavNode 候选

这里的输入 `new_ctnodes_` 不是全部当前轮廓顶点。`MatchContourWithNavGraph()` 已经先做过两层筛选：

1. 能匹配历史 NavNode 的 CTNode 已复用旧身份，不再进入 `new_ctnodes_`；
2. 未匹配点还必须已经完成有效角点/柱体分类，直墙中间点和 `free_direct=UNKNOW` 的点不会输出。

因此 `ExtractGraphNodes()` 接收的是“当前轮廓确认存在、没有旧身份可复用、几何上可能值得建图”的 CTNode。函数开始先清空内部结果：

```cpp
new_nodes_.clear();
if (new_ctnodes.empty()) return false;
```

这保证 `new_nodes_` 只属于当前这一张语义快照，不会混入上一轮尚未消费的候选。

**第一道门：按 source 检查创建半径。**

距离使用机器人到 CTNode 的 XY 平面距离：

```text
robot_distance = norm_flat(ctnode.position - robot_pos)
```

不同 source 的条件为：

```cpp
if (ctnode.source == DYNAMIC_LOCAL) {
    robot_distance <= sensor_range;
} else {
    robot_distance <= static_update_radius &&
    robot_distance <= static_stitch_radius;
}
```

当前默认值：

```text
动态新节点：<= 20.0 m
静态新节点：同时 <= 28.5 m 和 <= 28.5 m
```

这里静态使用两个门限的交集，实际有效创建半径是：

```text
min(static_update_radius, static_stitch_radius)
```

初始化时 `static_stitch_radius` 会被保证不小于 `static_update_radius`，所以正常配置下真正限制静态新节点创建的是 `static_update_radius`。动态节点仍使用 20 m 圆形范围，因此语义方窗四角的动态轮廓不会创建 NavNode；静态 28.5 m 半径则覆盖 20 m 半边长方窗的对角。

**第二道门：`IsAValidNewNode()` 检查语义来源和地形。**

函数先计算：

```cpp
is_near_new = FARUtil::IsPointNearNewPoints(ctnode.position, true);
```

旧 FAR 流程依赖“附近是否出现新点”判断是否值得建节点；当前 semantic-only 流程已经由当前快照轮廓直接证明节点存在，所以满足下列任一条件即可继续地形检查：

```text
当前静态 CTNode
或当前动态 CTNode
或 is_contour_necessary
或靠近 changed/new points
```

因为正常输入就是当前静态或动态 CTNode，所以 `is_near_new` 在当前主路径中通常不再决定“能不能创建”，但仍决定新节点是否允许成为 frontier：

```cpp
if (!is_near_new)
    new_node_ptr->is_block_frontier = true;
```

即，没有附近新变化证据的有效语义角点仍可进入图，但被阻止作为探索 frontier，避免把长期不变或非新变化位置误当作新边界目标。

地形策略仍是“未知乐观、已知矛盾拒绝”：

```text
TerrainHeightOfPoint() 没匹配到地面
    -> UNKNOWN，不单独拒绝节点

匹配到地面
    -> IsNavPointOnTerrainNeighbor(..., false) 必须通过

最后 IsPointOnTerrain()
    -> 节点高度必须与已知 terrain_height + vehicle_height 相符
```

如果一个 `is_contour_necessary` 节点地形失败，代码会撤销它的 necessary 标志，但本次仍不创建 NavNode。未知地形允许进入候选不代表边已安全；节点加入图后，每条 incident edge 还要独立通过静态、动态、Polygon 和地形路线验证。

**第三步：初始化 NavNode。**

`CreateNewNavNodeFromContour()` 先调用通用 `CreateNavNodeFromPoint()`：

- 创建新的 `NavNode` 对象；
- 设置初始位置并把它放入 `pos_filter_vec`，作为第一个位置样本；
- 初始化所有连接、投票、边状态和规划字段为空/默认值；
- `is_active=true`、`is_finalized=false`；
- 分配全局唯一递增 id。

随后继承当前 CTNode 的语义和轮廓字段：

```text
动态 CTNode -> source = DYNAMIC_LOCAL
静态 CTNode -> source = STATIC_CANDIDATE

observed_in_semantic_snapshot = true
is_contour_match              = true
ctnode                        = 当前 CTNode
position                      = CTNode 位置
free_direct                   = CTNode 分类
surf_dirs                     = 当前两侧表面方向
```

`UpdateNodeSurfDirs()` 会把当前方向作为第一个方向过滤样本。静态且位于当前轮廓画布裁剪带的点还会设置：

```cpp
is_transient_contour_endpoint = true;
```

这种节点只表达“墙在当前画布边缘被截断”，不能晋升成持久静态角点。

此时新 NavNode 只是放入函数内部的 `new_nodes_`，尚未进入 `globalGraphNodes_`、`staticCandidateGraphNodes_` 或 `dynamicLocalGraphNodes_`。函数返回值只表示是否产生了至少一个节点：

```text
true  -> 调用方通过 GetNewNodes() 取出 new_nodes_
false -> 本帧没有可加入图的新节点
```

在主循环中，只有 `is_stop_update_ == false` 才会调用 `ExtractGraphNodes()`；冻结图更新时不会创建本帧新 NavNode。

##### 3.2.7.2 `UpdateNavGraph()`：把本帧节点和边组装成完整快照图

函数输入：

```cpp
UpdateNavGraph(
    const NodePtrStack& new_nodes,
    const bool& is_freeze_vgraph,
    NodePtrStack& clear_node)
```

三个参数分别是本帧准备加入的节点、是否冻结普通图结构更新、以及供外部可视化本轮清理节点的输出集合。函数不是简单依次给新节点找最近邻，而是先完成全部局部候选验证，再统一选择和提交边，避免结果依赖 pair 遍历顺序。

**步骤 1：清空本轮输出和诊断。**

```cpp
clear_node.clear();
contour_edge_diagnostics_.clear();
```

`clear_node` 保存本轮被判为清理对象的共享指针，主循环稍后用橙色显示；真正从图容器删除由 `ClearMergedNodesInGraph()` 完成。

**步骤 2：非冻结模式下复检已有扩展局部节点。**

函数遍历 `extend_match_nodes_`：

- odom/goal 查询端点直接跳过；
- `STATIC_CANDIDATE/STATIC_GLOBAL` 如果本帧已匹配，调用 `ReEvaluateCorner()` 积累位置和方向 RANSAC；未匹配静态节点不在这里按旧 dumper 删除，而是留给 Commit 的静态证据状态机；
- `DYNAMIC_LOCAL` 跳过，因为它已经由 `FinalizeDynamicGraphUpdate()` 按当前快照结算；
- `PATH_HISTORY/navpoint` 跳过；
- 旧兼容来源节点复检失败时累计 `clear_dumper_count`，达到门限后标成 merged 并放入 `clear_node`；复检成功则回收一次 dumper。

在非静态环境且存在 internav 时，还会对 trajectory 连接做地形投票复检。这是旧 internav 兼容路径；当前 semantic graph 已在前面排除历史 navpoint，所以正常语义主路径通常不会靠它生成新拓扑。

**步骤 3：清掉 merged 节点，并提升本帧已匹配的 margin 节点。**

```cpp
ClearMergedNodesInGraph();
UpdateNearNodesWithMatchedMarginNodes(
    margin_near_nodes_, near_nav_nodes_, wide_near_nodes_);
```

前者对称清理 merged 节点的普通边、contour 边、trajectory 边、ID 索引和所在图容器。后者把当前轮廓重新确认的 margin 节点补入 near/wide，使其可以继续参加本轮连边。

这两步位于 freeze 判断之外，所以即使 `is_freeze_vgraph=true` 仍会执行。

**步骤 4：非冻结模式下把新节点正式加入对应生命周期容器。**

`AddNodeToGraph()` 按 source 分流：

```text
DYNAMIC_LOCAL    -> dynamicLocalGraphNodes_
STATIC_CANDIDATE -> staticCandidateGraphNodes_
其他正式来源     -> globalGraphNodes_
PATH_HISTORY     -> 拒绝加入语义图
```

新节点同时加入 `near_nav_nodes_` 并设置 `is_near_nodes=true`。然后重新调用 `MatchCTNodeWithNavNode()`，把新创建 NavNode 的 id 反向写回 CTNode：

```text
ctnode.is_global_match = true
ctnode.nav_node_id     = new_node.id
```

这样本帧新建身份和后续 contour 邻接查询使用同一套 CT-Nav 映射。

**步骤 5：维护 out-of-range contour 的历史关系。**

对于 `out_contour_nodes_`，函数尝试从当前 `near_nav_nodes_` 找对应节点：

- 当前匹配到：给该 contour pair 记录正投票；
- 以前记录过但当前不再匹配：给旧 pair 记录负投票。

这使超出严格局部范围的旧轮廓关系可以逐步回连或衰减，而不是在局部窗口移动时突然断开。

**步骤 6：复检 near 节点通向局部范围外的历史 visibility 边。**

每个 near 节点复制自己的旧 `connect_nodes`，对另一端不在 near、不是 odom、不是 `OutsideGoal`、且不是 contour 身份的边调用 `UpdateGraphEdge()`。验证失败的范围外节点进入 `outside_break_nodes`，后面会再与全部 near 节点尝试重连，避免仅因原锚点失效就丢掉本可由其他局部节点接续的外部图。

与此同时，near 节点和 out-contour 节点若都匹配当前轮廓，会根据本帧是否仍为 contour 邻接记录正/负 contour vote。

**步骤 7：先收集全部 near pair 的验证结果，不立即改邻接。**

函数枚举所有非 odom 的无序 pair：

```text
(near[0], near[1])
(near[0], near[2])
...
```

每个 pair 都进入 `all_pairs`；通过 `IsValidConnect()` 的进入 `valid_pairs`。验证内容包括：

- 两端是否可按凸角/柱体规则连接；
- 连线是否位于两端允许的自由方向；
- 当前 Polygon、持久静态点云和必要时当前动态点云是否阻挡；
- 中点/路线地形是否合理；
- visibility 投票是否成熟；
- 若 `is_check_contour=true`，同时记录或撤销本帧 contour 邻接投票。

这一段是 `O(N^2)` 的局部 pair 枚举，因此前面的 `UpdateGlobalNearNodes()` 必须先把全局图裁成 near 工作集；否则全局节点数量增长后，连边验证成本会平方增长。

静态—静态 pair 的结构验证故意不包含动态障碍：动态物体只应临时 mask 静态边，不能删除静态边身份或清空静态投票。只要 pair 包含 `DYNAMIC_LOCAL`，当帧结构验证就包含动态层；动态边投票队列长度为 1，响应当前快照。

这里传入 `apply_direction_filter=false`，先获得完整的几何有效候选集，稍后再基于全体候选统一做同方向稀疏化。这样不会出现“前面先加了哪条边，后面的判断结果就随之改变”的遍历顺序依赖。

**步骤 8：先提交 contour 身份和真实 contour route。**

所有 pair 的 contour 投票已经收集完后，对每个 near 节点调用 `TopTwoContourConnector()`：

- 当前可靠轮廓明确相邻的 pair 是当前权威关系，不必被旧投票拖住；
- 当前窗口之外的历史 contour 关系继续使用 top-two 投票规则；
- 候选边会把两端向障碍自由侧投影；
- 投影 route 必须通过静态点云、其他 Polygon、动态层和地形检查；
- 通过后保存 `validation_mode=CONTOUR_FOLLOW`、`route_start/end`、`route_cost` 和双向状态。

contour 身份先于 visibility 稀疏化提交，是因为同一 pair 可以同时具有 contour 和 visibility 两种身份；后面的 visibility 删除不能误删仍有效的 contour 邻接。

**步骤 9：对 visibility 候选做全局一致的同方向稀疏化并统一提交。**

对每条 `valid_pair`，从两个端点分别检查同一方向上是否存在更近的有效候选。如果两端都没有更近替代，或者该 pair 本身是 contour/boundary 关系，就加入 `selected_pairs`。

随后才遍历 `all_pairs` 统一执行 `ApplyValidatedGraphEdge()`：

```text
pair 在 selected_pairs
    -> AddPolyEdge + AddEdge

pair 未被选择或结构无效
    -> 删除 visibility 身份
       若仍有 contour/trajectory 身份，connect_nodes 继续保留
```

因此局部 visibility 结果由完整候选集合一次性决定，不依赖节点遍历顺序。

对静态—静态 visibility 边，结构有效后单独根据最新动态层设置 `dynamic_blocked`；静态身份和历史投票保持不变。对动态参与的边，当前动态几何失败会直接使本帧结构验证失败。

**步骤 10：尝试恢复范围外断点。**

前面收集的 `outside_break_nodes` 会再次与所有 near 节点调用 `UpdateGraphEdge()`，尝试换一个当前局部锚点接回历史图。每个候选仍需通过正常几何、投票和地形检查，并不是无条件 stitch。

**步骤 11：对所有静态边统一应用最新静态 route 检查和动态 mask。**

函数扫描 `globalGraphNodes_ + staticCandidateGraphNodes_` 中的静态—静态边。若一条 contour-follow 边保存了 clearance route，并且该 route 穿过当前可靠轮廓窗口，则重新用持久静态碰撞层检查：

```text
route 被静态层挡住 -> static_valid=false，边立即不可搜索
route 静态安全       -> static_valid=true
```

随后对所有静态边计算当前动态阻挡：

```text
contour-follow -> 检查保存的 route_start/end
普通 visibility -> 检查两节点间当前动态走廊
```

结果只写 `dynamic_blocked`。这一步放在 contour connector 之后，保证本帧刚新增的 contour 边也得到与旧静态边一致的动态 mask。

**步骤 12：更新 near 节点的 covered/frontier 状态并输出诊断。**

每个 near 节点重新计算：

- `is_covered`：节点是否已被 odom/internav 可见性和周边图充分覆盖；
- `is_frontier`：当前是否属于可用于探索的有效凸角边界；
- contour-follow 活动边数及拒绝原因统计。

诊断会区分当前不相邻、裁剪边、静态点云阻挡、自身/其他 Polygon 阻挡、动态阻挡、地形失败、投票未成熟和投影失败。

**步骤 13：无论是否 freeze，最后重建 odom/start 查询边。**

```cpp
UpdateOdomConnections();
```

该函数先删除 odom 的全部旧邻接，再扫描正式全局节点、静态候选和当前动态节点。候选必须是可用于 start 的凸角/柱体/boundary，并至少已有一条 active、非 odom/goal 的可搜索图边，避免把机器人连到永久孤点。每条 start 边都重新执行轮廓投影、持久静态层、当前动态层和地形验证。

把 odom 重建放在最后，可以保证机器人连接看到的是“本帧新节点已经加入、contour/visibility 已完成、动态 mask 已应用”的同一张图。

`is_freeze_vgraph=true` 时，函数会跳过旧节点大规模复检、新节点加入、out-contour 更新和 near pair 重连，但仍会：

- 清理已经标记 merged 的节点；
- 把匹配成功的 margin 节点补进局部工作集；
- 重建 odom/start 边。

而且外部仍会继续调用 `CommitSemanticGraphUpdate()`，所以 freeze 不能理解成“所有节点和生命周期完全不变”。

##### 3.2.7.3 `CommitSemanticGraphUpdate()`：提交静态生命周期和拓扑替换事务

`UpdateNavGraph()` 结束时，本帧节点、候选边、实际 route 和动态 mask 已经齐全，但静态候选是否晋升、旧静态节点是否有足够删除证据、旧角点/旧 contour 边是否有安全替代结构仍未最终提交。Commit 专门完成这些跨快照决策。

调用方传入：

```cpp
[this](const Point3D& point) {
    return map_handler_.QueryStaticNodeEvidence(point);
}
```

即 Commit 不自行解释 OctoMap，而是通过 `evidence_query` 查询某个旧静态节点位置在当前语义地图中的物理证据。

**步骤 1：为每个静态候选/全局节点确定当前证据。**

遍历集合：

```text
globalGraphNodes_ + staticCandidateGraphNodes_
```

但只处理 source 为 `STATIC_CANDIDATE` 或 `STATIC_GLOBAL` 的节点。证据选择为：

```cpp
node.observed_in_semantic_snapshot
    ? STATIC_OCCUPIED
    : evidence_query(node.position);
```

三种结果含义：

| `StaticNodeEvidence` | 含义 | 是否能累计删除 miss |
|---|---|---|
| `STATIC_OCCUPIED` | 本帧轮廓已匹配，或当前位置附近仍有明确静态障碍 | 否 |
| `EXPLICIT_FREE` | 当前方窗内的语义树明确证明旧障碍位置已变为 free/terrain support | 是 |
| `UNKNOWN` | 未观测、遮挡、方窗外、弱语义、动态占用或无可靠地图证据 | 否 |

`QueryStaticNodeEvidence()` 使用与语义提取一致的轴对齐方窗；位于方窗外直接返回 `UNKNOWN`，不会因为当前局部地图没包含该节点就把它当成 free。

**步骤 2：计算候选晋升所需的两个额外 gate。**

除了语义观测帧数，默认还要求：

```text
finalization_ready:
    static_promotion_requires_finalized=false
    或 node.is_finalized=true

edge_ready:
    static_promotion_requires_active_edge=false
    或至少存在一条 active、可搜索、非 odom/goal incident edge
```

只连接 odom 或 goal 不能证明节点属于可复用静态拓扑，因为这两类查询边每次都会重建。

当前 `default.yaml` 的 `static_confirm_frames=3`，但默认又要求 finalized；`node_finalize_thred=6` 使用严格 `>`，所以普通静态候选往往至少积累 7 个一致位置/方向样本后才能晋升。使用 `ssmi_bag_graph_test.launch` 时，`ssmi_bag.yaml` 会把 `static_confirm_frames` 覆盖为 5，但 finalized gate 通常仍是更慢的约束。

**步骤 3：`AdvanceStaticNodeLifecycle()` 推进静态状态机。**

普通静态节点规则如下：

| 当前情况 | 计数变化 | 动作 |
|---|---|---|
| 本帧 observed | `static_missed_count=0`；`static_seen_count+1`，最多到 confirm 门限 | 候选达到 seen 门限且两个 promotion gate 都通过时晋升 |
| 未 observed、位于 `static_update_radius` 内、证据为 `EXPLICIT_FREE` | `static_missed_count+1` | 达到 `static_remove_frames` 后加入待删除集合 |
| 未 observed、位于 update radius 内、证据为 `UNKNOWN/STATIC_OCCUPIED` | `static_missed_count=0` | 保留，不把遮挡或未知当删除证据 |
| 未 observed 的候选越过 `static_stitch_radius` | 无需等待 free miss | 删除未确认候选，防止局部候选无限滞留 |
| 已确认 `STATIC_GLOBAL` 离开局部范围 | 不因距离累计 miss | 保留全局静态知识 |

普通候选的 `static_seen_count` 是“累计的有效 observed 次数并在门限处封顶”，一次未观测不会直接把 seen 清零；但删除用的 `static_missed_count` 要求连续明确 `EXPLICIT_FREE`，因为中间一次 `UNKNOWN` 或 `STATIC_OCCUPIED` 就会清零。这两个计数不要理解成同一种连续帧投票。

裁剪端点 `is_transient_contour_endpoint=true` 使用独立规则：

- 本帧仍 observed：seen/missed 均保持 0，不允许晋升；
- 本帧未 observed：立即 REMOVE。

它只代表本帧画布切出来的临时墙端，不能污染持久静态图。

达到普通删除条件的节点此时只放入 `remove_nodes`，暂不立即从图删除；达到晋升条件的节点放入 `promote_nodes`，并在生命周期函数中把 source 改为 `STATIC_GLOBAL`。

**步骤 4：先提交成熟静态候选晋升。**

每个 `promote_node` 从 `staticCandidateGraphNodes_` 移到 `globalGraphNodes_`。随后按当前边身份重标 incident edge source：

```text
属于 contour_connects             -> STATIC_CONTOUR
另一端也是 STATIC_GLOBAL           -> STATIC_VISIBILITY
其他当前连接                       -> STITCH
```

先晋升再判断旧拓扑替换非常重要：如果当前快照已经形成新的稳定墙角和替代 contour route，这些新节点必须先进入持久层，才能作为删除旧角点/旧边的可靠 replacement 证据。

**步骤 5：原子提交旧静态角点替换。**

`UpdateStaticCornerTopology()` 只在语义事务进行中工作。对位于当前可靠 contour 方窗内的静态节点：

- 本帧重新匹配旧角点：`topology_missed_count` 减 1，保留可恢复阻尼；
- 本帧未匹配，但旧位置被当前静态长线段内部明确覆盖：记一次强拓扑矛盾；
- 未观测或证据不够明确：不推进矛盾计数。

强矛盾达到 `static_topology_remove_frames` 后仍不能直接删除旧角点，必须同时通过：

1. `contradiction_mature`：连续/累积矛盾已经成熟；
2. `replacement_topology_stable`：同一当前 Polygon 上存在已观测、已匹配、已 finalized 的 `STATIC_GLOBAL` 替代节点及通过碰撞验证、保存 clearance route 的 contour 边，而且替代 chord 确实经过旧角点附近；
3. `removal_preserves_connectivity`：移除旧点后不会破坏机器人当前可达图；没有 odom 时则保护 active 静态连通性。

三项同时通过才调用 `RemoveNodeFromGraph()`。旧点在 precommit 阶段不会先设 `topology_blocked=true`，因为提前屏蔽一个割点和提前删除一样可能使图断开。

**步骤 6：提交成熟的旧 contour 边替换。**

`CommitMatureContourEdgeReplacements()` 只检查 `STATIC_GLOBAL` 两端的历史 contour 边。当前可靠轮廓连续否定其相邻关系达到 `static_remove_frames` 后：

- 如果保存 route 已被静态物理障碍挡住，即 `static_valid=false`，成熟后直接删除 contour 身份；安全事实优先，即使删除后暂时不可达也不能继续搜索一条物理碰撞边；
- 如果旧 route 仍物理安全，必须存在 active 静态替代路径才删除，且该替代路径至少经过一条当前 observed、两端已匹配且 finalized、保存有效 clearance route 的 contour-follow 边；偶然存在一条 visibility shortcut 不足以证明新轮廓拓扑已经准备好；
- 替代尚未准备好时继续保留旧 contour 身份等待后续快照。

删除 contour 身份后，如果同一 pair 还有 visibility 身份，`connect_nodes` 仍然保留；只有不存在其他边身份时才真正删除总邻接。

**步骤 7：最后删除普通生命周期已确认删除的节点并结束事务。**

旧角点和 contour 边替换完成后，才遍历之前收集的 `remove_nodes`：

```cpp
for (const auto& node_ptr : remove_nodes)
    RemoveNodeFromGraph(node_ptr);

semantic_update_in_progress_ = false;
```

把普通删除放到最后，可以让本帧替代结构评估面对的是已经加入新节点、完成边验证并完成成熟候选晋升的完整图，而不是一个被提前挖掉旧节点的半成品。

##### 3.2.7.4 为什么三个阶段必须按这个顺序

以“旧静态墙角被当前直墙替代”为例：

```text
ExtractGraphNodes()
    先为当前轮廓中没有旧身份的新角点创建 STATIC_CANDIDATE
                         ↓
UpdateNavGraph()
    把新节点加入图，验证新 contour/visibility route 和动态 mask
                         ↓
CommitSemanticGraphUpdate()
    先让满足条件的新候选晋升 STATIC_GLOBAL
    再检查替代 route 是否稳定、删除旧点是否保持连通
    最后一次性删除真正过时的旧点/旧边
```

如果顺序反过来，先删除旧点再慢慢建立新点和新边，规划器会短暂看到断裂图；如果新 route 最后碰撞验证失败，旧拓扑已经丢失且没有可靠替代。当前顺序保证发布给规划器的是同一语义快照下“新节点、边状态、动态 mask 和生命周期决策全部完成”的一致结果。

### 3.3 Commit 之后：导出一致快照并交给规划器

生命周期提交完成后才调用：

- `GetNavGraph()` 生成当前可搜索图；
- `ExtractGlobalContours()` 更新全局轮廓集合；
- `GraphPlanner::UpdaetVGraph()`；
- 图消息和可视化发布。

主循环中的实际顺序是：

```text
CommitSemanticGraphUpdate(...)
        │
        │ 完成节点晋升、删除、替换和边状态提交
        ▼
DynamicGraph::GetNavGraph()
        │
        │ 从维护图中筛出本轮允许参加搜索的节点
        ▼
ContourGraph::ExtractGlobalContours()
        │
        │ 刷新全局/局部轮廓线段缓存和边界缓存
        ▼
GraphPlanner::UpdaetVGraph(nav_graph_)
        │
        │ 把筛选后的共享节点列表交给规划器
        ▼
UpdateGlobalGraph() / RViz 发布 / 后续路径查询
```

这三步并不是重复处理同一份数据，而是分别完成三种不同的工作：

| 函数 | 输入侧数据 | 主要输出 | 负责的问题 |
|---|---|---|---|
| `GetNavGraph()` | `DynamicGraph` 内维护的各类节点和边状态 | `NodePtrStack` | 哪些节点现在可以参加路径搜索 |
| `ExtractGlobalContours()` | `ContourGraph` 内登记的轮廓点对 | 多组 `PointPair` 缓存 | 后续碰撞检测和显示应使用哪些轮廓线段 |
| `UpdaetVGraph()` | `GetNavGraph()` 返回的节点指针列表 | `GraphPlanner::current_graph_` | 规划器本轮从哪一组节点开始搜索 |

#### 3.3.1 `DynamicGraph::GetNavGraph()`：从维护图筛出搜索图

`DynamicGraph` 内保存的是“维护图”，其内容比规划器实际使用的“搜索图”更宽。维护图中可能暂时保留：

- 尚未被本帧语义快照再次确认的 `STATIC_CANDIDATE`；
- 已经失去有效邻边、但仍需等待重连或生命周期判定的静态节点；
- 被合并、被拓扑阻塞或只用于历史记录的节点；
- 当前动态障碍暂时封锁的边；
- `ODOM`、`GOAL` 等特殊节点。

`GetNavGraph()` 的任务不是重新建图，而是对这些已有节点做一次“搜索资格过滤”。它依次扫描：

```text
globalGraphNodes_
staticCandidateGraphNodes_
dynamicLocalGraphNodes_
```

其中 `globalGraphNodes_` 还包含 `ODOM` 和可能存在的 `GOAL` 节点，不应简单理解成“只装静态全局轮廓点”。

第一层过滤由 `IsSearchEligible()` 完成。它最终调用 `IsGraphNodeSearchEligible()`，核心规则如下：

1. 节点不能为空；
2. 已被合并的节点不能搜索；
3. `is_navpoint` 节点不能作为普通图节点搜索；
4. `topology_blocked` 节点不能搜索；
5. `PATH_HISTORY` 只用于历史路径维护，不能搜索；
6. `STATIC_CANDIDATE` 必须在当前接受的语义快照中被观测到，即 `observed_in_semantic_snapshot == true`；
7. 只有 `ODOM`、`GOAL`、`STATIC_GLOBAL`、满足上一条的 `STATIC_CANDIDATE` 和 `DYNAMIC_LOCAL` 才可能通过来源类型检查。

可概括为：

```text
节点存在
  && 没被 merged
  && 不是 navpoint
  && 没被 topology_blocked
  && 不是 PATH_HISTORY
  && candidate 已被本快照观测
  && source 属于允许搜索的类型
```

需要特别注意：节点字段 `is_active` 并不是这里判定搜索资格的唯一开关。当前搜索入口更关注节点的生命周期状态、来源类型和边的正式状态。不要把轮廓模块中的 `is_active` 与规划图中的“可搜索”完全等同。

对于普通轮廓节点，只有节点本身合格还不够，还要通过第二层过滤：`HasActiveSearchEligibleIncidentEdge()`。也就是至少存在一条真正可用、而且对端同样可搜索的关联边。

一条边要成为这种有效关联边，至少需要：

- 邻居节点存在；
- 当前节点到邻居和邻居到当前节点的双向连接记录都存在；
- 双向 `GraphEdgeState` 都为 active；
- 邻居本身也通过节点搜索资格检查；
- 检查普通轮廓连通性时，不把 `ODOM`/`GOAL` 端点当成证明轮廓节点已稳定接入图的普通邻边。

`GraphEdgeState::IsActive()` 等价于同时满足：

```text
static_valid
  && !dynamic_blocked
  && !topology_blocked
  && active
```

因此，下面这些边都不能让一个普通节点进入搜索图：

- 静态几何或轮廓碰撞验证失败：`static_valid == false`；
- 当前被动态障碍遮挡：`dynamic_blocked == true`；
- 生命周期或拓扑规则封锁：`topology_blocked == true`；
- 总开关尚未激活：`active == false`；
- 只在一个方向存在或只有一个方向有效。

不同来源节点的处理有一点差异：

- `STATIC_GLOBAL`：必须搜索资格合格，并至少连接一条双向有效的普通图边；
- `STATIC_CANDIDATE`：除有效邻边外，还必须被当前语义快照观测；
- `DYNAMIC_LOCAL`：必须保留有效搜索资格和有效关联边；
- `ODOM`、`GOAL`：属于规划查询的特殊端点，不能机械地按普通轮廓孤点规则理解。

例如，一个静态墙角因为局部遮挡暂时失去全部有效邻边时，它仍可能留在 `globalGraphNodes_` 中供后续匹配、重连和生命周期判断使用，但本轮不会出现在 `GetNavGraph()` 返回值里。这样既避免立即丢失静态身份，又避免规划器穿过一个已经断开的孤立节点。

`GetNavGraph()` 返回的 `NodePtrStack` 是 `NavNodePtr`（共享指针）的列表。这里发生的是浅拷贝：

```text
复制的是 shared_ptr 列表
不是复制每个 NavNode 对象
也不是复制一套独立的边
```

所以它不会：

- 重新提取轮廓；
- 新建或重建边；
- 执行 Dijkstra/A*；
- 深拷贝一份与维护图完全隔离的节点图；
- 自动修复失效边。

返回列表决定“本轮搜索图有哪些成员”，但成员指向的仍是维护图中的同一批 `NavNode` 对象。

#### 3.3.2 `ContourGraph::ExtractGlobalContours()`：把登记的轮廓身份转换成碰撞缓存

节点图解决“怎么走”，轮廓缓存解决“某条候选连线是否穿过轮廓或边界”。`ExtractGlobalContours()` 会先清空上一轮导出的缓存：

```text
global_contour_
inactive_contour_
unmatched_contour_
boundary_contour_
local_boundary_
```

然后从两个长期维护的轮廓集合重新提取：

```text
global_contour_set_   -> 普通轮廓点对
boundary_contour_set_ -> 边界轮廓点对
```

这里的集合元素仍然是节点指针对；导出后的各个容器则主要是 `PointPair`，即两个端点坐标组成的线段。它不是重新运行 OpenCV 查找轮廓，也不是把所有线段重新排序成一个闭合 `Polygon`。

##### A. `global_contour_`

遍历 `global_contour_set_` 时，每个已登记的轮廓点对都会把当前两个节点位置写入 `global_contour_`：

```text
(node1, node2)
    -> (node1.position, node2.position)
    -> global_contour_
```

这一层不要求线段当前处于机器人附近，也不要求两个端点当前都 active。因此它表达的是“系统登记过的全局轮廓线段身份的当前坐标版本”。其主要用途包括：

- 全局或静态长边的轮廓碰撞判断；
- 全局轮廓可视化；
- 保存局部视野外仍需记忆的轮廓约束。

严格说，`global_contour_set_` 表示所有被登记进该集合的轮廓点对，不应仅凭变量名断言里面永远只有 `STATIC_GLOBAL` 节点。

##### B. 本地相关性的判定

只有被认为与本地区域有关的线段，才继续参与 `inactive_contour_` 和 `unmatched_contour_` 的分类。`IsEdgeInLocalRange()` 的逻辑是“任意一个端点满足即可”：

```text
node1.is_near_nodes
  || node2.is_near_nodes
  || node1 位于当前局部范围
  || node2 位于当前局部范围
```

位置范围判断还会检查高度差和与机器人之间的三维距离，大致要求位于传感器范围 `sensor_range` 内。因此，一条线段即使只有一端进入局部区域，也会被当作局部相关线段处理。这样能覆盖从局部区域向历史全局区域延伸的“接缝线段”。

##### C. `inactive_contour_`

对局部相关线段，代码调用的 `IsActiveEdge(node1, node2)` 名称容易令人误会。这个函数当前只检查：

```text
node1->is_active && node2->is_active
```

它并没有读取 `GraphEdgeState`，也没有检查：

- `static_valid`；
- `dynamic_blocked`；
- `topology_blocked`；
- 双向图边是否存在。

只要任意端点的节点级 `is_active` 为 false，该端点对就写入 `inactive_contour_`。所以这里的 inactive 含义是“轮廓端点当前未激活”，不是“规划图边当前不可通行”。例如，一条边被动态障碍设置成 `dynamic_blocked == true`，但两个端点仍是 `is_active == true`，它不会因此自动进入 `inactive_contour_`。

##### D. `unmatched_contour_`

如果局部相关线段的两个端点都 active，但至少有一个端点不属于 `is_near_nodes`，它会进入 unmatched 分支。这个缓存用于保留“局部新轮廓与旧全局轮廓之间尚未完全对齐的接缝约束”，避免局部碰撞检查遗漏视野边缘或近邻集合边缘的轮廓段。

如果某个端点已经通过匹配关系关联到当前轮廓节点，导出时会尝试用其匹配节点 `ctnode` 的当前位置替换对应端点，从而让碰撞缓存尽量贴合当前观测：

```text
历史端点位置
    └─ 若存在当前匹配 ctnode
       则使用 ctnode.position
```

当前实现使用 `if ... else if ...`：如果两个端点同时具有匹配关系，这个分支只会优先替换第一个满足条件的端点，而不会在同一次处理中同时替换两端。这是实现细节，不要把 `unmatched_contour_` 理解成“两端都已经完全重投影到本帧轮廓”。

`inactive_contour_` 和 `unmatched_contour_` 主要补充本地边碰撞判断；`global_contour_` 则承担更完整的全局轮廓约束。

##### E. `boundary_contour_` 和 `local_boundary_`

遍历 `boundary_contour_set_` 时：

1. 所有登记的边界点对都加入 `boundary_contour_`；
2. 位于局部范围的边界点对同时加入 `local_boundary_`；
3. 对局部边界调用 `IsValidBoundary()`；
4. 如果检测到一条此前尚未登记的无效边界，就把双方节点 ID 互相写入各自的 `invalid_boundary` 集合。

也就是说：

```text
boundary_contour_ = 全部已登记边界线段
local_boundary_   = 其中与当前局部范围有关的子集
```

`local_boundary_` 是先加入缓存、再做有效性登记的，因此“出现在 local_boundary_ 中”不等于“边界必然有效”。有效性结果由节点上的无效边界关系另行约束。

当某对边界首次被判断为无效时，函数会记录这个事实；以后再次遇到同一对节点，已有的 `invalid_boundary` 关系会直接使其保持无效。`ExtractGlobalContours()` 本身不会在这里自动恢复已登记的无效边界。

这些缓存后续大致按如下方式消费：

| 缓存 | 主要含义 | 典型用途 |
|---|---|---|
| `global_contour_` | 全部已登记普通轮廓线段 | 全局/静态边碰撞、RViz |
| `inactive_contour_` | 局部范围内含 inactive 端点的轮廓线段 | 本地边碰撞补充 |
| `unmatched_contour_` | 局部范围内未完全落入 near 集合的接缝线段 | 本地边碰撞补充 |
| `boundary_contour_` | 全部已登记边界线段 | 边界碰撞约束 |
| `local_boundary_` | 当前局部相关的边界线段 | `LocalBoundaryHandler()` 等局部处理 |

因此，`ExtractGlobalContours()` 的本质是“把节点形式的轮廓身份，刷新成后续碰撞检测和显示所需的几何线段缓存”，而不是改变规划图节点的生命周期。

#### 3.3.3 `GraphPlanner::UpdaetVGraph()`：把搜索成员列表交给规划器

函数名中的 `Updaet` 是现有代码中的拼写，实际实现非常简单：

```cpp
void GraphPlanner::UpdaetVGraph(const NodePtrStack& vgraph) {
  current_graph_ = vgraph;
}
```

它做的事情只有一件：把 `GetNavGraph()` 得到的共享节点指针列表赋给 `GraphPlanner::current_graph_`。

它不会在调用时：

- 立即求路径；
- 运行 Dijkstra 或 A*；
- 重新验证所有静态/动态碰撞；
- 创建、删除或重连节点边；
- 清空每个节点的 `gscore`；
- 深拷贝一套私有图。

这仍然是浅拷贝。可以把它理解成：

```text
DynamicGraph 持有 NavNode A/B/C
                ▲   ▲   ▲
                │ shared_ptr
GetNavGraph() 返回 [A, B, C]
                │ 复制指针列表
                ▼
GraphPlanner::current_graph_ = [A, B, C]
```

所以：

- 下一次 `UpdaetVGraph()` 前，`current_graph_` 的列表成员基本固定；
- 但列表中的节点对象仍与 `DynamicGraph` 共享；
- 节点的邻接表、父节点、代价和可达性等字段并不是天然隔离的快照；
- 后续规划查询会直接在这些共享节点对象上更新搜索状态。

这种责任分工是：

```text
DynamicGraph：维护节点身份、生命周期、连接和正式边状态
ContourGraph：维护轮廓身份及碰撞所需的线段缓存
GraphPlanner：在给定搜索成员和有效边状态上执行路径查询
```

#### 3.3.4 真正的路径搜索发生在后续规划周期

`UpdaetVGraph()` 只是交接搜索图。真正执行规划时，`GraphPlanner` 还会做可达性和查询端点处理。

没有有效目标时，规划器可以只从 odom 更新当前图的可达性：

```text
UpdateGraphTraverability(odom, nullptr)
```

有目标时，整体思路是：

```text
调整/更新目标点
    ↓
先从 odom 更新基础图可达性
    ↓
UpdateGoalNavNodeConnects(goal)
为当前查询目标重新验证可连接的图节点
    ↓
再次更新包含 goal 的可达性
    ↓
PathToGoal(...)
生成最终路径
```

先更新基础可达性再处理目标边，可以避免直接沿用上一轮查询残留的目标连接。目标节点是在查询阶段显式连接到 `current_graph_` 候选节点的，所以刚接收的新目标即使尚未作为普通成员出现在本轮 `current_graph_` 列表里，也仍可由目标连接逻辑单独处理。

`UpdateGraphTraverability()` 会先重置相关搜索字段，例如：

- `gscore`、`fgscore`；
- `parent`、`free_parent`；
- `is_traversable`、`is_free_traversable`。

然后执行两类类似 Dijkstra 的传播：

1. 普通可达传播：沿双向 active 的边扩展，得到 `gscore`、`parent` 和 `is_traversable`；
2. covered/free 可达传播：额外限制邻居满足 covered 条件，得到 `fgscore`、`free_parent` 和 `is_free_traversable`。

搜索还会排除无效边界关系。边代价优先使用已经保存且有效的 `route_cost`；若没有可用的路线代价，则退化为节点间三维欧氏距离。

这说明：

```text
GetNavGraph()       决定“谁有资格参加”
UpdaetVGraph()      决定“规划器当前拿到谁”
UpdateGraphTraverability()/PathToGoal()
                    决定“从起点实际能到谁、最终走哪条路”
```

节点进入 `current_graph_` 并不代表它必然能从 odom 到达；最终可达性仍由有效边、边界约束以及后续搜索传播决定。

#### 3.3.5 为什么必须按这个顺序发布

`CommitSemanticGraphUpdate()` 可能在同一批提交中完成：

- 候选节点晋升；
- 旧节点延迟删除；
- 替代 route 确认；
- 动态阻塞状态提交；
- 拓扑封锁与解除；
- 节点和邻边的最终增删。

因此必须先完成 commit，再运行 `GetNavGraph()`。否则搜索成员可能仍包含刚应删除的旧节点，或遗漏刚晋升且已经接好边的新节点。

随后运行 `ExtractGlobalContours()`，是为了让碰撞检测看到与已提交节点/轮廓身份相匹配的几何缓存。`UpdaetVGraph()` 本身不会读取或重建这些轮廓缓存，但后续目标连接、边验证和规划周期会同时依赖当前搜索图与轮廓碰撞数据，所以两者必须来自同一轮已提交状态。

最后才把 `nav_graph_` 交给 `GraphPlanner` 并发布图消息。这样规划器和 RViz 不会看到以下半成品组合：

- 新语义轮廓 + 旧搜索节点；
- 新节点 + 尚未提交的旧边状态；
- 已删除节点 + 仍引用旧位置的局部轮廓缓存；
- 新动态 mask + 尚未重算的边可通行状态。

如果主循环只是机器人 odom 移动，而没有接受新的语义快照，代码可以只更新 odom 连接，再执行 `GetNavGraph()` 和 `UpdaetVGraph()`；因为轮廓拓扑没有变化，此时通常不需要重复提取全部全局轮廓缓存。

最终可以把这段发布流程记成一句话：

> 先完成图维护事务，再筛选搜索成员，同时刷新碰撞几何，最后把共享节点列表交给规划器；交给规划器不等于已经完成路径搜索。

---

## 4. 轮廓查找：从障碍点到闭合多边形

### 4.1 局部画布尺寸和坐标变换

画布以 odom 为中心，基础尺寸：

```text
MAT_SIZE = ceil(2 * sensor_range / contour_grid_resolution)
若为偶数，再加 1，保证有中心像素
```

默认值：

```text
sensor_range = 20.0 m
contour_grid_resolution = 0.4 m
MAT_SIZE = ceil(40 / 0.4) + 1 = 101
resize_ratio = 3
MAT_RESIZE = 303
```

基础栅格是 0.4 m；放大图的一个像素对应约 `0.4 / 3 = 0.1333 m`。

世界点到图像：

```text
row = center + round((world_x - odom_x) / resolution * ratio)
col = center + round((world_y - odom_y) / resolution * ratio)
```

这里图像 row 对应世界 x，col 对应世界 y。转回世界坐标时做逆变换，z 暂时取 odom z。

### 4.2 障碍投影与膨胀

每个障碍点先落入基础栅格，然后固定对周围 `3 x 3` 像素加 1。这一步独立于 `Util/obs_inflate_size`，后者主要用于其他地形/局部处理。

对于当前语义 OctoMap 输入，调用参数 `is_verified_occupied=true`，二值化规则是：

```text
pixel > 0 -> occupied
```

因此当前主路径中 `CDetector/filter_count_value=3` **不参与语义障碍轮廓的保留门限**。它只影响非 verified 输入且 `is_static_env=false` 的旧原始点云去噪分支。

### 4.3 resize 和 box filter

基础占用图：

1. 转为 `CV_8UC1`，占用值乘 255；
2. 双线性 resize 到 `resize_ratio` 倍；
3. 使用 `kBlurSize x kBlurSize` box filter，且 `normalize=false`。

`kBlurSize` 不是 YAML 直接配置，而是：

```text
round(robot_collision_clearance / contour_grid_resolution)
```

默认 `round(0.45 / 0.4) = 1`，所以默认 box filter 实际没有扩大邻域。

### 4.4 `findContours` 和 RDP

OpenCV 参数：

```text
RETR_TREE
CHAIN_APPROX_TC89_L1
```

每条原始闭合轮廓再执行：

```text
approxPolyDP(epsilon = DIST_LIMIT * simplify_ratio, closed = true)
DIST_LIMIT = resize_ratio * 1.5
```

默认静态：

```text
epsilon = 3 * 1.5 = 4.5 resized pixels ≈ 0.6 m
```

默认动态 `dynamic_simplify_ratio=2`：

```text
epsilon = 9 pixels ≈ 1.2 m
```

动态轮廓因此更稀疏，减少移动物体的像素抖动角点。

### 4.5 拓扑过滤 `TopoFilterContours`

OpenCV hierarchy 用来处理嵌套轮廓。对每条轮廓：

- 少于 3 点：删除；
- 如果 `free_odom_resized_` 不在该多边形内：删除它的所有内部子轮廓；
- 顶层/同层轮廓本身仍可保留。

这一步的目标不是做全局语义拓扑配准，而是避免障碍内部的嵌套边界生成多余导航结构。

### 4.6 邻点和像素阶梯简化

`AdjecentDistanceFilter()` 做两件事：

1. 相邻顶点距离必须大于本次 RDP 的 `distance_limit`；
2. 若连续三点几乎在同一直墙方向，则递归删除中间墙点。

转回世界坐标后，又执行一次与 resize 无关的闭合轮廓共线简化：

- 中间点到首尾 chord 的距离不超过 `collinear_tolerance`；
- 两段方向距离 180° 不超过 `collinear_angle_deg`；
- 才删除中间点。

默认是 0.20 m 和 8°。真实直角、门槛折线等不会只因距离小就被删除。

---

## 5. CTNode 如何确定为墙、凸角、凹角或柱体

### 5.1 小轮廓压成柱体

`kPillarPerimeter = robot_dim * 4`，默认是 2.4 m。

`IsAPillarPolygon()` 当前实现累计相邻输入点段长，但没有把最后一点到第一点的闭合段计入该变量。只要累计值不大于门限，就把整个多边形压成一个位于顶点均值位置的 `PILLAR` CTNode。

### 5.2 表面方向

对普通轮廓的每个 CTNode，分别沿 `front` 和 `back` 向外找，直到距离当前顶点至少达到 `robot_collision_clearance`。再用 `ContourSurfDirs()` 得到两条稳定表面方向。

如果绕完整个小轮廓仍找不到足够远的参考点，整个多边形会转成柱体。

### 5.3 墙和角点

两个表面方向相加：

```text
topo_dir = normalize(dir1 + dir2)
```

- 和接近 0：两方向相反，是直墙，`free_direct=UNKNOW`；
- 非 0：在 `topo_dir` 方向取一个 `contour_grid_resolution` 距离的测试点；
- 测试点在多边形内：`CONVEX`；
- 测试点不在多边形内：`CONCAVE`。

这里的命名沿用代码语义。真正决定“自由侧投影方向”的 `NodeProjectDir()` 是：

- `CONCAVE`：沿 `topo_dir`；
- `CONVEX`：沿 `-topo_dir`；
- `PILLAR/UNKNOW`：无投影方向。

### 5.4 哪些未匹配 CTNode 能成为新节点

`MatchContourWithNavGraph()` 最后只输出：

- 未匹配；
- `free_direct != UNKNOW`；
- 柱体，或两个表面方向的点积没有被判为直墙；

的 CTNode。

默认 `accept_max_align_angle=4°`，内部墙过滤阈值为：

```text
cos(pi - 4° / 2) = cos(178°) ≈ -0.99939
```

表面方向极接近反向的点会被当成墙样本丢弃；真正有转角的点才进入新节点候选。

---

## 6. 当前轮廓与已有节点如何匹配

当前实现不是逐 CTNode 贪心抢最近节点，而是先生成所有合理 pair，再做全局排序的一对一分配，因此不依赖 `findContours()` 的遍历顺序。

### 6.1 候选 pair 的硬条件

CTNode 必须已分类，历史节点必须满足：

- 不是 odom；
- 不是历史 navpoint；
- 不是外部 goal；
- 高度差小于 `kTolerZ`；
- 静态只匹配静态，动态只匹配动态；
- pillar 只能匹配 pillar；
- 静态非 pillar 的 `free_direct` 类型必须一致；
- 连接 CTNode 和 NavNode 的短线不能被当前轮廓/碰撞层阻挡。

裁剪端点还有额外身份隔离：当前静态轮廓若是 `is_boundary_clipped`，只能匹配既有的 `STATIC_CANDIDATE + is_transient_contour_endpoint`，不能借用普通静态候选或已确认 `STATIC_GLOBAL` 的身份。

### 6.2 距离门限

基础派生值：

```text
kMatchDist = 2 * robot_dim + contour_grid_resolution
默认 = 2 * 0.6 + 0.4 = 1.6 m
```

非动态方向一致时：

```text
direction_score = (dot(topo_dir_old, topo_dir_new) - 0.5) / 0.5
```

动态 pair 的 `direction_score=1`；pillar pair 为 0.5。

弱方向的紧位置回退半径：

```text
max(2 * contour_grid_resolution, 0.5 * robot_dim)
默认 max(0.8, 0.3) = 0.8 m
```

最终：

```text
direction_score > 0:
    match_radius = kMatchDist * max(0.5, direction_score)
否则:
    match_radius = tight_position_radius
```

所以默认门限大致处于 0.8～1.6 m，但 source、类型、方向和连线自由性会继续约束，不能仅靠距离合并。

### 6.3 排序评分和稳定 tie-break

```text
score = distance / match_radius
      + (1 - max(0, direction_score)) * 0.10
```

按以下顺序排序：

1. score 小者优先；
2. score 相同时 NavNode id 小者优先；
3. 再按 CTNode x；
4. 最后按 CTNode y。

随后每个 NavNode 和每个 CTNode 都最多使用一次。

### 6.4 匹配成功的副作用

`MatchCTNodeWithNavNode()` 同时设置：

- `ctnode.is_global_match=true`；
- `ctnode.nav_node_id=node.id`；
- `node.ctnode=ctnode`；
- `node.is_contour_match=true`；
- `node.observed_in_semantic_snapshot=true`。

如果旧节点原本是裁剪产生的临时端点，而新匹配点已经不在裁剪带，会将它转回普通静态候选，并重置确认计数，重新走持久确认流程。

---

## 7. 新 NavNode 的创建、稳定化与准入

### 7.1 新节点范围门限

- 动态 CTNode：平面距离必须不超过 `sensor_range`，默认 20 m；
- 静态 CTNode：必须同时不超过 `static_update_radius` 和 `static_stitch_radius`，默认均为 28.5 m。

注意语义提取窗是方形，而动态新节点这里再次使用半径 20 m 的圆形门限。因此方窗四角的动态轮廓不会创建节点；静态层用 28.5 m 覆盖对角区域。

### 7.2 地形准入

当前策略对未知地形是乐观的：

- 找得到 terrain-support 时，节点必须落在允许的地形邻域和高度范围；
- 找不到 terrain-support 属于 UNKNOWN，不会单独拒绝节点；
- 最终 incident edge 仍必须通过静态/动态碰撞和轮廓相交检查。

### 7.3 新节点继承字段

`CreateNewNavNodeFromContour()`：

- 静态 CTNode -> `STATIC_CANDIDATE`；
- 动态 CTNode -> `DYNAMIC_LOCAL`；
- `observed_in_semantic_snapshot=true`；
- 继承 `ctnode`、`position`、`free_direct`、`surf_dirs`；
- 静态裁剪点设置 `is_transient_contour_endpoint=true`；
- 分配全局唯一递增 id。

### 7.4 RANSAC 稳定化

普通静态节点在未 finalized 时积累两个滑动窗口：

- `pos_filter_vec`：位置样本；
- `surf_dirs_vec`：两侧方向样本。

窗口最大长度 `filter_pool_size=12`。

位置内点条件：平面距离小于 `filter_pos_margin`。当前它被强制设成 `Util/robot_collision_clearance`，默认 0.45 m。

方向内点条件：两对方向总角差小于 `filter_dirs_margin`。当前它被强制设成 `Util/angle_noise`，默认 15°。

位置与方向的 RANSAC 内点数都满足：

```text
inlier_size > node_finalize_thred
```

才将节点 `is_finalized=true`。默认阈值为 6，因此至少需要 7 个一致样本，不是 6 个。

### 7.5 静态候选晋升

每次本帧观测到候选：

```text
static_seen_count += 1，最大不超过 static_confirm_frames
static_missed_count = 0
```

默认达到 3 帧后，还需通过两个可配置 gate：

- `static_promotion_requires_finalized=true`：几何已 RANSAC finalized；
- `static_promotion_requires_active_edge=true`：至少有一条 active、可搜索、非 odom/goal 的 incident edge。

因此默认配置下虽然 `static_confirm_frames=3`，实际常常要等到至少第 7 个稳定样本才能晋升；三帧只是语义确认计数下限。

晋升后从 `staticCandidateGraphNodes_` 移到 `globalGraphNodes_`，并重标 incident edge source。

### 7.6 裁剪端点永不晋升

`is_transient_contour_endpoint=true` 时：

- 本帧仍看到：seen/missed 都保持 0；
- 本帧不再看到：立即 REMOVE；
- 永远不会变为 `STATIC_GLOBAL`。

它只用于当前局部画布切断墙体时的临时 FAR 式墙端探索，不能污染持久地图。

### 7.7 静态节点删除

未匹配不等于删除。只有：

1. 节点距离机器人不超过 `static_update_radius`；
2. `QueryStaticNodeEvidence()` 返回 `EXPLICIT_FREE`；
3. 连续达到 `static_remove_frames`；

才删除。默认连续 3 个快照。

任何 `UNKNOWN`、仍为 `STATIC_OCCUPIED`、遮挡、轮廓简化都会把 `static_missed_count` 清零，不累计删除。

未确认候选如果移出 `static_stitch_radius` 会直接删除；已确认 `STATIC_GLOBAL` 离开更新区不会因距离被删。

### 7.8 动态节点生命周期

动态节点完全快照化：

- 只在当前动态轮廓内匹配动态节点；
- 一帧未匹配立即删除节点及所有连接；
- 匹配成功位置执行：

```text
new_position = (1 - alpha) * old_position + alpha * current_ct_position
```

默认 `alpha=0.65`，当前帧权重大于历史；碰撞判断仍使用未经 EMA 平滑的最新动态点云，所以平滑不会掩盖新障碍。

---

## 8. contour-follow 边：沿轮廓自由侧走

### 8.1 “轮廓相邻”不是简单数组相邻

`IsCTNodesConnectFromContour()` 要求两个 CTNode 属于同一 Polygon，然后沿 front/back 搜索：

- 如果直接走到另一个 CTNode，则相邻；
- 中间允许跳过未匹配 CTNode，但这些点必须位于两端连线、半径 `kNearDist=robot_dim` 的 cylinder 内；
- 遇到另一个已匹配 CTNode或明显偏离该直线的 CTNode就停止；
- 连接不能穿过 boundary contour。

这样可把一串共线未匹配点压成一条轮廓边，而不会跨越真正拐点。

### 8.2 contour vote 和 top-two

每帧确认当前轮廓相邻就向 `contour_votes` 追加 1，否则在两节点都匹配的情况下追加 0。窗口默认最多 8。

`IsVoteTrue(votes, false)` 的规则是：

```text
sum(votes) > floor(N / 3)
```

函数名 `TopTwoContourConnector()` 有历史含义：

- 对当前快照明确相邻的 pair，当前轮廓是权威证据，直接进入验证，不需要等待 top-two；
- 只有当前没有直接权威关系的历史候选，才使用投票强度排名前两名的规则。

### 8.3 自由侧自适应投影

对同一当前轮廓 pair，依次尝试：

```text
projection_min ... projection_max，步长 projection_step
默认 0.15, 0.225, 0.30, 0.375, 0.45, 0.525, 0.60 m
```

两个障碍角点都沿各自 `NodeProjectDir()` 投影到自由侧，得到真正供机器人中心通过的 `route_start/end`。选择第一个完全安全的投影距离。

### 8.4 每个候选投影的验证顺序

1. 两个节点仍是同一当前 Polygon、同 source 且轮廓相邻；
2. 对未裁剪 Polygon，route 中点必须仍在机器人同一侧，且 route 不得穿过自身 Polygon；
3. route 对持久静态点云做 **零端点排除** 的走廊检查；
4. route 不得被其他静态 Polygon/轮廓阻挡；
5. 动态轮廓边还必须通过完整动态层，作为本帧结构；
6. 最后由 `IsOnTerrainRoute()` 检查坡度和地面高度变化。

静态 contour 边通过静态几何后即保留其结构，再单独计算 `dynamic_blocked`。因此动态障碍挡住静态绕墙线时，边仍存在但本帧不可搜索。

### 8.5 route cost

```text
route_cost = |node1 - route_start|
           + |route_start - route_end|
           + |route_end - node2|
```

GraphPlanner 搜索优先使用这个代价，而不是节点锚点间的直线距离。下游 waypoint 也使用相同的 `route_start/end`，避免“验证一条线、执行另一条线”。

### 8.6 几何失败和拓扑消失分离

若当前仍是相邻轮廓，但没有任何投影通过静态几何：

- 已存在边立即 `static_valid=false`，保证安全；
- 不立即删除 `contour_connects`、投票或旧 route；
- 后续帧重新验证成功即可恢复。

若当前可靠轮廓明确表明两点不再相邻，则累计 `current_contour_misses`，但旧边仍不会立刻被拓扑删除，详见第 11 节。

---

## 9. visibility 边：自由空间可视直连

### 9.1 候选集

在当前 `near_nav_nodes_` 中，对所有非 odom 的无序节点 pair 先做完整验证。静态 pair 的结构验证不包含动态障碍；动态影响稍后单独做 mask。包含动态节点的 pair 当帧直接验证完整动态层。

### 9.2 几何/语义条件

`IsValidConnect()` 依次要求：

1. 节点不是同一点；
2. 任一端都不能是 `CONCAVE`，即 `IsConvexConnect()` 要求两端均非凹角；
3. 从每个非 pillar 角点看向另一端，方向必须位于该角点允许的自由扇区；
4. 投影后的线段通过静态/动态点云与 Polygon 检查；
5. terrain 检查通过；
6. 两端 finalized 但本帧丢失 contour 身份时，不能随意新连；
7. visibility 投票达到条件。

### 9.3 普通角点投影

普通 visibility 边使用固定 `Util/visibility_edge_projection`，默认 0.15 m，但实际投影不会超过节点距离的 40%。局部当前匹配点优先使用最新 CTNode 的自由侧方向；超出局部范围时使用持久 NavNode 方向。

当前普通“障碍节点—障碍节点” visibility 验证虽然使用了投影线段做碰撞判断，但通过后不会像 contour/odom/goal 边那样把这段投影几何写入 `edge_state`；其 `has_clearance_geometry` 保持 false，搜索代价退回两个 NavNode 锚点的欧氏距离。保存并复用精确 route 的主要是 contour-follow 和查询层边。

### 9.4 点云走廊检查

`IsEdgeCollisionFreeInCloud()` 沿线采样：

```text
step = max(0.75 * contour_grid_resolution, 0.05)
requested_radius = max(0.75 * contour_grid_resolution,
                       robot_collision_clearance)
actual_radius = hypot(requested_radius, step / 2)
```

默认：

```text
step = 0.30 m
requested_radius = 0.45 m
actual_radius ≈ 0.474 m
```

半步长以勾股方式加入半径，是为了使离散采样球的并集仍覆盖连续线段的最小横向 clearance。

采样点的 x/y 沿线变化，z 固定取两个端点高度范围的中值；KD-tree 做三维球查询。这是一种 2.5D 走廊近似，不是沿倾斜线段逐点插值 z。坡度和 terrain 高差由后面的 terrain 规则补充约束。

普通障碍锚点边默认在两端留出 exclusion，避免端点所属障碍体素把自己的 incident edge 错判为碰撞。已经投影成机器人中心轨迹的 contour、odom、goal route 则使用 0 exclusion，完整检查端点邻域。

terrain route 的当前硬条件是：

- 若三维边长大于 `kMatchDist`，垂直变化/水平距离不得大于 1，即不得超过约 45°；
- 在 route 中点半径 `kMatchDist` 内查询 terrain-support 的最小/最大高度；
- 无匹配 terrain 时按 UNKNOWN 可通行；
- 有匹配时要求 `maxH-minH <= kMarginHeight`，且 `minH+vehicle_height` 与 route 中点 z 的差不超过 `kTolerZ/2`。

默认门限分别是 `kMatchDist=1.6 m`、`kMarginHeight=1.6 m`、`kTolerZ/2=1.1 m`。这些 z 门限由 `floor_height` 派生，当前配置相对宽松。

### 9.5 Polygon 检查的局部和全局分支

若两个端点都在局部范围：

- 检查当前 `contour_polygons_`；
- 检查 `unmatched_contour_`；
- 检查 `inactive_contour_`；
- 用 Polygon 的 `is_robot_inside` 对比线段中点所在侧；
- pillar 按中心/顶点到线段的 clearance 判断。

如果任一端点不在局部范围：

- 检查 `global_contour_`；
- 同时检查当前局部 Polygon 是否相交。

边界裁剪 Polygon 被视为物理开轮廓，忽略 OpenCV 人工闭合产生的虚假内部和封口线；持久静态原始体素仍继续裁决碰撞。

### 9.6 visibility 投票

投票窗口：

- 动态相关边：1；
- odom 旧通用分支：`ceil(connect_votes_size/3)`；
- 普通静态边：`connect_votes_size`，默认 8。

平衡投票判真：

```text
sum(votes) > floor(N / 2)
```

此外 `IsPolygonEdgeVoteTrue()` 要求：

- pair 属于“直接连接”——至少一个自由节点或至少一个当前 contour match；或
- 投票队列长度大于 2。

因此当前轮廓中明确观测的节点 pair 可以很快建立；纯历史 pair 至少积累若干帧再稳定。

这里还有一个重要区别：普通 visibility 边一次静态几何失败时，代码先追加一张 0 票；只要历史窗口多数仍为真，旧 visibility 边仍可能暂时 active。等投票翻转后，才删除 visibility 身份。这个行为不同于 contour-follow 已保存 route 的 `static_valid=false` 即时屏蔽，也不同于 odom/goal 查询边的逐次直接重建。

### 9.7 同方向稀疏化

所有 pair 先验证，再统一选择，避免边集合在循环中变化造成顺序依赖。

从节点 A 看候选 B，如果存在同一很窄角扇区内更近的候选 C，则 A-B 被支配。角阈值使用：

```text
cos(Util/accept_max_align_angle)
默认 cos(4°)
```

等距离时 id 更小的节点获胜。contour pair 永远不受这种 visibility 稀疏化删除。

---

## 10. odom/start 边和 goal 边

### 10.1 odom/start 是轻量查询层

每次完整语义更新的最后一步，或机器人平面移动超过 `odom_connection_update_distance=0.2 m` 而地图没变时：

1. 删除 odom 的所有旧 incident edge；
2. 重新枚举当前所有可用节点；
3. 逐条做真实路线验证；
4. 保存为 `ODOM_CONNECT`。

候选规则：

- `STATIC_GLOBAL` 即使已离开当前语义窗，仍可作为全局 start anchor；
- `STATIC_CANDIDATE` 和 `DYNAMIC_LOCAL` 必须是本快照已观测、已匹配且有 CTNode；
- 候选必须是 convex、pillar 或显式 boundary；
- 候选还必须拥有至少一条 active 的非查询图边，避免 odom 连接到永久孤儿，形成只有两个蓝点的死路。

start 边不走历史投票，输入相同则结果确定。

### 10.2 终端边的渐进角点投影

对于“一个障碍角点 + 一个自由终端”的 odom/goal 边，使用和 contour 类似的渐进投影：

- 起始 `ContourGraph/projection_min`；
- 步长 `projection_step`；
- 上限取 `max(projection_max, clearance + leaf_size)`；
- 每次实际投影不超过角点到终端距离的 40%；
- 选择最近的安全投影。

默认终端上限是 `max(0.60, 0.45 + 0.4) = 0.85 m`，比普通 contour-follow 的 0.60 m 更大。

有可靠角点方向时，投影后的机器人中心 route 使用 0 endpoint exclusion；pillar 没有方向，保留旧式端点 exclusion。

### 10.3 goal 边

每次规划时 goal 连接会从当前图快照重新构建，不累计计时器投票。候选包括：

- 当前 odom；
- 当前可搜索图中的静态/动态 convex、pillar 或 boundary 节点；
- 不包括历史轨迹节点和 concave 节点。

只有当前从 odom 已可达的候选，才会尝试连 goal。

odom 到 goal 直连是特例：两端都是机器人中心自由点，不做角点投影，也不做端点排除，完整检查整段。

---

## 11. 全局静态节点边的连接、保持与删除

本章只讨论障碍图中的静态节点对，尤其是两端已经确认成 `STATIC_GLOBAL` 的持久边。阅读这一章时应始终区分四件事：

1. 节点是否仍存在于维护图；
2. 两个节点之间是否仍保存某种边身份；
3. 总邻接 `connect_nodes` 是否仍存在；
4. `GraphEdgeState` 当前是否允许规划器搜索。

这四件事不是同一个开关。一条边可以“身份仍在但暂时不可搜索”，一个节点也可以“仍在全局维护图但本轮不交给规划器”。

### 11.1 先区分三种“局部范围”

代码里没有唯一的“局部范围”，至少有三套与静态维护有关的范围：

| 范围 | 主要参数/来源 | 决定什么 | 超出后会不会直接删除全局静态节点或边 |
|---|---|---|---|
| 局部连边工作范围 | 静态节点主要使用 `static_stitch_radius` | 是否进入 `near_nav_nodes_`，参加本轮 pair 枚举、匹配和局部重连 | 不会 |
| 当前可靠轮廓窗口 | 当前轮廓画布去除不可靠边界带后的区域 | 当前轮廓是否有资格否定旧角点或旧 contour 邻接 | 不会；窗口外不累计这种拓扑 miss |
| 静态节点证据更新范围 | `static_update_radius` | 未匹配节点能否用 `EXPLICIT_FREE` 累计删除 miss | 不会；范围外不累计普通删除 miss |

因此，“节点不在局部范围”必须说明是哪一种范围。最常见的含义是它没有进入 `near_nav_nodes_`；这只代表它不参加本轮昂贵的局部两两连边，不代表它离开了 `globalGraphNodes_`，更不代表它和邻居的历史边应当消失。

### 11.2 静态节点对上实际保存了什么

同一对静态节点可以同时有两种主要边身份：

```text
poly_connects      = visibility 身份
contour_connects   = contour-follow 身份
connect_nodes      = 可供图遍历的总邻接
edge_states[id]    = 当前安全状态、来源和实际 route
```

关系可以表示为：

```text
visibility 身份 ─┐
                 ├─ 至少一种身份存在 -> connect_nodes 保留
contour 身份 ────┘

两种身份都不存在 -> 才删除普通总邻接和对应 edge_states
```

所以“删除 visibility 边”通常只是从 `poly_connects` 删除该身份；如果同一节点对还是 contour 邻接，路径总邻接仍然保留。反过来也一样。

边能否搜索还要额外满足双方状态：

```text
forward.edge_state.IsActive()
&& reverse.edge_state.IsActive()

IsActive()
  = static_valid
    && !dynamic_blocked
    && !topology_blocked
    && active
```

### 11.3 静态边最初怎样建立

#### 11.3.1 两端都在 near：正常的新边生成入口

`UpdateNavGraph()` 会枚举 `near_nav_nodes_` 中所有非 odom 的无序节点对。静态—静态 pair 先做结构验证，然后分成两条建边路径。

visibility 路径：

1. 两端满足角点/柱体方向规则；
2. 连线或投影路线不穿过当前 Polygon；
3. 不穿过持久静态碰撞层；
4. 地形路线有效或按当前未知地形策略允许；
5. visibility 投票达到要求；
6. 在完整候选集中进行同方向稀疏化，两端都没有更近替代时才选中；
7. 调用 `AddPolyEdge()` 加 visibility 身份，再调用 `AddEdge()` 加总邻接和双向状态。

contour-follow 路径：

1. 当前轮廓确认两节点相邻，或者窗口外历史 contour 投票达到 top-two；
2. 根据角点自由侧计算投影后的机器人中心 route；
3. route 通过静态点云、其他 Polygon、当前动态层和地形验证；
4. 调用 `AddContourConnect()` 和 `AddEdge()`；
5. 保存 `CONTOUR_FOLLOW`、`route_start/end`、`route_cost` 和 clearance geometry。

当前可靠轮廓直接确认的相邻关系是本帧权威关系，不需要等待旧 contour 投票慢慢成熟；窗口外历史关系才继续使用历史投票阻尼。

新边最初可能连接 `STATIC_CANDIDATE`。候选在 `CommitSemanticGraphUpdate()` 中晋升后，会按已经存在的边身份重新标记 incident edge source：

```text
已有 contour 身份                    -> STATIC_CONTOUR
两端都已是 STATIC_GLOBAL 的 visibility -> STATIC_VISIBILITY
连接尚未确认成完整静态对               -> STITCH
```

这只是在晋升提交时给边确定持久来源，不会为了改 source 再凭空建立一条未经验证的新边。

#### 11.3.2 一端 near、一端范围外：历史拼接和断点恢复入口

如果 near 节点已经连接一个不在 near 的历史节点，并且该 pair 不是 contour 身份，代码会调用：

```text
UpdateGraphEdge(near_node, outside_node, false)
```

这一步重新检查已有 visibility 边。通过则保留；失败则删除 visibility 身份，并把范围外端点加入 `outside_break_nodes`。随后让这个范围外端点与所有 near 节点再次尝试 `UpdateGraphEdge()`：

```text
原连接 A_near ── B_outside 失败
                       ↓
B_outside 分别尝试连接当前 near 集合中的 A/C/D/...
                       ↓
找到通过正常几何、投票和地形验证的新锚点才重新接回
```

这不是无条件跨范围拉边，而是“旧局部锚点失效后，允许范围外历史图寻找另一个当前局部锚点”。

#### 11.3.3 两端都在范围外：不在本轮新 visibility pair 枚举中

两个节点都不在 `near_nav_nodes_` 时，它们不会进入本轮 near-near visibility pair 枚举。因此：

- 本轮不会因为二者距离机器人远而创建新的普通 visibility 边；
- 已存在的历史边也不会仅因二者都离开 near 而删除；
- 历史 contour 身份不会用“当前没看到”作为负证据；
- 等机器人以后靠近、其中至少一端重新进入局部工作集后，再获得局部复检或拼接机会。

“范围外保持”是持久全局图能够跨机器人运动保存拓扑的关键，否则局部窗口每移动一次，身后的图都会被裁掉。

### 11.4 三种空间组合的维护规则总表

下表描述 `is_stop_update_ == false` 的正常语义更新。冻结图更新时会跳过 near pair 重连和静态边统一 mask 等大部分 `UpdateNavGraph()` 主体，不能把冻结模式套用到下表。

| 两端相对 `near_nav_nodes_` 的位置 | 新建普通 visibility | 已有 visibility | contour 身份 | 动态 mask | 仅因距离删除 |
|---|---|---|---|---|---|
| 两端都 near | 枚举全部 pair，验证后按方向稀疏化建立 | 本轮统一重新选择；无效或被更近同向边替代时移除 visibility 身份 | 当前相邻关系权威更新，保存实际 route | 重算 | 不会 |
| 一端 near、一端 outside | 不参加 near-near 全 pair；可通过范围外断点恢复建立拼接 | 旧边主动复检；失败后尝试改连其他 near 节点 | 通过 out-contour 匹配/投票维护历史接缝 | 重算 | 不会 |
| 两端都 outside | 本轮通常不新建普通 visibility | 默认保持历史身份和邻接，不做 near pair 结构重建 | 窗口外不因缺失观测累计拓扑矛盾 | 静态边统一扫描时仍重算 | 不会 |

这里的“保持”不等于无条件 active。后面仍可能因为最新物理证据设置 `static_valid=false`，或因为当前动态层设置 `dynamic_blocked=true`。

### 11.5 每张语义快照怎样维护已有静态边

在非冻结的正常语义更新中，按真实时序，一条已有静态边会经历下面的处理：

```text
BeginSemanticGraphUpdate()
    先清除旧 dynamic_blocked，等待本帧重算
        ↓
UpdateGlobalNearNodes()
    只重建本轮局部工作集；范围外全局节点仍留在维护图
        ↓
UpdateNavGraph()
    ├─ near-outside 历史 visibility 复检
    ├─ near-near 完整 pair 验证和稀疏化
    ├─ contour 身份与实际 route 提交
    └─ 扫描所有静态—静态总邻接，统一应用静态 route 检查和动态 mask
        ↓
CommitSemanticGraphUpdate()
    成熟后才提交 contour 身份替换、旧节点替换和节点删除
```

最后的静态边统一扫描覆盖：

```text
globalGraphNodes_ + staticCandidateGraphNodes_
```

而不是只扫描 near 节点。对保存了 clearance route 的 contour-follow 边：

- route 与当前可靠轮廓窗口相交时，用持久静态碰撞层重查 `static_valid`；
- route 不进入当前可靠窗口时，不拿局部缺失观测武断改写远处静态事实。

对所有静态边，动态层只重新计算 `dynamic_blocked`：contour-follow 检查保存 route，普通 visibility 检查两节点之间的当前动态走廊。

如果 `is_stop_update_ == true`，`BeginSemanticGraphUpdate()` 仍会执行，但 `UpdateNavGraph()` 会跳过上述大规模旧边复检、near pair 重连以及静态边统一 mask，只保留 merged 清理、匹配成功 margin 提升和 odom/start 重建等有限动作。冻结模式的目的就是暂停主体图更新，因此调试静态边状态时必须先确认该开关是否开启。

### 11.6 哪些情况只是暂时不可搜索，边身份仍保留

| 状态变化 | 是否立即不可搜索 | 是否删除边身份 | 后续能否自动恢复 |
|---|---:|---:|---:|
| `dynamic_blocked=true` | 是 | 否 | 动态障碍离开、下一快照重算后可以 |
| contour route 得到 `static_valid=false` | 是 | 否，先保留 contour 身份和历史 route | 后续静态验证重新通过可以；若矛盾成熟也可能真正删除 |
| `topology_blocked=true` | 是 | 否 | 取决于后续拓扑提交逻辑；当前实现不会因一次 miss 设置 |
| 节点不在 near | 否，距离本身不影响边 activity | 否 | 重新进入 near 后复检 |
| 节点没有任何 active 非查询邻边 | 该节点不进入本轮搜索图 | 节点和边维护记录可以仍在 | 后续重连后可以重新进入 |

动态障碍只是一张当前快照的安全 mask，不能证明静态墙角之间的历史拓扑已经消失。因此动态层不得清除静态投票、visibility/contour 身份或 `static_valid`。

### 11.7 visibility 身份什么时候真正删除

visibility 身份没有 contour 边那套 `current_contour_misses` 成熟事务。它在参加本轮结构复检时，出现以下结果会由 `RemoveVisibilityEdge()` 移除：

1. 几何、持久静态碰撞、Polygon、地形或投票验证未通过；
2. near-near pair 虽然结构可行，但同方向存在更近的有效 visibility 候选，未进入 `selected_pairs`；
3. near-outside 历史 visibility 复检失败。

删除动作分两层：

```text
ErasePolyEdge(A, B)
    删除 visibility 身份和相关 visibility 记录
        ↓
如果 A-B 没有 contour/trajectory 身份
    EraseEdge(A, B)
    对称删除 connect_nodes 和 edge_states
否则
    保留总邻接，由剩余身份继续支撑
```

两个端点都在范围外时不会调用 near pair 的这套 visibility 复检，所以不会仅因距离或当前局部地图没覆盖它们而触发该删除。

### 11.8 contour 身份什么时候真正删除

一次当前不相邻不能直接删除旧 contour 边。只有同时满足以下条件，才把一次观测记入 `current_contour_misses`：

- 正在语义更新事务中；
- 两端都是静态候选或静态全局节点；
- 两端都在当前可靠轮廓窗口内；
- 两端位置都确实被当前静态轮廓覆盖或匹配；
- 当前轮廓明确确认二者不相邻。

只要边在可靠窗口外、任一端没有被可靠观测，或者证据只是 UNKNOWN，就不推进这个 miss。当前轮廓重新确认相邻时，miss 清零。

当 miss 达到 `Graph/static_remove_frames` 后，`CommitMatureContourEdgeReplacements()` 才考虑提交删除：

```text
旧 route 已被静态物理障碍挡住
    -> static_valid=false
    -> 安全事实优先，删除 contour 身份

旧 route 仍静态安全
    -> 必须存在 active 静态替代路径
    -> 替代路径还要包含当前观测、两端 finalized、
       保存有效 clearance route 的新 contour-follow 边
    -> 条件满足才删除

没有可靠替代
    -> 即使 miss 已成熟也继续保留旧 contour 身份
```

删除 contour 身份后，如果同一 pair 仍有 visibility 身份，就继续保留 `connect_nodes`；没有其他身份时才真正 `EraseEdge()`。

### 11.9 静态节点删除怎样连带删除边

边也可能不是自己满足删除条件，而是因为端点节点被删除而级联消失。`STATIC_GLOBAL` 有两条主要删除路径。

#### 11.9.1 明确 free 的普通生命周期删除

必须同时满足：

1. 节点本帧未被匹配；
2. 节点在 `static_update_radius` 内；
3. `QueryStaticNodeEvidence()` 明确返回 `EXPLICIT_FREE`；
4. 连续达到 `static_remove_frames`。

`UNKNOWN`、遮挡、动态占用、当前方窗外和低语义置信度都不是删除证据，并会阻止普通 free miss 连续累计。已经确认的 `STATIC_GLOBAL` 离开 update/near 范围时不会因为距离删除。

未确认的 `STATIC_CANDIDATE` 是例外：它超过 `static_stitch_radius` 后可以直接清除，避免局部候选无限滞留；这条规则不能套到 `STATIC_GLOBAL`。

#### 11.9.2 旧角点被新轮廓替代

旧静态角点未匹配还不够。强拓扑矛盾要求：

1. 旧点位于可靠 contour 方窗；
2. 它贴近当前非动态、非 pillar、非裁剪 Polygon 的长线段内部；
3. 离线段两端有足够距离；
4. 附近没有一个当前有效角点，排除角点位置抖动。

默认观测容差为：

```text
max(2 * leaf_size, min(kMatchDist, clearance))
= max(0.8, min(1.6, 0.45))
= 0.8 m
```

默认 endpoint guard 为：

```text
max(clearance, 1.5 * observation_tolerance) = 1.2 m
```

每次强矛盾使 `topology_missed_count += 1`；一次重新匹配只减 1，形成可恢复 damper。达到 `static_topology_remove_frames` 后仍需三个 precommit gate：

1. `contradiction_mature`：矛盾成熟；
2. `replacement_topology_stable`：同一当前 Polygon 上已有已观测、已匹配、已 finalized 的 `STATIC_GLOBAL` 替代节点和 active contour route；
3. `removal_preserves_connectivity`：删除旧点不会破坏机器人当前可达图；没有 odom 时则保护 active 静态连通性。

只有三项同时通过，才在事务末尾原子删除旧点。

#### 11.9.3 删除节点时对所有关联边做对称清理

`RemoveNodeFromGraph()` 会清理：

- `connect_nodes` 和双方 `edge_states`；
- `poly_connects`、`edge_votes`、`potential_edges`；
- `contour_connects`、`contour_votes`、`potential_contours`；
- trajectory 兼容关系；
- id map、near/wide/margin 临时集合；
- global/candidate/dynamic 三个容器；
- 全局 contour set 中所有 incident contour edge。

因此节点删除是最彻底的边删除路径；也正因如此，静态全局节点的删除证据比普通 visibility 身份删除严格得多。

### 11.10 维护图里存在，不等于本轮搜索图使用

范围外 `STATIC_GLOBAL` 仍会被 `GetNavGraph()` 检查，而且距离本身不是搜索过滤条件。但它必须：

- 节点自身满足 search-eligible；
- 至少存在一条双向 active、对端也可搜索的非 odom/goal incident edge。

所以可能出现：

```text
节点和历史边仍在维护图
    + 边身份也没有删除
    + 但所有边都被 dynamic/static/topology 状态屏蔽
        ↓
本轮 GetNavGraph() 不导出该静态孤儿
        ↓
以后边恢复或重新拼接后，再次进入搜索图
```

这避免规划器使用断开的孤点，同时保留未来重新匹配和恢复全局拓扑的机会。

### 11.11 odom 和 goal 边不属于上述持久静态边

`STATIC_GLOBAL` 与另一个障碍节点之间的边可以跨快照保存；与 odom/goal 的边则是查询层连接：

- `UpdateOdomConnections()` 先清除 odom 的全部旧邻接，再用最新静态碰撞、动态层、轮廓和地形重建；旧 `STATIC_GLOBAL` 不会在预处理时仅因距离被单独裁掉，但旧 odom 边最终仍会统一重建；
- goal 边在每次规划查询中由 `UpdateGoalNavNodeConnects()` 重建，不作为持久静态拓扑证据；
- 静态候选晋升时，只连接 odom/goal 也不能满足“已经接入可复用静态图”的 active-edge gate。

### 11.12 一个完整例子：机器人离开后再返回

假设静态全局节点 A、B 在机器人附近建立了一条 contour-follow 边：

```text
第一次经过：A、B 都 near
    -> 当前轮廓确认相邻
    -> 保存 contour 身份、clearance route 和双向状态

机器人驶离：A、B 都 outside
    -> 不参加 near-near 新 pair 枚举
    -> 不因当前快照没看见而增加 contour miss
    -> 节点和历史边留在全局维护图

机器人在别处行驶
    -> 正常非冻结更新中，每张新快照仍重算静态边的 dynamic_blocked
    -> 距离本身不删除 A、B 或 A-B

机器人返回，A、B 重新进入可靠窗口
    ├─ 当前轮廓仍确认相邻：route 重验，miss 清零，边继续使用
    ├─ 临时动态物体挡路：只置 dynamic_blocked，身份保留
    ├─ 新静态几何挡住旧 route：static_valid=false，立即停止搜索
    └─ 当前轮廓连续确认 A、B 不再相邻：累计 miss，
       成熟且满足安全/替代条件后才删除 contour 身份
```

一句话总结全局静态边策略：

> 局部范围决定“本轮主动维护谁”，物理状态决定“现在能不能走”，连续可靠观测决定“历史身份是否真的消失”；距离和一次缺失观测本身都不删除全局静态拓扑。

---

## 12. 搜索图最终如何筛选

节点可搜索条件：

- 不是 merged；
- 不是 navpoint / `PATH_HISTORY`；
- `topology_blocked=false`；
- source 是 odom、goal、static global、dynamic local；
- static candidate 只有本帧 `observed_in_semantic_snapshot=true` 才临时可搜索。

`GetNavGraph()` 还会排除没有 active 非查询 incident edge 的静态/动态孤儿。确认静态孤儿仍保留在 matching graph 中，等待未来轮廓把它重新接回，但不会交给本轮搜索。

搜索扩展一条边时要求：

```text
neighbor 节点可搜索
双方 edge_state 都存在且 IsActive()
边界关系不在 invalid_boundary
```

代价优先使用 `GraphEdgeState::route_cost`；没有 clearance geometry 才退回节点欧氏距离。

---

## 13. 当前默认参数与派生值

本节数值按 `launch/far_planner.launch` 和 `launch/semantic_far_planner.launch` 默认加载的 `config/default.yaml` 计算。若使用 `ssmi_bag_graph_test.launch`，它会先加载 default，再用 `config/ssmi_bag.yaml` 覆盖，其中最影响生命周期的是 `static_confirm_frames: 5`，因此 SSMI 回放的候选语义确认下限是 5 帧而非 3 帧。若显式选择 `matterport.yaml`，轮廓分辨率为 0.2 m、`robot_dim=0.5 m`、visibility/contour 投票窗口为 10，下面的派生距离也应重新计算。

### 13.1 轮廓与几何参数

| YAML 参数 | 当前值 | 实际作用 |
|---|---:|---|
| `contour_grid_resolution` | 0.4 m | 轮廓基础栅格、持久静态碰撞量化、多个派生距离 |
| `sensor_range` | 20.0 m | 轮廓方画布半边长；部分动态节点逻辑又把它当径向半径 |
| `robot_dim` | 0.6 m | `kNearDist`；匹配距离、pillar 周长、frontier 周长派生 |
| `vehicle_height` | 0.5 m | terrain-support 高度到机器人节点高度的偏移 |
| `Util/robot_collision_clearance` | 0.45 m | 点云走廊 clearance、位置 RANSAC margin、若干投影/保护距离 |
| `Util/visibility_edge_projection` | 0.15 m | 普通 visibility 端点固定投影 |
| `Util/angle_noise` | 15° | 方向 RANSAC margin、方向扇区噪声 |
| `Util/accept_max_align_angle` | 4° | 墙过滤与同方向 visibility 稀疏化 |
| `ContourGraph/projection_min` | 0.15 m | contour/terminal 首次自由侧投影 |
| `ContourGraph/projection_step` | 0.075 m | 渐进投影步长 |
| `ContourGraph/projection_max` | 0.60 m | contour-follow 最大投影 |
| `ContourGraph/boundary_guard` | 0.40 m | 画布边缘不可信带 |

重要派生值：

| 派生量 | 公式 | 默认值 |
|---|---|---:|
| `kLeafSize` | `contour_grid_resolution` | 0.4 m |
| `kNearDist` | `robot_dim` | 0.6 m |
| `kHeightVoxel` | `2 * resolution` | 0.8 m |
| `kMatchDist` | `2 * robot_dim + resolution` | 1.6 m |
| `kTolerZ` | `floor_height - kHeightVoxel` | 2.2 m |
| `kMarginDist` | `sensor_range - kMatchDist` | 18.4 m |
| `kCellHeight` | `floor_height / 2.5` | 1.2 m |
| `kMarginHeight` | `kTolerZ - kCellHeight/2` | 1.6 m |
| pillar 周长门限 | `4 * robot_dim` | 2.4 m |
| frontier 周长门限 | `4 * kMatchDist` | 6.4 m |

### 13.2 轮廓提取参数

| YAML 参数 | 当前值 | 实际作用 |
|---|---:|---|
| `CDetector/resize_ratio` | 3.0 | resize 倍率；实现中计算尺寸时转成 `int` |
| `CDetector/filter_count_value` | 3 | 当前 verified semantic 路径不使用该计数门限 |
| `CDetector/dynamic_simplify_ratio` | 2.0 | 动态 RDP/最小邻距再乘 2 |
| `CDetector/collinear_tolerance` | 0.20 m | 世界坐标共线中点最大偏离 |
| `CDetector/collinear_angle_deg` | 8° | 只折叠接近 180° 的中间点；加载时 clamp 到 0～30° |
| `CDetector/is_save_img` | false | 是否保存基础占用图 |

### 13.3 节点与边维护参数

| YAML 参数 | 当前值 | 实际作用 |
|---|---:|---|
| `Graph/connect_votes_size` | 8 | 普通 visibility/contour 投票最大窗口 |
| `Graph/clear_dumper_thred` | 4 | 旧兼容节点复检失败 damper；超过门限才 merged |
| `Graph/node_finalize_thred` | 6 | 位置和方向 RANSAC 都要求内点数 `> 6` |
| `Graph/filter_pool_size` | 12 | RANSAC 滑动窗口 |
| `Graph/static_update_radius` | 28.5 m | 静态新点/删除证据处理半径 |
| `Graph/static_stitch_radius` | 28.5 m | 静态局部匹配和重连半径；至少不小于 update radius |
| `Graph/dynamic_position_alpha` | 0.65 | 动态匹配点 EMA 的当前帧权重，加载时 clamp 到 0～1 |
| `Graph/static_confirm_frames` | 3 | 候选晋升的语义观测下限 |
| `Graph/static_remove_frames` | 3 | 明确 free 的节点删除门限；也是 contour 边矛盾成熟门限 |
| `Graph/static_topology_remove_frames` | 5 | 旧角点被当前直墙内部否定的成熟门限 |
| `Graph/static_promotion_requires_finalized` | true | 晋升前要求 RANSAC 定型 |
| `Graph/static_promotion_requires_active_edge` | true | 晋升前要求非查询 active incident edge |

### 13.4 语义地图和地形参数

| YAML 参数 | 当前值 | 实际作用 |
|---|---:|---|
| `MapHandler/semantic_local_window_radius` | 20.0 m | 机器人中心轴对齐方窗半边长 |
| `MapHandler/terrain_search_radius` | 0.8 m | CTNode 找不到直接地面时的 XY 回退 |
| `MapHandler/terrain_neighbor_radius` | 1.0 m | 节点地面邻域支持半径 |
| `MapHandler/floor_height` | 3.0 m | 派生 z 容差和 cell height，不是“固定地面 z” |
| `MapHandler/semantic_top1_only` | true | 当前解码固定使用 top-1；false 尚不提供多类别输出模式 |
| `MapHandler/semantic_min_probability` | 0.55 | `SemanticOcTree` top-1 归一化概率下限；低于门限按 `UNKNOWN` 处理，`ColorOcTree` 不应用此门限 |

### 13.5 查询层参数

| YAML 参数 | 当前值 | 实际作用 |
|---|---:|---|
| `main_run_freq` | 2.5 Hz | 主循环；拓扑计数仍只随新语义快照推进 |
| `odom_connection_update_distance` | 0.2 m | 无新语义图时重建 start 边的移动阈值 |
| `odom_timeout` | 3.0 s | 输入 watchdog |
| `semantic_map_timeout` | 3.0 s | 输入 watchdog；小于等于 0 可关闭 |

---

## 14. 参数加载中容易踩坑的细节

### 14.1 两个 Graph 角度参数会被覆盖

源码先读取：

```text
Graph/connect_angle_thred
Graph/dirs_filter_margin
```

紧接着又无条件赋值：

```text
filter_dirs_margin  = Util/angle_noise
kConnectAngleThred  = Util/accept_max_align_angle
```

所以当前真正生效的是 `Util/angle_noise` 和 `Util/accept_max_align_angle`。单独调整上述两个 `Graph/*` key 不会改变最终行为。

### 14.2 `resize_ratio` 虽是 float，画布尺寸按 int 倍数计算

`MAT_RESIZE = MAT_SIZE * (int)resize_ratio`，但坐标换算和 OpenCV resize 又使用 float ratio。建议保持整数倍率，否则尺寸中心和坐标比例可能不完全一致。

### 14.3 方形语义窗和圆形范围函数并存

- OctoMap 和可靠 contour window 是轴对齐方形；
- 一些旧 FAR 工具函数仍用欧氏半径；
- 静态 stitch 半径专门扩大到约方窗对角；
- 动态新点依然被 20 m 圆截断。

调范围时必须先确认目标逻辑使用的是 `abs(dx/dy)` 方窗还是 `norm()` 圆。

### 14.4 UNKNOWN terrain 是可通行策略，不是已知 free

未知 terrain 不单独拒绝节点和边，但这不代表跳过碰撞检查。静态点云、动态点云、Polygon、全局 contour 和角点方向仍全部执行。

### 14.5 `static_remove_frames` 有两个职责

它既是静态节点连续明确 free 的删除帧数，又是旧 contour edge 连续不相邻的成熟帧数。若只想调其中一个，当前配置没有独立参数。

### 14.6 `is_stop_update_` 不是停止全部语义处理

冻结图更新时，系统仍会接收语义快照、提取和匹配轮廓、清理未匹配动态节点、调用 `UpdateNavGraph(..., true, ...)` 并提交生命周期；它主要阻止新节点加入和普通大规模重连。调试时不要把它理解为“图完全静止”。

### 14.7 YAML 中存在但当前主链路不生效或只作兼容的参数

- `MapHandler/semantic_top1_only` 会被读入结构体，但当前仍固定使用 top-1，没有根据 false 切换到多类别输出；`semantic_min_probability` 已实际用于 `SemanticOcTree` top-1 置信度过滤。
- `MapHandler/cell_length`、`map_grid_max_length`、`map_grad_max_height` 仍留在 `default.yaml`，但当前 `MapHandlerParams` 和 `LoadROSParams()` 没有加载这三个 key；它们不会改变当前 semantic-only 地图处理。
- `Util/dynamic_obs_dacay_time` 和 `Util/dyosb_update_thred` 会被加载到旧 FAR 兼容静态变量，但当前语义动态层明确执行 `effective_dynamic = current_dynamic`，没有下游动态保持计时器。
- `Util/new_points_decay_time` 仍然有效：它控制 changed-obstacle 点进入 `stack_new_cloud_` 后的保留时间，进而影响“附近是否有新变化”、frontier 和旧兼容复检；它不决定动态碰撞层的寿命。

---

## 15. 典型状态机

### 15.1 静态节点

```text
当前静态角点
    │ 未匹配历史身份
    ▼
STATIC_CANDIDATE（本帧可搜索）
    │ 每帧观测 seen++
    │ seen >= 3
    │ finalized（默认至少 7 个一致样本）
    │ 有 active 非查询边
    ▼
STATIC_GLOBAL（持久）
    │
    ├─ 未观测/遮挡/仍占用：保留，miss 清零
    ├─ 连续明确 free 3 帧：删除
    └─ 连续强拓扑矛盾 5 帧
         + 稳定替代轮廓
         + 删除不破坏当前可达性
         -> 原子删除旧角点
```

裁剪端点分支：

```text
STATIC_CANDIDATE + transient
    ├─ 当前仍看到：只用于本帧，不累计晋升
    ├─ 后续看到真实非裁剪角：转普通 candidate，重新计数
    └─ 下一帧没看到：立即删除
```

### 15.2 静态 contour 边

```text
当前轮廓相邻 + 找到安全自由侧 route
    -> static_valid=true，保存 route
       ├─ 动态不挡：active
       └─ 动态挡：dynamic_blocked=true，仅临时不可用

当前仍相邻，但 route 静态碰撞
    -> static_valid=false，立即不可搜索，保留身份等待恢复

可靠当前轮廓连续判定不相邻 3 帧
    -> 若 route 已物理失效：删除
    -> 若 route 仍安全且有合格替代路径：删除
    -> 否则继续保留
```

### 15.3 动态节点

```text
本帧动态 CTNode
    ├─ 匹配已有 dynamic：EMA 更新
    └─ 未匹配：创建 DYNAMIC_LOCAL

下一接受快照
    ├─ 再匹配：继续存在
    └─ 未匹配：立即删除
```

---

## 16. 调试时应该看什么

### 16.1 日志

源码已有节流统计：

- `DG contour-follow edges`：active 数和 not_adjacent、clipped、static_cloud、self_polygon、other_static、dynamic、terrain、offset、vote 拒绝数；
- `DG start connections`：候选、接受数、无拓扑孤儿、各类拒绝；
- `GP goal connections`：goal 候选和拒绝；
- `DG static promotion gate`：晋升、等待 finalized、等待 active edge；
- `DG static topology replacement`：矛盾、删除、等待替代、割点保护；
- `DG contour edge replacement`：成熟、删除、物理阻挡、等待替代；
- `MH persistent static collision layer`：持久体素数和明确 free 删除数。

### 16.2 可视化层

主流程会分别发布/显示：

- 当前 `ContourGraph::contour_graph_`；
- `global_contour_` 与 `unmatched_contour_`；
- static global、dynamic local、最终 search graph 三层；
- 持久静态碰撞云和当前动态障碍云；
- clear nodes、out contour nodes、`free_odom_p`。

排查“为什么有点没边”时建议按顺序看：

1. CTNode 是否出现且是不是 `UNKNOW` 直墙；
2. 是否匹配历史节点；
3. 新节点是否被范围/terrain gate 拒绝；
4. 是否只有查询边而没有非查询 incident edge；
5. edge diagnostic 的 reject reason；
6. `static_valid`、`dynamic_blocked`、`current_contour_misses`；
7. 是否因同方向更近节点被 visibility 稀疏化；
8. 是否仍是 candidate，且未通过 finalized/active-edge 晋升 gate。

---

## 17. 源码索引

| 主题 | 文件/函数 |
|---|---|
| 主更新时序 | [`src/far_planner.cpp`](src/far_planner.cpp)：`FARMaster::Loop()` |
| 参数加载和派生 | [`src/far_planner.cpp`](src/far_planner.cpp)：`FARMaster::LoadROSParams()` |
| 语义 OctoMap 分类 | [`src/map_handler.cpp`](src/map_handler.cpp)：`RefreshLocalTerrainSupportOctomap()` |
| 持久静态碰撞层 | [`src/map_handler.cpp`](src/map_handler.cpp)：`UpdatePersistentStaticObstacleLayer()` |
| 节点删除证据 | [`src/map_handler.cpp`](src/map_handler.cpp)：`QueryStaticNodeEvidence()`、`QueryStaticTreeEvidence()` |
| 点云转轮廓 | [`src/contour_detector.cpp`](src/contour_detector.cpp)：`BuildTerrainImgAndExtractContour()`、`ExtractRefinedContours()` |
| 世界坐标共线简化 | [`include/far_planner/contour_detector.h`](include/far_planner/contour_detector.h)：`SimplifyClosedContourCollinearVertices()` |
| 当前轮廓图和角点分类 | [`src/contour_graph.cpp`](src/contour_graph.cpp)：`UpdateContourGraph()`、`AnalysisSurfAngleAndConvexity()` |
| CTNode/NavNode 匹配 | [`src/contour_graph.cpp`](src/contour_graph.cpp)：`MatchContourWithNavGraph()` |
| contour 边几何 | [`src/contour_graph.cpp`](src/contour_graph.cpp)：`ValidateContourFollowEdge()` |
| visibility/terminal 边几何 | [`src/contour_graph.cpp`](src/contour_graph.cpp)：`ValidateVisibilityEdge*()`、`ValidateTerminalVisibilityEdgeWithRoute()` |
| 点云连续走廊 | [`src/contour_graph.cpp`](src/contour_graph.cpp)：`IsEdgeCollisionFreeInCloud()` |
| 节点生成和全图重连 | [`src/dynamic_graph.cpp`](src/dynamic_graph.cpp)：`ExtractGraphNodes()`、`UpdateNavGraph()` |
| odom 边 | [`src/dynamic_graph.cpp`](src/dynamic_graph.cpp)：`UpdateOdomConnections()` |
| 静态/动态生命周期 | [`src/dynamic_graph.cpp`](src/dynamic_graph.cpp)：`Begin/Finalize/CommitSemanticGraphUpdate()` |
| 角点/轮廓边替换 | [`src/dynamic_graph.cpp`](src/dynamic_graph.cpp)：`UpdateStaticCornerTopology()`、`CommitMatureContourEdgeReplacements()` |
| 纯生命周期规则 | [`include/far_planner/node_struct.h`](include/far_planner/node_struct.h) |
| goal 边和搜索 | [`src/graph_planner.cpp`](src/graph_planner.cpp)：`UpdateGoalNavNodeConnects()`、`UpdateGraphTraverability()` |
| 回归测试 | [`src/graph_lifecycle_policy_test.cpp`](src/graph_lifecycle_policy_test.cpp)、[`src/contour_simplification_policy_test.cpp`](src/contour_simplification_policy_test.cpp)、[`src/terminal_visibility_policy_test.cpp`](src/terminal_visibility_policy_test.cpp) |

---

## 18. 用伪代码复述整套逻辑

```text
on accepted semantic map:
    validate and decode map once
    current_static  = semantic static voxels in robot-centred square
    current_dynamic = semantic dynamic voxels in robot-centred square
    terrain         = terrain-support voxels
    persistent_static += current_static
    persistent_static -= cells with explicit-free evidence

main loop when semantic_graph_dirty:
    BeginSemanticGraphUpdate()

    static_contours  = ExtractContours(current_static, simplify=1)
    dynamic_contours = ExtractContours(current_dynamic, simplify=2)
    BuildCTGraph(static_contours, dynamic_contours)
    adjust CTNode and existing NavNode heights

    near_history = SelectNearNodesBySource()
    unmatched_ct = DeterministicOneToOneMatch(CTGraph, near_history)
    delete dynamic nodes unmatched in this snapshot
    smooth matched dynamic nodes

    new_nodes = []
    for ct in unmatched_ct:
        if source range valid and terrain policy accepts:
            new_nodes += NavNodeFromCT(ct)

    UpdateNavGraph:
        add new_nodes to source-specific containers
        update current contour votes
        validate all local visibility pairs without order dependence
        commit current contour identities
        keep only closest visibility neighbor in each angular sector
        apply static geometry results
        apply latest dynamic mask
        rebuild odom/start query edges last

    CommitSemanticGraphUpdate:
        promote stable static candidates
        collect explicit-free node deletions
        atomically replace obsolete static corners when safe
        replace mature obsolete contour edges when safe
        execute remaining deletions

    search_graph = eligible nodes with active non-query topology
    publish one internally consistent graph snapshot

on robot moved >= 0.2 m without new semantic map:
    clear and rebuild only odom/start edges

on planning request:
    rebuild goal edges from reachable current candidates
    Dijkstra-style expansion over bidirectionally active edge states
    use stored route_cost when clearance geometry exists
    waypoint follows stored route_start/route_end
```

## 19. 最后总结

当前实现最重要的不是某一个角点阈值，而是它把四种证据拆开维护：

- **当前 contour** 决定“本帧看到了什么几何关系”；
- **持久静态点云** 决定“物理上是否安全”；
- **节点/边时序计数** 决定“是否足够稳定，可以持久或删除”；
- **连通性 precommit 检查** 决定“删除旧拓扑会不会把当前可达图切断”。

因此看到一个旧节点或旧边还存在，并不一定是维护失败：它可能已经物理失效而被 `static_valid=false` 屏蔽，也可能正在等待稳定替代拓扑；反过来，一条静态边被动态物体挡住，也不应该从持久图中消失。判断问题时应同时查看节点 source、当前匹配状态、生命周期计数和 `GraphEdgeState`，不能只看 RViz 中是否画出一条线。
