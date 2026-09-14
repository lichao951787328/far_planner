# FAR Planner：从障碍轮廓提取到全局图构建与维护的完整流程

## 1. 总体结论

FAR Planner 采用的不是“每一帧重新构建整张全局图”，也不是“只保留机器人附近的局部图”，而是：

> 使用局部实时点云重建当前轮廓图，将当前轮廓与历史导航节点进行数据关联，然后只更新机器人附近的节点和边；未受当前观测影响的远处节点和边继续保存在全局导航图中，最终路径搜索在完整的全局图上执行。

完整数据流为：

```text
里程计与地形/扫描点云
        ↓
更新全局自由点和障碍点栅格
        ↓
从全局栅格取机器人周围点云
        ↓
障碍点云投影为局部二维占据图
        ↓
提取、简化障碍轮廓
        ↓
构造本轮临时 ContourGraph 和 CTNode
        ↓
分析角点方向、凸凹性并关联地形高度
        ↓
从全局 NavGraph 中选出当前附近的历史节点
        ↓
当前 CTNode 与历史 NavNode 匹配
        ↓
从未匹配 CTNode 中筛选真正需要新增的节点
        ↓
局部更新历史节点、可见性边、轮廓边和轨迹边
        ↓
通过多帧投票确认或删除节点和边
        ↓
更新持久化全局 NavGraph 和全局稀疏轮廓
        ↓
在完整全局 NavGraph 上搜索路径
```

系统中几类主要数据的范围和生命周期如下：

| 数据 | 范围 | 生命周期 |
|---|---|---|
| `surround_obs_cloud_`、`surround_free_cloud_` | 机器人周围 | 随机器人更新 |
| `terrain_height_grid_` | 机器人周围 | 局部滚动重建 |
| `contour_polygons_`、`contour_graph_`、`CTNode` | 当前局部观测 | 每轮清空并重建 |
| `world_obs_cloud_grid_`、`world_free_cloud_grid_` | 大范围世界坐标 | 跨帧累计 |
| `globalGraphNodes_`、`NavNode` 和历史图边 | 全局 | 跨帧持久化并增量维护 |
| GraphPlanner 搜索图 | 完整全局导航图 | 每轮接收最新全局图 |

---

## 2. 第一步：接收里程计并统一坐标系

系统订阅 `/odom_world`，将机器人位姿转换到配置的 `world_frame`，默认是 `map`。

此后使用的点云、轮廓点、CTNode、NavNode 和图边都采用同一世界坐标系。机器人位置同时写入：

```text
robot_pos_
FARUtil::robot_pos
```

机器人第一次获得里程计时，还会确定世界点云栅格的原点。世界栅格原点设置完成后不会像局部地形图一样持续跟随机器人移动。

---

## 3. 第二步：预处理地形点云并区分自由点和障碍点

在解释 FAR Planner 怎样处理点云前，首先需要把 launch 文件中的话题重映射和两个上游地形分析节点串起来。FAR Planner 源码内部订阅的是：

```text
/terrain_cloud
/terrain_local_cloud
/scan_cloud
```

但 `far_planner.launch` 默认进行了如下重映射：

```text
/terrain_cloud       ← /terrain_map_ext
/terrain_local_cloud ← /terrain_map
/scan_cloud          ← /registered_scan
```

因此，从系统实际运行的话题看，FAR Planner 有三路点云输入：

| FAR 内部订阅名 | 默认实际话题 | 上游来源 | FAR 中的主要作用 |
|---|---|---|---|
| `/terrain_cloud` | `/terrain_map_ext` | `terrain_analysis_ext` | 主地形点云；更新全局自由/障碍点云栅格，生成局部轮廓，建立局部高度图，发现环境变化 |
| `/terrain_local_cloud` | `/terrain_map` | `terrain_analysis` | 动态模式下的近场地形障碍图；用于局部 TerrainPlanner 验证轨迹节点和轨迹边 |
| `/scan_cloud` | `/registered_scan` | 配准后的激光点云来源 | 动态模式下做射线追踪，发现全局障碍栅格中已经消失或移动的旧障碍点 |

三路点云不是彼此独立生成的。实际生产关系是：

```text
                         ┌──────────────────────────────┐
/registered_scan ───────→│ terrain_analysis             │
/Odometry ──────────────→│ 近场地形分析                  │
                         └──────────────┬───────────────┘
                                        │ /terrain_map
                                        ↓
                         ┌──────────────────────────────┐
/registered_scan ───────→│ terrain_analysis_ext         │
/Odometry ──────────────→│ 近场 terrain_map + 远场扩展  │
                         └──────────────┬───────────────┘
                                        │ /terrain_map_ext
                                        ↓
                                  FAR Planner 主地形输入

/terrain_map ─────────────────────→ FAR Planner 动态近场输入
/registered_scan ─────────────────→ FAR Planner 动态射线输入
```

也就是说：

- `terrain_analysis` 从注册激光点云产生近场 `/terrain_map`；
- `terrain_analysis_ext` 同时读取注册激光点云和 `/terrain_map`，把近场结果与自己生成的远场结果融合，发布 `/terrain_map_ext`；
- FAR Planner 把 `/terrain_map_ext` 作为主要建图和轮廓输入；
- 在动态模式下，FAR 还直接使用 `/terrain_map` 和 `/registered_scan` 完成更局部、更及时的图维护。

### 3.1 `/registered_scan`：配准后的原始几何扫描

`/registered_scan` 是已经配准到地图/世界坐标附近的激光点云，是两级 terrain analysis 的原始点云来源，也是 FAR 动态障碍清理的直接输入。

`terrain_analysis` 和 `terrain_analysis_ext` 收到它后，首先按照机器人相对高度和水平距离裁剪，并把各自内部副本的 `intensity` 临时改成：

```text
当前扫描时间 - 系统启动时间
```

这个临时时间戳用于体素缓存中的点云衰减。它不会改变 ROS 中其他订阅者收到的消息，也不是最后发布给 FAR 的地形语义强度。

FAR Planner 直接订阅 `/registered_scan` 时只在动态模式使用，而且只使用点的空间位置构造扫描占据格和射线格；这里没有把 scan 的 `intensity` 当成自由/障碍高度判断依据。

FAR 对这路点云的用途是：

```text
当前注册扫描
→ 从机器人到每个当前回波做射线追踪
→ 射线路径标记为当前可见空间
→ 检查历史 surround_obs_cloud_ 中是否有点落在这些射线上
→ 如果有，说明历史障碍现在可能已移走
→ 作为 cur_dyobs_cloud_ 从全局障碍栅格中删除
```

因此这路点云主要解决的是“旧障碍已经消失”的负观测问题。只使用 `/terrain_map_ext` 的累积障碍点，很难仅凭新一帧没有看到旧点就判断旧障碍应被删除，而射线可以提供当前位置确实已经被看到为空的证据。

### 3.2 `/terrain_map`：`terrain_analysis` 生成的近场地形语义点云

`terrain_analysis` 同时读取 `/registered_scan` 和里程计，在机器人周围维护一个随车滚动的点云体素窗口。其主要处理过程是：

1. 按机器人相对高度和距离裁剪注册点云；
2. 把多帧扫描堆叠到随车移动的 terrain voxel 中；
3. 对体素中的点降采样；
4. 按时间衰减旧点，近距离点可以按参数免衰减；
5. 将三维点投影到二维 planar voxel；
6. 每个平面格通过最低点或高度分位数估计地面高度；
7. 计算每个点相对该格地面的高度 `disZ`；
8. 过滤高度不合理、样本不足或被其动态障碍规则抑制的点；
9. 发布 `/terrain_map`。

`/terrain_map` 的点字段语义为：

```text
x、y、z   = 点在 map 坐标系中的真实空间位置
intensity = 点相对当前平面格估计地面的高度差 disZ
```

当前 launch 中 `considerDrop=true`，所以 `terrain_analysis` 输出时使用的是：

```text
intensity = |point.z - estimated_ground_z|
```

这里的 `intensity` 不再是激光反射率，而是人为编码的地形高度语义。

如果启用 `noDataObstacle`，上游还可能在无数据平面格中生成合成点，并把其 `intensity` 设置为 `vehicleHeight`，用来把无数据区域表达成障碍。当前提供的 launch 默认 `noDataObstacle=false`，所以默认不会生成这类无数据障碍点。

`/terrain_map` 有两个消费者：

```text
消费者 1：terrain_analysis_ext
用途：保留近场高质量结果，并与扩展版的远场结果融合。

消费者 2：FAR Planner 的 /terrain_local_cloud
用途：仅在 is_static_env=false 时，建立局部 TerrainPlanner 占据图，
      检查轨迹节点是否被占据以及历史轨迹边是否仍可绕行。
```

FAR 收到 `/terrain_map` 后仍按自己的 `kFreeZ` 再分成：

```text
intensity < kFreeZ   → local_terrain_free_
intensity ≥ kFreeZ   → local_terrain_obs_
```

其中实际进入 `TerrainPlanner::SetLocalTerrainObsCloud()` 的是 `local_terrain_obs_`。这些障碍点会在 TerrainPlanner 的二维网格中按照 `obs_inflate_size` 膨胀，然后用于：

- 判断附近轨迹 NavNode 是否已经落入障碍；
- 在两个历史轨迹节点附近运行局部 A*；
- 如果连续找不到绕行路径，逐步删除对应 `trajectory_connects`。

虽然代码也生成了 `local_terrain_free_`，但当前 FAR Planner 源码中没有继续使用这组点。

### 3.3 `/terrain_map_ext`：`terrain_analysis_ext` 生成的近场与远场融合点云

`terrain_analysis_ext` 不是简单地对 `/terrain_map` 再做一次滤波。它同时订阅：

```text
/registered_scan
/terrain_map
/Odometry
```

它维护一个范围更大的随车滚动体素窗口，并对远场点重新估计地面高度。可选的 `checkTerrainConn` 会以车下方地面为种子执行 BFS，只保留高度连续的主地面区域，从而抑制天花板、悬空平台或与机器人所在主地面不连通的结构。

最终输出由两部分拼接：

```text
距离机器人 ≤ localTerrainMapRadius：
    直接复制 /terrain_map 中的近场点及其 intensity

距离机器人 > localTerrainMapRadius：
    使用 terrain_analysis_ext 自己的远场地面估计结果，
    intensity = |point.z - estimated_ground_z|
```

当前 `terrain_analysis_ext.launch` 中：

```text
localTerrainMapRadius = 4.0 m
vehicleHeight         = 1.5 m
checkTerrainConn      = false（launch 参数默认值）
```

所以 `/terrain_map_ext` 可以理解为：

> 近场采用 `terrain_analysis` 的精细结果，远场采用 `terrain_analysis_ext` 的大范围估计，最终统一发布的地形语义点云。

其点字段同样为：

```text
x、y、z   = map 坐标系中的点位置
intensity = 相对估计地面的高度差 disZ
```

### 3.4 FAR 为什么需要同时订阅 `/terrain_map_ext` 和 `/terrain_map`

两者虽然有近场重叠，但在 FAR 中进入的是两条不同用途的处理链：

```text
/terrain_map_ext
    范围更大
    ↓
FAR::TerrainCallBack
    ↓
主地图更新、自由/障碍分类、轮廓提取、CTNode 高度关联、
普通 visibility edge 的地形检查和新障碍发现

/terrain_map
    近场更细、时间衰减更短
    ↓
FAR::TerrainLocalCallBack
    ↓
local_terrain_obs_
    ↓
FAR 内部 TerrainPlanner
    ↓
动态模式下重新验证历史轨迹节点和轨迹边
```

第一条链路处理全局拓扑图的主要环境表示。第二条链路不参与轮廓提取，只服务于动态模式下的轨迹通道维护。

FAR 收到 `/terrain_map` 后，将其中的障碍点送入：

```cpp
terrain_planner_.SetLocalTerrainObsCloud(FARUtil::local_terrain_obs_);
```

这些点在 FAR 内部 TerrainPlanner 的二维栅格中膨胀并标记为占据。之后，系统用这张局部占据图执行两项检查：

1. 判断附近历史轨迹 NavNode 是否已经落入障碍；
2. 在两个历史轨迹节点之间运行局部 A*，检查它们附近是否仍存在可通行路径。

其维护过程为：

```text
历史 trajectory edge
        ↓
在 /terrain_map 构造的近场占据图中运行局部 A*
        ↓
找到路径                    找不到路径
    ↓                           ↓
降低该轨迹边的无效计数       增加该轨迹边的无效计数
                                ↓
                         连续失败超过阈值
                                ↓
                         删除 trajectory_connect
```

因此，`/terrain_map` 直接输入 FAR 的核心原因不是再次生成普通轮廓，而是为机器人历史走过的轨迹通道提供单独的近场可通行性复查。

#### 3.4.1 为什么不直接从 `/terrain_map_ext` 截取同样的近场

从数据内容上看，这两路点云确实存在重叠，因为 `/terrain_map_ext` 在 `localTerrainMapRadius` 内直接复制 `/terrain_map`。理论上可以只订阅 `/terrain_map_ext`，再由 FAR 自己裁剪一份近场点云，但当前设计保留单独 `/terrain_map` 有以下实际原因。

第一，两个上游地图的尺度和时间特性不同。

当前 launch 默认参数为：

```text
terrain_analysis：
    scanVoxelSize = 0.05 m
    decayTime     = 2 s

terrain_analysis_ext：
    scanVoxelSize = 0.1 m
    decayTime     = 10 s
```

因此基础 `/terrain_map` 更细、旧点衰减更快；扩展 `/terrain_map_ext` 范围更大、远场更粗、历史缓存更长。全局轮廓和普通 NavGraph 希望环境表示范围大且相对稳定，而历史轨迹边复查更关心近场障碍是否刚刚堵住通道，适合直接使用更新更快的 `/terrain_map`。

第二，`/terrain_map_ext` 只在机器人 4 m 半径内原样复制 `/terrain_map`，4 m 外使用扩展模块自己的远场估计：

```text
距离机器人 ≤ 4 m
→ 直接复制 /terrain_map

距离机器人 > 4 m
→ terrain_analysis_ext 重新估计，采用较粗、较长时的远场数据
```

而基础 `/terrain_map` 的有效区域大约由中心局部地形窗口决定，并不严格只到 4 m。FAR 内部 TerrainPlanner 检查一段历史轨迹边时，所需区域可能跨出扩展模块的 4 m 原样复制区域。直接订阅 `/terrain_map` 可以继续获得这部分基础近场算法生成的细粒度数据，而不是在 4 m 外立即切换到扩展版结果。

第三，FAR 内部 TerrainPlanner 不一定严格以机器人当前位置为中心。它会在当前轨迹中间节点变化时调用：

```text
terrain_planner_.UpdateCenterNode(cur_internav_ptr_)
```

将局部规划栅格中心设置为当前 `cur_internav_ptr_`。这个节点是历史轨迹图中的连接中心，可能与机器人当前位置有一定距离。TerrainPlanner 要验证的是“一段历史轨迹边附近是否仍有通路”，而不只是“机器人脚下附近是否有障碍”。因此它需要一份独立的近场地形障碍输入，而不能简单等同于主轮廓链路当前使用的局部截取结果。

第四，两路数据对应不同的维护时间尺度：

```text
/terrain_map_ext
→ 服务稳定的中长期轮廓和全局拓扑维护

/terrain_map
→ 服务较及时的近场轨迹边重新验证
```

所以 `/terrain_map` 不是 `/terrain_map_ext` 的平行替代，而是同时具有两个身份：

```text
作为 terrain_analysis_ext 的输入：
→ 给扩展地图提供高质量近场部分

作为 FAR Planner 的独立输入：
→ 给内部 TerrainPlanner 提供近场障碍，维护轨迹节点和轨迹边
```

#### 3.4.2 这两路输入是否存在工程冗余

存在部分冗余。`/terrain_map_ext` 的 4 m 近场部分本来就来自 `/terrain_map`，FAR 又单独订阅了一次 `/terrain_map`。这不是因为扩展地图完全没有近场数据，而是当前架构为“主拓扑图维护”和“轨迹边局部复查”分别保留了接口。

理论上可以重构为：

```text
只订阅 /terrain_map_ext
→ FAR 内部再裁剪近场
→ 同时供主图和 TerrainPlanner 使用
```

但这样会使 TerrainPlanner 在 4 m 外使用扩展版较粗、长时缓存的数据，并失去直接使用基础 `/terrain_map` 的短衰减特性。若要这样重构，需要重新评估：

- TerrainPlanner 实际所需空间范围；
- 4 m 融合边界是否足够；
- 轨迹边对动态障碍响应的延迟；
- 基础版和扩展版分辨率差异；
- 两路点云的时间同步。

当前源码中还存在一个相关事实：FAR 将 `/terrain_map` 分成了 `local_terrain_free_` 和 `local_terrain_obs_`，但真正传给内部 TerrainPlanner 的只有 `local_terrain_obs_`；`local_terrain_free_` 生成后没有后续消费者。

### 3.5 FAR 如何解释两路 terrain 点云的 `intensity`

FAR 不重新计算 `disZ`，而是直接按照：

```text
intensity < kFreeZ   → 自由地面点
intensity ≥ kFreeZ   → 障碍点
```

分类。默认：

```text
kFreeZ = 0.2 m
```

所以从 FAR 的视角看：

```text
离估计地面高度小于 0.2 m 的点
→ 地面/自由点

离估计地面高度不小于 0.2 m 的点
→ 障碍点
```

这一点非常关键：如果错误地给 FAR 输入普通 XYZI 激光点云，并让 `intensity` 仍表示反射率，那么 FAR 会把反射率与 0.2 比较，导致自由点/障碍点语义完全错误。`/terrain_cloud` 和 `/terrain_local_cloud` 必须是已经把 intensity 编码成离地高度的 terrain 点云。

上游 `terrain_analysis`/`terrain_analysis_ext` 的 `vehicleHeight` 默认是 1.5 m，它控制哪些离地高度点会被保留到 terrain 输出；FAR 自己的 `vehicle_height` 默认是 0.75 m，主要用于把图节点放到地面上方相应高度及检查图边地形。这两个同名概念作用不同，参数不一致不代表代码会自动完成换算，实际部署时应根据机器人和点云定义统一校准。

### 3.6 静态模式与动态模式下三路点云是否都生效

默认配置为：

```yaml
is_static_env: true
```

此时 FAR Planner 的回调逻辑会直接忽略：

```text
/terrain_local_cloud（实际 /terrain_map）
/scan_cloud（实际 /registered_scan）
```

真正驱动 FAR 建图和轮廓提取的只有：

```text
/terrain_cloud（实际 /terrain_map_ext）
```

动态模式 `is_static_env=false` 时三路都会生效：

```text
/terrain_map_ext
→ 主地图、轮廓、节点和普通边更新

/terrain_map
→ 近场 TerrainPlanner、轨迹节点/轨迹边验证

/registered_scan
→ 扫描射线、识别和删除已经消失的历史障碍
```

需要注意，`terrain_analysis_ext` 本身无论 FAR 是静态还是动态模式，都仍然可能读取 `/terrain_map` 和 `/registered_scan` 来生成 `/terrain_map_ext`。这里“忽略”只表示 FAR 自己的两个直接辅助回调不处理它们，不表示上游扩展地形节点不使用它们。

`/terrain_cloud` 到达后依次执行：

1. ROS `PointCloud2` 转换成 PCL 点云；
2. 按 `voxel_dim` 降采样；
3. 删除 NaN 和 Inf；
4. 通过 TF 转换到 `world_frame`；
5. 以机器人为中心按照 `terrain_range` 裁剪；
6. 根据点的 `intensity` 区分自由地面点和障碍点。

分类条件为：

```cpp
if (point.intensity < FARUtil::kFreeZ)
    // 自由地面点
else
    // 障碍点
```

默认主要参数为：

```text
voxel_dim          = 0.15 m
robot_dim          = 0.8 m
vehicle_height     = 0.75 m
sensor_range       = 30 m
terrain_range      = 15 m
local_planner_range = 5 m
```

这里得到当前输入中的：

```text
temp_free_ptr_    自由地面点
temp_obs_ptr_     障碍点
```

---

## 4. 第三步：更新全局点云栅格，再提取机器人周围点云

自由点和障碍点分别写入：

```text
world_free_cloud_grid_
world_obs_cloud_grid_
```

这两个栅格是大范围的世界坐标栅格。机器人离开一个区域后，该区域过去写入的静态点云仍然保留，因此静态环境下它们承担全局累计地图的作用。

但轮廓提取不会使用整张世界地图。系统根据机器人当前所在 cell，只组合附近 cell 中的点，输出：

```text
surround_free_cloud_    当前机器人周围的自由点
surround_obs_cloud_     当前机器人周围的障碍点
```

所以这一阶段的结构是：

```text
当前帧点云
    ↓ 写入
全局点云栅格
    ↓ 只读取机器人周围 cell
局部 surround 点云
```

也就是说，点云存储具有全局性，但轮廓感知是局部的。

---

## 5. 第四步：根据自由点建立局部滚动地形高度图

`surround_free_cloud_` 被用于更新 `terrain_height_grid_`。该栅格的中心随机器人移动，所以它是一张局部滚动地形图，而不是永久的全局高度图。

系统从机器人所在格开始执行四邻域 flood-fill。只有满足以下条件的相邻格才会继续扩展：

- 有地形观测；
- 高度与当前格连续；
- 高度差小于阈值。

这样得到的是与机器人当前所在位置连通的可通行地面，而不是所有被观测到的地面。

该地形图后面用于：

1. 给二维轮廓角点恢复三维高度；
2. 判断新节点是否位于机器人可达地形上；
3. 检查图边附近的坡度和高度连续性；
4. 动态环境中重新验证历史轨迹边。

---

## 6. 第五步：创建或更新机器人 odom 图节点

主循环通过 `DynamicGraph::UpdateRobotPosition()` 维护一个特殊的 odom NavNode。

第一次运行时创建：

```text
is_odom = true
```

并加入全局导航图。之后不再重复创建，只更新其位置。该节点代表全局 Graph 路径搜索的当前起点。

---

## 7. 第六步：把周围障碍点云投影为局部二维占据图

系统清空上一轮图像，然后创建一张以机器人为中心的二维矩阵：

```text
MAT_SIZE ≈ 2 × sensor_range / voxel_dim
```

默认约为 400 个像素，并调整为奇数，使机器人位于图像中心。

坐标映射关系为：

```text
世界坐标 x → 图像 row
世界坐标 y → 图像 col
机器人位置 → 图像中心
```

每个障碍点不仅写入一个像素，还写入周围 `3×3` 像素。这会填补部分离散点云间隙并产生轻量障碍膨胀。

动态模式下还会按照 `CDetector/filter_count_value` 对像素计数做阈值化，过滤观测数量不足的孤立点。之后图像被放大并执行 `boxFilter`，让像素轮廓更平滑，方便后续多边形拟合。

### 这一阶段能够提供多少碰撞安全性

这里的 `3×3` 写入和图像平滑只提供有限的隐式膨胀。它没有严格按照机器人的圆形或矩形 footprint 对障碍做 configuration-space 膨胀，也没有检查机器人沿某条轮廓线运动时的完整扫掠体积。

因此，从这一步得到的轮廓只能理解为“经过轻量处理的障碍几何边界”，不能理解为“机器人中心沿该轮廓运动一定安全”。

---

## 8. 第七步：提取并简化障碍轮廓

系统使用 OpenCV：

```cpp
cv::findContours(
    image,
    raw_contours,
    hierarchy,
    cv::RETR_TREE,
    cv::CHAIN_APPROX_TC89_L1);
```

`RETR_TREE` 保留轮廓父子关系，`CHAIN_APPROX_TC89_L1` 对像素边界进行初步简化。

随后使用 `approxPolyDP`，通过 Ramer-Douglas-Peucker 算法把密集边界进一步压缩成多边形折点。例如一面长直墙上的大量像素边界点，最后通常只保留端点和真正的转折点。

之后执行三类过滤：

1. 根据轮廓层次和机器人自由位置过滤不需要的内部嵌套轮廓；
2. 删除距离前一个保留顶点过近的点；
3. 连续三个点近似共线时，删除中间的直墙点。

过滤后少于三个顶点的轮廓会整体删除。

这里需要准确区分“噪声”和“后续不创建节点的候选”：

- 像素锯齿产生的密集邻近点，可以称为几何噪声或几何冗余；
- 直墙上的共线中间点不是传感器噪声，而是对稀疏拓扑图没有贡献的冗余点；
- 后续没有加入全局图的有效角点，也不一定是噪声，可能只是缺少足够的新环境证据。

最后，轮廓点从图像坐标转换回世界坐标，形成：

```text
realworld_contour_[0]
realworld_contour_[1]
...
```

此时 `z` 暂时采用机器人高度，稍后由地形图修正。

---

## 9. 第八步：用当前轮廓从零构造本轮 ContourGraph

进入 `ContourGraph::UpdateContourGraph()` 后，首先清空上一轮：

```text
contour_graph_
contour_polygons_
polys_ctnodes_
```

再由本轮 `realworld_contour_` 重新创建。

因此：

> ContourGraph 是当前观测周期的临时局部测量，不是跨帧保存的历史图。

每条轮廓首先转换成一个 `Polygon`，记录顶点、周长、是否包含机器人以及是否为 pillar。

周长不大于：

```text
kPillarPerimeter = robot_dim × 4
```

的轮廓被认为是小型柱状障碍。默认阈值约为 3.2 m。这类轮廓只在多边形中心创建一个 `PILLAR CTNode`。

对于普通多边形，每个折点创建一个 CTNode，并通过：

```text
front
back
```

组成闭环：

```text
CT0 ↔ CT1 ↔ CT2 ↔ ... ↔ CT0
```

这里的 `front/back` 只描述当前帧中同一障碍轮廓上的顺序关系。

---

## 10. 第九步：分析每个 CTNode 的表面方向和凸凹性

系统从 CTNode 沿 `front`、`back` 两个方向寻找距离足够远的轮廓点，计算角点两侧的墙面方向：

```text
surf_dirs.first
surf_dirs.second
```

如果邻接点过近，就继续沿轮廓向前查找，直到方向估计足够稳定。

然后根据两条表面方向和多边形内部关系，把 CTNode 分类为：

```text
CONVEX     凸角
CONCAVE    凹角
PILLAR     小型独立障碍
UNKNOW     基本属于直墙或无法得到有效角点方向
```

`UNKNOW` CTNode 可以继续帮助当前 Polygon 表达几何形状，但不会作为新增的全局 NavNode。

因此，“不满足轮廓几何条件”在这里具体指：

- 两侧表面接近一条直线，没有形成有意义的角；
- 表面方向无法稳定确定；
- 最终 `free_direct == UNKNOW`。

它们并不一定是错误点，更准确地说是“不适合作为稀疏拓扑导航节点的轮廓点”。

---

## 11. 第十步：根据地形高度修正 CTNode 和历史 NavNode

二维轮廓提取完成后，系统在局部地形高度图中查找 CTNode 附近地面，将其高度设置为：

```text
CTNode.z = 地面高度 + vehicle_height
```

同时限制在机器人当前楼层的高度容差范围内，避免错误关联到楼上或楼下。

系统也会调整当前局部范围内历史 NavNode 的高度，使轮廓节点和历史导航节点在三维高度上可以正确匹配。

---

## 12. 第十一步：从全局 NavGraph 中选择本轮局部候选节点

全局导航节点长期保存在：

```text
DynamicGraph::globalGraphNodes_
```

每轮调用 `UpdateGlobalNearNodes()` 扫描全局节点，然后根据机器人位置划分为：

```text
extend_match_nodes_    用于和当前 CTNode 匹配、重新验证的历史节点
near_nav_nodes_        本轮主要进行两两连边检查的节点
wide_near_nodes_       用于检查 odom 到周围节点连接的较宽集合
margin_near_nodes_     位于局部感知边缘的节点
internav_near_nodes_   附近轨迹节点
```

需要注意，代码中 `IsPointInLocalRange()` 虽然名字包含 `LocalRange`，实际距离阈值使用的是 `sensor_range`，默认约 30 m，不是 `local_planner_range = 5 m`。

远处全局节点不会因为没有出现在当前轮廓中而被删除；它们只是暂时不参加本轮主要匹配和两两连边更新。

---

## 13. 第十二步：当前 CTNode 与历史 NavNode 数据关联

匹配开始前，所有 NavNode 上一轮的临时 CTNode 关联被清空：

```text
NavNode.is_contour_match = false
NavNode.ctnode = nullptr
```

这只表示“本轮尚未匹配”，不会删除 NavNode，也不会立即删除它的历史边。

随后对每个当前 CTNode，在 `extend_match_nodes_` 中寻找合适的历史 NavNode。匹配同时考虑：

1. 三维高度是否接近；
2. pillar 是否只与 pillar 匹配；
3. 凸凹类型是否兼容；
4. 两侧表面方向是否相似；
5. 距离是否小于匹配阈值；
6. CTNode 与 NavNode 之间的关联线是否穿过当前 Polygon、边界或有关历史轮廓。

默认基础匹配距离为：

```text
kMatchDist = robot_dim × 2 + voxel_dim
           = 0.8 × 2 + 0.15
           = 1.75 m
```

实际阈值还会根据表面方向相似度缩小。两个点即使位置接近，如果角点朝向明显不同，也不会匹配。

所以匹配并不是完全“不管 ContourGraph”的最近邻匹配。当前 ContourGraph 在两处直接参与：

- 当前 CTNode 的 Polygon、凸凹类型和表面方向参与候选筛选；
- `IsCTMatchLineFreePolygon()` 使用当前 Polygon 和有关轮廓线检查 CTNode 到 NavNode 的匹配线是否可行。

匹配成功后建立本轮关联：

```text
CTNode.is_global_match = true
CTNode.nav_node_id     = NavNode.id

NavNode.is_contour_match = true
NavNode.ctnode            = CTNode
```

### ContourGraph 是否会随匹配结果延续到下一帧

不会。下一轮会先清空当前 ContourGraph，再根据新点云重新创建一批 CTNode 和 Polygon。

跨帧延续的是 NavNode 以及 NavNode 之间的边和投票。其时间关系是：

```text
t 时刻 CTNode/ContourGraph
        ↓ 匹配
固定 ID 的历史 NavNode
        ↓ 更新节点状态和边投票
全局 NavGraph
        ↓ 下一轮继续保留
t+1 时刻重新创建 CTNode/ContourGraph
        ↓ 再匹配到同一批或新的 NavNode
```

可以把它理解为：

```text
ContourGraph = 当前帧局部测量
NavGraph     = 跨帧全局状态
匹配过程     = 当前测量与历史状态的数据关联
```

---

## 14. 第十三步：从未匹配 CTNode 中筛选真正需要新增的节点

首先排除：

- 已经匹配历史 NavNode 的 CTNode；
- `free_direct == UNKNOW` 的直墙/无明确角点方向 CTNode；
- 非 pillar 但表面方向仍表现为直墙关系的 CTNode。

剩余的未匹配 CTNode 可能包括凸角、凹角和 pillar，但仍不会全部立即加入全局图。

`IsAValidNewNode()` 要求候选至少满足以下证据之一：

```text
附近存在近期新增或发生变化的障碍点
或
该 CTNode 是保持轮廓拓扑连续性所必需的点
```

同时还要求：

- 位置位于当前地形邻域；
- 高度可以关联到可通行地形。

这里没有通过的点不能统一称为“噪声点”，应该分成：

1. 图像锯齿、邻近重复点：几何噪声或冗余；
2. 直墙中间点：几何有效但拓扑无用；
3. `UNKNOW` CTNode：无法形成有效导航角点；
4. 有效角点但附近没有新障碍证据：暂时未确认的新节点候选；
5. 无法关联可通行地形：三维地形无效候选。

这种筛选主要防止局部轮廓轻微抖动时，全局图在同一角点附近反复生成重复节点。

---

## 15. 第十四步：将有效 CTNode 转换为持久化 NavNode

通过筛选的 CTNode 被转换成新的 NavNode，复制：

```text
position
surf_dirs
free_direct
ctnode
```

并分配递增的全局 ID：

```text
NavNode.id = id_tracker_++
```

随后加入：

```text
globalGraphNodes_
idx_node_map_
```

从这一刻开始，NavNode 就不再依赖当前 CTNode 的生命周期。下一轮当前 CTNode 被清空后，NavNode 仍然存在，并尝试匹配下一轮重新检测到的角点。

全局图由此逐步增长：

```text
局部轮廓角点
→ 与历史节点去重匹配
→ 有证据的新角点转换成 NavNode
→ NavNode 永久进入全局图，直到被维护逻辑确认失效
```

---

## 16. 第十五步：必要时增加机器人历史轨迹节点

除了障碍角点节点，系统还维护 `is_navpoint = true` 的轨迹节点。

当机器人已经离开当前轨迹连接位置，或者当前轨迹节点不再适合作为局部连接中心时，系统在历史自由位置创建一个轨迹 NavNode，并通过：

```text
trajectory_connects
```

连接前后轨迹节点。

这使全局图在空旷区域、长走廊、轮廓稀少区域和动态变化区域中仍然保留机器人已经实际走过的拓扑通道。

---

## 17. 第十六步：重新估计当前附近历史节点的位置、方向和凸凹性

系统对 `extend_match_nodes_` 中的历史节点执行重新验证。

如果历史 NavNode 本轮匹配到了 CTNode，就把当前观测位置和表面方向加入：

```text
pos_filter_vec
surf_dirs_vec
```

然后使用 RANSAC 计算稳定位置和稳定表面方向。默认滤波池最多保存 12 次观测；位置和方向的有效内点数超过 `node_finalize_thred` 后，节点成为：

```text
is_finalized = true
```

finalized 节点不会因普通小幅轮廓抖动持续移动。

如果节点附近出现新的障碍变化点，系统会重置 finalized、位置/方向滤波和部分边投票，使节点可以适应新环境。

---

## 18. 第十七步：延迟删除持续无法匹配的历史节点

历史节点本轮没有匹配到 CTNode 时，不会立即删除。

如果节点位于感知边缘，轮廓可能只是被图像范围截断，因此通常继续保留。

如果节点位于当前可靠感知范围内部，或者附近明确发生了环境变化，却连续无法匹配当前轮廓，则：

```text
clear_dumper_count += 1
```

重新匹配成功时，该计数逐步减小。

计数超过阈值后，节点被标记为 `is_merged`。这个名称容易造成误解：当前实现并没有真正把两个普通节点融合成一个节点，而是把该节点标记后从图中删除。

删除时会对称清理：

```text
connect_nodes
poly_connects
contour_connects
trajectory_connects
edge_votes
contour_votes
trajectory_votes
ID 映射
全局轮廓集合中的相关边
globalGraphNodes_ 中的节点
```

普通节点默认需要连续失效超过约 4 个更新周期；轨迹节点采用更高阈值，防止机器人实际走过的历史路径被轻易清除。

---

## 19. 第十八步：更新 odom 到周围节点的连接

系统先检查当前 odom 节点与：

```text
wide_near_nodes_
本轮新创建节点
```

之间的连接。

普通可见性连接需要满足：

1. 节点凸凹类型允许；
2. 连接方向位于角点自由空间方向；
3. 连接线不穿过当前局部 Polygon；
4. 不穿过当前未匹配或 inactive 历史轮廓；
5. 必要时不穿过全局历史轮廓；
6. 不穿过 boundary；
7. 地形高度、坡度和中点地面连续性允许；
8. 没有方向近似相同但更短的冗余连接。

通过后加入：

```text
poly_connects
connect_nodes
```

失败时更新无效票，并根据投票结果决定是否删除实际边。

odom 边使用更短的投票窗口。默认总窗口为 10，而 odom 边约使用 `ceil(10/3)=4`，因此机器人当前连接能更快响应变化。

---

## 20. 第十九步：正式加入新节点，并重建局部范围内的轮廓边

本轮新 NavNode 被正式加入全局图和当前 near 集合。随后系统利用当前 CTNode 的 `poly_ptr`、`front` 和 `back` 判断两个 NavNode 是否位于同一条障碍轮廓上。

假设：

```text
NavA ↔ CTA
NavB ↔ CTB
```

如果 `CTA` 和 `CTB`：

- 属于同一个当前 Polygon；
- 沿 `front/back` 可以连续到达；
- 中间没有其他已匹配 CTNode 阻断；
- 中间轮廓没有明显偏离 A、B 连线；
- 没有与 boundary 冲突；

就给 `NavA—NavB` 的 `contour_votes` 写入有效票，否则写入无效票。

每个 NavNode 最后只保留得票最高的两个轮廓邻居，对应多边形顶点的前后两个主要邻接方向。

确认后的轮廓边加入：

```text
contour_connects
connect_nodes
global_contour_set_
```

### 轮廓边的碰撞安全边界

轮廓边表达的是障碍轮廓上的拓扑邻接，并不是经过完整机器人 footprint 验证的轨迹。

`IsCTNodesConnectFromContour()` 主要检查当前同一 Polygon 上的轮廓顺序和 boundary，没有像普通可见性边那样完整检查该轮廓边是否与另一个独立当前 Polygon 相交。

因此，如果一个新的独立动态障碍突然横穿一条历史 contour edge，该边能否及时失效主要依赖：

- 新障碍改变当前轮廓拓扑；
- 原 NavNode 与 CTNode 的匹配关系变化；
- `contour_votes` 连续获得无效票；
- 下游局部规划器进行实时避障。

Graph 层本身不提供有限尺寸机器人沿轮廓边运动的即时、严格安全证明。

---

## 21. 第二十步：更新局部范围内的普通可见性边

系统对 `near_nav_nodes_` 两两调用 `IsValidConnect()`。

### 21.1 凸凹约束

普通跨自由空间连接通常不允许任一端是 `CONCAVE`。凹角仍可用于表达轮廓拓扑，但不作为普通视线直连端点。

### 21.2 自由方向约束

根据 NavNode 的两条 `surf_dirs` 判断连接方向是否落在角点自由空间一侧。如果 A 指向 B 的方向落入障碍内部方向，则拒绝连接；两个端点都要检查。

### 21.3 当前 Polygon 和历史轮廓碰撞检查

系统先将边端点向自由空间方向轻微投影，主要避免连接线在数值上恰好擦过角点。投影距离通常为 `voxel_dim`，默认约 0.15 m，不等价于完整机器人半径膨胀。

如果边完全位于当前局部范围，主要检查：

```text
当前 contour_polygons_
unmatched_contour_
inactive_contour_
boundary_contour_
```

如果边跨出局部范围，则还会使用：

```text
global_contour_
```

进行历史全局轮廓检查。

### 21.4 地形检查

系统检查：

- 节点间高度差和水平距离；
- 坡度是否过大；
- 边中点附近是否有可关联地形；
- 中点附近最大、最小高度差是否过大；
- 连线高度是否与地面一致。

### 21.5 方向冗余过滤

如果一个节点已经有方向近似相同但距离更短的连接，则拒绝更长连接，使局部 visibility graph 保持稀疏。

### 21.6 为什么仍不能认为完成了严格机器人碰撞检查

这里主要检查的是投影后的二维线段与 Polygon/轮廓线是否相交，并补充地形约束。它没有对机器人完整 footprint 沿边移动进行连续碰撞检测。

因此总体安全链条是：

```text
点云图像轻量膨胀
+ 图边线段/多边形相交检查
+ 地形连续性检查
+ waypoint 向自由空间投影
+ 下游局部规划器实时避障
```

Graph 层提供的是拓扑可行性近似，而不是最终运动控制级别的绝对碰撞保证。

---

## 22. 第二十一步：历史普通边与当前 ContourGraph 相交时的处理

假设上一时刻存在普通可见性边：

```text
NavA ───────────────── NavB
```

当前新 Polygon 穿过该边：

```text
NavA ───── ███ 新障碍 ███ ───── NavB
```

如果 A、B 处于本轮局部更新集合中，`IsNavNodesConnectFreePolygon()` 会发现当前边与 `contour_polygons_` 相交，并给：

```text
edge_votes[A][B]
edge_votes[B][A]
```

写入一个 `0`。

但是该边不一定在第一次相交时立即从图中消失。普通边按照历史投票判断，例如：

```text
1 1 1 1 1 1 1 1 1 0
```

仍然可能被认为有效。新障碍持续存在后，队列逐步变成：

```text
1 1 1 1 0 0 0 0 0 0
```

有效票不再占优势，边才从 `poly_connects` 和 `connect_nodes` 中删除。

这是为了抑制单帧点云噪声，但也意味着动态障碍突然进入时存在更新延迟。

如果该边同时属于历史 `trajectory_connects`，普通 Polygon 检查失败后仍可能因为轨迹连接而暂时保留。动态模式会使用局部 TerrainPlanner 搜索两轨迹节点之间是否仍存在可行路径，连续验证失败超过阈值后才删除轨迹连接。因此轨迹边同样不是即时删除。

---

## 23. 第二十二步：维护局部范围与远处历史图之间的连接

机器人移动后，可能出现：

```text
当前 near 节点 ─── 历史远处节点
```

系统不会因为一端离开当前局部范围就自动删除这种边。它会重新检查当前近处节点到历史范围外节点的连接：

- 当前局部 Polygon 是否否定该连接；
- 当前和历史轮廓是否发生相交；
- 地形是否允许；
- 连接投票是否仍然有效。

这样可以避免机器人移动时把全局图切成互不连通的多个局部图。

---

## 24. 第二十三步：拼接当前局部轮廓和历史范围外轮廓

一条长墙不可能始终完整落在同一张局部轮廓图中。机器人沿墙运动时，会依次看到墙的不同部分。

系统使用：

```text
out_contour_nodes_
out_contour_nodes_map_
```

暂时记录可能位于当前局部范围外、但需要与当前轮廓继续连接的历史轮廓节点。

当前近处 NavNode 匹配到新 CTNode 后，系统判断它与范围外历史节点是否仍位于同一条连续轮廓线上。匹配成功就增加轮廓有效票，否则增加无效票。

所以全局轮廓不是通过永久保存每一帧 Polygon 获得的，而是：

```text
局部轮廓片段
→ 匹配到固定 ID 的 NavNode
→ NavNode 间 contour vote
→ 跨局部范围拼接
→ 形成持久化全局稀疏轮廓
```

---

## 25. 第二十四步：动态障碍进入时的完整维护过程

以下过程只在：

```yaml
is_static_env: false
```

时启用完整动态处理。

### 25.1 新障碍进入

新障碍首先出现在新的地形/障碍点云中，随后：

1. 新障碍点写入 `world_obs_cloud_grid_`；
2. 当前障碍点与之前的 `surround_obs_cloud_` 比较；
3. 新位置进入 `cur_new_cloud_`；
4. 变化点短期累计到 `stack_new_cloud_`；
5. 更新后的 `surround_obs_cloud_` 生成新的局部占据图；
6. 新障碍形成新的 Polygon 和 CTNode；
7. 穿过新 Polygon 的旧普通边获得无效票；
8. 新 CTNode 因靠近 `stack_new_cloud_`，可能被创建为新 NavNode；
9. 新障碍持续存在后，旧边投票逐步失效并被删除；
10. 受影响的旧节点取消 finalized，重新积累位置、方向和连接投票。

因此，新动态障碍进入后，Graph 不会整体重建，而是在当前局部范围内增量增加新节点、修改旧节点并逐步断开冲突边。

### 25.2 动态障碍移动或离开

`ScanHandler` 从机器人向当前扫描点执行三维射线追踪。如果世界障碍栅格中原来有障碍点，但当前扫描射线能够穿过该位置，说明该历史障碍点现在可能已经为空。

这些点进入 `cur_dyobs_cloud_`，随后：

1. 对动态变化点适当膨胀；
2. 从 `world_obs_cloud_grid_` 删除对应旧障碍；
3. 从当前 `surround_obs_cloud_` 删除；
4. 把变化位置加入 `cur_new_cloud_`，通知 Graph 附近环境发生变化；
5. 下一轮局部轮廓重新提取，原障碍轮廓消失或改变；
6. 原 NavNode 可能无法匹配新 CTNode；
7. 节点删除计数和边无效票逐步增加；
8. 持续失效后删除旧节点和旧边；
9. 原来被障碍阻断的普通可见性边经过新一轮投票后可能重新建立。

动态障碍移动实际上同时包含：

```text
新位置：作为新障碍加入
旧位置：通过当前 scan 射线证明为空，再从历史地图移除
```

### 25.3 动态维护不是即时安全制动

节点和边使用投票、滤波与连续失效计数，因此动态 Graph 更偏向稳定的中长期拓扑维护，而不是瞬时避障：

```text
新障碍被观测
→ 当前轮廓改变
→ 边得到无效票
→ 连续多轮确认
→ 删除旧边或旧节点
```

快速移动障碍的即时制动和避让仍然应由下游局部规划器基于实时传感器完成。仅依赖 FAR Planner 的全局 Graph，不能保证在投票尚未收敛时立即避开动态障碍。

---

## 26. 第二十五步：更新 covered 和 frontier 状态

所有附近节点和边更新后，系统判断节点是否已经被充分观测。

节点在以下情况下可能成为 `covered`：

- 是 odom 或轨迹节点；
- 距离机器人或附近轨迹节点足够近；
- 从机器人或轨迹节点到该节点存在稳定连接；
- 连接方向覆盖了该角点的自由空间方向。

随后判断 frontier。一个节点成为 frontier 通常要求：

- 当前仍匹配到轮廓；
- 是凸角；
- 所在 Polygon 周长足够大；
- 尚未被充分覆盖；
- 没有被 `is_block_frontier` 禁止；
- 附近存在近期新观测证据。

frontier 状态也经过多帧投票，避免单帧跳变。

---

## 27. 第二十六步：由确认后的 contour edge 生成全局稀疏轮廓

图更新完成后，系统遍历：

```text
global_contour_set_
```

生成：

```text
global_contour_      所有持久化历史轮廓边
inactive_contour_    当前局部范围内端点不活动的历史轮廓
unmatched_contour_   当前只有部分端点匹配局部 CTNode 的历史轮廓
boundary_contour_    外部导入的边界
local_boundary_      当前机器人附近的边界
```

这些线段会反过来参与后续 Graph 边的碰撞检查。

系统没有永久保存每一帧的完整 `contour_polygons_`；永久保存的是确认后的 NavNode 和 NavNode 间 contour edge。因此其全局环境表达是一张稀疏拓扑轮廓图，不是完整的全局多边形地图。

---

## 28. 第二十七步：生成最终供搜索使用的统一图邻接关系

NavNode 中存在三类带语义的连接：

| 字段 | 含义 |
|---|---|
| `poly_connects` | 通过自由空间可见性检查得到的边 |
| `contour_connects` | 当前/历史障碍轮廓上的拓扑边 |
| `trajectory_connects` | 机器人历史运动产生的轨迹关系 |
| `connect_nodes` | 最终供 GraphPlanner 搜索的统一邻接表 |

普通可见性边和有效轮廓边会进入 `connect_nodes`。轨迹关系也会在连接判断中用于维持机器人实际走过的通道。

至此，当前局部观测已经融合回长期保存的：

```text
globalGraphNodes_
```

远处未被当前观测影响的节点和边继续保留，当前附近节点和边根据新轮廓进行增加、修正、投票失效或删除。

---

## 29. 第二十八步：在完整全局 NavGraph 上进行路径搜索

主循环把完整 `globalGraphNodes_` 传给 GraphPlanner。路径搜索不是仅在 `near_nav_nodes_` 中执行，而是从当前 odom 节点沿全局 `connect_nodes` 扩展。

规划器计算两套可达状态：

```text
is_traversable
gscore
parent
```

表示在完整图中的普通可达性；以及：

```text
is_free_traversable
fgscore
free_parent
```

表示只经过已经充分观测的 `covered` 节点时的可达性。

目标加入 Graph 后，系统检查目标与全局可达节点的连接，然后在完整全局图上生成路径。

---

## 30. 第二十九步：把图节点转换成更安全的局部 waypoint

GraphPlanner 输出的下一个节点位置不一定原样发送给底层。

对于凸角节点，`ExtendViewpointOnObsCloud()` 根据节点两侧表面方向计算自由空间方向，并沿该方向查询附近障碍点，把 waypoint 尽量从原始轮廓角点向自由空间延伸。

这个过程使用与机器人尺寸相关的 `kNearDist` 进行点云查询，可以让实际发送的 waypoint 通常比原始 CTNode/NavNode 更远离障碍。

但它仍然不是对整条机器人轨迹执行严格 footprint 碰撞检测。因此最终架构隐含的职责划分是：

```text
FAR Planner：提供全局稀疏拓扑路径和局部 waypoint
下游局部规划器：根据实时传感器完成轨迹生成、机器人 footprint 检查和快速动态避障
```

---

## 31. 最终总结

FAR Planner 的完整工作方式可以概括为：

```text
全局累计点云栅格
        ↓ 局部读取
当前机器人周围障碍点云
        ↓ 每轮重建
临时 ContourGraph / CTNode
        ↓ 数据关联
持久化 NavNode
        ↓ 局部增量更新与多帧投票
全局可见性图和全局稀疏轮廓
        ↓
全局路径搜索
        ↓
自由空间方向上的 waypoint 修正
        ↓
下游局部规划和动态避障
```

其中最关键的几个结论是：

1. `ContourGraph` 是局部临时测量，每轮清空重建，不跨帧延续；
2. `NavGraph` 是全局持久状态，CTNode 通过匹配不断修正固定 ID 的 NavNode；
3. 轮廓的跨帧延续不是保留旧 CTNode，而是通过 NavNode ID、`contour_votes` 和 `contour_connects` 实现；
4. 历史普通边与当前 Polygon 相交时会获得无效票，连续确认后才删除，因此动态响应存在延迟；
5. contour edge 对另一个独立 Polygon 横穿的检查弱于普通可见性边，不能单独视为严格安全轨迹；
6. 所谓未加入全局图的“噪声点”不全是噪声，还包括直墙冗余点、无明确角点方向的点、缺少新环境证据的候选以及无法关联可通行地形的候选；
7. 图层主要提供点/线级拓扑可行性，而不是完整机器人 footprint 的连续碰撞证明；
8. 系统整体是“局部感知和局部增量维护，全球图持久保存和全局搜索”。

---

# 基于局部 3D 语义 Voxel 地图的 FAR Planner 二值输入改造方案

## 32. 改造目标和最终选择

现在新增的上游地图是一个局部 3D Semantic Voxel Map。每个有效 voxel 至少可以提供：

| 信息 | 含义 |
|---|---|
| x、y、z | voxel 中心或代表点的空间位置 |
| label | 语义类别，如地面、草地、障碍、label 0 |
| semantic confidence | 语义分类的可信程度 |
| geometric traversability cost | 几何可通行性代价 |
| observations | voxel 被重复观测的次数 |
| last observed time | 最近一次观测时间 |

此外，系统还可以提供没有经过语义融合的当前帧原始点云。

本次改造明确采用下面的设计：

    3D 语义和几何信息
            ↓
    在 FAR 前端完成二值化
            ↓
    可通行地表点 intensity = 0
    不可通行障碍点 intensity = 1
            ↓
    继续使用 FAR 原有障碍轮廓
            ↓
    继续使用原有 CTNode、NavNode 和 NavGraph

也就是说：

1. 不在 FAR 内部建立软代价地图；
2. 不把连续可通行性代价加入 Graph 边权重；
3. 不修改 GraphPlanner 当前基于欧氏距离的搜索代价；
4. 连续代价只在进入 FAR 以前用于判断一个位置最终属于 free 还是 obstacle；
5. FAR 收到数据以后，仍然只处理 0/1 可通行关系。

这一选择与 FAR 的结构相符。FAR 的核心表达是障碍轮廓及其可见性拓扑，它擅长回答“两个区域能否连通”，并不擅长在大片连续地面上表达不同通行质量。强行把草地、粗糙地面等代价边界变成轮廓，反而会造成大量没有必要的角点和拓扑分割。

---

## 33. 为什么不能直接把 voxel_cloud 重映射到 FAR

当前 Local 3D Semantic Voxel Map 输出的 PointCloud2 中包含：

    x
    y
    z
    rgb
    label
    semantic_confidence
    traversability
    intensity
    observations

其中当前实现把：

    intensity = traversability_cost

因此其 intensity 范围通常是：

    0.0：容易通过
    1.0：不能通过

但 FAR 在 FARUtil::ExtractFreeAndObsCloud() 中直接执行：

    if point.intensity < kFreeZ:
        放入 free cloud
    else:
        放入 obstacle cloud

默认 kFreeZ 是一个很小的地形分类阈值，而不是连续路径代价阈值。因此如果直接重映射：

    voxel_cloud → /terrain_cloud

可能得到：

    普通地面 cost = 0.05 → free
    草地 cost = 0.40     → obstacle
    未知区域 cost = 0.50 → obstacle
    粗糙地面 cost = 0.65 → obstacle

此时草地和未知区域的边界都会被 FAR 当成实体障碍边界，进一步产生：

    障碍栅格
        ↓
    Polygon
        ↓
    CTNode
        ↓
    NavNode

这不是本次改造需要的结果。

此外，FAR 在 PrcocessCloud() 中使用 pcl::fromROSMsg() 转成 PCLPoint，而当前 PCLPoint 主要就是 PointXYZI 表达。label、semantic_confidence、traversability 和 observations 等自定义字段不会继续保存在 FAR 的普通点结构中。因此必须在进入 FAR 之前完成语义解释和二值化，不能期待 FAR 在后续流程里再恢复这些信息。

正确接口应当是：

    voxel_cloud
        ↓
    semantic_voxel_to_far_terrain
        ↓
    新的 FAR 专用 PointXYZI

其中新的 PointXYZI 约定为：

    free point:     intensity = 0.0
    obstacle point: intensity = 1.0

只要 FAR 配置中的 kFreeZ 位于 0 和 1 之间，就可以稳定分离。

---

## 34. 新增二值转换节点的职责

建议新增一个独立 ROS 节点：

    semantic_voxel_to_far_terrain

不要直接把大量语义判断嵌入 FARMaster::TerrainCallBack()。独立适配节点有以下优点：

1. FAR 仍然接收原格式 PointXYZI，不影响原有代码；
2. 可以独立观察二值化前后的点云；
3. 更容易调节阈值、机器人尺寸和语义规则；
4. 即使将来替换 FAR，3D 到 2.5D 二值地图模块仍然可以复用；
5. 可以单独处理 label 0、无语义点以及当前帧原始点云；
6. 不会把上游 PointCloud2 的自定义字段契约扩散到整个 FAR 工程。

这个节点建议订阅：

| 输入 | 用途 |
|---|---|
| semantic voxel cloud | 提供经过时序融合的 3D voxel、语义、几何代价和观测置信度 |
| odometry | 提供机器人位置、高度和当前所在的地表层 |
| TF | 把输入点统一转换到 FAR 的 world frame |
| current raw cloud，可选 | 提供最新一帧几何观测，用于快速局部障碍补充 |

建议发布：

| 输出 | 用途 |
|---|---|
| /far_binary/terrain_map | 当前局部二值地形，供 FAR 的 terrain_local_cloud 或扩展模块使用 |
| /far_binary/terrain_map_ext | 如果适配节点自己完成全套扩展，也可以直接作为 terrain_cloud |
| /far_binary/free_debug | 仅用于 RViz 查看最终 free 地表点 |
| /far_binary/obstacle_debug | 仅用于 RViz 查看最终硬障碍点 |
| /far_binary/unknown_debug | 查看 label 0 和没有语义覆盖的区域 |

如果仍然保留原 terrain_analysis_ext，则无需让适配器发布 terrain_map_ext，而是：

    /far_binary/terrain_map
            ↓
    terrain_analysis_ext
            ↓
    /far_binary/terrain_map_ext

---

## 35. 一帧 voxel 点云进入适配器后的完整执行顺序

### 35.1 接收并检查 PointCloud2 字段

收到 voxel_cloud 后，首先检查：

    x、y、z 是否存在
    label 是否存在
    semantic_confidence 是否存在
    traversability 或 geometric_cost 是否存在
    observations 是否存在
    header.stamp 是否有效
    header.frame_id 是否有效

PointCloud2 对同一帧中的所有点使用同一套字段定义。因此“部分点没有 label”不能表示成这些点缺少 label 字段，只能表示成：

    semantic_valid = 0

或者：

    label = UINT32_MAX

建议增加显式字段：

    uint8 semantic_valid

这样不会把一个正常数值 label 同“没有语义观测”混淆。

如果暂时不修改消息字段，也可以约定：

    label = 0xffffffff 表示没有语义观测

但必须保证适配器和 voxel map 使用完全一致的无效值定义。

### 35.2 变换到统一世界坐标系

对输入点使用该帧自己的时间戳查询 TF：

    input cloud frame
            ↓ TF at cloud timestamp
    FAR world frame

不能只使用最新 TF，否则机器人运动时会使地面和障碍发生空间错位。

如果 TF 查询失败，本帧应整体放弃，不应把传感器坐标系中的点误认为世界坐标点。

### 35.3 裁剪当前局部工作范围

以机器人位置为中心裁剪：

    |x - robot_x| <= terrain_range
    |y - robot_y| <= terrain_range
    z_min <= z - robot_z <= z_max

这样可以避免把整个 voxel map 都重复送入 FAR，也能保证后续 XY 柱处理的规模稳定。

### 35.4 建立 XY 柱

按照与 FAR 或 terrain analysis 接近的水平分辨率，把 voxel 分组：

    key_x = floor(x / resolution_xy)
    key_y = floor(y / resolution_xy)

同一个 key 下可能存在多个不同高度的 voxel：

    Column(x, y):
        voxel z1
        voxel z2
        voxel z3
        ...

此时不能把所有低代价 voxel 都直接发布成 free。必须先找出机器人所在层的支撑地面。

### 35.5 选择机器人所在层的支撑地面

每个 XY 柱中的候选地面应满足：

1. 点本身语义允许成为地表，或者语义未知但几何代价允许；
2. 高度接近机器人当前可达的地面层；
3. 与相邻 XY 柱的地面高度连续；
4. 坡度不超过机器人允许坡度；
5. 相邻台阶高度不超过机器人允许台阶；
6. 地表上方存在足够的机器人净空；
7. 观测次数或几何置信度达到最低要求。

例如同一柱中存在：

    z = 0.0  地面
    z = 0.8  桌面
    z = 2.5  天花板

机器人当前在 z 约等于 0 的地表层，则只能选择 z = 0.0 作为 ground_z。不能把桌面和天花板作为 free 地面，否则 FAR 的 terrain height 会被破坏。

地面层选择最好从机器人脚下或机器人附近的可靠地面开始，向外进行邻域传播：

    robot support cell
            ↓
    接受高度差和坡度允许的相邻地表
            ↓
    继续扩展到整个局部范围

这比每一柱独立选择最低点或最高点更可靠，也能避免多层结构串层。

### 35.6 检查机器人身体空间

找到 ground_z 后，继续检查该 XY 位置上方：

    ground_z + chassis_clearance
            到
    ground_z + robot_height

如果这个高度区间内存在有效占据 voxel，就把该 XY 单元判为障碍。

这一检查解决的问题是：

    地面本身是 free
    但地面上方有墙、桌沿、横杆或其他会撞到机器人的结构

如果只检查地面 voxel 的代价，这些结构可能被漏掉。

### 35.7 按语义和几何代价二值化

完成地面层与身体空间检查以后，才能输出最终 0/1 状态。推荐优先级为：

    明确障碍语义
        >
    机器人身体空间碰撞
        >
    几何不可通行
        >
    地面或草地语义
        >
    label 0 或无语义时的几何判断

也就是说，明确的障碍语义可以直接覆盖低几何代价；明确地面语义不能覆盖真实的几何碰撞。

### 35.8 对障碍执行配置空间膨胀

初始二值障碍仍然表示物体本身占据的位置。为了让障碍轮廓近似代表“机器人中心不能进入的位置”，应按机器人水平包络进行膨胀：

    inflation_radius =
        robot_horizontal_radius
        + localization_margin
        + voxel_discretization_margin

例如：

    robot_horizontal_radius = 0.35 m
    localization_margin     = 0.10 m
    discretization_margin   = 0.05 m
    inflation_radius        = 0.50 m

具体数值应使用实际机器人尺寸。

需要检查 FAR 和 terrain_analysis_ext 是否还会再次膨胀。如果适配器已经执行完整机器人半径膨胀，后级的膨胀参数必须相应减小，避免重复膨胀导致窄通道被错误封死。

### 35.9 生成 FAR 兼容点云

对最终可通行地表：

    output.x = ground_x
    output.y = ground_y
    output.z = ground_z
    output.intensity = 0.0

对最终障碍：

    output.x = obstacle_x
    output.y = obstacle_y
    output.z = obstacle_z
    output.intensity = 1.0

必须保留正确的地面 z，因为 FAR 会用 free cloud 更新 terrain height，并进一步修正 CTNode、NavNode 和 waypoint 的高度。

### 35.10 发布并进入 FAR 原流程

发布以后，后续执行顺序恢复为原 FAR 流程：

    FAR-compatible binary terrain cloud
            ↓
    TerrainCallBack()
            ↓
    ExtractFreeAndObsCloud()
            ↓
    free grid 和 obstacle grid
            ↓
    surround_obs_cloud
            ↓
    ContourDetector
            ↓
    Polygon 和 CTNode
            ↓
    CTNode 与 NavNode 匹配
            ↓
    全局 NavGraph 局部增量维护
            ↓
    GraphPlanner 使用欧氏边长搜索

从 TerrainCallBack() 开始，不再需要理解 label 或 traversability。

---

## 36. 推荐的二值判定规则

可以先定义以下语义状态：

    GROUND
    GRASS
    STATIC_OBSTACLE
    DYNAMIC_OBSTACLE
    SEMANTIC_UNKNOWN，也就是 label 0
    SEMANTIC_UNAVAILABLE，也就是没有 label

推荐的基础判断如下：

    if semantic is STATIC_OBSTACLE:
        obstacle

    else if semantic is DYNAMIC_OBSTACLE:
        obstacle

    else if robot body volume is occupied:
        obstacle

    else if geometric_cost >= block_threshold:
        obstacle

    else:
        free support surface

展开后：

| 输入类别 | 几何代价低 | 几何代价高 | 最终结果 |
|---|---:|---:|---|
| 地面 | 是 | 否 | free 或 obstacle |
| 草地 | 是 | 否 | free 或 obstacle |
| 明确障碍 | 不考虑 | 不考虑 | obstacle |
| 动态障碍 | 不考虑 | 不考虑 | obstacle |
| label 0 | 是 | 否 | 依靠几何代价 |
| 没有 label | 是 | 否 | 依靠几何代价 |

草地在该方案中没有“中等路径代价”这一状态。它只有：

    几何上能走 → free
    几何上不能走 → obstacle

这正是本次二值方案与软代价方案的区别。

---

## 37. label 0 与没有 label 的处理

虽然最终都可以依靠几何代价进行 0/1 判定，但两者在进入二值适配器以前必须分开。

### 37.1 label 0

label 0 表示：

    点进入了语义系统的有效识别区域
    但网络最终没有给出可识别类别

它仍然是一次有效的语义观测，只是类别为 unknown。

需要特别注意，当前 Local 3D Semantic Voxel Map 的默认配置曾把 0x000000 配置为 floor_or_background，并给出很低的通行代价。如果本系统定义 label 0 为未识别，就必须修改配置，否则 label 0 会被长期融合成安全地面。

可以将其配置为：

    name: semantic_unknown
    cost: 中性或偏保守值

即使最终使用几何代价二值化，也要避免上游融合过程先把 label 0 错当成地面。

### 37.2 没有 label

没有 label 表示：

    几何传感器观察到了这个点
    但语义传感器或语义网络没有覆盖到这里

推荐的数据表达为：

    semantic_valid = false
    label = INVALID_LABEL
    semantic_confidence = 0
    geometric_cost = 有效值

这些点应该：

1. 更新几何占据；
2. 更新几何可通行性；
3. 更新 geometry observation time；
4. 不更新语义类别投票；
5. 在适配器中依靠几何代价进入 free 或 obstacle。

如果把这些点强行设成 label 0，会使“没有语义覆盖”和“语义识别失败”在长期融合中相互污染。

---

## 38. 建议对 Local 3D Semantic Voxel Map 做的最小修改

当前 voxel map 的输入流程主要面向带有语义字段的整帧点云。对于更大视场的无标签几何点，建议增加双输入：

    /semantic_cloud
    /raw_geometry_cloud

语义点云回调更新：

    voxel occupancy/hit
    semantic label hypotheses
    semantic confidence
    geometric traversability
    semantic observation count
    semantic last observed time

原始几何点云回调只更新：

    voxel occupancy/hit
    geometric traversability，如果能够计算
    geometry observation count
    geometry last observed time

但不更新：

    semantic hypotheses
    semantic confidence

Voxel 内部建议至少区分：

    semantic_observation_count
    geometry_observation_count
    semantic_last_observed
    geometry_last_observed
    semantic_valid

如果暂时不想修改 voxel map 内部，也可以在 voxel map 之外把两种点云合并后送给二值适配器。但这种临时方法无法让无标签点参与 voxel 的长期几何融合，稳定性会弱一些。

---

## 39. 二值状态需要迟滞，而不是单帧硬阈值

虽然输出只有 0 和 1，内部判定仍建议使用两个阈值：

    obstacle_enter_threshold
    obstacle_exit_threshold

例如：

    cost >= 0.75：从 free 进入 obstacle
    cost <= 0.55：从 obstacle 恢复 free
    0.55 < cost < 0.75：维持上一状态

状态转移为：

    previous FREE:
        cost >= enter threshold → OBSTACLE
        otherwise              → FREE

    previous OBSTACLE:
        cost <= exit threshold → FREE
        otherwise              → OBSTACLE

这样可以避免：

    0.73 → 0.76 → 0.72 → 0.77

导致障碍轮廓和 Graph 边每帧反复出现、消失。

明确障碍语义可以快速进入 obstacle：

    semantic obstacle → immediately obstacle

但从 obstacle 恢复 free 应更加谨慎，可以增加：

    连续 N 帧几何可通行
    当前射线确实观测到该区域为空
    voxel 观测次数达到要求

因此总体策略是：

    障碍出现：快速进入
    障碍消失：延迟退出

---

## 40. 三个 FAR 点云输入的最终连接方式

### 40.1 terrain_cloud：用于稳定轮廓和全局图更新

推荐连接：

    semantic voxel cloud
            ↓
    binary terrain adapter
            ↓
    /far_binary/terrain_map
            ↓
    terrain_analysis_ext
            ↓
    /far_binary/terrain_map_ext
            ↓
    FAR /terrain_cloud

它主要负责：

1. 更新 FAR 的持久 free/obstacle 网格；
2. 生成 surround_obs_cloud；
3. 提取局部障碍轮廓；
4. 产生 CTNode；
5. 更新全局 NavGraph。

如果适配器已经实现了 terrain_analysis_ext 所做的所有必要扩展，也可以直接：

    /far_binary/terrain_map_ext → FAR /terrain_cloud

但第一版建议保留现有扩展模块，减少同时改变的模块数量。

### 40.2 terrain_local_cloud：用于局部连接和轨迹验证

推荐：

    /far_binary/terrain_map → FAR /terrain_local_cloud

这个输入主要生成：

    FARUtil::local_terrain_obs_

随后 DynamicGraph 内部的 TerrainPlanner 使用它检查局部轨迹节点和轨迹连接。

因此 terrain_local_cloud 应尽量及时。可以包含：

    已融合 voxel 硬障碍
    +
    当前帧中已确认但尚未来得及稳定融合的障碍

要注意，在 FAR 的 static environment 模式下，TerrainLocalCallBack() 会直接返回。如果需要使用这一局部动态检查，配置中必须启用动态环境模式。

### 40.3 scan_cloud：用于当前可见性和历史障碍清除

推荐：

    current raw cloud
            ↓
    配准到世界坐标系
            ↓
    /registered_scan
            ↓
    FAR /scan_cloud

FAR 使用它和 surround obstacle/free cloud 做射线关系判断，从历史障碍中提取当前可能已经消失的动态障碍。

必须明确：

    /scan_cloud 并不是主障碍地图输入

它主要用于判断旧障碍是否应被清除。当前新出现的障碍仍然需要进入：

    /terrain_cloud

或者至少快速进入：

    /terrain_local_cloud

如果只把新障碍放到 /scan_cloud，而二值 terrain map 没有更新，这个障碍不会完整参与正常的轮廓提取和长期图更新。

---

## 41. 动态障碍进入和离开时的完整过程

### 41.1 新动态障碍进入

例如原有图边为：

    NavNode A ───────── NavNode B

当前出现一个人：

    NavNode A ─── 人 ── NavNode B

推荐执行顺序：

1. 当前原始点云首先观察到人；
2. 几何分类或语义分类将其判为 obstacle；
3. 它立即进入二值 terrain_local_cloud；
4. TerrainPlanner 在验证局部轨迹或连接时发现不可通过；
5. voxel map 随后把该障碍融合进稳定地图；
6. 二值 terrain_cloud 中出现该障碍；
7. 下一轮局部轮廓重建包含该障碍；
8. 与新 Polygon 相交的历史边得到无效判断；
9. Graph 按 FAR 原有投票机制更新或删除边。

因此这里存在两个时间尺度：

    快速安全层：terrain_local_cloud
    稳定拓扑层：terrain_cloud + ContourGraph + NavGraph votes

这样无需修改 Graph 代价，同时避免完全等待拓扑投票后才响应障碍。

### 41.2 动态障碍离开

障碍离开后不能因为某一帧没有点就立即设为 free。推荐顺序：

1. 当前原始点云的射线真正穿过该区域；
2. 该区域没有被其他物体遮挡；
3. 连续若干帧未再看到障碍；
4. voxel 中障碍状态通过退出阈值或衰减恢复；
5. 二值适配器把该区域从 obstacle 改为 free；
6. terrain_cloud 不再包含该障碍；
7. 当前局部轮廓随之消失；
8. NavGraph 根据新的局部可见性重新建立可以恢复的边。

需要区分：

    voxel 因超时消失

和：

    传感器明确观察到该位置为空

前者只表示没有继续观测，不一定表示安全自由；后者才是更强的 free 证据。

---

## 42. 适配器的参考伪代码

整体回调可以组织为：

    VoxelCloudCallback(message):
        validateFields(message)
        cloud_world = transformAtMessageStamp(message)
        local_cloud = cropAroundRobot(cloud_world)

        columns = buildXYColumns(local_cloud)
        support_surface = selectRobotConnectedSurface(columns, robot_pose)

        free_points.clear()
        obstacle_points.clear()

        for each XY column:
            ground = support_surface[column]

            if ground does not exist:
                continue or apply unknown policy

            semantic_obstacle =
                ground.label is STATIC_OBSTACLE
                or ground.label is DYNAMIC_OBSTACLE

            body_collision =
                hasOccupiedVoxelBetween(
                    ground.z + chassis_clearance,
                    ground.z + robot_height)

            geometry_blocked =
                updateBinaryStateWithHysteresis(
                    column.previous_state,
                    column.geometric_cost,
                    enter_threshold,
                    exit_threshold)

            if semantic_obstacle
               or body_collision
               or geometry_blocked:
                obstacle_points.push(column obstacle representative)
            else:
                free_points.push(ground)

        obstacle_points = inflate(
            obstacle_points,
            robot_radius + safety_margin)

        output = merge(free_points, obstacle_points)

        for p in free_points:
            p.intensity = 0.0

        for p in obstacle_points:
            p.intensity = 1.0

        publish(output)
        publishDebugClouds()

未知地表策略可以配置：

    conservative:
        没有可靠 ground 或 cost 时不发布 free

    exploratory:
        有明确几何地表、没有碰撞且 cost 低时允许发布 free

无论使用哪种策略，都不建议把完全未观测空间直接发布成 free。

---

## 43. 推荐配置参数

适配节点可以提供：

| 参数 | 含义 |
|---|---|
| world_frame | FAR 使用的世界坐标系 |
| terrain_range | 发送给 FAR 的局部范围 |
| resolution_xy | XY 柱分辨率 |
| min_relative_z | 相对机器人最低处理高度 |
| max_relative_z | 相对机器人最高处理高度 |
| max_ground_step | 相邻地表最大允许台阶 |
| max_ground_slope | 最大允许坡度 |
| chassis_clearance | 底盘离地间隙 |
| robot_height | 机器人碰撞体高度 |
| robot_radius | 机器人水平碰撞半径 |
| localization_margin | 定位误差安全余量 |
| obstacle_enter_threshold | 从 free 进入 obstacle 的几何代价阈值 |
| obstacle_exit_threshold | 从 obstacle 恢复 free 的几何代价阈值 |
| minimum_observations | 稳定二值判断所需最少观测数 |
| minimum_semantic_confidence | 使用明确语义覆盖几何判断的最低置信度 |
| unknown_policy | label 0 的处理策略 |
| unavailable_policy | 没有语义覆盖时的处理策略 |
| use_raw_obstacle_supplement | 是否把当前帧障碍快速加入 local terrain |

阈值不应直接照抄示例值，应使用录制数据统计：

    可通行地面 cost 分布
    草地 cost 分布
    明确障碍 cost 分布
    label 0 cost 分布
    无标签区域 cost 分布

然后选择能够稳定分开“机器人确实能走”和“机器人不能走”的阈值。

---

## 44. 这一方案中需要修改和不需要修改的部分

### 44.1 必须新增或修改

1. 新增 semantic_voxel_to_far_terrain 适配节点；
2. 明确 label 0 的真实含义并修改 voxel map 配置；
3. 为没有语义覆盖的点增加 semantic_valid 或无效 label 表达；
4. 实现 3D voxel 到机器人当前地表层的投影；
5. 实现几何代价的 0/1 判定；
6. 实现机器人身体空间碰撞检查；
7. 实现二值状态迟滞；
8. 输出 intensity 只有 0 和 1 的 FAR 兼容点云；
9. 根据是否保留 terrain_analysis_ext 调整 ROS remap；
10. 确保当前新障碍不仅进入 scan_cloud，还能进入 local/main terrain。

### 44.2 不需要修改

1. ContourDetector 的基本轮廓提取过程；
2. Polygon 构造；
3. CTNode 角点生成；
4. CTNode 和 NavNode 的匹配方法；
5. NavNode 的全局持久化；
6. contour_votes 和 edge_votes 的基本机制；
7. GraphPlanner 的 EulerCost；
8. Graph 搜索算法；
9. Graph 边上增加语义或连续代价字段；
10. 单独建立软 traversability map。

---

## 45. 上线前必须检查的典型错误

### 45.1 把原 traversability 直接放进 intensity

错误结果：

    代价超过 kFreeZ 的草地、未知地面全部成为障碍

正确做法：

    适配器先二值判断
    然后重新写 intensity = 0 或 1

### 45.2 把 label 0 当成默认地面

错误结果：

    语义未识别区域被大面积认为安全

正确做法：

    修改 semantic class 配置
    并在适配器中让 label 0 依靠几何代价判断

### 45.3 把无标签点全部当障碍

错误结果：

    语义相机视场以外即使是平整地面也无法通行

正确做法：

    无标签点不参与语义融合
    但仍允许依靠几何可通行性进入 free

### 45.4 把无标签点全部当 free

错误结果：

    语义视场之外的墙和障碍可能被放行

正确做法：

    无标签不等于自由
    必须继续通过几何代价和身体空间占据检查

### 45.5 把所有 3D 低代价 voxel 都作为地面

错误结果：

    桌面、横梁、天花板影响地面高度
    多层场景发生串层

正确做法：

    按 XY 建柱
    只选择与机器人所在层连续的支撑面

### 45.6 重复膨胀

错误结果：

    适配器膨胀一次
    terrain_analysis_ext 再膨胀
    FAR 内部又按安全距离处理
    最终窄通道全部消失

正确做法：

    明确膨胀由哪一层负责
    对总安全半径进行统一核算

### 45.7 只向 scan_cloud 发送当前新障碍

错误结果：

    原始扫描被用于历史障碍射线判断
    但新障碍没有进入稳定轮廓地图

正确做法：

    新障碍最终必须进入二值 terrain_cloud
    需要快速响应时同时进入 terrain_local_cloud

---

## 46. 推荐的验证顺序

### 46.1 单点分类验证

分别构造：

    地面
    草地
    障碍
    label 0
    没有 label

确认每类点在不同 geometric cost 下输出正确的 0 或 1。

### 46.2 地面高度验证

在斜坡、台阶、桌面和多层结构附近观察：

    /far_binary/free_debug

确认 free 点始终位于机器人真正行驶的地表层。

### 46.3 膨胀验证

在已知宽度的窄通道中检查：

    原始障碍宽度
    膨胀后自由宽度
    机器人真实宽度

确认能通过的通道没有被误封，不能通过的通道不会保留拓扑连接。

### 46.4 轮廓验证

观察 FAR 输出的轮廓和 CTNode：

1. 草地边界不应产生轮廓；
2. label 0 与地面的交界不应仅因语义不同产生轮廓；
3. 墙和实体障碍应产生稳定轮廓；
4. 障碍进入时应出现新轮廓；
5. 障碍离开并被确认清除后轮廓才消失。

### 46.5 Graph 验证

确认：

1. Graph 仍然只表达二值自由空间连通性；
2. Graph 边代价仍是欧氏距离；
3. 新障碍进入后局部连接先被 terrain_local_cloud 阻断；
4. 随后稳定轮廓更新全局图边；
5. 障碍离开后图边不会因单帧抖动反复恢复和删除。

---

## 47. 本次二值改造的最终完整流程

综合起来，系统最终执行顺序为：

    带语义和几何代价的当前观测
                ↓
    Local 3D Semantic Voxel Map
                ↓
    得到带 label、confidence、cost、observations 的 3D voxel cloud
                ↓
    semantic_voxel_to_far_terrain
                ↓
    按时间戳转换到 world frame
                ↓
    裁剪机器人附近局部范围
                ↓
    按 XY 建立 voxel 柱
                ↓
    找到与机器人当前地表层连续的支撑面
                ↓
    检查地表上方机器人身体空间
                ↓
    明确障碍语义优先
                ↓
    地面、草地、label 0、无标签点使用几何代价完成二值判断
                ↓
    使用迟滞稳定 free/obstacle 状态
                ↓
    按机器人半径和安全余量膨胀 obstacle
                ↓
    free intensity = 0
    obstacle intensity = 1
                ↓
    发布 /far_binary/terrain_map
                ↓
        ┌───────┴────────────────┐
        ↓                        ↓
    FAR terrain_local       terrain_analysis_ext
        ↓                        ↓
    局部轨迹检查        /far_binary/terrain_map_ext
                                 ↓
                         FAR terrain_cloud
                                 ↓
                         free/obstacle grids
                                 ↓
                         局部障碍轮廓
                                 ↓
                         Polygon 和 CTNode
                                 ↓
                         CTNode/NavNode 匹配
                                 ↓
                         全局 NavGraph 局部维护
                                 ↓
                         欧氏距离图搜索

与此同时：

    当前帧真正原始点云
                ↓
    配准后的 /registered_scan
                ↓
    FAR /scan_cloud
                ↓
    当前可见性、射线关系和历史动态障碍清除

最终职责边界是：

    Local 3D Semantic Voxel Map：
        融合 3D 几何、语义和观测历史

    Binary Terrain Adapter：
        把复杂信息压缩成安全稳定的 free/obstacle

    FAR Planner：
        从二值障碍中提取轮廓并维护全局拓扑图

    当前帧原始点云：
        补充实时可见性和动态变化信息

因此，这个方案不是让 FAR 理解连续代价，而是在 FAR 之前把语义和几何信息充分利用完，再向 FAR 提供它最适合处理的二值世界。
