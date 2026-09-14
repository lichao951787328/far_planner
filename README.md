# FAR Planner 本地语义体素适配说明

本仓库是在 FAR Planner 基础上进行的本地语义体素地图适配版本。当前目标是以
`local3DSemanticVoxelMap` 发布的完整局部快照作为几何依据，在不融合低分辨率全局
占用地图的情况下提取当前轮廓，并维护可复用的历史导航拓扑。

原始 FAR Planner 项目：<https://github.com/MichaelFYang/far_planner>

详细的节点、边和生命周期说明见
[`contour_node_edge_maintenance_detailed.md`](contour_node_edge_maintenance_detailed.md)。

## 当前数据链路

```text
local voxel cloud
    -> 逐点语义/代价分类
       -> STATIC_OBSTACLE    -> current_static_obs_cloud_
       -> TRANSIENT_OBSTACLE -> current_dynamic_obs_cloud_
                                -> effective_dynamic_obs_cloud_
       -> TERRAIN_SUPPORT    -> semantic_terrain_support_cloud_
       -> IGNORE             -> 丢弃
    -> 静态、动态障碍分别投影到二维轮廓栅格
    -> static_contours_ / dynamic_contours_
    -> 当前帧 ContourGraph
    -> 与历史 Graph 匹配、晋升、冻结、恢复或删除
```

本地 voxel 模式下，明确属于 `dynamic_labels` 的点才进入瞬态层。其余达到障碍
代价阈值的点，包括未知标签和低语义置信度点，均作为普通静态候选；低代价地形
标签只提供地形支撑，其余点忽略。

二维轮廓栅格与上游 voxel 分辨率相互独立：通用 `default.yaml` 使用 `0.4 m`，
SSMI 和五分类配置覆盖为 `0.2 m`，上游局部 voxel 通常为 `0.10 m`。同一 XY 柱
内只要存在一个对应障碍层的三维点，该障碍层的二维像素即被占用；地形支撑点不
参与障碍轮廓投票，也不会抵消同一 XY 的障碍点。

### 画布扩展、FAR 拓扑图与配置空间图

局部 voxel box 是随机器人旋转的非对称矩形，轮廓画布则是世界坐标轴对齐的
正方形。代码先用局部矩形的外接圆扩大轮廓画布半径；这只是在相同基础分辨率下
增加周围的空白格，防止旋转后的 box 角落落到画布外，不会放大障碍，也不会把
额外未知空间变成已观测空间。

从 2026-09-12 的分步回退开始，每帧由同一份预处理障碍点生成两张用途不同的图。
第 1 步拆分图像职责，第 2 步将轮廓近似切换到 FAR 的 TC89 稀疏链路，第 3 步已把
同轮廓 edge 恢复为 FAR 的拓扑关系；有序 TC89 source chain 仅保留作诊断和顶点映射。

拓扑轮廓图执行 FAR 的图像阶段：

```text
按外接圆尺寸建立基础画布（0.2 m/格）
    -> 每个障碍 voxel 写入对应基础格
    -> 二值化后做 3x3 邻域扩张
    -> INTER_LINEAR resize_ratio=3
    -> topology_blur_size=2 的非归一化 boxFilter
    -> findContours(RETR_TREE, CHAIN_APPROX_TC89_L1)
    -> approxPolyDP / TopoFilter / AdjacentDistanceFilter
```

机器人安全碰撞图则继续执行：

```text
按外接圆尺寸建立基础画布（0.2 m/格）
    -> 每个障碍 voxel 只写入它对应的一个基础格
    -> 二值化
    -> 将基础格中心无插值地映射到 resize_ratio=3 的精细图（约 0.0667 m/格）
    -> 欧氏距离变换
    -> 距原始障碍不超过 robot_collision_clearance=0.45 m 的格设为占据
    -> 只供 edge、waypoint 和动态阻断的碰撞检查
```

因此 `117x117 -> 351x351` 是同一物理范围的重采样，不是再把约 `23.2 m` 的画布
扩大三倍。resize 也不会恢复上游已经在 `0.2 m` 投影中丢失的信息；它只让插值
后的边界和最终保留角点可以落在约 `0.0667 m` 的坐标间隔上。

`resize_ratio` 不增加传感器信息。拓扑图中的 3x3、双线性非零像素和 boxFilter
只影响障碍拓扑轮廓；配置空间图仍把每个基础占据格映射为一个精细 seed，再按米做
欧氏距离变换。两张图不再互相复用，因此 FAR 的拓扑平滑不会被误当成第二次机器人
净空膨胀。静态和明确动态层分别生成配置空间图；动态层只临时屏蔽搜索边。

`approxPolyDP` 的距离阈值则是另一个概念。代码设置：

```text
distance_limit_px = 1.5 * resize_ratio * simplify_ratio
pixel_size_after_resize = contour_grid_resolution / resize_ratio

distance_limit_world
  = distance_limit_px * pixel_size_after_resize
  = 1.5 * contour_grid_resolution * simplify_ratio
```

所以在 `0.2 m`、ratio=3、静态 simplify ratio=1 时，阈值确实是 `4.5` 个放大后
像素，即约 `0.3 m`，不是 `1.5*0.2/3=0.1 m`。后一个结果只在像素阈值仍固定为
`1.5` 时成立。当前设计刻意让物理简化容差不随 resize ratio 改变；resize 只细化
坐标采样，不保留更小的几何细节。该阈值是轮廓近似误差/删点容差，并不是统一向
外的 `0.3 m` 障碍膨胀。

### 与原版 FAR Planner 轮廓提取器的对照

上述核心图像链路不是本地 voxel 改造新加入的。原版 FAR Planner 已经执行：

```text
按 sensor_range / voxel_dim 建立画布
    -> 每个障碍点写入 3x3 邻域
    -> 动态环境按像素命中数量 threshold
    -> INTER_LINEAR resize
    -> boxFilter（resize 后不重新二值化）
    -> findContours(RETR_TREE, CHAIN_APPROX_TC89_L1)
    -> approxPolyDP，阈值为 1.5*resize_ratio 个放大后像素
    -> 拓扑过滤、邻点距离/共线过滤
    -> 除以 resize_ratio 转回世界坐标
```

所以 `3x3`、resize、boxFilter、非零插值晕边，以及物理上不随 resize ratio 改变
的 RDP 简化容差，均继承自原版。原版默认 `voxel_dim=0.15 m`、ratio=3 时，静态
RDP 容差约为 `1.5*0.15=0.225 m`；当前 SSMI/五分类配置使用 `0.2 m`，对应约
`0.3 m`。

当前 fork 在原版链路上新增或改变的是：

- 将上游 voxel 分辨率与 `contour_grid_resolution` 解耦；
- 栅格中心吸附到固定世界网格，减少机器人连续运动造成的投影相位抖动；
- 已分类的本地 voxel 使用任意一次命中即占用；
- 静态与明确动态障碍分别生成轮廓，动态轮廓可使用更大的简化比例；
- 用旋转非对称局部窗口的外接圆扩大空白画布；
- 旧的世界坐标共线简化参数仅为配置兼容保留，不再对 FAR 稀疏链路二次删点；
- 标记局部窗口裁剪端点并限制其 Graph 生命周期；
- 从 2026-09-12 的第 1 步起恢复 3x3、双线性 resize 和 boxFilter，专门生成
  FAR 拓扑轮廓图；按米定义的欧氏配置空间图只负责实际碰撞检查。

当前 `ConvertContoursToRealWorld()` 还连续调用了两次相同的
`ConvertCVToPoint3DVector()`。第二次会清空并重写同一个输出向量，因此结果不变，
只是一次冗余计算；原版只调用一次。此处已记录，尚未修改代码。

将 `resize_ratio` 从 3 改为 4 时，`117x117` 会变为 `468x468`，放大后像素间隔
约为 `0.05 m`；物理画布、`0.45 m` 配置空间半径和静态 RDP 的 `0.3 m` 容差均
不变。倍率只改变距离变换和轮廓坐标的离散误差及计算量。

### 轮廓扩边与节点匹配阈值

当前轮廓来自独立的 FAR 拓扑图，并不是按 `robot_collision_clearance` 膨胀后的机器人
中心配置空间边界。机器人净空只编码在另一张配置空间图中；五分类当前使用 `0.30 m`。
`0.2 m / resize_ratio` 决定精细栅格的离散间隔，RDP 的 `0.3 m` 只是删点/折线近似
容差，不能与机器人净空相加。

当前轮廓 CTNode 与历史 NavNode 的身份匹配已经与 `robot_dim`/碰撞净空解耦。
SSMI/五分类的静态匹配参数为：

```text
tight_radius = 0.4 m
max_radius   = 0.6 m（代码强制的静态硬上限）
```

普通角点先将两条轮廓表面方向合成角平分线，再按 CONVEX/CONCAVE 约定把它定向到
机器人自由侧。程序沿正反方向分别在当前二值配置空间采样三个距离；只有正向至少
两次为 FREE、反向至少两次为 OCCUPIED，方向才是可靠有向自由方向，证据相反时会
翻转。CLIP 不使用包含人工封口的角平分线，而从相邻真实轮廓边取得切线，在配置空间
图内比较两侧法向，并要求采样点仍位于当前观测窗口；缺图或两侧证据含混时才保持
不可靠方向。

设两个可靠自由方向的有符号点积为 `c=cos(theta)`：`c<0`（相反自由侧）直接拒绝
匹配；`c>=0` 时匹配半径为 `0.4+0.2*c m`。任一方向不可靠以及 PILLAR 都只能使用
`0.4 m` 紧位置回退。明确动态节点使用独立的
`ContourGraph/dynamic_match_max_radius`，默认 `1.4 m`，不再从 robot_dim 推导。
匹配还必须满足同一静/动态来源、相同角点类型、高度相容及匹配线不穿障碍。

候选的一对一排序现在直接使用真实米制距离 `score=distance`，方向只负责准入和在
`0.4~0.6 m` 内选择半径；不再使用 `distance/match_radius`，因此较大的准入半径
不会让更远候选在排序中显得更近。

匹配失败后的静态重复抑制又是另一层：通用默认半径 `0.5 m`，SSMI 和当前五分类
覆盖配置均为 `0.4 m`。它只阻止创建同类型、同方向、同高度的重复静态角点，不会把
两个已有 NavNode 做普通空间聚类合并。其中“当前角点与已匹配历史节点”还要求
局部连线自由；“同一帧当前角点与先前已接受的新角点”不再检查连线是否自由，也
不要求来自同一个 Polygon，所以两个不同轮廓的相似角点也会按该半径重复抑制。
小轮廓整体折叠为一个 PILLAR
使用的则是周长阈值 `4*robot_dim=2.4 m`，同样不是位置匹配半径。

## 局部观察窗口与 W_inner / W_guard / W_outside

### 几何定义

设上游局部快照在机器人坐标系中的完整矩形为：

```text
W_full = [min_x, max_x] x [min_y, max_y]
```

`boundary_guard = g` 后，三个期望区域可定义为：

```text
W_inner   = [min_x+g, max_x-g] x [min_y+g, max_y-g]
W_guard   = W_full - W_inner
W_outside = R^2 - W_full
```

窗口使用生成该份局部点云快照时的机器人位置和朝向，而不是下一次规划循环中可能
已经变化的 odom。`LocalObservationWindow2D::ToLocal()` 负责将世界点转换到这个
随机器人旋转的局部矩形中。

### 当前代码实际区分程度

当前代码没有引入统一的 `ObservationRegion` 枚举，但已经提供两层查询：

```text
Contains() / SegmentFullyContained()             -> W_inner
ContainsFull() / SegmentFullyContainedFull()     -> W_full
ContourGraph::IsPointInsideCurrentObservationWindow()
                                                   -> W_full 加轮廓处理 halo
```

最后一层 halo 使用 `boundary_guard` 并保留一个轮廓基础格的量化余量，用于容纳
`0.45 m` 配置空间边界超出原始 voxel box 的已知处理范围。它只允许当帧 CLIP
路由，不提供任何持久化、恢复或删除证据。

当前对应行为如下：

| 对象/操作 | `W_inner` | `W_guard`/处理 halo | 真正 `W_outside` |
|---|---|---|---|
| 当前轮廓顶点 | 可靠普通顶点 | `is_boundary_clipped=true` | 不允许用于路由 |
| 静态候选晋升 | 可以累计稳定观测 | 永不晋升；下一帧重新校核 | 不适用 |
| 历史节点维护 | 可以匹配、重验证 | 跳过负证据更新 | 冻结 |
| 新的 odom/start/goal 锚点 | 可以使用 | 当帧可用，仍须通过完整碰撞和地形检查 | 拒绝 |
| 当前 contour/visibility edge | 可以建立 | 当帧可建立，随 CLIP 节点一起失效 | 拒绝 |
| 静态 edge 恢复 | 整条路线完全位于可靠区时允许 | 不以空白恢复 | 不以空白恢复 |

由于输入点云本身被上游裁剪在 `W_full` 内，普通原始点通常不会出现在真正的
`W_outside`；但配置空间膨胀、轮廓闭合以及外接正方形画布可能生成位于完整窗口
边缘外的轮廓顶点。当前这些顶点与 `W_guard` 顶点使用同一个
`is_boundary_clipped` 身份，但只有落在 `W_full + boundary_guard halo` 内的点能作
为当帧路由端点；更远的画布伪顶点被拒绝。

对于历史维护，一条横跨 `W_inner` 和 `W_guard` 的历史 edge
会进入重验证，guard 内实际观测到的障碍仍可能使它阻断；但一条完全位于
`W_guard` 的历史 edge 与一条完全位于 `W_outside` 的 edge 都不会触发重验证，
两者统一被冻结。对当帧 CLIP 路由二者已经分开；对历史负证据仍有意保持保守。

### 与原版 FAR Planner 的差异

原版 MichaelFYang/FAR Planner 没有名为 `W_inner/W_guard/W_outside` 的显式区域
类型，也没有针对旋转非对称局部 voxel box 的矩形三态模型；但它具有功能上近似
的三层径向范围：

```text
隐式 inner:   distance < kMarginDist
隐式 guard:   kMarginDist <= distance < kSensorRange
隐式 outside: distance >= kSensorRange

kMarginDist = kSensorRange - kMatchDist
```

原版 `ReEvaluateCorner()` 只会因为轮廓未匹配而惩罚 margin 内的节点；处于边缘
环带的未匹配节点保持不变。它还通过多帧 dumper、连接投票和 contour vote 抑制
单帧变化。因此，“没有这些 W 名称”不等于没有边缘保护语义。

原版能够依靠这套隐式范围工作，还因为其 `surround_obs_cloud_` 来自持续累积的
世界障碍网格，而不是每帧完整替换的有限局部 box。障碍通过明确的自由空间/射线
更新清除，动态点也会从世界障碍网格移除，所以当前帧边缘的空白通常不会直接被
解释为历史空间已经自由。

本 fork 的条件不同：局部 voxel 快照每帧替换当前几何、历史 Graph 单独持久化，
而且观察范围是随机器人旋转的非对称矩形。圆形的 `kMarginDist/kSensorRange` 不能
准确表达其前、后、左、右边界，因此必须使用与采集位姿绑定的矩形窗口。

本 fork 已新增：

- 与点云采集时刻绑定的旋转非对称局部窗口；
- 至少一个轮廓栅格宽度的 `boundary_guard`；
- 裁剪轮廓端点身份隔离；
- 历史节点在可靠窗口外冻结；
- 静态 edge 只有在路线被完整可靠观测时才能因“无障碍”恢复；
- 当前静态障碍对部分进入窗口的历史 edge 仍可提供正阻断证据。

尚未实现的只是统一三态枚举；当前点/线段查询和行为语义已经固定为：

```text
W_inner:   正障碍和负观测都可信，可更新、恢复或删除历史状态
W_guard:   正障碍可信；空白不作为清除证据；轮廓端点只作瞬时对象
W_outside: 完全无观测证据，历史节点和边保持冻结
```

## 逐帧 Graph 调试

### 第一阶段：帧事务可视化

调试总开关是 FAR 私有参数 `Debug/enabled`，launch 参数名为 `farDebug`，默认均为
`false`。关闭时不会复制 IGNORE 点、匹配候选、Graph 前后快照或 OpenCV 图像；规划
行为和原来的可视化 topic 不受影响。开启后，一份 local voxel 快照从接收到 Graph
提交完成始终使用同一个采集时间戳，并发布：

| topic | 内容 |
|---|---|
| `/far_debug/classified_cloud` | 收到的每个有效 XYZ 点及其最终分类；红=静态、紫=明确动态、绿=地形支撑、灰=IGNORE，并保留 `label`、`semantic_confidence`、`cost` 字段 |
| `/far_debug/raster/static_base`、`dynamic_base` | 单格投影并二值化后的原始障碍栅格，尚未加入机器人净空 |
| `/far_debug/raster/static_processed`、`dynamic_processed` | FAR 拓扑栅格及 findContours/RDP 结果，黄色为最终轮廓；不是机器人配置空间碰撞图 |
| `/far_debug/current_contours` | 当前静态/动态世界坐标轮廓 |
| `/far_debug/current_ctnodes` | CONVEX、CONCAVE、PILLAR、UNKNOWN、裁剪端点、表面方向和拓扑方向 |
| `/far_debug/graph_before_update` | 本帧匹配前的全部历史节点和 edge 状态 |
| `/far_debug/match_results` | 接受匹配、全局一对一竞争失败和连线阻挡候选；文字显示距离、方向夹角和动态匹配半径 |
| `/far_debug/duplicate_suppressed` | 匹配失败后，被同帧/历史静态重复抑制的角点及 keeper |
| `/far_debug/node_events` | 新建、STATIC_CANDIDATE 晋升、位置更新、清理和其他删除 |
| `/far_debug/edge_events` | 新建、静态阻断、动态临时阻断、拓扑阻断、恢复和最终删除 |
| `/far_debug/graph_after_update` | Graph 提交后的全部节点、来源、seen/miss 计数和 edge 状态 |
| `/far_debug/color_legend` | RViz 可见的四列颜色图例，按分类/轮廓/CT、匹配/节点事件、Graph、edge 事件/搜索结果分组 |
| `/viz_current_eligible_graph` | 当前满足节点/edge 条件但可能不连通的候选搜索图 |
| `/viz_current_search_graph` | 从 odom 所在连通分量得到的最终 current search graph |
| `/far_debug/frame_summary_image` | 分类、轮廓栅格、匹配和最终 Graph 四联图 |
| `/far_debug/frame_done` | Graph、GraphPlanner 和调试发布全部完成后的源 cloud 时间戳；逐帧播放器以此作为事务完成确认 |

Graph before/after 中，橙色点是 `STATIC_CANDIDATE`，蓝色点是
`STATIC_GLOBAL`，紫色方块是 `DYNAMIC_LOCAL`，绿色/红色点分别是 odom/goal。
edge 灰色为 active，橙色为静态阻断，红色为动态阻断，紫色为拓扑阻断。每帧先发
`DELETEALL`，所以帧数减少时不会在 RViz 留下上一帧的幽灵 marker。

当前代码没有调用或实现真正的 `MergeNodeInGraph()` 空间聚类合并。调试界面因此不把
“当前 CTNode 复用历史 NavNode 身份”或“重复候选被抑制”误写成 NavNode merge：前者
显示在 `match_results`，后者显示在 `duplicate_suppressed`；删除则显示在
`node_events`。这三类事件的含义彼此独立。

推荐调试 RViz 配置为 `rviz/far_debug.rviz`。四联图也可以单独查看：

```bash
rqt_image_view /far_debug/frame_summary_image
```

若 RViz/rqt 图像区域太小，可让 FAR 直接打开一个可缩放的 OpenCV 窗口：

```bash
roslaunch far_planner five_class_2026_08_13_bag_navigation.launch \
  stepReplay:=true farDebug:=true debugOpenCvWindow:=true \
  debugOpenCvWindowWidth:=1200 debugOpenCvWindowHeight:=1000 \
  rvizConfig:=far_debug startLocalPlanner:=false bagStart:=0.15
```

窗口标题为 `FAR frame summary`，可以拖动边框放大或用桌面窗口管理器最大化。逐帧
暂停时 FAR 仍持续处理 HighGUI 事件，所以窗口不会因长时间检查当前帧而停止重绘；
调用一次 `/five_class_bag_stepper/next` 后才会换成下一帧。Docker 运行时需要正确转发
`DISPLAY` 并挂载 `/tmp/.X11-unix`。`debugOpenCvWindow` 默认关闭，避免无图形环境时
影响规划；即使窗口无法创建，ROS 图像 topic 和规划也会继续运行。

若需要保存四联图，先创建目标目录，再传
`debugSaveFrames:=true debugSaveDirectory:=/existing/path`。程序不会隐式创建目录。

### 第二阶段：bag 单帧事务播放器

`five_class_bag_frame_stepper` 不按墙钟连续播放 bag。每次 `next` 严格执行：

```text
读取下一条 /grids_points
  -> /clock 前进到该消息的 bag record time
  -> 发布截至该帧及少量 lookahead 的 /tf
  -> 缓存截至该帧的最新 odom（暂不发布）
  -> 验证 map <- cloud_frame 在 cloud header.stamp 有精确 TF
  -> 发布恰好一条 cloud
  -> 等待同时间戳 local voxel 输出
  -> 此时 map_start 已建立，再发布对应 odom
  -> 等待同时间戳 /far_debug/frame_done
  -> 返回 next 调用
```

之所以把 odom 放到 voxel ack 之后，是因为首个有效 cloud 才建立
`map -> map_start`。若暂停的 `/clock` 下提前触发 FAR odom 回调，其 ROS-time TF
等待无法自然超时，会造成首帧死锁。

启动当前五分类 bag 的逐帧调试：

```bash
roslaunch far_planner five_class_2026_08_13_bag_navigation.launch \
  stepReplay:=true farDebug:=true rvizConfig:=far_debug \
  startLocalPlanner:=false bagStart:=0.15
```

launch 默认从 `$(find far_planner)/../dateset/9-7/` 定位 bag，因此同一源码树挂载到
宿主机或 Docker 的不同目录时无需修改路径。使用其他 bag 时再传
`bag:=/容器内/绝对路径/file.bag`；这里必须是运行 ROS 节点的容器能够看到的路径，
不能直接使用仅在宿主机存在的路径。

逐帧控制：

```bash
rosservice call /five_class_bag_stepper/status
rosservice call /five_class_bag_stepper/next
rosservice call /five_class_bag_stepper/skip
```

`next` 正在处理时再次调用会返回 `BUSY`；TF 不存在或下游超时时，当前 cloud 保留，
可以修复后重试或显式 `skip`。该 2026-08-13 bag 的第 1 条 cloud 时间戳比最早动态
TF 早约 `0.1 s`。stepper 会仅对每个从未出现过的 child frame 使用 lookahead 内最近
一条 TF，在首帧 cloud stamp 生成一次 bootstrap；后续帧全部使用原始 TF 时间戳。
因此第 1 帧也能完成事务，不再要求跳过。`stepReplay=false` 时保持原有连续
`rosbag play` 行为。

逐帧模式让 FAR 使用 `ros::WallRate`，并关闭 odom/local-voxel 的墙钟超时，因而用户
在 RViz 停留任意长时间都不会丢帧或触发假超时；此设置仅在
`Debug/deterministic_step_mode=true` 时生效。

## 修改记录

### 2026-09-11：静态角点身份匹配与机器人尺寸解耦

1. 完整 bag 审计发现 235 次历史节点单帧位移超过 `0.75 m`，其中 14 个节点在十帧
   内出现明确的 `A -> B -> A` 身份往返；N18 可在方向差仅 `14 deg` 时接受
   `0.80 m` 匹配。旧公式 `kMatchDist=2*robot_dim+resolution=1.4 m` 把碰撞尺度用于
   身份关联，是这些错误匹配的直接放行条件。
2. 新增独立参数 `ContourGraph/static_match_tight_radius=0.4 m` 和
   `ContourGraph/static_match_max_radius=0.6 m`；代码无条件限制静态身份半径不超过
   `0.6 m`。PILLAR、CLIP 或方向证据不可靠的节点只使用 `0.4 m`。
3. CTNode/NavNode 单独保存 `free_space_dir` 及可靠性。CONVEX/CONCAVE 先提供自由侧
   候选符号，再沿正反方向在当前机器人中心配置空间的 `0.15/0.30/0.45 m` 附近
   采样；正向 FREE、反向 OCCUPIED 的多数证据确认方向，证据相反时翻转。越过
   `W_inner` 的空白、CLIP 和含混两侧均不充当可靠方向证据。
4. 两个可靠自由方向的有符号点积小于零时拒绝匹配；同向时只在 `0.4~0.6 m` 内
   插值半径。取消 `distance/match_radius` 评分，当前一对一分配直接按实际米制
   `distance` 排序，避免宽半径候选获得额外排序优势。
5. 明确动态节点使用独立参数 `dynamic_match_max_radius=1.4 m`，不再从 robot_dim
   推导。调试 CT 话题新增可靠绿色/不可靠灰色自由方向线及 `F+`/`F?` 标签。

验证记录：ROS Noetic Release 全工作区编译通过，far_planner 170 项自动测试全部通过。
对 `2026-08-13-02-24-30.bag` 从第 1 帧到第 2151 帧重新执行确定性逐帧回放，
2151 帧全部处理且无超时/崩溃。旧版的 `>0.75 m` 节点大跳由 235 次（分布在 205
帧）降为 0，因而此前 14 个由这些大跳组成的 `A -> B -> A` 身份往返全部消失。
代价是严格区分了更多历史角点：创建/删除由 `9990/9862` 增至 `10375/10206`，晋升
由 690 增至 868；Graph 平均节点由 123.84 增至 154.69、最终由 129 增至 170，
平均 edge 由 320.19 增至 530.49、最终由 342 增至 651。节点曲线在探索后稳定于
约 170~180，未随帧数继续线性增长；启发式“大量静态孤点”帧由 2051 降至 612，
Graph 主连通性反而改善。原有 15 帧 odom/start 断图没有改变，确认它与角点身份
匹配是独立问题。本 bag 没有明确动态点，动态半径分支仍只有单元/编译覆盖。

### 2026-09-11：统一配置空间轮廓与 Graph 整线碰撞模型

1. `ContourDetector` 不再把每个输入障碍 voxel 写成 3x3 基础格，也不再使用
   `INTER_LINEAR + boxFilter + 非零即占据` 形成隐式晕边。现在先单格二值投影，
   再把基础格中心无插值地映射到精细图，并用欧氏距离变换按
   `Util/robot_collision_clearance=0.45 m` 精确生成机器人中心配置空间。
2. 静态和明确动态障碍分别生成配置空间图。提取轮廓和 Graph 碰撞使用同一份
   `CV_8UC1` 二值快照；`resize_ratio=3` 时碰撞/轮廓栅格约为 `0.0667 m/格`。
3. `ContourGraph` 在配置空间可用时，以 supercover 栅格遍历检查候选线段经过的
   每一个格，替代 `0.15 m` 间隔采样和 `0.456 m` KD-tree 搜索球。斜线恰好经过
   栅格角点时同时检查两个正交邻格，禁止对角穿缝。原始点云 KD-tree 仅保留给
   未提供配置空间的兼容调用和旧测试。
4. 当前同一 Polygon 的 reduced-contour edge 由当前配置空间判定，不再被历史
   `global_contour_`、`unmatched_contour_` 或 `inactive_contour_` 反向否决；解决
   当前轮廓已经连续、却因旧 N9-N10 等历史线段得到 `OTHER_STATIC` 的缺边问题。
5. 当前轮廓相邻关系与运动可执行性分离：即使本帧自由侧路线因静态、动态或地形
   原因不可执行，也保留 `contour_connects + edge_states` 拓扑身份，但不调用
   `AddEdge()` 加入搜索图；后续帧可原位重验恢复。动态阻挡仍只临时屏蔽搜索边。
6. 当前 Polygon 已经是配置空间边界，因此局部 PILLAR 和历史轮廓检查不再额外
   叠加一次 `kNavClearDist`，避免双重膨胀。CLIP 的当帧使用、下一帧复核和三帧
   历史节点/edge 撤销策略保持不变。

主要修改文件：`src/contour_detector.cpp`、`include/far_planner/contour_detector.h`、
`src/contour_graph.cpp`、`include/far_planner/contour_graph.h`、
`src/dynamic_graph.cpp`、`src/far_planner.cpp` 和 `src/map_handler_test.cpp`。

验证记录：ROS Noetic Release 下 `far_planner` 与 `map_handler_test` 编译通过；
`map_handler_test _use_synthetic_map:=true` 共 141 项通过、0 失败，其中新增欧氏
膨胀、共享配置空间点查询、supercover 对角防穿缝和当前轮廓优先级回归。仓库
`run_tests_far_planner` 全部通过（Graph 生命周期/连接 46 项及其余分类、窗口、路径、
轮廓简化测试）。使用 `five_class_2026_08_13_bag_navigation.launch` 的逐帧模式从
第 1 帧推进到第 38 帧，所有帧均返回 `voxel=PROCESSED far=PROCESSED`。旧问题位置
在新配置空间轮廓中对应 `CT9/CT10 -> N345/N346`；该边同时出现在
`graph_after_update`、`viz_current_eligible_graph` 和 `viz_current_search_graph`，
`edge_diagnostics` 不再出现 `OTHER_STATIC`。

### 2026-09-10：CLIP 轮廓内部的旧角点完成三帧原子替换

1. 在五分类逐帧 bag 复现 `N11=(0.867,-4.0)`：第 1～6 帧它是当前轮廓角点并晋升
   `STATIC_GLOBAL`；第 7 帧起当前轮廓改为一条从持久角点延伸到 CLIP 的长边，但
   N11 因整个 polygon 带 `is_boundary_clipped` 而永远收不到拓扑否定证据，形成长期
   悬空节点。
2. 历史角点替代证据改为按局部线段判断，不再因远端存在 CLIP 而跳过整个 polygon。
   人工窗口闭合 cap 仍完全排除；投影点必须远离实际线段端点，并要求投影前后各
   `max(2*contour_grid_resolution, min(tolerance, boundary_guard))` 的局部段均位于
   `W_inner`。当前配置即前后各 `0.4 m`，因此靠近 W_guard/CLIP 的未知部分仍不能
   删除历史节点。
3. 已验证的“当前 `STATIC_GLOBAL`—瞬态 CLIP” contour-follow edge 现在可以作为旧
   内部角点的替代拓扑；至少一端必须是本帧匹配的已确认静态节点，CLIP—CLIP、普通
   未确认候选及无有效 clearance route 的边不能触发删除。CLIP 本身依然不晋升并在
   下一快照重新创建。
4. 不再要求替代端点 `is_finalized=true`。附近持续出现新 voxel 会重置 FAR 的
   RANSAC position/direction filter，使已晋升端点长期无法 finalized；端点的持久
   身份、本帧 CT 匹配、当前边碰撞校核和连续三帧线段证据共同承担稳定性。删除前的
   Graph 可达性检查仍保留，不能安全替代时节点停在 `topo=3` 而不会强删。
5. `/far_debug/graph_before_update` 和 `graph_after_update` 的静态节点标签在存在拓扑
   否定计数时追加 `topo=N`，可直接观察三帧累计过程。主要修改涉及
   `src/contour_graph.cpp`、`src/dynamic_graph.cpp`、`include/far_planner/node_struct.h`
   和 `src/debug_visualization.cpp`。

验证记录：ROS Noetic Release 编译通过；Graph 生命周期/连接策略 `46/46`、局部观察
窗口 `3/3`、`map_handler_test` 合成地图 `135/135` 通过。后者增加 frame-12 实际坐标
回归，分别验证 N11 所在物理 contour-to-CLIP 段能够提供证据、邻域触及 W_guard 的点
不能提供证据。五分类 bag 从第 1 帧重新执行至第 12 帧：N11 在第 7、8 帧显示
`topo=1/2`，第 9 帧 `/far_debug/node_events` 显示拓扑替代删除，第 9～12 帧不再出现
N11；当前持久角点—CLIP 轮廓边接替该局部轮廓。静态点云阻断仍会即时屏蔽搜索边，
本次没有放宽碰撞安全检查。

### 2026-09-10：旧静态角点拓扑替代确认改为三帧

1. 将 `Graph/static_topology_remove_frames` 的通用默认值及五分类 bag 覆盖值从 `5`
   改为 `3`。历史静态角点只有连续三次被可靠的当前轮廓线段内部观测所否定后，才
   达到拓扑替代成熟条件。
2. 本次仅缩短连续确认次数，不放宽删除安全条件：当前替代轮廓必须稳定、轮廓路线
   必须通过校核，而且删除旧节点后仍须保持当前 Graph 连通；任一条件不满足仍继续
   保留旧节点。
3. 此次只修改帧数时，被 `CLIP` 的 polygon 仍不提供旧节点消失证据，所以实测 N11
   仍保持历史身份。上方后续记录已将其修正为按 N11 附近的局部可靠线段判断，而不
   是放宽整个 CLIP polygon。

验证记录：ROS Noetic Release 编译通过；Graph 生命周期/连接策略 `45/45` 通过。
测试覆盖连续前两次否定不成熟、第三次否定成熟，以及中间未观测不累计、重新匹配
逐帧恢复计数的行为。

### 2026-09-10：扁三角形最长可视边稀疏化

1. 在 contour-follow、普通 visibility 和 odom/start 边全部完成当前帧碰撞校核后，
   增加扁三角形冗余边检查。若三条边当前均可搜索，候选直达边是严格最长边，且经
   第三点的两段绕行不超过直达距离的 `1.05`、三角形高/最长边不超过 `0.20`，则
   删除最长的普通 visibility 身份，保留两条较短替代边。
2. 按长度从大到小逐条处理；每次删除前重新确认两条替代边仍然活跃，因此重叠的
   多个扁三角形不会同时删掉全部旁路或切断原连通分量。contour-follow、trajectory
   和 goal edge 均为受保护身份，不会成为本规则的删除对象；它们可以作为证明一条
   visibility 长边冗余的短边。
3. 新增参数 `Graph/flat_triangle_pruning_enabled`、
   `Graph/flat_triangle_max_detour_ratio` 和
   `Graph/flat_triangle_max_altitude_ratio`。通用默认关闭，五分类 bag 配置启用并采用
   `true / 1.05 / 0.20`。该值最初按 `1.10` 完成下述验证，随后根据长距离扁三角形
   `N17-N21-N27` 的实测结果收紧为 `1.05`；新阈值只接受绕行增加不超过 5% 的边。
4. 被抑制边以 `TRIANGLE_SPARSIFIED` 诊断原因记录；`/viz_graph_topic` 新增独立青色
   namespace `visibility_triangle_sparsified`，颜色图例同步说明，便于与方向稀疏化及
   障碍阻断区分。

验证记录：ROS Noetic Release 编译通过；Graph 生命周期/连接策略 `45/45`、局部观察
窗口 `3/3`、`map_handler_test` 合成地图 `133/133` 通过。修改前五分类 bag 第 7 帧
eligible graph 为 79 条边；修改后逐帧从第 1 帧重新运行到第 8 帧，第 7 帧为 47 条边。
目标 `N2-N13` 已删除而 `N2-N58`、`N58-N13` 保留；`N1-N19` 已删除而 `N1-N20`、
`N20-N19` 保留。odom 所在原主连通分量的 25 个节点仍全部可达；原本即孤立的 N44
未被计为本次连通性退化。第 8 帧青色诊断层包含 `N2-N13` 与 `N1-N19`。

### 2026-09-10：发布可视化颜色图例

1. 新增 latched `MarkerArray` 话题 `/far_debug/color_legend`，每个已完成调试帧都在
   机器人附近刷新四列 `TEXT_VIEW_FACING` 图例，覆盖分类点云、静态/动态轮廓、CT
   类型、身份匹配、重复抑制、节点事件、Graph 节点/边状态、edge 事件以及
   eligible/search graph 的全部颜色含义。
2. `rviz/far_debug.rviz` 的 `Graph Result` 分组默认启用 `Color Legend`；图例独立于数据
   topic，避免同时打开多个 MarkerArray 时重复文字互相覆盖，并可单独关闭。
3. OpenCV `frame_summary_image` 顶部增加紧凑颜色提示，同时将标题区从 70 px 调整为
   90 px，不压住四个数据面板。
4. 根据实际 RViz 检查反馈，将 Marker 图例标题/正文放大到 `0.42/0.32 m`，行距增至
   `0.52 m`，四列使用独立 namespace；默认只展开 Graph 与 edge-event 两列，其余列
   可在 `Color Legend/Namespaces` 中按需打开，避免全部文字挤在一起。
5. 新增 latched `/far_debug/edge_diagnostics`，用 `Nxx-Nyy CONTOUR:原因` 直接标注未能
   生成的 contour-follow 候选；原始 visibility 拒绝仍留在 `/viz_graph_topic`，避免
   两类大量文字互相覆盖。
6. `graph_before/after` 对 contour route 补画“节点到投影路线”的首尾 anchor stub；
   `edge_events` 中旧身份删除线降低高度、当前新建线抬高，使同一几何位置发生
   CLIP 身份替换时最终的青色新边不会被红色旧边遮住。
7. `/viz_current_eligible_graph` 与 `/viz_current_search_graph` 改为 latched 快照发布。
   逐帧回放暂停时关闭再打开 RViz Display，新订阅会立即获得最近一次完整 Graph，
   不需要为了恢复显示而播放下一帧，也不使用周期重复消息占用带宽。

### 2026-09-10：修复相邻 CLIP 替代节点无法重建轮廓边

1. 在五分类逐帧 bag 第 2 帧复现：旧 CLIP 节点按设计被删除，当前 `CT15/CT16`
   分别新建为 `N28/N29`，但 `N13-N28` 与 `N28-N29` 虽是当前同一轮廓的相邻点，均
   被报告为 `CONTOUR:OUTSIDE_WINDOW`。
2. 根因是局部 box 的 `y_max=5.0 m`，轮廓处理 halo 为 `0.4 m`，而栅格角点为
   `y=5.400000095 m`；轮廓量化及 cloud-stamp/graph-update 位姿的毫米级偏差使严格
   `W_full + 0.4 m` 判断落在边界外。
3. 当前 CLIP 可用区改为 `W_full + boundary_guard + 1 个 contour cell`；本配置即
   `5.0 + 0.4 + 0.2 m`。额外一格只用于当前帧 CLIP 的 edge/start/goal 校核，仍不参与
   静态节点确认、删除证据或持久化，因此不会把窗口外历史结构误认为已观测。

验证记录：ROS Noetic Release 编译通过；`map_handler_test _use_synthetic_map:=true`
共 133 项通过、0 失败。重新启动
`five_class_2026_08_13_bag_navigation.launch` 并精确执行第 1、2 帧，两个事务均返回
`voxel=PROCESSED far=PROCESSED`；第 2 帧 `/far_debug/edge_events` 明确出现青色新建
`N13-N28` 和 `N28-N29`，`/far_debug/edge_diagnostics` 不再报告这两条 contour edge，
`graph_after_update` 的 active route 已通过 anchor stub 连到 N13/N28/N29。暂停在第 2
帧后对 eligible/search topic 分别建立两轮全新订阅，每次都在 `17 ms` 内收到同一份
`22 nodes / 76 edges` 的锁存快照，期间没有推进 bag。

### 2026-09-10：阻止 CLIP visibility edge 穿过真实轮廓边

1. 修复 `boundary-clipped polygon` 被整体跳过的问题。此前为避开 OpenCV 对裁剪轮廓
   生成的人工闭合边，Polygon 碰撞检查会忽略整条轮廓，导致首帧
   `CT2(6.20,5.40)-CT14(3.00,4.67)` 穿过 `CT0-CT1-CT2-CT3` 障碍区域后仍进入
   current search graph。
2. 现在逐段处理裁剪轮廓：只有“两端均不在可靠窗口内，并且共同贴在同一条局部
   观察窗口外边界/处理 halo 上”的段才作为人工 cap 豁免；其余已观测轮廓段继续
   参加静态或动态 Polygon 碰撞检查。CLIP 点从自由侧发出的合法连接不受影响。
3. 新增当前 bag 首帧坐标回归，验证穿过真实 `CT3-CT0` 边的 CLIP chord 返回
   `POLYGON_BLOCKED`，同时验证从 CT2 向窗口外自由侧连接仍返回 `NONE`。

验证记录：ROS Noetic Release 编译通过；`map_handler_test` 合成地图 132 项全部通过；
局部窗口 3 项、Graph 生命周期/连接 43 项、最短路 3 项、terminal visibility 5 项、
waypoint projection 4 项、轮廓简化/栅格对齐 6 项全部通过。使用
`five_class_2026_08_13_bag_navigation.launch` 逐帧执行第 1 帧后，Graph 中原
`N4(CT2)-N14(CT14)` 活跃边消失，而 `N3-N4` 与 `N4-N5` 两条 0.15 m 投影的
contour-follow 边均保留。

### 2026-09-10：CLIP 当帧连图、深凹轮廓折线与自由侧方向修正

1. `W_guard` 及轮廓处理 halo 内、且本帧明确匹配的静态 CLIP 节点现在可参与
   obstacle visibility、contour-follow、odom/start 和 goal 连接。所有边仍逐条经过
   当前静态云、明确动态云、Polygon 与 terrain 检查；真正窗口外的端点继续拒绝。
2. CLIP 身份仍为单帧对象：不能累计 `static_seen_count` 或晋升；下一快照若匹配到
   非 CLIP 真角点则清除瞬态标志并重新开始静态确认，否则节点和附属 edge 同步删除，
   当前帧再创建新的 CLIP。这避免移动的裁剪端点和边跨帧累积。
3. 删除 reduced-contour adjacency 中 `0.6 m` 圆柱近似作为“是否存在 edge”的条件。
   两个保留 NavNode 之间未入 Graph 的 CTNode 现在成为 edge 内部 route support；例如
   `CT5-CT6-CT7` 保存为三点机器人中心折线，而不是危险的 `CT5-CT7` 直弦，也不会因
   CT6 是凹角而丢失整条轮廓通路。
4. `GraphEdgeState` 保存有方向的 `route_points`；route cost、静态/动态复检、terrain
   检查、GraphPlanner 局部 waypoint、RViz 线路和 OpenCV 四联图均沿完整折线处理。
5. 凸角连接方向由“只允许两侧切向扇区”改为“仅排除两条表面射线围成的障碍内部
   扇区”。障碍位于第一象限时，第二、第三、第四象限均可作为几何候选；最终仍由
   碰撞验证与同方向稀疏化控制边数。
6. 修复 bag 首 cloud 早于首条动态 TF 时逐帧播放器无法启动的问题：只对首次出现的
   child frame 生成一次最近未来 TF bootstrap，不改变后续记录时间线。

验证记录：ROS Noetic 独立容器 Release 编译通过；`LocalObservationWindow` 3 项、
Graph 生命周期/连接策略 43 项、最短路策略 3 项全部通过；`map_handler_test`
合成地图共 129 项通过，新增深凹三点 route 和 Q1 障碍/Q3 自由方向回归均通过。
使用 `five_class_2026_08_13_bag_navigation.launch` 的逐帧模式先从第 1 帧连续执行至
第 4 帧；加入“CLIP 不复用旧 CLIP 身份”的最终收紧并重新编译后，又重新执行第 1、
第 2 帧。各帧均返回 `voxel=PROCESSED far=PROCESSED`，所有核心节点保持存活。第 1
帧 contour-follow active=19；start 候选 19、通过 13、窗口外预过滤 0。最终第 2 帧
`node_events` 显示上一帧 7 个边界端点被删除并创建当前端点，未发生 CLIP 累积。

### 2026-09-10：修正逐帧播放器默认 bag 路径

1. 默认 bag 改为相对 `$(find far_planner)` 定位，消除宿主机与 Docker 中 `HOME`、
   工作区挂载前缀不同导致的启动即退出。
2. stepper 在调用 rosbag 解析前显式检查文件可读性；失败时打印完整展开路径和
   `bag:=...` 修复方式，避免真正异常被 `required process has died` 的连锁关闭信息
   淹没。

### 2026-09-10：OpenCV 四联图独立窗口

1. 增加 `Debug/show_opencv_window` 及窗口初始宽高参数，直接显示每帧生成的
   `frame_summary_image`，同时保留原 ROS image topic。
2. FAR 主循环持续泵送 HighGUI 事件，使逐帧暂停期间窗口仍能移动、缩放和重绘。
3. 无 `DISPLAY` 或窗口创建失败时自动降级到 ROS topic，不中止规划进程。

### 2026-09-09：两阶段逐帧 Graph 调试链路

1. 在点云最终分类处旁路记录四类点和原始语义字段，不改变进入规划点云的内容。
2. 在静态/动态轮廓各自完成投影与提取时保存基础图、处理图和最终轮廓。
3. 在 CTNode 与历史 Graph 匹配处记录候选半径、方向角、可视性、一对一分配结果和
   重复抑制关系。
4. 在 `BeginSemanticGraphUpdate()` 前及 `CommitSemanticGraphUpdate()` 后按稳定 node id
   抓取 Graph 快照，用差分生成节点和 edge 生命周期事件；最终 eligible/search graph
   使用同一已提交快照。
5. 增加 `far_debug.rviz` 和四联图，所有新增发布由 `farDebug` 总开关控制。
6. 增加 bag frame stepper，使用 cloud stamp 对 voxel/FAR 两级 ack，并针对首帧
   `map_start` 建立顺序延后 odom 发布。
7. 保留连续回放路径；`stepReplay` 只负责在两种播放器间互斥切换。

当时验证记录：在 ROS Noetic 容器中完成全工作区 Release 编译；原实现从第 2 帧起
可形成完整事务，第 1 帧因 TF 晚 0.1 s 报告 `TF_MISSING`。这个首帧限制已由上面的
2026-09-10 TF bootstrap 修正。相关 `LocalVoxelPolicy` 5 项测试当时全部通过。

### 2026-09-09：局部 voxel 单一几何源与 Graph 生命周期适配

修改过程和设计决策：

1. 将本地 voxel 消息按 `label + semantic confidence + traversability/intensity`
   原子分类为静态障碍、明确动态障碍、地形支撑和忽略点。
2. 修正瞬态层语义：只有明确动态标签进入 `TRANSIENT_OBSTACLE`；未知或低置信度
   的高代价几何不再因“不确定”而被当作动态障碍。
3. 本地模式取消全局占用地图与当前局部几何的复合；当前静态/动态点云负责当前
   碰撞，历史 Graph 只保存经过多帧确认的拓扑身份。
4. 静态和动态障碍分别提取轮廓，避免动态目标与墙体在同一幅轮廓图中粘连。
5. 引入与采集位姿绑定的旋转非对称观察窗口，匹配 SSMI/五分类上游局部 box。
6. 引入裁剪端点标志，阻止 OpenCV 在窗口边界形成的人工闭合角点晋升为持久节点。
7. 历史静态节点只在可靠观察窗口内重验证；窗口外节点冻结，机器人返回后再恢复
   匹配和维护。
8. 静态 edge 使用连续阻断观测维护；当前动态障碍只临时关闭 edge，不删除静态
   edge 身份。静态 edge 的无障碍恢复要求整条验证路线位于可靠观察区。
9. 增加局部窗口、点云分类、地图双输入兼容和 Graph 生命周期相关测试及诊断输出。

窗口持久化证据仍以 `W_inner` 为唯一可靠区；当帧 CLIP 路由则额外使用 `W_full`
和处理 halo 查询，使 guard 与真正 outside 不再共用 start/goal/edge 行为。

### 2026-09-11：稠密轮廓安全路由与历史 edge 全路线复检

问题：只保留 RDP 简化后的角点时，角点之间的直弦可能切过圆角、凹口或局部毛刺；
如果某个旧节点下一帧没有匹配 CTNode，过去的 edge 还可能绕开当前轮廓邻接检查，
继续穿过新观测到的静态障碍。

本次修改沿用 FAR Planner 的“简化角点维护拓扑、当前障碍负责可执行性”分层方式，
但不再让简化多边形本身充当唯一的运动几何：

1. `findContours` 改用 `CHAIN_APPROX_NONE` 保存逐像素稠密边界；RDP 和原有相邻点
   过滤仍只生成 CTNode。每个简化顶点同时保存其对应稠密边界下标。
2. 同轮廓 edge 先沿正确方向取得稠密边界弧，再投影到机器人中心的自由侧。每个
   简化 CT 角点（包括凹角支撑点）都是不可跨越的路线锚点，避免把
   `CT5-CT6-CT7` 直接简化为穿障碍的长弦。
3. 稠密路线只在替代线段通过当前配置空间碰撞检查时才做有界步长简化。拓扑邻接
   可以保留为 edge 身份，但碰撞失败的 edge 不进入 search graph。
4. 普通静态可见 edge 也保存创建时实际使用的投影路线，而不只保存两个障碍角点
   之间的原始直线。
5. 每帧对 `connect_nodes + contour_connects + poly_connects` 的并集做历史复检；只要
   保存路线的任一分段与 `W_inner` 相交，就用当前静态/动态配置空间检查。因此旧点
   没有匹配当前 CTNode，也不能让 edge 绕过新障碍。
6. 当前静态阻挡在第一次观测时立即把 edge 从 search graph 屏蔽；连续 3 个已接收
   的局部快照都阻挡后才删除 edge 身份。当前动态阻挡只做当帧屏蔽。无障碍恢复仍
   要求完整路线位于 `W_inner`，部分观测不会被当作 FREE 证据。
7. 当前同轮廓边和当帧 near-near 可见边已在各自创建阶段验证，历史复检会跳过它们，
   防止一帧内把阻挡计数累计两次。

主要涉及 `contour_detector.*`、`contour_graph.*`、`node_struct.h`、
`dynamic_graph.cpp` 和 `far_planner.cpp`。没有引入全局占用地图；几何依据仍只有当前
局部语义 voxel 快照，Graph 只负责保存跨帧身份与路线。

验证记录：ROS Noetic 容器中 `far_planner`、`map_handler_test` 和
`graph_lifecycle_policy_test` 编译通过；Graph 生命周期策略 49 项全部通过，新增测试
确认静态物理阻挡首帧屏蔽、连续 3 帧才允许删除身份；地图合成测试的稠密/简化轮廓
映射、深凹三角支撑和狭窄通道用例通过。指定五分类 launch 以逐帧模式从第 1 帧连续
推进至第 36 帧，各帧均返回 `voxel=PROCESSED far=PROCESSED`，Graph 更新约
4--15 ms，未发生节点崩溃、处理超时或逐帧确认丢失。

### 2026-09-11：当前轮廓桥接节点与投影端点安全校核

逐帧检查发现两类后处理问题：当前轮廓中间已经匹配的历史静态节点，可能因为不在
持久主连通分量而被 eligible/search graph 过滤，使两侧已验证轮廓边同时丢失；另一类
历史角点的固定自由侧投影可能仍落在新一帧配置空间障碍内，但旧的端点排除距离会把
这段占用隐藏起来。

1. `STATIC_GLOBAL` 仍以持久主连通分量作为通常准入条件；增加一个严格的当帧例外：
   节点必须本帧被观测并匹配当前 CTNode，而且至少有一条连接同一当前多边形、双向
   active、以 `CONTOUR_FOLLOW` 验证的 edge。它只作为当前 search graph 的局部轮廓桥，
   不修改其持久主图身份。
2. 不跨过中间已匹配角点建立直连。例如原来的 N333--N334 仍不直连，而由中间 N9
   保留 N333--N9--N334 两条真实轮廓边。
3. 普通可见 edge 在检查整条投影路线前，先要求每个实际移动过的投影端点本身位于
   当前静态（以及启用时的动态）配置空间 FREE 单元；两端均能正常投影时，整条路线
   使用零端点盲区。PILLAR/方向未知点无法产生独立机器人中心端点，继续保留两格以内
   的兼容排除。
4. 对相距约 0.422 m、自由方向相差约 75 度的 N17/N338 类相邻凸角不做合并。当前
   `static_duplicate_radius=0.4 m` 已先在距离上拒绝；即便放大为 0.5 m，同角点方向
   约 30 度的一致性门限仍应保留这段真实短轮廓边，因此不调整该参数。

验证记录：Graph 生命周期策略 50/50 通过；地图/轮廓合成测试确认投影端点占用会
立即得到 `STATIC_CLOUD_BLOCKED`，清空占用后同一路线恢复。独立 ROS master 上将指定
五分类 bag 推进到对应 `bag_time=1786559075.213378`：原场景坐标对应的三个点
`(1.933,-5.067)`、`(5.333,-3.867)`、`(6.733,-3.467)` 均进入 eligible/search graph，
并明确存在前两点和后两点之间的两条 edge。

### 2026-09-11：局部 voxel 输入的 0.01 m XY 开运算预处理

为避免离散毛刺点同时进入轮廓生成和静态碰撞检查，FAR 在收到完整的局部 voxel
消息、完成语义分层之后、提交 `SetLocalVoxelSnapshot()` 之前统一执行预处理：

1. 最初直接使用 `0.10 m` 输入 voxel 网格和 `0.10 m` 开运算半径；bag 第 1 帧静态点
   从 351 减少到 84，说明一格宽结构元素会过度删除细障碍。当前实现将形态学栅格与
   输入 voxel 分辨率解耦，固定使用 `0.01 m/pixel` 的细图。
2. 上游点云仍是 `0.10 m` voxel。若只把一个 voxel 画成细图上的一个像素，相邻点会
   相隔约 10 格并被腐蚀全部删除；因此每个点按其真实 `0.10 x 0.10 m` voxel 占用面积
   填入细图。这是输入 voxel 已代表的面积，不是额外障碍膨胀。
3. 静态障碍、明确动态障碍和地形支撑分别栅格化，避免同一 XY 的稠密地面替孤立
   障碍通过形态学检查。每一层使用椭圆结构元素执行开运算，即先腐蚀、再膨胀。
4. `local_voxel_morphology_raster_resolution` 默认固定为 `0.01 m/pixel`；日常只调整
   `local_voxel_morphology_radius`，其默认值也是 `0.01 m`，对应 1 格半径和 `3 x 3`
   结构元素。物理半径会始终根据细图分辨率换算成像素半径。
5. 只保留原始 voxel 中心在开运算后仍被占用的三维点。膨胀阶段不会生成新的三维
   高度点；同一保留 XY voxel 内的多个高度样本全部保留。
6. 过滤后的三层点云作为一次原子 snapshot 提交，因此后续轮廓栅格、静态/动态
   碰撞检查、edge 复检、局部规划点云和 Graph 更新使用的是同一份预处理结果，原始
   被删除点不会再通过其他 FAR 点云旁路参与几何判断。
7. `local_voxel_morphology_enabled=false` 可整体关闭。无效参数、非有限坐标或异常大的
   投影范围采用 fail-open，保留该层原始点，避免预处理故障静默清空碰撞几何。
8. 调试分类点云保留被删除点用于对照，`debug_class=4`，RViz/OpenCV 图例为青色
   `morphology removed`；这些点仅用于显示，不进入 FAR 几何层。

主要涉及 `local_voxel_morphology.h`、`far_planner.cpp`、调试可视化、三套配置及
`local_voxel_morphology_test.cpp`。测试覆盖细图半径换算、输入 voxel 面积栅格化、不同
高度样本映射、开关关闭和异常范围 fail-open。由于默认 `0.01 m` 半径小于输入 voxel
半宽，它主要修整 voxel 占用面积的细像素边缘，通常不会删除一个完整的孤立
`0.10 m` voxel；若希望删除这种单 voxel 毛刺，半径必须大于约 `0.05 m`。更新后的
形态学测试 4/4、Graph 策略回归 50/50 通过；指定 bag 第 1 帧得到静态
`351 -> 351`、地形 `909 -> 909`，符合 0.01 m 半径不误删完整输入 voxel 的预期，
且逐帧事务正常返回 `voxel=PROCESSED far=PROCESSED`。

### 2026-09-11：稠密轮廓失败时的凸包绕行与 UNKNOWN 身份归并

对照上游 FAR Planner 后，没有把当前提取器整体退回原版的稀疏轮廓流程。上游以
`findContours + approxPolyDP` 得到稀疏多边形，适合生成少量拓扑角点，但没有当前
fork 已具备的预处理后统一配置空间栅格、稠密边界映射和逐段碰撞校验。直接替换会
重新引入 RDP 弦线靠近或切入障碍的问题。

也没有根据当前 start/goal 是否落入某个障碍凸包，逐帧切换该障碍的“原轮廓/凸包”
身份。障碍表示一旦依赖查询端点，机器人或目标移动就可能改变 CTNode 数量和顺序，
而且直接用凸包替换凹障碍会永久抹掉真实可通行的凹槽。当前采用更保守的组合方式：

1. 输入仍先经过 `0.01 m` 细图开运算，再按实际
   `robot_collision_clearance` 生成统一配置空间栅格。
2. 原始稠密轮廓和 RDP 角点仍是首选；同轮廓 edge 先沿稠密边界向自由侧投影并逐段
   验证。
3. 只有非 PILLAR、非 CLIP 的静态轮廓在所有稠密投影距离均失败时，才计算该障碍的
   凸包。程序选取与原稠密边界弧相对应的一侧，构造向外偏移的机器人中心折线。
4. 凸包只提供一次候选绕行路线，不替换 `Polygon`，也不把凸包内部宣告为自由空间；
   候选的每个支撑点必须位于当前观察窗口，每一段仍须通过当前预处理后静态配置空间
   和原多边形校验。明确动态层可继续当帧屏蔽该路线。

跨帧节点维护增加了有锚点的静态历史身份归并。保留方必须是本帧已经一对一匹配到
CTNode 的普通静态节点；待淘汰方必须是 `W_inner` 内未匹配、非 CLIP、同角点类型、
同高度、距离不超过 `Graph/static_duplicate_radius`，并具有同向且可靠的有符号自由
方向。对待淘汰节点位置的证据定义为：

```text
UNKNOWN       -> 允许进入身份归并候选
EXPLICIT_FREE -> 允许进入身份归并候选
STATIC_OCCUPIED -> 拒绝归并，保留独立静态身份
```

这里 `UNKNOWN` 只说明“没有证据支持这是另一个独立障碍角点”，不是节点消失证据。
普通静态删除策略仍然要求连续 `EXPLICIT_FREE` 或成熟的轮廓拓扑替代证据。归并提交前
还会模拟移除旧节点；如果会降低机器人当前可达性或切断静态拓扑，旧节点继续保留，
不会简单按距离聚类，也不会无校验地搬移旧 edge。调试开启时，成功归并显示在
`/far_debug/duplicate_suppressed`，文字格式为 `MERGE N旧 ->N保留`。

验证记录：ROS Noetic Release 全目标编译通过；Graph 生命周期/连接策略 52/52、
地图与轮廓合成测试 148/148 通过。新增用例覆盖 `UNKNOWN` 与 `EXPLICIT_FREE` 均可
归并、`STATIC_OCCUPIED` 拒绝归并、相反自由方向拒绝归并，以及凹边稠密投影被阻挡后
凸包自由侧路线仍可通过完整碰撞校验。指定五分类 launch 重新加载新二进制后逐帧从
第 1 帧推进到第 60 帧，全部返回 `voxel=PROCESSED far=PROCESSED`；第 50 帧实际记录
到一次 `UNKNOWN` 归并（旧 N22 -> 当前 N512，距离 0.267 m），未出现 frame 13 停滞、
处理超时或进程退出。

## 变更记录维护约定

### 2026-09-12：FAR 分步恢复——第 1 步，拆分拓扑图与安全碰撞图

本步只修改轮廓检测器的图像输入职责，没有修改 `ContourGraph`、CLIP 生命周期、
节点匹配或 Graph edge 维护：

1. 同一份当前预处理静态/动态障碍点先投影为基础占据图。
2. 新增独立 FAR 拓扑图：基础占据格做 `3x3` 邻域扩张，随后
   `INTER_LINEAR resize` 和非归一化 `boxFilter`。参数
   `CDetector/topology_blur_size` 默认 `2`，不再由机器人净空隐式控制。
3. `findContours/RDP` 改为消费拓扑图；本步仍保留现有
   `CHAIN_APPROX_NONE`、稠密轮廓映射和后续稠密边验证，下一步再单独切换为 FAR
   的 `CHAIN_APPROX_TC89_L1` 稀疏链路。
4. 原有欧氏距离变换配置空间继续严格使用原始基础占据格和
   `robot_collision_clearance`，只提供给 edge/waypoint 碰撞检查；FAR 的 3x3 和
   boxFilter 不进入该图，因此不会形成第二次机器人净空膨胀。
5. `/far_debug/raster/static_processed` 与 `dynamic_processed` 现在显示 FAR 拓扑图及
   黄色提取轮廓，不再表示配置空间图。

验证：ROS Noetic 容器内 `far_planner` 与 `map_handler_test` 编译通过；新增合成检查
确认拓扑图和配置空间图尺寸一致但占据内容不同，同时原有单点 `0.45 m` 欧氏配置
空间边界检查通过；Graph 生命周期与连边策略回归测试 `52/52` 通过。本步未启动 bag，
也没有评价实际场景 Graph 效果，避免把后续 TC89/拓扑连边变化混入本步结论。

### 2026-09-12：FAR 分步恢复——第 2 步，切换 TC89 稀疏轮廓

本步只改变第 1 步 FAR 拓扑图之后的轮廓链近似，不修改 Graph 连边、CLIP 或跨帧
节点生命周期：

1. `findContours` 从 `CHAIN_APPROX_NONE` 切换为原版 FAR 使用的
   `RETR_TREE + CHAIN_APPROX_TC89_L1`。
2. TC89 source chain 继续经过原版顺序的 `approxPolyDP`、`TopoFilterContours` 和
   `AdjecentDistanceFilter`；静态 RDP 物理容差仍为
   `1.5 * contour_grid_resolution`，五分类配置下为 `0.30 m`。
3. 取消 RDP 之后额外执行的世界坐标 `collinear_tolerance/collinear_angle_deg` 删点，
   避免在 FAR 已完成角点简化后再次改变轮廓顶点身份。两个参数暂时保留用于旧配置
   兼容，但不参与 FAR 稀疏链路。
4. `RemoveWallConnection` 的角度恢复为原版 FAR 的
   `cos(Util/accept_max_align_angle / 2)`；当前五分类值为 `4 deg`。
5. 现有 `dense_*` 数据名暂时承载有序 TC89 source chain，并维持 RDP 顶点到 source
   chain 的下标映射，避免本步同时改变 Graph。它已不再是逐像素稠密边界；下一步
   才会让同轮廓 edge 停止把该 chain 当作机器人要执行的路线。

验证：ROS Noetic Release 编译通过；轮廓合成检查确认 RDP 顶点仍能有序映射到 TC89
source chain，独立配置空间测试继续通过且未出现失败项；Graph 生命周期/连边策略
`52/52` 通过。指定五分类逐帧 launch 在关闭 RViz/局部规划器的情况下推进 3 帧，
三帧均返回 `voxel=PROCESSED far=PROCESSED`；第 1 帧生成 25 个 local contour
vertices、19 条 active contour-follow edges，进程无崩溃。测试结束后已关闭 launch
及 ROS master。本步只证明链路可运行，不据此判断最终 Graph 连通效果。

### 2026-09-12：FAR 分步恢复——第 3 步，轮廓 edge 恢复为拓扑关系

本步只改变同一当前轮廓上两个 Graph 节点之间的 `CONTOUR_FOLLOW` 语义；普通跨轮廓
可见 edge、start/goal query edge、CLIP 生命周期和跨帧节点删除策略均未修改：

1. `GetContourChain()` 恢复原版 FAR 的前后向遍历规则：优先按 `front` 顺序查找；遇到
   另一个已经匹配 Graph 的 CTNode 立即停止；未匹配的中间 CTNode 只有位于端点弦线
   `FARUtil::kNearDist` 圆柱内才可跳过。当前 `kNearDist=robot_dim=0.5 m`，它不是
   `robot_collision_clearance`。
2. 深凹折线不再被藏进一条 edge 的稠密执行路线中。如果中间角点离弦线超过
   `kNearDist`，端点直连失败；现有 `EnclosePolygonsCheck()` 会把阻断角点标为
   `is_contour_necessary`，随后由 `端点—中间点—端点` 分段承接轮廓拓扑。
3. `ValidateContourFollowEdge()` 不再进行自由侧多档投影、TC89 source chain 采样、
   配置空间逐段碰撞或凸包绕行。成功结果只包含两个轮廓端点和欧氏拓扑代价，
   `has_clearance_geometry=false`；这条 RViz 显示弦不是机器人将执行的中心轨迹。
4. 同轮廓拓扑 edge 不再因为静态/动态点落在显示弦上而被屏蔽，也不会在历史 edge
   复检时把原始弦线送入点云碰撞检查。轮廓相邻性的连续确认/矛盾仍负责其跨帧身份；
   上游 FAR 原有的地形连接检查继续保留。
5. 真正运动时仍由 waypoint 自由侧投影/反弹以及 start、goal、普通 visibility query
   edge 的配置空间检查提供机器人净空。因此窄通道可以保留“障碍轮廓相邻”这一拓扑
   事实，但不等于机器人可以直接沿显示弦穿行。
6. 当前轮廓替代旧 topology edge 时，不再要求替代 edge 保存
   `has_clearance_geometry`；只要求两端本帧确认、同属当前轮廓且双向 active，从而避免
   把原本成立的 FAR 拓扑替代误判为不完整。

主要涉及 `src/contour_graph.cpp`、`src/dynamic_graph.cpp`、
`include/far_planner/node_struct.h` 及相应回归测试。旧的投影/凸包辅助函数暂时作为未调用
兼容代码保留，不再位于 `CONTOUR_FOLLOW` 生产路径；TC89/RDP 映射仍用于诊断。

验证：ROS Noetic Release 编译通过；`map_handler_test _use_synthetic_map:=true` 为
`146/146`，Graph 生命周期/连边策略为 `52/52`。指定五分类逐帧 launch 从第 1 帧推进
到第 12 帧，全部返回 `voxel=PROCESSED far=PROCESSED`，候选晋升和连续三帧拓扑替代
正常，未发生崩溃或事务超时。测试结束后已关闭 launch 和 ROS master。

### 2026-09-12：FAR 分步恢复——第 4 步，限定 CLIP 为当前窗口端点

本步只收紧裁剪轮廓的人工闭合边，不改变普通静态角点、跨轮廓 visibility edge 或
节点晋升/删除阈值。CLIP 继续采用“当前帧可用、跨帧不继承身份”的策略：

1. OpenCV 会把被局部观测窗口截断的障碍轮廓闭合成多边形，其中沿窗口边界连接两个
   CLIP 点的线段只是栅格裁剪产生的人工封口，并不代表实际观测到的障碍表面。
   `GetContourChain()` 现在会识别并拒绝穿过该人工封口的前/后向链，因此它不会再生成
   `CLIP—CLIP` 的 `CONTOUR_FOLLOW` 假拓扑 edge。
2. CLIP 顶点本身没有被禁用。沿真实障碍表面的 `内部角点—CLIP` 轮廓关系仍可建立；
   当前帧明确观测到且已经匹配 CTNode 的凸 CLIP，仍可作为 start/goal 连接候选，也仍可
   参与通过多边形和局部碰撞校验的普通 visibility edge。
3. CLIP 不匹配上一帧的任何普通静态节点或旧 CLIP，不累计静态确认次数，也不晋升为
   `STATIC_GLOBAL`；若下一帧仍被窗口裁剪，会按新观测重新创建。若同一物理位置后来变成
   非 CLIP，它按普通静态角点重新进入候选、连续观测和晋升流程。
4. 没有放开凹角作为 start/goal 锚点。修改前检查五分类 bag 前 20 帧时，当前 CLIP 均为
   `CONVEX`，因此现有凹角过滤不是这些场景中 CLIP 断边的原因。

主要涉及 `src/contour_graph.cpp`、`src/map_handler_test.cpp` 和
`src/graph_lifecycle_policy_test.cpp`，没有新增配置参数。回归测试增加了第一帧裁剪轮廓：
人工 `CT2—CT3` 封口必须被拒绝，而物理 `CT3—CT0` 轮廓边必须保留；同时显式验证当前
CLIP 可作为 start/goal 候选、离开当前观测后立即失去该资格。

验证：ROS Noetic Release 编译通过；`map_handler_test _use_synthetic_map:=true` 为
`148/148`，Graph 生命周期/连边策略为 `52/52`。指定五分类逐帧 launch 从第 1 帧推进
到第 12 帧，全部返回 `voxel=PROCESSED far=PROCESSED`。第 1 帧 active
contour-follow edge 从修改前的 18 条降为 15 条，而 start edge 仍为
`unique=20, validated=20, accepted=15, no_topology_edge=0`：删除的是 3 条人工窗口封口，
没有使当前 CLIP 或普通节点失去 start 可达性。候选晋升、三帧拓扑替代及事务处理正常，
未发生崩溃或超时。测试结束后已关闭 launch 和 ROS master。

### 2026-09-12：FAR 分步恢复——第 5 步，CLIP_ATTEMPT 与执行安全

本步完成第 4 步之后保留的 CLIP 未知方向语义。第 4 步“拒绝人工封口”是中间状态，
本步将其替换为显式、可撤销但不冒充已确认轮廓的尝试关系：

1. 新增 `EdgeValidationMode::CLIP_ATTEMPT`。只有同一当前裁剪多边形上的两个 CLIP，
   且 front/back 链确实穿过 OpenCV 人工窗口封口时，才能得到该模式。它可进入当前帧
   Graph 搜索，但 `has_clearance_geometry=false`，不等于观察到了障碍表面，也不等于
   整条封口弦具有机器人净空。
2. 真实 `内部角点—CLIP` 或沿实际障碍表面的 `CLIP—CLIP` 仍标为
   `CONTOUR_FOLLOW`；人工封口不会参与持久静态替代、静态晋升或旧 edge 恢复。
   CLIP 下一帧重新创建，因此 `CLIP_ATTEMPT` 与端点一起自动撤销。
3. CLIP 自由方向只使用非人工封口邻边作为真实轮廓切线，并比较两侧法向在当前静态
   配置空间图中的 FREE/OCCUPIED 采样；还要求至少两个采样位于当前观测窗口。由此人工
   封口方向不再污染 waypoint 外推。
4. waypoint 外推优先使用上述已验证有向自由方向。任何投影或动量修正后的最终 waypoint
   若越出当前观测窗口，或被当前静态、动态、地形层阻挡，就不发布为安全前进点，而是
   返回机器人当前位置停止等待下一帧。窗口外方向仍由 `CLIP_ATTEMPT/free_space_dir`
   表达为探索意图，但不是已确认执行几何。
5. `/far_debug/graph_after_update` 中 `CLIP_ATTEMPT` 为粉红粗线；
   `/far_debug/edge_events` 中新建的尝试边也是粉红线；
   `/viz_current_eligible_graph`、`/viz_current_search_graph` 和 `/viz_graph_topic` 使用粉红
   覆盖线显示尝试边。颜色说明同步加入 `/far_debug/color_legend`。
6. `five_class_live_navigation.launch` 和指定 bag launch 新增
   `farMainRunFreq` 启动参数，默认仍为 `2.5 Hz`。它只用于把自动逐帧回归临时提高到
   `50 Hz`，不改变人工逐帧调试节奏或部署默认值。

主要涉及 `include/far_planner/node_struct.h`、`src/contour_graph.cpp`、
`src/dynamic_graph.cpp`、`src/far_planner.cpp`、两套可视化代码、相关测试和 launch。

验证：ROS Noetic Release 编译通过；Graph 生命周期/连边策略 `53/53`，waypoint 投影
策略 `5/5`，`map_handler_test _use_synthetic_map:=true` 为 `149/149`。新增用例验证了
人工封口得到 `CLIP_ATTEMPT`、真实轮廓边仍为 `CONTOUR_FOLLOW`、尝试边可搜索但没有
净空几何、CLIP 法向来自真实墙面自由侧，以及窗口外 waypoint 不能成为已确认执行点。
指定 bag 从 `bagStart=0.15` 自动逐帧处理到末尾，共成功完成 2150 个点云帧；第 2151 次
`next` 正确返回 `DONE: end of bag`。全程每帧均为
`voxel=PROCESSED, far=PROCESSED`，包括此前关注的第 13 帧；日志未出现
`ERROR/FATAL`、TF 失败、处理超时或节点退出。日志周期采样中首帧为
`active=15, clip_attempt=3`，后续窗口移动时持续出现 1～3 条尝试边。测试结束后已关闭
launch、FAR、逐帧播放器及 ROS master。

### 2026-09-12：CLIP 封口显示与跨轮廓 visibility 窗口约束

五分类 bag 第 1 帧中，`N4—N5`、`N6—N7`、`N15—N16` 已经作为三条
`CLIP_ATTEMPT` 存在于 Graph，但 `/viz_current_eligible_graph` 和
`/viz_current_search_graph` 把装有 3 对端点的 Marker 误发成默认 `ARROW(type=0)`，
导致 RViz 看起来像是没有轮廓边。同时，不同裁剪轮廓上的 `N5—N15` 被当作普通
`VISIBILITY`；它的机器人中心投影端点约为 `(4.780, 5.522)` 与
`(3.106, 5.506)`，已经越过上游局部体素窗口的 `max_y=5.0 m`，却仍进入了当前
eligible/search Graph。

本次修改：

1. 两个当前 Graph 话题的 `clip_attempt_marker.type` 显式设为 `LINE_LIST`，三条人工
   封口现在以粉红粗线稳定显示。
2. 新增 `IsSegmentFullyInsideCurrentExecutionWindow()`，检查零 halo 的上游局部体素
   足迹。`IsPointInsideCurrentObservationWindow()` 仍保留 guard 和一个轮廓栅格的
   容差，只用于解释 CLIP 拓扑锚点；这部分容差不再被普通跨轮廓 visibility 当作已
   观测自由空间。
3. 只要普通跨轮廓 visibility 的任一端是瞬时 CLIP，其投影后的机器人中心线段就必须
   完全位于上述精确窗口内，否则返回 `CLIPPED_CONTOUR`，不创建可搜索 visibility
   edge。第 1 帧 `N5—N15` 因此被移除。
4. 该限制没有施加到 `start/goal ↔ CLIP` 查询边。它们仍按既定方案进行多档投影和
   最终 waypoint 安全门控，从而保留利用当前墙端进行尝试导航的能力；同轮廓人工
   封口 `CLIP_ATTEMPT` 和真实 `内部角点—CLIP` 轮廓拓扑也不受影响。

主要涉及 `src/planner_visualizer.cpp`、`include/far_planner/contour_graph.h`、
`src/contour_graph.cpp` 和 `src/map_handler_test.cpp`，没有新增配置参数。

验证：ROS Noetic Release 编译通过；`map_handler_test _use_synthetic_map:=true` 为
`150/150`，Graph 生命周期/连边策略为 `53/53`，局部窗口策略为 `3/3`，waypoint
投影策略为 `5/5`。重新运行指定 launch 的第 1 帧后，三个 Graph 话题都包含且仅包含
粉红 `N4—N5`、`N6—N7`、`N15—N16` 三条 `CLIP_ATTEMPT`，Marker 类型均为
`LINE_LIST(type=5)`；`N5—N15` 已不在 active、eligible 或 search edge 集合中。
start 连接仍为 `unique=20, validated=20, accepted=15, no_topology_edge=0`，与收紧
普通跨轮廓 CLIP visibility 前一致。

### 2026-09-12：更正——CLIP 当前帧参与普通 visibility

上一节中的零 halo 跨轮廓窗口限制是一次中间试验，随后根据规划语义更正并撤销；
`IsSegmentFullyInsideCurrentExecutionWindow()` 及其 visibility 拒绝分支已从生产代码
移除。最终规则是：CLIP 的特殊性只约束跨帧身份和人工封口语义，不限制本帧通过既有
方向、多边形、静态/动态碰撞及地形检查的普通 visibility 连边。

因此当前帧中：

1. 不同轮廓的 `CLIP ↔ CLIP`、`普通角点 ↔ CLIP` 仍可成为普通 `VISIBILITY`；第 1
   帧的 `N5—N15`、`N15—N2`、`N5—N14`、`N7—N8` 均恢复。
2. 同一裁剪轮廓的 OpenCV 人工封口仍单独标记为粉红 `CLIP_ATTEMPT`，即
   `N4—N5`、`N6—N7`、`N15—N16`；它们仍然随当前 CLIP 在下一帧撤销重建。
3. `start/goal ↔ CLIP` 保持原逻辑；实际 waypoint 继续接受最终碰撞和窗口门控。
4. 普通 visibility 候选恢复后仍经过扁三角形冗余简化。当前参数保持
   `flat_triangle_max_detour_ratio=1.05`、`flat_triangle_max_altitude_ratio=0.2`，因此
   `N5—N1/N20` 只是候选，不保证最终保留；三角形成立时只删除唯一最长边。
5. 上一节修复的 `clip_attempt_marker.type=LINE_LIST` 保留，没有撤销。

验证：ROS Noetic Release 编译通过，`map_handler_test` 为 `150/150`。指定 bag 第 1
帧的 `/far_debug/graph_after_update`、`/viz_current_eligible_graph`、
`/viz_current_search_graph` 均包含 `N5—N15`、`N15—N2`、`N5—N14`、`N7—N8`；三个
粉红人工封口 Marker 均为 `LINE_LIST(type=5)`。本帧扁三角形阶段报告
`triangle_pruned=27`。

### 2026-09-12：消除当前 CLIP 轮廓边的临界窗口误拒绝

指定 bag 的 `cloud_stamp=1786559071.310469` 帧中，顶部轮廓 CLIP 被栅格量化到世界
坐标 `y=5.4 m`。局部窗口在下一帧发生约毫米级位姿移动后，这些本帧刚生成的 CLIP
会略微越过 `max_y + boundary_guard + contour_grid_resolution = 5.4 m` 的数值边界。
原实现随后又在 `ValidateContourFollowEdge()` 中调用
`IsPointInsideCurrentObservationWindow()`，导致两组轮廓所有 CLIP 入射边分别以
`CONTOUR:OUTSIDE_WINDOW` 或 `CLIP_ATTEMPT:OUTSIDE_WINDOW` 被拒绝。

本次移除了这项重复的端点窗口检查。进入 `ValidateContourFollowEdge()` 的两个 CT 节点
已经属于本帧重建的同一个 contour graph，CLIP 的当前快照来源无需再由量化后的世界
坐标证明。随后检查发现普通 visibility 的最终提交、start 候选及 goal 候选中还残留
同类坐标门：候选已通过方向、多边形、静态/动态和地形检查，仍会在
`ApplyValidatedGraphEdge()` 中被静默移除，且不会留下拒绝诊断。本次一并改为调用
`IsCurrentSnapshotContourEndpoint()`，由 `is_transient_contour_endpoint +
observed_in_semantic_snapshot + is_contour_match + ctnode` 判断它是否属于当前快照，不再
重复比较临界窗口坐标。

CLIP 的跨帧安全性仍由原有瞬时生命周期保证：CLIP 不继承旧 ID、不升级为长期静态
节点，下一帧撤销并按新轮廓重建。普通 visibility 仍须通过方向、多边形、静态/动态、
地形、同方向稀疏化和扁三角形简化；waypoint 的执行期窗口及碰撞门也没有放宽。

新增回归用例显式把局部窗口原点从 `y=0` 移至 `y=-0.006 m`，确认窗口函数会拒绝
`y=5.4 m` 的量化 CLIP，但该 CLIP 的物理轮廓边仍为 `CONTOUR_FOLLOW`、人工封口仍为
`CLIP_ATTEMPT`。ROS Noetic Release 编译通过，`map_handler_test` 为 `151/151`，全部
catkin gtest 通过。重新逐帧运行问题帧后，原来六条 `OUTSIDE_WINDOW` 记录消失：两组
`内部角点—CLIP` 物理边和两条 `CLIP—CLIP` 人工封口均进入
`/viz_current_search_graph`。继续验证普通跨轮廓 visibility 后，问题场景中的
`CT3—CT15`、`CT3—CT14`、`CT0—CT15`（不同逐帧起点可能导致 CT 编号偏移，但几何
位置相同）也全部进入 search graph；未修改方向稀疏化和扁三角形冗余简化规则。

后续修改规划行为时，应在本文件“修改记录”中追加：

- 日期和修改目标；
- 修改前的问题与可复现场景；
- 关键设计决策和未采用方案；
- 涉及的主要文件、参数及默认值；
- 验证方法、测试结果和仍存在的限制。

只改注释、格式或不影响行为的机械调整可以合并记录；影响点云分类、轮廓身份、
节点生命周期、edge 阻断/恢复或地图数据源的修改必须单独记录。

### 2026-09-12：保留维持同轮廓连通所必需的 CONCAVE 节点

指定 bag 第 26 帧中，原 Graph 的 `N198=(2.20,-5.13)` 与
`N8=(5.80,-3.73)` 属于同一条当前静态轮廓，两点之间还存在
`CT10=(2.47,-3.53)`。CT10 到端点弦线的距离约为 `1.395 m`，超过 FAR
轮廓缩减所用的 `kNearDist=0.50 m`，所以不能把两端直接连接；普通 visibility
也被当前障碍多边形判为 `POLYGON_BLOCKED`。正确拓扑应是保留中间折点，形成
`端点—CT10—端点`，而不是强制增加一条跨过深凹结构的长边。

旧实现存在两个连续过滤：

1. `EnclosePolygonsCheck()` 只检查两个已经匹配历史 Graph 的端点。新出现的 CLIP
   虽是当前帧合法轮廓锚点，却不参与必要点判定。
2. 即使 CT 被标成 `is_contour_necessary`，静态 `CONCAVE` 仍会在
   `MatchContourWithNavGraph()` 和 `DynamicGraph::IsAValidNewNode()` 中被无条件过滤。
   旧必要点判定还要求 `is_ground_associate=true`，与当前局部语义管线的
   “缺少地形样本为 UNKNOWN、已知高度冲突才拒绝”规则不一致。

本次修改后，每条当前静态物理轮廓以“已匹配历史节点 + 本帧可接受的凸角/CLIP”作为
缩减锚点。对相邻锚点间的轮廓链递归计算中间 CONCAVE 到端点弦线的距离；若最大偏差
超过 `kNearDist`，只把偏差最大的合法凹点标为 `NEC`，再递归检查两侧，直到各段都可
由 FAR 圆柱约束表达。OpenCV 在窗口边界形成的人工封口不参加这一过程，它仍保持
`CLIP_ATTEMPT` 语义。缺少地形样本不会在“拓扑必要性”阶段过滤节点；后续真正创建
NavNode 时仍执行既有地形高度一致性判断。

普通 CONCAVE 仍然不进入 Graph；只有 `is_contour_necessary=true` 的 CONCAVE 例外。
该节点保留 `CONCAVE` 类型，因此 `IsConvexConnect()` 继续禁止它建立普通跨轮廓
visibility，只能承接当前同轮廓的两段拓扑关系。调试显示中，当前 CT 和
`/far_debug/graph_after_update` 的对应 NavNode 标签都会附加 `NEC`。

验证结果：ROS Noetic Release 编译通过；`map_handler_test` 为 `152/152`，catkin
单元测试为 `190/190`。新增回归用例确认没有地形匹配的深凹点也会作为必要拓扑点进入
新节点候选。隔离 ROS master 逐帧回放到相同第 26 帧后，CT10 显示
为 `CT10 CONCAVE S NEC F+`，Graph 在 `(2.47,-3.53)` 创建必要节点，
`/viz_current_search_graph` 中实际出现
`(2.20,-5.13)—(2.47,-3.53)—(5.80,-3.73)` 两段边；原来不安全的两端直连没有恢复。
继续推进到第 50 帧没有进程异常；该帧 30 个 CT 中有 6 个 CONCAVE，其中仅 4 个因
轮廓缩减需要而标为 `NEC`，说明普通凹点过滤仍然生效。
