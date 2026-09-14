# FAR Planner 与 Local 3D Semantic Voxel Map 接口改造方案

## 1. 分支与基线

本分支名为：

```text
far-planner-interfeace-semantic
```

仓库根目录首先恢复为 MichaelFYang/FAR Planner `melodic-noetic` 分支中
`src/far_planner` 的原始 ROS 包，基线提交为：

```text
MichaelFYang/far_planner: 2799b6964c141cacd1c32a14b19bc7abffbe0e52
```

随后只在该基线上增加 Local 3D Semantic Voxel Map 接口和必要的 FAR
输入修正，不继承原仓库其他实验分支中的语义图、bag 测试和拓扑图改动。

## 2. 最终数据链路

本方案不再运行 `terrain_analysis`、`terrain_analysis_ext`，也不使用
`/terrain_map_ext`：

```text
/local_3d_semantic_voxel_map/voxel_cloud
                    |
                    v
       semantic_voxel_to_far_terrain
                    |
                    v
          /far_binary/terrain_map
              |              |
              |              +--> FAR /terrain_local_cloud
              |                   局部 TerrainPlanner 和历史轨迹边检查
              |
              +------------------> FAR /terrain_cloud
                                   持久 free/obstacle grid、轮廓和 NavGraph

/livox/lidar (CustomMsg) --+
                           +--> far_scan_input_adapter
/raw_scan (PointCloud2) ---+          |               |
                                      |               +--> /far_scan/registered_scan_origin
                                      +--> /far_scan/registered_scan
                                                     |
                                                     +--> FAR /scan_cloud
                                                          当前射线和动态障碍清除

/state_estimation ---------------> FAR /odom_world
```

只启动 semantic bridge 和 FAR（需要外部已经提供 registered scan 及
同 stamp origin）：

```bash
roslaunch far_planner semantic_interface.launch
```

常用覆盖参数：

```bash
roslaunch far_planner semantic_interface.launch \
  voxel_cloud_topic:=/local_3d_semantic_voxel_map/voxel_cloud \
  registered_scan_topic:=/registered_scan \
  registered_scan_origin_topic:=/far_scan/registered_scan_origin \
  odom_topic:=/state_estimation \
  world_frame:=map
```

如果上游只有原始 `PointCloud2` 或 Livox `CustomMsg`，还必须启动
`scan_input_adapter.launch`。本分支的 0914 数据集整链路启动文件已经
同时包含 Local 3D map、scan adapter、bridge 和 FAR。

如果系统没有安装 `graph_decoder`，使用：

```bash
roslaunch far_planner semantic_interface.launch graph_decoder:=false
```

### 2.1 原始点云双格式适配

新增 `far_scan_input_adapter`，同一个输入 topic 可接受：

| 输入格式 | 时间处理 | 强度处理 |
|---|---|---|
| `sensor_msgs/PointCloud2` | 使用 `header.stamp` 查 TF | 保留 intensity；缺少时填 0 |
| `livox_ros_driver2/CustomMsg` | `timebase + offset_time`，默认按 2 ms 分桶去畸变 | reflectivity 写入 intensity |

适配器用 `topic_tools/ShapeShifter` 判断线上 datatype 和 MD5，因此 FAR
包不需要在编译时依赖 `livox_ros_driver2`。当前内置解码器严格对应
`livox_ros_driver2/CustomMsg` 的 MD5
`e4d6829bdfe657cb6c21a746c86b21a6`；其他 MD5 会被明确拒绝，不会
按错误布局解码。

Livox 每个点使用自己的采集时刻进行配准，并用整包中点时刻作为
输出帧的参考 stamp。`PointCloud2` 没有通用的逐点时间字段契约，
因此按整帧 `header.stamp` 变换。两者最终都输出 `map` 坐标下的
`PointXYZI` PointCloud2。

### 2.2 TF、外参和 scan origin

当前数据集已确认 `wuba_base <- livox_frame` 是单位变换：

```yaml
extrinsic_source: params
extrinsic_xyz: [0.0, 0.0, 0.0]
extrinsic_rpy: [0.0, 0.0, 0.0]
```

所以 Livox 点的配准链为：

```text
map <- wuba_base(scan acquisition time) <- livox_frame(identity)
```

不使用“最新 TF”。TF 暂时还没有播放到所需时刻时，帧会进入有界
队列；如果 TF 时间线已经越过该帧，或 wall-time 等待超时，则明确丢帧。
用 wall time 而非 ROS time 计超时，避免 bag 的 `/clock` 暂停导致永久阻塞。

除点云外，适配器还发布同 stamp 的
`/far_scan/registered_scan_origin`。FAR 按精确 stamp 配对点云与光束原点，
再执行动态障碍射线清除。这避免在回调延迟时把“当前里程计位置”
误当成旧扫描的发射位置。

0914 bag 的整链路启动方式为：

```bash
roslaunch far_planner 0914_semantic_far_navigation.launch rviz:=false
```

## 3. Bridge 的输入与输出契约

新增节点：

```text
semantic_voxel_to_far_terrain
```

输入点云必须至少包含下列字段和类型：

| 字段 | 类型 | 用途 |
|---|---|---|
| `x/y/z` | `float32` | voxel 空间位置 |
| `label` | `uint32` | 查询共享 schema 的导航角色 |
| `traversability` | `float32` | 语义与几何融合后的最终代价 |
| `observations` | `uint32` | 排除尚未达到最低观测次数的 voxel |

bridge 输出仍以二值 `intensity` 表达 FAR terrain，同时在同一条
`PointCloud2` 中携带一个保护字段：

| 分类 | `intensity` | `static_obstacle` (`uint8`) |
|---|---:|---:|
| free | 0.0 | 0 |
| 普通/动态可清除 obstacle | 1.0 | 0 |
| schema `role=static_obstacle`，且未关闭 FAR 保护 | 1.0 | 1 |
| `geometric_obstacle`（`far_static_protection=false`） | 1.0 | 0 |

`intensity` 仍然严格为 0/1，未修改的原始 FAR 会忽略额外字段并继续按二值
terrain 使用。语义版 FAR 读取 `static_obstacle`，让它只影响动态障碍清除，
不会改变轮廓、占据图和 TerrainPlanner 对 obstacle 的正常处理。保护标记与
terrain 共用同一个 header、同一条消息，因此不会新增跨 topic 时间同步问题。

调试输出：

```text
/semantic_voxel_to_far_terrain/protected_static_debug
/FAR_protected_static_debug
```

前者是 bridge 产生的所有受保护点，后者是 FAR 完成 TF、独立降采样和范围
裁剪后在当前 terrain 帧实际使用的保护点。

原始上游的终点选择按钮由独立 `goalpoint_rviz_plugin` 包提供，只复制
`src/far_planner` 时会丢失该插件。本分支已将兼容类名
`rviz/GoalPointTool` 直接集成到 `far_planner` 包。它默认订阅
`/fusion_localization` 获取高度，向 `/goal_point` 发布
`geometry_msgs/PointStamped`，目标 frame 为 `map`。这些话题、frame 以及
是否发布原始 FAR `/joy` 启动指令均可在 RViz Tool Properties 中配置；
默认 RViz 配置已启用该工具，快捷键为 `W`。

输出使用输入消息原始 `header.stamp`，不会把定时器重发的旧快照伪装成新
数据。bridge 本身允许重复 stamp；是否已经消费由 FAR 的三个输入回调分别
判断。

bridge 使用消息采集时刻的 TF 转到 `world_frame`。推荐 Local 3D map 本身就
输出 `map` 坐标，以减少 TF 失败和时间偏差。

## 4. 语义二值规则

共享契约来自：

```text
config/five_class_semantic_schema.yaml
```

判断优先级如下：

1. `role=static_obstacle`：输出 obstacle；默认设置 `static_obstacle=1`；
2. `far_static_protection=false`：即使导航 role 是 `static_obstacle`，也保持
   `static_obstacle=0`。当前用于语义不确定的 `geometric_obstacle`，使其中可能
   存在的行人、车辆等仍可被 registered scan 动态清除；
3. `role=dynamic_obstacle`：输出 obstacle，但保持 `static_obstacle=0`，允许
   registered scan 在目标消失后清除；
4. 其他角色：`traversability >= obstacle_threshold` 输出 obstacle，但不自动
   获得静态保护；
5. 其余有效点输出 free；
6. 代价不是有限数时，默认按 obstacle 处理；
7. 观测次数小于 `minimum_observations` 的点不输出。

当前参数：

```yaml
obstacle_threshold: 0.75
minimum_observations: 1
missing_cost_is_obstacle: true
```

### label 0

label 0 表示已经产生有效空间点，但没有可靠语义类别，schema 中设置为
`geometry_only=true`。Local 3D map 会把它规范化为无有效语义 label，同时
保留几何代价。bridge 因此不依赖输出中是否仍然存在 `label==0`，而使用最终
`traversability` 完成二值判断。

### grass

当前 schema 中 grass 的真实导航配置为：

```yaml
role: static_obstacle
semantic_cost: 1
```

因此当前实现把 grass 当作硬障碍。本分支同时修正了原来与 `role` 冲突的
`meaning` 文字；算法判断仍然只依赖结构化的 `role` 和代价字段。

## 5. FAR 的 terrain 预处理修改

原始 FAR 对 terrain 的处理顺序为：

```text
整个 PointXYZI 点云降采样 -> 按 intensity 分类
```

PCL `VoxelGrid<PointXYZI>` 会平均 intensity。例如同一 leaf 内一个障碍点和
五个自由点会得到 `1/6=0.167`，在 `kFreeZ=0.2` 时错误变成 free。

修改后的顺序为：

```text
移除非有限点
    -> 按 kFreeZ 拆成 free/obstacle，同时提取 static_obstacle 保护点
    -> 三类点分别转换到 world frame
    -> free、obstacle 和 static protection 分别 VoxelGrid
    -> 删除与 obstacle leaf 重合的 free centroid
```

该逻辑同时用于：

```text
FAR /terrain_cloud
FAR /terrain_local_cloud
```

`/scan_cloud` 继续使用原始 `PrcocessCloud()`，不能按 intensity 拆分，因为
registered scan 的 intensity 不承担 FAR terrain 分类语义。

二值配置将：

```yaml
Util/terrain_free_Z: 0.5
```

同一个三维 FAR leaf 中同时出现 free 和 obstacle 时固定采用 obstacle，避免
结果随点密度变化。

静态保护点必须单独降采样，不能先和普通 obstacle 混合后平均语义标记；否则
同一 leaf 中的点数比例会决定是否受保护，重新引入与原 intensity 类别混合
相同的问题。

## 6. FAR 内部严格时间戳去重

FAR 分别保存：

```text
last_terrain_stamp_
last_terrain_local_stamp_
last_scan_stamp_
```

各回调只接受：

```text
stamp > 对应输入最后一次成功使用的 stamp
```

不能使用一个全局 stamp，因为同一采集时刻的二值地图需要同时被
`terrain_cloud` 和 `terrain_local_cloud` 各消费一次，registered scan 也可能
具有相同时间戳。

只有点云转换和处理成功后才推进 last stamp。下列情况不会消耗 stamp：

- stamp 为零；
- odometry 尚未初始化；
- TF 失败；
- FAR 主地图更新处于暂停状态；
- 消息时间小于或等于已成功消费时间。

执行 `ResetEnvironmentAndGraph()` 时会同时清空三个时间戳，允许 rosbag 从
较早时间重新播放。

## 7. 动态障碍

必须设置：

```yaml
is_static_env: false
```

`/registered_scan` 必须是真正的当前帧原始几何扫描，并且已经配准到
`world_frame`。它用于 FAR 的射线关系和历史障碍清除，不是主障碍地图。

新障碍必须最终进入：

```text
/far_binary/terrain_map -> FAR /terrain_cloud
```

需要快速阻断历史轨迹连接时，同一消息也会进入
`/terrain_local_cloud`。只把新障碍发送到 `/scan_cloud` 不会建立正常轮廓。

当配置：

```yaml
protect_static_obstacles: true
```

FAR 会在动态清除候选的数量阈值判断之前，删除与当前
`static_obstacle=1` 点重合的候选。普通动态候选通过阈值并膨胀后会再次执行
保护过滤，防止相邻动态点的膨胀区域反过来删除静态语义 voxel。当前帧的
静态语义还会撤销同 voxel 中历史 `stack_dyobs_cloud_` 的短期抑制，使以前被
误判为“已消失”的井盖能够重新进入 FAR obstacle map。

这一保护只改变“raw scan 射线能否把历史障碍解释为已消失”，不改变：

- 静态点作为 obstacle 进入主地图和局部 TerrainPlanner；
- obstacle-wins；
- `dynamic_obstacle` 和纯代价障碍的正常清除；
- 上游 Local 3D map 对语义类别本身的更新或撤销。

## 8. 已知危险点

### 8.1 去掉 terrain_map_ext 后的范围损失

FAR 只能使用 Local 3D voxel map 当前提供的空间范围。必须满足：

```text
FAR terrain_range <= Local 3D map 有效范围
```

否则远处障碍不会提前形成轮廓，拓扑图只能随机器人移动逐步更新。若实际
测试证明覆盖不足，应扩大 Local 3D map，或以后重新增加一个定义清晰的远场
补充通道，而不是直接恢复旧 ext 的混合语义。

### 8.2 bridge 不生成未观测自由空间

bridge 只转换 Local 3D map 中实际存在的 voxel，不会为没有观测的区域合成
free 点。传感器遮挡、视场外和地图裁剪区域仍然是未知，不得当作自由空间。

### 8.3 3D 表面与机器人身体空间

当前 bridge 对每个已有 voxel 做二值分类，不重新拟合地面，也不凭空完成
严格的机器人扫掠体碰撞检查。它依赖 Local 3D map 的几何 traversability
正确地把墙、台阶、悬空结构等标成高代价。桌沿、横杆或多层地面若在上游
代价中被错误标低，仍可能进入 free cloud，必须用真实数据检查。

### 8.4 障碍优先可能使窄通道更保守

同一 FAR voxel 内采用 obstacle wins。`voxel_dim` 太大时，边界附近的 free
会被删除，窄通道可能变窄。应让 `voxel_dim` 接近 Local 3D map 分辨率，并用
实际机器人尺寸验证，不要仅靠减小 `kFreeZ` 调整。

### 8.5 重复膨胀

bridge 当前不做 footprint 膨胀，保留 FAR 原有安全距离和 TerrainPlanner
障碍膨胀。如果以后在 bridge 中增加配置空间膨胀，必须同步重新核算 FAR 的
`robot_dim`、`nav_clear_dist` 和 `obs_inflate_size`，否则窄通道会被重复封闭。

### 8.6 时间倒退和乱序消息

严格 `stamp > last_stamp` 会同时丢弃重复消息和乱序旧消息。rosbag seek、
循环播放或 `/clock` 回退后，必须重置 FAR 或重启节点。内容发生变化但 stamp
不变的消息也会被视为同一帧，因此所有真正的新数据必须带新采集时间。

### 8.7 TF 时间一致性

bridge 使用 voxel cloud 的消息时间查询 TF。TF 缓存中缺少该时刻变换时整帧
会被丢弃。适配器同样只使用采集时刻 TF；输出在 `map` 坐标系。
FAR 对绕过适配器的非 world-frame scan/terrain 也已改为使用消息 stamp，
不再使用 `ros::Time(0)`。

### 8.8 齐平井盖被几何动态清除的实测现象

使用 0914 bag 完整回放检查得到：

- `/grids_points` 共 1453 帧，`/livox/lidar` 共 1906 帧；
- terrain 与最近 Livox 扫描中心通常相差约 50 ms；按 bag 到达顺序，FAR
  处理 terrain 时通常持有比它新约 50 ms 的 scan，少数帧约 150 ms；
- 在井盖出现区间抽样 20 秒，11 个 terrain 更新中有 label 4 点进入
  `/FAR_dynamic_obs_debug`，合计匹配 140 个井盖点；
- 当前 `Util/dyosb_update_thred=2`，上述帧通过了 `size > threshold`，因此确实
  执行了删图，不只是调试显示。

代表帧：

```text
terrain stamp                 1787148583.437001
FAR 当时使用的 scan stamp    1787148583.487010
scan 比 terrain 新            50.0 ms
井盖点到最近 raw scan 回波    0.032 m
结果                          仍进入 dynamic clear
```

因此“只有明显的 TF/时间错位，使井盖落在回波端点之前才会误清除”不成立。
即使 raw scan 已经在井盖 3.2 cm 内产生回波，原始 FAR 逻辑仍可能清除它。

### 8.9 井盖误清除的代码原因

原始 `ScanHandler::SetCurrentScanCloud()` 先复制 scan，并用当前 FAR free cloud
删除重叠回波。重叠判断采用 `1.2 * voxel_dim`；当前 `voxel_dim=0.10 m`，即
约 0.12 m。井盖和路面基本齐平，井盖回波很容易与周围 free 地面落入同一个
重叠 leaf，因此不能设置用于终止射线的 `SCAN_BIT`。

但是这个过滤只作用于“哪些回波是阻挡终点”。随后发射清除射线时，源码又
遍历未过滤的完整 `scanCloudIn`。于是同一个井盖附近回波可能同时满足：

```text
与 free 重叠 -> 不再是 SCAN_BIT 阻挡终点
仍在原 scan  -> 继续向端点生成 RAY_BIT
```

邻近或更远的地面回波也可能让射线穿过井盖 obstacle voxel。历史 obstacle
只要位于 `RAY_BIT` voxel 就进入动态清除候选，因此平面语义障碍不具备墙体
那样稳定的几何遮挡特征。50～150 ms 的跨流异步和不同 voxel 原点会放大
误判，但不是唯一原因。

本分支通过同帧 `static_obstacle` 字段解决这个接口语义冲突：这类点仍参与
建图和碰撞，但在阈值前及动态候选膨胀后都从几何清除集合中排除。

### 8.10 静态保护的风险与边界

静态保护相信上游 schema 和当前 Local 3D 语义结果。若行人、车辆或可移动
物体被错误分类成 `static_obstacle`，它不会被 FAR 的 raw-scan 射线清除，
只能等待上游 Local 3D map 更正/撤销该语义。因此：

- `dynamic_obstacle` 不得配置成 static role；
- label 5 `geometric_obstacle` 虽为兼容 Local3D 的导航角色而保留
  `role=static_obstacle`，但已明确设置 `far_static_protection=false`。它在 FAR 中
  仍是 `intensity=1` 的障碍，同时可以按 registered scan 的几何证据清除；
- `far_static_protection` 是 FAR 接口策略，不改变 Local3D 对 `role`、
  `semantic_cost` 和 `global_map` 的处理；不要把语义不确定类别错误地设为 true；
- 保护只覆盖当前 terrain 消息仍携带的静态点。点在进入 bridge 前已被 Local
  3D map 删除或改类时，FAR 无法知道它曾经是静态语义；
- 关闭 `protect_static_obstacles` 或使用没有该字段的旧 terrain producer 时，
  FAR 回退为原始“所有障碍都可被几何射线清除”的行为。

registered scan 的量程裁剪、遮挡或配准误差仍可能误清除未受保护的普通障碍。
动态清除阈值和 decay 参数必须用 bag 逐帧验证；原始 scan 不能使用累计点云
替代，否则射线的当前可见性含义会失效。

### 8.11 不再表达软代价

进入 FAR 后只剩 free/obstacle。flat ground、rough ground 等可通行类别之间
不再有路径偏好差异，GraphPlanner 仍使用欧氏距离。这是本分支明确接受的
设计边界；若以后需要“可走但尽量避开”的行为，需要独立的连续代价规划层，
不能通过修改 FAR 的二值 intensity 阈值隐式实现。

### 8.12 CustomMsg ABI 和数据质量

CustomMsg 的线上字段布局受 MD5 保护，但升级 Livox driver 后如果消息定义
变化，必须同步更新解码器和测试。`point_num` 与序列化 points 数量不一致时
只使用两者较小值。NaN/Inf 和原点零值会被删除，因此输出点数不一定
等于包内 `point_num`。

### 8.13 PointCloud2 的原点契约

如果 PointCloud2 已经在 `world_frame`，单凭点云不能推出扫描发射原点，
必须正确设置 `pointcloud_origin_frame`（当前为 `wuba_base`），并保证该帧时刻
TF 可用。如果 PointCloud2 仍在传感器或车体坐标，其 `header.frame_id`
就是变换和光束原点的依据。普通 PointCloud2 当前不做逐点去畸变；
如需要，上游必须提供可明确解释的逐点时间字段契约。

## 9. 验证清单

1. bridge 的 intensity 仍只有 0 和 1，并包含 `uint8 static_obstacle`；
2. label 0/无有效语义点按照几何 traversability 分类；
3. grass、manhole_cover 进入 obstacle 且保护字段为 1；geometric_obstacle 进入
   obstacle，但保护字段必须为 0；
4. 一个 obstacle 与多个 free 落入同一 FAR leaf 时 obstacle 不消失；
5. 同一 stamp 重发时，main/local/scan 三个回调各自只成功消费一次；
6. 同一 stamp 的 main terrain 和 local terrain 不会互相误去重；
7. 新障碍先阻断局部轨迹连接，随后更新主轮廓；
8. `dynamic_obstacle` 移走后，registered scan 能提供清除证据；
9. Local 3D map 边界内的实际覆盖能够满足 `terrain_range`；
10. rosbag 时间回退后执行 FAR reset，三个输入能重新开始消费；
11. `PointCloud2` 有/无 intensity 都能输出标准 `PointXYZI`；
12. Livox 点使用 `timebase + offset_time` 的采集时刻配准；
13. 输出 scan 和 scan origin 的 frame/stamp 完全一致；
14. bag 开头早于首个 TF 的帧被明确丢弃，不会卡住后续队列；
15. 整链路能稳定发布 registered scan/origin，并初始化 FAR V-Graph；
16. label 4 与动态射线相交时不会进入最终动态删除点云；
17. 其他动态候选膨胀后也不会覆盖 `/FAR_protected_static_debug` 的 voxel；
18. 旧 terrain 没有 `static_obstacle` 字段时仍可运行，并明确提示保护不可用。

## 10. 本分支实测结果

在 ROS Noetic 容器中已完成：

- `catkin_make --pkg far_planner -j2`：通过；
- Livox 线上解码器 3 个 gtest：全部通过；
- RViz 实例已成功加载 `rviz/GoalPointTool`；pluginlib 可发现插件，并实测
  建立 `/goal_point` 发布器、`/fusion_localization` 订阅器和 `/joy` 发布器；
- 合成 semantic terrain 接口测试：flat ground 输出 `(0,0)`，manhole_cover
  输出 `(1,1)`，geometric_obstacle、dynamic_obstacle 与纯代价障碍输出
  `(1,0)`；二元组依次为 `(intensity, static_obstacle)`；
- 合成 PointCloud2 运行时测试：有/无 intensity 两帧都通过，无字段时
  输出 intensity 为 0，已有值 42.5 被保留，scan/origin 精确同 stamp；
- 0914 真实 bag 的 Livox CustomMsg：输出约 10 Hz，实测单帧约 8 万点；
- 整链路测试：实测收到 80064 点 registered scan、7098 点二值 terrain、
  精确配对的 scan/origin 和 `/robot_vgraph`，V-Graph 持续更新。
- 修改静态保护后的 0914 bag 2 倍速完整回放：抽查到的 1768 条 bridge
  terrain 消息全部包含正确类型的 `uint8 static_obstacle` 字段；250 个 FAR
  更新中有 248 帧仍发布非空动态点，证明保护没有把动态清除整体关闭；
- 井盖时间段 35 秒定点回放：80 个 FAR 调试周期中 74 帧仍有普通动态删除
  点；按 FAR/PCL 实际 float32 的 0.12 m leaf 分箱，同 stamp 的 protected 与
  最终 dynamic 删除集合重叠为 0。监视器捕获的 3 次表面重叠均伴随
  0.38～0.40 秒 debug stamp 差，属于订阅丢帧后跨周期比较，未计入结果。

bag 最开头 14 帧 Livox 扫描的采集时间早于记录中第一个
`map -> wuba_base` TF，它们无法在不外推位姿的前提下配准，因此被按设计
明确丢弃；首个可配准帧之后的输出和 FAR 更新正常。
