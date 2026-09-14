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

bridge 输出普通 `PointXYZI`：

```text
free      intensity = 0.0
obstacle  intensity = 1.0
```

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

1. `role=static_obstacle` 或 `role=dynamic_obstacle`：直接输出 obstacle；
2. 其他角色：`traversability >= obstacle_threshold` 输出 obstacle；
3. 其余有效点输出 free；
4. 代价不是有限数时，默认按 obstacle 处理；
5. 观测次数小于 `minimum_observations` 的点不输出。

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
    -> 按 kFreeZ 拆成 free/obstacle
    -> 分别转换到 world frame
    -> free 和 obstacle 分别 VoxelGrid
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

### 8.8 动态障碍清除的误判

registered scan 的量程裁剪、遮挡或配准误差都可能被解释为历史障碍消失。
动态清除阈值和 decay 参数必须用 bag 逐帧验证；原始 scan 不能使用累计点云
替代，否则射线的当前可见性含义会失效。

### 8.9 不再表达软代价

进入 FAR 后只剩 free/obstacle。flat ground、rough ground 等可通行类别之间
不再有路径偏好差异，GraphPlanner 仍使用欧氏距离。这是本分支明确接受的
设计边界；若以后需要“可走但尽量避开”的行为，需要独立的连续代价规划层，
不能通过修改 FAR 的二值 intensity 阈值隐式实现。

### 8.10 CustomMsg ABI 和数据质量

CustomMsg 的线上字段布局受 MD5 保护，但升级 Livox driver 后如果消息定义
变化，必须同步更新解码器和测试。`point_num` 与序列化 points 数量不一致时
只使用两者较小值。NaN/Inf 和原点零值会被删除，因此输出点数不一定
等于包内 `point_num`。

### 8.11 PointCloud2 的原点契约

如果 PointCloud2 已经在 `world_frame`，单凭点云不能推出扫描发射原点，
必须正确设置 `pointcloud_origin_frame`（当前为 `wuba_base`），并保证该帧时刻
TF 可用。如果 PointCloud2 仍在传感器或车体坐标，其 `header.frame_id`
就是变换和光束原点的依据。普通 PointCloud2 当前不做逐点去畸变；
如需要，上游必须提供可明确解释的逐点时间字段契约。

## 9. 验证清单

1. bridge 输出只有 intensity 0 和 1；
2. label 0/无有效语义点按照几何 traversability 分类；
3. grass 始终进入 obstacle；
4. 一个 obstacle 与多个 free 落入同一 FAR leaf 时 obstacle 不消失；
5. 同一 stamp 重发时，main/local/scan 三个回调各自只成功消费一次；
6. 同一 stamp 的 main terrain 和 local terrain 不会互相误去重；
7. 新障碍先阻断局部轨迹连接，随后更新主轮廓；
8. 障碍移走后，registered scan 能提供清除证据；
9. Local 3D map 边界内的实际覆盖能够满足 `terrain_range`；
10. rosbag 时间回退后执行 FAR reset，三个输入能重新开始消费。
11. `PointCloud2` 有/无 intensity 都能输出标准 `PointXYZI`；
12. Livox 点使用 `timebase + offset_time` 的采集时刻配准；
13. 输出 scan 和 scan origin 的 frame/stamp 完全一致；
14. bag 开头早于首个 TF 的帧被明确丢弃，不会卡住后续队列；
15. 整链路能稳定发布 registered scan/origin，并初始化 FAR V-Graph。

## 10. 本分支实测结果

在 ROS Noetic 容器中已完成：

- `catkin_make --pkg far_planner -j2`：通过；
- Livox 线上解码器 3 个 gtest：全部通过；
- 合成 PointCloud2 运行时测试：有/无 intensity 两帧都通过，无字段时
  输出 intensity 为 0，已有值 42.5 被保留，scan/origin 精确同 stamp；
- 0914 真实 bag 的 Livox CustomMsg：输出约 10 Hz，实测单帧约 8 万点；
- 整链路测试：实测收到 80064 点 registered scan、7098 点二值 terrain、
  精确配对的 scan/origin 和 `/robot_vgraph`，V-Graph 持续更新。

bag 最开头 14 帧 Livox 扫描的采集时间早于记录中第一个
`map -> wuba_base` TF，它们无法在不外推位姿的前提下配准，因此被按设计
明确丢弃；首个可配准帧之后的输出和 FAR 更新正常。
