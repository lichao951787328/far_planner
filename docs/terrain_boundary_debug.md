# 静态障碍地面恢复调试

调试输出与当前简化算法使用同一份中间结果，只观察、不二次判断，也不会改变导航输出。
它针对共享语义表中全部 `static_obstacle`，不再只针对 label 0；
`dynamic_obstacle` 和 `ignore` 只显示为排除项。

## 两个开关

```yaml
terrain_boundary_debug_opencv_enabled: true
terrain_boundary_debug_rviz_enabled: true
```

- `terrain_boundary_debug_opencv_enabled=true`：打开自动刷新的 OpenCV 九宫格窗口，
  并允许在窗口内点击栅格查询判断记录。
- `terrain_boundary_debug_rviz_enabled=true`：同时发布 RViz 的 `stages_image` 和
  `decision_cloud`，在九个面板显示机器人位置，并订阅 `/clicked_point` 查询记录。
- 两个开关都为 `false`：不构造或发布任何调试数据；地面恢复算法仍正常执行。

下面只是数值和路径设置，不是功能开关：

```yaml
terrain_boundary_debug_statistics_interval_frames: 10
terrain_boundary_debug_max_samples_per_reason: 1
terrain_boundary_debug_clicked_point_topic: /clicked_point
terrain_boundary_debug_clicked_point_radius: 0.20
terrain_boundary_debug_csv_path: ""
```

其余配置只是统计间隔、点击半径和可选 CSV 路径，不再分别控制某一项显示功能。

## 3×3 阶段图

阶段图是一张全高度投影的二维 XY 栅格，不再按 Z 层切换：

1. `terrain projection`：所有 `role=terrain` 真实体素投影出的原始白色栅格；
2. `after dilation`：使用 `terrain_boundary_closing_radius` 膨胀；
3. `after erosion`：使用相同核腐蚀，即闭运算结果；
4. `recoverable static`：所有真实 `role=static_obstacle` 体素；
5. `dynamic/ignore excluded`：明确不允许恢复的类别；
6. `newly white`：闭运算结果减去原始 terrain；
7. `supported static candidates`：闭运算后的完整地面支持区与真实静态体素的交集；
8. `terrain reference found`：半径内找到原始 terrain 参考的候选；
9. `recovered`：参考高差通过并实际输出为 terrain 的体素。

第 6 幅专门显示闭运算新填了哪些位置，只用于解释形态学效果。第 7 幅还包括同一 XY
原本已有 terrain、其他 Z 存在静态体素的情况，因此第 6 幅没有而第 7 幅有是正常的。
第 4 幅有而第 3 幅没有表示静态体素不在闭运算支持区；第 3 幅有而第 4 幅没有表示该处
没有真实静态体素；第 3、4 幅同一位置都有时第 7 幅应当出现。第 7 幅有、第 9 幅没有
则看 decision cloud 的蓝色或橙色原因。算法始终不会凭空创建体素。

## decision cloud 颜色与字段

| `reason` | RGB | 名称 | 含义 |
|---:|---|---|---|
| 0 | `(110,110,110)` 灰 | `outside_closed_terrain_support` | 该静态体素不在闭运算后的完整地面支持区 |
| 1 | `(0,128,255)` 蓝 | `no_terrain_reference` | 搜索半径内没有原始 terrain 体素 |
| 2 | `(255,165,0)` 橙 | `height_difference_too_high` | 候选与附近 terrain 的绝对 Z 高差超过阈值 |
| 3 | `(0,255,0)` 绿 | `recovered` | 已复制附近 terrain 的标签和代价 |

点云保留 `x/y/z/rgb/reason/failure_mask`，并增加与简化逻辑对应的字段：

- `original_label`：恢复前静态类别；
- `proposed`：是否由 OpenCV 提出；
- `reference_found`：是否找到原始 terrain；
- `reference_x/y/z`、`reference_label`：实际使用或用于解释失败的参考点；
- `reference_distance_xy`：候选到参考点的平面距离；
- `height_difference`：两点的绝对 Z 高差；
- `replacement_label`：成功时写入的 terrain label，失败时为无效 label。

RViz 应将 PointCloud2 的 Color Transformer 设为 `RGB8`。显示为球体时，颜色只表示
上表的决策原因，与 `/FAR_dynamic_obs_debug` 的洋红色动态障碍显示没有关系。

## 点击与 CSV

OpenCV 任意面板点击某个 XY，会打印该列全部可恢复静态体素的原始 label、参考点、
高差、阈值和结果。RViz 的 Publish Point 会在三维半径内打印最近记录。

需要离线统计时，把 `terrain_boundary_debug_csv_path` 设为绝对路径。CSV 逐帧追加
同样的简化字段；诊断结束后应恢复为空字符串，避免文件持续增长。

## 确认导航链路使用同一结果

恢复器先替换局部 `voxels` 快照，之后才依次生成：

- `/local_3d_semantic_voxel_map/voxel_cloud`：FAR 当前轮廓和局部语义层；
- `/local_3d_semantic_voxel_map/traversability_cost_cloud`：localPlanner 静态代价输入；
- `/local_3d_semantic_voxel_map/global_semantic_admission_grid`：SemanticOctomap 全局建图输入。

三路都来自同一个过滤后快照。节点内部保存的时序融合原图仍保留原观测，每次新输入帧
重新生成过滤快照；这避免调试/形态学结果反向污染语义融合，同时保证每次下游看到的都是
当帧过滤结果。
