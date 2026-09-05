# 统一语义类别表配置说明

五分类 bag 和 live 导航都由 launch 把一个 YAML 文件加载到全局参数
`/semantic_schema`。默认文件是
`config/five_class_semantic_schema.yaml`。局部 voxel、SemanticOctomap 和 FAR
都从这一个参数快照生成自己的类别角色。对于这两条 five-class 启动链，该文件是
类别定义的唯一基准；`five_class_local_voxel.yaml` 和 `five_class_bag.yaml` 只保留算法、
阈值、范围、话题及调试参数，不再保存 label/颜色/角色/语义代价副本。

## 启动和切换配置

使用默认配置：

```bash
roslaunch far_planner five_class_bag_navigation.launch \
  bag:=/path/to/input.bag
```

临时使用另一份类别表，不修改默认文件：

```bash
roslaunch far_planner five_class_bag_navigation.launch \
  bag:=/path/to/input.bag \
  semanticSchema:=/absolute/path/to/my_semantic_schema.yaml
```

live 模式使用相同参数：

```bash
roslaunch far_planner five_class_live_navigation.launch \
  semanticSchema:=/absolute/path/to/my_semantic_schema.yaml
```

这些参数只在节点启动时读取。修改 YAML 后必须重新启动整条导航链路，不能只用
`rosparam set` 热切换。重新启动也能避免旧 SemanticOctomap 中按旧角色保存的障碍
继续残留。

## 顶层参数

| 参数 | 类型/范围 | 含义 |
|---|---|---|
| `version` | 整数，当前为 `1` | 配置格式版本。解析器只接受已支持的版本。 |
| `navigation/derive_runtime_roles` | 布尔值 | five-class 配置必须为 `true`，各节点从统一类别表自动派生角色。通用节点仍保留无共享 schema 时的兼容代码，但 five-class 专用 YAML 已删除重复回退表。 |
| `input/label_field` | 非空字符串 | 输入 `PointCloud2` 中语义标签字段名。 |
| `input/traversability_field` | 非空字符串 | 输入点云中原始几何可通行代价字段名。 |
| `defaults/initial_floor_label` | 非负整数 | SemanticOctomap 初始化地面类别，必须是允许进入全局图的 `terrain`。 |

## `unknown` 参数

这里的 unknown 是无效、未配置或 `UINT32_MAX` 标签，不是有真实点的 label 0。

| 参数 | 类型/可选值 | 含义 |
|---|---|---|
| `unknown/policy` | `exclude`、`map_to_fallback` | 丢弃无效标签，或将其映射到一个已配置类别。导航建议使用 `exclude`。 |
| `unknown/fallback_label` | 非负整数 | `map_to_fallback` 时使用的目标类别。`exclude` 时不会使用。 |
| `unknown/name` | 字符串 | 日志和图例中的名称。 |
| `unknown/meaning` | 字符串 | 给维护者阅读的说明，不参与算法。 |
| `unknown/rgb` | `[R,G,B]`，每项 0–255 | 无效类别显示颜色，不能与已配置类别颜色重复。 |

## `traversability` 参数

| 参数 | 类型/范围 | 含义 |
|---|---|---|
| `traversability/override_semantics` | 布尔值 | 是否在代价过高时把全局语义统一改写成障碍。`false` 可避免局部瞬时几何噪声污染持久地图。 |
| `traversability/obstacle_threshold` | 浮点数 `[0,1]` | 上述全局语义覆盖的高代价阈值。它不是 FAR 局部瞬时障碍阈值。 |
| `traversability/obstacle_label` | 非负整数 | 覆盖时写入的类别，必须是 `role=static_obstacle` 且 `global_map=true`。 |

## 每个 `classes` 条目的参数

| 参数 | 类型/可选值 | 含义 |
|---|---|---|
| `label` | 唯一非负整数 | 输入点云中的类别编号。 |
| `name` | 非空字符串 | 类别稳定名称，用于日志和图例。 |
| `meaning` | 字符串 | 类别的人类可读说明，不参与算法。 |
| `rgb` | 唯一 `[R,G,B]` | SemanticOctomap 保存颜色，FAR 也用该颜色识别全局类别。 |
| `role` | `terrain` | 可通行语义。几何代价过高时仍会成为局部瞬时障碍。 |
| `role` | `static_obstacle` | 持久静态障碍，可进入 FAR 静态轮廓和全局地图。 |
| `role` | `dynamic_obstacle` | 动态障碍，只影响最新局部快照，`global_map` 必须为 `false`。 |
| `role` | `ignore` | 不参与导航，`global_map` 必须为 `false`。 |
| `semantic_cost` | 浮点数 `[0,1]` | 类别先验代价。当前局部 voxel 使用 `maximum` 融合，因此最终代价不会低于 measured traversability。 |
| `global_map` | 布尔值 | 是否允许类别进入持久 SemanticOctomap。 |

## 自动派生关系

| 类别表条件 | 自动提供给下游的内容 |
|---|---|
| `role=terrain` | FAR terrain label、FAR 全局 terrain 颜色组、旧障碍撤销的自由地形标签，以及 OpenCV 白色地面栅格来源 |
| `role=static_obstacle` | FAR static label、FAR 全局障碍颜色组、旧障碍撤销跟踪标签，以及 OpenCV 闭运算地面支持区内允许恢复的真实体素 |
| `role=dynamic_obstacle` | FAR dynamic label/颜色组、局部和全局动态类别排除集合；不会进入 OpenCV 恢复候选 |
| `role=ignore` | 不参与导航，也不会进入 OpenCV 恢复候选 |
| `semantic_cost` | 局部 voxel 语义代价和最终 traversability 的语义下限 |
| `rgb` | 局部可视化颜色、SemanticOctomap 颜色以及 FAR 全局类别匹配键 |

因此修改某个类别时，只改共享 schema 的对应 `classes` 条目。以下重复参数已经从
five-class 专用下游 YAML 删除，不能再在那里维护第二份：

- 局部 voxel 的 `semantic_classes`、`semantic_field`、`cost_field`；
- 地面恢复的 `terrain_boundary_*_labels`；
- 障碍撤销的 terrain/static/dynamic label 集合；
- FAR 的 `local_voxel_*_labels` 和 obstacle/terrain/dynamic groups。

`ssmi_revocation_ambiguous_obstacle_labels` 仍保留在局部 voxel 配置中，因为它不是类别
角色，而是“哪些已配置静态类别需要抗语义抖动”的专项算法策略。闭运算半径、高差、
置信度、代价阈值、地图范围和调试参数同理，仍由各自算法 YAML 管理。

FAR 当前的局部障碍阈值为 `0.55`。如果一个 `terrain` 类别希望正常通行，
`semantic_cost` 通常应小于 `0.55`。即使语义代价较低，只要输入的 measured
traversability 达到 `0.55`，该体素仍会进入瞬时障碍层。

## label 0 与真正 unknown 的区别

label 0 已经有真实的 3D 体素、语义观测和 measured traversability，只是语义含义
不明确。因此建议保持：

```yaml
role: static_obstacle
semantic_cost: 1.00
```

恢复算法会处理所有真实存在的 `static_obstacle` 体素，包括 label 0 和 manhole。
闭运算产生但没有真实点的位置不会创建新体素。位于闭运算后完整地面支持区内、与
半径内最近原始 `terrain` 体素高差不超过阈值的静态体素，才复制该参考体素的 label、
置信度和代价；否则仍保持原静态障碍。完整支持区既包含闭运算新增的小孔/窄缝，也
包含同一 XY 原本已有 terrain、其他 Z 又出现静态噪点的位置。

## 案例一：grass 和 rough_ground 都可通行

这是默认配置。用不同代价表达路径偏好：平地优先，其次草地，最后粗糙地面。

```yaml
- label: 2
  name: grass
  meaning: traversable grass with a larger preference cost than flat ground
  rgb: [0, 128, 0]
  role: terrain
  semantic_cost: 0.20
  global_map: true

- label: 3
  name: rough_ground
  meaning: traversable rough ground; allowed but less preferred than grass
  rgb: [160, 82, 45]
  role: terrain
  semantic_cost: 0.40
  global_map: true
```

结果：两者进入可通行集合并构成恢复所用白色栅格；但 rough_ground 的规划代价更高。

## 案例二：grass 不可通行，rough_ground 可通行

只需要修改 grass 条目，不需要再改 FAR、撤销器和 OpenCV 的 label 列表：

```yaml
- label: 2
  name: grass
  meaning: non-traversable grass
  rgb: [0, 128, 0]
  role: static_obstacle
  semantic_cost: 1.00
  global_map: true
```

rough_ground 条目保持案例一。结果：grass 自动进入所有静态障碍集合，rough_ground
仍然是可通行地形。若一小块 grass 位于闭运算地面支持区且与 rough_ground/平地齐平，
它也会按统一的静态恢复规则被填成最近地面类别。

## 案例三：让恢复更保守或更积极

恢复范围不再按类别单独设置；安全边界集中在局部 voxel 配置的两个参数：

```yaml
# 更保守：只闭合窄缝，并只接受几乎齐平的点
terrain_boundary_closing_radius: 0.10
terrain_boundary_max_height_difference: 0.05

# 更积极：允许闭合更宽区域和轻微起伏
# terrain_boundary_closing_radius: 0.30
# terrain_boundary_max_height_difference: 0.15
```

第一组只消除很窄的地面欠分割；第二组也可能恢复较宽的 manhole、路面色带或低矮
物体，现场测试时应先增大半径，再按机器人可跨越高度调整高差。

## 案例四：grass 和 rough_ground 都设为静态障碍

将两个条目都设置为：

```yaml
role: static_obstacle
semantic_cost: 1.00
global_map: true
```

结果：只有 flat_ground 构成白色地面；label 0、grass、rough_ground 和
manhole_cover 默认都是静态障碍，但位于闭运算地面支持区且高差通过时仍会按统一
规则恢复为 flat_ground。

## 案例五：新增 gravel 类别

假设上游新增 label 5，颜色为 `[100,100,60]`，允许通行且可作为恢复参考：

```yaml
- label: 5
  name: gravel
  meaning: traversable compact gravel
  rgb: [100, 100, 60]
  role: terrain
  semantic_cost: 0.30
  global_map: true
```

重启后 label 5 自动进入局部/FAR 地形集合、全局地形颜色组和恢复参考集合，不需要
修改 C++ 或其他 label 数组。必须保证输入点云确实使用 label 5 和相同语义含义。

## 案例六：新增动态类别

例如新增 label 12 行人：

```yaml
- label: 12
  name: person
  meaning: moving person
  rgb: [220, 20, 60]
  role: dynamic_obstacle
  semantic_cost: 1.00
  global_map: false
```

结果：它进入最新局部瞬时障碍层，但不会被写进持久 SemanticOctomap，也不能作为
静态恢复候选或地面参考。

## 常见错误

- 不要把有真实观测的 label 0 从 `classes` 删除；否则它会落入真正 unknown 策略。
- 不要为了“禁止恢复”把行人、车辆写成 `static_obstacle`；动态类别必须使用 `dynamic_obstacle`。
- 所有 `static_obstacle` 都使用同一套闭运算和高差规则；当前没有按单个静态类别设置例外。
- 不要给不同类别重复使用同一个 RGB；FAR 的全局语义匹配会产生歧义。
- `dynamic_obstacle` 和 `ignore` 不允许 `global_map: true`。
- 将 terrain 的 `semantic_cost` 设置得高于 FAR 局部障碍阈值，会使它持续成为瞬时障碍。
- 切换类别角色后要重启整套 launch，不能保留按旧角色建立的全局地图。
