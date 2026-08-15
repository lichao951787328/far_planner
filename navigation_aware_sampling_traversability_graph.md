# Navigation-Aware Sampling + Traversability-Weighted Graph

## 1. 目标与总体思想

本文档给出一套面向复杂地形机器人导航的稀疏图构建与搜索方法。核心目标不是将环境简单离散为大量规则栅格，也不是完全依赖障碍轮廓或随机采样，而是：

1. **只在真正会影响导航决策的位置产生节点（node）**；
2. **用稀疏图（graph）表达长期可复用的环境结构**；
3. **用边（edge）同时表达几何距离和地形可通行性代价**；
4. **在未知/增量环境中只更新局部节点和边，而不是反复重建全图**；
5. **通过带权图搜索得到兼顾路径长度、地形难度、风险和不确定性的路径**。

整套框架可以概括为：

```text
3D/2D Environment Representation
        ↓
Traversability / Semantic / Geometry Map
        ↓
Navigation-Aware Candidate Sampling
        ↓
Candidate Node Suppression & Sparsification
        ↓
Local Edge Construction
        ↓
Traversability-Aware Edge Evaluation
        ↓
Incremental Sparse Graph
        ↓
A* / Dijkstra / Weighted Search
        ↓
Global or Long-Range Route
```

该方法的核心不是某一种特定地图结构。底层地图可以是：

- 2D occupancy grid；
- elevation map；
- semantic grid；
- 3D semantic OctoMap 的二维投影；
- surfel / point cloud 投影地图；
- 其他能够提供几何和地形属性查询的数据结构。

因此，四叉树可以作为底层压缩和查询结构，但**不是方法成立的必要条件**。

---

# 2. 与 Voronoi、PRM、FAR Planner 的核心差异

可以从“为什么这里需要一个节点”这一问题区分不同 roadmap 方法。

## 2.1 PRM

PRM 的节点来源主要是随机采样：

\[
q_i \sim P(C_{free})
\]

节点存在的原因是：

> 这里被采样到了，并且位于自由空间中。

其优点是通用、容易实现，但节点本身通常不对应显式的导航意义。

---

## 2.2 Voronoi Roadmap

Voronoi 的节点和边来自障碍物诱导的自由空间骨架：

\[
d(x,O_i)=d(x,O_j)
\]

其节点往往位于：

- 中轴；
- 岔路；
- 瓶颈；
- 通道中心。

节点存在的原因是：

> 这里是自由空间几何拓扑的重要位置。

---

## 2.3 FAR Planner

FAR Planner 的图主要由障碍轮廓几何和可见性关系诱导。

节点通常与障碍多边形顶点、导航拐点等结构相关，edge 表达节点间的 visibility。

节点存在的原因是：

> 这里是绕障、视线连接或路径转折的重要几何位置。

---

## 2.4 本方法

本文方法更强调：

> **只要该位置会显著影响未来的导航决策，它就值得成为 node。**

因此节点可以来自：

\[
\boxed{
Node = f(Geometry, Traversability, Semantic, Topology, Frontier, Coverage)
}
\]

也可以用一句更概括的话表示：

\[
\boxed{Node \iff Navigation\ Decision\ Changes}
\]

即节点不再只由障碍物决定，也不依赖随机撒点，而是由“导航决策复杂度”决定。

---

# 3. 地图输入与基本属性

设机器人维护一个环境地图，每个位置 \(x=(x,y)\) 至少能够查询以下部分信息：

\[
m(x)=\{o(x),s(x),\tau(x),r(x),u(x),g(x)\}
\]

其中：

- \(o(x)\)：occupancy，是否被障碍占据；
- \(s(x)\)：semantic label，例如 road、grass、gravel、mud；
- \(\tau(x)\)：traversability，可通行性；
- \(r(x)\)：risk，局部风险；
- \(u(x)\)：uncertainty，观测不确定性；
- \(g(x)\)：局部几何特征，例如 slope、roughness、height variance、step height 等。

## 3.1 Traversability 的建议定义

可采用：

\[
\tau(x)\in[0,1]
\]

其中：

- \(\tau=1\)：非常容易通过；
- \(\tau\rightarrow 0\)：越来越难；
- \(\tau=0\)：不可通行。

需要注意：规划中最好不要直接使用 traversability 作为 cost，因为高 traversability 应对应低 cost。

因此定义单位距离 terrain cost：

\[
c_t(x)=f(\tau(x))
\]

最简单可以使用：

\[
c_t(x)=1+\lambda_t(1-\tau(x))
\]

如果希望在低 traversability 区域快速增加惩罚，可以采用非线性函数，例如：

\[
c_t(x)=1+\lambda_t\left(\frac{1}{\tau(x)+\epsilon}-1\right)
\]

或者：

\[
c_t(x)=\exp(\lambda_t(1-\tau(x)))
\]

这样当地形接近不可通行状态时，cost 会快速增加。

---

# 4. 整体打点策略

本文建议采用**多源候选节点生成 + 稀疏化**，而不是单一采样策略。

最终候选节点集合定义为：

\[
\boxed{
V_{cand}
=
V_{terrain}
\cup
V_{bottle}
\cup
V_{junction}
\cup
V_{frontier}
\cup
V_{coverage}
}
\]

分别对应：

1. Traversability Cost Transition Node；
2. Bottleneck / Geometric Topology Node；
3. Junction Node；
4. Frontier Node；
5. Sparse Coverage Node。

然后经过节点合并、NMS、冗余删除等步骤得到最终图节点：

\[
V = Suppress(V_{cand})
\]

---

# 5. 第一类节点：Traversability Cost Transition Node

这是本方法最关键的一类节点。

## 5.1 为什么不是单纯 Semantic Boundary

例如：

```text
grass | gravel
      |
      ●
```

语义变化通常会引起通行代价变化，但 semantic 不应成为唯一依据。

因为即使语义不变，也可能出现：

```text
grass flat  →  grass steep slope
```

此时：

\[
s(x_1)=s(x_2)
\]

但：

\[
|c_t(x_1)-c_t(x_2)| \gg 0
\]

因此更准确的定义是：

> **Traversability Cost Transition Node：在单位通行代价场发生明显变化的位置产生节点。**

---

## 5.2 检测方法

可在二维 cost field 上计算局部梯度：

\[
\nabla c_t(x)
\]

若：

\[
\|\nabla c_t(x)\| > \theta_{grad}
\]

则该位置是候选 transition point。

也可以使用邻域差异：

\[
\max_{y\in\mathcal{N}(x)} |c_t(x)-c_t(y)| > \theta_t
\]

即可认为存在明显 terrain-cost transition。

---

## 5.3 避免沿整条边界产生过多节点

一条 grass/gravel 边界可能非常长，因此不能每一个 grid cell 都变 node。

建议：

1. 提取 cost-transition contour；
2. 对 contour 进行折线简化；
3. 只保留：
   - 端点；
   - 曲率大的位置；
   - 和其他 transition contour 相交的位置；
   - 局部通道口；
   - 有实际路径穿越价值的位置。

可以使用 Douglas-Peucker 或基于曲率的 contour simplification。

最终得到：

```text
原始 transition contour：
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

稀疏化后：
●-------------●-----------●
```

---

# 6. 第二类节点：Bottleneck / Geometric Topology Node

这一类节点负责避免纯 traversability 方法漏掉几何通道结构。

例如：

```text
████████       ████████
████████   ●   ████████
████████       ████████
```

左右地形都可能是相同 grass，但中间是唯一狭窄通道。

这种位置必须存在 node。

## 6.1 Clearance

定义障碍距离场：

\[
D(x)=\min_{o\in O}\|x-o\|
\]

局部 clearance 小、且位于自由空间连通关键位置的点，可以作为 bottleneck candidate。

---

## 6.2 利用 Voronoi 作为 proposal generator

不需要使用完整 Voronoi roadmap。

可以仅计算局部 Voronoi skeleton：

```text
obstacle    free corridor    obstacle
██████          |            ██████
██████          ●            ██████
██████          |            ██████
```

然后只提取：

- Voronoi branch point；
- Voronoi local minimum clearance；
- narrow passage center；
- topology change point。

也就是说：

\[
Voronoi\ Skeleton \rightarrow Node\ Proposal
\]

而不是：

\[
Voronoi\ Skeleton = Final\ Graph
\]

这可以兼顾 Voronoi 的 topology 感知能力，又避免整个图被障碍几何完全支配。

---

# 7. 第三类节点：Junction Node

Junction 表示局部有多个不同可通行方向，是典型 route decision point。

例如：

```text
             road
               |
               |
grass ---------●--------- gravel
               |
               |
              mud
```

一个简单条件是局部区域连接度：

\[
deg(v)\geq 3
\]

但实际检测可以发生在图构建之前。

可基于：

- skeleton branch；
- terrain-region adjacency；
- local free-space directions；
- local connected-component branch。

这一类节点的重要意义在于：

> 它对应真正的路径选择位置。

因此通常应设置较高的保留优先级，不应轻易被 NMS 删除。

---

# 8. 第四类节点：Frontier Node

未知环境中需要让图随着探索向未知区域增长，因此需要 frontier node。

Frontier 定义为：

\[
known\ free \leftrightarrow unknown
\]

的边界。

原始 frontier cell 通常很多：

```text
????????????????????????????
----------------------------
known known known known known
```

不应该每一个 frontier cell 都变 node。

## 8.1 Frontier clustering

先将连续 frontier cell 聚类：

\[
F = \{F_1,F_2,\dots,F_k\}
\]

每个 cluster 选一个或少数代表点。

代表点可以是：

- cluster centroid；
- 最大 clearance 点；
- 最大 information gain 点；
- 最接近当前 graph 的位置；
- 满足可达性的候选点。

得到：

```text
?????????????????????????
      ●            ●
-------------------------
known region
```

---

# 9. 第五类节点：Sparse Coverage Node

仅依赖关键结构节点会有一个问题：

如果存在大片均匀、开阔且没有明显 transition / obstacle / frontier 的区域，则可能一个 node 都没有。

例如：

```text
+-----------------------------------+
|                                   |
|              grass                |
|                                   |
+-----------------------------------+
```

因此需要少量 coverage node 保证图的连通。

## 9.1 推荐策略：最小距离约束

定义已有图节点集合为 \(V\)。

若某个可通行位置满足：

\[
\min_{v_i\in V}\|x-v_i\| > r_{cover}
\]

则可以在该区域添加 coverage node。

也可以使用：

- Poisson-disk sampling；
- farthest point sampling；
- blue-noise sampling；
- local distance-threshold sampling。

相比纯 PRM，其目标不是随机逼近整个自由空间，而是：

> 只填补已有 graph 表达不到的大片区域。

---

# 10. 候选节点优先级

建议不同 node 类型有不同保留优先级：

```text
最高：Junction / Bottleneck
  ↓
Traversability Transition
  ↓
Frontier
  ↓
Coverage
```

可以定义 node importance：

\[
I(v)=
w_bI_b+
w_jI_j+
w_tI_t+
w_fI_f+
w_cI_c
\]

但工程初期不建议把所有东西都揉成一个连续 score。

更容易解释和调试的方法是：

1. 类型级优先级；
2. 同类型内使用 score 排序；
3. 最后 NMS。

---

# 11. Node Suppression / NMS

多种 proposal source 会产生重复点。

例如：

- transition point 和 bottleneck point 很接近；
- junction 同时也是 Voronoi branch；
- frontier 点附近又产生 coverage node。

因此必须做 node suppression。

## 11.1 距离条件

若：

\[
\|v_i-v_j\| < r_{merge}
\]

则两个 node 进入竞争。

## 11.2 合并规则

优先保留：

1. higher-priority node type；
2. higher importance；
3. larger clearance；
4. lower uncertainty；
5. 更靠近局部代表位置。

例如：

```text
coverage ● ● transition
```

若距离小于阈值，则保留 transition node。

---

# 12. 图节点的数据结构

一个 node 不应该只有二维坐标。

推荐至少保存：

```cpp
struct Node {
    int id;

    Vec2 position;

    NodeType type;  // bottleneck / junction / transition / frontier / coverage

    float clearance;
    float traversability;
    float uncertainty;

    int semantic_label;

    bool valid;

    double last_observed_time;
};
```

如果希望加入长期经验，可继续保存：

```cpp
float traversal_success_rate;
float slip_score;
float tracking_error;
float energy_cost;
int visit_count;
```

---

# 13. 连边的基本原则

得到节点以后，需要构建 edge。

不建议像全 visibility graph 一样连接所有互相可见的点，否则开阔区域中可能出现：

\[
O(N^2)
\]

数量级的边。

建议使用局部候选连接。

## 13.1 邻域候选

对于每个 node \(v_i\)，查找：

\[
\mathcal{N}_k(v_i)
\]

即：

- K nearest neighbors；或
- radius neighbor。

例如：

\[
k=5\sim10
\]

或者：

\[
\|v_i-v_j\|<r_{connect}
\]

作为候选 edge。

---

# 14. 连边不是简单 Collision-Free，而是 Traversable Corridor

传统 PRM 可能只检查：

\[
CollisionFree(v_i,v_j)
\]

本文建议检查：

\[
Traversable(v_i,v_j)
\]

即两个 node 之间必须存在一条对机器人实际尺寸而言可接受的可通行走廊。

---

# 15. 为什么不能只检查一条理想直线

机器人有宽度。

如果只检查中心线：

```text
     center line
●----------------●
```

可能出现中心线没撞障碍，但机器人身体两侧进入危险区域。

因此应考虑 edge corridor：

\[
\mathcal{C}_{ij}=Tube(e_{ij},r_{robot}+r_{safe})
\]

其中：

- \(r_{robot}\)：机器人 footprint 半径近似；
- \(r_{safe}\)：安全裕度。

对于四足/轮足平台，可使用：

- 圆形 footprint；
- 矩形 footprint；
- 基于实际机身轮廓的 swept volume。

---

# 16. Edge Validity

一条候选边必须满足：

## 16.1 Occupancy 条件

\[
\max_{x\in\mathcal{C}_{ij}} p_{occ}(x) < \theta_{occ}
\]

否则：

\[
e_{ij}=invalid
\]

---

## 16.2 Traversability 硬约束

如果：

\[
\min_{x\in\mathcal{C}_{ij}}\tau(x)<\tau_{critical}
\]

则这条边不能通过。

或者等价地：

\[
\max c_t(x)>c_{critical}
\]

则：

\[
C(e_{ij})=\infty
\]

---

## 16.3 几何约束

可以额外要求：

\[
Slope(x)<Slope_{max}
\]

\[
Roughness(x)<Roughness_{max}
\]

\[
StepHeight(x)<H_{max}
\]

超过硬件极限就直接 invalid，而不是只提高 cost。

这是非常重要的区别：

- **软约束** → 加代价；
- **硬约束** → 删除 edge。

---

# 17. Edge 的几何长度

两节点二维坐标为：

\[
p_i=(x_i,y_i),\quad p_j=(x_j,y_j)
\]

最基本边长：

\[
L_{ij}=\|p_i-p_j\|_2
\]

若地图包含高度，并希望更加准确，可以使用 3D surface length。

例如对采样点：

\[
x_k=(x_k,y_k,z_k)
\]

则：

\[
L_{ij}^{3D}\approx\sum_k\|x_{k+1}-x_k\|
\]

初期二维规划可以先采用 2D 距离，将 slope 等影响计入 terrain cost。

---

# 18. 最重要部分：Edge Terrain Cost

本文推荐的基本思想是：

\[
\boxed{
C_{terrain}(e_{ij})
=
\int_{e_{ij}} c_t(x)\,ds
}
\]

这比：

\[
\alpha L+\beta T
\]

更自然。

原因是：

> 地形困难度本质上是单位距离代价，走得越远应该累计得越多。

---

# 19. 离散计算 Edge Cost

地图是离散的，因此实际计算采用沿 edge/corridor 采样。

在 edge 上取：

\[
x_1,x_2,\dots,x_N
\]

采样间隔为：

\[
\Delta s
\]

则：

\[
C_{terrain}
\approx
\sum_{k=1}^{N} c_t(x_k)\Delta s
\]

例如一条边经过：

- 5 m grass，单位 cost 1.2；
- 3 m gravel，单位 cost 1.8；
- 1 m mud，单位 cost 3.0。

则：

\[
C=5\times1.2+3\times1.8+1\times3.0=14.4
\]

---

# 20. 为什么不能只看平均 Terrain Cost

如果直接使用：

\[
\bar c_t=\frac{1}{L}\int c_t(x)ds
\]

然后：

\[
C=L\bar c_t
\]

本质上等价于积分，在累计代价上是正确的。

但还有一个问题：**局部极危险区域可能被平均掉。**

例如：

```text
road road road MUD road road road
```

mud 只有很短一段。

整体平均 cost 可能还不高，但机器人实际通过这一小段时风险很大。

因此需要额外引入 risk term。

---

# 21. 最大风险项

定义：

\[
R_{max}(e_{ij})
=
\max_{x\in\mathcal{C}_{ij}} r(x)
\]

或者简单使用：

\[
R_{max}=\max c_t(x)
\]

最终：

\[
\boxed{
C(e_{ij})
=
\int c_t(x)ds
+
\lambda_rR_{max}
}
\]

并设置：

\[
R_{max}>R_{critical}
\Rightarrow
e_{ij}=invalid
\]

这样可以避免平均值隐藏极端风险。

---

# 22. 不确定性代价

未知或低置信度环境不能和已知平坦区域等价。

定义 edge uncertainty：

\[
U_{ij}
=
\frac{1}{L_{ij}}
\int_{e_{ij}}u(x)ds
\]

最终：

\[
C(e_{ij})
=
\int c_t(x)ds
+
\lambda_rR_{max}
+
\lambda_uU_{ij}
\]

注意：

- 探索模式下 \(\lambda_u\) 可以小；
- 安全导航模式下 \(\lambda_u\) 应增大。

---

# 23. 推荐的第一版 Edge Cost

为了避免初期参数过多，建议第一版使用：

\[
\boxed{
C(e_{ij})
=
\int_{e_{ij}}
[1+\lambda_t c_t(x)]\,ds
+
\lambda_rR_{ij}
+
\lambda_uU_{ij}
}
\]

其中：

- 第一项中的 1：基础距离代价；
- \(c_t(x)\)：地形难度；
- \(R_{ij}\)：最大局部风险；
- \(U_{ij}\)：不确定性。

若某处不可通行：

\[
c_t(x)>c_{critical}
\]

则：

\[
C(e_{ij})=\infty
\]

---

# 24. Terrain Cost 的组成

地形代价不应只由 semantic label 决定。

可以定义：

\[
c_t(x)
=
f(
semantic,
slop e,
roughness,
step,
height\ variance,
slip\ prior
)
\]

例如线性第一版：

\[
c_t(x)
=
 w_s c_s(x)
+w_p c_p(x)
+w_r c_r(x)
+w_h c_h(x)
\]

其中：

- \(c_s\)：semantic cost；
- \(c_p\)：slope cost；
- \(c_r\)：roughness cost；
- \(c_h\)：height/step cost。

建议各分量先归一化到 \([0,1]\)。

---

# 25. Semantic Cost Example

可以给不同 semantic 一个先验：

| Terrain | Prior Cost |
|---|---:|
| road | 0.05 |
| flat floor | 0.05 |
| grass | 0.20 |
| gravel | 0.40 |
| loose rock | 0.60 |
| mud | 0.75 |
| deep vegetation | 0.80 |
| obstacle | invalid |

这只是 prior，不代表最终 traversability。

例如同样 grass：

```text
flat grass   → low cost
steep grass  → high cost
```

---

# 26. Experience-Aware Traversability

机器人通过环境以后，可以将真实执行结果反馈给 graph。

例如观测：

- slip ratio；
- velocity tracking error；
- roll/pitch vibration；
- body height fluctuation；
- foot/wheel impact；
- power consumption；
- traversal success/failure。

定义经验代价：

\[
c_{exp}(x)
\]

则：

\[
c_t(x)
=
(1-\alpha)c_{perception}(x)
+
\alpha c_{exp}(x)
\]

这样地图不只是“看起来能不能走”，还会逐渐学习：

> 机器人自己实际走起来是否困难。

---

# 27. Edge 数据结构

建议 edge 不仅保存一个 total cost。

```cpp
struct Edge {
    int id;
    int from;
    int to;

    float length;

    float terrain_integral_cost;
    float mean_traversability_cost;
    float max_risk;
    float uncertainty_cost;

    float total_cost;

    bool valid;

    double last_update_time;
};
```

这样未来可以改变 planner 偏好，而不需要重新采样全部地图。

---

# 28. Edge Directionality

复杂地形中，edge 可能不是对称的。

例如坡地：

```text
A ---- uphill ---- B
```

A→B 和 B→A 的难度不同。

因此可以使用 directed graph：

\[
C_{A\rightarrow B}\neq C_{B\rightarrow A}
\]

例如：

\[
c_{slope}(x,\theta_{travel})
\]

依赖机器人运动方向与坡度方向。

这在四足/轮足机器人中很有价值。

---

# 29. Graph Sparsification：冗余节点删除

随着探索持续进行，graph 不应只增不减。

考虑：

```text
A ----- B ----- C
```

如果：

1. B 不是 junction；
2. B 不是 bottleneck；
3. B 不是 frontier；
4. B 不是重要 transition；
5. A 和 C 可直接连接；
6. 直接连接代价和原路径接近；

即：

\[
|C(A,C)-[C(A,B)+C(B,C)]|<\epsilon_c
\]

则 B 可以删除。

---

# 30. Incremental Graph Update

机器人在未知环境中持续获取新观测。

定义当前观测改变的区域：

\[
\Omega_{update}
\]

只在这个区域及其 buffer 范围内执行增量更新：

\[
\Omega' = Dilate(\Omega_{update},r_{update})
\]

更新流程：

```text
new sensor observation
        ↓
update local map
        ↓
find changed region Ω
        ↓
recompute local node proposals
        ↓
match old/new nodes
        ↓
add / remove / move local nodes
        ↓
recompute affected edges
        ↓
run graph search if needed
```

---

# 31. 新节点添加

新候选节点 \(v_{new}\) 进入 graph 前：

1. valid occupancy；
2. traversable；
3. 和已有节点距离大于最低分辨率；
4. 不被更高优先级节点 NMS 掉；
5. 能够与至少一个已有节点建立有效边。

如果都满足：

\[
V_{t+1}=V_t\cup\{v_{new}\}
\]

---

# 32. Node Matching

局部重检测时，不能把附近所有旧点删掉再创建全新 ID。

需要 old-new matching。

若：

\[
\|v_i^{old}-v_j^{new}\|<r_{match}
\]

并且 node type 一致或兼容，则认为是同一个结构。

更新：

- position；
- score；
- semantic；
- traversability；
- timestamp。

而不是创建新 node。

这样 global graph 的 node ID 更稳定。

---

# 33. 节点删除

node 可以在以下情况删除：

## 33.1 被障碍占据

\[
p_{occ}(v)>\theta_{occ}
\]

---

## 33.2 原结构消失

例如原来的 grass/mud transition 被后续观测修正为全 grass。

则 transition node 失去意义。

---

## 33.3 frontier 消失

原来的 unknown region 已经被观测，frontier node 需要删除或转化为其他 node 类型。

---

## 33.4 冗余删除

通过 graph sparsification 删除。

---

# 34. Edge 增量更新

如果 map changed region 与 edge corridor 相交：

\[
\mathcal{C}_{ij}\cap\Omega_{update}\neq\emptyset
\]

则重新评估：

- occupancy；
- traversability；
- risk；
- uncertainty；
- total cost。

否则 edge 不需要重新计算。

这是增量效率的关键。

---

# 35. Edge 失效但 Node 不一定删除

例如原来：

```text
A ---------------- B
```

后来中间出现障碍：

```text
A -------████----- B
```

则：

\[
e_{AB}=invalid
\]

但 A 和 B 本身仍可能是重要 node，因此不应该同时删除。

这使 node topology 与 edge validity 解耦。

---

# 36. 图搜索

构建带权图：

\[
G=(V,E,C)
\]

其中：

- \(V\)：稀疏导航节点；
- \(E\)：可通行边；
- \(C(e)\)：edge cost。

可使用：

- Dijkstra；
- A*；
- Weighted A*；
- D* Lite（如果频繁动态更新）；
- LPA*。

第一版建议使用 A*。

---

# 37. A* Cost

\[
f(n)=g(n)+h(n)
\]

其中：

\[
g(n)=\sum_{e\in path}C(e)
\]

也就是从 start 到当前 node 的累计 edge cost。

---

# 38. A* Heuristic

为保持 admissible，可使用最低单位成本：

\[
h(n)=c_{min}\|p_n-p_g\|
\]

其中：

\[
c_{min}=\min_x c_t(x)
\]

如果 edge cost 采用：

\[
\int [1+\lambda_t c_t(x)]ds
\]

且最小单位代价不低于 1，则直接：

\[
h(n)=\|p_n-p_g\|
\]

即可。

---

# 39. Start / Goal 如何接入 Graph

start 和 goal 不一定恰好落在已有 node 上。

因此查询时动态插入：

\[
v_s,v_g
\]

然后分别与附近 K 个 graph node 尝试建立临时 edge。

```text
Start
  ●
 / \
●   ●------●------●
                 / \
                ●   ● Goal
```

规划结束后临时节点和临时 edge 可以删除。

---

# 40. 当 Goal 位于未知区域

如果 goal 不在当前 known graph 内：

可先规划到：

- closest frontier；
- best information-gain frontier；
- 朝 goal 方向最优 frontier。

形式上：

\[
F^*=
\arg\min_{F_i}
\left[
C(start,F_i)+\lambda_g d(F_i,goal)-\lambda_I InfoGain(F_i)
\right]
\]

这属于 exploration-aware extension。

---

# 41. 搜索示例

假设两条路线：

```text
Route A: 10 m grass
Route B: 7 m mud
```

设：

\[
c_{grass}=1.2
\]

\[
c_{mud}=3.0
\]

则：

\[
C_A=10\times1.2=12
\]

\[
C_B=7\times3=21
\]

因此即使 B 更短，A* 仍会选择 A。

这就是 weighted terrain graph 的核心意义。

---

# 42. 一个更复杂示例

地图：

```text
                Goal
                 G

       gravel       grass
          \           /
           ●---------●
           |         |
           |  mud    |
           ●---------●
             \
              S
```

可能存在：

路径 1：

\[
S\rightarrow mud\rightarrow G
\]

距离短，但 terrain cost 高。

路径 2：

\[
S\rightarrow gravel\rightarrow grass\rightarrow G
\]

距离长，但单位通行代价低。

最终 graph search 实际优化：

\[
\min_{P}\sum_{e\in P} C(e)
\]

而不是：

\[
\min_{P}\sum_{e\in P} L(e)
\]

---

# 43. 整套算法伪代码

## 43.1 Incremental Graph Update

```text
Algorithm: UpdateNavigationGraph(M_t, G_t, NewObservation)

Input:
    current map M_t
    current graph G_t = (V_t, E_t)
    new sensor observation Z_t

Output:
    updated map M_{t+1}
    updated graph G_{t+1}

1. M_{t+1} ← FuseObservation(M_t, Z_t)

2. Ω_update ← DetectChangedRegion(M_t, M_{t+1})
3. Ω_local  ← Dilate(Ω_update, update_radius)

4. Remove / mark local stale candidate information in Ω_local

5. V_terrain  ← DetectTraversabilityTransitions(M_{t+1}, Ω_local)
6. V_bottle   ← DetectBottlenecks(M_{t+1}, Ω_local)
7. V_junction ← DetectJunctions(M_{t+1}, Ω_local)
8. V_frontier ← DetectFrontiers(M_{t+1}, Ω_local)
9. V_cover    ← AddSparseCoverageSamples(M_{t+1}, G_t, Ω_local)

10. V_cand ← Union(
        V_terrain,
        V_bottle,
        V_junction,
        V_frontier,
        V_cover)

11. V_cand ← NodeNMS(V_cand)

12. Match V_cand against old nodes in Ω_local

13. Update matched nodes
14. Add unmatched valid new nodes
15. Remove old nodes whose navigation structure disappeared
16. Remove redundant nodes by local graph sparsification

17. E_affected ← edges whose endpoint or corridor intersects Ω_local
18. Remove / invalidate E_affected

19. For each updated/new node v_i:
        N ← KNN(v_i, V, k)
        For each v_j in N:
            if TraversableCorridor(v_i, v_j, M_{t+1}):
                edge ← EvaluateEdge(v_i, v_j, M_{t+1})
                if edge.valid:
                    Add edge

20. Return M_{t+1}, G_{t+1}
```

---

# 44. Edge Evaluation 伪代码

```text
Algorithm: EvaluateEdge(v_i, v_j, Map)

Input:
    node v_i
    node v_j
    terrain map

Output:
    weighted edge e_ij

1. path_line ← straight segment(v_i.position, v_j.position)

2. corridor ← Inflate(path_line, robot_radius + safety_margin)

3. samples ← SampleCorridor(corridor, resolution = Δs)

4. if any occupancy(sample) > occ_threshold:
       return INVALID

5. if any traversability(sample) < tau_critical:
       return INVALID

6. if any slope(sample) > slope_max:
       return INVALID

7. if any step_height(sample) > step_max:
       return INVALID

8. length ← ComputeGeometricLength(path_line)

9. terrain_integral ← 0
10. max_risk ← 0
11. uncertainty_sum ← 0

12. for each sample x_k:
        c_t ← TerrainCost(x_k)
        terrain_integral += c_t * Δs
        max_risk = max(max_risk, Risk(x_k))
        uncertainty_sum += Uncertainty(x_k) * Δs

13. uncertainty ← uncertainty_sum / length

14. total_cost ← terrain_integral
                 + lambda_r * max_risk
                 + lambda_u * uncertainty

15. return Edge(
        valid=true,
        length=length,
        terrain_cost=terrain_integral,
        max_risk=max_risk,
        uncertainty=uncertainty,
        total_cost=total_cost)
```

---

# 45. Search 伪代码

```text
Algorithm: Plan(start, goal, Graph)

1. Insert temporary start node v_s
2. Connect v_s to nearby valid graph nodes

3. Insert temporary goal node v_g
4. Connect v_g to nearby valid graph nodes

5. Run A*:

       g(v_s) = 0
       h(v) = c_min * EuclideanDistance(v, v_g)

       f(v) = g(v) + h(v)

       edge transition:
           g(v_j) = g(v_i) + C(e_ij)

6. Extract minimum-cost node sequence

7. Optionally smooth / locally optimize graph path

8. Remove temporary start/goal nodes

9. Return graph route
```

---

# 46. 路径后处理

Graph path 是一串 node：

\[
P=[v_0,v_1,\dots,v_n]
\]

可以继续做：

- shortcut；
- spline smoothing；
- local trajectory optimization；
- MPC reference generation；
- local policy subgoal generation。

但 shortcut 不能只做 collision check，仍需检查：

\[
C(v_i,v_j)
\]

是否优于原来的多段 edge。

例如只有在：

\[
C(v_i,v_j)
<
\sum_{k=i}^{j-1}C(v_k,v_{k+1})
\]

且满足安全约束时，才执行 shortcut。

---

# 47. 与局部规划/运动控制的接口

该 graph 的输出不一定直接是速度命令。

可输出：

1. graph node sequence；
2. next subgoal；
3. local corridor；
4. desired direction；
5. terrain-aware reference path。

然后交给：

- local planner；
- locomotion policy；
- RL navigation policy；
- MPC；
- trajectory optimizer。

例如：

```text
Global Sparse Graph
       ↓
next graph node / local goal
       ↓
Local Motion Policy
       ↓
velocity / foothold / wheel command
```

---

# 48. 设计中的三个层次必须分开

建议明确区分：

## Level 1：Map

保存完整或相对密集的环境信息。

## Level 2：Sparse Graph

保留导航决策结构。

## Level 3：Trajectory / Control

处理机器人真实动力学和短时避障。

即：

```text
Dense/Semi-dense Map
        ↓
Sparse Navigation Graph
        ↓
Local Trajectory / Policy
```

这样 graph 不需要承担所有底层运动细节。

---

# 49. 为什么这一方案比单纯 Quadtree 更一般

如果强制使用 Quadtree：

\[
Map\rightarrow Quadtree\rightarrow Graph
\]

方法会依赖特定空间划分结构。

而本文逻辑是：

\[
Map
\rightarrow
Navigation\ Structure\ Detection
\rightarrow
Sparse\ Graph
\]

底层地图可以替换，但上层算法仍成立。

因此更适合形成一个独立的规划方法。

---

# 50. 可以形成的方法命名

以下名字可作为未来论文中临时概念名：

- Navigation-Aware Sparse Roadmap；
- Traversability-Aware Sparse Graph；
- Terrain-Aware Sparse Roadmap；
- Navigation-Complexity-Aware Roadmap；
- Semantic Traversability Graph；
- Terrain-Weighted Navigation Graph。

如果突出两大核心，可以用：

> **Navigation-Aware Sampling and Traversability-Weighted Roadmap**

简称可以后续再设计。

---

# 51. 核心创新表达

整套方法最核心的三点可以概括为：

## 51.1 Navigation-aware node sampling

不是随机打点，也不是只围绕障碍打点，而是：

\[
Node\iff Navigation\ Decision\ Changes
\]

节点来源于：

- geometry bottleneck；
- topology junction；
- traversability transition；
- frontier；
- sparse coverage。

---

## 51.2 Traversability-weighted edge

edge 不再只代表：

\[
CollisionFree?
\]

或者：

\[
Visible?
\]

而是表示：

> 这段走廊是否可通行，以及实际通过需要付出多少地形代价。

核心公式：

\[
\boxed{
C(e)
=
\int_e c_t(x)\,ds
+
\lambda_rR_e
+
\lambda_uU_e
}
\]

---

## 51.3 Incremental graph maintenance

graph 不随运行时间无限增长，而根据局部环境结构：

- 添加重要 node；
- 删除失效 node；
- 删除冗余 node；
- 重新评估受环境变化影响的 edge。

理想情况下：

\[
Graph\ Size
\propto
Navigation\ Complexity
\]

而不是：

\[
Graph\ Size
\propto
Exploration\ Time
\]

---

# 52. 建议的第一阶段最小可实现版本（MVP）

为了避免一开始系统过于复杂，建议第一阶段只实现：

## 地图

2D traversability grid：

\[
\tau(x)\in[0,1]
\]

加 occupancy。

## 节点

只使用：

1. Voronoi bottleneck / branch；
2. traversability gradient transition；
3. sparse coverage。

frontier 和 experience 先暂缓。

## 连边

KNN + straight corridor check。

## Edge cost

\[
C(e)
=
\sum_k[1+\lambda_t c_t(x_k)]\Delta s
+
\lambda_r\max_k c_t(x_k)
\]

## 搜索

A*。

这个版本已经可以做非常清晰的对比实验。

---

# 53. 第二阶段扩展

加入：

- semantic；
- frontier；
- directed slope cost；
- dynamic update；
- uncertainty；
- graph sparsification。

---

# 54. 第三阶段扩展

加入机器人经验反馈：

\[
Perception\ Traversability
+
Execution\ Experience
\rightarrow
Updated\ Traversability
\]

例如通过：

- slip；
- tracking error；
- energy；
- body instability；
- traversal failure。

使 graph 成为长期 experience-aware navigation memory。

---

# 55. 建议的实验对比

如果未来做论文实验，可与以下方法比较：

1. grid A*；
2. PRM；
3. Voronoi roadmap；
4. visibility graph / FAR-style graph；
5. 仅长度加权 graph；
6. 本文 terrain-weighted graph。

指标：

- path length；
- integrated terrain cost；
- maximum encountered terrain risk；
- success rate；
- graph node count；
- graph edge count；
- graph update time；
- planning time；
- replanning time；
- robot execution energy；
- slip / instability；
- unknown environment exploration efficiency。

---

# 56. 一个很重要的消融实验

可以分别去掉不同 node source：

- w/o bottleneck；
- w/o transition；
- w/o coverage；
- w/o frontier。

观察：

- 是否漏掉窄通道；
- 是否错误进入高代价地形；
- 是否出现大区域断图；
- 是否探索失败。

这会直接验证打点机制是否合理。

---

# 57. Edge Cost 消融

可以比较：

### 仅长度

\[
C=L
\]

### 长度 + 平均 terrain

\[
C=L\bar c_t
\]

### 积分 terrain

\[
C=\int c_tds
\]

### 积分 + max risk

\[
C=\int c_tds+\lambda_rR_{max}
\]

### 完整版本

\[
C=\int c_tds+\lambda_rR_{max}+\lambda_uU
\]

这会非常清晰地显示 risk term 和 uncertainty term 的作用。

---

# 58. 参数建议

主要参数包括：

## Sampling

- \(r_{cover}\)：coverage sampling radius；
- \(r_{merge}\)：node NMS radius；
- \(\theta_{grad}\)：terrain cost transition threshold；
- \(\theta_{bottle}\)：bottleneck threshold；
- frontier cluster size threshold。

## Edge

- \(k\)：KNN 数量；
- \(r_{connect}\)：最大连接距离；
- \(r_{safe}\)：机器人安全膨胀半径；
- \(\Delta s\)：edge cost sampling resolution。

## Cost

- \(\lambda_t\)：terrain cost 权重；
- \(\lambda_r\)：最大风险权重；
- \(\lambda_u\)：不确定性权重；
- \(\tau_{critical}\)：硬不可通行阈值。

---

# 59. 参数的尺度问题

建议将各 cost 分量归一化，以免：

\[
\lambda
\]

完全由量纲差异决定。

例如：

\[
c_t\in[0,1]
\]

\[
r\in[0,1]
\]

\[
u\in[0,1]
\]

这样 \(\lambda_t,\lambda_r,\lambda_u\) 更容易解释。

---

# 60. 最终整体逻辑总结

整套方法可以压缩成四个连续问题。

## 60.1 哪里打点？

在会改变导航决策的位置打点：

\[
\boxed{
V=
V_{transition}
\cup
V_{bottleneck}
\cup
V_{junction}
\cup
V_{frontier}
\cup
V_{coverage}
}
\]

然后 NMS 和 sparsification。

---

## 60.2 哪些点连边？

只对局部 KNN / radius neighbor 尝试连接。

必须满足：

\[
TraversableCorridor(v_i,v_j)=true
\]

而不是仅仅中心线 collision-free。

---

## 60.3 边的代价是什么？

基本形式：

\[
\boxed{
C(e)
=
\int_e c_t(x)ds
+
\lambda_rR_e
+
\lambda_uU_e
}
\]

其中距离通过积分自然进入总代价。

---

## 60.4 怎么搜索？

构造：

\[
G=(V,E,C)
\]

用 A*：

\[
f(n)=g(n)+h(n)
\]

其中：

\[
g(n)=\sum C(e)
\]

\[
h(n)=c_{min}\|p_n-p_g\|
\]

最终得到：

> **不是几何最短路线，而是综合距离、地形难度、风险和不确定性的最低总代价路线。**

---

# 61. 一句话方法定义

可以把整套方法概括为：

> **通过导航重要性驱动的非均匀节点采样构建稀疏 roadmap，并将节点间路径段上的几何长度、地形可通行性、局部风险和环境不确定性积分为 edge cost，再通过增量带权图搜索获得适用于复杂地形的低风险低代价导航路线。**

对应英文可写为：

> **A navigation-aware sparse roadmap is incrementally constructed by sampling nodes at geometric bottlenecks, traversability transitions, topological junctions, exploration frontiers, and under-represented free regions. Each edge is evaluated by integrating terrain-dependent traversal cost along a robot-sized traversable corridor, with additional penalties for local risk and uncertainty. A weighted graph search then finds a route that balances geometric distance and terrain difficulty rather than minimizing Euclidean path length alone.**

