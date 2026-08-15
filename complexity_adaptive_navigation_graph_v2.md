# Complexity-Adaptive Navigation Graph
## 面向复杂地形无图导航的稀疏打点、连边、边代价、增量维护与搜索方案（设计草案 v2）

> 目标：给后续代码实现、参数探索、算法消融和论文撰写提供一份统一的技术路线文档。  
> 核心思想：**不要让图的规模主要由地图面积或固定采样分辨率决定，而应尽量由“导航复杂度”决定。**  
> 推荐主线：  
>
> **Complexity-Adaptive Navigation Sampling + Robot-Scale Corridor Evaluation + Traversability-Weighted Sparse Graph**
>
> 中文可暂称：**复杂度自适应导航稀疏图（Complexity-Adaptive Navigation Graph, CANG）**。

---

# 1. 为什么要做这套方法

传统方法通常存在以下几种典型的“打点逻辑”：

- **PRM**：自由空间随机采样，点本身不表达环境结构；
- **Voronoi**：点/边来自障碍物诱导的自由空间骨架；
- **FAR Planner**：主要由障碍轮廓、可见性和几何转折产生图结构；
- **TRG-planner**：从当前参考节点出发，在固定扩展半径附近随机采样，并通过地形稳定性和边的 traversal risk 进行筛选，再以 wavefront 方式增量扩展。

这些方法各有优点，但对于包含草地、碎石、泥地、坡地、粗糙地形等“连续可通行但代价不同”的环境，仍存在一个核心问题：

> **图结构很容易更多地反映“几何可达性”，而不是“导航决策复杂度”。**

我们希望构建的图满足：

1. **简单区域很稀疏**；
2. **复杂区域自动变密**；
3. **重要拓扑位置不能因稀疏化而丢失**；
4. **节点可以很少，但边的地形评价仍保持高分辨率**；
5. **图可以随着机器人探索增量扩展和增量简化**；
6. **边代价同时考虑长度、地形通行难度、风险和不确定性**；
7. **搜索得到的不只是“最短路径”，而是“安全/高效/可执行的低总代价路径”**。

核心设计目标可概括为：

\[
\boxed{
\text{Graph Density} \propto \text{Navigation Complexity}
}
\]

而不是：

\[
\text{Graph Density} \propto \text{Map Area}
\]

也不是：

\[
\text{Graph Density} \propto \text{Fixed Sampling Resolution}
\]

---

# 2. 总体框架

整个系统建议拆成 8 个模块：

```text
3D / 2.5D / Semantic Terrain Map
            │
            ▼
[1] Local Terrain Descriptor
            │
            ▼
[2] Navigation Structure Detection
            │
            ├─ bottleneck
            ├─ junction
            ├─ traversability transition
            ├─ frontier
            └─ coverage gap
            │
            ▼
[3] Complexity-Adaptive Node Sampling
            │
            ▼
[4] Node Validation & Node Suppression
            │
            ▼
[5] Candidate Edge Generation
            │
            ▼
[6] Robot-Scale Corridor Evaluation
            │
            ▼
[7] Weighted Sparse Graph + Incremental Update
            │
            ▼
[8] A* / Dijkstra / Incremental Search
```

建议将地图层和图层明确分开：

- **地图层**：保持相对高分辨率，用于地形评价；
- **图层**：尽可能稀疏，只保存“导航决策结构”。

这条原则非常重要：

\[
\boxed{
\text{Sparse Graph} \neq \text{Sparse Terrain Evaluation}
}
\]

---

# 3. 地图输入与每个位置的基础属性

底层地图不限定具体形式，可以是：

- 2D traversability grid；
- elevation map；
- semantic grid；
- semantic OctoMap 的地面投影；
- surfel / point cloud 投影；
- 多层 terrain map。

对任意位置 \(x\)，希望最终可查询：

\[
m(x)=
\{
O(x),
S(x),
\tau(x),
R(x),
U(x),
G(x)
\}
\]

其中：

- \(O(x)\)：occupancy / collision state；
- \(S(x)\)：semantic label 或 semantic embedding；
- \(\tau(x)\)：traversability；
- \(R(x)\)：risk；
- \(U(x)\)：uncertainty；
- \(G(x)\)：几何信息，例如 slope、roughness、height variation、step 等。

推荐约定：

\[
\tau(x)\in[0,1]
\]

其中：

- \(\tau=1\)：非常容易通过；
- \(\tau=0\)：不可通行。

但用于图搜索时不要直接使用 \(\tau\)，应转成“单位距离通行代价”：

\[
c_t(x)=f(\tau(x))
\]

例如：

\[
c_t(x)=1+\lambda_t(1-\tau(x))
\]

或者采用更强调危险地形的非线性形式：

\[
c_t(x)=
1+\lambda_t
\left(
\frac{1}{\tau(x)+\epsilon}-1
\right)
\]

也可以：

\[
c_t(x)=\exp(\lambda_t(1-\tau(x)))
\]

第一版建议从线性形式开始，后续再比较非线性映射。

---

# 4. 节点的基本定义

一个节点不应只理解成数学点：

\[
v_i=(x_i,y_i)
\]

更推荐：

\[
v_i=
\{
p_i,
type_i,
state_i,
descriptor_i,
confidence_i,
timestamp_i
\}
\]

其中：

- \(p_i\)：节点位置；
- \(type_i\)：节点类型；
- \(state_i\)：valid / invalid / frontier / stale；
- \(descriptor_i\)：局部地形描述；
- \(confidence_i\)：观测可信度；
- \(timestamp_i\)：最近更新时间。

建议节点类型：

```cpp
enum class NodeType {
    START,
    GOAL,
    BOTTLENECK,
    JUNCTION,
    TRAVERSABILITY_TRANSITION,
    FRONTIER,
    COVERAGE,
    EXPERIENCE_KEYPOINT
};
```

其中 `EXPERIENCE_KEYPOINT` 可留作以后扩展。

---

# 5. 核心打点策略：Structural Nodes + Coverage Nodes

最终节点集合：

\[
V=
V_{\text{struct}}
\cup
V_{\text{coverage}}
\]

其中：

\[
V_{\text{struct}}
=
V_{\text{bottleneck}}
\cup
V_{\text{junction}}
\cup
V_{\text{transition}}
\cup
V_{\text{frontier}}
\]

## 5.1 Structural Nodes

这些节点代表真正的导航决策变化，应优先保留。

### 5.1.1 Bottleneck Node

典型场景：

```text
████████       ████████
████████   ●   ████████
████████       ████████
```

作用：

- 保证狭窄通道不会因为稀疏打点漏掉；
- 保留关键连通位置。

候选检测方式：

#### 方案 A：局部 clearance 极小值

定义：

\[
C(x)=d(x,O)
\]

在自由空间中寻找局部瓶颈：

\[
C(x)<\theta_c
\]

并且横向自由宽度在某个局部邻域内取得较小值。

优点：
- 直观；
- 易实现。

缺点：
- 对噪声敏感；
- junction 与 bottleneck 容易混淆。

#### 方案 B：局部 Voronoi / GVD 辅助

不把 Voronoi 当最终 roadmap，只把它作为 proposal generator。

流程：

```text
Local obstacle map
      ↓
local distance field
      ↓
local Voronoi skeleton
      ↓
extract narrow skeleton segments
      ↓
bottleneck candidates
```

优点：
- 对狭窄通道天然敏感；
- 拓扑信息较好。

缺点：
- 需要额外维护 distance field / skeleton。

**推荐：第一版优先采用方案 B 或 A+B。**

---

## 5.2 Junction Node

定义：

> 多条具有独立导航意义的通路在某区域发生汇合或分叉的位置。

例如：

```text
        road
          |
          |
grass ---●--- gravel
          |
          |
         mud
```

可以来自：

- skeleton branch；
- region adjacency degree；
- frontier branch；
- 局部连通分量结构变化。

简单判据：

\[
deg_{\text{local}}(x)\ge 3
\]

第一版可以依赖局部 skeleton branch point；
后续可以发展为基于 traversability-region adjacency 的 junction。

---

## 5.3 Traversability Transition Node

不建议简单理解成“语义边界点”。

真正应该保留的是：

> **地形通行代价发生显著变化的位置。**

判据可以写成：

\[
|\Delta c_t|>\theta_T
\]

或者：

\[
\|\nabla c_t(x)\|>\theta_{\nabla T}
\]

这样：

- grass → gravel，如果代价接近，可能不需要点；
- grass → mud，如果代价突变，应该保留；
- grass → steep grass，即使 semantic 没变，也可能产生节点。

### 方案 A：基于 cost gradient

\[
K_t(x)=\|\nabla c_t(x)\|
\]

优点：
- 与最终规划目标直接一致；
- 不依赖 semantic 标签。

### 方案 B：基于区域分割

先把相似 traversability 区域分成 region，再在 region interface 产生代表节点。

优点：
- 图更结构化；
- 更适合大尺度区域。

缺点：
- 分割质量决定上层图质量。

### 方案 C：基于 path-relevant transition

不是所有代价突变都打点，只保留“可能被路径穿越”的 transition。

例如：
- 边界离当前 graph 很远；
- 被障碍完全隔开；
- 无法形成有效 edge；

则不产生 node。

**推荐探索顺序：A → A+C → B+C。**

---

## 5.4 Frontier Node

用于未知环境增量扩展。

推荐不要把每个 frontier cell 都变成 node。

流程：

```text
frontier cells
    ↓
clustering
    ↓
frontier segment
    ↓
representative point
```

代表点可选：

### 方案 A：frontier 中点
最简单。

### 方案 B：最大 clearance 点
更适合机器人通过。

### 方案 C：最大 information gain 点
适合探索。

### 方案 D：目标方向偏置
如果已知 goal 大致方向，则选择更靠近 goal 的 frontier representative。

推荐第一版：
- cluster + max-clearance；
- 如果有目标，再增加 goal-bias。

---

## 5.5 Coverage Node

Coverage node 只解决一个问题：

> 大片简单区域没有 structural node 时，graph 仍然能够穿过。

例如：

```text
●-------------------------●-------------------------●
```

而不是：

```text
●--●--●--●--●--●--●--●--●--●
```

Coverage node 应服从最小距离约束：

\[
d(v_{\text{new}},V)>r_s(x)
\]

其中 \(r_s(x)\) 是自适应采样半径。

---

# 6. 节点密度控制：Complexity-Adaptive Sampling

这是整个方案的重要贡献候选之一。

定义局部导航复杂度：

\[
K(x)\in[0,1]
\]

可以由以下部分组成：

\[
K(x)
=
w_tK_t(x)
+
w_gK_g(x)
+
w_oK_o(x)
+
w_uK_u(x)
+
w_sK_s(x)
\]

其中：

- \(K_t\)：traversability variation；
- \(K_g\)：几何变化，例如 slope / roughness；
- \(K_o\)：obstacle / clearance complexity；
- \(K_u\)：uncertainty；
- \(K_s\)：semantic transition，可选。

归一化后：

\[
0\le K(x)\le1
\]

定义采样半径：

\[
\boxed{
r_s(x)
=
r_{\max}
-
K(x)(r_{\max}-r_{\min})
}
\]

因此：

- 简单区域：\(K\rightarrow0\Rightarrow r_s\rightarrow r_{\max}\)
- 复杂区域：\(K\rightarrow1\Rightarrow r_s\rightarrow r_{\min}\)

---

# 7. 节点密度的三个实现层级

## 7.1 第一版：三级密度

最推荐最先实现。

```text
simple terrain   -> r_far
medium terrain   -> r_mid
complex terrain  -> r_near
```

满足：

\[
r_{\text{far}}>r_{\text{mid}}>r_{\text{near}}
\]

优点：
- 参数可解释；
- 易调试；
- 容易做消融。

---

## 7.2 第二版：连续复杂度函数

实现：

\[
r_s=f(K)
\]

优点：
- 更平滑；
- 更适合论文表达。

---

## 7.3 第三版：目标节点预算控制

给定最大节点预算：

\[
N\le N_{\max}
\]

动态调整：

\[
r_{\max},r_{\min},\theta_T
\]

使 graph size 满足计算预算。

适合长期大尺度导航。

---

# 8. Structural Node 优先级高于密度约束

非常重要：

\[
\boxed{
Topology/Cost Preservation > Sparsity
}
\]

普通 coverage node 必须服从：

\[
d(v,V)>r_s
\]

但 structural node 不应该简单因为距离太近就删除。

例如：

- 一个 junction；
- 一个 bottleneck；
- 一个强 traversability transition；

即使间距小于 \(r_s\)，只要它们表达不同导航意义，就可以保留。

---

# 9. Node Validation：机器人不是一个点

参考 TRG 的思想，node validity 应该在 robot footprint 上判断。

定义：

\[
\mathcal{F}_i
=
B(p_i,r_{\text{robot}})
\]

或者采用真实 footprint polygon。

检查：

- occupancy；
- height variation；
- slope；
- roughness；
- local traversability；
- uncertainty。

例如：

\[
valid(v_i)
=
\mathbf{1}
[
O(\mathcal{F}_i)<\theta_O
\land
R(\mathcal{F}_i)<\theta_R
]
\]

### 可选策略

#### A. Hard validation

有任一点超过风险阈值就 invalid。

适合安全优先。

#### B. Robust percentile

例如取 90% percentile：

\[
R_{90}(\mathcal{F}_i)<\theta
\]

对少量地图噪声更鲁棒。

#### C. Learned foothold/body feasibility

后期可接一个 locomotion-aware predictor。

第一版建议 A/B。

---

# 10. Node Suppression / Merge

节点候选生成后不应直接全部加入图。

### 基本空间抑制

若：

\[
\|p_i-p_j\|<r_{\text{merge}}
\]

且两个节点信息相似，则只留一个。

但不要只看距离。

### 推荐信息条件

可以计算：

\[
D_{info}(i,j)
=
w_pD_p
+
w_tD_t
+
w_sD_s
+
w_kD_k
\]

其中：

- \(D_p\)：空间距离；
- \(D_t\)：traversability descriptor 差异；
- \(D_s\)：semantic 差异；
- \(D_k\)：node type 差异。

如果：

\[
D_{info}<\theta_{merge}
\]

则 merge。

第一版可使用规则：

```text
if both are COVERAGE and distance < r_merge:
    merge
elif same structural type and descriptors similar:
    merge
else:
    keep both
```

---

# 11. Candidate Edge Generation

不建议像纯 visibility graph 一样，在开阔区域连接所有可见节点。

否则：

\[
|E|\rightarrow O(|V|^2)
\]

推荐候选边策略：

## 11.1 KNN

\[
N_k(v_i)
\]

对每个节点只检查最近的 \(k\) 个节点。

优点：
- 简单；
- 图稀疏。

缺点：
- 在非均匀节点密度下可能漏长距离重要连接。

---

## 11.2 Radius Search

\[
\|p_i-p_j\|<r_e(x)
\]

其中 \(r_e\) 也可以自适应。

---

## 11.3 Hybrid KNN + Radius

推荐：

```text
candidate if:
    distance < r_edge_max
and
    j is among k nearest neighbors
```

再增加少量 long-range shortcut。

---

## 11.4 Structural Shortcut

对于：

- junction；
- bottleneck；
- major frontier；

允许更远连边。

用于减少全局 hop 数。

---

# 12. Edge 不应该只检查“中心线”

这是核心原则之一。

不要只评价：

```text
●----------------●
```

而要评价机器人尺度 corridor：

```text
 /----------------\
●                  ●
 \----------------/
```

定义 edge corridor：

\[
\mathcal{C}_{ij}
\]

宽度至少与：

\[
2r_{\text{robot}}
\]

相关。

---

# 13. Corridor 的三种实现方案

## 13.1 Capsule Corridor

线段 + 两端半圆。

最适合实际 collision / terrain query。

优点：
- 几何直观；
- 与机器人宽度一致。

---

## 13.2 Ellipse Corridor

类似 TRG：

- 两个 node 作为 focal point；
- minor axis 与 robot size 相关。

优点：
- 对整段区域拟合友好。

---

## 13.3 Multi-slice Corridor

沿 edge 分成很多小 slice：

```text
|--1--|--2--|--3--|--4--|--5--|
```

每段独立计算局部 traversability。

这是本方案最推荐的方式，因为：

\[
\boxed{
Graph Edge Can Be Long,
Terrain Evaluation Remains Local
}
\]

也就是说，graph 可以稀，但地形分析不能粗。

---

# 14. Edge Validity

一条边成立需要满足：

\[
valid(e_{ij})=
collisionFree(\mathcal C_{ij})
\land
riskAcceptable(\mathcal C_{ij})
\land
traversable(\mathcal C_{ij})
\]

第一版建议硬约束：

\[
\max_{x\in\mathcal C_{ij}}R(x)<R_{\text{critical}}
\]

以及：

\[
\min_{x\in\mathcal C_{ij}}\tau(x)>\tau_{\text{critical}}
\]

但注意：
- 如果地图噪声较大，可以用 percentile 替代 max/min；
- 例如 95% percentile。

---

# 15. Edge Cost：长度 × 地形代价

核心形式：

\[
\boxed{
C(e_{ij})
=
\int_{e_{ij}}
c_t(x)\,ds
}
\]

如果 \(c_t(x)=1\)，那么：

\[
C(e)=L_e
\]

如果地形更难：

\[
c_t(x)>1
\]

则边代价自动增大。

离散实现：

\[
C(e_{ij})
\approx
\sum_{k=1}^{N}
c_t(x_k)\Delta s
\]

---

# 16. Direction-Aware Traversability

参考 TRG 的重要启发：

同一块坡地，从不同方向进入，风险可能不同。

因此更一般地：

\[
\boxed{
c_t(x,\theta)
}
\]

而不是：

\[
c_t(x)
\]

于是：

\[
C(e_{ij})
=
\int_{e_{ij}}
c_t(x,\theta_{ij})ds
\]

这会自然引出：

\[
C_{ij}\neq C_{ji}
\]

因此虽然第一版可以先用无向图，后续非常值得研究：

> **Directed Traversability Graph**

---

# 17. Direction-Aware Cost 的几种实现

## 17.1 解析几何模型

根据：

- slope magnitude；
- travel direction；
- slope gradient direction；
- lateral slope；
- longitudinal slope；

定义：

\[
c_t(x,\theta)=
1+
w_l c_{\text{lon}}
+
w_{lat}c_{\text{lat}}
\]

类似 TRG 的 longitudinal / lateral risk。

---

## 17.2 Semantic + Geometry

\[
c_t=
w_s c_{semantic}
+
w_g c_{geometry}
+
w_d c_{direction}
\]

例如：

- road：低 cost；
- grass：中等；
- mud：高；
- cross-slope：额外惩罚。

---

## 17.3 Learned Traversability Cost

输入：

- local terrain patch；
- desired travel direction；
- robot state；

输出：

\[
\hat c_t
\]

适合以后接 RL / locomotion experience。

第一版建议先用 17.1 + 17.2。

---

# 18. Edge Risk：不能让平均值掩盖局部危险

只计算平均 terrain cost 可能有问题。

例如：

```text
road road road MUD road road
```

mud 很短，但可能非常危险。

因此建议：

\[
C_e=
\underbrace{\int_e c_t(x,\theta)ds}_{累计代价}
+
\lambda_r
\underbrace{R_e}_{局部风险项}
+
\lambda_u
\underbrace{U_e}_{不确定性项}
\]

其中：

\[
R_e=\max_{x\in\mathcal C_e}R(x)
\]

或：

\[
R_e=P_{95}(R(x))
\]

\(U_e\) 可以是 corridor 内 uncertainty 均值或高分位数。

第一版推荐：

\[
\boxed{
C_e=
L_e
+
\lambda_t\sum c_t\Delta s
+
\lambda_rR_{95}
+
\lambda_u\bar U
}
\]

或者更简洁：

\[
\boxed{
C_e=
L_e(1+\lambda_t\bar c_t)
+
\lambda_rR_{95}
+
\lambda_u\bar U
}
\]

---

# 19. 与 TRG Edge Cost 的关系

TRG 的搜索累计代价可写成：

\[
\Delta C_e=d_e(1+\Gamma w_e)
\]

本质也是：

\[
\text{Length} \times \text{Risk-Adjusted Unit Cost}
\]

本方案进一步扩展为：

- risk 不只由局部几何平面决定；
- 可以融合 semantic；
- 可以积分整条长 edge；
- 可以方向相关；
- 可以考虑 uncertainty；
- 可以考虑 robot experience。

---

# 20. Sparse Graph + Dense Edge Evaluation

这是减少 graph node 的关键。

假设两个节点相距 4m：

```text
●--------------------------------●
```

这不代表中间只检查一次。

底层仍然以地图分辨率或固定采样步长：

\[
\Delta s=0.05m,\ 0.1m,\ ...
\]

查询：

```text
●-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-●
```

因此：

> 可以用少量 node 表示大尺度拓扑，同时保留高分辨率地形风险。

这也是本方案区别于“简单把 TRG 的扩展半径调大”的关键。

---

# 21. Edge Length 也可以自适应

定义：

\[
L_{\max}(x)=f(K(x))
\]

简单区域：

\[
L_{\max}\uparrow
\]

复杂区域：

\[
L_{\max}\downarrow
\]

例如概念上：

```text
simple:
●--------------------------------●

complex:
●------●------●------●
```

第一版可先固定最大 edge length；
第二版再做 complexity-adaptive edge length。

---

# 22. 图稀疏化：构图后继续删冗余点

即使打点已经稀疏，仍建议增加 graph simplification。

考虑：

```text
A ------ B ------ C
```

如果 B 不是 structural node，且：

\[
valid(A,C)=true
\]

并且：

\[
C_{AC}
\le
(1+\epsilon_c)
(C_{AB}+C_{BC})
\]

那么删除 B：

```text
A ---------------- C
```

其中：

\[
\epsilon_c
\]

是允许的代价恶化比例。

这是一个非常重要的“图密度旋钮”。

---

# 23. Graph Sparsification 的三个方案

## 23.1 Degree-2 Shortcut

只处理：

\[
deg(B)=2
\]

最安全、最简单。

第一版推荐。

---

## 23.2 Local Shortcut Search

对每个普通 coverage node 尝试：

- 删除该 node；
- 检查邻居间能否 shortcut；
- 检查总代价变化。

---

## 23.3 Spanner-like Sparsification

目标保持：

\[
d_G(i,j)
\le
t\cdot d_{G_{dense}}(i,j)
\]

其中 \(t\) 是 stretch factor。

适合论文后期进一步理论化。

---

# 24. Local Fine Graph + Global Sparse Graph

推荐长期导航最终采用双尺度图：

\[
\boxed{
Local\ Fine\ Graph
+
Global\ Sparse\ Graph
}
\]

机器人附近：

- 较密；
- 高频更新；
- 高精度 corridor evaluation。

远处已经稳定探索区域：

- 只保留 structural nodes；
- coverage nodes 大量压缩；
- edge 可更长。

这样可以显著降低全局搜索规模。

---

# 25. 增量构图逻辑

每次新观测产生一个更新区域：

\[
\Omega_{\text{update}}
\]

只处理：

\[
\Omega_{\text{update}}\oplus margin
\]

流程：

```text
new measurement
      ↓
update terrain map
      ↓
compute changed region Ωupdate
      ↓
update local descriptors
      ↓
re-detect structural candidates
      ↓
remove invalid/stale local nodes
      ↓
add new nodes
      ↓
update local candidate edges
      ↓
evaluate edge corridor
      ↓
local graph sparsification
      ↓
merge into global graph
```

---

# 26. 节点增删规则

## 添加

满足任一：

- 新 bottleneck；
- 新 junction；
- 新 significant transition；
- 新 frontier；
- coverage gap 超过阈值。

## 删除

满足任一：

1. 节点 footprint 已不可通行；
2. 原 structural condition 消失；
3. frontier 已被探索，不再是 frontier；
4. coverage node 被 shortcut 替代；
5. 多个 node 信息冗余；
6. 长期 stale 且不影响拓扑。

---

# 27. 边增删规则

## 添加

两个节点：

- 空间相邻；
- corridor valid；
- graph local connectivity 有价值；
- 不产生明显冗余。

## 删除

满足：

- obstacle 阻断；
- risk 超阈值；
- traversability 下降；
- uncertainty 高到不可接受；
- 两节点之一 invalid；
- edge 被更优 shortcut 取代。

---

# 28. Start / Goal 如何接入图

每次规划时不需要把 start/goal 永久加入全局图。

### Start

创建临时节点：

\[
v_s
\]

连接到附近：

- KNN；
- 可通行 structural node；
- nearby coverage node。

### Goal

同理：

\[
v_g
\]

如果 goal 尚在 unknown 区域，则：

1. 选择 goal 方向上最合适 frontier；
2. 规划至 frontier；
3. 探索更新；
4. 重复 replanning。

这与 TRG 的 frontier-subgoal 思想类似。

---

# 29. Graph Search

基础推荐 A*。

\[
f(n)=g(n)+h(n)
\]

其中：

\[
g(n)=
\sum_{e\in path}C_e
\]

如果最低单位距离代价：

\[
c_{\min}\ge1
\]

则可以使用：

\[
h(n)=c_{\min}\|p_n-p_g\|
\]

保证 heuristic 保守。

第一版：

\[
h(n)=\|p_n-p_g\|
\]

最简单。

---

# 30. 搜索算法可探索方案

## 30.1 A*

第一版首选。

优点：
- 简单；
- baseline 清晰。

## 30.2 D* Lite / LPA*

如果图频繁局部更新，非常值得。

优点：
- 不必每次从头搜索；
- 更适合在线增量地图。

## 30.3 ARA*

如果希望 anytime planning：

- 先快速给次优解；
- 有时间再优化。

## 30.4 Hierarchical Search

在 global sparse graph 上规划大路线，
再在 local fine graph 上细化。

非常推荐后期探索。

---

# 31. 推荐的数据结构

## Node

```cpp
struct GraphNode {
    uint64_t id;

    Eigen::Vector3d position;

    NodeType type;
    NodeState state;

    float traversability;
    float risk;
    float uncertainty;

    float local_complexity;

    double last_update_time;

    std::vector<uint64_t> neighbor_ids;
};
```

## Edge

```cpp
struct GraphEdge {
    uint64_t from;
    uint64_t to;

    float length;

    float terrain_cost;
    float risk_cost;
    float uncertainty_cost;
    float total_cost;

    float max_risk;
    float min_traversability;

    bool valid;

    // 如果未来做有向图，可分别保存 forward/backward cost
};
```

---

# 32. 推荐模块接口

## TerrainMap

```cpp
class TerrainMap {
public:
    TerrainCell query(const Eigen::Vector2d& p) const;

    TerrainPatch queryPatch(
        const Eigen::Vector2d& p,
        double radius) const;

    CorridorData queryCorridor(
        const Eigen::Vector2d& a,
        const Eigen::Vector2d& b,
        double width,
        double resolution) const;
};
```

## NodeSampler

```cpp
class NodeSampler {
public:
    std::vector<NodeCandidate> sample(
        const TerrainMap& map,
        const Graph& graph,
        const LocalRegion& region);
};
```

内部可分别实现：

- BottleneckDetector
- JunctionDetector
- TransitionDetector
- FrontierDetector
- CoverageSampler

## EdgeEvaluator

```cpp
class EdgeEvaluator {
public:
    EdgeEvaluation evaluate(
        const GraphNode& a,
        const GraphNode& b,
        const TerrainMap& map);
};
```

## GraphManager

负责：

- insert；
- merge；
- invalidate；
- sparsify；
- local/global graph；
- spatial index。

推荐使用：

- KD-tree；
- voxel hash；
- R-tree；
- nanoflann。

---

# 33. 第一版完整伪代码

```text
INPUT:
    updated terrain map M
    current graph G
    robot pose x_robot
    goal x_goal

1. Determine changed local region Ω

2. Update terrain descriptors in Ω

3. Detect structural node candidates:
       B = bottleneck_detector(Ω)
       J = junction_detector(Ω)
       T = transition_detector(Ω)
       F = frontier_detector(Ω)

4. Generate coverage candidates:
       for uncovered region q in Ω:
           K = navigation_complexity(q)
           r = adaptive_spacing(K)
           if distance(q, existing_graph) > r:
               add coverage candidate

5. Validate candidates with robot footprint

6. Suppress / merge redundant candidates

7. Insert valid new nodes into G

8. Remove or invalidate stale local nodes

9. Candidate edge generation:
       for each changed/new node vi:
           neighbors = hybrid_knn_radius(vi)

10. For each candidate edge (vi, vj):
        corridor = build_robot_scale_corridor(vi, vj)
        if corridor invalid:
            reject edge
        else:
            compute direction-aware terrain cost
            compute risk
            compute uncertainty
            assign total edge cost

11. Remove invalid outdated local edges

12. Local graph sparsification:
        degree-2 shortcut
        redundant coverage node removal

13. Attach current robot start node

14. Attach goal node or select best frontier sub-goal

15. Run A* / D* Lite

16. Output sparse graph path

17. Send path to local path follower / local motion policy
```

---

# 34. 第一版建议不要一次做得太复杂

推荐实现顺序：

## Phase 0：最小 baseline

- traversability grid；
- coverage sampling；
- fixed minimum spacing；
- KNN；
- collision + terrain integral；
- A*。

目的：先把完整 pipeline 跑通。

## Phase 1：Adaptive Density

加入：

- simple / medium / complex 三级 spacing；
- complexity score；
- degree-2 simplification。

这一阶段即可测试：

> 是否真的比 TRG/PRM 更稀。

## Phase 2：Structural Nodes

依次加入：

1. frontier；
2. bottleneck；
3. junction；
4. traversability transition。

逐个做消融。

## Phase 3：Robot-Scale Corridor

从中心线检查升级：

\[
line \rightarrow capsule/corridor
\]

## Phase 4：Direction-Aware Edge Cost

加入：

- longitudinal slope；
- lateral slope；
- directional risk。

## Phase 5：Local/Global Hierarchy

加入：

\[
LocalFineGraph + GlobalSparseGraph
\]

## Phase 6：Experience-Aware

机器人走过后，根据：

- slip；
- tracking error；
- body motion；
- energy；
- failure；

更新：

\[
c_t^{map}
\rightarrow
c_t^{experience}
\]

---

# 35. 参数表建议

| 参数 | 含义 | 影响 |
|---|---|---|
| \(r_{robot}\) | 机器人 footprint 半径 | node/edge 安全尺度 |
| \(r_{min}\) | 复杂区域最小 spacing | 控制最大节点密度 |
| \(r_{max}\) | 简单区域最大 spacing | 控制全局稀疏程度 |
| \(r_{merge}\) | node merge 距离 | 冗余节点数量 |
| \(k\) | KNN 邻居数量 | edge density |
| \(r_{edge,max}\) | 最大候选边长度 | 图连通与复杂度 |
| \(\theta_T\) | transition 阈值 | transition node 数量 |
| \(\theta_R\) | risk hard threshold | 安全性 |
| \(\tau_{critical}\) | 最低可通行阈值 | edge validity |
| \(\lambda_t\) | terrain 权重 | 距离/地形平衡 |
| \(\lambda_r\) | risk 权重 | 安全偏好 |
| \(\lambda_u\) | uncertainty 权重 | 未知区域偏好 |
| \(\epsilon_c\) | graph shortcut 容忍误差 | 稀疏程度 |
| \(\Delta s\) | edge corridor 采样步长 | terrain evaluation 精度 |

---

# 36. 建议设置三档 Graph Density Mode

仅作为工程方便，不代表最终论文参数。

## Fine

目标：
- 复杂环境；
- 高安全要求。

特点：
- 小 \(r_{max}\)；
- 小 \(\epsilon_c\)；
- 更多 structural candidates。

## Balanced

默认模式。

## Sparse

目标：
- 长距离；
- 大地图；
- 已探索区域。

特点：
- 大 \(r_{max}\)；
- 大 \(\epsilon_c\)；
- 更积极 shortcut。

---

# 37. 论文/实验最重要的评价指标

不能只比较 path length。

建议至少：

## Graph Size

\[
N_V=|V|
\]

\[
N_E=|E|
\]

## Planning Time

- graph construction time；
- graph update time；
- A* search time；
- total replanning time。

## Path Quality

- geometric path length；
- terrain-weighted path cost；
- max risk；
- average risk。

## Safety

- execution success；
- slip；
- body-ground collision；
- tracking error；
- local replanning frequency。

## Sparsity-Quality Tradeoff

非常重要：

\[
\frac{\text{Path Cost}_{ours}}
{\text{Path Cost}_{dense}}
\]

对比：

\[
\frac{|V|_{ours}}
{|V|_{dense}}
\]

理想结果：

> 大幅减少 node，而 path quality 只轻微下降，甚至安全性更好。

---

# 38. 最重要的消融实验

建议一定做：

## Ablation A：固定 spacing vs adaptive spacing

证明：

\[
adaptive density
\]

确实有价值。

## Ablation B：只有 coverage vs + structural nodes

验证：

- bottleneck；
- junction；
- transition；

是否减少漏路。

## Ablation C：中心线 edge evaluation vs corridor evaluation

验证 robot-scale corridor 的必要性。

## Ablation D：distance-only vs terrain-weighted

验证 terrain cost。

## Ablation E：direction-independent vs direction-aware

验证坡地不同方向代价。

## Ablation F：无 sparsification vs sparsification

验证 graph size / search time。

---

# 39. 推荐 baseline

最有价值的 baseline：

- Grid A*；
- PRM / PRM*；
- Voronoi roadmap；
- FAR Planner；
- TRG-planner；
- 本方案不同消融版本。

---

# 40. 与 TRG 的明确区别

TRG 的主要逻辑：

```text
reference node
    ↓
fixed-radius circular random sampling
    ↓
terrain validation
    ↓
merge/add
    ↓
edge risk evaluation
    ↓
wavefront expansion
```

本方案：

```text
local map / frontier
       ↓
navigation structure analysis
       ↓
structural node proposals
       ↓
complexity-adaptive coverage
       ↓
robot-scale validation
       ↓
sparse candidate graph
       ↓
dense corridor terrain evaluation
       ↓
graph sparsification
       ↓
incremental weighted graph search
```

核心区别：

\[
\boxed{
TRG:\ Sampling \rightarrow Terrain\ Evaluation
}
\]

本方案：

\[
\boxed{
Terrain/Topology\ Structure
\rightarrow
Sampling
\rightarrow
Terrain\ Evaluation
}
\]

---

# 41. 这套方案最可能形成的几个创新点

## 创新点候选 1
**Navigation-complexity-adaptive graph density**

节点密度由局部导航复杂度决定，而不是固定扩展半径。

## 创新点候选 2
**Structural-node-preserving graph sparsification**

在保留：

- bottleneck；
- junction；
- significant cost transition；
- frontier；

的前提下主动压缩 coverage nodes。

## 创新点候选 3
**Sparse graph + dense corridor evaluation**

图结构可以很稀，但 edge 的地形评价仍然保持高分辨率。

## 创新点候选 4
**Direction-aware traversability-weighted edges**

\[
C_{ij}\neq C_{ji}
\]

可自然发展为有向风险图。

## 创新点候选 5
**Local fine / global sparse incremental graph**

当前机器人附近精细，远处稳定区域持续压缩。

---

# 42. 当前最推荐的“主方案”

如果现在就要开始写代码，我建议主线不要同时探索所有高级版本，而采用：

### Node
- bottleneck；
- frontier；
- traversability transition；
- coverage。

junction 可以在 bottleneck/skeleton branch 基础上加入。

### Density
- 三级 adaptive spacing。

### Node validation
- robot footprint。

### Edge candidate
- KNN + max radius。

### Edge validity
- capsule corridor。

### Edge cost

\[
C_e=
L_e
+
\lambda_t \sum c_t(x_k,\theta_e)\Delta s
+
\lambda_rR_{95}
\]

第一版方向项可以先关闭：

\[
c_t(x,\theta)\rightarrow c_t(x)
\]

后面再加入。

### Simplification
- degree-2 shortcut。

### Search
- A*。

### Incremental
- local changed-region update；
- frontier expansion。

这一版已经足够形成完整系统。

---

# 43. 第一版最值得探索的参数

优先调：

1. \(r_{min}\)
2. \(r_{max}\)
3. \(\theta_T\)
4. \(k\)
5. \(r_{edge,max}\)
6. \(\epsilon_c\)
7. \(\lambda_t\)
8. \(\lambda_r\)

不要一开始加入太多参数。

---

# 44. 一个完整代码目录建议

```text
complexity_adaptive_graph/
├── include/
│   ├── terrain_map/
│   │   ├── terrain_map.hpp
│   │   └── terrain_descriptor.hpp
│   │
│   ├── sampling/
│   │   ├── node_sampler.hpp
│   │   ├── bottleneck_detector.hpp
│   │   ├── junction_detector.hpp
│   │   ├── transition_detector.hpp
│   │   ├── frontier_detector.hpp
│   │   └── coverage_sampler.hpp
│   │
│   ├── graph/
│   │   ├── graph_node.hpp
│   │   ├── graph_edge.hpp
│   │   ├── graph_manager.hpp
│   │   └── graph_sparsifier.hpp
│   │
│   ├── edge/
│   │   ├── corridor_builder.hpp
│   │   ├── edge_validator.hpp
│   │   └── edge_cost_evaluator.hpp
│   │
│   └── planning/
│       ├── astar.hpp
│       └── goal_connector.hpp
│
├── src/
├── config/
│   ├── graph.yaml
│   ├── sampling.yaml
│   └── terrain_cost.yaml
│
├── test/
│   ├── synthetic_maps/
│   ├── test_sampling.cpp
│   ├── test_edge_cost.cpp
│   ├── test_sparsification.cpp
│   └── test_planning.cpp
│
└── launch/
```

---

# 45. 建议的可视化调试信息

为了后面调算法，强烈建议 RViz 分图层显示：

- structural nodes；
- coverage nodes；
- frontier nodes；
- invalid nodes；
- graph edges；
- edge cost color；
- bottleneck detector；
- traversability gradient；
- local complexity；
- adaptive sampling radius；
- removed shortcut nodes；
- planned global path。

尤其建议每个 node 用颜色区分 type。

---

# 46. 推荐先做的合成地图测试

## Test 1：大开阔平地

目标：
节点应该非常稀。

## Test 2：狭窄走廊

目标：
不能因为稀疏采样漏掉通道。

## Test 3：三岔路

目标：
junction 必须保留。

## Test 4：草地 + 泥地

目标：
虽然都可通行，但规划应倾向低代价区域。

## Test 5：短泥地捷径 vs 长草地绕路

验证：

\[
Length\ vs\ Traversability
\]

平衡。

## Test 6：坡地不同方向

验证 direction-aware risk。

## Test 7：未知区域增量扩展

验证 frontier update。

## Test 8：地图持续扩大

验证：

\[
|V|
\]

是否增长得比 TRG / PRM 慢。

---

# 47. 需要特别警惕的失败模式

## 47.1 过度稀疏

现象：
- 漏窄通道；
- 无法绕障碍；
- 路径质量大幅下降。

解决：
- structural node；
- local refinement；
- smaller \(r_{min}\)。

## 47.2 transition node 爆炸

现象：
噪声导致 traversability gradient 到处变化。

解决：
- smoothing；
- hysteresis；
- minimum region size；
- \(\theta_T\)；
- confidence gating。

## 47.3 Edge 太长跨越复杂地形

解决：
- dense corridor sampling；
- segmentation；
- adaptive \(L_{max}\)；
- risk hard threshold。

## 47.4 图不连通

解决：
- fallback coverage samples；
- temporary PRM-like samples；
- local expansion radius increase。

## 47.5 节点类型冲突

例如一个点同时是：
- bottleneck；
- transition；
- frontier。

建议：

```cpp
std::bitset<NodeRole>
```

而不是只能有一个 type。

后期可改成多标签。

---

# 48. 最终一句话定义

本方案不是“随机地在可行区域放点”。

而是：

> **在真正会改变机器人导航决策的位置优先保留节点，在简单区域只用少量 coverage node 保证连通；再通过机器人尺度的高分辨率 corridor evaluation 计算长度、地形、方向风险和不确定性组成的 edge cost，并通过增量维护和持续稀疏化得到适合长期全局搜索的稀疏加权图。**

数学上可以简化为：

\[
\boxed{
Node
\iff
Navigation\ Decision\ Structure
}
\]

以及：

\[
\boxed{
C(e)
=
\int_e c_t(x,\theta_e)ds
+
\lambda_rR_e
+
\lambda_uU_e
}
\]

最终目标：

\[
\boxed{
\min_{\pi}
\sum_{e\in\pi}C(e)
}
\]

同时满足：

\[
\text{Safety Constraints}
\]

和：

\[
\text{Graph Sparsity Constraints}
\]

---

# 49. 当前建议的研究主线

如果后续准备写论文，可以把方法主线收敛成：

1. **复杂度自适应节点采样**；
2. **导航结构关键点保留**；
3. **机器人尺度 corridor 边评价**；
4. **方向感知 traversability cost**；
5. **增量 graph sparsification**；
6. **risk-aware graph search**。

其中最应该优先验证的是：

\[
\boxed{
Adaptive Sampling + Sparse Graph
}
\]

因为这决定了你的方法是否真的能比 TRG / PRM 在大尺度环境下更适合长期全局规划。

---

# 50. 参考与直接启发来源

本设计中以下思路受到 TRG-planner 的直接启发：

- wavefront 式增量 graph expansion；
- node 使用 robot-scale 区域进行有效性判断；
- 近距离 sampled node merge；
- robot-width / ellipse-like edge region；
- direction-aware traversal risk；
- edge cost 同时考虑距离和 risk；
- frontier node 驱动未知环境继续扩图。

本方案在此基础上重点改进：

- 从“固定尺度随机覆盖”转向“导航复杂度自适应采样”；
- 从“节点数量随可达面积快速增长”转向“simple region aggressive sparsification”；
- 从“edge 与单一局部平面近似较强耦合”转向“长边 + 局部密集 corridor evaluation”；
- 增加 structural-node-preserving graph sparsification；
- 引入 semantic / traversability / uncertainty / experience 等更丰富的 edge cost。

主要参考论文：

**TRG-planner: Traversal Risk Graph-Based Path Planning in Unstructured Environments for Safe and Efficient Navigation**  
Dongkyu Lee, I Made Aswin Nahrendra, Minho Oh, Byeongho Yu, Hyun Myung.

---

# 51. 最后给未来写代码时的优先级

如果未来重新打开这份文件，只需要先记住下面 6 条：

1. **Structural node 不能为了稀疏随便删。**
2. **Coverage node 才是主要压缩对象。**
3. **Graph 稀疏不代表 terrain evaluation 稀疏。**
4. **Edge cost 应天然同时包含“走多远”和“多难走”。**
5. **复杂区域密、简单区域稀。**
6. **构图之后还要持续 sparsify，而不是只增不减。**

这 6 条就是当前方案最重要的设计原则。
